// ============================================================
// INCUBADORA FIAP — Egg Incubator PID Controller
// ============================================================
// Hardware:
//   - Enclosure: Wooden box 30x30x30 cm
//   - Sensor: DHT11 digital temperature & humidity sensor
//   - Actuator: Incandescent lamp (< 200W) on mains (127V/220V)
//   - Switching: Relay module (mechanical, slow PWM to preserve life)
//   - Controller: ESP32 (programmed via PlatformIO)
//
// Control:
//   - PID with 20s slow-PWM relay cycle
//   - Anti-windup ±50 (handles box-opened disturbances)
//   - Safety: over-temp cutoff, sensor fault detection, watchdog
//
// Output:
//   - JSON telemetry at 1 Hz over Serial (115200 baud) for debugging
//   - HTTP API on port 80 over WiFi for dashboard consumption
//     GET  /data             → latest telemetry JSON (includes humidity)
//     GET  /status           → system health + WiFi info
//     POST /control/power    → {"enabled": true/false} — turn system on/off
//     POST /control/setpoint → {"setpoint": 38.5}     — change target temp (30–42°C)
//
// Notes (DHT11):
//   - Digital single-wire protocol (no ADC needed)
//   - Temperature range: 0–50°C, resolution: 1°C, accuracy: ±2°C
//   - Humidity range: 20–90% RH, resolution: 1%, accuracy: ±5%
//   - Minimum sampling interval: ~1s (we read at 1 Hz — perfect)
//   - EMA filter smooths the 1°C steps for more stable PID control
//   - Powered from 3.3V or 5V; data pin needs 10kΩ pull-up to VCC
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h> // ESP32 task watchdog timer
#include <DHT.h>          // Adafruit DHT sensor library

// ============ WIFI CONFIGURATION ============
// TODO: Replace with your network credentials
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

WebServer server(80); // HTTP server on port 80

// ============ PIN DEFINITIONS ============

#define PIN_DHT    4       // GPIO4 — data pin for DHT11 sensor (with 10kΩ pull-up to VCC)
#define DHTTYPE    DHT11   // DHT11 sensor type (change to DHT22 for higher precision)
#define PIN_RELAY  23      // GPIO23 — digital output for the relay module

DHT dht(PIN_DHT, DHTTYPE);

// Set to true if your relay module is active-LOW (most common relay boards)
#define RELAY_ACTIVE_LOW false

// ============ SYSTEM CONSTANTS ============

const unsigned long INTERVAL_READINGS = 1000;   // Sensor reading interval (ms) — 1 Hz
const unsigned long PERIOD_PWM        = 20000;  // Relay slow-PWM period (ms) — 20s to preserve relay life

// Setpoint (mutable — controlled via dashboard)
float setpoint = 38.0;  // Target incubation temperature (°C)

// Setpoint safety bounds (dashboard can only set within this range)
const float SETPOINT_MIN = 30.0;
const float SETPOINT_MAX = 42.0;

// Safety thresholds
const float TEMP_MAX_CUTOFF = 42.0;  // Hard over-temperature cutoff — lamp OFF immediately (°C)
const float TEMP_MIN_VALID  = 0.0;   // Minimum plausible DHT11 reading (°C)
const float TEMP_MAX_VALID  = 50.0;  // Maximum DHT11 reading (°C) — sensor spec limit

// ============ PID PARAMETERS ============

float Kp = 15.0;
float Ki = 0.5;
float Kd = 5.0;

// PID state
float errorAccumulated = 0.0;  // Accumulated error for integral term
float previousError    = 0.0;  // Previous error for derivative term

// ============ STATE VARIABLES ============

unsigned long previousMillisReadings = 0;  // Last sensor reading timestamp
unsigned long previousMillisPWM      = 0;  // Last PWM period start timestamp
unsigned long timeOn                 = 0;  // Calculated relay ON time for current period
unsigned long activeTimeOn           = 0;  // Latched relay ON time (updated only at PWM period boundary)

// EMA temperature & humidity filters
float filteredTemp     = 0.0;
float filteredHumidity = 0.0;
float latestHumidity   = 0.0;  // Latest humidity for telemetry
bool  firstReading     = true;

// Safety state
bool sensorFault    = false;
bool overTempFault  = false;

// System control (dashboard can toggle)
bool systemEnabled = false; // Starts OFF — dashboard must explicitly enable

// Latest telemetry snapshot (updated every reading cycle, served via HTTP)
String latestJson = "{}";

// ============ HELPER FUNCTIONS ============

/// Sets the relay state, respecting active-LOW/HIGH configuration
void setRelay(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  } else {
    digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  }
}

/// Connects to WiFi network. Blocks until connected (with timeout).
void setupWiFi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" FAILED (will retry in background)");
  }
}

/// Builds the telemetry JSON string from current values
String buildJson(float temp, float humidity, float e, float pidOutput,
                 float pTerm, float iTerm, float dTerm,
                 bool relayOn, unsigned long uptime) {
  String json = "{";
  json += "\"system_enabled\":" + String(systemEnabled ? "true" : "false");
  json += ",\"temperatura\":"  + String(temp, 2);
  json += ",\"umidade\":"      + String(humidity, 1);
  json += ",\"setpoint\":"    + String(setpoint, 1);
  json += ",\"potencia_pwm\":" + String(pidOutput, 2);
  json += ",\"erro\":"        + String(e, 2);
  json += ",\"p\":"           + String(pTerm, 2);
  json += ",\"i\":"           + String(iTerm, 2);
  json += ",\"d\":"           + String(dTerm, 2);
  json += ",\"relay\":"       + String(relayOn ? 1 : 0);
  json += ",\"uptime_ms\":"   + String(uptime);
  json += "}";
  return json;
}

// ============ HTTP ROUTE HANDLERS ============

/// GET /data — returns latest telemetry as JSON
void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", latestJson);
}

/// GET /status — system health, network info, and control state
void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String status = "{";
  status += "\"status\":\"OK\"";
  status += ",\"system_enabled\":" + String(systemEnabled ? "true" : "false");
  status += ",\"setpoint\":" + String(setpoint, 1);
  status += ",\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  status += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  status += ",\"rssi\":" + String(WiFi.RSSI());
  status += ",\"sensor_fault\":" + String(sensorFault ? "true" : "false");
  status += ",\"over_temp_fault\":" + String(overTempFault ? "true" : "false");
  status += ",\"uptime_ms\":" + String(millis());
  status += "}";
  server.send(200, "application/json", status);
}

/// Handles CORS preflight requests (supports GET + POST)
void handleCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

/// POST /control/power — turn the incubator system on or off
/// Body: {"enabled": true} or {"enabled": false}
void handlePower() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String body = server.arg("plain");

  if (body.indexOf("true") >= 0) {
    systemEnabled = true;
    // Reset PID state for a clean start
    errorAccumulated = 0.0;
    previousError = 0.0;
    Serial.println("[CTRL] System ENABLED");
    server.send(200, "application/json", "{\"system_enabled\":true}");
  } else if (body.indexOf("false") >= 0) {
    systemEnabled = false;
    setRelay(false); // Immediately turn off heating
    timeOn = 0;
    activeTimeOn = 0;
    errorAccumulated = 0.0;
    previousError = 0.0;
    Serial.println("[CTRL] System DISABLED");
    server.send(200, "application/json", "{\"system_enabled\":false}");
  } else {
    server.send(400, "application/json", "{\"error\":\"INVALID_BODY\",\"expected\":\"{\\\"enabled\\\": true/false}\"}");
  }
}

/// POST /control/setpoint — change the target temperature
/// Body: {"setpoint": 38.5}
void handleSetpoint() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String body = server.arg("plain");

  // Simple JSON parsing — find the number after "setpoint":
  int idx = body.indexOf("setpoint");
  if (idx < 0) {
    server.send(400, "application/json", "{\"error\":\"MISSING_FIELD\",\"expected\":\"{\\\"setpoint\\\": 38.5}\"}");
    return;
  }

  // Find the colon after "setpoint", then parse the number
  int colonIdx = body.indexOf(':', idx);
  if (colonIdx < 0) {
    server.send(400, "application/json", "{\"error\":\"INVALID_FORMAT\"}");
    return;
  }

  float newSetpoint = body.substring(colonIdx + 1).toFloat();

  // Safety clamp
  if (newSetpoint < SETPOINT_MIN || newSetpoint > SETPOINT_MAX) {
    String err = "{\"error\":\"OUT_OF_RANGE\"";
    err += ",\"min\":" + String(SETPOINT_MIN, 1);
    err += ",\"max\":" + String(SETPOINT_MAX, 1);
    err += ",\"received\":" + String(newSetpoint, 1);
    err += "}";
    server.send(400, "application/json", err);
    return;
  }

  setpoint = newSetpoint;
  // Reset integral to avoid windup from the old setpoint
  errorAccumulated = 0.0;
  previousError = 0.0;

  Serial.print("[CTRL] Setpoint changed to ");
  Serial.println(setpoint, 1);

  String response = "{\"setpoint\":" + String(setpoint, 1) + "}";
  server.send(200, "application/json", response);
}

/// 404 handler
void handleNotFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "application/json", "{\"error\":\"NOT_FOUND\"}");
}

/// Reads temperature and humidity from the DHT11 sensor, filtered with EMA.
/// DHT11 has 1°C resolution — the EMA filter smooths the steps for stable PID.
/// Also updates latestHumidity for telemetry.
/// Returns -999.0 as a sentinel value if the sensor reading fails.
float readTemperature() {
  float rawTemp     = dht.readTemperature(); // Celsius
  float rawHumidity = dht.readHumidity();

  // DHT library returns NaN on read failure (checksum error, timeout, etc.)
  if (isnan(rawTemp) || isnan(rawHumidity)) {
    return -999.0; // Sentinel: sensor fault
  }

  // Sensor range validation
  if (rawTemp < TEMP_MIN_VALID || rawTemp > TEMP_MAX_VALID) {
    return -999.0; // Sentinel: out of valid range
  }

  // Apply EMA filter for noise smoothing
  // DHT11's 1°C resolution makes filtering especially important
  // to avoid PID jitter at step boundaries
  if (firstReading) {
    filteredTemp     = rawTemp;
    filteredHumidity = rawHumidity;
    firstReading     = false;
  } else {
    filteredTemp     = 0.70 * filteredTemp     + 0.30 * rawTemp;     // alpha = 0.30 (more responsive since DHT11 is less noisy than ADC)
    filteredHumidity = 0.70 * filteredHumidity + 0.30 * rawHumidity;
  }

  latestHumidity = filteredHumidity;
  return filteredTemp;
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);

  // Initialize DHT11 sensor
  dht.begin();
  delay(1000); // DHT11 needs ~1s to stabilize after power-on

  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false); // Ensure the lamp is OFF at startup

  // Connect to WiFi
  setupWiFi();

  // Register HTTP routes — data & status
  server.on("/data",   HTTP_GET,     handleData);
  server.on("/data",   HTTP_OPTIONS, handleCORS);
  server.on("/status", HTTP_GET,     handleStatus);
  server.on("/status", HTTP_OPTIONS, handleCORS);

  // Register HTTP routes — controls
  server.on("/control/power",    HTTP_POST,    handlePower);
  server.on("/control/power",    HTTP_OPTIONS, handleCORS);
  server.on("/control/setpoint", HTTP_POST,    handleSetpoint);
  server.on("/control/setpoint", HTTP_OPTIONS, handleCORS);

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("[HTTP] Server started on port 80");

  previousMillisPWM = millis();
  previousMillisReadings = millis();

  // Enable ESP32 task watchdog timer — resets the MCU if loop() hangs for > 3 seconds
  esp_task_wdt_init(3, true);
  esp_task_wdt_add(NULL);

  Serial.println("{\"status\":\"BOOT\",\"setpoint\":" + String(setpoint, 1) + "}");
}

// ============ MAIN LOOP ============

void loop() {
  esp_task_wdt_reset(); // Pet the watchdog

  server.handleClient(); // Handle incoming HTTP requests

  unsigned long currentMillis = millis();

  // Reconnect WiFi if disconnected
  static unsigned long lastWifiCheck = 0;
  if (currentMillis - lastWifiCheck > 10000) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Disconnected, reconnecting...");
      WiFi.reconnect();
    }
  }

  // ---- 1. SENSOR READING + PID CALCULATION (every INTERVAL_READINGS ms) ----

  if (currentMillis - previousMillisReadings >= INTERVAL_READINGS) {
    previousMillisReadings = currentMillis;
    float temperature = readTemperature();

    // --- SAFETY: Sensor fault check ---
    if (temperature < -900.0) {
      sensorFault = true;
      setRelay(false); // Kill heating immediately
      timeOn = 0;
      Serial.println("{\"error\":\"SENSOR_FAULT\",\"action\":\"HEATING_OFF\"}");
      return; // Skip PID this cycle
    }
    sensorFault = false;

    // If system is disabled, still read temp (for monitoring) but skip PID
    if (!systemEnabled) {
      bool relayOn = false;
      latestJson = buildJson(temperature, latestHumidity, 0, 0, 0, 0, 0, relayOn, currentMillis);
      Serial.println(latestJson);
      return;
    }

    // --- SAFETY: Over-temperature cutoff ---
    if (temperature > TEMP_MAX_CUTOFF) {
      overTempFault = true;
      setRelay(false); // Kill heating immediately
      timeOn = 0;
      errorAccumulated = 0.0; // Reset integral to prevent windup
      Serial.print("{\"error\":\"OVER_TEMP\",\"temperatura\":");
      Serial.print(temperature, 2);
      Serial.println(",\"action\":\"HEATING_OFF\"}");
      return; // Skip PID this cycle
    }
    overTempFault = false;

    // --- PID CALCULATION ---

    float e  = setpoint - temperature;
    float dt = INTERVAL_READINGS / 1000.0; // Convert to seconds for proper units

    // Proportional
    float pTerm = Kp * e;

    // Integral with time-normalization and conditional anti-windup
    // Only accumulate when output is not saturated.
    // The ±50 clamp handles large disturbances (e.g., box lid opened during inspection)
    // preventing the integral from spiraling while the system recovers.
    float tentativeOutput = Kp * e + Ki * errorAccumulated + Kd * ((e - previousError) / dt);
    if (tentativeOutput > 0.0 && tentativeOutput < 100.0) {
      errorAccumulated += e * dt;
    }
    // Hard clamp — anti-windup protection
    if (errorAccumulated > 50.0) errorAccumulated = 50.0;
    if (errorAccumulated < -50.0) errorAccumulated = -50.0;

    float iTerm = Ki * errorAccumulated;

    // Derivative with time-normalization
    float derivative = (e - previousError) / dt;
    previousError = e;
    float dTerm = Kd * derivative;

    // Final PID output (clamped to 0–100%)
    float pidOutput = pTerm + iTerm + dTerm;
    if (pidOutput > 100.0) pidOutput = 100.0;
    if (pidOutput < 0.0) pidOutput = 0.0;

    // Convert power percentage to relay ON time within the PWM period
    timeOn = (unsigned long)((pidOutput / 100.0) * PERIOD_PWM);

    // --- BUILD JSON + OUTPUT ---
    bool relayOn = (currentMillis - previousMillisPWM) < activeTimeOn;
    latestJson = buildJson(temperature, latestHumidity, e, pidOutput, pTerm, iTerm, dTerm, relayOn, currentMillis);

    // Also output to Serial for debugging
    Serial.println(latestJson);
  }

  // ---- 2. RELAY PWM CONTROL ----

  // At the start of each new PWM period, latch the calculated timeOn
  // This prevents erratic relay switching if PID updates mid-period
  if (currentMillis - previousMillisPWM >= PERIOD_PWM) {
    previousMillisPWM = currentMillis;
    activeTimeOn = timeOn; // Latch the new duty cycle for this period
  }

  // Drive the relay based on the latched duty cycle
  if (systemEnabled && !sensorFault && !overTempFault) {
    if ((currentMillis - previousMillisPWM) < activeTimeOn) {
      setRelay(true);
    } else {
      setRelay(false);
    }
  } else {
    setRelay(false); // Disabled or fault state: always OFF
  }
}