/*
  ESP32 Server Cabinet Fan Controller - Final Production Version
  ----------------------------------------------------------------
  Hardware: ESP32, SHT31-D (0x44), Elegoo Bi-Color OLED (0x3C), Arctic P12/F14 PWM
  
  Features:
    - 24/7 Hardware-safety critical design (Watchdog, Active-HIGH Alert)
    - SHT31 Periodic Data Acquisition Mode
    - Non-blocking Wi-Fi dashboard & WebSockets
    - Clean Serial logging (Boot & Network state changes only)
    - Anti-Stall Kickstart (Tach watchdog)
    - True Non-blocking Soft Start
    - Bi-color OLED layout optimization
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_task_wdt.h"

// ---------------------------------------------------------------------
// Wi-Fi Settings
// ---------------------------------------------------------------------
const char* ssid = "YOUR_SSID";         // CHANGE THIS
const char* password = "YOUR_PASSWORD"; // CHANGE THIS

WebServer server(80);
WebSocketsServer webSocket(81);
unsigned long lastWifiRetry = 0;
const unsigned long WIFI_RETRY_INTERVAL = 30000; // 30 seconds
bool webEmergencyOverride = false;
bool lastWifiState = false; 

// ---------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define PIN_ALERT     4     // SHT31 ALERT pin (push-pull, active HIGH, using pull-down)
#define PIN_FAN_PWM   25    
#define PIN_FAN_TACH  34    // WITH 10k PULL-UP TO 3.3V

#define SHT31_ADDR    0x44
#define OLED_ADDR     0x3C
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ---------------------------------------------------------------------
// Fan curve & Safety config
// ---------------------------------------------------------------------
const float TEMP_FAN_ON      = 30.0;  
const float TEMP_FAN_OFF     = 27.0;  
const float TEMP_MIN_SPEED   = 30.0;  
const float TEMP_MAX_SPEED   = 45.0;  
const uint8_t DUTY_MIN       = 70;    
const uint8_t DUTY_MAX       = 255;

const float ALERT_TEMP_SET   = 46.0;  
const float ALERT_TEMP_CLEAR = 43.0;  

const int PWM_CHANNEL   = 0;
const int PWM_FREQ_HZ   = 25000;  
const int PWM_RES_BITS  = 8;
const int TACH_PULSES_PER_REV = 2;

const unsigned long SENSOR_INTERVAL_MS  = 2000;
const unsigned long DISPLAY_INTERVAL_MS = 500;
const unsigned long TACH_WINDOW_MS      = 1000;
const int MAX_CONSECUTIVE_SENSOR_FAILS  = 5;
const uint8_t WDT_TIMEOUT_S             = 10;

// ---------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

volatile uint32_t tachPulseCount = 0;
void IRAM_ATTR tachISR() { tachPulseCount++; }

unsigned long lastSensorRead   = 0;
unsigned long lastDisplayDraw  = 0;
unsigned long lastTachWindow   = 0;
unsigned long lastRampTime     = 0;
unsigned long stallDetectTime  = 0;

float currentTempC   = NAN;
float currentHumRH   = NAN;
bool  sensorValid    = false;
int   consecutiveFails = 0;

bool fanOn = false;
uint8_t currentDuty = 0;
uint8_t targetDuty  = 0;
uint16_t currentRPM = 0;

bool hwEmergencyOverride = false;   
bool sensorFaultMode     = false;   

// ---------------------------------------------------------------------
// I2C & Sensor Helpers
// ---------------------------------------------------------------------
uint8_t sht31_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else crc <<= 1;
    }
  }
  return crc;
}

bool i2cWriteCommand(uint8_t addr, uint16_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmd >> 8); Wire.write(cmd & 0xFF);
  return (Wire.endTransmission() == 0);
}

bool i2cWriteCommandWithData(uint8_t addr, uint16_t cmd, uint16_t value) {
  uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
  uint8_t crc = sht31_crc8(buf, 2);
  Wire.beginTransmission(addr);
  Wire.write(cmd >> 8); Wire.write(cmd & 0xFF);
  Wire.write(buf[0]); Wire.write(buf[1]); Wire.write(crc);
  return (Wire.endTransmission() == 0);
}

uint16_t encodeAlertThreshold(float tempC, float rhPercent) {
  uint16_t rhRaw   = (uint16_t)round((rhPercent / 100.0) * 65535.0);
  uint16_t tempRaw = (uint16_t)round(((tempC + 45.0) / 175.0) * 65535.0);
  return ((rhRaw >> 9) << 9) | (tempRaw >> 7);
}

bool configureSHT31Alerts() {
  bool ok = true;
  ok &= i2cWriteCommandWithData(SHT31_ADDR, 0x611D, encodeAlertThreshold(ALERT_TEMP_SET, 99.0));
  ok &= i2cWriteCommandWithData(SHT31_ADDR, 0x6116, encodeAlertThreshold(ALERT_TEMP_CLEAR, 99.0));
  ok &= i2cWriteCommandWithData(SHT31_ADDR, 0x6100, encodeAlertThreshold(-40.0, 0.0));
  ok &= i2cWriteCommandWithData(SHT31_ADDR, 0x610B, encodeAlertThreshold(-39.0, 0.0));
  return ok;
}

void initSHT31PeriodicMode() {
  // 1. Clear Status Register (Resets the ALERT pin to LOW)
  i2cWriteCommand(SHT31_ADDR, 0x3041); 
  delay(20);
  
  // 2. Write the High/Low temperature safety thresholds
  configureSHT31Alerts(); 
  
  // 3. Start Periodic Mode (1 Measurement Per Second, High Repeatability)
  i2cWriteCommand(SHT31_ADDR, 0x2130);
  delay(20);
}

bool readSHT31(float &tempC, float &rh) {
  // Command 0xE000 fetches the latest data from the periodic background task
  if (!i2cWriteCommand(SHT31_ADDR, 0xE000)) return false; 
  delay(10); 
  
  if (Wire.requestFrom((int)SHT31_ADDR, 6) != 6) return false;

  uint8_t buf[6];
  for (int i = 0; i < 6; i++) buf[i] = Wire.read();

  if (sht31_crc8(buf, 2) != buf[2] || sht31_crc8(buf + 3, 2) != buf[5]) return false;

  uint16_t rawT = (buf[0] << 8) | buf[1];
  uint16_t rawH = (buf[3] << 8) | buf[4];

  tempC = -45.0 + 175.0 * ((float)rawT / 65535.0);
  rh    = 100.0 * ((float)rawH / 65535.0);
  return true;
}

void resetI2CBus() {
  Wire.end();
  delay(20);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
}

// ---------------------------------------------------------------------
// Non-Blocking Fan Control
// ---------------------------------------------------------------------
uint8_t computeDuty(float tempC) {
  if (tempC <= TEMP_MIN_SPEED) return DUTY_MIN;
  if (tempC >= TEMP_MAX_SPEED) return DUTY_MAX;
  float frac = (tempC - TEMP_MIN_SPEED) / (TEMP_MAX_SPEED - TEMP_MIN_SPEED);
  return (uint8_t)(DUTY_MIN + frac * (DUTY_MAX - DUTY_MIN));
}

void processFanDuty() {
  if (currentDuty == targetDuty) return;

  // Step the fan duty by 1 every 15ms (smooth, non-blocking soft start)
  if (millis() - lastRampTime >= 15) {
    if (currentDuty < targetDuty) currentDuty++;
    else currentDuty--;
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_FAN_PWM, currentDuty);
#else
    ledcWrite(PWM_CHANNEL, currentDuty);
#endif

    lastRampTime = millis();
  }
}

// ---------------------------------------------------------------------
// Web Server Setup
// ---------------------------------------------------------------------
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Cabinet Fan Dashboard</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family: Arial;background:#f4f4f9;padding:20px;}";
  html += ".card{background:#fff;border-radius:8px;padding:20px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}";
  html += ".btn{background:#007BFF;color:white;padding:12px 20px;border:none;border-radius:5px;cursor:pointer;margin-right:10px;}";
  html += ".btn-danger{background:#dc3545;} .btn-secondary{background:#6c757d;}";
  html += ".emergency{color:#dc3545;font-weight:bold;}</style></head><body>";
  html += "<h2>Cabinet Dashboard</h2><div class='card'>";
  html += "<p><strong>Temp:</strong> <span id='temp'>--</span>&deg;C</p>";
  html += "<p><strong>Humidity:</strong> <span id='hum'>--</span>%</p>";
  html += "<p><strong>Fan:</strong> <span id='fan'>--</span>% (<span id='rpm'>--</span> RPM)</p>";
  html += "<p><strong>Status:</strong> <span id='status'>--</span></p>";
  html += "<hr><button class='btn btn-danger' onclick='sendMsg(\"emergency\")'>Web 100%</button>";
  html += "<button class='btn btn-secondary' onclick='sendMsg(\"auto\")'>Auto Mode</button>";
  html += "</div><script>";
  html += "const ws = new WebSocket('ws://' + window.location.host + ':81');";
  html += "ws.onmessage = e => { const d = JSON.parse(e.data);";
  html += "document.getElementById('temp').textContent = d.temp;";
  html += "document.getElementById('hum').textContent = d.hum;";
  html += "document.getElementById('fan').textContent = d.fan;";
  html += "document.getElementById('rpm').textContent = d.rpm;";
  html += "document.getElementById('status').textContent = d.status;";
  html += "document.getElementById('status').className = d.alert ? 'emergency' : ''; };";
  html += "function sendMsg(msg) { ws.send(msg); }";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg == "emergency") webEmergencyOverride = true;
    else if (msg == "auto") webEmergencyOverride = false;
  }
}

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBooting Fan Controller...");

  // Watchdog
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  esp_task_wdt_config_t wdtConfig = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&wdtConfig);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  pinMode(PIN_ALERT, INPUT_PULLDOWN); 
  pinMode(PIN_FAN_TACH, INPUT);    
  attachInterrupt(digitalPinToInterrupt(PIN_FAN_TACH), tachISR, FALLING);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_FAN_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_FAN_PWM, PWM_CHANNEL);
#endif

  // Safe startup state 
  targetDuty = 120;
  currentDuty = 120;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_FAN_PWM, currentDuty);
#else
  ledcWrite(PWM_CHANNEL, currentDuty);
#endif

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay(); 
    display.setTextSize(1); 
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); // This inherently lands perfectly in the Yellow zone
    display.println("Booting System..."); 
    display.display();
  }

  // Initialize SHT31 for Periodic Mode and Clear the Status Register
  initSHT31PeriodicMode();

  // Non-blocking Wi-Fi attempt (Max 5 seconds)
  Serial.print("Attempting Wi-Fi connection...");
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart < 5000)) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" Timeout. Will retry in background.");
  }

  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("\nSystem Setup Complete.");
}

// ---------------------------------------------------------------------
// Main loop 
// ---------------------------------------------------------------------
void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();
  
  // Background Wi-Fi Reconnection (Non-blocking)
  if (WiFi.status() != WL_CONNECTED && (now - lastWifiRetry >= WIFI_RETRY_INTERVAL)) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    lastWifiRetry = now;
  }

  // Log Wi-Fi state changes and print IP
  bool currentWifiState = (WiFi.status() == WL_CONNECTED);
  if (currentWifiState && !lastWifiState) {
    Serial.println();
    Serial.print("Wi-Fi Connected! IP Address: ");
    Serial.println(WiFi.localIP());
  } else if (!currentWifiState && lastWifiState) {
    Serial.println();
    Serial.println("Wi-Fi Disconnected. Will retry in background...");
  }
  lastWifiState = currentWifiState;

  // Process web requests
  server.handleClient();
  webSocket.loop();

  // Hardware emergency check: reads active HIGH directly from sensor
  hwEmergencyOverride = (digitalRead(PIN_ALERT) == HIGH);

  // Tach measurement window 
  if (now - lastTachWindow >= TACH_WINDOW_MS) {
    noInterrupts();
    uint32_t pulses = tachPulseCount;
    tachPulseCount = 0;
    interrupts();

    float windowSec = (now - lastTachWindow) / 1000.0;
    if (windowSec <= 0) windowSec = TACH_WINDOW_MS / 1000.0;
    currentRPM = (uint16_t)((pulses / (float)TACH_PULSES_PER_REV) * (60.0 / windowSec));
    lastTachWindow = now;
  }

  // Sensor read 
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    float t, h;
    if (readSHT31(t, h)) {
      currentTempC = t;
      currentHumRH = h;
      sensorValid = true;
      consecutiveFails = 0;
      sensorFaultMode = false;
    } else {
      consecutiveFails++;
      if (consecutiveFails >= MAX_CONSECUTIVE_SENSOR_FAILS) {
        sensorValid = false;
        sensorFaultMode = true;
        resetI2CBus();
        initSHT31PeriodicMode(); // Re-initialize periodic mode after a bus reset
      }
    }
  }

  // Decide fan speed target
  if (hwEmergencyOverride || webEmergencyOverride) {
    targetDuty = DUTY_MAX;
    fanOn = true;
  } else if (sensorFaultMode) {
    targetDuty = 180; // Fail toward MORE cooling
    fanOn = true;
  } else if (sensorValid) {
    if (!fanOn && currentTempC >= TEMP_FAN_ON) fanOn = true;
    if (fanOn && currentTempC <= TEMP_FAN_OFF) fanOn = false;
    targetDuty = fanOn ? computeDuty(currentTempC) : 0;
  }

  // Anti-Stall Kickstart: If fan should be moving but RPM is 0 for 2+ seconds
  if (targetDuty > 0 && currentRPM == 0) {
    if (stallDetectTime == 0) stallDetectTime = now;
    else if (now - stallDetectTime > 2000) targetDuty = DUTY_MAX; // Kickstart it
  } else {
    stallDetectTime = 0;
  }

  // Apply smooth transition
  processFanDuty();

  // WebSocket Updates (Every 1s to avoid flooding)
  static unsigned long lastWsUpdate = 0;
  if (now - lastWsUpdate >= 1000 && WiFi.status() == WL_CONNECTED) {
    lastWsUpdate = now;
    String statusStr = hwEmergencyOverride ? "HW OVERRIDE" : (webEmergencyOverride ? "WEB OVERRIDE" : (sensorFaultMode ? "SENSOR FAULT" : (fanOn ? "COOLING" : "IDLE")));
    if (targetDuty > 0 && currentRPM == 0 && (now - stallDetectTime > 2000)) statusStr = "FAN STALL KICK!";
    
    String json = "{\"temp\":\"" + (sensorValid ? String(currentTempC, 1) : "--") +
                  "\",\"hum\":\"" + (sensorValid ? String(currentHumRH, 1) : "--") +
                  "\",\"fan\":\"" + String((currentDuty * 100) / 255) +
                  "\",\"rpm\":\"" + String(currentRPM) +
                  "\",\"status\":\"" + statusStr +
                  "\",\"alert\":" + (hwEmergencyOverride || webEmergencyOverride || sensorFaultMode ? "true" : "false") + "}";
    webSocket.broadcastTXT(json);
  }

  // Display Updates
  if (now - lastDisplayDraw >= DISPLAY_INTERVAL_MS) {
    lastDisplayDraw = now;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // --- YELLOW SECTION (Top 16 pixels) ---
    display.setCursor(0, 0); 
    if (hwEmergencyOverride) display.println("STATUS: HW OVERRIDE");
    else if (webEmergencyOverride) display.println("STATUS: WEB OVERRIDE");
    else if (sensorFaultMode) display.println("STATUS: SENSOR FAULT");
    else if (stallDetectTime > 0 && currentRPM == 0) display.println("STATUS: FAN STALLED!");
    else if (fanOn) display.println("STATUS: COOLING");
    else display.println("STATUS: IDLE");

    // --- BLUE SECTION (Bottom 48 pixels) ---
    // Moving to Y=20 pushes the text safely past the yellow boundary
    display.setCursor(0, 20); 
    if (sensorValid) {
      display.printf("Temp:  %.1f C\nHumid: %.1f %%\n", currentTempC, currentHumRH);
    } else {
      display.println("Temp:  -- (fault)\nHumid: -- (fault)");
    }
    display.printf("Fan:   %d%%\nRPM:   %u\n", (currentDuty * 100) / 255, currentRPM);
    
    display.display();
  }
}
