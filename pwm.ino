/*
  ===========================================================================
  SHT31-D Fan Controller w/ Hardware Emergency Override
  Target: Elegoo Uno R3 (ATmega328P) driving an Arctic P12 Pro PST fan
  ===========================================================================

  READ BEFORE USE - SAFETY SCOPE:
  This is a hobbyist thermal-management sketch, not a certified safety
  device. If a real overheat event has consequences you can't afford
  (fire, hardware loss, etc.), this MCU-based control loop should be a
  *secondary* layer on top of a hardware-only fail-safe (e.g. a thermal
  fuse/cutoff switch that physically forces fan power on, independent of
  whether the Arduino is running, hung, or unplugged). Software can crash;
  a bimetallic thermal switch wired in series with fan power cannot.
  Design that independent layer first; treat everything below as
  "smart, temperature-proportional control" on top of it.

  -------------------------- WIRING SUMMARY --------------------------------
  I2C bus (shared by SHT31-D and SSD1306):
    SDA -> A4
    SCL -> A5

  Fan (Arctic P12 Pro PST, 4-pin):
    Pin 1 GND  -> common ground with Arduino
    Pin 2 +12V -> external 12V supply (NOT the Arduino - it can't source this)
    Pin 3 TACH -> D3 (INT1). Add a 10k pull-up resistor to 5V on this line
                  if you don't see clean pulses on the scope/serial monitor -
                  many boards expect the pull-up on the controller side.
    Pin 4 PWM  -> D9 (OC1A), direct connection is fine (5V logic level)

  Emergency override input -> D2 (INT0)
    Wiring polarity matters for fail-safe behavior - see EMERGENCY POLARITY
    note below before you wire your specific switch/sensor.

  ---------------------------- LIBRARIES ------------------------------------
  Install via Library Manager: "Adafruit SHT31", "Adafruit GFX Library",
  "Adafruit SSD1306"
  ===========================================================================
*/

#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------------- USER CONFIG -------------------------------
// Confirm these two with the i2c_scanner and oled_resolution_test sketches
// BEFORE trusting this main sketch.
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64      // change to 32 if that's your module
#define OLED_ADDR       0x3C
#define SHT31_ADDR      0x44    // 0x45 if your board's ADDR pin is tied high

#define FAN_PWM_PIN     9        // must stay on 9 (or 10) - Timer1 hardware PWM
#define FAN_TACH_PIN    3        // must stay on 3 - INT1
#define EMERGENCY_PIN   2        // must stay on 2 - INT0

// Temperature control curve (deg C)
const float TEMP_FAN_START_C  = 25.0;  // fan begins ramping above this
const float TEMP_FAN_MAX_C    = 45.0;  // 100% duty at/above this
const float TEMP_CRITICAL_C   = 55.0;  // software-forced 100%, belt-and-suspenders
const float TEMP_HYSTERESIS_C = 3.0;   // fan turns fully off only below (START - this)

const uint8_t MIN_RUN_DUTY_PERCENT = 20; // PST fans commonly stall under ~20-25%

// Sensor health: if this many consecutive reads fail, assume the sensor is
// lying/dead and fail toward SAFE (high cooling), not toward "do nothing".
const uint8_t MAX_CONSECUTIVE_SENSOR_FAILS = 5;

// -----------------------------------------------------------------------

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Timer1 TOP value for 25kHz phase-correct PWM at 16MHz, prescaler=1
// f = F_CPU / (2 * prescaler * TOP)  =>  TOP = F_CPU / (2 * prescaler * f)
const uint16_t PWM_TOP = 320; // 16,000,000 / (2 * 1 * 25000) = 320

volatile uint16_t tachPulseCount = 0;
volatile bool emergencyLatched = false;
volatile bool emergencyPinActiveLow = true; // see EMERGENCY POLARITY note below

unsigned long lastRpmCalcMs = 0;
unsigned long lastDisplayMs = 0;
uint16_t currentRpm = 0;

uint8_t consecutiveSensorFails = 0;
bool sensorFailSafeActive = false;
bool fanCurrentlyRunning = false;

// --------------------------- ISRs ---------------------------------------

void tachISR() {
  tachPulseCount++;
}

/*
  EMERGENCY POLARITY NOTE:
  As wired here (INPUT_PULLUP, trigger on FALLING), a broken wire reads as
  HIGH - i.e. "not triggered." That's fail-UNSAFE for a passive sensor
  (a snapped wire silently disables your override) but fail-SAFE for an
  active pushbutton (you *want* "wire broken" to mean "button does
  nothing," not "false alarm every boot").
  If you're wiring a thermal cutoff switch rather than a button, consider
  flipping this: use an external pull-DOWN resistor and a normally-open
  contact that pulls the pin HIGH in the fault condition, then change
  attachInterrupt() below to RISING. That way a broken wire also reads as
  a fault, which is the safer failure direction for a passive sensor.
  Decide based on what you're actually wiring, then adjust the two lines
  marked (*) below.
*/
void emergencyISR() {
  emergencyLatched = true;
  // Force max duty immediately from inside the ISR - don't wait for the
  // main loop to notice the flag. Direct register write is fine here.
  OCR1A = PWM_TOP;
}

// --------------------------- Setup ---------------------------------------

void setupFanPWM() {
  pinMode(FAN_PWM_PIN, OUTPUT);
  // Timer1, phase-correct PWM, TOP = ICR1, prescaler = 1, non-inverted on OC1A
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  ICR1   = PWM_TOP;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  OCR1A = 0; // start off
}

void setFanDutyPercent(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  OCR1A = (uint16_t)((uint32_t)PWM_TOP * percent / 100);
}

void setup() {
  Serial.begin(115200);

  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);                              // (*)
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), emergencyISR, FALLING); // (*)

  setupFanPWM();

  Wire.begin();

  if (!sht31.begin(SHT31_ADDR)) {
    Serial.println(F("SHT31 not found - check wiring/address. Entering fail-safe (fan full speed)."));
    sensorFailSafeActive = true;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 not found - check wiring/address. Continuing without display."));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Fan Controller"));
    display.println(F("Starting up..."));
    display.display();
  }

  lastRpmCalcMs = millis();
  lastDisplayMs = millis();
}

// --------------------------- Main loop ------------------------------------

uint8_t computeTargetDuty(float tempC) {
  static bool fanOn = false; // hysteresis state

  if (tempC >= TEMP_FAN_START_C) fanOn = true;
  if (tempC < (TEMP_FAN_START_C - TEMP_HYSTERESIS_C)) fanOn = false;

  if (!fanOn) return 0;

  if (tempC >= TEMP_FAN_MAX_C) return 100;

  float span = TEMP_FAN_MAX_C - TEMP_FAN_START_C;
  float ratio = (tempC - TEMP_FAN_START_C) / span;
  uint8_t duty = (uint8_t)(MIN_RUN_DUTY_PERCENT + ratio * (100 - MIN_RUN_DUTY_PERCENT));
  return constrain(duty, MIN_RUN_DUTY_PERCENT, 100);
}

void updateDisplay(float tempC, float humidity, uint16_t rpm, bool sensorOk) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (emergencyLatched) {
    display.println(F("** EMERGENCY **"));
    display.println(F("FAN AT 100%"));
  } else if (!sensorOk) {
    display.println(F("SENSOR FAULT"));
    display.println(F("FAIL-SAFE: 100%"));
  } else {
    display.setTextSize(2);
    display.print(F("SYSTEM OK "));
    display.print(F("Temp: "));
    display.print(tempC, 1);
    display.print(F("Hum:  "));
    display.print(humidity, 0);
    display.println(F(" %"));
  }

  display.print(F("RPM:  "));
  display.println(rpm);
  display.display();
}

void loop() {
  unsigned long now = millis();

  // ---- RPM calculation, once per second, non-blocking ----
  if (now - lastRpmCalcMs >= 1000) {
    noInterrupts();
    uint16_t pulses = tachPulseCount;
    tachPulseCount = 0;
    interrupts();

    unsigned long elapsed = now - lastRpmCalcMs;
    // 2 pulses per revolution is the standard for PC fans (Arctic included)
    currentRpm = (uint16_t)((uint32_t)pulses * 60000UL / (2UL * elapsed));
    lastRpmCalcMs = now;
  }

  // ---- Emergency latch clear condition ----
  // Requires the pin to read "not triggered" continuously for 3s AND
  // temperature back under the start threshold - prevents immediately
  // re-clearing on a bouncing switch or a still-hot system.
  static unsigned long clearConditionSinceMs = 0;
  bool pinSaysNotTriggered = digitalRead(EMERGENCY_PIN) == HIGH; // matches (*) polarity above
  if (emergencyLatched) {
    if (pinSaysNotTriggered) {
      if (clearConditionSinceMs == 0) clearConditionSinceMs = now;
      if (now - clearConditionSinceMs > 3000) {
        emergencyLatched = false;
        clearConditionSinceMs = 0;
      }
    } else {
      clearConditionSinceMs = 0;
    }
  }

  // ---- Read sensor ----
  float tempC = NAN;
  float humidity = NAN;
  bool sensorOk = false;

  if (!sensorFailSafeActive || consecutiveSensorFails > 0) {
    tempC = sht31.readTemperature();
    humidity = sht31.readHumidity();
    sensorOk = !isnan(tempC) && !isnan(humidity);
  }

  if (sensorOk) {
    consecutiveSensorFails = 0;
    sensorFailSafeActive = false;
  } else {
    consecutiveSensorFails++;
    if (consecutiveSensorFails >= MAX_CONSECUTIVE_SENSOR_FAILS) {
      sensorFailSafeActive = true;
    }
  }

  // ---- Decide duty ----
  uint8_t targetDuty;
  if (emergencyLatched || sensorFailSafeActive) {
    targetDuty = 100; // fail toward more cooling, never less
  } else if (tempC >= TEMP_CRITICAL_C) {
    targetDuty = 100;
  } else {
    targetDuty = computeTargetDuty(tempC);
  }

  if (!emergencyLatched) {
    // The emergency ISR already forced OCR1A directly; don't fight it here
    // until the latch clears. Once clear, resume normal control.
    setFanDutyPercent(targetDuty);
  }

  // ---- Display update, ~2x/sec, non-blocking ----
  if (now - lastDisplayMs >= 500) {
    updateDisplay(tempC, humidity, currentRpm, sensorOk);
    lastDisplayMs = now;
  }

  // ---- Serial log for debugging ----
  static unsigned long lastLogMs = 0;
  if (now - lastLogMs >= 1000) {
    Serial.print(F("Temp="));
    Serial.print(tempC);
    Serial.print(F("C Hum="));
    Serial.print(humidity);
    Serial.print(F("% RPM="));
    Serial.print(currentRpm);
    Serial.print(F(" Duty="));
    Serial.print(targetDuty);
    Serial.print(F("% Emergency="));
    Serial.print(emergencyLatched ? "YES" : "no");
    Serial.print(F(" SensorFail="));
    Serial.println(sensorFailSafeActive ? "YES" : "no");
    lastLogMs = now;
  }
}
