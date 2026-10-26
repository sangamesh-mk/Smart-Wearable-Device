/*****************************************************************************
 * SMART WEARABLE DEVICE
 * ESP8266 NodeMCU (ESP-12E)
 *
 * FEATURES
 *  - OLED (SSD1306 128x64) live status display
 *  - DHT11 Temperature & Humidity
 *  - Real-time heartbeat (BPM) detection from an analog pulse sensor
 *  - MQ-2 gas sensor monitoring
 *  - MPU6050 accelerometer-based fall detection, PLUS gyroscope-based
 *    shake detection (shaking the sensor triggers the same alert - handy
 *    for testing/demoing without an actual drop)
 *  - Blynk IoT cloud integration (virtual pins, events, SuperChart)
 *  - LED blinks + active buzzer beeps during a fall/shake alert
 *
 * PIN MAP
 *  OLED   SDA -> D2   SCL -> D1   (shared I2C bus with MPU6050)
 *  MPU6050 SDA -> D2  SCL -> D1
 *  DHT11  DATA -> D4
 *  Pulse Sensor Signal -> A0
 *  MQ2    Digital Out -> D7
 *  LED    -> D5
 *  Buzzer -> D6
 *
 * VIRTUAL PINS
 *  V0 Fall Detection   V1 Temperature   V2 Gas Status
 *  V3 Pulse BPM        V4 LED Control   V5 OLED Enable/Disable
 *  V6 System Enable    V7 SuperChart    V8 Humidity
 *****************************************************************************/

// ---------------- Blynk Credentials (must be defined before Blynk include) ----------------
#define BLYNK_TEMPLATE_ID   "TMPL3CpM8gFi3"
#define BLYNK_TEMPLATE_NAME "SMART WEARABLE DEVICE"
#define BLYNK_AUTH_TOKEN    "e7QWXjskuJn72SO1Dkz5Jkt2MR-47INW"
#define BLYNK_PRINT Serial

// ---------------- Libraries ----------------
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- WiFi & Blynk Credentials ----------------
char ssid[] = "Sangamesh MK";
char pass[] = "12345678910";
char auth[] = BLYNK_AUTH_TOKEN;

// ---------------- Pin Definitions ----------------
#define DHTPIN     D4
#define DHTTYPE    DHT11
#define PULSE_PIN  A0
#define MQ2_PIN    D7
#define LED_PIN    D5
#define BUZZER_PIN D6

// ---------------- OLED Configuration ----------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- Sensor Objects ----------------
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

// ---------------- Blynk Timer ----------------
BlynkTimer timer;

// ================================================================
// GLOBAL STATE
// ================================================================

// System / widget state (controlled from Blynk)
bool systemEnabled  = true;   // V6
bool oledEnabled    = true;   // V5
bool manualLedState = false;  // V4

// DHT11 readings
float temperature = 0.0;
float humidity    = 0.0;

// ---- Pulse sensor (real beat-to-beat detection, no fake mapping) ----
int  pulseBaseline = 512;              // Slowly-adapting rolling baseline of the analog signal
bool pulsePeakDetected = false;        // True while signal is above threshold (prevents re-triggering)
unsigned long lastBeatTime = 0;        // Timestamp of the last confirmed beat
int  bpm = 0;                          // Smoothed BPM value
int  bpmArray[4] = {0, 0, 0, 0};       // Rolling buffer of the last 4 instantaneous BPM readings
int  bpmIndex = 0;
const int PULSE_THRESHOLD_OFFSET = 40; // How far above the baseline counts as a heartbeat pulse

// Gas sensor
bool gasDetected = false;

// MPU6050 readings
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float totalAccel = 0.0;
float totalGyro  = 0.0;

// ---- Fall detection state machine ----
// Stage 1: free-fall (near weightlessness, total accel drops close to 0)
// Stage 2: impact spike within a short window after the free-fall
enum FallState { FALL_IDLE, FALL_FREEFALL_DETECTED };
FallState fallState = FALL_IDLE;
unsigned long freeFallTimestamp = 0;
const float FREE_FALL_THRESHOLD   = 3.0;   // m/s^2 - near-zero G during a fall
const float IMPACT_THRESHOLD      = 18.0;  // m/s^2 - sudden shock on landing
const unsigned long FALL_WINDOW_MS = 700;  // Max time between free-fall and impact

// ---- Shake / sudden-movement detection (gyroscope) ----
// Triggers the exact same alert as a real fall (OLED + buzzer + LED blink),
// so you can demo/test the whole alert pipeline just by shaking the MPU6050
// instead of needing an actual drop.
const float SHAKE_GYRO_THRESHOLD = 4.0; // rad/s - lower = more sensitive, raise = needs a harder shake

bool fallDetected = false;
unsigned long fallDetectedTime = 0;
const unsigned long FALL_DISPLAY_HOLD_MS = 5000; // How long the fall/shake alert stays latched
bool ledBlinkOn = false; // Toggled by blinkAlertLED() while an alert is active

// Alert de-duplication flags (so each event fires once per episode, not every 2s)
bool highTempAlertSent = false;
bool highHRAlertSent   = false;
bool lowHRAlertSent    = false;
bool gasAlertSent      = false;
bool fallAlertSent     = false;

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================
void connectWiFiAndBlynk();
void showStartupScreen();
void samplePulse();
void readDHT();
void readPulse();
void readMQ2();
void sampleMPU6050();
void readMPU6050();
void updateOLED();
void sendToBlynk();
void checkAlerts();
void blinkAlertLED();

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MQ2_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Shared I2C bus for OLED (0x3C) and MPU6050 (0x68) - no address conflict
  Wire.begin(D2, D1);

  // ---- OLED init ----
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 OLED allocation failed"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  showStartupScreen();

  // ---- DHT11 init ----
  dht.begin();

  // ---- MPU6050 init ----
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 not found. Check wiring."));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // ---- WiFi + Blynk ----
  connectWiFiAndBlynk();

  // ---- Timers (no delay() is used inside loop()) ----
  timer.setInterval(20L,   samplePulse);    // Fast sampling needed for real beat detection
  timer.setInterval(100L,  sampleMPU6050);  // Fast sampling so a brief fall/shake isn't missed
  timer.setInterval(250L,  blinkAlertLED);  // Makes the LED actually blink during an alert
  timer.setInterval(2000L, readDHT);
  timer.setInterval(2000L, readPulse);
  timer.setInterval(2000L, readMQ2);
  timer.setInterval(2000L, readMPU6050);    // Serial reporting only (data sampled above)
  timer.setInterval(2000L, updateOLED);
  timer.setInterval(2000L, sendToBlynk);
  timer.setInterval(2000L, checkAlerts);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println(F("System Ready"));
  display.display();
  Serial.println(F("System Ready"));
}

// ================================================================
// LOOP - kept clean, all real work happens through BlynkTimer
// ================================================================
void loop() {
  Blynk.run();
  timer.run();
}

// ================================================================
// STARTUP / CONNECTION SCREENS
// ================================================================
void showStartupScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("SMART WEARABLE"));
  display.println(F("DEVICE"));
  display.println();
  display.println(F("Connecting WiFi..."));
  display.display();
}

void connectWiFiAndBlynk() {
  WiFi.begin(ssid, pass);
  Serial.print(F("Connecting to WiFi"));
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); // Blocking only during initial setup(), never inside loop()
    Serial.print(F("."));
  }
  Serial.println();
  Serial.println(F("WiFi Connected"));

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("WiFi Connected"));
  display.println(F("Connecting Blynk..."));
  display.display();

  Blynk.config(auth);
  Blynk.connect(5000);

  Serial.println(F("Blynk Connected"));
}

// ================================================================
// PULSE SENSOR - fast sampling (called every 20ms via timer)
// Real threshold-crossing + beat-interval algorithm (no fake BPM mapping)
// ================================================================
void samplePulse() {
  if (!systemEnabled) return;

  int signalValue = analogRead(PULSE_PIN); // 0-1023
  unsigned long now = millis();

  // Slowly-adapting baseline tracks the resting signal level so the
  // threshold self-adjusts to each person's sensor placement/skin contact
  pulseBaseline = (pulseBaseline * 15 + signalValue) / 16;

  int dynamicThreshold = pulseBaseline + PULSE_THRESHOLD_OFFSET;

  // Rising edge above threshold = candidate heartbeat
  if (signalValue > dynamicThreshold && !pulsePeakDetected) {
    unsigned long interval = now - lastBeatTime;

    // Only accept intervals inside a physiologically valid range
    // (40-200 BPM -> 300ms-1500ms) to reject noise/double triggers
    if (interval > 300 && interval < 1500) {
      int instantBPM = 60000 / interval;

      bpmArray[bpmIndex] = instantBPM;
      bpmIndex = (bpmIndex + 1) % 4;

      int sum = 0;
      for (int i = 0; i < 4; i++) sum += bpmArray[i];
      bpm = sum / 4; // Smoothed average of last 4 beats
    }

    lastBeatTime = now;
    pulsePeakDetected = true;
  }

  // Signal has to drop back below threshold before the next beat can register
  if (signalValue < dynamicThreshold - 10) {
    pulsePeakDetected = false;
  }
}

// ================================================================
// DHT11 - Temperature & Humidity
// ================================================================
void readDHT() {
  if (!systemEnabled) return;

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  temperature = t;
  humidity = h;

  Serial.print(F("Temperature: "));
  Serial.print(temperature);
  Serial.println(F(" C"));

  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.println(F(" %"));
}

// ================================================================
// PULSE REPORTING (BPM value itself is computed continuously in samplePulse)
// ================================================================
void readPulse() {
  if (!systemEnabled) return;

  // If no beat has registered recently, there is no valid contact/reading
  if (millis() - lastBeatTime > 3000) {
    bpm = 0;
  }

  Serial.print(F("Pulse BPM: "));
  Serial.println(bpm);
}

// ================================================================
// MQ-2 GAS SENSOR
// ================================================================
void readMQ2() {
  if (!systemEnabled) return;

  // NOTE: MQ2 breakout modules vary by comparator wiring - some pull the
  // DO pin HIGH when gas exceeds the threshold, others pull it LOW.
  // HIGH = gas detected is used here (the more common wiring); flip this
  // comparison if your module behaves the opposite way during testing.
  int gasState = digitalRead(MQ2_PIN);
  gasDetected = (gasState == HIGH);

  Serial.print(F("Gas Status: "));
  Serial.println(gasDetected ? F("Gas Detected") : F("Gas Normal"));
}

// ================================================================
// MPU6050 - FAST SAMPLING (called every 100ms)
// A real fall's free-fall + impact only lasts a few hundred ms, so this
// needs to run much faster than the 2s reporting cycle or it gets missed.
// ================================================================
void sampleMPU6050() {
  if (!systemEnabled) return;

  // Auto-clear a latched fall/shake alert after the hold period so a new
  // one can be detected, and make sure the blinking LED is switched off
  if (fallDetected && (millis() - fallDetectedTime > FALL_DISPLAY_HOLD_MS)) {
    fallDetected = false;
    fallAlertSent = false;
    ledBlinkOn = false;
    if (!manualLedState) {
      digitalWrite(LED_PIN, LOW); // don't leave the LED stuck on after the alert clears
    }
  }

  sensors_event_t a, g, temp_evt;
  mpu.getEvent(&a, &g, &temp_evt);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  totalAccel = sqrt(accX * accX + accY * accY + accZ * accZ);
  totalGyro  = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // ---- Fall detection state machine (true fall via accelerometer) ----
  if (fallState == FALL_IDLE) {
    // Stage 1: near-weightlessness indicates the wearer may be in free fall
    if (totalAccel < FREE_FALL_THRESHOLD) {
      fallState = FALL_FREEFALL_DETECTED;
      freeFallTimestamp = millis();
    }
  } else if (fallState == FALL_FREEFALL_DETECTED) {
    if (millis() - freeFallTimestamp <= FALL_WINDOW_MS) {
      // Stage 2: a sharp impact shortly after free-fall confirms a real fall
      if (totalAccel > IMPACT_THRESHOLD) {
        fallDetected = true;
        fallDetectedTime = millis();
        fallState = FALL_IDLE;
      }
    } else {
      // No impact arrived in time - treat as a false alarm and reset
      fallState = FALL_IDLE;
    }
  }

  // ---- Shake / sudden-movement detection (gyroscope) ----
  // Shaking the sensor by hand spikes angular velocity well above normal
  // wear/movement, so this triggers the exact same alert as a real fall.
  if (!fallDetected && totalGyro > SHAKE_GYRO_THRESHOLD) {
    fallDetected = true;
    fallDetectedTime = millis();
    fallState = FALL_IDLE; // cancel any in-progress free-fall watch
    Serial.println(F("Shake / sudden movement detected on gyroscope!"));
  }
}

// ================================================================
// MPU6050 - SERIAL REPORTING (every 2 seconds)
// Just reports the values that sampleMPU6050() has already been keeping
// up to date in the background.
// ================================================================
void readMPU6050() {
  if (!systemEnabled) return;

  Serial.print(F("Acceleration X: ")); Serial.print(accX);
  Serial.print(F(" Y: ")); Serial.print(accY);
  Serial.print(F(" Z: ")); Serial.println(accZ);

  Serial.print(F("Fall Status: "));
  Serial.println(fallDetected ? F("FALL DETECTED") : F("Normal"));
}

// ================================================================
// LED BLINK - toggles the LED every 250ms while an alert (fall or shake)
// is active, so the light actually blinks instead of staying steady on
// ================================================================
void blinkAlertLED() {
  if (!systemEnabled) return;

  if (fallDetected) {
    ledBlinkOn = !ledBlinkOn;
    digitalWrite(LED_PIN, ledBlinkOn ? HIGH : LOW);
  }
}

// ================================================================
// OLED DISPLAY UPDATE
// ================================================================
void updateOLED() {
  if (!oledEnabled) return; // V5 disables OLED updates

  display.clearDisplay();

  if (!systemEnabled) {
    display.setTextSize(2);
    display.setCursor(15, 25);
    display.println(F("SYSTEM OFF"));
    display.display();
    return;
  }

  if (fallDetected) {
    // Dedicated full-screen fall alert
    display.setTextSize(2);
    display.setCursor(10, 8);
    display.println(F("!! FALL !!"));
    display.setTextSize(1);
    display.setCursor(0, 38);
    display.println(F("FALL DETECTED"));
    display.setCursor(0, 50);
    display.println(F("Check on user!"));
    display.display();
    return;
  }

  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print(F("Temp: "));
  display.print(temperature, 1);
  display.println(F(" C"));

  display.setCursor(0, 12);
  display.print(F("Humidity: "));
  display.print(humidity, 1);
  display.println(F(" %"));

  display.setCursor(0, 24);
  display.print(F("Pulse: "));
  display.print(bpm);
  display.println(F(" BPM"));

  display.setCursor(0, 36);
  display.print(F("Gas: "));
  display.println(gasDetected ? F("DETECTED") : F("Normal"));

  display.setCursor(0, 48);
  display.print(F("Fall: "));
  display.println(F("Normal"));

  display.display();
}

// ================================================================
// SEND SENSOR DATA TO BLYNK
// ================================================================
void sendToBlynk() {
  if (!systemEnabled) return;
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V8, humidity);
  Blynk.virtualWrite(V3, bpm);
  Blynk.virtualWrite(V2, gasDetected ? "Gas Detected" : "Gas Normal");
  Blynk.virtualWrite(V0, fallDetected ? 1 : 0);

  // SuperChart - Temperature, Humidity, Pulse BPM plotted together
  Blynk.virtualWrite(V7, temperature, humidity, bpm);
}

// ================================================================
// ALERT LOGIC - buzzer + Blynk events
// ================================================================
void checkAlerts() {
  if (!systemEnabled) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  bool buzzerShouldBeOn = false;

  // ---- High temperature ----
  if (temperature > 38.0) {
    buzzerShouldBeOn = true;
    if (!highTempAlertSent) {
      Blynk.logEvent("high_temp", "High temperature detected!");
      highTempAlertSent = true;
    }
  } else {
    highTempAlertSent = false;
  }

  // ---- High heart rate ----
  if (bpm > 120) {
    buzzerShouldBeOn = true;
    if (!highHRAlertSent) {
      Blynk.logEvent("high_hr", "High heart rate detected!");
      highHRAlertSent = true;
    }
  } else {
    highHRAlertSent = false;
  }

  // ---- Low heart rate (only meaningful once a real reading exists) ----
  if (bpm > 0 && bpm < 50) {
    buzzerShouldBeOn = true;
    if (!lowHRAlertSent) {
      Blynk.logEvent("low_hr", "Low heart rate detected!");
      lowHRAlertSent = true;
    }
  } else {
    lowHRAlertSent = false;
  }

  // ---- Gas detection ----
  if (gasDetected) {
    buzzerShouldBeOn = true;
    // Don't override the LED while it's actively blinking for a fall/shake alert
    if (!fallDetected) {
      digitalWrite(LED_PIN, HIGH);
    }
    if (!gasAlertSent) {
      Blynk.logEvent("gas_detected", "Gas detected!");
      gasAlertSent = true;
    }
  } else {
    gasAlertSent = false;
    // Only auto-clear the LED if the user hasn't manually forced it on via V4,
    // and only if a fall/shake alert isn't currently blinking it
    if (!manualLedState && !fallDetected) {
      digitalWrite(LED_PIN, LOW);
    }
  }

  // ---- Fall detection ----
  if (fallDetected) {
    buzzerShouldBeOn = true;
    if (!fallAlertSent) {
      Blynk.logEvent("fall_detected", "Fall detected!");
      fallAlertSent = true;
    }
  }

  digitalWrite(BUZZER_PIN, buzzerShouldBeOn ? HIGH : LOW);
}

// ================================================================
// BLYNK VIRTUAL PIN HANDLERS
// ================================================================

// V4 - Manual LED control
BLYNK_WRITE(V4) {
  manualLedState = param.asInt();
  if (systemEnabled) {
    digitalWrite(LED_PIN, manualLedState ? HIGH : LOW);
  }
}

// V5 - OLED enable/disable
BLYNK_WRITE(V5) {
  oledEnabled = param.asInt();
  if (!oledEnabled) {
    display.clearDisplay();
    display.display();
  }
}

// V6 - System enable/disable switch
BLYNK_WRITE(V6) {
  systemEnabled = param.asInt();

  if (!systemEnabled) {
    // Stop monitoring: force everything off immediately
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    if (oledEnabled) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(15, 25);
      display.println(F("SYSTEM OFF"));
      display.display();
    }
    Serial.println(F("System disabled via Blynk."));
  } else {
    Serial.println(F("System re-enabled. Resuming monitoring."));
  }
}

// Re-sync widget states whenever Blynk (re)connects
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V4, V5, V6);
}
