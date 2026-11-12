
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// Sensor pins
const int gasPin = A0;
const int ldrPin = 9;          // LDR module digital output (D0)

// LED pins
const int greenLED = 5;
const int amberLED = 6;
const int redLED = 7;

// Alarm and button
const int buzzer = 8;
const int emergencyButton = 2;

// Gas threshold (placeholder; calibrate for your sensor)
const int GAS_THRESHOLD = 60;

// Fall detection thresholds (prototype values)
const float LOW_G_THRESHOLD = 4.0;
const float IMPACT_THRESHOLD = 22.0;

const unsigned long FALL_WINDOW_MS = 1500;
const unsigned long FALL_ALARM_MS = 10000;

Adafruit_ADXL345_Unified accel =
    Adafruit_ADXL345_Unified(12345);

bool lowGDetected = false;
unsigned long lowGTime = 0;
unsigned long fallAlarmUntil = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(ldrPin, INPUT);

  pinMode(greenLED, OUTPUT);
  pinMode(amberLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(emergencyButton, INPUT_PULLUP);

  digitalWrite(greenLED, LOW);
  digitalWrite(amberLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(buzzer, LOW);

  if (!accel.begin()) {
    Serial.println("ERROR: ADXL345 not detected!");

    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);

    // Sensor fault alarm
    while (true) {
      digitalWrite(buzzer, HIGH);
      delay(200);
      digitalWrite(buzzer, LOW);
      delay(200);
    }
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);

  Serial.println("Miner safety harness ready.");
  Serial.println("Check LDR polarity and calibrate gas threshold.");
}

void loop() {
  unsigned long now = millis();

  // Read sensors
  int gasValue = analogRead(gasPin);
  int lightValue = digitalRead(ldrPin);

  sensors_event_t event;
  accel.getEvent(&event);

  // Total acceleration magnitude, in m/s^2
  float magnitude = sqrt(
      event.acceleration.x * event.acceleration.x +
      event.acceleration.y * event.acceleration.y +
      event.acceleration.z * event.acceleration.z
  );

  // Hazard conditions
  bool gasWarning = (gasValue >= GAS_THRESHOLD);

  bool lowLight = (lightValue == HIGH);

  bool manualEmergency =
      (digitalRead(emergencyButton) == LOW);

  // Simple possible-fall detection
  if (magnitude < LOW_G_THRESHOLD && !lowGDetected) {
    lowGDetected = true;
    lowGTime = now;
  }

  if (lowGDetected &&
      (now - lowGTime <= FALL_WINDOW_MS) &&
      magnitude > IMPACT_THRESHOLD) {

    fallAlarmUntil = now + FALL_ALARM_MS;
    lowGDetected = false;

    Serial.println("Possible fall detected!");
  }

  if (lowGDetected &&
      (now - lowGTime > FALL_WINDOW_MS)) {
    lowGDetected = false;
  }

  bool fallAlarm =
      ((long)(fallAlarmUntil - now) > 0);

  // Red LED and buzzer indicate urgent alarms
  bool redWarning =
      gasWarning || fallAlarm || manualEmergency;

  bool alarm =
      gasWarning || fallAlarm || manualEmergency;

  // Green means all monitored conditions are normal
  bool normalStatus =
      !gasWarning && !lowLight &&
      !fallAlarm && !manualEmergency;

  digitalWrite(greenLED, normalStatus ? HIGH : LOW);
  digitalWrite(amberLED, lowLight ? HIGH : LOW);
  digitalWrite(redLED, redWarning ? HIGH : LOW);

  if (alarm) {
    tone(buzzer, 2000);  // 2 kHz alarm tone
  } else {
    noTone(buzzer);
  }

  // Serial diagnostics
  Serial.print("Gas: ");
  Serial.print(gasValue);

  Serial.print(" | LDR D0: ");
  Serial.print(lightValue);

  Serial.print(" | Acceleration: ");
  Serial.print(magnitude);

  Serial.print(" | Fall alarm: ");
  Serial.print(fallAlarm);

  Serial.print(" | Button: ");
  Serial.println(manualEmergency);

  delay(20);
}
