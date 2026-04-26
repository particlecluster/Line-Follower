// ============================================================
//  6-Sensor Line Follower + HC-SR04 + TCS34725 + BUZZER
//  RED   = stop 10 seconds
//  BLUE  = slow speed
//  GREEN = stop + play buzzer melody (special feature)
//  Colour thresholds calibrated to actual sensor readings
// ============================================================

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// --- Motor Pins ---
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6
#define ENB 9

// --- IR Sensor Pins ---
const int sensorPins[6] = {7, 8, 10, 11, 12, 13};
const int weights[6]    = {5, 3, 1, -1, -3, -5};

// --- Ultrasonic Pins ---
#define TRIG_PIN A0
#define ECHO_PIN A1

// --- Buzzer Pin ---
#define BUZZER_PIN A2

// --- Obstacle Settings ---
#define STOP_DISTANCE   3
#define ROTATE_DURATION 500

// --- PID Constants ---
float Kp = 80.0;   // was 50.0
float Ki = 0.0;
float Kd = 45.0;   // was 30.0

int baseSpeed    = 70;
int currentSpeed = 70;
int maxSpeed     = 255;

float error = 0, lastError = 0, integral = 0;

// --- Colour Sensor ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

unsigned long lastColourRead = 0;
#define COLOUR_INTERVAL 300

String lastColour = "UNKNOWN";

// --- Note frequencies ---
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_G3  196
#define NOTE_A4  440

// ============================================================
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ============================================================
String getColourName(float r, float g, float b) {
  float total = r + g + b;
  if (total < 350) return "UNKNOWN";  // sensor in air or too dark

  float rn = r / total;
  float gn = g / total;
  float bn = b / total;

  // WHITE background — ignore it (very high total, rn ~0.47-0.54)
  if (total > 3000 && rn > 0.46 && rn < 0.56) return "UNKNOWN";

  // RED — rn > 0.58, gn < 0.22, bn < 0.20
  if (rn > 0.58 && gn < 0.22 && bn < 0.20) return "RED";

  // GREEN — gn dominant, rn clearly lower
  if (gn > 0.40 && gn > rn && gn > bn && rn < 0.36) return "GREEN";

  // BLUE — bn ≈ gn, both clearly above rn
  if (bn > 0.35 && gn > 0.33 && rn < 0.32 && bn > rn && gn > rn) return "BLUE";

  return "UNKNOWN";
}

// ============================================================
//  BUZZER FUNCTIONS
// ============================================================
void playTone(int freq, int duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration + 30);
  noTone(BUZZER_PIN);
}

void beepRed() {
  playTone(NOTE_G3, 200);
  delay(100);
  playTone(NOTE_G3, 200);
  delay(100);
  playTone(NOTE_G3, 200);
}

void playGreenMelody() {
  playTone(NOTE_C4, 200);
  playTone(NOTE_E4, 200);
  playTone(NOTE_G4, 200);
  playTone(NOTE_C5, 400);
  delay(150);
  playTone(NOTE_G4, 150);
  playTone(NOTE_C5, 500);
}

void beepBlue() {
  playTone(NOTE_A4, 100);
  delay(50);
  playTone(NOTE_C5, 100);
}

// ============================================================
//  COLOUR ACTIONS
// ============================================================
void handleColour(String colour) {
  if (colour == lastColour) return;
  lastColour = colour;

  Serial.print("ACTION for colour: ");
  Serial.println(colour);

  if (colour == "RED") {
    setMotors(0, 0);
    beepRed();
    Serial.println("RED: Stopping for 10 seconds...");
    delay(10000);
    currentSpeed = baseSpeed;
    Serial.println("RED: Resuming.");

  } else if (colour == "BLUE") {
    currentSpeed = 30;
    beepBlue();
    Serial.println("BLUE: Slow speed mode ON.");

  } else if (colour == "GREEN") {
    setMotors(0, 0);
    Serial.println("GREEN: Playing special melody!");
    playGreenMelody();
    currentSpeed = baseSpeed;
    Serial.println("GREEN: Done. Resuming.");

  } else {
    if (currentSpeed != baseSpeed) {
      currentSpeed = baseSpeed;
      Serial.println("Normal speed resumed.");
    }
  }
}

// ============================================================
void readAndPrintColour() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  String colourName = getColourName((float)r, (float)g, (float)b);

  Serial.println("---------- COLOUR ----------");
  Serial.print("R: "); Serial.print(r);
  Serial.print("  G: "); Serial.print(g);
  Serial.print("  B: "); Serial.print(b);
  Serial.print("  Clear: "); Serial.println(c);
  Serial.print("Detected: "); Serial.println(colourName);
  Serial.println("----------------------------");

  handleColour(colourName);
}

// ============================================================
void setup() {
  for (int i = 0; i < 6; i++)
    pinMode(sensorPins[i], INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("TCS34725 found!");
  } else {
    Serial.println("TCS34725 NOT found — check wiring!");
    while (1);
  }
}

// ============================================================
void loop() {

  // --- Read colour every 300ms ---
  if (millis() - lastColourRead >= COLOUR_INTERVAL) {
    readAndPrintColour();
    lastColourRead = millis();
  }

  // --- Check obstacle ---
  long dist = getDistance();
  if (dist < STOP_DISTANCE) {
    setMotors(0, 0);
    delay(300);
    spinRight();
    delay(ROTATE_DURATION);
    return;
  }

  // --- Line following ---
  int sensorValues[6];
  int weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 6; i++) {
    sensorValues[i] = (digitalRead(sensorPins[i]) == HIGH) ? 1 : 0;
    weightedSum += sensorValues[i] * weights[i];
    activeCount += sensorValues[i];
  }

  if (activeCount == 0) {
    if (lastError > 0) spinRight();
    else               spinLeft();
    return;
  }

  if (activeCount == 6) {
    moveForward(currentSpeed, currentSpeed);
    return;
  }

  error = (float)weightedSum / activeCount;
  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed  = constrain(currentSpeed + (int)correction, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(currentSpeed - (int)correction, -maxSpeed, maxSpeed);

  setMotors(leftSpeed, rightSpeed);
}

// ============================================================
void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENA, leftSpeed);

  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENB, rightSpeed);
}

void moveForward(int l, int r) { setMotors(l, r); }
void spinLeft()  { setMotors(-currentSpeed,  currentSpeed); }
void spinRight() { setMotors( currentSpeed, -currentSpeed); }
