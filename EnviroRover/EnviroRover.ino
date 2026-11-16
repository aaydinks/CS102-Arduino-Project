/*
  EnviroRover — CS102 Arduino Project
  ------------------------------------
  A two-wheel-drive rover that:
    1. reads temperature + humidity from a DHT sensor,
    2. shows the readings on an LED board (WS2812 / NeoPixel 8x8 matrix),
    3. drives around on its two rear gear motors (L293D / L298N driver),
    4. avoids obstacles: an ultrasonic sensor on a servo scans left/right
       whenever something is in the way, and the rover turns toward open space.

  Parts
    - Arduino Uno
    - 2 x hobby gear motors (rear wheels; front is a caster / free wheel)
    - L293D motor driver module (an L298N works with the same code & wiring)
    - HC-SR04 ultrasonic sensor mounted on an SG90-style servo (faces forward)
    - DHT22 (or DHT11) temperature/humidity sensor
    - 8x8 NeoPixel (WS2812) LED board

  Behaviour
    - LED colour = temperature (blue = cool ... green = comfy ... red = hot)
    - LED fill   = humidity     (rows light from the bottom, 0-100 %)
    - Driving    = forward until an obstacle is closer than OBSTACLE_CM, then
                   stop, sweep the servo left/right, and turn toward the side
                   with more room (or spin around if both sides are blocked).
    - The rover STOPS if it gets too hot or the DHT sensor fails.
    - Serial monitor (9600 baud) logs readings, distances and rover state.

  Wiring (Arduino Uno)
    DHT22/DHT11  DATA -> D2   (10k pull-up to 5V if your module has none)
    NeoPixel     DIN  -> D6   (330 ohm in series; 1000 uF cap across 5V/GND)
    Servo        SIG  -> D9   (power the servo from the 5V rail / battery, NOT D9)
    HC-SR04      TRIG -> D12, ECHO -> D13
    Driver       ENA(EN1,2) -> D5 (PWM)   IN1 -> D7   IN2 -> D8     left motor
                 ENB(EN3,4) -> D3 (PWM)   IN3 -> D10  IN4 -> D11    right motor
    Motors on the driver's OUT1/OUT2 (left) and OUT3/OUT4 (right).
    Power the driver from a separate battery pack and join its GND to Arduino GND.
    (Bare L293D chip: the enable pins are 1,2EN and 3,4EN; module labels vary.)

  Pin note: on an Uno the Servo library disables analogWrite() on D9 and D10,
  which is why the motor enables use D5 and D3 instead.

  Libraries (Arduino IDE > Library Manager):
    - "DHT sensor library" by Adafruit  (+ "Adafruit Unified Sensor")
    - "Adafruit NeoPixel"
    - Servo (built in)

  NOTE: The CS102-Arduino-Project GitHub repo is empty, so pins and behaviour
  are my own choices. Edit the CONFIG section to match your build.
*/

#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// ───────────────────────────── CONFIG ─────────────────────────────

// Temperature / humidity sensor
#define DHT_PIN        2
#define DHT_TYPE       DHT22      // change to DHT11 if that's what you have

// LED board (8x8 = 64 pixels; change WIDTH/HEIGHT for other sizes)
#define LED_PIN        6
#define MATRIX_WIDTH   8
#define MATRIX_HEIGHT  8
#define NUM_LEDS       (MATRIX_WIDTH * MATRIX_HEIGHT)
#define LED_BRIGHTNESS 40         // 0-255; keep low to limit current draw

// Ultrasonic sensor + servo
const uint8_t TRIG_PIN  = 12;
const uint8_t ECHO_PIN  = 13;
const uint8_t SERVO_PIN = 9;
const uint8_t SERVO_CENTER = 90;  // degrees; trim if the sensor isn't straight
const uint8_t SERVO_LEFT   = 150;
const uint8_t SERVO_RIGHT  = 30;  // swap LEFT/RIGHT if your servo is mirrored

// Motor driver (L293D / L298N)
const uint8_t ENA = 5, IN1 = 7,  IN2 = 8;    // left motor
const uint8_t ENB = 3, IN3 = 10, IN4 = 11;   // right motor
const uint8_t DRIVE_SPEED = 180;             // 0-255 PWM
const uint8_t TURN_SPEED  = 160;
const uint8_t LEFT_TRIM   = 0;               // add PWM to the left/right motor if the
const uint8_t RIGHT_TRIM  = 0;               // rover drifts (applied to DRIVE_SPEED)

// Navigation
const int OBSTACLE_CM          = 20;   // react when closer than this
const int CLEAR_CM             = 999;  // value used when no echo is received
const unsigned long TURN_90_MS = 600;  // time to spin ~90 degrees (calibrate!)
const unsigned long PING_EVERY_MS = 60;

// Safety / sensing
const float TEMP_STOP_C        = 35.0; // stop driving above this
const float TEMP_RESUME_C      = 33.0; // resume below this (hysteresis)
const unsigned long SENSE_EVERY_MS = 2000;  // DHT22 needs >= 2 s between reads
const unsigned long LED_EVERY_MS   = 100;   // LED refresh (keeps servo jitter low)

// Colour scale for temperature display
const float TEMP_MIN_C = 10.0;
const float TEMP_MAX_C = 40.0;

// ───────────────────────────── STATE ──────────────────────────────

DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_NeoPixel matrix(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo scanServo;

float temperatureC = NAN;
float humidity     = NAN;
bool  sensorOk     = false;
bool  tooHot       = false;

enum RoverState { FORWARD, TURNING, HALTED };
RoverState state = FORWARD;
unsigned long stateStart = 0;
unsigned long turnDuration = 0;
bool turningRight = true;
uint8_t closeHits = 0;                 // consecutive "too close" pings

unsigned long lastSense = 0, lastPing = 0, lastLed = 0;
int lastDistance = CLEAR_CM;

// ───────────────────────────── MOTORS ─────────────────────────────

// speed: -255..255 (negative = reverse)
void setMotor(uint8_t en, uint8_t inA, uint8_t inB, int speed) {
  if (speed > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else if (speed < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else                { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }
  analogWrite(en, constrain(abs(speed), 0, 255));
}

void driveForward() {
  setMotor(ENA, IN1, IN2, constrain(DRIVE_SPEED + LEFT_TRIM,  0, 255));
  setMotor(ENB, IN3, IN4, constrain(DRIVE_SPEED + RIGHT_TRIM, 0, 255));
}
void spinRight()  { setMotor(ENA, IN1, IN2,  TURN_SPEED); setMotor(ENB, IN3, IN4, -TURN_SPEED); }
void spinLeft()   { setMotor(ENA, IN1, IN2, -TURN_SPEED); setMotor(ENB, IN3, IN4,  TURN_SPEED); }
void stopMotors() { setMotor(ENA, IN1, IN2, 0);           setMotor(ENB, IN3, IN4, 0);           }

// ───────────────────────────── ULTRASONIC ─────────────────────────

// Distance in cm, or CLEAR_CM if no echo (nothing in range).
int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);   // ~4 m max, 25 ms timeout
  if (us == 0) return CLEAR_CM;
  return (int)(us / 58UL);                               // microseconds -> cm
}

// Median of three pings (filters out the occasional bad echo).
int readDistanceFiltered() {
  int a = readDistanceCm(); delay(15);
  int b = readDistanceCm(); delay(15);
  int c = readDistanceCm();
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// Stop, look left and right, then start a turn toward the more open side.
void scanAndChooseTurn() {
  stopMotors();

  scanServo.write(SERVO_LEFT);
  delay(450);
  int left = readDistanceFiltered();

  scanServo.write(SERVO_RIGHT);
  delay(700);
  int right = readDistanceFiltered();

  scanServo.write(SERVO_CENTER);
  delay(300);

  Serial.print(F("Scan  left: "));  Serial.print(left);
  Serial.print(F(" cm  right: ")); Serial.print(right);
  Serial.println(F(" cm"));

  if (left < OBSTACLE_CM && right < OBSTACLE_CM) {
    turningRight = true;                        // boxed in: turn around
    turnDuration = TURN_90_MS * 2;
  } else {
    turningRight = (right >= left);
    turnDuration = TURN_90_MS;
  }

  state = TURNING;
  stateStart = millis();
  turningRight ? spinRight() : spinLeft();
}

// ───────────────────────────── NAVIGATION ─────────────────────────

void updateRover() {
  unsigned long now = millis();

  // Safety first
  if (!sensorOk || tooHot) {
    if (state != HALTED) { stopMotors(); state = HALTED; }
    return;
  }
  if (state == HALTED) {                // conditions cleared -> resume driving
    state = FORWARD;
    closeHits = 0;
    driveForward();
    return;
  }

  if (state == FORWARD) {
    if (now - lastPing >= PING_EVERY_MS) {
      lastPing = now;
      lastDistance = readDistanceCm();
      // Require two close readings in a row so a stray echo doesn't stop us
      closeHits = (lastDistance < OBSTACLE_CM) ? closeHits + 1 : 0;
      if (closeHits >= 2) {
        closeHits = 0;
        scanAndChooseTurn();            // blocking ~1.5 s while the rover is stopped
      }
    }
  } else if (state == TURNING) {
    if (now - stateStart >= turnDuration) {
      state = FORWARD;
      closeHits = 0;
      driveForward();
    }
  }
}

// ───────────────────────────── LED BOARD ──────────────────────────

// Map temperature to a blue -> green -> red colour.
uint32_t tempToColor(float t) {
  float x = constrain((t - TEMP_MIN_C) / (TEMP_MAX_C - TEMP_MIN_C), 0.0, 1.0);
  uint8_t r, g, b;
  if (x < 0.5) {                       // blue -> green
    float k = x / 0.5;
    r = 0;
    g = (uint8_t)(255 * k);
    b = (uint8_t)(255 * (1.0 - k));
  } else {                             // green -> red
    float k = (x - 0.5) / 0.5;
    r = (uint8_t)(255 * k);
    g = (uint8_t)(255 * (1.0 - k));
    b = 0;
  }
  return matrix.Color(r, g, b);
}

// Pixel index for (col, row); row 0 = bottom. Assumes plain left-to-right,
// row-by-row wiring. For a serpentine (zig-zag) board, flip odd rows.
uint16_t pixelIndex(uint8_t col, uint8_t row) {
  uint8_t physRow = (MATRIX_HEIGHT - 1) - row;
  return physRow * MATRIX_WIDTH + col;
}

void showReadings() {
  matrix.clear();

  if (!sensorOk) {                     // error pattern: blinking red X
    if ((millis() / 500) % 2) {
      for (uint8_t i = 0; i < MATRIX_WIDTH && i < MATRIX_HEIGHT; i++) {
        matrix.setPixelColor(pixelIndex(i, i), matrix.Color(255, 0, 0));
        matrix.setPixelColor(pixelIndex(MATRIX_WIDTH - 1 - i, i), matrix.Color(255, 0, 0));
      }
    }
    matrix.show();
    return;
  }

  uint32_t color = tempToColor(temperatureC);
  float h = constrain(humidity, 0.0, 100.0);
  uint8_t rowsLit = (uint8_t)round(h / 100.0 * MATRIX_HEIGHT);   // 0..HEIGHT

  for (uint8_t row = 0; row < rowsLit; row++)
    for (uint8_t col = 0; col < MATRIX_WIDTH; col++)
      matrix.setPixelColor(pixelIndex(col, row), color);

  // Too hot: flash the top row white as a warning
  if (tooHot && (millis() / 250) % 2) {
    for (uint8_t col = 0; col < MATRIX_WIDTH; col++)
      matrix.setPixelColor(pixelIndex(col, MATRIX_HEIGHT - 1), matrix.Color(255, 255, 255));
  }
  matrix.show();
}

// ───────────────────────────── TEMP / HUMIDITY ────────────────────

void readSensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();           // Celsius

  if (isnan(h) || isnan(t)) {
    sensorOk = false;
    Serial.println(F("DHT read failed - check wiring"));
    return;
  }

  sensorOk     = true;
  humidity     = h;
  temperatureC = t;

  // Hysteresis so the rover doesn't stutter around the threshold
  if (!tooHot && t >= TEMP_STOP_C)       tooHot = true;
  else if (tooHot && t <= TEMP_RESUME_C) tooHot = false;

  Serial.print(F("Temp: "));        Serial.print(t, 1);
  Serial.print(F(" C  Humidity: ")); Serial.print(h, 1);
  Serial.print(F(" %  Dist: "));     Serial.print(lastDistance);
  Serial.print(F(" cm  State: "));
  switch (state) {
    case FORWARD: Serial.println(F("forward")); break;
    case TURNING: Serial.println(turningRight ? F("turning right") : F("turning left")); break;
    case HALTED:  Serial.println(F("HALTED")); break;
  }
}

// ───────────────────────────── ARDUINO ────────────────────────────

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  stopMotors();

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);

  dht.begin();
  matrix.begin();
  matrix.setBrightness(LED_BRIGHTNESS);
  matrix.clear();
  matrix.show();

  // Start-up animation: sweep a green row up the board
  for (uint8_t row = 0; row < MATRIX_HEIGHT; row++) {
    matrix.clear();
    for (uint8_t col = 0; col < MATRIX_WIDTH; col++)
      matrix.setPixelColor(pixelIndex(col, row), matrix.Color(0, 255, 0));
    matrix.show();
    delay(80);
  }

  Serial.println(F("EnviroRover ready"));
  delay(2000);                         // let the DHT settle before first read
  readSensor();
  lastSense = millis();

  if (sensorOk && !tooHot) { state = FORWARD; driveForward(); }
  else                     { state = HALTED;  stopMotors();   }
}

void loop() {
  unsigned long now = millis();

  if (now - lastSense >= SENSE_EVERY_MS) {
    lastSense = now;
    readSensor();
  }

  updateRover();

  if (now - lastLed >= LED_EVERY_MS) {
    lastLed = now;
    showReadings();                    // NeoPixel updates briefly disable interrupts
  }
}
