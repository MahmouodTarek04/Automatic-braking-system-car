#include "BluetoothSerial.h"
#include <Arduino.h>

BluetoothSerial serialBT;

// Bluetooth signal store in this variable
char btSignal;

// Initial speed
int Speed = 100;

// Declare channels for PWM output
#define R 0
#define L 1

// PWM pins for controlling the speed
int enA = 5; 
int enB = 23;

// Motor controlling pins
int IN1 = 22;
int IN2 = 21;
int IN3 = 19;
int IN4 = 18;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 13;

// Buzzer pin
const int buzzerPin = 2;

// Distance threshold for braking
const int distanceThreshold = 60; 

// Add pin declarations for front and back lights
int frontLight1 = 17; // Pin for the first front light
int frontLight2 = 16; // Pin for the second front light
int backLight1 = 26;  // Pin for the first back light
int backLight2 = 25;  // Pin for the second back light

// Flags for blinking behavior
bool waitingLightsOn = false;
bool blinkingBackwards = false;

// Function prototypes
long getDistance();
void forward();
void backward();
void left();
void right();
void stop();
void ForwardLeft();
void ForwardRight();
void BackLeft();
void BackRight();
void hornOn();
void hornOff();
void handleBackwardBlinking(bool enable);

void setup() {
  Serial.begin(115200);

  // Bluetooth name
  serialBT.begin("Mahmoud Tarek");

  // Output pin declarations
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Setup PWM channels
  ledcSetup(R, 5000, 8);
  ledcAttachPin(enA, R);
  ledcSetup(L, 5000, 8);
  ledcAttachPin(enB, L);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic sensor pin declarations
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Buzzer pin declaration
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Setup pins for front and back lights
  pinMode(frontLight1, OUTPUT);
  pinMode(frontLight2, OUTPUT);
  pinMode(backLight1, OUTPUT);
  pinMode(backLight2, OUTPUT);

  // Turn off all lights initially
  digitalWrite(frontLight1, LOW);
  digitalWrite(frontLight2, LOW);
  digitalWrite(backLight1, LOW);
  digitalWrite(backLight2, LOW);
}

void loop() {
  // Check the distance using the ultrasonic sensor
  long distance = getDistance();

  // Handle Bluetooth commands
  while (serialBT.available()) {
    btSignal = serialBT.read();

    // Adjust speed based on signal
    if (btSignal >= '0' && btSignal <= '9') {
      Speed = (btSignal - '0') * 15 + 100;
    } else if (btSignal == 'q') {
      Speed = 255;
    }

    // Display incoming signal in the serial monitor
    Serial.println(btSignal);

    // Control car movement based on signal
    if (distance < distanceThreshold && btSignal == 'F') {
      Serial.println("Obstacle detected. Stopping forward motion.");
      stop();
    } else {
      switch (btSignal) {
        case 'F':
          handleBackwardBlinking(false);
          forward();
          break;
        case 'B':
          handleBackwardBlinking(true);
          backward();
          break;
        case 'L':
          handleBackwardBlinking(false);
          left();
          break;
        case 'R':
          handleBackwardBlinking(false);
          right();
          break;
        case 'G':
          handleBackwardBlinking(false);
          ForwardLeft();
          break;
        case 'I':
          handleBackwardBlinking(false);
          ForwardRight();
          break;
        case 'H':
          handleBackwardBlinking(true);
          BackLeft();
          break;
        case 'J':
          handleBackwardBlinking(true);
          BackRight();
          break;
        case 'S':
          handleBackwardBlinking(false);
          stop();
          break;
        case 'V':
          hornOn();
          break;
        case 'v':
          hornOff();
          break;
        case 'W':
          digitalWrite(frontLight1, HIGH);
          digitalWrite(frontLight2, HIGH);
          break;
        case 'w':
          digitalWrite(frontLight1, LOW);
          digitalWrite(frontLight2, LOW);
          break;
        case 'U':
          digitalWrite(backLight1, HIGH);
          digitalWrite(backLight2, HIGH);
          break;
        case 'u':
          digitalWrite(backLight1, LOW);
          digitalWrite(backLight2, LOW);
          break;
        case 'X':
          waitingLightsOn = true;
          break;
        case 'x':
          waitingLightsOn = false;
          digitalWrite(frontLight1, LOW);
          digitalWrite(frontLight2, LOW);
          digitalWrite(backLight1, LOW);
          digitalWrite(backLight2, LOW);
          break;
        default:
          break;
      }
    }
  }

  // Handle waiting lights blinking
  if (waitingLightsOn) {
    static unsigned long previousMillis = 0;
    const unsigned long interval = 500;

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Toggle all lights
      bool lightState = digitalRead(frontLight1);
      digitalWrite(frontLight1, !lightState);
      digitalWrite(frontLight2, !lightState);
      digitalWrite(backLight1, !lightState);
      digitalWrite(backLight2, !lightState);
    }
  }

  // Handle backward blinking
  if (blinkingBackwards) {
    static unsigned long lastBlinkTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastBlinkTime >= 500) {
      lastBlinkTime = currentMillis;

      // Toggle buzzer and backlights
      bool buzzerState = digitalRead(buzzerPin);
      digitalWrite(buzzerPin, !buzzerState);
      digitalWrite(backLight1, !buzzerState);
      digitalWrite(backLight2, !buzzerState);
    }
  }
}

void handleBackwardBlinking(bool enable) {
  if (enable && !blinkingBackwards) {
    blinkingBackwards = true;
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(backLight1, HIGH);
    digitalWrite(backLight2, HIGH);
  } else if (!enable && blinkingBackwards) {
    blinkingBackwards = false;
    digitalWrite(buzzerPin, LOW);
    digitalWrite(backLight1, LOW);
    digitalWrite(backLight2, LOW);
  }
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void forward() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void backward() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void left() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  ledcWrite(R, 0);
  ledcWrite(L, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void hornOn() {
  digitalWrite(buzzerPin, HIGH);
}

void hornOff() {
  digitalWrite(buzzerPin, LOW);
}

void ForwardLeft() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed / 2);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void ForwardRight() {
  ledcWrite(R, Speed / 2);
  ledcWrite(L, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void BackLeft() {
  ledcWrite(R, Speed);
  ledcWrite(L, Speed / 2);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void BackRight() {
  ledcWrite(R, Speed / 2);
  ledcWrite(L, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

