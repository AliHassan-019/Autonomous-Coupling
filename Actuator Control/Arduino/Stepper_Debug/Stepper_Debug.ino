// Minimal stepper motion sketch
// Simple one-time move with smooth, slow stepping.

const int stepPin = 2;
const int dirPin = 5;
const int enPin = 8;

const unsigned long stepDelay = 5000; // microseconds for slow, quiet stepping
const unsigned int stepsPerCm = 42.5; // calibration: 425 steps per 1 cm
const unsigned int travelCm = 5; // change this for the distance you want
const unsigned long stepsToMove = stepsPerCm * travelCm;
static bool hasMoved = false;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(enPin, HIGH); // disable motor until movement begins
}

void loop() {
  if (hasMoved) {
    return;
  }

  digitalWrite(dirPin, HIGH);
  digitalWrite(enPin, LOW);
  delay(50);

  for (unsigned long i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  digitalWrite(enPin, HIGH);
  hasMoved = true;
}
