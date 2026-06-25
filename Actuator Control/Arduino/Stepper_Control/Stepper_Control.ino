#include <AccelStepper.h>

// CNC Shield V3, X axis pins
const int stepPin = 2;
const int dirPin = 5;
const int enablePin = 8;

const int microstepFactor = 16;
const float actuatorLengthCm = 81.0;

// Calibrated from measurement:
// GOTO 5 moved 8 cm, and GOTO 10 moved 16 cm.
// 680 * commanded / actual = 680 * 10 / 16 = 425 steps/cm.
const float stepsPerCm = 425.0;

const float defaultFullStepSpeed = 100.0;
const float minFullStepSpeed = 20.0;
const float maxFullStepSpeed = 180.0;
float currentFullStepSpeed = defaultFullStepSpeed;
float currentSpeedStepsPerSecond = defaultFullStepSpeed * microstepFactor;

const unsigned int minPulseWidthUs = 10;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

String inputString = "";

long cmToSteps(float cm) {
  cm = constrain(cm, 0.0, actuatorLengthCm);
  return lround(cm * stepsPerCm);
}

float stepsToCm(long steps) {
  return (float)steps / stepsPerCm;
}

void printStatus() {
  Serial.print("POS_CM=");
  Serial.print(stepsToCm(stepper.currentPosition()), 3);
  Serial.print(" TARGET_CM=");
  Serial.print(stepsToCm(stepper.targetPosition()), 3);
  Serial.print(" REMAINING_CM=");
  Serial.print(stepsToCm(stepper.distanceToGo()), 3);
  Serial.print(" SPEED=");
  Serial.println(currentFullStepSpeed, 1);
}

void updateMoveSpeed() {
  long remaining = stepper.distanceToGo();

  if (remaining > 0) {
    stepper.setSpeed(currentSpeedStepsPerSecond);
  } else if (remaining < 0) {
    stepper.setSpeed(-currentSpeedStepsPerSecond);
  } else {
    stepper.setSpeed(0);
  }
}

void setFullStepSpeed(float requestedSpeed) {
  currentFullStepSpeed = constrain(requestedSpeed, minFullStepSpeed, maxFullStepSpeed);
  currentSpeedStepsPerSecond = currentFullStepSpeed * microstepFactor;
  stepper.setMaxSpeed(currentSpeedStepsPerSecond);
  updateMoveSpeed();

  Serial.print("SPEED=");
  Serial.println(currentFullStepSpeed, 1);
}

void goToDistance(float targetCm) {
  targetCm = constrain(targetCm, 0.0, actuatorLengthCm);
  stepper.moveTo(cmToSteps(targetCm));
  updateMoveSpeed();

  Serial.print("GOTO_CM=");
  Serial.println(targetCm, 3);
}

void moveRelative(float distanceCm) {
  float currentCm = stepsToCm(stepper.currentPosition());
  goToDistance(currentCm + distanceCm);
}

void stopMotor() {
  stepper.moveTo(stepper.currentPosition());
  stepper.setSpeed(0);
  Serial.println("STOPPING");
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command == "HOME") {
    stepper.setCurrentPosition(0);
    stepper.moveTo(0);
    stepper.setSpeed(0);
    Serial.println("HOME_SET_TO_0_CM");
    return;
  }

  if (command.startsWith("GOTO ")) {
    goToDistance(command.substring(5).toFloat());
    return;
  }

  if (command.startsWith("MOVE ")) {
    moveRelative(command.substring(5).toFloat());
    return;
  }

  if (command.startsWith("SPEED ")) {
    setFullStepSpeed(command.substring(6).toFloat());
    return;
  }

  if (command == "STOP") {
    stopMotor();
    return;
  }

  if (command == "STATUS") {
    printStatus();
    return;
  }

  Serial.print("UNKNOWN_COMMAND=");
  Serial.println(command);
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputString);
      inputString = "";
    } else if (c != '\r') {
      inputString += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(80);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);  // CNC Shield enable is active LOW

  stepper.setMinPulseWidth(minPulseWidthUs);
  stepper.setMaxSpeed(currentSpeedStepsPerSecond);
  stepper.setSpeed(0);
  stepper.setCurrentPosition(0);

  Serial.println("Distance control ready");
  Serial.println("Commands: HOME, GOTO 30, MOVE 5, MOVE -5, SPEED 100, STOP, STATUS");
}

void loop() {
  readSerialCommands();
  stepper.runSpeedToPosition();
}
