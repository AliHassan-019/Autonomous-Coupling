// Arduino UNO R4 + CNC Shield V3 + A4988
// X-axis motor control through Serial Monitor

const int stepPin   = 2;   // X STEP
const int dirPin    = 5;   // X DIR
const int enablePin = 8;   // Enable for all A4988 drivers

#define step10cm 4250   // Steps required for 10 cm travel

bool isMoving = false;

void setup() {
  Serial.begin(9600);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  // A4988 enable is active LOW
  digitalWrite(enablePin, LOW);

  Serial.println("Stepper Motor Control Ready!");
  Serial.println("Commands:");
  Serial.println("MOVE <steps> <speed_us> <direction>");
  Serial.println("DIST <distance_cm> <speed_us> <direction>");
  Serial.println("STOP");
  Serial.println("STATUS");
  Serial.println("");
  Serial.println("Example:");
  Serial.println("MOVE 200 1000 1");
  Serial.println("DIST 10 1000 1");
}

// direction: 1 = forward, 0 = backward
void moveMotor(long steps, int speedUs, int direction) {
  if (steps <= 0) {
    Serial.println("ERROR: Steps must be greater than zero");
    return;
  }

  if (speedUs < 100 || speedUs > 5000) {
    Serial.println("ERROR: Speed must be between 100 and 5000 microseconds");
    return;
  }

  isMoving = true;

  if (direction == 1) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  delayMicroseconds(10);

  Serial.print("Moving ");
  Serial.print(steps);
  Serial.print(" steps, speed=");
  Serial.print(speedUs);
  Serial.print(" us, direction=");
  Serial.println(direction);

  for (long i = 0; i < steps; i++) {
    // Check for STOP command periodically
    if (i % 20 == 0 && Serial.available() > 0) {
      String incoming = Serial.readStringUntil('\n');
      incoming.trim();

      if (incoming == "STOP") {
        Serial.println("Movement stopped");
        break;
      }
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speedUs);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(speedUs);
  }

  digitalWrite(stepPin, LOW);
  isMoving = false;

  Serial.println("Movement complete");
}

void moveDistance(float distanceCm, int speedUs, int direction) {
  if (distanceCm <= 0) {
    Serial.println("ERROR: Distance must be greater than zero");
    return;
  }

  long steps = (long)(distanceCm * step10cm / 10.0);

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.print(" cm = ");
  Serial.print(steps);
  Serial.println(" steps");

  moveMotor(steps, speedUs, direction);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) {
      return;
    }

    if (command.startsWith("MOVE ")) {
      // Format: MOVE <steps> <speed_us> <direction>
      int space1 = command.indexOf(' ');
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);

      if (space1 == -1 || space2 == -1 || space3 == -1) {
        Serial.println("ERROR: Use MOVE <steps> <speed_us> <direction>");
        return;
      }

      long steps = command.substring(space1 + 1, space2).toInt();
      int speedUs = command.substring(space2 + 1, space3).toInt();
      int direction = command.substring(space3 + 1).toInt();

      if (direction != 0 && direction != 1) {
        Serial.println("ERROR: Direction must be 0 or 1");
        return;
      }

      moveMotor(steps, speedUs, direction);
    }

    else if (command.startsWith("DIST ")) {
      // Format: DIST <distance_cm> <speed_us> <direction>
      int space1 = command.indexOf(' ');
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);

      if (space1 == -1 || space2 == -1 || space3 == -1) {
        Serial.println("ERROR: Use DIST <distance_cm> <speed_us> <direction>");
        return;
      }

      float distanceCm = command.substring(space1 + 1, space2).toFloat();
      int speedUs = command.substring(space2 + 1, space3).toInt();
      int direction = command.substring(space3 + 1).toInt();

      if (direction != 0 && direction != 1) {
        Serial.println("ERROR: Direction must be 0 or 1");
        return;
      }

      moveDistance(distanceCm, speedUs, direction);
    }

    else if (command == "STOP") {
      isMoving = false;
      digitalWrite(stepPin, LOW);
      Serial.println("No active movement to stop");
    }

    else if (command == "STATUS") {
      Serial.print("Motor enabled: ");
      Serial.println(digitalRead(enablePin) == LOW ? "Yes" : "No");

      Serial.print("Motor moving: ");
      Serial.println(isMoving ? "Yes" : "No");
    }

    else if (command == "ENABLE") {
      digitalWrite(enablePin, LOW);
      Serial.println("A4988 enabled");
    }

    else if (command == "DISABLE") {
      digitalWrite(enablePin, HIGH);
      Serial.println("A4988 disabled");
    }

    else {
      Serial.println("Unknown command");
      Serial.println("Use MOVE, DIST, STOP, STATUS, ENABLE, or DISABLE");
    }
  }
}