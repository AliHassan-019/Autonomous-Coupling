// define Pin connections
const int stepPin = 2;
const int dirPin = 5;

#define stepsPerRevolution 34000   /// change la langueur de deplacement du chariot
#define step10cm 4250   /// nombres de pas pour faire 10cm du distance 

// Global variables for motor control
int motorSpeed = 150;  // delay in microseconds between steps (smaller = faster)
boolean isMoving = false;
volatile long stepsToMove = 0;
volatile int motorDirection = 0;  // 0 = stop, 1 = forward, -1 = backward

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Initialize pins to LOW
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  
  Serial.println("Stepper Motor Control Ready!");
  Serial.println("Commands: MOVE <steps> <speed> <direction>");
  Serial.println("  direction: 1=forward, 0=backward");
  Serial.println("  speed: microseconds delay (50-500)");
}

// Function to move stepper motor for specific number of steps
void moveMotor(long steps, int speed, int direction) {
  if (steps <= 0 || speed < 50 || speed > 500) {
    Serial.println("ERROR: Invalid parameters");
    return;
  }
  
  isMoving = true;
  
  // Set direction
  if (direction == 1) {
    digitalWrite(dirPin, HIGH);  // Forward
  } else {
    digitalWrite(dirPin, LOW);   // Backward
  }
  
  // Generate step pulses
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
    
    // Allow interrupt/serial check every 100 steps
    if (i % 100 == 0) {
      if (Serial.available()) {
        break;  // Stop if new command received
      }
    }
  }
  
  isMoving = false;
  digitalWrite(stepPin, LOW);
  Serial.print("Movement complete: ");
  Serial.print(steps);
  Serial.println(" steps");
}

// Function to move by distance in cm
void moveDistance(float distanceCm, int speed, int direction) {
  long steps = (long)(distanceCm * step10cm / 10.0);
  moveMotor(steps, speed, direction);
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("MOVE")) {
      // Parse: MOVE <steps> <speed> <direction>
      int space1 = command.indexOf(' ');
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);
      
      long steps = command.substring(space1 + 1, space2).toInt();
      int speed = command.substring(space2 + 1, space3).toInt();
      int direction = command.substring(space3 + 1).toInt();
      
      moveMotor(steps, speed, direction);
    }
    else if (command.startsWith("DIST")) {
      // Parse: DIST <distance_cm> <speed> <direction>
      int space1 = command.indexOf(' ');
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);
      
      float distance = command.substring(space1 + 1, space2).toFloat();
      int speed = command.substring(space2 + 1, space3).toInt();
      int direction = command.substring(space3 + 1).toInt();
      
      moveDistance(distance, speed, direction);
    }
    else if (command == "STOP") {
      isMoving = false;
      Serial.println("Motor stopped");
    }
    else if (command == "STATUS") {
      Serial.print("Motor moving: ");
      Serial.println(isMoving ? "Yes" : "No");
    }
    else {
      Serial.println("Unknown command. Use: MOVE, DIST, STOP, STATUS");
    }
  }
}