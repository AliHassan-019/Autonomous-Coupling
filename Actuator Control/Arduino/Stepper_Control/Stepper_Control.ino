// define Pin connections
const int stepPin = 2;
const int dirPin = 5;
 
#define stepsPerRevolution 34000   /// change la langueur de deplacement du chariot
#define step10cm 4250   /// nombres de pas pour faire 10cm du distance 
 
void setup() {
 
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
 
}
void loop() {
	
	//
	delay(1000);          // une secande = temps milliseconde 
	//un movamant pour un 10 cm 
	// Set the spinning direction clockwise:
	digitalWrite(dirPin, HIGH);
	// Spin the stepper motor 5 revolutions fast:
	for (int i = 0; i < step10cm; i++) {
		// These four lines result in 1 step:
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(150);   // change la vitesse du moteur
		digitalWrite(stepPin, LOW);
		delayMicroseconds(150);     // Change la vitesse moteur
	}
	delay(10000); // temps de pause pour 10 secandes 
	
	//un movamant pour un 20 cm 
	// Set the spinning direction clockwise:
	digitalWrite(dirPin, HIGH);
	// Spin the stepper motor 5 revolutions fast:
	for (int i = 0; i < (step10cm*2); i++) {
		// These four lines result in 1 step:
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(150);   // change la vitesse du moteur
		digitalWrite(stepPin, LOW);
		delayMicroseconds(150);     // Change la vitesse moteur
	}
	delay(10000); // temps de pause pour 10 secandes 

	//un movamant pour un 10 cm 
	// Set the spinning direction clockwise:
	digitalWrite(dirPin, HIGH);
	// Spin the stepper motor 5 revolutions fast:
	for (int i = 0; i < step10cm; i++) {
		// These four lines result in 1 step:
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(150);   // change la vitesse du moteur
		digitalWrite(stepPin, LOW);
		delayMicroseconds(150);     // Change la vitesse moteur
	}
	
	delay(10000); // temps de pause pour 10 secandes 

  //un movamant pour un 40 cm 
	// Set the spinning direction clockwise:
	digitalWrite(dirPin, HIGH);
	// Spin the stepper motor 5 revolutions fast:
	for (int i = 0; i < (step10cm*4); i++) {
		// These four lines result in 1 step:
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(150);   // change la vitesse du moteur
		digitalWrite(stepPin, LOW);
		delayMicroseconds(150);     // Change la vitesse moteur
	}
	delay(10000); // temps de pause pour 10 secandes 
		
	// Set the spinning direction :
	digitalWrite(dirPin, LOW);
	//Spin the stepper motor 5 revolutions fast:
	for (int i = 0; i < stepsPerRevolution; i++) {
		// These four lines result in 1 step:
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(150);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(150);
	}
	delay(1000);
}