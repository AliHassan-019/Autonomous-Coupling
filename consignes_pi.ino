#include <PID_v1.h>

// === CONSTANTES PHYSIQUES ===
const float ticksParTourSortie = 960.0;
const float diametrePoulieCm = 2.0;
const float circonferencePoulie = 3.1416 * diametrePoulieCm;
const float toleranceCm = 0.5;

// === DÉCLARATION DES BROCHES ===
// Format : IN1 = PWM, IN2 = DIR
const int IN1[6]      = {7, 12, 5, 6, 10, 11};     // PWM
const int IN2[6]      = {26, 27, 32, 34, 23, 25}; // DIR
const int encoderA[6] = {2, 3, 20, 18, 19, 21};
const int encoderB[6] = {28, 33, 30, 48, 35, 39};     

// === VARIABLES DES MOTEURS ===
volatile long encoderTicks[6] = {0};
double positionCm[6] = {0};
double positionCibleCm[6] = {0};
double pidOutput[6] = {0};

// PID
double Kp = 2 , Ki = 0.001, Kd = 0.3;
PID* pid[6];

// === LECTURE DES CONSIGNES ===
String inputString = "";
bool nouvelleConsigne = false;

// === INTERRUPTIONS ENCODEURS ===
void lireEncodeur0() { encoderTicks[0] += (digitalRead(encoderA[0]) == digitalRead(encoderB[0])) ? 1 : -1; }
void lireEncodeur1() { encoderTicks[1] += (digitalRead(encoderA[1]) == digitalRead(encoderB[1])) ? 1 : -1; }
void lireEncodeur2() { encoderTicks[2] += (digitalRead(encoderA[2]) == digitalRead(encoderB[2])) ? 1 : -1; }
void lireEncodeur3() { encoderTicks[3] += (digitalRead(encoderA[3]) == digitalRead(encoderB[3])) ? 1 : -1; }
void lireEncodeur4() { encoderTicks[4] += (digitalRead(encoderA[4]) == digitalRead(encoderB[4])) ? 1 : -1; }
void lireEncodeur5() { encoderTicks[5] += (digitalRead(encoderA[5]) == digitalRead(encoderB[5])) ? 1 : -1; }

void (*interruptFuncs[6])() = {
  lireEncodeur0, lireEncodeur1, lireEncodeur2,
  lireEncodeur3, lireEncodeur4, lireEncodeur5
};

void setup() {
  Serial3.begin(9600);
  Serial.begin(9600);
  Serial3.println("Entrez 6 positions en cm séparées par des virgules :");
  Serial3.println("Entrez 6 positions en cm séparées par des virgules :");
  for (int i = 0; i < 6; i++) {
    pinMode(IN1[i], OUTPUT);  // PWM
    pinMode(IN2[i], OUTPUT);  // DIR
    pinMode(encoderA[i], INPUT_PULLUP);
    pinMode(encoderB[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA[i]), interruptFuncs[i], CHANGE);

    pid[i] = new PID(&positionCm[i], &pidOutput[i], &positionCibleCm[i], Kp, Ki, Kd, DIRECT);
    pid[i]->SetMode(AUTOMATIC);
    pid[i]->SetOutputLimits(-54, 54); // PWM
  }
}

void loop() {
  lireConsignesSerie();

  float erreurs[6];
  bool moteurActif[6];

  //  Calcul des positions et erreur
  for (int i = 0; i < 6; i++) {
    noInterrupts();
    long ticks = encoderTicks[i];
    interrupts();

    float tours = (float)ticks / ticksParTourSortie;
    positionCm[i] = tours * circonferencePoulie;

    erreurs[i] = positionCibleCm[i] - positionCm[i];
    moteurActif[i] = abs(erreurs[i]) >= toleranceCm;
  }

  // Commande des moteurs 
  for (int i = 0; i < 6; i++) {
    if (!moteurActif[i]) {
      analogWrite(IN1[i], 0);  // Arrêt
    } else {
      int pwm = 54;
      if (erreurs[i] > 0) {
        digitalWrite(IN2[i], LOW);   // Avant
      } else {
        digitalWrite(IN2[i], HIGH);  // Arrière
      }
      analogWrite(IN1[i], pwm);
    }

    // Affichag
    Serial.print("M");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(positionCm[i], 2);
    Serial.print("cm  ");
  }
  Serial.println();

  delay(1);
}


void lireConsignesSerie() {
  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n') {
      nouvelleConsigne = true;
    } else {
      inputString += c;
    }
  }

  if (nouvelleConsigne) {
    
    int index = 0;
    int lastPos = 0;
    inputString += ',';
    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ',' && index < 6) {
        String val = inputString.substring(lastPos, i);
        positionCibleCm[index] = val.toFloat();
        lastPos = i + 1;
        index++;
      }
    }
    Serial.println("Cibles mises à jour !");
    inputString = "";
    nouvelleConsigne = false;
  }
}
