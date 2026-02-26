#include <PID_v1.h>

// === CONSTANTES PHYSIQUES ===
const float ticksParTourSortie = 960.0;
const float diametrePoulieCm = 2.0;
const float circonferencePoulie = 3.1416 * diametrePoulieCm;
const float toleranceCm = 0.3;

// === DÉCLARATION DES BROCHES ===
const int IN1[6]      = {7, 12, 5, 6, 10, 11};     // PWM
const int IN2[6]      = {26, 27, 32, 34, 23, 25};  // DIR
const int encoderA[6] = {2, 3, 20, 18, 19, 21};
const int encoderB[6] = {28, 33, 30, 48, 35, 39};

// === POTENTIOMÈTRE ===
const int potPin = A0;  // Entrée analogique du potentiomètre
int potValue = 0;
int pwmBase = 0;         // Valeur PWM entre 53 et 78

// === VARIABLES DES MOTEURS ===
volatile long encoderTicks[6] = {0};
double positionCm[6] = {0};
double positionCibleCm[6] = {0};
double pidOutput[6] = {0};

// === PID ===
double Kp = 2.1, Ki = 0.001, Kd = 0.01;
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
  // Tout passe maintenant par Serial (USB)
  Serial.begin(115200);
  inputString.reserve(100);

  // Ce message s'affichera sur l'interface de la Raspberry au démarrage
  Serial.println("Arduino connecte via USB - Pret");

  pinMode(potPin, INPUT);

  for (int i = 0; i < 6; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(encoderA[i], INPUT_PULLUP);
    pinMode(encoderB[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA[i]), interruptFuncs[i], CHANGE);

    pid[i] = new PID(&positionCm[i], &pidOutput[i], &positionCibleCm[i], Kp, Ki, Kd, DIRECT);
    pid[i]->SetMode(AUTOMATIC);
    pid[i]->SetOutputLimits(-53, 53);
  }
}

void loop() {
  // === 1. LECTURE DES CONSIGNES DE LA RASPBERRY (VIA USB) ===
  lireConsignesSerie();

  // === 2. RÉGLAGE DE LA VITESSE ===
  potValue = analogRead(potPin);
  pwmBase = map(potValue, 0, 1023, 53, 78);

  // === 3. CALCUL PID ET COMMANDE MOTEURS ===
  for (int i = 0; i < 6; i++) {
    noInterrupts();
    long ticks = encoderTicks[i];
    interrupts();

    float tours = (float)ticks / ticksParTourSortie;
    positionCm[i] = tours * circonferencePoulie;

    pid[i]->Compute();

    float erreur = abs(positionCibleCm[i] - positionCm[i]);

    if (erreur < toleranceCm) {
      analogWrite(IN1[i], 0);
      digitalWrite(IN2[i], LOW);
    } else {
      int pwm = pwmBase;
      if (pidOutput[i] > 0) {
        digitalWrite(IN2[i], LOW);
        analogWrite(IN1[i], pwm);
      } else {
        digitalWrite(IN2[i], HIGH);
        analogWrite(IN1[i], 255 - pwm);
      }
    }
  }

  // === 4. ENVOI DU FEEDBACK À L'INTERFACE RASPBERRY (VIA USB) ===
  // On envoie les positions réelles pour que l'interface les affiche
  String line = "";
  for (int i = 0; i < 6; i++) {
    line += String(positionCm[i], 2);
    if (i < 5) line += ",";
  }
  Serial.println(line);

  delay(20);
}

// === Fonction de lecture modifiée pour Serial (USB) ===
void lireConsignesSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      nouvelleConsigne = true;
    } else if (c != '\r') { // Ignorer le retour chariot
      inputString += c;
    }
  }

  if (nouvelleConsigne) {
    int index = 0;
    int lastPos = 0;
    inputString += ','; // Séparateur final

    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ',' && index < 6) {
        String val = inputString.substring(lastPos, i);
        positionCibleCm[index] = val.toFloat();
        lastPos = i + 1;
        index++;
      }
    }

    // Feedback de confirmation pour l'interface
    // Serial.println("Cibles mises a jour !");

    inputString = "";
    nouvelleConsigne = false;
  }
}
