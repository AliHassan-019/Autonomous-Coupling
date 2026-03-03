// === EPM TEST with BTS7960 ===
// RPWM = D9, LPWM = D10, REN = D7, LEN = D8
// Commandes série :
// - droite  => activation (ON)
// - gauche  => désactivation (OFF)
// - test    => cycle ON/OFF
// - pulse=NNN => durée de l'impulsion (ms)

#define RPWM 9
#define LPWM 10
#define REN 7
#define LEN 8

// Paramètres d'impulsion (ajustables selon l'EPM)
unsigned int pulseMs = 300;
unsigned int settleMs = 50;

void etatRepos() {
  // Etat sûr : aucun PWM actif
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

void activerEPM() {
  // Sens d'activation : courant via RPWM
  Serial.println(">> Activation (magnet ON)");
  etatRepos();
  delay(settleMs);
  digitalWrite(LPWM, LOW);
  analogWrite(RPWM, 255);
  delay(pulseMs);
  analogWrite(RPWM, 0);
  etatRepos();
}

void desactiverEPM() {
  // Sens de désactivation : courant via LPWM
  Serial.println(">> Désactivation (magnet OFF)");
  etatRepos();
  delay(settleMs);
  digitalWrite(RPWM, LOW);
  analogWrite(LPWM, 255);
  delay(pulseMs);
  analogWrite(LPWM, 0);
  etatRepos();
}

void cycleTest() {
  activerEPM();
  delay(2000);
  desactiverEPM();
  delay(2000);
}

String readCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    return cmd;
  }
  return "";
}

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  // Les broches EN restent à HIGH pour activer le pont en H
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  etatRepos();

  Serial.begin(9600);
  Serial.println("EPM Test Ready");
  Serial.println("Commandes: droite (ON), gauche (OFF), test (cycle), pulse=NNN (ms)");
}

void loop() {
  String cmd = readCommand();

  if (cmd == "droite") {
    activerEPM();
  } else if (cmd == "gauche") {
    desactiverEPM();
  } else if (cmd == "test") {
    cycleTest();
  } else if (cmd.startsWith("pulse=")) {
    int val = cmd.substring(6).toInt();
    if (val >= 20 && val <= 2000) {
      pulseMs = (unsigned int)val;
      Serial.print("Pulse mise a jour: ");
      Serial.print(pulseMs);
      Serial.println(" ms");
    } else {
      Serial.println("Valeur invalide (20..2000 ms)");
    }
  } else if (cmd.length() > 0) {
    Serial.println("Commande inconnue. Utilise: droite | gauche | test | pulse=NNN");
  }
}
