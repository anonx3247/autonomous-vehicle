//////////////////////////////////////////////////////-
// Ce programme a été développé à CENTRALESUPELEC
// Merci de conserver ce cartouche
// Copyright  (c) 2024  CENTRALE-SUPELEC
// Département Electronique et électromagnétisme
// //////////////////////////////////////////////////-
//
// fichier : serial_link.ino
// auteur  : P.BENABES
// Copyright (c) 2024 CENTRALE-SUPELEC
// Revision: 1.0  Date: 20/08/2024
//
// //////////////////////////////////////////////////-
//
// DESCRIPTION DU SCRIPT :
// programme tournant sur les cartes arduino pour la
// commande des moteurs du robot
//
//////////////////////////////////////////////////////

#include "parameters.h"
#include <Servo.h>
//#include "SR04.h"

#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : -1))

char feedback;  // si non nul indique que les commandes doivent renvoyer un acquittement

// gestion du multitache. définition de la période de chaque tache
int delay1 = 100;                    // délai tache 1 de test d'arrivée
int delay2 = 500;                    // délai regulation moteur
int delay3 = 100;                    // démarrage progressif des moteurs
int delay4 = 100;                    // tache de détection des obstacleDetectedacles
int delay5 = 200;                    // tache de détection des obstacleDetectedacles
int time1, time2, time3, time4, time5;  // temps du prochain evenement
bool isCustomTaskOn = false;              // lancement de la tache 1 de test d'arrivée
bool isMotorSpeedCalculationOn = true;               // lancement de la tache 2 de calcul de vitesse
bool isProgressiveAccelerationOn = false;              // lancement de la tache 3 d'accélération progressive
bool isCollisionDetectionOn = false;              // lancement de la tache 4 de détection de collision par IR
bool isServoRotationOn = false;              // lancement de la tache 5 de rotation du servomoteur
bool obstacleDetected = false;                 // obstacleDetectedacle détecté

char c, CharIn, m;
bool cardConnected;         // indique si la carte est connectée
int communicationMode = 0;     // indique le mode de communication 0=tout en ASCII, 1 les commandes sont en binaire et les retours en ascii, 2 tout est en binaire
char retstring[96];  // chaine de retour de commande

// variables de la gestion des moteurs
long int volatile CountIncr1, CountIncr2 = 0;  // valeur des compteurs incrémentaux en 32 bits
int m1Voltage, m2Voltage;                              // tension appliquée au moteur
int echelon1Voltage, echelon2Voltage;                              // tension de consigne à atteindre pour le démarrage en douceur
int mVoltage;                                      // pointeur sur ces variables
char motA, motB;                               // pointeur sur les moteurs
int startSpeed = 10;                               // vitesse de démarrage des moteurs

// variables pour le calcul de la vitesse
long int v1, lv1;
long int v2, lv2;
long int v3, lv3;
int vitesse1, vitesse2;

//SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);  // gestion du capteur ultrasonore
long a;
int v;

// gestion du servomoteur
Servo frontServo;   // create servo objects
int servoPosition = 0;   // commande de position du servomoteur
int servoMin = 30;  // position min du servomoteur
int servoMax = 150;
int servoSpeed = 10;  // vitesse de rotation du servomoteur
int servoDirection = 1;    // sens de rotation du servomoteur


///////////////////////////////////////////////////////////////////////////
//   mise en route des taches périodiques
/////////////////////////////////////////////////////////////////////////
inline void customTaskOn() {
  isCustomTaskOn = true;
  time1 = (int)millis() + delay1;
}
inline void motorSpeedCalculationOn() {
  isMotorSpeedCalculationOn = true;
  time2 = (int)millis() + delay2;
}
inline void progressiveAccelerationOn() {
  isProgressiveAccelerationOn = true;
  time3 = (int)millis() + delay3;
}
inline void collisionDetectionOn() {
  isCollisionDetectionOn = true;
  time4 = (int)millis() + delay4;
}
inline void servoRotationOn() {
  isServoRotationOn = true;
  time5 = (int)millis() + delay5;
}

///////////////////////////////////////////////////////////////////////////
//
//   INITIALISATION DE L'ARDUINO
//
/////////////////////////////////////////////////////////////////////////

void dummy() {
  Serial.println("ER");
}

void init_arduino() {

  // on eteint les moteurs
  m1Voltage = 0;
  m2Voltage = 0;
  set_motor1(m1Voltage);
  set_motor2(m2Voltage);

  servoMin = 30;  // position min du servomoteur
  servoMax = 150;
  servoPosition = (servoMin + servoMax) / 2;
  frontServo.write(servoPosition);
  obstacleDetected = false;

  isCustomTaskOn = false;
  motorSpeedCalculationOn();
  isProgressiveAccelerationOn = false;
  isCollisionDetectionOn = false;
  isServoRotationOn = false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);          // vitesse de la liaison série
  Serial.setTimeout(SERIAL_TIMEOUT);  // timeout pour attendre une fin de message
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1SNS, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2SNS, OUTPUT);
  time1 = (int)millis() + delay1;
  time2 = (int)millis() + delay2;
  time3 = (int)millis() + delay3;
  time4 = (int)millis() + delay4;
  time5 = (int)millis() + delay5;
  frontServo.attach(ServofrontPin);

  init_arduino();

  attachInterrupt(digitalPinToInterrupt(ENCODER1B), IntIncrem1, FALLING);  // attache l'interruption à l'encodeur 1
  attachInterrupt(digitalPinToInterrupt(ENCODER2B), IntIncrem2, FALLING);  // attache l'interruption à l'encodeur 2
}

////////////////////////////////////////////////////////////
//
// la boucle principale
//
/////////////////////////////////////////////////////////////


void loop() {

  if (Serial.available()) {
    // on lit le premier caractère qui arrive
    CharIn = Serial.read();
    if (CharIn == 'A') CONNECT_code();      // demande de connection
    else if (cardConnected) decod_serial(CharIn);  // on ne fait un decodage de commande que si on est connecté
  }

  // lancement des differentes taches périodiques
  if (customTaskOn) customTask();  // tache periodique non définie
  if (motorSpeedCalculationOn) motorSpeedCalculation();  // tache de calcul de la vitesse moteur toujours en route
  if (progressiveAccelerationOn) progressiveAcceleration();  // tache d'accélération progressive des moteurs
  if (collisionDetectionOn) collisionDetection();  // tache de détection de collision
  if (servoRotationOn) servoRotation();  // tache de détection de collision
}



//////////////////////////////////////////////////////////////////////////
//
// LES FONCTIONS DE COMMUNICATION
//
//////////////////////////////////////////////////////////////////////////

// Routine de recuperation d'un caractere
char GetChar(char def) {
  if (Serial.available() > 0)  // liaison série vide
    return (Serial.read());
  else
    return (def);
}

// routine de récupération d'un nombre entier sur la liaison série
// le codage sera ascii ou binaire selon la variable 'communicationMode'
int GetInt(int def) {
  int tmp;
  if (communicationMode == 0) {
    c = Serial.peek();
    if (c == ' ') {
      Serial.read();
      c = Serial.peek();
    }
    if ((c == '-') || ((c >= '0') && (c <= '9')))
      return (Serial.parseInt());  // on recupere la commande du moteur
    else
      return (def);
  }  // renvoie la valeur par défaut
  else {
    Serial.readBytes((char*)&tmp, 2);
    return (tmp);
  }
}

// routine de récupération d'un nombre entier long (32 bits) sur la liaison série
// le codage sera ascii ou binaire selon la variable 'communicationMode'
long int GetLong(long def) {
  long int tmp;
  if (communicationMode == 0) {
    c = Serial.peek();
    if (c == ' ') {
      Serial.read();
      c = Serial.peek();
    }
    if ((c == '-') || ((c >= '0') && (c <= '9'))) {
      tmp = Serial.parseInt();
      return (tmp);  // on recupere la commande du moteur
    } else
      return (def);
  }  // renvoie la valeur par défaut
  else {
    Serial.readBytes((char*)&tmp, 4);
    return (tmp);
  }
}

// bibliothèque de renvoi de valeurs binaires sur la liaison série
inline void write_i8(char num) {
  Serial.write(num);
}

void write_i16(int num) {
  Serial.write((uint8_t*)&num, 2);
}

void write_i32(long num) {
  Serial.write((uint8_t*)&num, 4);
}

/////////////////////////////////////////////////
//  LES FONCTIONS MOTEUR
////////////////////////////////////////////////

// code de mise en route de moteur
inline void set_motor(char MPWM, char MSNS, char sns, int nivm) {
  if (obstacleDetected == true) nivm = 0;  // si on n'a pas d'obstacleDetectedcle
  analogWrite(MPWM, abs(nivm));
  digitalWrite(MSNS, sns ? (nivm >= 0 ? 0 : 1) : (nivm >= 0 ? 1 : 0));
}

inline void set_motor1(int nivm) {
  set_motor(motor1PWM, motor1SNS, 0, nivm);
}

inline void set_motor2(int nivm) {
  set_motor(motor2PWM, motor2SNS, 1, nivm);
}


//////////////////////////////////////////////////////////////////////////////
//
// LES CODES DE TRAITEMENT DES COMMANDES
//
///////////////////////////////////////////////////////////////////////////////


// routine renvoyant le message d'acquitement simple (OK ou OBstacle)
inline void RetAcquitSimpl() {
  if (feedback == 1)
    if (obstacleDetected == false) Serial.println("OK");
    else Serial.println("OB");
}


// code de connection de la carte
void CONNECT_code() {
  delay(1);  // indispensable et pas trop long sinon le caractère suivant n'est pas arrivé
  c = GetChar(0);
  feedback = c - '0';
  cardConnected = true;
  communicationMode = GetChar(0);
  if (communicationMode > 0) communicationMode -= '0';
  else communicationMode = 0;
  init_arduino();

  RetAcquitSimpl();
  if (feedback == 2) {
    sprintf(retstring, "OK Arduino connecte version 1.0 en mode %d", communicationMode);
    Serial.println(retstring);
  }
}

// code de deconnection de la carte
void DISCONNECT_code() {
  init_arduino();
  Serial.println("OK Arduino deconnecte");
  cardConnected = false;
}


// code de remise à 0 des encodeurs
void RESETENC_code() {
  GetLong(0);
  GetLong(0);
  CountIncr1 = 0;
  CountIncr2 = 0;
  RetAcquitSimpl();  // acquitement de la commande en mode feedback=1
  if (feedback == 2)
    Serial.println("Ok encodeurs de position remis à 0");
}

// code pour commander 1 moteur
void SINGLEMOTOR_code() {
  char s;
  delay(1);  // indispensable et pas trop long
  m = GetChar(0);
  if (m == '1') {
    mVoltage = m1Voltage;
    motA = motor1PWM;
    s = 0;
    motB = motor1SNS;
  } else if (m == '2') {
    mVoltage = m2Voltage;
    motA = motor2PWM;
    s = 1;
    motB = motor2SNS;
  }
  if ((m == '1') || (m == '2')) {
    mVoltage = GetInt(0);  // recupere la valeur de la commande
    GetInt(0);
    GetLong(0);
    set_motor(motA, motB, s, mVoltage);  // envoie la commande au moteur
    RetAcquitSimpl();                // acquitement de la commande en mode feedback=1
    if (feedback == 2)
      if (obstacleDetected == false) {
        sprintf(retstring, "OK Moteur %c mis à la tension : %d", m, mVoltage);
        Serial.println(retstring);
      } else
        Serial.println("OB stacle détecté moteur non allumé");
  } else {
    Serial.readString();
    Serial.println("Erreur commande incomplète");
  }
}


// code pour commander les 2 moteurs en même temps
void DUALMOTOR_code() {
  delay(1);  // indispensable et pas trop long

  // on recupere les parametres
  m1Voltage = GetInt(0);      // on lit la valeur du premier moteur
  m2Voltage = GetInt(m1Voltage);  // on lit la valeur du deuxième moteur
  GetLong(0);

  // on envoie la commande aux 2 moteurs
  set_motor1(m1Voltage);
  set_motor2(m2Voltage);
  if ((m1Voltage == 1) && (m2Voltage == 0)) isProgressiveAccelerationOn = false;

  //reponse de la commande
  RetAcquitSimpl();
  if (feedback == 2)
    if (obstacleDetected == false) {
      sprintf(retstring, "OK Moteurs mis aux tensions : %d %d", m1Voltage, m2Voltage);
      Serial.println(retstring);
    } else
      Serial.println("OB stacle détecté moteur non allumé");
}


// code pour la mise en route progressive des 2 moteurs en même temps
void DUALMOTORSLOW_code() {
  delay(1);  // indispensable et pas trop long

  // on recupere les parametres
  echelon1Voltage = GetInt(0);      // on lit la valeur du premier moteur ;
  echelon2Voltage = GetInt(echelon1Voltage);  // on lit la valeur du deuxième moteur ;
  startSpeed = GetInt(25);    // on lit la vitesse
  GetInt(0);

  // on lance la tache d'accélération
  if (obstacleDetected == false)
    progressiveAccelerationOn();

  // réponse de la commande
  RetAcquitSimpl();
  if (feedback == 2)
    if (obstacleDetected == false) {
      sprintf(retstring, "OK Moteurs démarrage progressif : %d %d %d", echelon1Voltage, echelon2Voltage, startSpeed);
      Serial.println(retstring);
    } else
      Serial.println("OB stacle détecté moteur non allumé");
}


// code pour régler la consigne de vitesse pour envoyer les 2 moteurs à une certaine position
void SERVO_code() {
  delay(1);  // indispensable et pas trop long

  // onn recupere les paramètres
  servoPosition = GetInt(0);  // on recupere la vitesse pour positionner le moteur
  GetInt(0);
  GetLong(0);
  if (servoPosition < servoMin) servoPosition = servoMin;
  if (servoPosition > servoMax) servoPosition = servoMax;

  frontServo.write(servoPosition);

  // on renvoie la réponse
  RetAcquitSimpl();
  if (feedback == 2) {
    sprintf(retstring, "OK servomoteur en %d", servoPosition);
    Serial.println(retstring);
  }
}

// code pour régler la consigne min et max des servomoteurs
void SERVO_minmax() {
  delay(1);  // indispensable et pas trop long

  // onn recupere les paramètres
  servoMin = GetInt(0);  // on recupere la vitesse pour positionner le moteur
  servoMax = GetInt(0);
  GetLong(0);
  if (servoMin < 0) servoMin = 0;
  if (servoMax > 270) servoMax = 270;

  // on renvoie la réponse
  RetAcquitSimpl();
  if (feedback == 2) {
    sprintf(retstring, "servomoteur min max = %d %d", servoMin, servoMax);
    Serial.println(retstring);
  }
}


// code de mise en route de la protection moteur
// chaque appel à cette commande réinitialise la détection et autorise le redémarrage des moteurs
void PROTECT_IR_code() {
  delay(1);  // indispensable et pas trop long
  m = GetChar(0);
  if (m == '0') {
    isCollisionDetectionOn = false;
    obstacleDetected = false;
  } else if (m == '1') {
    collisionDetectionOn();
    obstacleDetected = false;
  }
  if (feedback == 1) Serial.println("OK");
  if (feedback == 2) {
    Serial.print("OK protection moteur : ");
    Serial.println(isCollisionDetectionOn);
  }
}


//////////////////////////////////////////////////////////////////////////////
//
// LES CODES DE TRAITEMENT DES QUESTIONS
//
///////////////////////////////////////////////////////////////////////////////

// renvoie la position de 2 encodeurs
void ENCODER_DUAL_code() {
  v1 = CountIncr1;
  v2 = CountIncr2;
  if (communicationMode == 2) {
    write_i32(v1);
    write_i32(v2);
  } else {
    Serial.print(v1);
    Serial.print(" ");
    Serial.println(v2);
  }
}


// renvoie le temps courant et la position d'un encodeur
void ENCODERS_TIME_code() {
  delay(1);  // indispensable et pas trop long sinon le caractère suivant n'est pas arrivé
  c = GetChar(0);
  if (c == '1')
    v2 = CountIncr1;
  else if (c == '2')
    v2 = CountIncr2;

  v1 = millis();
  if (communicationMode == 2) {
    write_i32(v1);
    write_i32(v2);
  } else {
    Serial.print(v1);
    Serial.print(" ");
    Serial.print(v2);
  }
}

// renvoie la vitesse des 2 moteurs
void SPEED_DUAL_code() {
  if (communicationMode == 2) {
    write_i16(vitesse1);
    write_i16(vitesse2);
    write_i16(0);
    write_i16(0);
  } else {
    Serial.print(vitesse1);
    Serial.print(" ");
    Serial.println(vitesse2);
  }
}

// renvoie le temps courant et la valeur du capteur infrarouge
void INFRARED_TIME_code() {
  v1 = millis();
  if (communicationMode == 2) {
    write_i32(v1);
    write_i16(analogRead(IR_pin));
    write_i16(0);
  } else {
    Serial.print(v1);
    Serial.print(" ");
    Serial.println(analogRead(IR_pin));
  }
}

// renvoie la valeur du capteur ultrasons
void ULTRASON_code() {
  //  a=sr04.Distance();
  //  if (communicationMode==2)
  //    write_i16(a);
  //  else
  //    Serial.println(a);
}

// renvoie la tension sur le moteur
void VALMOTOR_code() {
  if (communicationMode == 2) {
    write_i16(m1Voltage);
    write_i16(m2Voltage);
    write_i16(0);
    write_i16(0);
  } else {
    Serial.print(m1Voltage);
    Serial.print(" ");
    Serial.println(m2Voltage);
  }
}

/////////////////////////////////////////////////////
// LE BIG TABLEAU D'AIGUILLAGE DES FONCTIONS
/////////////////////////////////////////////////////


void (*UpperFn[20])() = {
  // tableau des fonctions pour un code en majuscule
  CONNECT_code,                                 // A
  RESETENC_code,                                // B
  DUALMOTOR_code,                               // C
  DUALMOTORSLOW_code,                           // D
  dummy,                                        // E
  dummy,                                        // F
  SERVO_code,                                   // G
  dummy,                                        // H
  PROTECT_IR_code, dummy, dummy, dummy, dummy,  // I,J,K,L,M
  ENCODER_DUAL_code,                            // N
  ENCODERS_TIME_code,                           // O
  SPEED_DUAL_code, dummy,                       // P,Q
  INFRARED_TIME_code,                           // R
  ULTRASON_code,                                // S
  VALMOTOR_code                                 // T
};

void (*LowerFn[20])() = {
  // tableau des fonctions pour un code en minuscule
  DISCONNECT_code,                              // a
  RESETENC_code,                                // b
  SINGLEMOTOR_code,                             // c
  DUALMOTORSLOW_code,                           // d
  dummy,                                        // e
  dummy, SERVO_minmax, dummy,                   // f,g,h
  PROTECT_IR_code, dummy, dummy, dummy, dummy,  // i,J,K,L,M
  ENCODER_DUAL_code,                            // n
  ENCODERS_TIME_code,                           // o
  SPEED_DUAL_code, dummy,                       // p,q
  INFRARED_TIME_code,                           // r
  ULTRASON_code,                                // S
  VALMOTOR_code                                 // T
};


// fonction de décodage des messages
inline void decod_serial(char CharIn) {
  if ((CharIn >= 'A') && (CharIn <= 'T'))  // pour les fonctions 'majuscule'
    UpperFn[CharIn - 'A']();
  else if ((CharIn >= 'a') && (CharIn <= 't'))  // pour les fonctions 'miniscule'
    LowerFn[CharIn - 'a']();
  else
    dummy();
}


////////////////////////////////////////////////////////////////////
//
// LES TACHES PERIODIQUES
//
////////////////////////////////////////////////////////////////////


// tache libre
inline void customTask() {
  if (((int)millis() - time1) > 0)  // si on a atteint le temps programmé
  {

    // A COMPLETER


    time1 = time1 + delay1;
  }
}

// tache de calcul de la vitesse du moteur
inline void motorSpeedCalculation() {
  if (((int)millis() - time2) > 0)  // si il s'est passé 1 seconde depuis la dernière lecture
  {

    // A COMPLETER


    time2 = time2 + delay2;
  }
}

// tache gérant le démarrage (ou l'arret) progressif des moteurs
inline void progressiveAcceleration() {
  if (((int)millis() - time3) > 0)  // si il s'est passé 1 seconde depuis la dernière lecture
  {
    if (m1Voltage < echelon1Voltage) {
      m1Voltage += startSpeed;  // accélération du moteur 1
      if (m1Voltage > echelon1Voltage) m1Voltage = echelon1Voltage;
    }
    if (m1Voltage > echelon1Voltage) {
      m1Voltage -= startSpeed;  // décélération du moteur 1
      if (m1Voltage < echelon1Voltage) m1Voltage = echelon1Voltage;
    }

    if (m2Voltage < echelon2Voltage) {
      m2Voltage += startSpeed;  // accélération du moteur 1
      if (m2Voltage > echelon2Voltage) m2Voltage = echelon1Voltage;
    }
    if (m2Voltage > echelon2Voltage) {
      m2Voltage -= startSpeed;  // décélération du moteur 1
      if (m2Voltage < echelon2Voltage) m2Voltage = echelon2Voltage;
    }

    set_motor1(m1Voltage);
    set_motor2(m2Voltage);

    if ((m1Voltage == echelon1Voltage) && (m2Voltage == echelon2Voltage))
      isProgressiveAccelerationOn = false;

    time3 = time3 + delay3;
  }
}

// tache de détection des obstacleDetectedacles

inline void collisionDetection() {
  if (((int)millis() - time4) > 0)  // si on a atteint le temps programmé
  {
    if (analogRead(IR_pin) > 500)  // on a détecté un obstacleDetectedacle
    {
      obstacleDetected = true;  // indique que l'on a détecté un obstacleDetectedacle
      m1Voltage = 0;
      m2Voltage = 0;
      set_motor1(0);
      set_motor2(0);
      isProgressiveAccelerationOn = false;  // arret du démarrage progressif
    }
    time4 = time4 + delay4;
  }
}

// tache de rotation du servomoteur

inline void servoRotation() {
  if (((int)millis() - time5) > 0)  // si on a atteint le temps programmé
  {

    // A COMPLETER EVENTUELLEMENT

    time5 = time5 + delay5;
  }
}

////////////////////////////////////////////////////////////
//
// les routines d'interruption
//
////////////////////////////////////////////////////////////

// interruption du premier codeur incrémental
void IntIncrem1() {                                     // interruption du décodeur incrémental
  int sns = (digitalRead(ENCODER1A) == LOW) ? +1 : -1;  // on détermine le sens de rotation
  CountIncr1 += sns;                                    // on incrémente le compteur
}

// interruption du deuxième codeur incrémental
void IntIncrem2() {                                     // interruption du décodeur incrémental
  int sns = (digitalRead(ENCODER2A) == LOW) ? -1 : +1;  // on détermine le sens de rotation
  CountIncr2 += sns;                                    // on incrémente le compteur
}
