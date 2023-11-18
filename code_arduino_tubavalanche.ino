/*

  ----------------------------------------------------------------------------------------------------
  Code pour le Tubavalanche

  © DROULEZ Martin 2021-2023
  ----------------------------------------------------------------------------------------------------

*/



/* NOTES UTILES :
    - les valeurs vérifiées ne sont pas celles qui doivent être utilisées en conditions réelles, ici elles permettent des tests et démonstration
    - le code est commenté afin d'être compréhensible par tous, malgré tout, si vous rencontrez des problèmes n'hésitez pas à me contacter
    - vous trouverez tout au long du code des "Serial.println" commenté, ils permettent lorsqu'ils sont actifs d'afficher toutes les valeurs mesurées dans le moniteur série



    Pour plus d'informations : martin.droulez2@gmail.com
*/



/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Début du code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */



/* -------------------------------------------------- Ajout des bibliothèques : -------------------------------------------------- */

#include <Stepper.h> //librairie pour le moteur
#include <avr/wdt.h> //librairie pour le reset
#include "Wire.h" //librairie pour le gyroscope
#include "DHT.h" //librairie pour le capteur de température / humidité
#include <GY521.h>
#include <Adafruit_MPU6050.h>




/* -------------------------------------------------- Définition des variables : -------------------------------------------------- */



/* Moteur : */
int nombreDePas = 48 * 64; //nombre de pas du moteur pas-à-pas
Stepper monMoteur(nombreDePas, 9, 11, 10, 8); //ports du moteur



/* Bouton : */
int portBouton = 7; //port du bouton
boolean etatBouton = false; //définition de l'état du bouton



/* Capteur à ultrason :
  int pinTrig = 4; //port Trig du capteur à ultrasons
  int pinEcho = 3; //port Echo du capteur à ultrasons
  long temps; //variable pour le temps
  long distance; //variable pour la distance */


/* Vérification de l'état de la pile : */
unsigned long intervalleAppuis; //durée de pression nécessaire sur le bouton



/* Reset : */
unsigned long tempsAvantReset; //pour le temps d'appui avant le reset



/* Gyroscope : */
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // variables pour le gyroscope
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

int minVal = 265; int maxVal = 402; //valeurs minimales et maximales

double x; double y; double z;

boolean avalanche = false; //détecte si une avalanche a eu lieu
boolean avalancheStock = false; //stocke si une avalanche a eu lieu

boolean declencheur1 = false; //stocke si le premier déclencheur a été activé (seuil bas)
boolean declencheur2 = false; //stocke si le deuxième déclencheur a été activé (seuil haut)
boolean declencheur3 = false; //stocke si le troisème déclencheur a été activé (changement d'orientation)

byte declencheur1compte = 0; //stocke le compte depuis que le premier déclencheur a été "true"
byte declencheur2compte = 0; //stocke le compte depuis que le deuxième déclencheur a été "true"
byte declencheur3compte = 0; //stocke le compte depuis que le troisième déclencheur a été "true"
int changementAngle = 0; //valeur du changement d'angle



/* LED : */
const int brocheLed = 5; //la LED est sur le port 5

int etatLed = LOW;

long precedentMillis = 0; //pour le clignotement
long intervale = 1000;



/* Capteur de température / humidité : */
#define DHTPIN 4 //le capteur est sur le port 4

#define DHTTYPE DHT11 //définition du type de capteur utilisé

DHT dht(DHTPIN, DHTTYPE); //déclaration du capteur

float temperature = 0.0;
float humidite = 0.0;



/* -------------------------------------------------- Mise en place : -------------------------------------------------- */

void setup() {

  /* Moteur : */
  monMoteur.setSpeed(9); //définition de la vitesse du moteur



  /* Bouton : */
  pinMode(portBouton, INPUT); //définition du mode ENTRÉE pour le bouton



  /* Capteur à ultrason :
    pinMode(pinTrig, OUTPUT); //définition du mode SORTIE pour le port Trig du capteur à ultrasons
    pinMode(pinEcho, INPUT); //définition du mode ENTRÉ pour le port Echo du capteur à ultrasons
    digitalWrite(pinTrig, LOW); */



  /* Gyroscope : */
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);



  /* Buzzer : */
  pinMode (2, OUTPUT); //déclaration de la broche du buzzer



  /* LED : */
  pinMode (brocheLed, OUTPUT); //déclaration de la broche de la LED



  /* Capteur de température /humidité : */
  dht.begin();



  /* Moniteur serie : */
  Serial.begin(9600); //définition du nombre de baud pour le moniteur série
}



/* -------------------------------------------------- Boucle : -------------------------------------------------- */

void loop () {

  donneesGyroscope();

  /* Code pour la mesure avec le capteur à ultrasons :
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    temps = pulseIn(pinEcho, HIGH);

    if (temps > 25000) {

    Serial.println("Echec de la mesure");

    } else {

    temps = temps / 2;
    distance = (temps * 340) / 10000.0;

    } */

  donneesTemperatureHumidite();



  detectionAvalanche();


  /* Code pour faire tourner le moteur : */
  //if (avalancheStock == true) { //désactivé pour les essais

  //appuiBouton();

  if (digitalRead(portBouton) == HIGH && etatBouton == false) { //détection de l'appui sur le bouton

    etatBouton = true;  // On mémorise l'état du bouton de façon définitive, jusqu'au reset

  } else {

    etatBouton == false;

  }

  if (x >= 210 && x <= 290 && y >= 160 && y <= 240) { //vérification de l'orientation du gyroscope (vers le haut)

    clignotement();

    if (etatBouton == true) { //vérification de l'appui sur le bouton, de la température et de l'humidité (condition valeurs de la température et de l'humidité : && temperature > 0 && humidite > 0)

      creuse();

    } else { //(condition pour la température à ajouter en condition réelle)

      arretMoteur(); //le moteur n'avance pas si les conditions ne sont pas remplies

    }

  } else {

    noTone(2); //le buzzer n'émet aucun son si les conditions ne sont pas remplies

    digitalWrite(brocheLed, LOW); //la LED ne s'allume pas si les conditions ne sont pas remplies

    arretMoteur(); //le moteur n'avance pas si les conditions ne sont pas remplies

  }

  if (etatBouton == false) {

    arretMoteur(); //si l'état du bouton est sur FALSE, le moteur s'arrête de tourner

  }



  /* Code pour la vérification de l'état de la pile : */
  while (digitalRead(portBouton) == HIGH) {

    intervalleAppuis = millis(); //détection de la durée de la pression sur le bouton

  }

  if (intervalleAppuis > 2000) { //les composants s'activent pour vérifier au bout d'une pression de 2 secondes

    digitalWrite(5, HIGH); //la LED s'allume
    tone(2, 428); //le buzzer joue un son
    delay(100);
    tone(2, 528);
    delay(400);
    digitalWrite(5, LOW); //après 500 ms tout s'éteint
    noTone(2);

  }

  /* Code pour le reset : */
  while (digitalRead(portBouton) == HIGH) {

    tempsAvantReset = millis(); //détection de la durée de la pression sur le bouton

  }
  //}

  if (tempsAvantReset > 10000) { //reset si la durée de la pression sur le bouton est supérieure à 10 secondes

    wdt_enable(WDTO_15MS);
    for (;;);

  }
}



/* ---------------------------------------- Récupération des données du gyroscope : ---------------------------------------- */

void donneesGyroscope() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //2050, 77, 1947 sont les valeurs pour la calibration de l'accéléromètre (pour la détection d'avalanche)
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;

  //270, 351, 136 sont les valeurs pour la calibration du gyroscope (pour la détection d'avalanche)
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  //Serial.println( "Orientation : x :" + String(x) + " y :" + String(y) + " z :" + String(z));
}



/* ---------------------------------------- Récupération des données du capteur température / humidité : ---------------------------------------- */

void donneesTemperatureHumidite() {
  temperature = dht.readTemperature(); //lecture de la température
  humidite = dht.readHumidity(); //lecture de l'humidité
  //Serial.println("Temperature = " + String(temperature) + " °C");
  //Serial.println("Humidité = " + String(humidite) + " %");
}



/* ---------------------------------------- Détection d'une avalanche : ---------------------------------------- */

void detectionAvalanche() {

  // calcul du vecteur d'amplitude sur 3 axes
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int AM = Raw_AM * 10;  //comme les valeurs se trouvent entre 0 et 1, elles sont multiplées par 10 pour être utilisées dans les conditions
  //Serial.println("Amplitude =" + AM);

  if (declencheur3 == true) {

    declencheur3compte++;

    if (declencheur3compte >= 10) {

      changementAngle = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);

      //Serial.println(changementAngle);

      if ((changementAngle >= 0) && (changementAngle <= 10)) { //si le changement d'orientation reste entre 0 et 10°

        avalanche = true;
        declencheur3 = false;
        declencheur3compte = 0;

        //Serial.println(changementAngle);

      } else { //ici, la personne a normalement retrouvé une orientation normale

        declencheur3 = false;
        declencheur3compte = 0;

        //Serial.println("Déclencheur 3 désactivé");

      }
    }
  }

  if (avalanche == true) { //si une avalanche est détectée

    //Serial.println("Avalanche détectée");

    tone(2, 900); //activation du buzzer avec un son aïgu
    digitalWrite(brocheLed, HIGH);

    delay(5000);

    avalanche = false;
    avalancheStock = true;
  }

  if (declencheur2compte >= 6) { //accorde 0,5s de changement d'orientation

    declencheur2 = false;
    declencheur2compte = 0;

    //Serial.println("Déclencheur 2 désactivé");

  }

  if (declencheur1compte >= 6) { //accorde 0.5s avant que l'amplitude dépasse le seuil haut

    declencheur1 = false;
    declencheur1compte = 0;

    //Serial.println("Déclencheur 1 désactivé");

  }

  if (declencheur2 == true) {

    declencheur2compte++;

    changementAngle = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);

    //Serial.println(changementAngle);

    if (changementAngle >= 30 && changementAngle <= 400) { //si le changement d'orientation se trouve entre 80 et 100°

      declencheur3 = true;
      declencheur2 = false;
      declencheur2compte = 0;

      //Serial.println(changementAngle);
      //Serial.println("Déclencheur 3 activé");

    }
  }

  if (declencheur1 == true) {

    declencheur1compte++;

    if (AM >= 12) { //si l'amplitude dépasse le seuil haut (3g)

      declencheur2 = true;

      //Serial.println("Déclencheur 2 activé");

      declencheur1 = false;
      declencheur1compte = 0;

    }
  }

  if (AM <= 7 && declencheur2 == false) { //si l'amplitude dépasse le seuil bas (0,4g)

    declencheur1 = true;

    //Serial.println("Déclencheur 1 activé");

  }

  delay(100);
}



/* ---------------------------------------- Détection d'un appui sur le bouton : ---------------------------------------- */

void appuiBouton() {
  if (digitalRead(portBouton) == HIGH && etatBouton == false) { //détection de l'appui sur le bouton

    etatBouton = true;  // On mémorise l'état du bouton de façon définitive, jusqu'au reset

  }
}



/* ---------------------------------------- Fonction de clignotement de la LED et du buzzer : ---------------------------------------- */

void clignotement() {

  unsigned long presentMillis = millis(); // stocke la valeur courante de la fonction millis()

  if (etatBouton == false) {

    if (presentMillis - precedentMillis > intervale) { //pour le clignotement

      precedentMillis = presentMillis; // mémorise la valeur de la fonction millis()

      if (etatLed == LOW) { // inverse la variable d''état de la LED
        etatLed = HIGH;
        tone (2, 400); // le buzzer émet un son grave
      } else {
        etatLed = LOW;
        tone(2, 900); // le buzzer émet un son aïgu
      }
    }

    digitalWrite(brocheLed, etatLed);

  } else if (etatBouton == true) {
    etatLed = LOW;
    noTone(2);
  }
}



/* ---------------------------------------- Fonction de creusage : ---------------------------------------- */

void creuse() {
  digitalWrite(brocheLed, LOW);
  tone(2, 600);

  monMoteur.step(nombreDePas / 2); //le moteur tourne en continu
}



/* ---------------------------------------- Fonction d'arrêt du moteur : ---------------------------------------- */

void arretMoteur() {
  monMoteur.step(0);
}



/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Fin du code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */




/*
  ----------------------------------------------------------------------------------------------------
  Code pour le Tubavalanche

  © DROULEZ Martin 2021-2022
  ----------------------------------------------------------------------------------------------------
*/
