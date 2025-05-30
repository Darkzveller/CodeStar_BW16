#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include "Vl53l0x.h"
#include "rgb_lcd.h"
#include "FreeRTOS.h"
#include "task.h"
#include <WiFi.h>
// Cca
char* ssid = "cdf_crac";
const char* password = "cracadmin";
const char* wifiIP = "192.168.0.1";  // Adresse IP du serveur
const char* serverIP = "192.168.0.101";

const int wifiPort = 8080;    // Port utilisé pour tester la connexion WiFi
const int socketPort = 5050;  // Port utilisé pour le socket principal

WiFiClient wifiClient;    // Client pour tester la connexion WiFi (8080)
WiFiClient socketClient;  // Client pour le vrai socket (5050)

#define PWMG PA13
#define SWITCHG PA15
#define PWMD PA12
#define SWITCHD PA14
#define BRAS PA30

TaskHandle_t suivi_ligne = nullptr;

rgb_lcd lcd;

int nb_tof = 2;

int mesures[2];
Vl53l0x monCapteur[2];
VL53L0X_RangingMeasurementData_t measure;




// Crée un objet pour l'ADS1015
Adafruit_ADS1X15 ads;
float erreur = 0, erreurP = 0;
float commande = 0, kp = 1.5, kd = 0.4, ki = 0;
int mg = 0, md = 0;
int vmax = 150;
int suivi = 0;
float cgmax = 0, cgmin = 100000, cdmax = 0, cdmin = 100000;
float cg, cd;

// unsigned long intervalle = 1000;

int etatCapteurs = 0;
int old = 0;

void machineTof()
{
  bool ready;
    VL53L0X_RangingMeasurementData_t rangingMeasurementData;
    int temps = millis();
    int delta = temps - old;
    old = temps;
// #ifdef DEBUG
//     Serial.print(delta);
//     Serial.print(" ");
// #endif
    switch (etatCapteurs) {
      case 0:
        monCapteur[0].startContinuousRangingMeasurement();
        etatCapteurs = 1;
        break;
      case 1:
        monCapteur[0].isReadyContinuousRangingMeasurement(ready);
        if (ready)
          etatCapteurs = 2;
        break;
      case 2:
        monCapteur[0].readContinuousRangingMeasurement(&rangingMeasurementData);
        mesures[0] = (rangingMeasurementData.RangeStatus == 0) ? rangingMeasurementData.RangeMilliMeter : 2000;
        etatCapteurs = 3;
        break;
      case 3:
        monCapteur[1].startContinuousRangingMeasurement();
        etatCapteurs = 4;
        break;
      case 4:
        monCapteur[1].isReadyContinuousRangingMeasurement(ready);
        if (ready)
          etatCapteurs = 5;
        break;
      case 5:
        monCapteur[1].readContinuousRangingMeasurement(&rangingMeasurementData);
        mesures[1] = (rangingMeasurementData.RangeStatus == 0) ? rangingMeasurementData.RangeMilliMeter : 2000;
        etatCapteurs = 0;
        break;
    }
//     for (int i = 0; i < 2; i++) {
//       if (capteurDist[i] >= 2000) {
// #ifdef DEBUG
//         Serial.print("xxx ");
// #endif
//       } else {
// #ifdef DEBUG
//         Serial.print(capteurDist[i]);
//         Serial.print(" ");
// #endif
//       }
//     }
}

void faireFete()
{
  int n = 2000;
  
  while (1) {
    digitalWrite(BRAS, 1);
    delayMicroseconds(n);
    digitalWrite(BRAS, 0);
    delayMicroseconds(20000-n);
    delay(250);  
    if (n==2000) {
      n = 1600;
    } else {
      n = 2000;
    }
  }
}

void read_tof() {
  for (int i = 0; i < nb_tof; i++) {
    monCapteur[i].performSingleRangingMeasurement(&measure);
    mesures[i] = measure.RangeMilliMeter;  // on range toutes les valeurs dans une liste
    Serial.print(mesures[i]);            //on print les deux au cas où y ait une merde
    Serial.print(" ");
  }
  Serial.println("");
}

void init_tof() {

  //fonction pour lancer les tof ! Tout ce dont on a besoin c'est du nombre de tof à init

  for (int i = 0; i < nb_tof; i++) {  // jusqu'à ce qu'on soit à nb_tof -> mini est de 1 tof donc nb_tof = 1

    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();
    delay(100);
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    status = monCapteur[i].begin(I2C_DEFAULT_ADDR, false);
    if (VL53L0X_ERROR_NONE != status) {
      Serial.println("start vl53l0x mesurement failed!");
      monCapteur[i].printPalError(status);
      while (1)
        ;
    }
    monCapteur[i].continuousRangingInit();
    if (VL53L0X_ERROR_NONE != status) {
      Serial.println("start vl53l0x mesurement failed!");
      monCapteur[i].printPalError(status);
      while (1)
        ;
    }
    monCapteur[i].changeAddress(0x50 + i);

    // Wire.beginTransmission(0x70);
    // Wire.write(1 << i);
    // Wire.endTransmission();
    // delay(1000);
    // if (!monCapteur[i].begin(0x29, false)) Serial.print("good");  // on regarde si le démarrage est bon
    // delay(1000);                                                  // delay obligatoire pour lui laisser le temps de bien boot
    // monCapteur[i].changeAddress(0x50 + i);                        // on change l'adresse après le delay
  }

  Wire.beginTransmission(0x70);
  Wire.write(0x0F);  // le multiplexeur est entièrement ouvert
  Wire.endTransmission();

  // delay(1000);  // delay pour laisser le temps au cas où
}

void etalonnage() {
  unsigned long maintenant = millis();
  unsigned long start = maintenant;

  while ((maintenant - start) < 10000) {
    maintenant = millis();
    float captDroit = ads.readADC_SingleEnded(0);   // Lire la valeur du canal A0
    float captGauche = ads.readADC_SingleEnded(1);  // Lire la valeur du canal A1

    if(captGauche > cgmax) cgmax = captGauche;
    if(captGauche < cgmin) cgmin = captGauche;
    if(captDroit > cdmax) cdmax = captDroit;
    if(captDroit < cdmin) cdmin = captDroit;

    Serial.println(maintenant-start);
  }
}

void suivi_de_ligne(void*) {
  unsigned long dernierTOF = 0;
  const unsigned long intervalleTOF = 100;  // 100 ms
  float dErrF = 0;
  int nbFin = 0;
  mesures[0] = 0;
  mesures[1] = 0;
  int tempsDebut;

  while (1) {
//    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (1) {
      // unsigned long maintenant = millis();
      // if (maintenant - dernierTOF >= intervalleTOF) {
      //   dernierTOF = maintenant;
      //   read_tof();  // lecture des distances TOF toutes les 100 ms
      // }
      machineTof();

      int16_t captDroit = ads.readADC_SingleEnded(0);   // Lire la valeur du canal A0
      int16_t captGauche = ads.readADC_SingleEnded(1);  // Lire la valeur du canal A1

      // Serial.print(captGauche);
      // Serial.print(" ");
      // Serial.print(captDroit);
      // Serial.println(" ");

      if ((suivi != 0) && (millis() - tempsDebut >= 99000)) {
        md = mg = 0;
        suivi = 3;
      }

      switch (suivi) {
        case 0:
          if ((mesures[0] > 100) && (mesures[1] > 100)) {
            tempsDebut = millis();
            vTaskDelay(85000);
            suivi = 1;
          }
          break;

        case 1:
          erreur = (float)captGauche - (float)captDroit;
          dErrF = 0.0*dErrF + 1.0*(erreur-erreurP);
          commande = (float)(kp * erreur + kd * dErrF);

          mg = (int)(vmax + commande);
          md = (int)(vmax - commande);

          if (mg > vmax) mg = vmax;
          if (md > vmax) md = vmax;
          if (mg < -vmax/2) mg = -vmax/2;
          if (md < -vmax/2) md = -vmax/2;

          if (((captGauche > 1000) && (captDroit > 1000)) || ((captGauche < 600) && (captDroit < 600))) {
            nbFin++;
            if (nbFin > 4) suivi = 2;
            mg = 70;
            md = 120;
          } else {
            nbFin -= 2;
            if (nbFin<0) nbFin = 0;
          }
          if ((mesures[0] < 100) || (mesures[1] < 100)) suivi = 4;
          erreurP = erreur;
          break;

        case 2:
          //delay(1000);
          mg = 70;
          md = 120;
          // digitalWrite(SWITCHD, LOW);
          // digitalWrite(SWITCHG, HIGH);
          // analogWrite(PWMG, mg);
          // analogWrite(PWMD, md);
          // if ((mesures[0] < 100) || (mesures[1] < 100)) suivi = 4;
          // delay(350);
          // suivi = 3;
          if ((captGauche < 200) || (captDroit < 200)) {
            md = 0;
            mg = 0;
            suivi = 3;
          }
          break;
        case 3:
          mg = 0;
          md = 0;
          digitalWrite(SWITCHD, LOW);
          digitalWrite(SWITCHG, HIGH);
          analogWrite(PWMG, mg);
          analogWrite(PWMD, md);
          lcd.setRGB(0, 255, 0);
          lcd.setCursor(0, 0);
          // lcd.print("C'est carreeee !");
          lcd.print("Score = 30 pts");
          faireFete();
          break;
        case 4:
          mg = 0;
          md = 0;
          if ((mesures[0] > 100) && (mesures[1] > 100)) suivi = 1;
          break;
      }
      digitalWrite(SWITCHD, md<0);
      digitalWrite(SWITCHG, mg>=0);
      analogWrite(PWMG, abs(mg));
      analogWrite(PWMD, abs(md));
      vTaskDelay(pdMS_TO_TICKS(5));  // Pause de 1ms
    }
  }
}

void init_wifi() {

  // 🔹 Connexion au réseau WiFi
  WiFi.begin(ssid, password);
  Serial.print(" Connexion au WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnecté au WiFi !");
  Serial.print("Adresse IP du BW16: ");
  Serial.println(WiFi.localIP());

  // 🔹 Tester la connexion sur le port 8080
  Serial.print("Connexion au serveur sur le port 8080... ");
  if (wifiClient.connect(wifiIP, wifiPort)) {
    Serial.println("Connexion réussie au port 8080 !");
    wifiClient.println("Test connexion WiFi OK");
  } else {
    Serial.println("Échec de connexion au port 8080.");
  }
}

void init_socket() {
  Serial.print("Connexion au serveur sur le port 5050... ");
  if (socketClient.connect(serverIP, socketPort)) {
    Serial.println("Connexion réussie au socket 5050 !");
    socketClient.println("Hello Server, je suis BW16 !");
  } else {
    Serial.println("Échec de connexion au socket 5050.");
  }
}

void read_socket(void*) {
  while (1) {
    if (socketClient.available()) {
      String response = socketClient.readString();
      if(response == "START_STAR" ) {
        xTaskNotifyGive(suivi_ligne);
      }
      if(response == "ETALO") {
        etalonnage();
      }
      //Serial.println("Réponse du serveur : " + response);
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Pause de 100ms
  }
}



void setup() {
  Serial.begin(115200);
  pinMode(SWITCHG, OUTPUT);
  pinMode(SWITCHD, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(PWMG, OUTPUT);
  analogWrite(PWMD, 0);
  analogWrite(PWMG, 0);
  pinMode(BRAS, OUTPUT);  // attache le servo au pin spécifié
  analogWrite(BRAS, 0);
  // Wire.begin();  // Initialise le bus I2C

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);

  lcd.begin(16, 2, false);
  lcd.setRGB(255, 255, 255);

  Serial.println("Avant ADS");
  if (!ads.begin(ADS1X15_ADDRESS, &Wire, true)) {
    Serial.println("Erreur de communication avec l'ADS1015");
    lcd.setRGB(255, 0, 0);
    lcd.setCursor(0, 0);
    lcd.print("Pb ADS1015");
    while (1)
      ;
  }
  init_tof();
  // init_wifi();
  // init_socket();

  lcd.setCursor(0, 0);
  lcd.setRGB(0, 255, 0);
  lcd.print("Init OK !");
  delay(250);
  lcd.clear();
  xTaskCreate(suivi_de_ligne, "Suivi", 4096, NULL, 1, &suivi_ligne);
  // xTaskCreate(read_socket, "Socket", 4096, NULL, 2, 0);
}



void loop() {
}
//   // lcd.setCursor(0, 0);
//   // lcd.print("Hello");










//   // analogWrite(BRAS, 250);
//   // delay(1000);
//   // analogWrite(BRAS, 50);
//   // delay(1000);
// }
