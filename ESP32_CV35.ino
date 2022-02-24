/* Ph CORBEL 10/2019
  Gestion Feux de Signalisation CV35
  (basé sur Feux de Signalisation V2-12)
  cas particulier:
  gestion alimentation routeur auto reboot
  pas de deep sleep, remplacé par Allumage/extinction
  transmission SMS remplacé par Wifi + MQTT
  entree 1 recoit Alarme Chargeur Batterie externe

  2 feux Violet et Blanc
  Etat des feux
              | Violet | Blanc | Feux | Cde
  OFF         |    0   |   0   |  0   |  D Feux D + Tqt Ouvert (si Tqt)
  Violet Fixe |    1   |   0   |  1   |  F
  Violet Cli  |  Cliv1 |   0   |  7   |  V Feux Violet Cli Marche à Vue
  Blanc Fixe  |    0   |   1   |  2   |  O
  Blanc Cli 1 |    0   |  Cli1 |  3   |  M
  Blanc Cli 2 |    0   |  Cli2 |  4   |  S
  Carré Tqt F |    1   |   0   |  5   |  C pas de cde, affichage seulement
  Carré Tqt O |    0   |   0   |  6   |  Z pas de cde, Feux D + Tqt Fermé

  cadence clignotant parametrable
  SlowBlinker = 500 Cli1 0.5s ON, 0.5s OFF
  Cli2 1s OFF, 0.15s ON/OFF pendant 1s
  FastRater = 1000/1300/1600/1900 FastBlinker = 150
  FastRater = 1000/1300/(1400)/1700/1800 FastBlinker = (200)

	Circulation = CalendrierCircule ^ FlagCircule (OU exclusif)
	CalCircule	|	FlagCircule | Circulation
				1			|			0				|			1
				0			|			1				|			1
				0			|			0				|			0
				1			|			1				|			0

	Librairie TimeAlarms.h modifiée a priori pas necessaire nonAVR = 12

  to do 
  1-augmenter nombre tentative conncet Wifi 15->30
  2-parametre reset routeur 1,2 ou 3 fois

  V1-4 05/10/21 installé le 07/10/2021
  Compilation LOLIN D32,default,80MHz, ESP32 1.0.6
  Arduino IDE 1.8.16 : 929858 70%, 46280 14% sur PC
  Arduino IDE 1.8.16 : 929826 70%, 46280 14% sur raspi
  mise à jour toutes les biblio et IDE et ESP32
  voir https://github.com/knolleary/pubsubclient/issues/624
  Ajout timer watchdog, si bloqué dans mqtt attempt -> resetsoft
  au demarrage si resetsoft, reprendre valeur Feux enregistré dans fichier SPIFFS status.txt
  mise à l'heure automatique 1fois/24h

  V1-3 25/09/2021 installé Cv35 30/09/2021
  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 : 908890 69%, 45728 13% sur PC
  Arduino IDE 1.8.10 : 908874 69%, 45728 13% sur raspi

  revue surveillance Alarm Acquisition(se declenchait tout le temps)
  Ajour info log pour tracker probleme blocage

  V1-2 01/09/2021 installé Cv35 10/09/2021
  remplacement mise a l'heure NTP et TZ

  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 : 907090 69%, 45624 13% sur PC
  Arduino IDE 1.8.10 : x 69%, x 13% sur raspi

  V1-1 29/08/2021 pas installé
  suite echec mise à l'heure reseau 4G
  1 - si echec mise a l'heure, heure imposée 2021/08/01/ 08:00:00
  2 - suppression reset hard sur echec
  3 - ajouter surveillance Alarm Acquisition, si arreté suite majheure automatique
      relancer les Alarm
  4 - ajouter commande MAJHEURE=yyyymmddhhmmss ou MAJHEURE => NTP
  5 - ajouter Bouton Reset sur page web
  6 - suppression rearmenent tempo AutoF sur demande ST

  V1-0 29/07/2020 installé
  Version Coupure periodique Alim routeur

*/
String ver        = "V1-4-1";
int    Magique    = 8;

#include "esp_system.h"
#include <rom/rtc.h>
#include <Battpct.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone
#include <WiFiUdp.h>
#include <TimeAlarms.h>
#include <PubSubClient.h>
#include <EEPROM.h>               // variable en EEPROM(SPIFFS)
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"
#include <ArduinoJson.h>
#include "credentials_tpcf_Cv35.h"
#include <ESP32_FTPClient.h>

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

#define RESET_PIN     18   // NC declaré par Sim800l.h
#define LED_PIN       5    // declaré par Sim800l.h
#define PinChckFblc   4    // Entrée verification Cde Feux Blanc
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define Pin12V        39   // Alim générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinIp1        32   // Entrée Ip1 Alarme Batterie externe, 0=Alarme
#define PinBattSol    33   // Mesure Batterie Solaire 24V
#define PinFBlc       21   // Sortie Commande Feux Blanc

#define PinAlimExt12  19   // Sortie Commande coupure Routeur 12V normal = HIGH relais externe repos, coupure = LOW
#define PinFVlt       15   // Sortie Commande Feux Violet
#define PinReset      13   // Reset Hard
#define PinLum        34   // Mesure Luminosité
#define PinAlimLum    25   // Alimentation LDR
#define PinSirene     26   // Sortie Sirene Actif = LOW

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
const String soft = "ESP32_CV35.ino.d32"; // nom du soft

int wdtTimeout = 25;  //time in s to trigger the watchdog
hw_timer_t *timer = NULL;

unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 12V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log
char filelumlut[13]      = "/lumlut.txt";   // fichier en SPIFFS LUT luminosité
char filestatus[13]      = "/status.txt";   // fichier en SPIFFS status Feux

// static const char ntpServerName[] = "fr.pool.ntp.org";
// int timeZone = +1;

const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String fl = "\n";                   // saut de ligne SMS
String Id ;                         // Id du materiel sera lu dans EEPROM00
char   replybuffer[261];            // Buffer de reponse SIM800
volatile int IRQ_Cpt_Ip1  = 0;      // IRQ Ip1
volatile unsigned long rebond1 = 0; // antirebond IRQ
byte DbounceTime = 20;              // antirebond
byte confign = 0;                   // position enregistrement config EEPROM
int  recordn = 300;                 // position enregistrement log EEPROM
// bool Allume  = false;
byte BlcPwmChanel = 0;
byte VltPwmChanel = 1;
bool isBlinking = false;
bool blinker = false;

RTC_DATA_ATTR int  Feux = 0; // Etat des Feux voir tableau au début
RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
// RTC_DATA_ATTR bool FlagMasterOff           = false; // Coupure Allumage en cas de pb
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR bool FlagCircule             = false; // circule demandé -> inverse le calendrier, valid 1 seul jour
RTC_DATA_ATTR bool FileLogOnce             = false; // true si log > seuil alerte

bool FlagAlarmeCdeFBlc       = false; // Alarme defaut commande Feux Blanc
bool FlagLastAlarmeCdeFBlc   = false;
bool FlagAlarme12V           = false; // Alarme tension 12V
bool FlagLastAlarme12V       = false;
bool FlagAlarmeBattExt       = false; // Alarme Regulateur Batterie Externe
bool FlagLastAlarmeBattExt   = false;
bool FlagReset               = false; // Reset demandé
bool jour                    = false;	// jour = true, nuit = false
bool gsm                     = true;  // carte GSM presente utilisé pour test sans GSM seulement

int CoeffTension[4];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB
long   Tension12        = 0; // Tension 24V Allumage
int    Lum              = 0; // Luminosité 0-100%
int    TableLum[11][2];      // Table PWM en fonction Luminosité
unsigned long timeracquisition = 0;// marque heure passage dans Acquisition

WebServer server(80);
File UploadFile;

typedef struct               // declaration structure  pour les log
{
  char    dt[10];            // DateTime 0610-1702 9+1
  char    Act[2];            // Action A/D/S/s 1+1
  char    Name[15];          // 14 car
} champ;
champ record[5];

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  int     anticip;         // temps anticipation du reveille au lancement s
  long    DebutJour;       // Heure message Vie, 7h matin en seconde = 7*60*60
  long    FinJour;         // Heure fin jour, 20h matin en seconde = 20*60*60
  long    AlimExt1;        // Heure 1 Coupure Routeur
  long    AlimExt2;        // Heure 2 Coupure Routeur
  long    AlimExt3;        // Heure 3 Coupure Routeur
  int     TOffExt;         // durée coupure Routeur en s
  int     RouteurAuto;     // true reset Auto du routeur à H1,H2,H3, false marche permanente
  long    RepeatWakeUp;    // Periodicité WakeUp Jour non circulé
  bool    Ip1;             // E1 Actif
  int     SlowBlinker;     // ms
  int     FastBlinker;     // ms
  int     FastRater;       // ms
  int     FVltPWM;         // Modulation Feux Violet %
  int     FBlcPWM;         // Modulation Feux Blanc %
  int     Dsonn;           // Durée Sonnerie
  int     BattLow;         // Seuil Alarme Batterie faible %
  bool    LumAuto;         // luminosité Auto=true
  bool    AutoF;           // true Retour automatique F si O/S apres TempoAutoF
  int     TempoAutoF;      // temps AutoF (s)
  char    Idchar[11];      // Id
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    receiveTopic[17];// channel Id input from server
  char    sendTopic[17];   // channel Id output to server
  char    permanentTopic[20]; // channel Id permanent from/to server
  int     mqttPort;        // Port serveur MQTT
  char    ftpServeur[26];  // serveur ftp
  char    ftpUser[9];      // user ftp
  char    ftpPass[16];     // pwd ftp
  int     ftpPort;         // port ftp
  int     hete;            // decalage Heure été UTC
  int     hhiver;          // decalage Heure hiver UTC
  char    mySSID[21];      // SSID Routeur
  char    myPASSWORD[21];  // PASSWORD Routeur
} ;
config_t config;

Ticker SlowBlink;          // Clignotant lent
Ticker FastBlink;          // Clignotant rapide
Ticker FastRate;           // Repetition Clignotant rapide
Ticker ADC;                // Lecture des Adc

AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée
AlarmId Auto_F;            // Tempo AutoF
AlarmId Alim_Ext_1;        // Heure 1 reset Routeur
AlarmId Alim_Ext_2;        // Heure 2 reset Routeur
AlarmId Alim_Ext_3;        // Heure 3 reset Routeur
AlarmId TSonn;             // tempo durée de la sonnerie

bool AcquisitionOK = false;// surveille boucle acquisition

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
int GTMOffset = 0; // SET TO UTC TIME
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", GTMOffset*60*60, 24*60*60*1000);

// Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 60*config.hhete};     // Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60*config.hhiver};       // Central European Standard Time
Timezone CE(CEST, CET);
// unsigned int localPort = 8888;  // local port to listen for UDP packets
// time_t getNtpTime();
//---------------------------------------------------------------------------
void IRAM_ATTR handleInterruptIp1() { // Entrée 1

  portENTER_CRITICAL_ISR(&mux);
  if (xTaskGetTickCount() - rebond1 > DbounceTime) {
    IRQ_Cpt_Ip1++;
    rebond1 = xTaskGetTickCount(); // equiv millis()
  }
  portEXIT_CRITICAL_ISR(&mux);

}

void IRAM_ATTR resetModule() { // arrive ici si watchdog
  // enregistrer Feux dans SPIFFS pour redemarrage idem
  ets_printf("reboot\n");
  esp_restart(); // "SW_CPU_RESET" case 12
  // ResetHard(); // "POWERON_RESET"
}
//---------------------------------------------------------------------------
void setup() {

  message.reserve(261);

  Serial.begin(115200);
  Serial.println();

  pinMode(PinIp1     , INPUT_PULLUP);
  pinMode(PinFBlc    , OUTPUT);
  pinMode(PinFVlt    , OUTPUT);
  pinMode(PinAlimExt12 , OUTPUT);
  pinMode(PinAlimLum , OUTPUT);
  pinMode(LED_PIN    , OUTPUT);
  pinMode(PinChckFblc, INPUT_PULLUP);
  pinMode(PinSirene  , OUTPUT);
  digitalWrite(PinAlimExt12 , HIGH); // Alimentation Routeur ON, relais externe repos
  digitalWrite(PinAlimLum , HIGH); // Alimentation de la LDR
  digitalWrite(PinSirene , HIGH);  // Alimentation Sirene
  adcAttachPin(PinBattProc);
  adcAttachPin(Pin12V);
  adcAttachPin(PinBattUSB);
  adcAttachPin(PinBattSol);
  adcAttachPin(PinLum);
  
  // parametrage PWM pour les feux
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(BlcPwmChanel, 1000, 8);
  ledcAttachPin(PinFBlc, BlcPwmChanel);
  ledcSetup(VltPwmChanel, 1000, 8);
  ledcAttachPin(PinFVlt, VltPwmChanel);

  ledcWrite(VltPwmChanel, 0); // Feux Violet 0
  ledcWrite(BlcPwmChanel, 0); // Feux Blanc 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms
  /* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);

  EEPROM.get(confign, config); // lecture config
  recordn = sizeof(config) + 1;
  Serial.print("len config ="), Serial.println(sizeof(config));
  EEPROM.get(recordn, record); // Lecture des log
  delay(500);
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println("Nouvelle Configuration !");
    config.magic         = Magique;
    config.anticip       = 2700;
    config.DebutJour     = 8  * 60 * 60;
    config.FinJour       = 19 * 60 * 60;
    config.RepeatWakeUp  = 60 * 60;
    config.Ip1           = true;
    config.SlowBlinker   = 500;
    config.FastBlinker   = 150;
    config.FastRater     = 1000;
    config.FBlcPWM       = 75;
    config.FVltPWM       = 75;
    config.LumAuto       = true;
    config.AutoF         = true;
    config.TempoAutoF    = 3600;
    String temp          = "TPCF_CV35";
    config.hete          = 2;
    config.hhiver        = 1;
    config.AlimExt1      = 5 *60*60 + (15*60);
    config.AlimExt2      = 13*60*60 + (15*60);
    config.AlimExt3      = 21*60*60 + (15*60);
    config.TOffExt       = 2;
    config.RouteurAuto   = true;
    config.Dsonn         = 60;
    config.BattLow       = 50;
    
    tempSSID.toCharArray(config.mySSID, (tempSSID.length() + 1));
    tempPASSWORD.toCharArray(config.myPASSWORD, (tempPASSWORD.length() + 1));
    temp.toCharArray(config.Idchar, 11);
    
    // declaration des Topics MQTT
    Sbidon = temp + "/input";
    Sbidon.toCharArray(config.sendTopic,(Sbidon.length()+1));
    Sbidon = temp + "/output";
    Sbidon.toCharArray(config.receiveTopic,(Sbidon.length()+1));
    Sbidon = temp + "/permanent";
    Sbidon.toCharArray(config.permanentTopic,(Sbidon.length()+1));
    
    config.ftpPort = tempftpPort;
    config.mqttPort = tempmqttPort;
    
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur,(tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser,(tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass,(tempftpPass.length() + 1));

    EEPROM.put(confign, config);
    EEPROM.commit();
    delay(100);
    // valeur par defaut des record (log)
    for (int i = 0; i < 5 ; i++) {
      temp = "";
      temp.toCharArray(record[i].dt, 10);
      temp.toCharArray(record[i].Act, 2);
      temp.toCharArray(record[i].Name, 15);
    }
    EEPROM.put(recordn, record);// ecriture des valeurs par defaut
    EEPROM.commit();
  }
  EEPROM.end();
  PrintEEPROM();
  
  Id  = String(config.Idchar);
  Id += fl;

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(config.Idchar);
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    // arreter WatchdogTimer
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print("Start updating ");
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println("End");
    delay(100);
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });
  
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed...");
    SPIFFS_present = false;
  } else {
    Serial.println("SPIFFS initialised... file access enabled...");
    SPIFFS_present = true;
  }

  setup_wifi();
  // Serial.println("Starting UDP");
  // Udp.begin(localPort);
  timeClient.begin(8888);// port configuré sur routeur SML
  delay (1000);
  Majheure();
  // setSyncProvider(getNtpTime); // mise à l'heure
  // Serial.println(displayTime(0));
  // setSyncInterval(3600);       // intervalle mise à l'heure, 30 s en phase demarrage
  // AIntru_HeureActuelle();      // calcul jour/nuit et timeZone
  // setSyncProvider(getNtpTime); // mise à l'heure new timezone
  // Serial.println(displayTime(0));
  AIntru_HeureActuelle();      // calcul jour/nuit et timeZone
  
  mqttClient.setServer(config.mqttServer, config.mqttPort);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(300);// long message "param"
  
  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  OuvrirLumLUT();             // ouvre le fichier lumLUT

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  DebutJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);
  Alarm.enable(DebutJour);

  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  Auto_F = Alarm.timerRepeat(config.TempoAutoF, AutoFermeture);
  Alarm.disable(Auto_F);
  
  Alim_Ext_1 = Alarm.alarmRepeat(config.AlimExt1, AlimExt);
  Alim_Ext_2 = Alarm.alarmRepeat(config.AlimExt2, AlimExt);
  Alim_Ext_3 = Alarm.alarmRepeat(config.AlimExt3, AlimExt);

  TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  if(config.RouteurAuto){
    Alarm.enable(Alim_Ext_1);
    Alarm.enable(Alim_Ext_2);
    Alarm.enable(Alim_Ext_3);
  } else {
    Alarm.disable(Alim_Ext_1);
    Alarm.disable(Alim_Ext_2);
    Alarm.disable(Alim_Ext_3);
  }

  ActiveInterrupt();

  Serial.print("flag Circule :"), Serial.println(FlagCircule);

  MajLog("Auto","Lancement");
  
  server.on("/",         HomePage);
  server.on("/download", File_Download);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST, []() {
    server.send(200);
  }, handleFileUpload);
  server.on("/delete",   File_Delete);
  server.on("/dir",      SPIFFS_dir);
  server.on("/cal",      CalendarPage);
  server.on("/LumLUT",   LumLUTPage);
  server.on("/datetime", handleDateTime); // renvoie Date et Heure
  server.on("/reset",    demandereset);
  server.begin();
  Serial.println("HTTP server started");
  
  if(rtc_get_reset_reason(0) == 12){ // SW_CPU_RESET, reset watchdog
    Feux = read_status();
    GestionFeux();
  }
  ArduinoOTA.begin();
  recvOneChar();
  timeracquisition = millis();// marque heure passage dans Acquisition
  timer = timerBegin(0, 80, true);                            //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);            //attach callback
  timerAlarmWrite(timer, wdtTimeout * uS_TO_S_FACTOR, false); //set time in us
  timerAlarmEnable(timer);
  Acquisition();
}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar();

  if(millis() - timeracquisition > 20000){ // > boucle Acquisition = 10s
    timeracquisition = millis();
    // Alarm ont ete arretées suite pb mise a l'heure
    MajLog("Auto", "Acquisition > 10s");// renseigne log
    arret_Alarm();
    lancement_Alarm();
    Acquisition();
  }
  timerWrite(timer, 0); //reset timer (feed watchdog)

  if (rebond1 > millis()) rebond1 = millis();

  verifAlarmeBattExt(); // Verification Alarme Batterie Externe

  VerifCdeFBlc(); // Verification commande Feux Blanc
  
  mqttClient.loop();
  server.handleClient(); // Listen for client connections
  ArduinoOTA.handle();
  Alarm.delay(0);
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {
  timeracquisition = millis();
  static int cpt = 0; // compte le nombre de passage boucle
  static bool firstdecision = false;
  // static byte cptwifiattempt = 0;

  if((WiFi.status() != WL_CONNECTED) || year()< 2000 || timeStatus() == timeNotSet){
    // reconnexion Wifi et mise à l'heure si raté au lancement
    Serial.println("tentative cnx Wifi");
    setup_wifi();
    Majheure();
    // setSyncProvider(getNtpTime); // mise à l'heure
    // if(cptwifiattempt ++ > 10)ResetHard();
  }

  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
    reconnect();
  }
  AIntru_HeureActuelle();

  if (cpt > 3 && !firstdecision && timeStatus()!= timeNotSet) {
    /* une seule fois au demarrage attendre au moins 3 passages
       et mise a l'heure OK*/
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
    // setSyncInterval(300); // retour intervalle MAJheure normal
  }

  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 || CoeffTension[3] == 0) {
    OuvrirFichierCalibration(); // patch relecture des coeff perdu
  }

  Serial.println(displayTime(0));
  if(cpt == 0 || cpt> 4320){// 12heures
    MajLog("Auto", "freemem = " + String(ESP.getFreeHeap()));
    cpt = 1;
  }
  cpt ++;
  // Serial.print(F(" Freemem = ")), Serial.println(ESP.getFreeHeap());
  static byte nalaTension = 0;
  static byte nRetourTension = 0;
  static byte nala12V = 0;
  static byte nRetour12V = 0;
  Tension12       = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  VUSB            = map(adc_mm[2] / nSample, 0, 4095, 0, CoeffTension[2]);
  TensionBatterie = map(adc_mm[3] / nSample, 0, 4095, 0, CoeffTension[3]);
  Lum             = map(adc_mm[4] / nSample, 0 , 4095, 100, 0); // Luminosité 0-100%

  Serial.print("luminosité = "), Serial.print(Lum);
  Serial.print(" lumlut = "), Serial.println(lumlut(Lum));

  // en cas de feux fixe rafraichissement commande en fonction lum
  // les feux M et S sont automatiquement ajusté par blink
  if (Feux == 1) Update_FVlt(); // Violet
  if (Feux == 2) Update_FBlc(); // Blanc

  if (Tension12 < 1000  || VUSB < 4000) { // tension 12V & USB
    if(nala12V ++ > 3){
      FlagAlarme12V = true;
      nala12V = 0;
    }
  }
  else if (Tension12 > 1100  && VUSB > 4500) {
    if(nRetour12V ++ > 3){
      FlagAlarme12V = false;
      nala12V = 0;
      nRetour12V = 0;
    }      
  } else{
    if(nala12V > 0) nala12V --;
  }
  
  if (BattPBpct(TensionBatterie, 12) < config.BattLow) {
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (BattPBpct(TensionBatterie, 12) > 80) { //hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
    if (nRetourTension == 4) {
      FlagAlarmeTension = false;
      nRetourTension = 0;
      nalaTension = 0;
    }
  }
  else {
    if (nalaTension > 0)nalaTension--;		//	efface progressivement le compteur
  }

  message = "Batt Solaire = ";
  message += float(TensionBatterie / 100.0);
  message += "V ";
  message += String(BattPBpct(TensionBatterie, 12));
  message += "%";
  message += ", 12V = ";
  message += String(float(Tension12 / 100.0));
  message += ", Batt Proc = ";
  message += (String(VBatterieProc) + "mV ");
  message += String(BattLipopct(VBatterieProc));
  message += "%, V USB = ";
  message += (float(VUSB / 1000.0));
  message += ("V");
  message += fl;
  Serial.print(message);

  if (FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
  }
  envoie_alarme();

  digitalWrite(LED_PIN, 0);
  Alarm.delay(20);
  digitalWrite(LED_PIN, 1);

  Serial.println();
}
//---------------------------------------------------------------------------
void GestionFeux() {
  record_status(); // enregistre Feux dans fichier SPIFFS
  switch (Feux) {
    case 0: // Violet 0, Blanc 0
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, LOW); // extinction Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 1: // Violet 1, Blanc 0
      Serial.println("Feux Violet");
      Update_FVlt();
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 2: // Violet 0, Blanc 1
      Serial.println("Feux Blanc");
      ledcWrite(VltPwmChanel, 0);
      Update_FBlc();
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      break;
    case 3: // Violet 0, Blanc Cli1
      Serial.println("Feux Blc Clignotant lent");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      FastBlink.detach();
      FastRate.detach();
      SlowBlink.attach_ms(config.SlowBlinker, blink);
      break;
    case 4: // Violet 0, Blanc Cli2
      Serial.println("Feux Blc Clignotant rapide");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
      isBlinking = true;
      blinker = false;
      FastRate.attach_ms(config.FastRater, toggle);
      break;
    case 7: // V, Violet Cli, Blanc 0
      Serial.println("Feux Vlt Clignotant lent");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinAlimLum, HIGH); // allumage Alim LDR
      digitalWrite(PinFVlt, LOW);
      FastBlink.detach();
      FastRate.detach();
      SlowBlink.attach_ms(config.SlowBlinker, blink);
      break;
    default:// idem 0 Violet 0, Blanc 0
      Serial.println("Feux Eteint");
      ledcWrite(VltPwmChanel, 0);
      ledcWrite(BlcPwmChanel, 0);
      digitalWrite(PinFVlt, LOW);
      digitalWrite(PinFBlc, LOW);
      digitalWrite(PinAlimLum, LOW); // extinction Alim LDR
      SlowBlink.detach();
      FastBlink.detach();
      FastRate.detach();
  }
}
//---------------------------------------------------------------------------
void toggle() {
  if (isBlinking) {
    FastBlink.detach();
    isBlinking = false;
  }
  else {
    FastBlink.attach_ms(config.FastBlinker, blink);
    isBlinking = true;
  }
}
//---------------------------------------------------------------------------
void blink() {
  if (blinker) {
    if(Feux == 3 || Feux == 4){// M ou S 
      ledcWrite(BlcPwmChanel, 0);
    } else if (Feux == 7){     // V
      ledcWrite(VltPwmChanel, 0);
    }
    blinker = false;
  } else {
    if(Feux == 3 || Feux == 4){// M ou S 
      Update_FBlc();
    } else if (Feux == 7){     // V
      Update_FVlt();
    }
    blinker = true;
  }
}
//---------------------------------------------------------------------------
void Update_FVlt() {
  if (config.LumAuto) {
    ledcWrite(VltPwmChanel, 255 * lumlut(Lum) / 100);
  }
  else {
    ledcWrite(VltPwmChanel, 255 * config.FVltPWM / 100);
  }
}
//---------------------------------------------------------------------------
void Update_FBlc() {
  if (config.LumAuto) {
    ledcWrite(BlcPwmChanel, 255 * lumlut(Lum) / 100);
  }
  else {
    ledcWrite(BlcPwmChanel, 255 * config.FBlcPWM / 100);
  }
}
//---------------------------------------------------------------------------
void Extinction() {
  Serial.println("Exctinction");

  Feux = 0;     // D tout eteint pas de Taquet
  
  GestionFeux();
  
  MajLog("Auto", "Feux = " + String(Feux));
  generationMessage();
  EnvoyerSms(true); // envoie serveur
}
//---------------------------------------------------------------------------
void AutoFermeture() {
  // fin de TempoAutoF
  // Feux à F si Feux = O/S/V rien faire si M
  if (Feux == 2 || Feux == 4 || Feux == 7) {
    Feux = 1;
    GestionFeux(); // Violet 1, Blanc 0
    generationMessage();
    EnvoyerSms(true); // envoie serveur
    MajLog("AutoF", "FCV");
  }
  Alarm.disable(Auto_F);
  Serial.println("fermeture auto F");
}
//---------------------------------------------------------------------------
void traite_sms(byte slot) {
  /* il y a 50 slots dispo
  	si slot= 0, demande depuis message mqtt
  	si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms
  */
  String textesms;													// texte du SMS reçu
  textesms.reserve(261);

  bool sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours
  
  String nom = "serveur";
  if (slot == 99){
    sms = false;
    nom = "console";
  } 

  textesms = String(replybuffer);
  if(!(textesms.indexOf("Wifi") > -1 || textesms.indexOf("MQTTDATA") > -1 
  || textesms.indexOf("MQTTSERVEUR") > -1 || textesms.indexOf("WIFIDATA") > -1 
  || textesms.indexOf("FTPDATA") > -1 || textesms.indexOf("FTPSERVEUR") > -1)){
    textesms.toUpperCase();
  }
  
  textesms.replace(" ", "");// supp tous les espaces
  Serial.print("textesms  = "), Serial.println(textesms);
  messageId();
  if (textesms.indexOf("ETAT") == 0 || textesms.indexOf("ST") == 0) {// "ETAT? de l'installation"
    // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
    generationMessage();
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("SYS") > -1) {    
    message += "Ver: ";
    message += ver + fl;
    message += "V Batt Sol= ";
    message += String(float(TensionBatterie / 100.0));
    message += "V, ";
    message += String(BattPBpct(TensionBatterie, 12));
    message += " %" + fl;
    message += "V USB= ";
    message += (float(VUSB / 1000.0));
    message += "V" + fl;
    message += "12V =";
    message += String(float(Tension12 / 100.0)) + fl;
    message += "Batterie Externe :";
    if(FlagAlarmeBattExt){
      message += "KO";
    } else{
      message += "OK";
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("ID=") == 0) {			//	Id= nouvel Id
    String temp = textesms.substring(3);
    if (temp.length() > 0 && temp.length() < 11) {
      Id = "";
      temp.toCharArray(config.Idchar, 11);
      sauvConfig();														// sauvegarde en EEPROM
      Id = String(config.Idchar);
      Id += fl;
    }
    messageId();
    message += "Nouvel Id";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("LOG") == 0) {	// demande log des 5 derniers commandes
    File f = SPIFFS.open(filelog, "r"); // taille du fichier log en SPIFFS
    message = "local log size :";
    message += String(f.size()) + fl;
    f.close();
    for (int i = 0; i < 5; i++) {
      message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
    }
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("ANTICIP") > -1) { // Anticipation du wakeup
    if (textesms.indexOf(char(61)) == 7) {
      int n = textesms.substring(8, textesms.length()).toInt();
      if (n > 9 && n < 3601) {
        config.anticip = n;
        sauvConfig();														// sauvegarde en EEPROM
      }
    }
    message += "Anticipation WakeUp (s) = ";
    message += config.anticip;
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("DEBUT") == 0) {     //	Heure Message Vie/debutJour
    if (textesms.indexOf(char(61)) == 5) {
      long i = atol(textesms.substring(6).c_str()); //	Heure message Vie
      if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
        config.DebutJour = i;
        sauvConfig();                               // sauvegarde en EEPROM
        Alarm.disable(DebutJour);
        Alarm.write(DebutJour,config.DebutJour);
        // FinJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);// init tempo
        Alarm.enable(DebutJour);
        AIntru_HeureActuelle();
      }
    }
    message += "Debut Journee = ";
    message += Hdectohhmm(config.DebutJour);
    message += "(hh:mm)";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("TIMESAVING") == 0) {
    // TIMESAVING=2:1  +-UTC ete:hiver
    if ((textesms.indexOf(char(61))) == 10) {
      int x = textesms.indexOf(":");
      int i = atoi(textesms.substring(11, x).c_str());
      int j = atoi(textesms.substring(x + 1).c_str());
      if (i < 13 && i > -13 && j < 13 && j > -13) {
        config.hete = i;
        config.hhiver = j;
        sauvConfig();
        // MajHeure();        
      }
    }
    message += "TIMESAVING UTC e:h= ";
    message += String(config.hete) + ":" + String(config.hhiver) + fl;
    EnvoyerSms(sms);
  }
    else if (textesms.indexOf("TIME") == 0) {
    message += "Heure Sys = ";
    message += displayTime(0);
    message += fl;
    EnvoyerSms( sms);
  }
  else if (textesms.indexOf("MAJHEURE") == 0) {	//	forcer mise a l'heure yyyymmddhhmmss =MAJHEURE=20210805180000
    bool valid = false;
    // Serial.println(textesms.substring(9,textesms.length()));
    // Serial.println(textesms.substring(9,textesms.length()).length());
    if(textesms.substring(9,textesms.length()).length() == 14){
      String payload = textesms.substring(9,textesms.length());
      int an   = payload.substring(0,4).toInt();
      int mois = payload.substring(4,6).toInt();
      int jour = payload.substring(6,8).toInt();
      int heur = payload.substring(8,10).toInt();
      int minu = payload.substring(10,12).toInt();
      int seco = payload.substring(12,14).toInt();
      if(an>2000 and an<2200){valid = true;}
      if(mois>0 and mois<13){valid = true;}
      if(jour>0 and jour<32){valid = true;}
      if(heur>0 and heur<24){valid = true;}
      if(minu>0 and minu<60){valid = true;}
      if(seco>0 and seco<60){valid = true;}
      // Serial.println(an);
      // Serial.println(mois);
      // Serial.println(jour);
      // Serial.println(heur);
      // Serial.println(minu);
      // Serial.println(seco);
      if(valid){
        arret_Alarm();
        setTime(heur,minu,seco,jour,mois,an);// mise a la dateHeure
        lancement_Alarm();
        AIntru_HeureActuelle();
      }
    } else {
      arret_Alarm();
      Majheure(); // mise à l'heure
      lancement_Alarm();
      AIntru_HeureActuelle();
    }
    message += "Mise a l'heure" + fl;
    message += "Heure Sys = ";
    message += displayTime(0);
    message += fl;
    EnvoyerSms( sms);
  }
  else if (textesms.indexOf("FIN") == 0) {			//	Heure Fin de journée
    if ((textesms.indexOf(char(61))) == 3) {
      long i = atol(textesms.substring(4).c_str()); //	Heure
      if (i > 0 && i <= 86340) {									  //	ok si entre 0 et 86340(23h59)
        config.FinJour = i;
        sauvConfig();															// sauvegarde en EEPROM
        Alarm.disable(FinJour);
        Alarm.write(FinJour,config.FinJour);
        // FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
        Alarm.enable(FinJour);
      }
    }
    message += "Fin Journee = ";
    message += Hdectohhmm(config.FinJour);
    message += "(hh:mm)";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("AUTOF") == 0) {
    if ((textesms.indexOf(char(61))) == 5) { // =
      if (textesms.substring(6) == "1" || textesms.substring(6) == "0") {
        config.AutoF = textesms.substring(6).toInt();
        sauvConfig();	// sauvegarde en EEPROM
      }
    }
    message += "AutoF ";
    if (config.AutoF == 1) {
      message += "Auto";
    }
    else {
      message += "Manu";
    }
    message += fl;
    message +=  "TempoAutoF (s) = ";
    message += config.TempoAutoF + fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("TEMPOAUTOF") == 0) {
    if ((textesms.indexOf(char(61))) == 10) { // =
      if (textesms.substring(11).toInt() > 100 && textesms.substring(11).toInt() < 36000) {
        config.TempoAutoF = textesms.substring(11).toInt();
        sauvConfig();	// sauvegarde en EEPROM
        Alarm.disable(Auto_F);
        Alarm.write(Auto_F,config.TempoAutoF);
      }
    }
    message += "AutoF ";
    if (config.AutoF == 1) {
      message += "Auto";
    }
    else {
      message += "Stop";
    }
    message += fl;
    message +=  "TempoAutoF (s) = ";
    message += config.TempoAutoF + fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("LUMACTUELLE") == 0) {
    message += "Lum ";
    if (config.LumAuto) {
      message += "Auto";
    }
    else {
      message += "Manu";
    }
    message += fl;
    message += "luminosite = ";
    message += String(Lum);
    message += "\nlumlut = ";
    message += String(lumlut(Lum));
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("LUMAUTO") == 0) {
    if ((textesms.indexOf(char(61))) == 7) { // =
      if (textesms.substring(8) == "1" || textesms.substring(8) == "0") {
        config.LumAuto = textesms.substring(8).toInt();
        sauvConfig();	// sauvegarde en EEPROM
      }
    }
    message += "Luminosite ";
    if (config.LumAuto) {
      message += "Auto";
    }
    else {
      message += "Manu";
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("LUMLUT") > -1) { // Luminosité Look Up Table
    // format valeur de luminosité Feux pour chaque valeur luminosite ambiante
    // de 100 à 0 pas de 10
    // LUMLUT=95,90,80,75,60,50,40,30,30,30,30
    bool flag = true; // validation du format
    byte nv = 0; // compteur virgule
    byte p1 = 0; // position virgule
    if (textesms.indexOf("{") == 0) { // json
      DynamicJsonDocument doc(200);
      int f = textesms.lastIndexOf("}");
      // Serial.print("pos }:"),Serial.println(f);
      // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");
      DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
      if(!err){
        JsonArray LUMLUT = doc["LUMLUT"];
        for (int i = 0; i < 11; i++) {
          TableLum[i][1] = LUMLUT[i];
        }
      }
      else{
        flag = false; // erreur json
      }
    }
    else if ((textesms.indexOf(char(61))) == 6) { // =
      Sbidon = textesms.substring(7, textesms.length());
      for (int i = 0; i < Sbidon.length(); i++) {
        p1 = Sbidon.indexOf(char(44), p1 + 1); // ,
        if ((p1 > 0 && p1 < 255)) {
          nv ++;
          if (nv == 10)break;
        }
        else {
          flag = false;
        }
        // Serial.printf("%s%d,%s%d\n","p1=",p1,"flag=",flag);
      }
      // Serial.print("flag="),Serial.println(flag);
      // }
      // else {
      // flag = false;
      // }
      if (flag) { // format ok
        p1 = 0;
        byte p2 = 0;
        for (int i = 0; i < 11; i++) {
          p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
          TableLum[i][1] = Sbidon.substring(p1, p2).toInt();
          // Serial.printf("%s%d,%s%d\n","p1=",p1,"p2=",p2);
          p1 = p2 + 1;
          TableLum[i][0] = 100 - i * 10;
          if (!(TableLum[i][1] >= 0 && TableLum[i][1] < 101)) flag = false;
          // Serial.printf("%03d,%03d\n",TableLum[i][0],TableLum[i][1]);
        }
      }
    }
    if (flag) { // données OK on enregistre
      EnregistreLumLUT();
    }
    else { // données KO on enregistre pas, et on relie les donnéesnen mémoire
      OuvrirLumLUT();
    }
    if (!sms) {
      // si serveur reponse json
      DynamicJsonDocument doc(200);
      JsonArray lumlut = doc.createNestedArray("lumlut");
      for (int i = 0; i < 11; i++) {
        lumlut.add(TableLum[i][1]);
      }
      String jsonbidon;
      serializeJson(doc, jsonbidon);
      message += jsonbidon;
    } else {
      message += "Table Luminosite (%)\n";
      char bid[10];// 1 ligne
      for (int i = 0; i < 11; i++) {
        sprintf(bid, "%03d,%03d\n", TableLum[i][0], TableLum[i][1]);
        message += String(bid);
      }
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("MOIS") > -1) { // Calendrier pour un mois
    /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
      demande calendrier pour un mois donné ; format : MOIS=mm? */
    bool flag = true; // validation du format
    bool W = true; // true Write, false Read
    int m = 0;
    if (textesms.indexOf("{") == 0) { // json
      DynamicJsonDocument doc(540);
      int f = textesms.lastIndexOf("}");
      // Serial.print("pos }:"),Serial.println(f);
      // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");,1,1,1,1,1,0,0,0,0,0,0]}";
      DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
      if(!err){
        m = doc["MOIS"]; // 12
        JsonArray jour = doc["JOUR"];
        for (int j = 1; j < 32; j++) {
          calendrier[m][j] = jour[j - 1];
        }
        // Serial.print("mois:"),Serial.println(m);
        EnregistreCalendrier(); // Sauvegarde en SPIFFS
        // message += F("Mise a jour calendrier \nmois:");
        // message += m;
        // message += " OK (json)";
      }
      else{
        message += " erreur json ";
        flag = false;
      }
    }
    else { // message normal mois=12,31*0/1
      byte p1 = textesms.indexOf(char(61)); // =
      byte p2 = textesms.indexOf(char(44)); // ,
      if (p2 == 255) {                      // pas de ,
        p2 = textesms.indexOf(char(63));    // ?
        W = false;
      }

      m = textesms.substring(p1 + 1, p2).toInt(); // mois

      // printf("p1=%d,p2=%d\n",p1,p2);
      // Serial.println(textesms.substring(p1+1,p2).toInt());
      // Serial.println(textesms.substring(p2+1,textesms.length()).length());
      if (!(m > 0 && m < 13)) flag = false;
      if (W && flag) { // Write
        if (!(textesms.substring(p2 + 1, textesms.length()).length() == 31)) flag = false; // si longueur = 31(jours)

        for (int i = 1; i < 32; i++) { // verification 0/1
          if (!(textesms.substring(p2 + i, p2 + i + 1) == "0" || textesms.substring(p2 + i, p2 + i + 1) == "1")) {
            flag = false;
          }
        }
        if (flag) {
          // Serial.println(F("mise a jour calendrier"));
          for (int i = 1; i < 32; i++) {
            calendrier[m][i] = textesms.substring(p2 + i, p2 + i + 1).toInt();
            // Serial.print(textesms.substring(p2+i,p2+i+1));
          }
          EnregistreCalendrier(); // Sauvegarde en SPIFFS
          // message += F("Mise a jour calendrier mois:");
          // message += m;
          // message += " OK";
        }
      }
      if(!flag) {
        // printf("flag=%d,W=%d\n",flag,W);
        message += " erreur format ";
      }
    }
    if (flag) { // demande calendrier pour un mois donné
      // if (!sms) {
        // si serveur reponse json  {"mois":12,"jour":[1,2,4,5,6 .. 31]}
        DynamicJsonDocument doc(540);
        doc["mois"] = m;
        JsonArray jour = doc.createNestedArray("jour");
        for (int i = 1; i < 32; i++) {
          jour.add(calendrier[m][i]);
        }
        String jsonbidon;
        serializeJson(doc, jsonbidon);
        message += jsonbidon;
        // message +="{\"mois\":" + String(m) + "," +fl;
        // message += "\"jour\":[";
        // for (int i = 1; i < 32 ; i++){
        // message += String(calendrier[m][i]);
        // if (i < 31) message += ",";
        // }
        // message += "]}";
      // }
      // else {
        // message += "mois = ";
        // message += m;
        // message += fl;
        // for (int i = 1; i < 32 ; i++) {
          // message += calendrier[m][i];
          // if ((i % 5)  == 0) message += " ";
          // if ((i % 10) == 0) message += fl;
        // }
      // }
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms == "CIRCULE") {
    bool ok = false;
    /* demande passer en mode Circulé pour le jour courant,
      sans modification calendrier enregistré en SPIFFS */
    if (!(calendrier[month()][day()] ^ FlagCircule)) {
      // calendrier[month()][day()] = 1;
      message += F("OK, Circule");
      FlagCircule = !FlagCircule;
      ok = true;
    }
    else {
      message += "Jour deja Circule";
    }
    message += fl;
    EnvoyerSms(sms);
    if (ok) {
      // if (sms)EffaceSMS(slot);
      SignalVie();
      // action_wakeup_reason(4);
    }
  }
  else if (textesms == "NONCIRCULE") {
    bool ok = false;
    /* demande passer en mode nonCirculé pour le jour courant,
      sans modification calendrier enregistré en SPIFFS 
      extinction Feux*/
    if (calendrier[month()][day()] ^ FlagCircule) {
      // calendrier[month()][day()] = 0;
      message += "OK, NonCircule";
      FlagCircule = !FlagCircule;
      ok = true;
    }
    else {
      message += "Jour deja NonCircule";
    }
    message += fl;
    EnvoyerSms(sms);
    if (ok) {
      if (sms){
        // EffaceSMS(slot);
      }
      Extinction();
      action_wakeup_reason(4);
    }
  }
  else if (textesms.indexOf("TEMPOWAKEUP") == 0) { // Tempo wake up
    if ((textesms.indexOf(char(61))) == 11) {
      int i = textesms.substring(12).toInt(); //	durée
      if (i > 59 && i <= 36000) { // 1mn à 10H
        config.RepeatWakeUp = i;
        sauvConfig();															// sauvegarde en EEPROM
      }
    }
    message += "Tempo repetition Wake up (s)=";
    message += config.RepeatWakeUp;
    EnvoyerSms(sms);
  }
  else if (textesms == "RST") {               // demande RESET
    message += "Le systeme va etre relance";  // apres envoie du SMS!
    message += fl;
    FlagReset = true;                            // reset prochaine boucle
    EnvoyerSms(sms);
  }
  else if (textesms == "ROUTRST") {          // demande RESET Routeur
    message += "Reset routeur va etre relance";
    message += fl;
    EnvoyerSms(sms);
    Alarm.delay(1000);
    digitalWrite(PinAlimExt12,LOW); // coupure, Alimentation du relais externe
    Alarm.delay(config.TOffExt*1000);
    digitalWrite(PinAlimExt12,HIGH); // retour normal, relais externe repos
    MajLog("Auto","RouteurReset");
  }
  else if (textesms.indexOf("CALIBRATION=") == 0) {
    /* 	Mode calibration mesure tension
        recoit message "CALIBRATION=.X"
        entrer mode calibration
        Selection de la tenssion à calibrer X
        X = 1 Tension12 : Pin12V : CoeffTension1
        X = 2 VBatterieProc : PinBattProc : CoeffTension2
        X = 3 VUSB : PinBattUSB : CoeffTension3
        X = 4 TensionBatterie : PinBattSol : CoeffTension4
        effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        recoit message "CALIBRATION=1250" mesure réelle en V*100
        calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        applique nouveau coeff
        stock en EEPROM
        sort du mode calibration

        variables
        FlagCalibration true cal en cours, false par defaut
        Static P pin d'entrée
        static int tensionmemo memorisation de la premiere tension mesurée en calibration
        int CoeffTension = CoeffTensionDefaut 7000 par défaut
    */
    Sbidon = textesms.substring(12, 16); // texte apres =
    //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
    long tension = 0;
    if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
      if (Sbidon.substring(1, 2) == "1" ) {
        M = 1;
        P = Pin12V;
        coef = CoeffTension[0];
      }
      if (Sbidon.substring(1, 2) == "2" ) {
        M = 2;
        P = PinBattProc;
        coef = CoeffTension[1];
      }
      if (Sbidon.substring(1, 2) == "3" ) {
        M = 3;
        P = PinBattUSB;
        coef = CoeffTension[2];
      }
      if (Sbidon.substring(1, 2) == "4" ) {
        // for (int i = 0; i < 5 ; i++) {
          // adc_read(); // lecture des adc
          // Alarm.delay(100);
        // }
        M = 4;
        P = PinBattSol;
        coef = CoeffTension[3];
      }
      Serial.print("mode = "), Serial.print(M), Serial.println(Sbidon.substring(1, 2));
      FlagCalibration = true;

      coef = CoeffTensionDefaut;
      tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
      // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
      // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
      tensionmemo = tension;
    }
    else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
      // si Calibration en cours et valeur entre 0 et 8000
      Serial.println(Sbidon.substring(0, 4));
      /* calcul nouveau coeff */
      coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
      // Serial.print("Coeff Tension = "),Serial.println(coef);
      tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
      // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
      CoeffTension[M - 1] = coef;
      FlagCalibration = false;
      Recordcalib();														// sauvegarde en SPIFFS
    }
    else {
      message += F("message non reconnu");
      message += fl;
      FlagCalibration = false;
    }
    message += F("Mode Calib Tension ");
    message += String(M) + fl;
    message += F("TensionMesuree = ");
    message += tension;
    message += fl;
    message += F("Coeff Tension = ");
    message += coef;
    if (M == 1) {
      message += fl;
      message += F("Batterie = ");
      message += String(BattPBpct(tension, 12));
      message += "%";
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(Id.substring(5, 9)) == 1) { // cherche CVXX
    if (textesms.indexOf("D") == 0) {
      Extinction(); // Violet 0, Blanc 0
      MajLog(nom, "DCV");
    }
    else if (textesms.indexOf("F") == 0) {
      EffaceAlaCdeFBlc();
      Feux = 1;
      GestionFeux(); // Violet 1, Blanc 0
      MajLog(nom, "FCV");
    }
    else if (textesms.indexOf("O") == 0) {
      EffaceAlaCdeFBlc();
      Feux = 2;
      GestionFeux(); // Violet 0, Blanc 1
      MajLog(nom, "OCV");
      if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
    }
    else if (textesms.indexOf("M") == 0) {
      EffaceAlaCdeFBlc();
      Feux = 3;
      GestionFeux(); // Violet 0, Blanc Manoeuvre Cli lent
      MajLog(nom, "MCV");
      // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
    }
    else if (textesms.indexOf("S") == 0) {
      EffaceAlaCdeFBlc();
      Feux = 4;
      GestionFeux(); // Violet 0, Blanc Secteur Cli rapide
      MajLog(nom, "SCV");
      if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
    }
    else if (textesms.indexOf("V") == 0) {
      EffaceAlaCdeFBlc();
      Feux = 7;
      GestionFeux(); // Violet Cli, Blanc 0
      MajLog(nom, "VCV");
      if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
    }
    else {
      message += "non reconnu" + fl;
      MajLog(nom, "non reconnu");
    }
    generationMessage();
    if (Feux != 0) { // seulement si different de DCV, doublon DCV envoie automatiquement une reponse dans Extinction()
      EnvoyerSms(true); // envoie serveur
    }
    // evite de repondre 2 fois au serveur
    // if (!smsserveur)EnvoyerSms(sms); // reponse si pas serveur
  }
  else if (textesms.indexOf(F("FBLCPWM")) == 0) {
    if (textesms.substring(7, 8) == "=") {
      int i = textesms.substring(8, textesms.length()).toInt();
      if (i > 4 && i < 101) {
        config.FBlcPWM = i;
        sauvConfig();
      }
    }
    GestionFeux();
    message += "Blanc PWM =";
    message += config.FBlcPWM;
    message += "%";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("FVLTPWM")) == 0) {
    if (textesms.substring(7, 8) == "=") {
      int i = textesms.substring(8, textesms.length()).toInt();
      if (i > 4 && i < 101) {
        config.FVltPWM = i;
        sauvConfig();
      }
    }
    GestionFeux();
    message += "Violet PWM =";
    message += config.FVltPWM;
    message += "%";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("SLOWBLINKER")) == 0) {
    if (textesms.substring(11, 12) == "=") {
      int i = textesms.substring(12, textesms.length()).toInt();
      if (i > 4 && i < 2001) {
        config.SlowBlinker = i;
        sauvConfig();
      }
    }
    GestionFeux();
    message += "SlowBlinker =";
    message += config.SlowBlinker;
    message += "ms";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("FASTBLINKER")) == 0) {
    if (textesms.substring(11, 12) == "=") {
      int i = textesms.substring(12, textesms.length()).toInt();
      if (i > 4 && i < 2001) {
        config.FastBlinker = i;
        sauvConfig();
      }
    }
    GestionFeux();
    message += "FastBlinker =";
    message += config.FastBlinker;
    message += "ms";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("FASTRATER")) == 0) {
    if (textesms.substring(9, 10) == "=") {
      int i = textesms.substring(10, textesms.length()).toInt();
      if (i > 4 && i < 3001) {
        config.FastRater = i;
        sauvConfig();
      }
    }
    GestionFeux();
    message += "FastRater =";
    message += config.FastRater;
    message += "ms";
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("PARAM")) >= 0) {
    //message param divisé en 2 trop long depasse long 1sms 160c
    // {"PARAM1":{"SLOWBLINKER":500,"FASTBLINKER":150,"FASTRATER":1000,"DEBUT":"08:00:00","FIN":"20:10:00"}}

    bool erreur = false;
    // Serial.print("position X:"),Serial.println(textesms.substring(7, 8));
    if(textesms.substring(7, 8) == "1"){ // PARAM1
      // Serial.print("position ::"),Serial.println(textesms.substring(9, 10));
      if (textesms.substring(9, 10) == ":") {
        // json en reception sans lumlut
        DynamicJsonDocument doc(200);
        DeserializationError err = deserializeJson(doc, textesms);
        if(err){
          erreur = true;
        }
        else{
          // Serial.print(F("Deserialization succeeded"));
          JsonObject param = doc["PARAM1"];
          config.SlowBlinker = param["SLOWBLINKER"];
          config.FastBlinker = param["FASTBLINKER"];
          config.FastRater = param["FASTRATER"];
          // Serial.print("debut:");
          // Serial.println(Hhmmtohdec(param["DEBUT"]));
          // Serial.print("fin:");
          // Serial.println(Hhmmtohdec(param["FIN"]));
          config.DebutJour = Hhmmtohdec(param["DEBUT"]);
          config.FinJour = Hhmmtohdec(param["FIN"]);
          sauvConfig();
          Alarm.disable(FinJour);
          Alarm.write(FinJour,config.FinJour);
          // FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee);// init tempo
          Alarm.enable(FinJour);
          Alarm.disable(DebutJour);
          Alarm.write(DebutJour,config.DebutJour);
          // FinJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);// init tempo
          Alarm.enable(DebutJour);
        }
      }
      else{
        erreur = true;
      }
    }
    else if(textesms.substring(7, 8) == "2"){ // PARAM2
      if (textesms.substring(9, 10) == ":") {
        // json en reception sans lumlut
        DynamicJsonDocument doc(200);
        DeserializationError err = deserializeJson(doc, textesms);
        if(err){
          erreur = true;
        }
        else{
          // Serial.print(F("Deserialization succeeded"));
          JsonObject param = doc["PARAM2"];
          config.LumAuto = param["LUMAUTO"];
          config.FBlcPWM = param["FBLCPWM"];
          config.FVltPWM = param["FVLTPWM"];
          config.AutoF = param["AUTOF"];
          config.TempoAutoF = param["TEMPOAUTOF"];
          // Serial.print("AutoF:"),Serial.print(config.AutoF);
          // Serial.print("TempoAutoF:"),Serial.print(config.TempoAutoF);
          sauvConfig();
        }
      }
    }
    if(!erreur){
      // ne fonctionne pas
      // const size_t capacity = JSON_ARRAY_SIZE(11) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(11);
      // calculer taille https://arduinojson.org/v6/assistant/
      DynamicJsonDocument doc(500);
      JsonObject param = doc.createNestedObject("param");
      param["slowblinker"] = config.SlowBlinker;
      param["fastblinker"] = config.FastBlinker;
      param["fastrater"] = config.FastRater;
      param["debut"] = Hdectohhmm(config.DebutJour);
      param["fin"] = Hdectohhmm(config.FinJour);
      param["autof"] = config.AutoF;
      param["tempoautof"] = config.TempoAutoF;
      param["fblcpwm"] = config.FBlcPWM;
      param["fvltpwm"] = config.FVltPWM;
      param["lumauto"] = config.LumAuto;

      JsonArray param_lumlut = param.createNestedArray("lumlut");
      for (int i = 0; i < 11; i++) {
        param_lumlut.add(TableLum[i][1]);
      }
      String jsonbidon;
      serializeJson(doc, jsonbidon);
      // serializeJson(doc, Serial);
      message += jsonbidon;
    }
    else{
      message += "erreur json";
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("E1ACTIVE")) == 0) {
    bool valid = false;
    if (textesms.substring(8, 9) == "=") {
      if (textesms.substring(9, 10) == "1") {
        if (!config.Ip1) {
          config.Ip1 = true;
          sauvConfig();
          ActiveInterrupt();
          valid = true;
          MajLog(nom, textesms);
        }
      }
      else if (textesms.substring(9, 10) == "0") {
        if (config.Ip1) {
          config.Ip1 = false;
          sauvConfig();
          DesActiveInterrupt();
          valid = true;
          MajLog(nom, textesms);
        }
      }
      if (valid) {
        sauvConfig();															// sauvegarde en EEPROM
      }
    }
    message += "Entree 1 ";
    if (config.Ip1) {
      message += "Active";
    }
    else {
      message += "InActive";
    }
    message += fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("ROUTEUR") > -1){
    // {"ROUTEUR":{"AUTO":1,"TOFF":2,"H1":"05:15","H2":"13:15","H3":"21:15"}}
    bool erreur = false;
    if (textesms.indexOf(":") == 10) { // format json
      DynamicJsonDocument doc(196); //https://arduinojson.org/v6/assistant/
      DeserializationError err = deserializeJson(doc, textesms);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject routeur = doc["ROUTEUR"];
        config.RouteurAuto = routeur["AUTO"];
        config.TOffExt = routeur["TOFF"];
        config.AlimExt1 = Hhmmtohdec(routeur["H1"]);
        config.AlimExt2 = Hhmmtohdec(routeur["H2"]);
        config.AlimExt3 = Hhmmtohdec(routeur["H3"]);
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    if(!erreur){
      DynamicJsonDocument doc (196);
      JsonObject routeur = doc.createNestedObject("ROUTEUR");
      routeur["AUTO"] = config.RouteurAuto;
      routeur["TOFF"] = config.TOffExt;
      routeur["H1"]   = Hdectohhmm(config.AlimExt1);
      routeur["H2"]   = Hdectohhmm(config.AlimExt2);
      routeur["H3"]   = Hdectohhmm(config.AlimExt3);
      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    EnvoyerSms(sms);
  }
  else if (gsm && textesms.indexOf(F("UPLOADLOG")) == 0) {//upload log
    message += "lancement upload log" + fl;
    MajLog(nom, "upload log");// renseigne log
    ESP32_FTPClient ftp (config.ftpServeur,config.ftpPort,config.ftpUser,config.ftpPass, 5000, 2);
    ftp.OpenConnection();
    // Creation fichier et ecriture
    ftp.InitFile("Type A");
    char tempo[101]; // var temporaire 100 char maxi
    String fichier = String(config.Idchar) + "/log.txt";
    fichier.toCharArray(tempo,(fichier.length()+1));
    ftp.NewFile(tempo);
    Serial.print("Serveur ftp : "),Serial.println(ftp.isConnected());

    if(ftp.isConnected()){
      File f = SPIFFS.open(filelog, "r");// ouverture fichier log en SPIFFS
      while(f.available()){        
        Sbidon = f.readStringUntil('\n');
        Sbidon.toCharArray(tempo,(Sbidon.length()+1));
        ftp.Write(tempo);
        ftp.Write("\n");
      }
      f.close();
    }
    ftp.CloseFile();
    ftp.CloseConnection();
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf(F("DELETELOG")) == 0) {//delete log
    if(SPIFFS.exists(filelog)){
      SPIFFS.remove(filelog);
      FileLogOnce = false;
    }
    MajLog(nom, "delete log");// renseigne log
    message += "efface log" + fl;
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("FTPDATA") > -1) {
    // Parametres FTPDATA=Serveur:User:Pass:port
    // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass",,"port":00}}
    bool erreur = false;
    bool formatsms = false;
    if (textesms.indexOf(":") == 10) { // format json
      DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
      DeserializationError err = deserializeJson(doc, textesms);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject ftpdata = doc["FTPDATA"];
        strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
        strncpy(config.ftpUser,     ftpdata["user"],    11);
        strncpy(config.ftpPass,     ftpdata["pass"],    16);
        config.ftpPort         =    ftpdata["port"];
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    else if ((textesms.indexOf(char(61))) == 7) { // format sms
      formatsms = true;
      byte w = textesms.indexOf(":");
      byte x = textesms.indexOf(":", w + 1);
      byte y = textesms.indexOf(":", x + 1);
      byte zz = textesms.length();
      if (textesms.substring(y + 1, zz).toInt() > 0) { // Port > 0
        if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
          Sbidon = textesms.substring(7, w);
          Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
          Sbidon = textesms.substring(w + 1, x);
          Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
          Sbidon = textesms.substring(x + 1, y);
          Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
          config.ftpPort = textesms.substring(y + 1, zz).toInt();
          sauvConfig();													// sauvegarde en EEPROM
        }
        else {
          erreur = true;
        }
      } else {
        erreur = true;
      }
    }
    if (!erreur) {
      if (formatsms) {
        message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
        message += fl;
        message += F("Parametres FTP :");
        message += fl;
        message += "Serveur:" + String(config.ftpServeur) + fl;
        message += "User:"    + String(config.ftpUser) + fl;
        message += "Pass:"    + String(config.ftpPass) + fl;
        message += "Port:"    + String(config.ftpPort) + fl;
      }
      else {
        DynamicJsonDocument doc(210);
        JsonObject FTPDATA = doc.createNestedObject("FTPDATA");
        FTPDATA["serveur"] = config.ftpServeur;
        FTPDATA["user"]    = config.ftpUser;
        FTPDATA["pass"]    = config.ftpPass;
        FTPDATA["port"]    = config.ftpPort;
        Sbidon = "";
        serializeJson(doc, Sbidon);
        message += Sbidon;
        message += fl;
      }
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
    // case sensitive
    // FTPSERVEUR=xyz.org
    if (textesms.indexOf(char(61)) == 10) {
      Sbidon = textesms.substring(11);
      Serial.print("ftpserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("FTPserveur =");
    message += String(config.ftpServeur);
    message += F("\n au prochain demarrage");
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("WIFIDATA") > -1) { // Borne WIFI
    // case sensitive
    // {"WIFIDATA":{"SSID":"abcd","PASSWORD":"pass"}}
    bool erreur = false;
    if (textesms.indexOf(":") == 11) { // format json
      DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
      DeserializationError err = deserializeJson(doc, textesms);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject wifidata = doc["WIFIDATA"];
        strncpy(config.mySSID,wifidata["SSID"], 21);
        strncpy(config.myPASSWORD,wifidata["PASSWORD"], 21);
        sauvConfig();													// sauvegarde en EEPROM
        setup_wifi();
      }
    }
    if(!erreur){
      message += "Wifi : " + fl;
      message += String(config.mySSID) + fl;
      message += String(config.myPASSWORD) + fl;
      
    } else {
      message += "Erreur format";
      message += fl;
    }    
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("MQTTDATA") > -1){ // parametre MQTT
    // case sensitive
    // Parametres MQTTDATA=serveur:user:pass:port
    bool erreur = false;
    if (textesms.indexOf(":") == 11) { // format json
      DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
      DeserializationError err = deserializeJson(doc, textesms);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject mqttdata = doc["MQTTDATA"];
        strncpy(config.mqttServer,   mqttdata["serveur"], 26);
        strncpy(config.mqttUserName, mqttdata["user"],    11);
        strncpy(config.mqttPass,     mqttdata["pass"],    16);
        config.mqttPort            = mqttdata["port"];
        sauvConfig();													// sauvegarde en EEPROM
      }
    }
    if (!erreur) {
      DynamicJsonDocument doc(210);
      JsonObject MQTTDATA = doc.createNestedObject("MQTTDATA");
      MQTTDATA["serveur"] = config.mqttServer;
      MQTTDATA["user"]    = config.mqttUserName;
      MQTTDATA["pass"]    = config.mqttPass;
      MQTTDATA["port"]    = config.mqttPort;
      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    } else {
      message += "Erreur format";
      message += fl;
    }
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
    // case sensitive
    if (textesms.indexOf(char(61)) == 11) {
      Sbidon = textesms.substring(12);
      Serial.print("mqttserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("MQTTserveur =");
    message += String(config.mqttServer);
    message += F("\n au prochain demarrage");
    EnvoyerSms(sms);
  }
  else if (textesms == "RSTALACDEFBLC") {
    // demande reset Alarme Cde Feux Blanc
    EffaceAlaCdeFBlc();
    message += "Reset Alarme en cours";
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("SIRENE") == 0) { // Lancement SIRENE V2-12
    digitalWrite(PinSirene, LOW);		// Marche Sonnerie
    Alarm.enable(TSonn);					    // lancement tempo
    message += "Lancement Sirene";
    message += fl;
    message += config.Dsonn;
    message += "(s)";
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("SILENCE") == 0 ) {		//	Arret Sirene        
    /*	Arret Sonnerie au cas ou? sans envoyer SMS */
    digitalWrite(PinSirene, HIGH); // Arret Sonnerie
    Alarm.disable(TSonn);         // on arrete la tempo sonnerie
    message += "Arret Sirene";
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("SONN") == 0) { //	Durée Sonnerie
    if (textesms.indexOf(char(61)) == 4) {
      int i = atoi(textesms.substring(5).c_str());			//	Dsonn Sonnerie
      if (i > 4  && i <= 300){ //	ok si entre 10 et 300
        config.Dsonn = i;
        sauvConfig();																// sauvegarde en EEPROM
      }
    }
    message += "Param Sirene = ";
    message += config.Dsonn;
    message += "(s)";
    EnvoyerSms(sms);
  }
  else if (textesms.indexOf("LOWBATT") == 0) { //	Seuil Alarme Batterie faible
    if (textesms.indexOf(char(61)) == 7) {
      int i = atoi(textesms.substring(8).c_str());			//	Dsonn Sonnerie
      if (i > 0  && i <= 100){ //	ok si entre 0 et 100
        config.BattLow = i;
        sauvConfig();																// sauvegarde en EEPROM
      }
    }
    message += "Seuil Alarme Batterie = ";
    message += config.BattLow;
    message += "%";
    EnvoyerSms(sms);
  }
  //**************************************
  else {
    message += F("message non reconnu !");
    message += fl;
    if (nom != F("Moi meme")) EnvoyerSms(sms);
  }
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarme12V != FlagLastAlarme12V) {
    SendEtat = true;
    MajLog(F("Auto"), F("Alarme12V"));
    FlagLastAlarme12V = FlagAlarme12V;
  }
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    MajLog(F("Auto"), F("AlarmeTension"));
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if(FlagAlarmeBattExt != FlagLastAlarmeBattExt){
    SendEtat = true;
    MajLog(F("Auto"), F("AlarmeBattExterne"));
    FlagLastAlarmeBattExt = FlagAlarmeBattExt;
  }
  if (FlagAlarmeCdeFBlc != FlagLastAlarmeCdeFBlc) {
    SendEtat = true;
    MajLog(F("Auto"), "Alarme Cde FBlc");
    FlagLastAlarmeCdeFBlc = FlagAlarmeCdeFBlc;
  }
  if (SendEtat) { 						// si envoie Etat demandé
    generationMessage();
    EnvoyerSms(true);     		// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void generationMessage() {
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarme12V || FlagAlarmeBattExt  || FlagAlarmeCdeFBlc) {
    message += "--KO--------KO--";
  }
  else {
    message += "-------OK-------";
  }
  message += fl;
  
  // message += "Allumage = ";
  // message += Allume;
  // message += fl;
  switch (Feux) {
    case 0: // Violet 0, Blanc 0
      message += "D";
      break;
    case 1: // Violet 1, Blanc 0
      message += "F";
      break;
    case 2: // Violet 0, Blanc 1
      message += "O";
      break;
    case 3: // Violet 0, Blanc Manoeuvre Cli lent
      message += "M";
      break;
    case 4: // Violet 0, Blanc Secteur Cli rapide
      message += "S";
      break;
    case 7: // Violet Cli, Blanc 0
      message += "V";
      break;
  }
  message += String(Id.substring(5, 9));
  message += fl;
  message += "Batterie : ";
  if (!(FlagAlarmeTension || FlagAlarme12V )) {
    message += "OK, ";
    message += String(BattPBpct(TensionBatterie, 12));
    message += "%" + fl;
  } else {
    message += "Alarme, ";
    message += String(BattPBpct(TensionBatterie, 12));
    message += "%" + fl;
    message += "V USB =";
    message += String(float(VUSB / 1000.0)) + fl;
    message += "12V = ";
    message += String(float(Tension12 / 100.0)) + fl;
  }
  message += "Batterie Externe :";
  if(FlagAlarmeBattExt){
    message += "KO";
  } else{
    message += "OK";
  }
  message += fl;

  if ((calendrier[month()][day()] ^ FlagCircule)) {
    message += "Jour Circule" + fl;
  } else {
    message += "Jour Non Circule" + fl;
  }
  if(FlagAlarmeCdeFBlc){
    message += "Defaut Cde Feux Blanc" + fl;
  }
}
//---------------------------------------------------------------------------
void EnvoyerSms(bool sms) {
  if (sms) { // envoie sms
    message = "\"" + message + "\"";    
    message.toCharArray(replybuffer, message.length() + 1);
    int n = mqttClient.publish(config.sendTopic, replybuffer);
    if (n>0) {
      Serial.print(F("send mqtt OK:")),Serial.println(n);
    } else {
      Serial.print(F("send mqtt KO:")),Serial.println(n);
    }
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(message);
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget) { // Htarget Heure de reveil visée
  /* calcul durée entre maintenant et Htarget*/
  long SleepTime = 0;
  long Heureactuelle = HActuelledec();
  if (Heureactuelle < Htarget) {
    SleepTime = Htarget - Heureactuelle;
  }
  else {
    if (Heureactuelle < 86400) { // < 24h00
      SleepTime = (86400 - Heureactuelle) + Htarget;
    }
  }
  return SleepTime;
}
//---------------------------------------------------------------------------
long HActuelledec() {
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes
  return Heureactuelle;
}
//---------------------------------------------------------------------------
void SignalVie() {
  Serial.println(F("Signal vie"));
  AIntru_HeureActuelle();
  
  if ((calendrier[month()][day()] ^ FlagCircule) && jour) { // jour circulé
    // && jour pour cas lancement de nuit pas d'allumage
    Sbidon = F("Jour circule ou demande circulation");
    Serial.println(Sbidon);
    MajLog(F("Auto"), Sbidon);    
    Feux = 1;
    MajLog("Auto", "FCV");
    GestionFeux(); // Violet 1, Blanc 0
  }
  generationMessage();
  EnvoyerSms(true);
  action_wakeup_reason(4);
}
//---------------------------------------------------------------------------
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
  // n = 0 ; dd/mm/yyyy hh:mm:ss
  // n = 1 ; yyyy-mm-dd hh:mm:ss
  char bid[20];
  if (n == 0) {
    sprintf(bid, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  }
  else {
    sprintf(bid, "%4d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  }
  return String(bid);
}
//---------------------------------------------------------------------------
void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
  static int index = 0;
  String temp;
  if (month() < 10) {
    temp =  "0" + String(month());
  }
  else {
    temp = String(month());
  }
  if (day() < 10 ) {
    temp += "0" + String(day());
  }
  else {
    temp += String(day());
  }
  if (hour() < 10) {
    temp += "-0" + String(hour());
  }
  else {
    temp += "-" + String(hour());
  }
  if (minute() < 10) {
    temp += "0" + String(minute());
  }
  else {
    temp += String(minute());
  }
  temp  .toCharArray(record[index].dt, 10);
  nom   .toCharArray(record[index].Name, 15);
  action.toCharArray(record[index].Act, 2);

  EEPROM.begin(512);
  EEPROM.put(recordn, record);// ecriture des valeurs par defaut
  EEPROM.commit();
  EEPROM.end();
  if (index < 4) {
    index ++;
  }
  else {
    index = 0;
  }
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("- failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F(" - not a directory"));
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("  DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("  FILE: "));
      Serial.print(file.name());
      Serial.print(F("\tSIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  file.close();
}
//---------------------------------------------------------------------------
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    return;
  }
  String buf = "";
  int i = 0;
  // Serial.println("- read from file:");
  while (file.available()) {
    int inchar = file.read();
    if (isDigit(inchar)) {
      buf += char(inchar);
      i ++;
    }
  }
  int m = 0;
  int j = 0;
  for (int i = 0; i < 372; i++) { // 12mois de 31 j =372
    j = 1 + (i % 31);
    if (j == 1) m ++;
    calendrier[m][j] = buf.substring(i, i + 1).toInt();
  }
}
//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- message appended");
  } else {
    // Serial.println("- append failed");
  }
}
//---------------------------------------------------------------------------
void MajLog(String Id, String Raison) { // mise à jour fichier log en SPIFFS
  if(SPIFFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        // String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        // char num[13];
        // number.toCharArray(num, 13);
        EnvoyerSms(true);
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("Fichier log plein\n");
      message += String(f.size());
      message += F("\nFichier efface");
      if (gsm) {
        // String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        // char num[13];
        // number.toCharArray(num, 13);
        EnvoyerSms(true);
      }
      SPIFFS.remove(filelog);
      FileLogOnce = false;
    }
    f.close();
    /* preparation de la ligne */
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    Id = ";" + Id + ";";
    Raison += "\n";
    strcat(Cbidon, Id.c_str());
    strcat(Cbidon, Raison.c_str());
    Serial.println(Cbidon);
    appendFile(SPIFFS, filelog, Cbidon);
  }
  else{
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon,config.Idchar);
    strcat(Cbidon,fl.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"),Serial.println(Cbidon);
  }
}
//---------------------------------------------------------------------------
void EnregistreCalendrier() { // remplace le calendrier

  SPIFFS.remove(filecalendrier);
  Sbidon = "";
  char bid[63];
  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Sbidon += calendrier[m][j];
      if (j < 31)Sbidon += char(59); // ;
    }
    Serial.println(Sbidon);
    Sbidon += fl;
    Sbidon.toCharArray(bid, 63);
    appendFile(SPIFFS, filecalendrier, bid);
    Sbidon = "";
  }
}
//---------------------------------------------------------------------------
void EnregistreLumLUT() {
  SPIFFS.remove(filelumlut);
  char bid[9];
  for (int i = 0; i < 11; i++) {
    sprintf(bid, "%d,%d\n", TableLum[i][0], TableLum[i][1]);
    appendFile(SPIFFS, filelumlut, bid);
  }
}
//---------------------------------------------------------------------------
void OuvrirLumLUT() {
  if (SPIFFS.exists(filelumlut)) {
    File f = SPIFFS.open(filelumlut, "r");
    for (int i = 0; i < 11; i++) { //Read 11 lignes
      String s = f.readStringUntil('\n');
      int pos = s.indexOf(",");
      TableLum[i][0] = s.substring(0, pos).toInt();
      TableLum[i][1] = s.substring(pos + 1, s.length() ).toInt();
    }
    f.close();
  }
  else {
    Serial.println("Fichier LumLUT n'existe pas, creation val par defaut");
    char bid[9];
    for (int i = 0; i < 11; i++) {
      int v2;
      int v1 = 100 - i * 10;
      if (i < 10) {
        v2 = v1;
      }
      else {
        v2 = 10;
      }
      sprintf(bid, "%d,%d\n", v1, v2);
      appendFile(SPIFFS, filelumlut, bid);
      TableLum[i][0] = v1;
      TableLum[i][1] = v2;
    }
  }
  // for (int i = 0; i < 11 ; i++) {
  //   Serial.print(TableLum[i][0]), Serial.print(","), Serial.println(TableLum[i][1]);
  // }
}
//---------------------------------------------------------------------------
int lumlut(int l) {
  // retourn la valeur lut en fonction de lum actuelle
  for (int i = 0; i < 11; i++) {
    if (l >= TableLum[i][0]) {
      // Serial.printf("%s%d,%d\n","lumlut=",l,TableLum[i][1]);
      return TableLum[i][1];;
    }
  }
  return 0;
}
//---------------------------------------------------------------------------
void OuvrirCalendrier() {

  // this opens the file "f.txt" in read-mode
  listDir(SPIFFS, "/", 0);
  bool f = SPIFFS.exists(filecalendrier);
  // Serial.println(f);
  File f0 = SPIFFS.open(filecalendrier, "r");

  if (!f || f0.size() == 0) {
    Serial.println(F("File doesn't exist yet. Creating it")); // creation calendrier defaut
    char bid[63];
    Sbidon = "";
    for (int m = 1; m < 13; m++) {
      for (int j = 1; j < 32; j++) {
        if (m == 1 || m == 2 || m == 3 || m == 11 || m == 12) {
          Sbidon += "0;";
        }
        else {
          Sbidon += "1;";
        }
      }
      Serial.println(Sbidon);
      Sbidon += fl;
      Sbidon.toCharArray(bid, 63);
      appendFile(SPIFFS, filecalendrier, bid);
      Sbidon = "";
    }
  }
  readFile(SPIFFS, filecalendrier);

  // for (int m = 1; m < 13; m++) {
  //   for (int j = 1; j < 32; j++) {
  //     Serial.print(calendrier[m][j]), Serial.print(char(44));
  //   }
  //   Serial.println();
  // }
  // listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee() {
  // fin de journée extinction
  jour = false;
  FlagCircule = false;
  FirstWakeup = true;
  digitalWrite(PinAlimLum , LOW); // couper alimentation LDR
  Extinction();
  Serial.println("Fin de journee extinction");
  // TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);// xx mn avant
  // calculTimeSleep();
  Sbidon  = "FinJour, extinction";
  // Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog(F("Auto"), Sbidon);
  // DebutSleep(); // pas de sleep pour CV35
}
//---------------------------------------------------------------------------
void PrintEEPROM() {
  Serial.print("Version = ")                 , Serial.println(ver);
  Serial.print("ID = ")                      , Serial.println(config.Idchar);
  Serial.print("magic = ")                   , Serial.println(config.magic);
  Serial.print("Debut Jour = ")              , Serial.println(Hdectohhmm(config.DebutJour));
  Serial.print("Fin jour = ")                , Serial.println(Hdectohhmm(config.FinJour));
  Serial.print("Reset Routeur H1 = ")        , Serial.println(Hdectohhmm(config.AlimExt1));
  Serial.print("Reset Routeur H2 = ")        , Serial.println(Hdectohhmm(config.AlimExt2));
  Serial.print("Reset Routeur H3 = ")        , Serial.println(Hdectohhmm(config.AlimExt3));
  Serial.print("Reset Routeur Off (s) = ")   , Serial.println(config.TOffExt);
  Serial.print("Routeur Auto = ")            , Serial.println(config.RouteurAuto);
  Serial.print("Duree Sirene = ")            , Serial.println(config.Dsonn);
  Serial.print("Alarme Batterie Faible (%) = ") , Serial.println(config.BattLow);
  Serial.print("T anticipation Wakeup = ")   , Serial.println(config.anticip);
  Serial.print("Tempo repetition Wake up (s)= "), Serial.println(config.RepeatWakeUp);
  Serial.print("Entrée Externe 1 Active = ") , Serial.println(config.Ip1);
  Serial.print("Vitesse SlowBlinker = ")     , Serial.println(config.SlowBlinker);
  Serial.print("Vitesse FastBlinker = ")     , Serial.println(config.FastBlinker);
  Serial.print("Vitesse RepetFastBlinker = "), Serial.println(config.FastRater);
  Serial.print("PWM Blanc = ")               , Serial.println(config.FBlcPWM);
  Serial.print("PWM Violet = ")              , Serial.println(config.FVltPWM);
  Serial.print("Luminosité Auto = ")         , Serial.println(config.LumAuto);
  Serial.print("Auto F si O/S = ")           , Serial.println(config.AutoF);
  Serial.print("Tempo Auto (s) = ")          , Serial.println(config.TempoAutoF);
  Serial.print("SSID Routeur = ")            , Serial.println(config.mySSID);
  Serial.print("PWD Routeur = ")             , Serial.println(config.myPASSWORD);
  Serial.print("ftp serveur = ")             , Serial.println(config.ftpServeur);
  Serial.print("ftp port = ")                , Serial.println(config.ftpPort);
  Serial.print("ftp user = ")                , Serial.println(config.ftpUser);
  Serial.print("ftp pass = ")                , Serial.println(config.ftpPass);
  Serial.print("mqtt serveur = ")            , Serial.println(config.mqttServer);
  Serial.print("mqtt user = ")               , Serial.println(config.mqttUserName);
  Serial.print("mqtt pass = ")               , Serial.println(config.mqttPass);
  Serial.print("mqtt port = ")               , Serial.println(config.mqttPort);
  Serial.print("mqtt receive topic = ")      , Serial.println(config.receiveTopic);
  Serial.print("mqtt send topic = ")         , Serial.println(config.sendTopic);
  Serial.print("mqtt permanent topic = ")    , Serial.println(config.permanentTopic);
}
//---------------------------------------------------------------------------
void ResetHard() {
  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 4; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print(F("Creating Data File:")), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    CoeffTension[2] = CoeffTensionDefaut;
    CoeffTension[3] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print(F("Coeff T Batt 24V = ")), Serial.print(CoeffTension[3]);
  Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
  Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
  Serial.print(F(" Coeff T 12V = "))		, Serial.println(CoeffTension[0]);

}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.println(CoeffTension[3]);
  f.close();
}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec) {
  // convert heure decimale en hh:mm:ss
  String hhmm;
  if (int(Hdec / 3600) < 10) hhmm = "0";
  hhmm += int(Hdec / 3600);
  hhmm += ":";
  if (int((Hdec % 3600) / 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) / 60);
  hhmm += ":";
  if (int((Hdec % 3600) % 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) % 60);
  return hhmm;
}
//---------------------------------------------------------------------------
long Hhmmtohdec(String h){
  // convert heure hh:mm:ss en decimale
  int H = h.substring(0,2).toInt();
  int M = h.substring(3,5).toInt();
  int S = h.substring(6,8).toInt();
  long hms = H*3600 + M*60 + S;
  return hms;
}
//---------------------------------------------------------------------------
void DesActiveInterrupt() {
  if (config.Ip1) {
    detachInterrupt(digitalPinToInterrupt(PinIp1));
  }
}
//---------------------------------------------------------------------------
void ActiveInterrupt() {
  if (config.Ip1) {
    attachInterrupt(digitalPinToInterrupt(PinIp1), handleInterruptIp1, FALLING);
  }
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() {

  long Heureactuelle = HActuelledec();

  if (config.FinJour > config.DebutJour) {
    if ((Heureactuelle > config.FinJour && Heureactuelle > config.DebutJour)
        || (Heureactuelle < config.FinJour && Heureactuelle < config.DebutJour)) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  else {
    if (Heureactuelle > config.FinJour && Heureactuelle < config.DebutJour) {
      // Nuit
      IntruD();
    }
    else {	// Jour
      IntruF();
    }
  }
  // if(HeureEte()){
  //   // timeZone = config.hete;
  // } else {
  //   // timeZone = config.hhiver;
  // }
}
//---------------------------------------------------------------------------
void IntruF() { // Charge parametre Alarme Intrusion Jour
  jour = true;
  // Serial.println(F("Jour"));
}
//---------------------------------------------------------------------------
void IntruD() { // Charge parametre Alarme Intrusion Nuit
  jour = false;
  // Serial.println(F("Nuit"));
}
//---------------------------------------------------------------------------
void DebutSleep() {

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print(F("s ;"));
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();

  if (TIME_TO_SLEEP == 1) {
    Serial.println(F("pas de sleep on continue"));
    return;
  }
  //Go to sleep now
  Serial.println(F("Going to sleep now"));

  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println(F("This will never be printed"));
  Serial.flush();

}
//---------------------------------------------------------------------------
void action_wakeup_reason(byte wr) { // action en fonction du wake up
  Serial.print(F("Wakeup :")), Serial.print(wr);
  Serial.print(F(", jour :")), Serial.print(jour);
  Serial.print(F(" ,Calendrier :")), Serial.print(calendrier[month()][day()]);
  Serial.print(F(" ,FlagCircule :")), Serial.println(FlagCircule);
  byte pin = 0;
  Serial.println(F("***********************************"));
  if (wr == 99 || wr == 32 || wr == 33 || wr == 34) {
    pin = wr;
    wr = 3;
  }
  if (wr == 0)wr = 4; // demarrage normal, decision idem timer

  switch (wr) {
    case 2: break; // ne rien faire ESP_SLEEP_WAKEUP_EXT0

    case 3: // ESP_SLEEP_WAKEUP_EXT1

      /* declenchement externe pendant deep sleep
      	si nuit ou jour noncirculé
      	on reste en fonctionnement pendant TempoAnalyse
      	avant retour deep sleep*/
      // WupAlarme = true;
      // LastWupAlarme = true;
      // Alarm.enable(TempoAnalyse); // debut tempo analyse ->fonctionnement normal
      Sbidon = F("Externe Debut ");
      Sbidon += String(pin);
      MajLog(F("Alarme"), Sbidon);
      // }
      break;

    case 4: // SP_SLEEP_WAKEUP_TIMER
      if (FirstWakeup) { // premier wake up du jour avant DebutJour
        // SignalVie();
        // ne rien faire, attendre DebutJour
        FirstWakeup = false;
        if (HActuelledec() > config.DebutJour) {
          // premier lancement en journée
          if(rtc_get_reset_reason(0) != 12){ // si pas SW_CPU_RESET, reset watchdog
            SignalVie();
          }
        }
        break;
      } else { // nouveau wake up en journée (suite reset routeur)
        Sbidon = F("nouveau wakeup journée");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);    
        // Feux = 1; // on garde valeur Feux memorisée
        MajLog("Auto", "Feu=" + String(Feux));
        GestionFeux();
      }
      if ((calendrier[month()][day()] ^ FlagCircule) && jour) { // jour circulé & jour
      //rien
      }
      else { // non circulé
        Sbidon = F("Jour noncircule ou nuit");
        Serial.println(Sbidon);
        MajLog(F("Auto"), Sbidon);
        // pas de sleep CV35
        // calculTimeSleep();
        // if (TIME_TO_SLEEP <= config.anticip) { // on continue sans sleep attente finjour
          // Sbidon = F("on continue sans sleep");
          // Serial.println(Sbidon);
          // MajLog(F("Auto"), Sbidon);
        // }
        // else {
          // DebutSleep();
        // }
      }
      break;

    case 5: break;  // ne rien faire ESP_SLEEP_WAKEUP_TOUCHPAD
    case 6: break;  // ne rien faire ESP_SLEEP_WAKEUP_ULP
      // default: break; // demarrage normal
  }
}
//---------------------------------------------------------------------------
void calculTimeSleep() {

  AIntru_HeureActuelle(); // determine si jour/nuit

  if (jour && (HActuelledec() + config.RepeatWakeUp) > config.FinJour) {
    if (HActuelledec() > (config.FinJour - config.anticip)) {
      /* eviter de reporter 24H si on est à moins de anticip de FinJour */
      TIME_TO_SLEEP = 1; // si 1 pas de sleep
    }
    else {
      TIME_TO_SLEEP = DureeSleep(config.FinJour - config.anticip);
      Serial.print(F("time sleep calcul 1 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else if (!jour) {
    if (HActuelledec() < (config.DebutJour - config.anticip)) {
      TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);
      Serial.print(F("time sleep calcul 2 : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
    else if (HActuelledec() < 86400) {
      TIME_TO_SLEEP = (86400 - HActuelledec()) + config.DebutJour - config.anticip;
      Serial.print(F("time sleep calcul 2bis : ")), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else {
    TIME_TO_SLEEP = config.RepeatWakeUp;
    Serial.print(F("time sleep calcul 3 : ")), print_uint64_t(TIME_TO_SLEEP);
    Serial.println("");
  }

  /* Garde fou si TIME_TO_SLEEP > 20H00 c'est une erreur, on impose 1H00 */
  if (TIME_TO_SLEEP > 72000) {
    TIME_TO_SLEEP = 3600;
    Sbidon = "jour ";
    Sbidon += jour;
    Sbidon = ", Calendrier ";
    Sbidon += calendrier[month()][day()];
    Sbidon = ", flagCirc ";
    Sbidon += FlagCircule;
    MajLog("Auto", Sbidon);
    Sbidon = "Attention erreur Sleep>20H00 ";
    Sbidon += Hdectohhmm(TIME_TO_SLEEP);
    MajLog("Auto", Sbidon);
  }

  Sbidon = "lance timer : ";
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog("Auto", Sbidon);
}
//---------------------------------------------------------------------------
int get_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint64_t wakeup_pin_mask;
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : return ESP_SLEEP_WAKEUP_EXT0; // 2
    case ESP_SLEEP_WAKEUP_EXT1: //{// 3
      wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_pin_mask != 0) {
        int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
        Serial.print("Wake up from GPIO "); Serial.println(String(pin));
        return pin; // pin
      } else {
        Serial.println(" Wake up from GPIO ?");
        return 99; // 99
      }
      break;
    // }
    case ESP_SLEEP_WAKEUP_TIMER    : return ESP_SLEEP_WAKEUP_TIMER; // 4
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return ESP_SLEEP_WAKEUP_TOUCHPAD; // 5
    case ESP_SLEEP_WAKEUP_ULP      : return ESP_SLEEP_WAKEUP_ULP; // 6
    default : return 0; // Serial.println("Wakeup was not caused by deep sleep"); break;// demarrage normal
  }
  Serial.flush();
}
//---------------------------------------------------------------------------
void print_uint64_t(uint64_t num) {

  char rev[128];
  char *p = rev + 1;

  while (num > 0) {
    *p++ = '0' + ( num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    Serial.print(*p--);
  }
}
//---------------------------------------------------------------------------
void init_adc_mm(void) {
  //initialisation des tableaux
  /* valeur par defaut facultative,
  	permet d'avoir une moyenne proche
  	du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  unsigned int ini_adc4 = 0;// val defaut adc 4
  unsigned int ini_adc5 = 0;// val defaut adc 5
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    adc_hist[3][plus_ancien] = ini_adc4;
    adc_hist[4][plus_ancien] = ini_adc5;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  adc_mm[3] = ini_adc4;
  adc_mm[4] = ini_adc5;
}
//---------------------------------------------------------------------------
void adc_read() {
  read_adc(Pin12V, PinBattProc, PinBattUSB, PinBattSol, PinLum); // lecture des adc
}
//---------------------------------------------------------------------------
void read_adc(int pin1, int pin2, int pin3, int pin4, int pin5) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[5];
  for (byte i = 0; i < 5; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    if (i == 2)sample[i] = moyenneAnalogique(pin3);
    if (i == 3)sample[i] = moyenneAnalogique(pin4);
    if (i == 4)sample[i] = moyenneAnalogique(pin5);

    //calcul MoyenneMobile
    adc_mm[i] = adc_mm[i] + sample[i] - adc_hist[i][plus_ancien];

    //cette plus ancienne valeur n'est plus utile, on y stocke la plus récente
    adc_hist[i][plus_ancien] = sample[i];
  }
  plus_ancien ++;
  if (plus_ancien == nSample) { //gestion du buffer circulaire
    plus_ancien = 0;
  }
}
//--------------------------------------------------------------------------------//
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
void HomePage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Parametres</h3><br>";
  webpage += "<table align='center'>";
  webpage += "<tr>";
  webpage += "<td>Version</td>";
  webpage += "<td>";	webpage += ver;	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Id</td>";
  webpage += "<td>";	webpage += String(config.Idchar);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Debut Jour</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.DebutJour);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Fin Jour</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.FinJour);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Reset Routeur H1</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.AlimExt1);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Reset Routeur H2</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.AlimExt2);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Reset Routeur H3</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.AlimExt3);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Reset Routeur Off (s)</td>";
  webpage += "<td>";	webpage += config.TOffExt;	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Routeur Auto </td>";
  webpage += "<td>";	webpage += config.RouteurAuto;	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Dur&eacute;e Sir&egrave;ne (5-300s)</td>";
  webpage += "<td>";	webpage += config.Dsonn;	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>Alarme Batterie faible (%)</td>";
  webpage += "<td>";	webpage += config.BattLow;	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Anticipation WakeUp (s)</td>";
  webpage += "<td>";	webpage += String(config.anticip);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>Vitesse SlowBlinker (5-2000ms)</td>";
  webpage += "<td>";	webpage += String(config.SlowBlinker);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Vitesse FastBlinker (5-2000ms)</td>";
  webpage += "<td>";	webpage += String(config.FastBlinker);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>R&eacute;p&eacute;tition FastBlink (5-3000ms)</td>";
  webpage += "<td>";	webpage += String(config.FastRater);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>PWM Blanc (%)</td>";
  webpage += "<td>";	webpage += String(config.FBlcPWM);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>PWM Violet (%)</td>";
  webpage += "<td>";	webpage += String(config.FVltPWM);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Luminosit&eacute; Auto</td>";
  webpage += "<td>";	webpage += String(config.LumAuto);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Tempo r&eacute;p&eacute;tition Wake up Jour Circul&eacute; (s)</td>";
  webpage += "<td>";	webpage += String(config.RepeatWakeUp);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Auto F si O/S</td>";
  webpage += "<td>";	webpage += String(config.AutoF);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Tempo AutoF(100-36 000 s)</td>";
  webpage += "<td>";	webpage += String(config.TempoAutoF);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Entr&eacute;e 1</td>";
  webpage += "<td>";
  if (config.Ip1) {
    webpage += "Active";
  } else {
    webpage += "Inactive";
  }
  webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>SSID Routeur</td>";
  webpage += "<td>";	webpage += String(config.mySSID);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>PWD Routeur</td>";
  webpage += "<td>";	webpage += String(config.myPASSWORD);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>ftp Serveur</td>";
  webpage += "<td>";	webpage += String(config.ftpServeur);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp Port</td>";
  webpage += "<td>";	webpage += String(config.ftpPort);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp User</td>";
  webpage += "<td>";	webpage += String(config.ftpUser);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp Pass</td>";
  webpage += "<td>";	webpage += String(config.ftpPass);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>MQTT Serveur</td>";
  webpage += "<td>";	webpage += String(config.mqttServer);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>MQTT User</td>";
  webpage += "<td>";	webpage += String(config.mqttUserName);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>MQTT Pass</td>";
  webpage += "<td>";	webpage += String(config.mqttPass);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>MQTT Port</td>";
  webpage += "<td>";	webpage += String(config.mqttPort);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>MQTT receiveTopic</td>";
  webpage += "<td>";	webpage += String(config.receiveTopic);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>MQTT sendTopic</td>";
  webpage += "<td>";	webpage += String(config.sendTopic);	webpage += "</td>";
  webpage += "</tr>";
  
  webpage += "<tr>";
  webpage += "<td>MQTT permanentTopic</td>";
  webpage += "<td>";	webpage += String(config.permanentTopic);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "</table><br>";

  webpage += "<a href='/download'><button>Download</button></a>";
  webpage += "<a href='/upload'><button>Upload</button></a>";
  webpage += "<a href='/delete'><button>Delete</button></a>";
  webpage += "<a href='/dir'><button>Directory</button></a>";
  // webpage += "<a href='/Tel_list'><button>Tel_list</button></a>";
  webpage += "<a href='/cal'><button>Calendar</button></a>";
  webpage += "<a href='/LumLUT'><button>LumLUT</button></a>";
  webpage += "<a href='/reset'><button>Reset</button></a>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void LumLUTPage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Table Luminosit&eacute;</h3><br>";
  webpage += "<table align='center'>";
  webpage += "<tr>";
  webpage += "<th> Lum Ambiante % </th>";
  webpage += "<th> Lum Feux %</th>";
  webpage += "</tr>";
  for (int i = 0; i < 11; i++) {
    webpage += "<tr>";
    webpage += "<td>"; webpage += TableLum[i][0] ; webpage += "</td>";
    webpage += "<td>"; webpage += TableLum[i][1] ; webpage += "</td>";
    webpage += "</tr>";
  }
  webpage += "</table><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(config.mySSID);

  WiFi.begin(config.mySSID, config.myPASSWORD);
  byte cpt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Alarm.delay(500);
    Serial.print(".");
    if(cpt ++ > 15){
      MajLog("Auto","Wifi connect > 15");
      return;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
//---------------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  byte cpt = 0;
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(config.Idchar, config.mqttUserName, config.mqttPass)) {
      Serial.println("connected");
      // Subscribe
      mqttClient.subscribe(config.receiveTopic);//("TPCF_CV35/output");
      mqttClient.subscribe(config.permanentTopic);//("TPCF_CV35/permanent");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      Alarm.delay(5000);
    }
    cpt ++;
    if(cpt > 2){
      MajLog("Auto","MQTT > 2");
      return;
    }
  }
}
//---------------------------------------------------------------------------
void callback(char* topic, byte* buffer, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Sbidon = "";
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)buffer[i]);
    Sbidon += (char)buffer[i];
  }
  Sbidon.toCharArray(replybuffer, Sbidon.length() + 1);
  Serial.println();
  // Serial.print("topic:"),Serial.print(topic);
  // Serial.print(", receivetopic:"),Serial.println(config.receiveTopic);
  if (String(topic) == config.receiveTopic) { // nouveau message arrivé
    traite_sms(0);
  }
  if (String(topic) == config.permanentTopic) { // nouveau message arrivé
    // si jour noncircule, si permanentTopic = CIRCULE set FlagCircule = true
    Serial.print("permanent topic:"),Serial.println(Sbidon);
    if(Sbidon == "CIRCULE" || Sbidon == "NONCIRCULE"){
      //renvoyer "" sur permanentTopic pour eviter repetition
      char rep[2] = "";
      bool n = mqttClient.publish(config.permanentTopic, rep,true);
      if (n>0) {
        Serial.print(F("send mqtt OK:")),Serial.println(n);
      } else {
        Serial.print(F("send mqtt KO:")),Serial.println(n);
      }
      // traite sms CIRCULE/NONCIRCULE
      traite_sms(0);
    }
  }
}
//---------------------------------------------------------------------------
bool HeureEte() {
  // return true en été, false en hiver (1=dimanche)
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  return Hete;
}
//---------------------------------------------------------------------------
void Majheure(){
  static bool first = true;
  if (timeClient.update()){
    Serial.println ( "Adjust local clock" );
    unsigned long epoch = timeClient.getEpochTime();
    setTime(epoch);
    setTime(CE.toLocal(now()));
  }else{
    Serial.println ( "NTP Update not WORK!!" );
    if(first){
      setTime(1627804800);//2021/08/01 08:00:00
      first = false;
    }
  }
  Serial.print("timeset:"),Serial.println(timeStatus());    
  Serial.println(displayTime(0));
}
//---------------------------------------------------------------------------
/*-------- NTP code ----------*/
// const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
// byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// time_t getNtpTime(){
//   static bool first = true;
//   IPAddress ntpServerIP; // NTP server's ip address

//   while (Udp.parsePacket() > 0) ; // discard any previously received packets
//   Serial.println("Transmit NTP Request");
//   // get a random server from the pool
//   WiFi.hostByName(ntpServerName, ntpServerIP);
//   Serial.print(ntpServerName);
//   Serial.print(": ");
//   Serial.println(ntpServerIP);
//   sendNTPpacket(ntpServerIP);
//   static uint32_t beginWait = millis();
//   while (millis() - beginWait < 1500) {
//     int size = Udp.parsePacket();
//     if (size >= NTP_PACKET_SIZE) {
//       Serial.println("Receive NTP Response");
//       Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
//       unsigned long secsSince1900;
//       // convert four bytes starting at location 40 to a long integer
//       secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
//       secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
//       secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
//       secsSince1900 |= (unsigned long)packetBuffer[43];
//       // bug : date erronée 07/02/2036 07:28:16
//       // si secsSince1900 = 0
//       Serial.print("timestamp:"),Serial.println(secsSince1900);
//       if(secsSince1900 == 0){
//         first = false;
//         return 0;//idem not sync
//       }
//       first = false;
//       return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
//     }
//     Alarm.delay(1);
//   }
//   Serial.println("No NTP Response :-(");
//   if(first){
//     first = false;
//     return 1627804800;//2021/08/01 08:00:00
//   } else {
//     return 0; // return 0 if unable to get the time
//   }
//   return 0; // pas utile mais au cas ou!,return 0 if unable to get the time
// }
//---------------------------------------------------------------------------
// void sendNTPpacket(IPAddress &address){
//   // send an NTP request to the time server at the given address
//   // set all bytes in the buffer to 0
//   memset(packetBuffer, 0, NTP_PACKET_SIZE);
//   // Initialize values needed to form NTP request
//   // (see URL above for details on the packets)
//   packetBuffer[0] = 0b11100011;   // LI, Version, Mode
//   packetBuffer[1] = 0;     // Stratum, or type of clock
//   packetBuffer[2] = 6;     // Polling Interval
//   packetBuffer[3] = 0xEC;  // Peer Clock Precision
//   // 8 bytes of zero for Root Delay & Root Dispersion
//   packetBuffer[12] = 49;
//   packetBuffer[13] = 0x4E;
//   packetBuffer[14] = 49;
//   packetBuffer[15] = 52;
//   // all NTP fields have been given values, now
//   // you can send a packet requesting a timestamp:
//   Udp.beginPacket(address, 123); //NTP requests are to port 123
//   Udp.write(packetBuffer, NTP_PACKET_SIZE);
//   Udp.endPacket();
// }
//---------------------------------------------------------------------------
void CalendarPage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Calendrier</h3><br>";
  webpage += "<table align='center'>";

  for (int m = 1; m < 13; m ++) {
    webpage += "<tr>";
    webpage += "<td>"; webpage += Mois[m]; webpage += "</td>";
    for (int j = 1; j < 32; j++) {
      webpage += "<td>";	webpage += calendrier[m][j];	webpage += "</td>";
      if (j % 5 == 0)webpage += "<td> </td>";
    }
    webpage += "</tr>";
  }

  webpage += "</table><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {
  append_page_header();
  webpage += "<h3>Select File to Upload</h3>";
  webpage += "<FORM action='/fupload' method='post' enctype='multipart/form-data'>";
  webpage += "<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>";
  webpage += "<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  server.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += "<h3>File was successfully uploaded</h3>";
      webpage += "<h2>Uploaded File Name: "; webpage += uploadfile.filename + "</h2>";
      webpage += "<h2>File Size: "; webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);
      OuvrirCalendrier();
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += "<h3 class='rcorners_m'>SPIFFS Contents</h3><br>";
      webpage += "<table align='center'>";
      webpage += "<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>";
      printDirectory("/", 0);
      webpage += "</table>";
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += "<h3>No Files Found</h3>";
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   // Stop is needed because no content length was sent
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      webpage += "<tr><td>" + String(file.name()) + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      webpage += "<td>" + file_size(file.size()) + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file
  if (SPIFFS_present) {
    SendHTML_Header();
    File dataFile = SPIFFS.open("/" + filename, "r"); // Now read data from SPIFFS Card
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println("File deleted successfully");
        webpage += "<h3>File '" + filename + "' has been erased</h3>";
        webpage += "<a href='/delete'>[Back]</a><br><br>";
      }
      else
      {
        webpage += "<h3>File was not deleted - error</h3>";
        webpage += "<a href='delete'>[Back]</a><br><br>";
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  webpage += "<h3>"; webpage += heading1 + "</h3>";
  webpage += "<FORM action='/"; webpage += command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += "<input type='text' name='"; webpage += arg_calling_name; webpage += "' value=''><br>";
  webpage += "<type='submit' name='"; webpage += arg_calling_name; webpage += "' value=''><br><br>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  webpage += "<h3>No SPIFFS Card present</h3>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += "<h3>File does not exist</h3>";
  webpage += "<a href='/"; webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += "<h3>Could Not Create Uploaded File (write-protected?)</h3>";
  webpage += "<a href='/"; webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleDateTime() { // getion Date et heure page web
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void AlimExt(){
  if(config.RouteurAuto){
    digitalWrite(PinAlimExt12,LOW); // coupure, Alimentation du relais externe
    Alarm.delay(config.TOffExt*1000);
    digitalWrite(PinAlimExt12,HIGH); // retour normal, relais externe repos
    MajLog("Auto","RouteurAutoReset");
  } else {
    digitalWrite(PinAlimExt12,HIGH); // confirmation normal
  }
}
//---------------------------------------------------------------------------
void VerifCdeFBlc(){
  // si F != 2(Ouvert) on mesure entree PinChckFblc si == 0 Alarme
  // on mesure accumulation sur 2 secondes si ratio < 35% 
  static unsigned long tmesure = millis();
  if (tmesure > millis()) tmesure = millis();
  static int compteurmesureres = 0;
  static int accumesureres = 0;
  int periodemesures = 2000;
  if(Feux != 2){
    if(millis()- tmesure > periodemesures){// periode mesure > periodemesures
      if(compteurmesureres > 1200){
        // pour eviter fausses alarmes quand proc occupé par ailleurs
        Serial.print("tmesure:"),Serial.print(tmesure);
        Serial.print(" Cpt Cde FBLc:"),Serial.print(compteurmesureres);
        Serial.print(", accu:"),Serial.print(accumesureres);
        Serial.print(", %:"),Serial.println((float)accumesureres/compteurmesureres);
        if((float)accumesureres/compteurmesureres < .35 ){// .5 = M, .75 = S
          Serial.println("Alarme Cde Feux Blanc");
          FlagAlarmeCdeFBlc = true;
          if(!FlagLastAlarmeCdeFBlc){ // si premiere fois
            Extinction();
          }
        }
      }
      tmesure = millis();
      compteurmesureres = 0;
      accumesureres = 0;
    }
    accumesureres += digitalRead(PinChckFblc);
    compteurmesureres ++;
  }
}
//---------------------------------------------------------------------------
void EffaceAlaCdeFBlc(){
  // efface l'alarme Cde Feu Blanc, pour rendre de nouveau
  // operationnel l'extinction en cas de nouveau probleme
  FlagAlarmeCdeFBlc = false;
  FlagLastAlarmeCdeFBlc = false;
}
//---------------------------------------------------------------------------
void ArretSonnerie() {
  Serial.print(F("Fin tempo Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(PinSirene, HIGH);	// Arret Sonnerie
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
}
//---------------------------------------------------------------------------
void arret_Alarm(){
  Alarm.disable(DebutJour);
  Alarm.disable(FinJour);
  Alarm.disable(loopPrincipale);
  Alarm.disable(Alim_Ext_1);
  Alarm.disable(Alim_Ext_2);
  Alarm.disable(Alim_Ext_3);
}
//---------------------------------------------------------------------------
void lancement_Alarm(){
  Alarm.enable(DebutJour);
  Alarm.enable(FinJour);
  Alarm.enable(loopPrincipale);
  if(config.RouteurAuto){
    Alarm.enable(Alim_Ext_1);
    Alarm.enable(Alim_Ext_2);
    Alarm.enable(Alim_Ext_3);
  }
}
//---------------------------------------------------------------------------
void demandereset(){
  SendHTML_Header();
  Serial.println("Reset Hard");
  FlagReset = true;
  SendHTML_Content();
}
//---------------------------------------------------------------------------
void verifAlarmeBattExt(){
//*************** Verification Alarme Batterie Externe ***************
  static unsigned long startE1 = millis();
  static bool FlagStartE1 = true;
  // lecture entree Ip1 (IRQ_Cpt_Ip1)
  if (config.Ip1 && digitalRead(PinIp1) == 0 && !FlagAlarmeBattExt){
    if(FlagStartE1){
      FlagStartE1 = false;
      startE1 = millis();
    }
    if(millis() - startE1 > 5000){ // temporisation lecture
      MajLog("Auto", "Alarme Batt Externe");// taquet Ouvert
      FlagStartE1 = true;
      FlagAlarmeBattExt = true;
    }
  } else if(config.Ip1 && digitalRead(PinIp1) == 1 && FlagAlarmeBattExt){
    if(FlagStartE1){
      FlagStartE1 = false;
      startE1 = millis();
    }
    if(millis() - startE1 > 5000){ // temporisation lecture
      MajLog("Auto", "Fin Alarme Batt Externe");// taquet Fermé
      FlagStartE1 = true;
      FlagAlarmeBattExt = false;
    }
  }
  if(!FlagStartE1 && (millis() - startE1 > 7000)){ // reset tempo lecture
    FlagStartE1 = true;
    // Serial.print("FlagStartE1:"),Serial.println(FlagStartE1);
  }
}
//---------------------------------------------------------------------------
void record_status(){
  File f = SPIFFS.open(filestatus,"w+");
  f.println(String(Feux));
  f.close();
}
//---------------------------------------------------------------------------
int read_status(){
  File f = SPIFFS.open(filestatus,"r");
  Sbidon = f.readStringUntil('\n');
  f.close();
  if(Sbidon.toInt() >= 0 && Sbidon.toInt() < 8){
    return Sbidon.toInt();
  }
  return 0;
}
//---------------------------------------------------------------------------
/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  while (Serial.available() > 0) { // if
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

void interpretemessage(String demande) {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1);
    bidons.toCharArray(replybuffer, bidons.length() + 1);
    traite_sms(99);//	traitement SMS en mode test local
  }
}
//---------------------------------------------------------------------------
