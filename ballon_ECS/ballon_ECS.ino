/*
   Gestion d'un ballon d'eau chaude, de la chaudière et de la circulation
   d'eau à partir de 2 capteurs de température LM35.

   (c) 2019 Valéry MARZLIN et Stéphane TRONEL
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define I2C_LCD 0x27
//#define I2C_LCD 0x3f

#define CODE_VERSION  "v1.5"

/*
   Définir la variable DEBUG_SERIAL pour activer les traces dans la console
*/
//#define DEBUG_SERIAL

/*
   Définir TEMP_NEGATIVE si on veut que les capteurs LM35 soient capable de lire une
   température négative. Voir note ci-dessous.
*/
#define TEMP_NEGATIVE

/*
   Définition du brochage utilisé sur l'Arduino UNO
   ATTENTION:
    - pin A4 et A5 sont réservées à l'I2C.
    - pin 1 et 2 sont réservées à l'USB (RX/TX)

    Matériel à ajouter autour de l'Arduino:
 *  * Encodeur rotatif sur PIN_ENC_* (CLK et DIR sont parfois notés A et B)
      - VCC et GND connectés sur VCC et GND de l'Arduino
      - CLK et DIR (parfois notés A et B) connecté sur PIN_ENC_CLK et PIN_ENC_DIR
      - le switch connecté sur PIN_ENC_SW
 *  * Platine de relais sur PIN_ALARME, PIN_CURCUL et PIN_COMMANDE (en plus de VCC et GND)
 *  * 2 capteurs LM35 avec:
      - VCC connecté au VCC de l'Arduino
      - Vout connecté sur PIN_CAP
      - GND du LM35 connecté:
        - soit sur le GND de l'Arduino, dans ce cas les mesures de températures négatives
          ou proche de 0 ne sont pas possible, et il ne faut pas définir la variable
          TEMP_NEGATIVE
        - soit sur la broche PIN_REF de l'Arduino ce qui permet de mesurer les températures
          légèrement négatives. Dans ce cas, il faut:
          - relier PIN_REF de l'Arduino sur la cathode d'une diode (par exemple 1N400x
            ou 1N4148) et l'anode sur GND de l'Arduino
          - définir la variable TEMP_NEGATIVE
 *  * un écran LCD 20x4 I2C connecté sur le bus I²C de l'Arduino. Une résistance de pull-up
      entre 1kΩ et 10kΩ est conseillées entre SDA et VCC et une autre de même valeur entre
      SCL et VCC.
*/
#define PIN_ENC_CLK 2
#define PIN_ENC_DIR 3
#define PIN_ENC_SW 4
#define PIN_ALARME 7
#define PIN_CIRCUL 8
#define PIN_COMMANDE 9
#define PIN_CAPHAUT A0
#define PIN_CAPBAS A1

#ifdef TEMP_NEGATIVE
#define PIN_REF A2
#endif

/*
   Réglages concernant l'affichage LCD I2C
*/
LiquidCrystal_I2C lcd(I2C_LCD, 20, 4);

/*
   Autres réglages
*/
#define LOOP_DELAY 100   // Délai en fin de boucle (en ms). Minimum recommandé 100ms.
#define TIMER 50         // Délai d'inactivité (en nombre de boucle) au delà duquel 
// on revient sur le menu d'origine
#define ADDR_CONF 0      // Adresse EEPROM contenant le début de la configuration
#define AREF 1100L       // Tension de référence pour les ADC internes de l'Arduino (en mV)

#define RELAY_ON  LOW    // Défini la valeur avec laquelle le relais est en position travail
#define RELAY_OFF HIGH   // Défini la valeur avec laquelle le relais est en position repos

#define TEMP_MIN 3       // Température minimale (défaut =  3,0°C)
#define TEMP_MAX 88      // Température maximale (defaut = 88,0°C)
#define TEMP_BAS 40      // Seuil pour la sonde basse (défaut = 40,0°C)
#define TEMP_HAU 60      // Seuil pour la sonde haute (défaut = 60,0°C)
#define TEMP_CIR 25      // Seuil pour la circulation (défaut = 25,0°C)

/*
   Définition des variables globales

   Note: toutes les températures sont en dizièmes de degrés Celcius
*/
// Configuration
short tempMin = TEMP_MIN * 10;    // Température minimale
short tempMax = TEMP_MAX * 10;    // Température maximale
short seuilBas = TEMP_BAS * 10;   // Seuil pour la sonde basse
short seuilHaut = TEMP_HAU * 10;  // Seuil pour la sonde haute
short seuilCircu = TEMP_CIR * 10; // Seuil pour la circulation
// Mesures
short tempHaut;                   // Température de la sonde haute (moyenne de 10 mesures)
short tempBas;                    // Température de la sonde basse (moyenne de 10 menures)
// Calibration
char tempOffsetH = 0;             // Offset de température pour la sonde haute
char tempOffsetB = 0;             // Offset de température pour la sonde basse

bool chauffage = false;   // Indique si la commande de chauffage doit être activée
bool alarme = false;      // Indique si l'alarme est activée
bool alarmAck = false;    // Indique si l'alarme a été aquitée
bool circulation = false; // Indique si le circulateur est activé
bool btnPressed = false;  // Indique si le bouton est enfoncé (gestion de l'anti-rebond)
bool modified = false;    // Indique si des réglages ont été modifiés et doivent être
// sauvegardés dans l'EEPROM.
bool alrmTBasse = false;  // Indique alarme de température trop basse
bool alrmTHaute = false;  // Indique alarme de température trop haute
byte selected = 0;        // Indique le menu sélectionné
volatile char rot = 0;    // Indique si l'encodeur a été tourné
unsigned int idle = 0;    // Indique le temps d'inactivité de l'utilisateur

short tH[10];             // Tableau contenant les 10 dernières mesures de la sonde haute
short tB[10];             // Tableau contenant les 10 dernières mesures de la sonde basse
byte tPos = 0;            // Position courante dans les tableaux de mesures

/*
   Caractères spéciaux pour l'écran LCD
   Il est possible de définir 8 caractères
*/
byte cloche[8] = {0x4, 0xe, 0xe, 0xe, 0x1f, 0x0, 0x4};
byte degres[8] = {0x6, 0x9, 0x9, 0x6, 0x0, 0x0, 0x0};
byte flechg[8] = {0x1, 0x3, 0x7, 0xf, 0x7, 0x3, 0x1};
byte flechd[8] = {0x10, 0x18, 0x1c, 0x1e, 0x1c, 0x18, 0x10};
byte chauff[8] = {0x9, 0x12, 0x12, 0x9, 0x9, 0x12, 0x1f};
byte chatop[8] = {0x0, 0xe, 0x11, 0x11, 0x11, 0x11, 0x11};
byte chabot[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0xe, 0x0};
byte circul[8] = {0x0, 0xe, 0x17, 0x11, 0x17, 0xe, 0x0};

#define CHAR_DEGRES 0
#define CHAR_CLOCHE 1
#define CHAR_CHAUFF 2
#define CHAR_CIRCUL 3
#define CHAR_FLECHD 4
#define CHAR_FLECHG 5
#define CHAR_CHATOP 6
#define CHAR_CHABOT 7

/*
   Définition des menus
*/
#define SIZE_MENU0 5
#define SIZE_MENU1 3
#define SIZE_MENU2 3
const char menu0[SIZE_MENU0][21] = {
  " RAZ Config         ",
  " RAZ Calibration    ",
  " Calibration sondes ",
  " Reglage limites    ",
  " Sortie             "
};
const char menu1[SIZE_MENU1][21] = {
  "  Sonde Haute       ",
  "  Sonde Basse=Haute ",
  "  Retour            "
};
const char menu2[SIZE_MENU2][21] = {
  "  Temp. Minimale    ",
  "  Temp. Maximale    ",
  "  Retour            "
};
const char emptyLine[] = "                    ";

/***********************************************
 *                                             *
     Programme d'initialisation de l'Arduino
 *                                             *
 ***********************************************/
void setup() {
  // Défini les ports de sortie et les met à leur valeur par défaut
  pinMode(PIN_COMMANDE, OUTPUT);
  pinMode(PIN_ALARME, OUTPUT);
  pinMode(PIN_CIRCUL, OUTPUT);
  digitalWrite(PIN_COMMANDE, RELAY_OFF);
  digitalWrite(PIN_ALARME, RELAY_OFF);
  digitalWrite(PIN_CIRCUL, RELAY_OFF);

  // Défini les ports d'entrée
#ifdef TEMP_NEGATIVE
  pinMode(PIN_REF, INPUT);
#endif
  pinMode(PIN_CAPBAS, INPUT);
  pinMode(PIN_CAPHAUT, INPUT);

  // Indique d'utiliser la référence de tension interne (1,1V) pour les entrées analogiques
  analogReference(INTERNAL);

  // Active les résistances de pull-up intégrées à l'Arduino pour l'encodeur rotatif
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DIR, INPUT_PULLUP);

  // Initialise l'écran LCD
#ifdef DEBUG_SERIAL
  Serial.begin(9600);
  Serial.print("Initialisation LCD [0x");
  Serial.print(I2C_LCD, HEX);
  Serial.print("]...");
#endif
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.noAutoscroll();
  lcd.print(" GESTION BALLON");
  lcd.setCursor(1, 1);
  lcd.print(CODE_VERSION);
  lcd.createChar(CHAR_DEGRES, degres);
  lcd.createChar(CHAR_CLOCHE, cloche);
  lcd.createChar(CHAR_FLECHG, flechg);
  lcd.createChar(CHAR_FLECHD, flechd);
  lcd.createChar(CHAR_CHAUFF, chauff);
  lcd.createChar(CHAR_CHATOP, chatop);
  lcd.createChar(CHAR_CHABOT, chabot);
  lcd.createChar(CHAR_CIRCUL, circul);
#ifdef DEBUG_SERIAL
  Serial.println("Ok");
#endif

  // Préchargement des tableaux de mesures
  lcd.setCursor(0, 2);
  lcd.write('[');
  lcd.setCursor(11, 2);
  lcd.write(']');
  lcd.setCursor(1, 2);
  tPos = 10;
  while (tPos-- > 0)
  {
    delay(50);
    lcd.write('#');
    tB[tPos] = readTemp(PIN_CAPBAS);
    tH[tPos] = readTemp(PIN_CAPHAUT);
    delay(50); // delay minimum entre 2 lectures sur les sondes de température
  }
  delay(500);
  lcd.clear();

  // Arme l'interruption pour détecter les rotations de l'encodeur.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), routineInterruption, FALLING);

  // Vérifie si le bouton de l'encodeur est enfoncé au reset.
  // Si oui, affiche le menu de configuration dès qu'il est relâché
  bool configure = false;
  while (encoderPressed())
  {
    configure = true;
    delay(25);
  }
  if (configure)
  {
    configuration();
  }
  lcd.clear();

  loadEEPROM(ADDR_CONF, 3);
  sanityCheck();
#ifdef DEBUG_SERIAL
  Serial.println("Let's go! :)");
#endif
}

/************************************************
 *                                              *
                Boucle principale
 *                                              *
 ************************************************/
void loop() {
  if (tPos == 0)
  { // Relève la températre tous les 10 "tours" (on ne peut faire d'une
    // mesure de températures avec les sondes LM35 toutes les 100ms minimum)

    // Calcul de la moyenne des 10 derniers relevés de température
    long tempB = 0;
    long tempH = 0;
    for (byte i = 9; i > 0; i--)
    {
      tempB += tB[i];
      tempH += tH[i];
    }

    tempBas = (short)(tempB / 10L);
    tempHaut = (short)(tempH / 10L);
#ifdef DEBUG_SERIAL
    /*
        Serial.print("Températures (H/B):");
        Serial.print((float)tempBas/10.0);
        Serial.print('/');
        Serial.println((float)tempHaut/10.0);
    */
#endif

    /*
       Gestion de l'alarme

       Alarme active si (tempBas ou tempHaut) < tempMin ou si (tempBas ou tempHaut) > tempMax
       L'alarme est déscativée par n'importe quelle action manuelle sur l'encodeur (rotation
       ou pression). Le symbole d'alarme reste affiché sur l'écran tant qu'elle est en cours,
       qu'elle soit acquitée ou non.
    */
    alrmTBasse = min(tempBas, tempHaut) < tempMin;
    alrmTHaute = max(tempBas, tempHaut) > tempMax;
    alarme = alrmTBasse || alrmTHaute;
    if (!alarme)
    {
      alarmAck = false;
    }
    digitalWrite(PIN_ALARME, (alarme && !alarmAck) ? RELAY_ON : RELAY_OFF);

    /*
       Gestion du chauffage en fonction des températures des sondes

       Allumage du chauffage si tempHaut < seuilHaut
       Arrêt  du  chauffage  si tempBas  > seuilBas
       Quoi qu'il arrive, du plus prioritaire au moins prioritaire:
        1. arrêt du chauffage si alarme température trop haute
        2. allumage du chauffage si alarme température trop basse
    */
    if (tempHaut < seuilHaut)
    {
      chauffage = true;
    }
    if (tempBas > seuilBas)
    {
      chauffage = false;
    }
    if (alrmTBasse)
    {
      chauffage = true;
    }
    if (alrmTHaute)
    {
      chauffage = false;
    }
    digitalWrite(PIN_COMMANDE, chauffage ? RELAY_ON : RELAY_OFF);

    /*
       Gestion de la circulation

       Circulation active si tempHaut >= seuilCircu.
       @todo Prévoir un hystérésis?
    */
    circulation = (tempHaut > seuilCircu);
    digitalWrite(PIN_CIRCUL, circulation ? RELAY_ON : RELAY_OFF);
  }

  tPos++;
  tPos %= 10;
  // Relève de la température
  tB[tPos] = readTemp(PIN_CAPBAS);
  tH[tPos] = readTemp(PIN_CAPHAUT);

  // Vérification si action de l'utilisateur --------------------
  // Gestion du bouton de l'encodeur
  if (encoderPressed())
  { // bouton enfoncé
    if (!btnPressed)
    { // Gère l'anti-rebond
      alarmAck = true;
      selected = ++selected % 4;
      btnPressed = true;
    }
    lcd.backlight();
    idle = TIMER;
  }
  else
  {
    btnPressed = false;
  }

  // Gestion de la rotation de l'encodeur
  if (rot)
  {
    switch (selected)
    {
      case 1: // le bouton modifie le seuil haut
        seuilHaut = constrain(seuilHaut + rot, tempMin, tempMax);
        // seuilBas = min(seuilBas, seuilHaut);
        modified = true;
        break;
      case 2: // le bouton modifie le seuil bas
        seuilBas = constrain(seuilBas + rot, tempMin, tempMax);
        // seuilHaut = max(seuilHaut, seuilBas);
        modified = true;
        break;
      case 3: // le bouton modifie le seuil de circulation
        seuilCircu = constrain(seuilCircu + rot, tempMin, tempMax);
        modified = true;
        break;
    }
    rot = 0;
    alarmAck = true;
    lcd.backlight();
    idle = TIMER;
  }

  // Mise à jour de l'écran LCD
  lcd.home();
  lcd.write(CHAR_CHATOP);
  lcd.write(CHAR_FLECHG);
  printTemp(tempHaut);
  lcd.print(' ');
  printTemp(seuilHaut);
  lcd.print((selected == 1) ? (char)CHAR_FLECHG : ' ');

  lcd.setCursor(0, 1);
  lcd.write(CHAR_CHABOT);
  lcd.write(CHAR_FLECHG);
  printTemp(tempBas);
  lcd.print(' ');
  printTemp(seuilBas);
  lcd.print((selected == 2) ? (char)CHAR_FLECHG : ' ');

  lcd.setCursor(10, 2);
  printTemp(seuilCircu);
  lcd.print((selected == 3) ? (char)CHAR_FLECHG : ' ');

  lcd.setCursor(0, 3);
  if (alrmTBasse)
  {
    lcd.print("! Temp. trop basse !");
  }
  else if (alrmTHaute)
  {
    lcd.print("! Temp. trop haute !");
  }
  else
  {
    lcd.print(emptyLine);
  }

  lcd.setCursor(19, 0);
  lcd.write(alarme ? CHAR_CLOCHE : ' ');
  lcd.setCursor(19, 1);
  lcd.write(chauffage ? CHAR_CHAUFF : ' ');
  lcd.setCursor(19, 2);
  lcd.write(circulation ? CHAR_CIRCUL : ' ');

  // temporisation
  delay(LOOP_DELAY);

  // Si personne n'a touché au bouton pendant le timeout, revenir au menu initial.
  if (idle > 0)
  {
    idle--;
    if (idle <= 0)
    {
      selected = 0;
      if (modified)
      {
        saveEEPROM(ADDR_CONF, 1);
        modified = false;
      }
    }
  }
}

/*
   Lit la température depuis une sonde et la retourne en dizièmes de degrés Celcius
*/
short readTemp(byte pinData)
{
  long a = analogRead(pinData);
#ifdef TEMP_NEGATIVE
  long r = analogRead(PIN_REF);
  long t = (AREF * (a - r)) >> 10;
#else
  long t = (AREF * a) >> 10;
#endif

  if (pinData == PIN_CAPHAUT)
  {
    t += tempOffsetH;
  }
  else
  {
    t += tempOffsetB;
  }
  return (short)t;
}

/*
   Affiche la température (en dizième de degrés) sur l'écran LCD à l'emplacement du curseur
   La taille est fixe: 7 caractères. Gère les températures de -99,9°C à 999.9°C.
   0123456
   -99.9°C
*/
void printTemp(short temp)
{
  char t[7];
  t[6] = 'C';
  t[5] = (char)CHAR_DEGRES;
  if (temp < -999)
  {
    t[4] = '-';
    for (byte i = 2; i > 0; i--)
    {
      t[i] = '-';
    }
  }
  else if (temp > 9999)
  {
    t[4] = '+';
    for (byte i = 2; i > 0; i--)
    {
      t[i] = '+';
    }
  }
  else
  {
    bool neg = (temp < 0);
    temp = abs(temp);
    t[4] = '0' + (temp % 10);
    temp /= 10;
    t[2] = '0' + (temp % 10);
    temp /= 10;
    t[1] = '0' + (temp % 10);
    temp /= 10;
    t[0] = '0' + (temp % 10);
    if (t[0] == '0')
    {
      t[0] = ' ';
    }
    if ((t[1] == '0') && (t[0] == ' '))
    {
      t[1] = ' ';
    }
    if (neg)
    {
      t[0] = '-';
    }
  }
  t[3] = ',';
  for (byte i = 0; i < sizeof(t); i++)
  {
    lcd.write(t[i]);
  }
}

/******************************************************
 *                                                    *
                   GESTION EEPROM
 *                                                    *
 ******************************************************/
/*
   Carte de la mémoire (l'octet de poids fort en premier):
 *  * Configuration
   0x00 0x01: température min
   0x02 0x03: température max
   0x04 0x05: seuil bas
   0x06 0x07: seuil haut
   0x08 0x09: seuil circulateur
 *  * Calibration
   0x0a: offset capteur haut
   0x0b: offset capteur bas
   0x0c 0x0d: 0xFFFF (réservé)
 *  * CRC
   0x0e 0x0f: CRC-16 bits
*/
/*
   Sauvegarde la configuration dans l'EEPROM
*/
void saveEEPROM(unsigned int addr, byte action)
{
  sanityCheck();
  unsigned int addr0 = addr;
#ifdef DEBUG_SERIAL
  displayConfig();
  Serial.print("Ecriture EEPROM ");
  Serial.println(action, BIN);
  displayEEPROM(addr0);
#endif
  lcd.clear();
  lcd.home();
  lcd.print("Sauvegarde en cours");
  lcd.setCursor(0, 1);

  word crc = crcCalc(tempMin, tempMax, seuilBas, seuilHaut, seuilCircu, tempOffsetH, tempOffsetB);
  if (action & 1)
  {
#ifdef DEBUG_SERIAL
    Serial.print(" >>Config 0x");
    Serial.println(addr, HEX);
#endif
    EEPROM.update(addr++, highByte(tempMin));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(tempMin));
    lcd.write('.');
    EEPROM.update(addr++, highByte(tempMax));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(tempMax));
    lcd.write('.');
    EEPROM.update(addr++, highByte(seuilBas));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(seuilBas));
    lcd.write('.');
    EEPROM.update(addr++, highByte(seuilHaut));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(seuilHaut));
    lcd.write('.');
    EEPROM.update(addr++, highByte(seuilCircu));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(seuilCircu));
    lcd.write('.');
  }
  if (action & 2)
  {
    addr = addr0 + 10;
#ifdef DEBUG_SERIAL
    Serial.print(" >>Calibration 0x");
    Serial.println(addr, HEX);
#endif
    EEPROM.update(addr++, tempOffsetH);
    lcd.write('.');
    EEPROM.update(addr++, tempOffsetB);
    lcd.write('.');
    EEPROM.update(addr++, 0xff);
    lcd.write('.');
    EEPROM.update(addr++, 0xff);
    lcd.write('.');
  }
  if (action & 3)
  {
    addr = addr0 + 14;
#ifdef DEBUG_SERIAL
    Serial.print(" >>CRC 0x");
    Serial.println(addr, HEX);
#endif
    EEPROM.update(addr++, highByte(crc));
    lcd.write('.');
    EEPROM.update(addr++, lowByte(crc));
    lcd.write('.');
  }
#ifdef DEBUG_SERIAL
  displayEEPROM(addr0);
#endif
  lcd.setCursor(0, 2);
  lcd.print("OK");
  delay(500);
  lcd.clear();
}

#ifdef DEBUG_SERIAL
void displayEEPROM(unsigned int addr)
{
  Serial.print("0x00: ");
  for (byte i = 0; i < 8; i++)
  {
    byte value = EEPROM.read(addr++);
    Serial.print(value >> 4, HEX);
    Serial.print(value % 16, HEX);
  }
  Serial.println();
  Serial.print("0x08: ");
  for (byte i = 0; i < 8; i++)
  {
    byte value = EEPROM.read(addr++);
    Serial.print(value >> 4, HEX);
    Serial.print(value % 16, HEX);
  }
  Serial.println();
}

void displayConfig()
{
  Serial.print("tempMin=0x");
  Serial.print(tempMin, HEX);
  Serial.print("=");
  Serial.println(tempMin);
  Serial.print("tempMax=0x");
  Serial.print(tempMax, HEX);
  Serial.print("=");
  Serial.println(tempMax);
  Serial.print("seuilBas=0x");
  Serial.print(seuilBas, HEX);
  Serial.print("=");
  Serial.println(seuilBas);
  Serial.print("seuilHaut=0x");
  Serial.print(seuilHaut, HEX);
  Serial.print("=");
  Serial.println(seuilHaut);
  Serial.print("seuilCircu=0x");
  Serial.print(seuilCircu, HEX);
  Serial.print("=");
  Serial.println(seuilCircu);

  Serial.print("tempOffsetH=0x");
  Serial.print(tempOffsetH, HEX);
  Serial.print("=");
  Serial.println((int)tempOffsetH);
  Serial.print("tempOffsetB=0x");
  Serial.print(tempOffsetB, HEX);
  Serial.print("=");
  Serial.println((int)tempOffsetB);
}
#endif

/*
   Lit la configuration depuis l'EEPROM.
   Retourne true si la configuration lue n'est pas valide.
*/
bool loadEEPROM(unsigned int addr, byte action)
{
  unsigned int addr0 = addr;
  short tMin    = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  short tMax    = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  short sBas    = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  short sHaut   = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  short sCircu  = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  char tOffsetH = (char)EEPROM.read(addr++);
  char tOffsetB = (char)EEPROM.read(addr++);
  addr++;
  addr++;
  word crc      = (((word)EEPROM.read(addr++)) << 8) + (word)EEPROM.read(addr++);
  word crc2     = crcCalc(tMin, tMax, sBas, sHaut, sCircu, tOffsetH, tOffsetB);

#ifdef DEBUG_SERIAL
  displayConfig();
  Serial.print("Lecture EEPROM ");
  Serial.println(action, BIN);
  displayEEPROM(addr0);
  Serial.print("CRC (EEPROM/calc):");
  Serial.print(crc, HEX);
  Serial.print("/");
  Serial.print(crc2, HEX);
  Serial.println((crc == crc2) ? " OK" : " KO!!");
#endif
  if ((tMin < tMax) && (crc == crc2))
  {
    if (action & 1)
    {
      tempMin = tMin;
      tempMax = tMax;
      seuilBas = sBas;
      seuilHaut = sHaut;
      seuilCircu = sCircu;
    }
    if (action & 2)
    {
      tempOffsetH = tOffsetH;
      tempOffsetB = tOffsetB;
    }
    sanityCheck();
    modified = false;
#ifdef DEBUG_SERIAL
    displayConfig();
#endif
    return false;
  }
  else
  {
#ifdef DEBUG_SERIAL
    displayConfig();
#endif
    return true;
  }
}

// Calcule la CRC-16
word crcCalc(short a, short b, short c, short d, short e, char f, char g)
{
  byte data[14];
  data[0] = highByte(a);
  data[1] = lowByte(a);
  data[2] = highByte(b);
  data[3] = lowByte(b);
  data[4] = highByte(c);
  data[5] = lowByte(c);
  data[6] = highByte(d);
  data[7] = lowByte(d);
  data[8] = highByte(e);
  data[9] = lowByte(e);
  data[10] = f;
  data[11] = g;
  data[12] = 0xff;
  data[13] = 0xff;
  word crc = 0xFFFF;

  for (byte i = 0; i < sizeof(data); ++i) {
    word dbyte = data[i];
    crc ^= dbyte << 8;

    for (byte j = 0; j < 8; ++j) {
      word mix = crc & 0x8000;
      crc = (crc << 1);
      if (mix)
      {
        crc = crc ^ 0x1021;
      }
    }
  }
  return crc;
}

/*
   Menu de configuration

   Arborescence du menu (la valeur entre [] correspond au menuItem correspondant)
   - RAZ Config [1]
     - NON [10]
     - OUI [11] ==> razConfig()
   - RAZ Calibration [2]
     - NON [20]
     - OUI [21] ==> razCalibration()
   - Calibration sondes [3]
     - temp+/- [30]
       - adjust [300] ==> ajuste tOffsetH
     - T1=T2 [31]
       - NON [310]
       - OUI [311] ==> ajuste tOffsetB
     - Retour [32]
   - Réglage limites [4]
     - lim- [40]
       - adjust [400] ==> ajuste tempMin
     - lim+ [41]
       - adjust [410] ==> ajuste tempMax
     - Retour [42]
   - Sortie [5]
*/
void configuration()
{
  int menuItem = 1;
  char level = 0;
  bool configuration = true;
  bool caliChanged = false, confChanged = false;
  if (loadEEPROM(ADDR_CONF, 3))
  {
    caliChanged = true;
    confChanged = true;
  }
  bool needRefresh = true;
  bool needRedraw = true;
  lcd.clear();
  lcd.home();
  lcd.print("** CONFIGURATION **");
  char item;
  while (configuration)
  {
    if (needRedraw || needRefresh)
    {
#ifdef DEBUG_SERIAL
      /*
            Serial.print((int)level);
            Serial.print("[");
            Serial.print(menuItem);
            Serial.println("]");
      */
#endif

      switch (level)
      {
        case 0:
          for (byte i = 1; i < 4; i++)
          {
            item = menuItem + i - 3;
            lcd.setCursor(0, i);
            if ((item >= 0) && (item < SIZE_MENU0))
            {
              lcd.print(menu0[item]);
            }
            else
            {
              lcd.print(emptyLine);
            }
          }
          lcd.setCursor(0, 2);
          lcd.write((char)CHAR_FLECHD);
          break;
        case 1:
          if ((menuItem / 10) < 3)
          {
            if (needRedraw)
            {
              clearBottom(3);
              lcd.print(menu0[(menuItem / 10) - 1]);
              lcd.setCursor(0, 2);
              lcd.print("  NON               ");
              lcd.setCursor(0, 3);
              lcd.print("  OUI               ");
            }
            lcd.setCursor(1, 2);
            lcd.write((menuItem % 10) == 0 ? (char)CHAR_FLECHD : ' ');
            lcd.setCursor(1, 3);
            lcd.write((menuItem % 10) == 0 ? ' ' : (char)CHAR_FLECHD);
          }
          else
          {
            if (needRedraw)
            {
              clearBottom(3);
              lcd.print(menu0[(menuItem / 10) - 1]);
            }
            for (byte i = 2; i < 4; i++)
            {
              item = menuItem % 10 + i - 2;
              lcd.setCursor(0, i);
              if ((menuItem / 10) == 3)
              {
                if ((item >= 0) && (item < SIZE_MENU1))
                {
                  lcd.print(menu1[item]);
                }
                else
                {
                  lcd.print(emptyLine);
                }
              }
              else
              {
                if ((item >= 0) && (item < SIZE_MENU2))
                {
                  lcd.print(menu2[item]);
                }
                else
                {
                  lcd.print(emptyLine);
                }
              }
            }
            lcd.setCursor(1, 2);
            lcd.write((char)CHAR_FLECHD);
          }
          break;
        case 2:
          if (needRedraw)
          {
            clearBottom(2);
            if ((menuItem / 100) == 3)
            {
              lcd.print(menu1[(menuItem / 10) % 10]);
            }
            else
            {
              lcd.print(menu2[(menuItem / 10) % 10]);
            }
          }
          if (menuItem == 300)
          {
            lcd.setCursor(10, 3);
            printTemp(tempHaut);
            lcd.write((char)CHAR_FLECHG);
          }
          else if (menuItem == 400)
          {
            lcd.setCursor(10, 3);
            printTemp(tempMin);
            lcd.write((char)CHAR_FLECHG);
          }
          else if (menuItem == 410)
          {
            lcd.setCursor(10, 3);
            printTemp(tempMax);
            lcd.write((char)CHAR_FLECHG);
          }
          else
          {
            if (needRedraw)
            {
              lcd.setCursor(0, 3);
              lcd.print("   NON   OUI        ");
            }
            lcd.setCursor(2, 3);
            lcd.write((menuItem % 10) ? ' ' : (char)CHAR_FLECHD);
            lcd.setCursor(6, 3);
            lcd.write((menuItem % 10) ? ' ' : (char)CHAR_FLECHG);
            lcd.setCursor(8, 3);
            lcd.write((menuItem % 10) ? (char)CHAR_FLECHD : ' ');
            lcd.setCursor(12, 3);
            lcd.write((menuItem % 10) ? (char)CHAR_FLECHG : ' ');
          }
          break;
      }
      needRefresh = false;
      needRedraw = false;
    }

    if (encoderPressed())
    { // bouton enfoncé
      bool escape = true;
      switch (menuItem)
      {
        case 1:
        case 2:
        case 3:
        case 4:
        case 30:
        case 31:
        case 40:
        case 41:
          escape = false;
          break;
        case 10:
        case 20:
        case 300:
        case 310:
        case 32:
        case 400:
        case 410:
        case 42:
          break;
        case 11:  // RAZ Config
          tempMin =    TEMP_MIN * 10;
          tempMax =    TEMP_MAX * 10;
          seuilBas =   TEMP_BAS * 10;
          seuilHaut =  TEMP_HAU * 10;
          seuilCircu = TEMP_CIR * 10;
          confChanged = true;
          break;
        case 21:  // RAZ Calibration
          tempOffsetH = 0;
          tempOffsetB = 0;
          caliChanged = true;
          break;
        case 311: // T1=T2
          tempOffsetB = (char)(tempHaut - tempBas);
          caliChanged = true;
          break;
        case 5:
          configuration = false;
          break;
      }
      if (escape)
      {
        level--;
        menuItem /= 10;
      }
      else
      {
        level++;
        menuItem *= 10;
      }
      while (encoderPressed())
      {
        delay(25);
      }
      needRedraw = true;
      rot = 0;
    }

    if (rot)
    {
      if (level == 0)
      {
        menuItem = constrain(menuItem + rot, 1, SIZE_MENU0);
      }
      else
      {
        if ((menuItem / 10) == 3)
        {
          menuItem = (10 * (menuItem / 10)) + constrain((menuItem % 10) + rot, 0, SIZE_MENU1 - 1);
        }
        else if ((menuItem / 10) == 4)
        {
          menuItem = (10 * (menuItem / 10)) + constrain((menuItem % 10) + rot, 0, SIZE_MENU2 - 1);
        }
        else if (menuItem == 300)
        {
          tempHaut -= tempOffsetH;
          tempOffsetH = constrain(tempOffsetH + rot, -100, 100);
          tempHaut += tempOffsetH;
          caliChanged = true;
        }
        else if (menuItem == 400)
        {
          tempMin = constrain(tempMin + rot, -10, min(1100, tempMax));
          confChanged = true;
        }
        else if (menuItem == 410)
        {
          tempMax = constrain(tempMax + rot, max(-10, tempMin), 1100);
          confChanged = true;
        }
        else
        {
          menuItem = (10 * (menuItem / 10)) + constrain((menuItem % 10) + rot, 0, 1);
        }
      }
      needRefresh = true;
      rot = 0;
    }

    delay(LOOP_DELAY);

    tPos++;
    tPos %= 10;
    // Relève de la température
    tB[tPos] = readTemp(PIN_CAPBAS);
    tH[tPos] = readTemp(PIN_CAPHAUT);
    if (tPos == 0)
    {
      // Calcul de la moyenne des 10 derniers relevés de température
      long tempB = 0;
      long tempH = 0;
      for (byte i = 9; i > 0; i--)
      {
        tempB += tB[i];
        tempH += tH[i];
      }

      tempBas = (short)(tempB / 10L);
      tempHaut = (short)(tempH / 10L);
    }
  }

  byte action = 0;
  if (confChanged)
  {
    sanityCheck();
    action += 1;
  }
  if (caliChanged)
  {
    action += 2;
  }
#ifdef DEBUG_SERIAL
  Serial.println(action);
#endif
  if (action)
  {
    saveEEPROM(ADDR_CONF, action);
  }
#ifdef DEBUG_SERIAL
  Serial.println("SORTIE CONFIGURATION");
#endif
}

/*
   Efface les n dernières lignes de l'écran
*/
void clearBottom(byte lines)
{
  byte l = lines;
  while (lines > 0)
  {
    lcd.setCursor(0, 4 - lines);
    lcd.print(emptyLine);
    lines--;
  }
  lcd.setCursor(0, 4 - l);
}

/*
   Règles de base pour garantir des valeurs intègres
*/
void sanityCheck()
{
  tempMin = min(tempMin, tempMax);
  seuilBas = constrain(seuilBas, tempMin, tempMax);
  seuilHaut = constrain(seuilHaut, tempMin, tempMax);
  seuilCircu = constrain(seuilCircu, tempMin, tempMax);
}

/*
   Vérifie si le bouton est enfoncé.
*/
bool encoderPressed() {
  return (digitalRead(PIN_ENC_SW) == LOW);
}

/*
   Routine appelée automatiquement en cas d'interruption
   causée par la rotation de l'encodeur.

   ATTENTION: ne pas modifier de variable qui n'est pas "volatile"
   dans une interruption.

   Note: il n'y a pas de vérification de débordement car il
   est supposé que cette variable est lue et remise à zéro
   assez fréquemment pour éviter un débordement.
*/
void routineInterruption()  {
  if (digitalRead(PIN_ENC_DIR))
  {
    rot++;
  }
  else
  {
    rot--;
  }
}
