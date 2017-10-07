//////////////////////////////////////////////////////////////////////////////////////////
/*
   Barographe (version 2017.10.07)
   Copyright 2015, 2016, 2017 - Eric Sérandour
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
*/
//////////////////////////////////////////////////////////////////////////////////////////

/*
  L'électronique :
  
  * Carte Arduino Mega 2560 R3
  * Shield Gameduino 2
  * Proto Shield Arduino
  * Baromètre BMP180 (I2C)

  Le circuit :
  
    Les entrées analogiques :
  =============================
  * Accéléromètre 3D
    X : X direction output voltage => A0    // Non utilisée dans ce programme
    Y : Y direction output voltage => A1    // Non utilisée dans ce programme
    Z : Z direction output voltage => A2    // Non utilisée dans ce programme
 
    Les entrées / sorties numériques :
  ======================================
  * D8 in GPU SEL
  * D9 in SD SEL
  * D11 in SPI-MOSI -> D51 sur l'Arduino Mega. (Master Out Slave In : Sortie de données séries)
  * D12 out SPI-MISO -> D50 sur l'Arduino Mega. (Master In Slave Out : Entrée de données séries)
  * D13 in SPI-SCK -> D52 sur l'Arduino Mega. (Serial ClocK : Pour synchroniser les échanges de données)
  Remarque : on peut accéder aux broches MOSI, MISO et SCK par l'intermédiaire du connecteur ICSP.
  L'intéret est que cela ne demande pas un recablage lorsque que l'on passe de la Mega à la Uno
  et inversement.

    Les ports de communication :
  ======================================
  Le baromètre BMP180 est relié à :
  * Bus I2C : SDA (A4 sur l'Arduino Uno, C20 sur l'Arduino Mega)
  * Bus I2C : SCL (A5 sur l'Arduino Uno, C21 sur l'Arduino Mega)
  Remarque : on peut accéder à ces broches par l'intermédiaire du connecteur à 10 points,
  après AREF : SDA pour la 9ème broche et SCL pour la 10ème broche. L'intéret est que cela
  ne demande pas un recablage lorsque que l'on passe de la Mega à la Uno et inversement.
  
    Bilan :
  ===========
    Sont utilisées : A0, A1, A2,
                     D2, D8, D9, D11, D12, D13, D50, D51, D52, D53
*/

//////////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>  // On importe la bibliothèque EEPROM
// Le microcontrôleur de la carte Arduino Uno dispose de 1024 octets de mémoire EEPROM.
// L'EEPROM est une mémoire non volatile dans laquelle les valeurs sont conservées lorsque
// la carte est mise hors-tension. La bibliothèque EEPROM permet de lire et d'écrire dans
// l'EEPROM.
#include <SPI.h>  // On importe la bibliothèque SPI
// Cette librairie permet de communiquer avec des périphériques SPI, la carte Arduino
// étant le composant "maître". La carte Gameduino 2 est un périphérique SPI.
#include <GD2.h>  // On importe la bibliothèque Gameduino 2
#include <SD.h>   // On importe la bibliothèque SD
const int CHIP_SELECT_SD = 9;

const int WIDTH = 480;       // Largeur de l'écran
const int HEIGHT = 272;      // Hauteur de l'écran
const int Y_MAX = HEIGHT-1;

const int TAILLE_TAMPON = WIDTH/2; // L'Arduino est limitée en mémoire. Au delà ça bugge.
int valeur[TAILLE_TAMPON];         // Création d'un tampon pour stocker les données
const int VALEUR_MIN = 10*976;     // On ne dépasse pas 32767 (2 octets)
const int VALEUR_MAX = 10*1050;    // On ne dépasse pas 32767 (2 octets)

// *** BMP180
#include <Wire.h>            // On importe la bibliothèque Wire pour l'I2C
#define ADRESSE_BMP180 0x77  // 119 en décimal
float pressionAbsolue = 0;
float pressionRelative = 0;
float altitude = 0;
float temperature = 0;       // Température à l'intérieur de la centrale
float altitudeReference = 0; // Altitude connue rentrée au clavier
float pressionReference = 0; // Pression à l'altitude de référence
float pressionMax = 0;
float pressionMoy = 0;
float pressionMin = 0;

// La gestion du temps
boolean debutChrono = false;                
const unsigned long TEMPO_PRESSION = 3*360000; // Un nouveau point toutes les 3*6 minutes 
                                               // 24h * 3600s * 1000ms / TAILLE_TAMPON  (en ms)      
const unsigned long TEMPO_VEILLE = 180000; // 3 minutes entre 2 ecrans de veille 
unsigned long maintenant = 0;
unsigned long instantRefPression = 0;
unsigned long instantRefVeille = 0;

char nombreFormate[8]; // Pour le formatage des nombres avec la fonction dtostrf()
const int POLICE_18 = 18;

const int TAG_TOUCHE_0 = 11; // On ne peut pas utiliser 0 comme Tag (0 = no touch)
const int TAG_TOUCHE_1 = 1;
const int TAG_TOUCHE_2 = 2;
const int TAG_TOUCHE_3 = 3;
const int TAG_TOUCHE_4 = 4;
const int TAG_TOUCHE_5 = 5;
const int TAG_TOUCHE_6 = 6;
const int TAG_TOUCHE_7 = 7;
const int TAG_TOUCHE_8 = 8;
const int TAG_TOUCHE_9 = 9;
const int TAG_TOUCHE_CLEAR = 10;
const int TAG_TOUCHE_OK = 12;
const int TAG_TOUCHE_SAVE = 13;
const int TAG_TOUCHE_LOAD = 14;

boolean enregistrement = false;
boolean chargement = false;
boolean erreur = false;

#define CODE_ENREGISTREMENT 2222

//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{   
  Serial.begin(9600);
  // On the Gameduino 2 Shield, CS for SD card is pin 9. Note that even if it's not
  // used as the CS pin, the hardware CS pin (10 on most Arduino boards, 53 on the Mega)
  // must be left as an output or the SD library functions will not work.
  // pinMode(10,OUTPUT); // Seulement sur l'Arduino Uno
  pinMode(53,OUTPUT); // Seulement sur l'Arduino Mega
  
  // Les 3 broches 11,12,13 du shield Gameduino 2 sont reroutées vers le bus SPI de la
  // carte Arduino Mega par 3 fils. Afin que les broches D11,D12,D13 de la carte Arduino ne
  // perturbent pas ce reroutage, on les définit comme des entrées.
  pinMode(11,INPUT); // Seulement sur l'Arduino Mega
  pinMode(12,INPUT); // Seulement sur l'Arduino Mega
  pinMode(13,INPUT); // Seulement sur l'Arduino Mega
  
  GD.begin(~GD_STORAGE); //  We start the GD library without SD card support
  GD.__end(); // On désactive GD le temps d'utiliser SD pour éviter un conflit sur le bus SPI
  // See if the card is present and can be initialized.
  if (!SD.begin(CHIP_SELECT_SD)) {
    Serial.println("Card failed, or not present");
    // Don't do anything more.
    return;
  }
  GD.resume(); // On réactive GD  
  Wire.begin();
  initBarometre(ADRESSE_BMP180);
}  

//////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  afficherBarometre(ADRESSE_BMP180);
}

//////////////////////////////////////////////////////////////////////////////////////////

void afficherClavier()
{
  int x0 = 186, x1 = 240, x2 = 294;
  int y0 =  74, y1 = 128, y2 = 182, y3=236;
  touche(x0,y0,TAG_TOUCHE_1); touche(x1,y0,TAG_TOUCHE_2); touche(x2,y0,TAG_TOUCHE_3);
  touche(x0,y1,TAG_TOUCHE_4); touche(x1,y1,TAG_TOUCHE_5); touche(x2,y1,TAG_TOUCHE_6);
  touche(x0,y2,TAG_TOUCHE_7); touche(x1,y2,TAG_TOUCHE_8); touche(x2,y2,TAG_TOUCHE_9);
  touche(x0,y3,TAG_TOUCHE_CLEAR); touche(x1,y3,TAG_TOUCHE_0); touche(x2,y3,TAG_TOUCHE_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////

void touche(int x, int y, byte label)
{
  const int LARGEUR_TOUCHE = 46;
  const int HAUTEUR_TOUCHE = 46;
  
  GD.ColorRGB(0x00ff00);
  GD.Begin(RECTS);
  GD.LineWidth(16*2);
  GD.Vertex2ii(x - LARGEUR_TOUCHE / 2, y - HAUTEUR_TOUCHE / 2);
  GD.Vertex2ii(x + LARGEUR_TOUCHE / 2, y + HAUTEUR_TOUCHE / 2);  
  GD.ColorRGB(0x000000);
  GD.LineWidth(16*2);
  GD.Tag(label);
  GD.Vertex2ii(x - LARGEUR_TOUCHE / 2 + 1, y - HAUTEUR_TOUCHE / 2 + 1);
  GD.Vertex2ii(x + LARGEUR_TOUCHE / 2 - 1, y + HAUTEUR_TOUCHE / 2 - 1);
  GD.ColorRGB(0x00ff00);
  switch (label) {
    case TAG_TOUCHE_SAVE :
      GD.cmd_text(x,y,POLICE_18, OPT_CENTER, "SAVE");
      break;
    case TAG_TOUCHE_LOAD :
      GD.cmd_text(x,y,POLICE_18, OPT_CENTER, "LOAD");
      break;          
    case TAG_TOUCHE_CLEAR :      
      GD.cmd_text(x,y,POLICE_18, OPT_CENTER, "CLR");
      break;    
    case TAG_TOUCHE_OK :      
      GD.cmd_text(x,y,POLICE_18, OPT_CENTER, "OK");
      break;
    case TAG_TOUCHE_0 :      
      GD.cmd_number(x,y,POLICE_18, OPT_CENTER, 0);
      break;
    default :
      GD.cmd_number(x,y,POLICE_18, OPT_CENTER, label);
  } 
}

//////////////////////////////////////////////////////////////////////////////////////////

void enregistreFichier()
{
  File dataFile;
  erreur = false;
  // Sortie sur carte SD
  // Efface le fichier data.txt précédent  
  if (SD.exists("data.txt")) {
    SD.remove("data.txt");
  }
  // Crée le fichier data.txt
  dataFile = SD.open("data.txt", FILE_WRITE);
  // If the file is available, write to it.
  if (dataFile) {
    for (int i=0; i<TAILLE_TAMPON; i++) {
      dataFile.print(valeur[i]);
      dataFile.println("");
    }
    dataFile.close(); 
  }
  // If the file isn't open, pop up an error.
  else {
    Serial.println("Error opening data.txt");
    erreur = true;  
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

void chargeFichier()
{
  String chaine = "";
  int i = 0;  
  // Lecture du fichier data.txt
  File dataFile = SD.open("data.txt", FILE_READ);
  erreur = false;
  if (dataFile) {
    // Read from the file until there's nothing else in it
    while (dataFile.available()) {
      byte caractere = dataFile.read();
      // Convert the incoming byte to a char and add it to the string
      chaine = chaine + (char)caractere;
      // Si on atteint la fin d'une ligne
      if (caractere == (char)10) {
        // Chacune des lignes du fichier y compris la dernière se termine par un retour à
        // la ligne codé par 2 octets invisibles : 13 (CR) puis 10 (LF)
        // On supprime ces 2 derniers octets
        chaine = chaine.substring(0,chaine.length()-2);
        valeur[i] = chaine.toInt(); // Conversion d'une chaine en un nombre entier
        Serial.print(i);
        Serial.print(";");
        Serial.println(valeur[i]);
        i++;
        chaine = "";
      }
    }
    // Close the file
    dataFile.close();
    statistiquesPressions();
  }
  else {
    // If the file didn't open, print an error
    Serial.println("Error opening data.txt");
    erreur = true;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
/*
    ALTI-BAROMETRE BMP180
*/
//////////////////////////////////////////////////////////////////////////////////////////

void afficherBarometre(int adresseI2C)
{
  // Réglage de l'altitude
  reglageAltitude(adresseI2C);
  
  // Affichage des mesures
  boolean quitter = false;
  debutChrono = true;
  do {
    byte codeErreur = barometreRead(adresseI2C);
    if (codeErreur != 0) {
      // Afficher Erreur
      GD.ClearColorRGB(0x000000);
      GD.Clear(); // Efface l'écran
      GD.ColorRGB(0x00ff00); // Texte en vert
      GD.cmd_text(20, 25, POLICE_18, OPT_CENTERY, "BMP180 : Erreur");
      dtostrf(codeErreur, 1, 0, nombreFormate);
      GD.cmd_text(148, 25, POLICE_18, OPT_CENTERY, nombreFormate);
      GD.swap(); // On peut afficher.      
      delay(5000);
      quitter = true;
    }
    else {
      barometreRead(adresseI2C);
      // Gestion du tampon qui stocke les mesures sur 24 h de la pression relative
      maintenant = millis();
      if (debutChrono) {
        instantRefPression = maintenant;
        debutChrono = false;
      }
      unsigned long dureePression = maintenant - instantRefPression;
      if (dureePression >= TEMPO_PRESSION) {
        // Décalage des données vers la gauche dans le tampon
        for (int i=0; i<TAILLE_TAMPON-1; i++) {
          valeur[i] = valeur[i+1];
        }
        // On accueille une nouvelle donnée dans le tampon
        valeur[TAILLE_TAMPON-1] = int(10 * pressionRelative); // 10 * : Pour récupérer la 1ère décimale
        // Détermination des pressions moyenne, maximum et minimum
        statistiquesPressions();
        debutChrono = true;
      }
      unsigned long dureeVeille = maintenant - instantRefVeille;   
      if (dureeVeille >= TEMPO_VEILLE) {
        GD.ClearColorRGB(0x000000);
        GD.Clear(); // Efface l'écran
        GD.ColorRGB(0x00ff00); // Texte en vert
        GD.cmd_text(random(WIDTH), random(HEIGHT), POLICE_18, OPT_CENTER, "TOUCHEZ");
        GD.cmd_text(random(WIDTH), random(HEIGHT), POLICE_18, OPT_CENTER, "LE");
        GD.cmd_text(random(WIDTH), random(HEIGHT), POLICE_18, OPT_CENTER, "BAROGRAPHE");
        GD.swap();  // On peut afficher
        for (int i=0; i<10; i++) {
          delay(100);
          GD.get_inputs();
          if (GD.inputs.rz != 32767) { // S'il y a une pression sur l'écran
            instantRefVeille = maintenant;
            break;
          }
        }
      }
      else {      
        GD.ClearColorRGB(0x000000);
        GD.Clear(); // Efface l'écran
        // Bouton RAZ (Remise à zéro)
        // touche(432,47,TAG_TOUCHE_CLEAR);
        // Bouton ENR (Enregistrement)
        touche(378,224,TAG_TOUCHE_SAVE);
        // Bouton USB (Transfert vers USB)
        touche(432,224,TAG_TOUCHE_LOAD);            
        GD.Tag(0); // Pour tous les éléments qui suivent
        // Tracé de l'axe central correspondant à 1013 hPa
        for (int i=0; i<24; i++) {
          GD.ColorRGB(0x00ff00); // Axe en vert
          GD.LineWidth(8); // 16*0.5
          GD.Begin(LINES);
          GD.Vertex2ii(4+20*i,136);
          GD.Vertex2ii(15+20*i,136);
        }
        // Tracé des séparateurs jounaliers
        for (int i=0; i<3; i++) {
          GD.Vertex2ii(i*WIDTH/3,126);
          GD.Vertex2ii(i*WIDTH/3,146);
        }
        GD.Vertex2ii(WIDTH-1,126);
        GD.Vertex2ii(WIDTH-1,146);              
        // Tracé de la courbe à l'écran  
        GD.ColorRGB(0xff0000); // Courbe en rouge
        GD.LineWidth(16); // 16*1
        GD.Begin(LINES);
        for (int i=0; i<TAILLE_TAMPON-1; i++) {
          if (valeur[i] != -1) {
            GD.Vertex2ii(WIDTH / TAILLE_TAMPON * i, Y_MAX - map(valeur[i], VALEUR_MIN, VALEUR_MAX, 0, Y_MAX));   
            GD.Vertex2ii(WIDTH / TAILLE_TAMPON * (i+1), Y_MAX - map(valeur[i+1], VALEUR_MIN, VALEUR_MAX, 0, Y_MAX));
          }
        }      
        // Affichage du texte et des mesures
        GD.ColorRGB(0x00ff00); // Texte en vert
        GD.cmd_text(20, 25, POLICE_18, OPT_CENTERY, "Altitude station  : ");
        dtostrf(altitudeReference, 6, 1, nombreFormate);
        GD.cmd_text(180, 25, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, 25, POLICE_18, OPT_CENTERY, "m");
        GD.cmd_text(20, 40, POLICE_18, OPT_CENTERY, "Pression absolue  : ");
        dtostrf(pressionAbsolue, 6, 1, nombreFormate);
        GD.cmd_text(180, 40, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, 40, POLICE_18, OPT_CENTERY, "hPa");  
        GD.cmd_text(20, 55, POLICE_18, OPT_CENTERY, "Pression relative : ");
        dtostrf(pressionRelative, 6, 1, nombreFormate);
        GD.cmd_text(180, 55, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, 55, POLICE_18, OPT_CENTERY, "hPa");
        GD.cmd_text(20, 70, POLICE_18, OPT_CENTERY, "Altitude          : ");
        dtostrf(altitude, 6, 1, nombreFormate);
        GD.cmd_text(180, 70, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, 70, POLICE_18, OPT_CENTERY, "m");
        GD.cmd_text(20, Y_MAX - 55, POLICE_18, OPT_CENTERY, "Pression maximum  : ");
        dtostrf(pressionMax, 6, 1, nombreFormate);
        GD.cmd_text(180, Y_MAX - 55, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, Y_MAX - 55, POLICE_18, OPT_CENTERY, "hPa"); 
        GD.cmd_text(20, Y_MAX - 40, POLICE_18, OPT_CENTERY, "Pression moyenne  : ");
        dtostrf(pressionMoy, 6, 1, nombreFormate);
        GD.cmd_text(180, Y_MAX - 40, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, Y_MAX - 40, POLICE_18, OPT_CENTERY, "hPa");              
        GD.cmd_text(20, Y_MAX - 25, POLICE_18, OPT_CENTERY, "Pression minimum  : ");
        dtostrf(pressionMin, 6, 1, nombreFormate);
        GD.cmd_text(180, Y_MAX - 25, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(236, Y_MAX - 25, POLICE_18, OPT_CENTERY, "hPa");
        GD.cmd_text(20, Y_MAX - 70 , POLICE_18, OPT_CENTERY, "Sur 72 heures");  
        // Rond rouge pour indiquer que le tracé rouge concerne la pression relative
        GD.ColorRGB(0xff0000);  
        GD.PointSize(16*5); // 5 pixels  
        GD.Begin(POINTS);
        GD.Vertex2ii(275,54);
        if (erreur) {
          GD.cmd_text(WIDTH/2, HEIGHT/2, POLICE_18, OPT_CENTER, "ERREUR CARTE SD");
        }
        else if (enregistrement) {
          if (motDePasse()) {
            GD.ClearColorRGB(0x000000);
            GD.Clear();  // Efface l'écran
            GD.ColorRGB(0xff0000);  // Texte en rouge
            GD.cmd_text(WIDTH/2, HEIGHT/2, POLICE_18, OPT_CENTER, "ENREGISTREMENT EN COURS");
          }
          else {
            enregistrement = false;
          }
        }
        else if (chargement) {
          if (motDePasse()) {
            GD.ClearColorRGB(0x000000);
            GD.Clear();  // Efface l'écran
            GD.ColorRGB(0xff0000);  // Texte en rouge
            GD.cmd_text(WIDTH/2, HEIGHT/2, POLICE_18, OPT_CENTER, "CHARGEMENT EN COURS");
          }
          else {
            chargement = false;
          }
        }
        GD.swap(); // L'écran est pret. On peut afficher.
        
        if (erreur) {
          delay(2000);
          erreur = false;
        }
        else if (enregistrement) {
          GD.__end(); // On désactive GD le temps d'utiliser SD pour éviter un conflit sur le bus SPI
          enregistreFichier();
          GD.resume(); // On réactive GD
          enregistrement = false;
          delay(1000);
        }
        else if (chargement) {
          GD.__end(); // On désactive GD le temps d'utiliser SD pour éviter un conflit sur le bus SPI
          chargeFichier();
          GD.resume(); // On réactive GD
          chargement = false;
        }

        
        GD.get_inputs();
        switch (GD.inputs.tag) {
          case TAG_TOUCHE_CLEAR :
            quitter = true;
            break;
          case TAG_TOUCHE_SAVE :            
            enregistrement = true;
            break;
          case TAG_TOUCHE_LOAD :
            chargement = true;
            break;            
        }
      }
    }
  } while (quitter == false);   
}

//////////////////////////////////////////////////////////////////////////////////////////

void statistiquesPressions()
{
  // Détermination des pressions moyenne, maximum et minimum
  pressionMoy = 0;
  pressionMin = VALEUR_MAX / 10;
  pressionMax = VALEUR_MIN / 10;     
  int nbPoints = 0;
  for (int i = 0 ; i<TAILLE_TAMPON; i++) {          
    if  (valeur[i] != -1) {
      nbPoints++;
      pressionMoy+=valeur[i]/10.0;        
      if (valeur[i] >= int(10*pressionMax)) {
        pressionMax = valeur[i]/10.0;
      }
      if (valeur[i] <= int(10*pressionMin)) {
        pressionMin = valeur[i]/10.0;
      }
    }          
  }
  pressionMoy /= nbPoints;
}

//////////////////////////////////////////////////////////////////////////////////////////

void reglageAltitude(int adresseI2C)
{  
  for (int i=0; i< TAILLE_TAMPON; i++) {
    valeur[i] = -1;
  }
  barometreRead(adresseI2C);
  pressionReference = pressionAbsolue;
  altitudeReference = 0;
  boolean quitter = false;
  int nbPressionTouche = 0;
  do {
    GD.get_inputs();
    switch (GD.inputs.tag) {
      case TAG_TOUCHE_CLEAR :
        altitudeReference = 0;
        GD.Clear(); // Efface l'écran
        nbPressionTouche = 0;
        break;
      case TAG_TOUCHE_OK :
        barometreRead(adresseI2C);
        pressionReference = pressionAbsolue;
        pressionMax = pressionRelative;
        pressionMin = pressionRelative;
        pressionMoy = pressionRelative;        
        quitter = true;      
        break;
      default :
        GD.Clear(); // Efface l'écran    
        GD.ColorRGB(0x00ff00); // Texte en vert
        GD.cmd_text(140, 25, POLICE_18, OPT_CENTERY, "Altitude station : ");
        dtostrf(altitudeReference, 6, 0, nombreFormate);
        GD.cmd_text(276, 25, POLICE_18, OPT_CENTERY, nombreFormate);
        GD.cmd_text(332, 25, POLICE_18, OPT_CENTERY, "m");
        afficherClavier();
        boolean toucheOK = false;
        int valeurTouche;        
        while (GD.inputs.tag != 0) {
          GD.get_inputs();
          switch (GD.inputs.tag) {
            case TAG_TOUCHE_0 : valeurTouche = 0; toucheOK = true; break;
            case TAG_TOUCHE_1 : valeurTouche = 1; toucheOK = true; break; 
            case TAG_TOUCHE_2 : valeurTouche = 2; toucheOK = true; break; 
            case TAG_TOUCHE_3 : valeurTouche = 3; toucheOK = true; break; 
            case TAG_TOUCHE_4 : valeurTouche = 4; toucheOK = true; break; 
            case TAG_TOUCHE_5 : valeurTouche = 5; toucheOK = true; break; 
            case TAG_TOUCHE_6 : valeurTouche = 6; toucheOK = true; break; 
            case TAG_TOUCHE_7 : valeurTouche = 7; toucheOK = true; break; 
            case TAG_TOUCHE_8 : valeurTouche = 8; toucheOK = true; break; 
            case TAG_TOUCHE_9 : valeurTouche = 9; toucheOK = true; break;
          }
        }
        if ((toucheOK) && (nbPressionTouche < 4)) { // La touche doit etre relachée pour etre prise en compte       
          altitudeReference = 10 * altitudeReference + valeurTouche;
          nbPressionTouche++;
        }
    }
    GD.swap();
  } while (quitter == false); 
}

//////////////////////////////////////////////////////////////////////////////////////////

int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md; // Calibration coefficients
uint16_t ac4, ac5, ac6;                     // Calibration coefficients
// Ultra low power       : oss = 0, osd =  5 ms
// Standard              : oss = 1, osd =  8 ms
// High resolution       : oss = 2, osd = 14 ms
// Ultra high resolution : oss = 3, osd = 26 ms
const uint8_t OSS = 3;     // Set oversampling setting
const uint8_t OSD = 26;    // with corresponding oversampling delay

//////////////////////////////////////////////////////////////////////////////////////////

void initBarometre(int adresseI2C) // Voir le Data sheet du BMP180, à la page 15.
{
  // Read calibration data from the EEPROM of the BMP180
  ac1 = readRegister16(adresseI2C, 0xAA);
  ac2 = readRegister16(adresseI2C, 0xAC);
  ac3 = readRegister16(adresseI2C, 0xAE);
  ac4 = readRegister16(adresseI2C, 0xB0);
  ac5 = readRegister16(adresseI2C, 0xB2);
  ac6 = readRegister16(adresseI2C, 0xB4);
  b1  = readRegister16(adresseI2C, 0xB6);
  b2  = readRegister16(adresseI2C, 0xB8);
  mb  = readRegister16(adresseI2C, 0xBA);
  mc  = readRegister16(adresseI2C, 0xBC);
  md  = readRegister16(adresseI2C, 0xBE);
}

//////////////////////////////////////////////////////////////////////////////////////////

uint16_t readRegister16(int adresseI2C, uint8_t code)
{
  uint16_t value = 0;
  Wire.beginTransmission(adresseI2C);         // Start transmission to device 
  Wire.write(code);                           // Sends register address to read from
  byte error = Wire.endTransmission();        // End transmission
  if (error == 0) {
    Wire.requestFrom(adresseI2C, 2);          // Request 2 bytes from device
    while (Wire.available() < 2);             // Wait until bytes are ready
    value = (Wire.read() << 8) + Wire.read();
  }
  return value;
}

//////////////////////////////////////////////////////////////////////////////////////////

byte barometreRead(int adresseI2C) // Voir le Data sheet du BMP180, à la page 15.
{
  int32_t x1, x2, x3, b3, b5, b6, ut, up, t, p;
  uint32_t b4, b7;
  int16_t msb, lsb, xlsb;
  byte error = 0;
  
  // Read uncompensated temperature value (ut)
  Wire.beginTransmission(adresseI2C);          // Start transmission to device
  Wire.write(0xf4);                            // Sends register address
  Wire.write(0x2e);                            // Write data
  error = Wire.endTransmission();              // End transmission 
  if (error == 0) { // On continue
    delay(5);                                  // Data sheet suggests 4.5 ms 
    
    Wire.beginTransmission(adresseI2C);        // Start transmission to device
    Wire.write(0xf6);                          // Sends register address to read from
    error = Wire.endTransmission();            // End transmission
    if (error == 0) { // On continue
      Wire.requestFrom(adresseI2C, 2);         // Request 2 bytes (0xf6, 0xf7)
      while (Wire.available() < 2);            // Wait until bytes are ready
      msb = Wire.read();
      lsb = Wire.read();
  
      ut = ((int32_t)msb << 8) + (int32_t)lsb;
  
      // Read uncompensated pressure value (up)
      Wire.beginTransmission(adresseI2C);      // Start transmission to device
      Wire.write(0xf4);                        // Sends register address
      Wire.write(0x34 + (OSS << 6));           // Write data
      error = Wire.endTransmission();          // End transmission
      if (error == 0) { // On continue
        delay(OSD);                            // Oversampling setting delay
  
        Wire.beginTransmission(adresseI2C);    // Start transmission to device
        Wire.write(0xf6);                      // Sends register address to read from
        error = Wire.endTransmission();        // End transmission
        if (error == 0) { // On continue
          Wire.requestFrom(adresseI2C, 3);     // Request 3 bytes (0xf6, 0xf7, 0xf8)
          while (Wire.available() < 3);        // Wait until bytes are ready
          msb = Wire.read();
          lsb = Wire.read();
          xlsb = Wire.read();
  
          up = (((int32_t)msb << 16) + ((int32_t)lsb << 8) + ((int32_t)xlsb)) >> (8 - OSS);
  
          // Calculate true temperature
          x1 = (ut - (int32_t)ac6) * (int32_t)ac5 >> 15;
          x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
          b5 = x1 + x2;
          t = (b5 + 8) >> 4;
          temperature = t / 10.0f;  // temperature in celsius                         
  
          // Calculate true pressure
          // On étend la taille de certaines variables pour éviter des dépassements.
          // Par exemple, dans la 2ème ligne, x1 est int32_t, b2 est int16_t
          // et b6 est int_32t d'où le (int32_t)b2.
          // Pour gagner en vitesse de calcul, on utilise << ou >> :
          // << : un décalage de 1 bit vers la gauche revient à multiplier par 2
          // >> : un décalage de 1 bit vers la droite revient à diviser par 2
          b6 = b5 - 4000;          
          x1 = ((int32_t)b2 * (b6 * b6 >> 12)) >> 11;          
          x2 = (int32_t)ac2 * b6 >> 11;          
          x3 = x1 + x2;          
          b3 = ((((int32_t)ac1 * 4 + x3) << OSS) + 2) >> 2;          
          x1 = (int32_t)ac3 * b6 >> 13;          
          x2 = ((int32_t)b1 * (b6 * b6 >> 12)) >> 16;
          x3 = ((x1 + x2) + 2) >> 2;
          b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
          b7 = ((uint32_t)up - (uint32_t)b3) * (uint32_t)(50000 >> OSS);
          if (b7 < 0x80000000) { p = (b7 << 1) / b4; }
          else { p = (b7 / b4) << 1; }
          x1 = (p >> 8) * (p >> 8);
          x1 = (x1 * 3038) >> 16;
          x2 = (-7357 * p) >> 16;
          p = p + ((x1 + x2 + 3791) >> 4);
          pressionAbsolue = p / 100.0f;  // pression in hPa
  
          // Calculate pressure at sea level
          pressionRelative = pressionAbsolue / pow((1.0f - (altitudeReference / 44330.0f)), 5.255f);
          
          // Calculate absolute altitude
          float pression0 = pressionReference / pow((1.0f - (altitudeReference / 44330.0f)), 5.255f);
          altitude = 44330.0f * (1 - pow(pressionAbsolue / pression0, (1 / 5.255f)));
        }
      }
    }
  }
  return error;
}

//////////////////////////////////////////////////////////////////////////////////////////

boolean motDePasse()
{
  boolean autorisationEnregistrement = false;
  int codeEnregistrement = 0;
  boolean quitter = false;
  int nbPressionTouche = 0;
  do {
    GD.get_inputs();
    switch (GD.inputs.tag) {
      case TAG_TOUCHE_CLEAR :
        codeEnregistrement = 0;
        GD.Clear();  // Efface l'écran
        nbPressionTouche = 0;
        break;
      case TAG_TOUCHE_OK :
        quitter = true;
        break;
      default :
        GD.Clear();  // Efface l'écran
        GD.ColorRGB(0x00ff00);  // Texte en vert
        GD.cmd_text(173, 25, POLICE_18, OPT_CENTERY, "Votre code : ");
        dtostrf(codeEnregistrement, 6, 0, nombreFormate);
        GD.cmd_text(261, 25, POLICE_18, OPT_CENTERY, nombreFormate);
        afficherClavier();
        boolean toucheOK = false;
        int valeurTouche;
        while (GD.inputs.tag !=0) {
          GD.get_inputs();
          switch (GD.inputs.tag) {
            case TAG_TOUCHE_0 : valeurTouche = 0; toucheOK = true; break;
            case TAG_TOUCHE_1 : valeurTouche = 1; toucheOK = true; break;
            case TAG_TOUCHE_2 : valeurTouche = 2; toucheOK = true; break;
            case TAG_TOUCHE_3 : valeurTouche = 3; toucheOK = true; break;
            case TAG_TOUCHE_4 : valeurTouche = 4; toucheOK = true; break;
            case TAG_TOUCHE_5 : valeurTouche = 5; toucheOK = true; break;
            case TAG_TOUCHE_6 : valeurTouche = 6; toucheOK = true; break;
            case TAG_TOUCHE_7 : valeurTouche = 7; toucheOK = true; break;
            case TAG_TOUCHE_8 : valeurTouche = 8; toucheOK = true; break;
            case TAG_TOUCHE_9 : valeurTouche = 9; toucheOK = true; break;
          }
        }
        if ((toucheOK) && (nbPressionTouche < 4)) {  // La touche doit être relachée pour être prise en compte
          codeEnregistrement = 10 * codeEnregistrement + valeurTouche;
          nbPressionTouche++;
        }
    }
    GD.swap();
    } while (quitter == false);
    if (codeEnregistrement == CODE_ENREGISTREMENT) {
      autorisationEnregistrement = true;
    }
  return autorisationEnregistrement;
}

