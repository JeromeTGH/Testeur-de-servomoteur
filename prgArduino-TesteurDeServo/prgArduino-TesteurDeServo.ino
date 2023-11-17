/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TesteurDeServo.ino
  
  Description :   Programme permettant de tester un servomoteur, tout en pouvant
                  régler les seuils haut/bas pour le rapport cyclique du signal PWM,
                  avec affichage sur écran OLED
                  
  Remarques :     - l'arduino utilisé ici sera un modèle Nano
                  - la librairie utilisée pour contrôler le servomoteur sera la "bibliotèque Servo", native sous Arduino IDE
                    https://www.arduino.cc/reference/en/libraries/servo/
                  - la librairie utilisée pour contrôler l'écran OLED (modèle 0.96" avec contrôleur SSD1306) sera celle d'Adafruit
                    https://github.com/adafruit/Adafruit_SSD1306?pseSrc=pgEcranOledArduino
                                    
  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       03.11.2023

*/


// Inclusion des librairies dont nous allons nous servir ici
#include <Servo.h>
#include <Adafruit_SSD1306.h>


// Affectation des broches de l'Arduino
#define pinArduinoRaccordeeAuBoutonDuHaut   2       // La pin D2 de l'Arduino est connectée au "bouton du HAUT" (bloc de 5 boutons : haut/bas/gauche/droite/ok)
#define pinArduinoRaccordeeAuBoutonDeDroite 3       // La pin D3 de l'Arduino est connectée au "bouton de DROITE"
#define pinArduinoRaccordeeAuBoutonDuBas    4       // La pin D4 de l'Arduino est connectée au "bouton du BAS"
#define pinArduinoRaccordeeAuBoutonDeGauche 5       // La pin D5 de l'Arduino est connectée au "bouton de GAUCHE"
#define pinArduinoRaccordeeAuBoutonDuCentre 6       // La pin D6 de l'Arduino est connectée au "bouton du CENTRE" (noté "ok")
#define pinArduinoRaccordeeAuServomoteur    9       // La pin D9 de l'Arduino est connectée à la broche "signal" du servomoteur
#define pinArduinoRaccordeeAuPotentiometre  A0      // La pin A0 de l'Arduino est connectée au point milieu du potentiomètre de réglage de la position angulaire du servo
#define pinArduinoRaccordeeAuSDAecranOLED   A4      // La pin A4 de l'Arduino est connectée à la broche SDA de l'écran OLED
#define pinArduinoRaccordeeAuSCLecranOLED   A5      // La pin A5 de l'Arduino est connectée à la broche SCL de l'écran OLED


// Autres constantes
#define frequenceSignalPwmServo             50      // La fréquence PWM est de 50 Hz (non utilisé ici, c'est juste pour rappel)
#define valeurMinParDefautImpulsionServo    1000    // Par défaut, la durée d'impulsion min servo sera de 1 ms minimum (1000 µs) ; mais cela pourrait être moins, si souhaité (nécessaire, parfois)
#define valeurMaxParDefautImpulsionServo    2000    // Par défaut, la durée d'impulsion max servo sera de 2 ms maximum (2000 µs) ; mais cela pourrait être plus, si souhaité (nécessaire, parfois)
#define valeurMinLimiteImpulsionServo       544     // Limite : la durée d'impulsion servo ne devra jamais être inférieure à 544 µs (imposé par la librairie Servo.h)
#define valeurMaxLimiteImpulsionServo       2400    // Limite : la durée d'impulsion servo ne devra jamais être supérieure à 2400 µs (imposé par la librairie Servo.h)
#define nombreDePixelsEnLargeurEcranOLED    128     // Taille de l'écran OLED, en pixel, au niveau de sa largeur
#define nombreDePixelsEnHauteurEcranOLED    64      // Taille de l'écran OLED, en pixel, au niveau de sa hauteur
#define brocheResetOLED                     -1      // Reset de l'écran OLED partagé avec l'Arduino, d'où la valeur à -1 ici (et non un numéro de pin)
#define adresseI2CdeLecranOLED              0x3C    // Adresse i2c de l'écran OLED (généralement 0x3C par défaut, sinon 0x3D)


// Variables librairies
Servo servomoteur;
Adafruit_SSD1306 ecran_oled(nombreDePixelsEnLargeurEcranOLED, nombreDePixelsEnHauteurEcranOLED, &Wire, brocheResetOLED);

// Autres variables
int ligne_selectionnee_dans_menu = 1;



// ========================
// Initialisation programme
// ========================
void setup() {

    // Initialisation de la liaison série (PC <-> arduino nano), pour le débuggage, si besoin
    Serial.begin(9600);
    Serial.println(F("================"));
    Serial.println(F("Testeur de servo"));
    Serial.println(F("================"));
    Serial.println("");

    // Initialisation de l'écran OLED
    if(!ecran_oled.begin(SSD1306_SWITCHCAPVCC, adresseI2CdeLecranOLED)) {
      Serial.println(F("[ERREUR] Impossible de communiquer avec l'écran OLED (arrêt du programme)"));
      Serial.flush();
      while(1);
    } else {
      Serial.println(F("Écran OLED présent (initialisation du contrôleur SSD1306 réussie)"));
    }
    

    // Définition des entrées/sorties de l'Arduino
    pinMode(pinArduinoRaccordeeAuBoutonDuHaut, INPUT_PULLUP);
    pinMode(pinArduinoRaccordeeAuBoutonDeDroite, INPUT_PULLUP);
    pinMode(pinArduinoRaccordeeAuBoutonDuBas, INPUT_PULLUP);
    pinMode(pinArduinoRaccordeeAuBoutonDeGauche, INPUT_PULLUP);
    pinMode(pinArduinoRaccordeeAuBoutonDuCentre, INPUT_PULLUP);    
    // Remarques : 
    //   - les résistances pull-up internes de l'Arduino sont activées sur les broches ci-dessus, car il n'y en a pas sur le montage, à proprement parler
    //   - pas besoin d'utiliser "pinMode" pour déclarer la broche "signal servo" en sortie, car cela est implicitement fait par la biliothèque Servo
    //   - pas besoin d'utiliser "pinMode" pour déclarer A0 en entrée, car toutes les broches sont de type INPUT, au démarrage programme
    //   - pas besoin d'utiliser "pinMode" pour gérer les lignes SDA (A4) et SCL (A5), car cela est géré par la librairie gérant l'écran OLED


    // Raccordement logiciel de la ligne "signal" du servomoteur, à la broche arduino (D9, pour rappel)
    servomoteur.attach(pinArduinoRaccordeeAuServomoteur);


    // Affiche le menu
    ecran_oled.setTextColor(SSD1306_WHITE);
    afficheMenu();



    // Petite pause, avant d'attaquer la boucle loop
    delay(500);


    // Test :
    servomoteur.writeMicroseconds(1500);   //  Met le servo en position médiane

}

// ======================
// Fonction : afficheMenu
// ======================
void afficheMenu() {

  // Effaçage de la mémoire tampon de l'écran OLED
  ecran_oled.clearDisplay();                           

  // Titre
  ecran_oled.setTextSize(2);
  ecran_oled.setCursor(10, 0);
  ecran_oled.println("SERVOTEST");

  // Choix
  ecran_oled.setTextSize(1);
  ecran_oled.setCursor(10, 20);
  ecran_oled.println("T(min) : 1000");
  ecran_oled.setCursor(10, 30);
  ecran_oled.println("T(max) : 2000");
  ecran_oled.setCursor(10, 50);
  ecran_oled.println("Reinitialiser");

  // Curseur (symbole ">")
  ecran_oled.setCursor(0, (1 * 10) + 10);
  ecran_oled.println(">");

  // Affichage (transfert de la mémoire tampon à l'écran OLED)
  ecran_oled.display();
}



// =================
// Boucle principale
// =================
void loop() {

//    // Lecture des signaux du KY-040 arrivant sur l'arduino
//    int etatActuelDeLaLigneCLK = digitalRead(pinArduinoRaccordementSignalCLK);
//    int etatActuelDeLaLigneSW  = digitalRead(pinArduinoRaccordementSignalSW);
//    int etatActuelDeLaLigneDT  = digitalRead(pinArduinoRaccordementSignalDT);
//
//    // *****************************************
//    // On regarde si la ligne SW a changé d'état
//    // *****************************************
//    if(etatActuelDeLaLigneSW != etatPrecedentLigneSW) {
//
//        // Si l'état de SW a changé, alors on mémorise son nouvel état
//        etatPrecedentLigneSW = etatActuelDeLaLigneSW;
//
//        // Puis on affiche le nouvel état de SW sur le moniteur série de l'IDE Arduino
//        if(etatActuelDeLaLigneSW == LOW)
//            Serial.println(F("Bouton SW appuyé"));
//        else
//            Serial.println(F("Bouton SW relâché"));
//
//        // Petit délai de 10 ms, pour filtrer les éventuels rebonds sur SW
//        delay(10);
//    }

   

    // ********************************
    // Puis on reboucle … à l'infini !
    // ********************************
   
}
