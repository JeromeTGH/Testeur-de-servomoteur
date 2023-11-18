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
#include <EEPROM.h>
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


// Constantes concernant le servomoteur
      // Juste pour rappel, concernant les servomoteurs :
      //   - durée/période du signal de commande servomoteur            => 20 ms (20000 µs)     => cela correspond à un signal à 50 Hz, tout simplement
      //   - délai d'impulsion pour tourner le servo de -90° environ    =>  1 ms (1000 µs)
      //   - délai d'impulsion pour tourner le servo de +90° environ    =>  2 ms (2000 µs)
      //   - délai minimal d'impulsion (imposé par librairie Servo.h)   =>  0.544 ms (544 µs)   => je vais prendre 550 µs dans ce code, pour simplifier les choses
      //   - délai maximal d'impulsion (imposé par librairie Servo.h)   =>  2.400 ms (2400 µs)
#define valeurMinSeuilBasImpulsionServo       550     // Avec ces 3 valeurs (550/1000/1400), on spécifie que la valeur min d'une impulsion servo
#define valeurDefautSeuilBasImpulsionServo    1000    // sera comprise entre 550 et 1400 µs (avec 1000µs, par défaut)
#define valeurMaxSeuilBasImpulsionServo       1400
#define valeurMinSeuilHautImpulsionServo      1600    // Avec ces 3 valeurs (1600/2000/2400), on spécifie que la valeur max d'une impulsion servo
#define valeurDefautSeuilHautImpulsionServo   2000    // sera comprise entre 1600 et 2400 µs (avec 2000µs, par défaut)
#define valeurMaxSeuilHautImpulsionServo      2400
#define pasDeModificationDelaiImpulsionServo  50      // Avec les boutons de navigation du menu, on pourra modifier les valeurs par pas de 50 µs


// Constantes concernant l'écran OLED
#define nombreDePixelsEnLargeurEcranOLED    128     // Taille de l'écran OLED, en pixel, au niveau de sa largeur
#define nombreDePixelsEnHauteurEcranOLED    64      // Taille de l'écran OLED, en pixel, au niveau de sa hauteur
#define brocheResetOLED                     -1      // Reset de l'écran OLED partagé avec l'Arduino, d'où la valeur à -1 ici (et non un numéro de pin)
#define adresseI2CdeLecranOLED              0x3C    // Adresse i2c de l'écran OLED (généralement 0x3C par défaut, sinon 0x3D)


// Constantes concernant l'EEPROM interne à l'Arduino
      // Pour rappel :
      //     - la mémoire d'un Arduino Nano (microcontrôleur ATmega328P, donc) dispose de 1024 octets d’EEPROM (allant de l'adresse eeprom 0 à 1023)
      //     - par défaut, si l'arduino est neuf ou si son EEPROM n'a jamais servi, les cases eeprom contienent la valeur 255
#define adresseMemoireValeurBasseImpulsionServo       0       // On utilisera 2 octets ici, pour pouroir stocker une valeur entre 0 et 65535 (idéal pour nos "~1000 µs")
#define adresseMemoireValeurHauteImpulsionServo       2       // On utilisera 2 octets ici, pour pouvoir stocker une valeur entre 0 et 65535 (idéal pour nos "~2000 µs")
#define adresseMemoireCodeArduinoNeufOuPas            4       // On utilisera 2 octets ici, pour pouvoir stocker un code "aléatoire particulier",qui nous dira si notre
#define valeurParticulierePourDireSiArduinoNeufOuPas  9876    // arduino a déjà servi ou non (toute valeur autre que 2 octets à 255 aurait pu convenir, dans l'absolu)


// Constantes du menu de navigation
#define nombreDeLigneMaxDansMenu            3

// Variables librairies
Servo servomoteur;
Adafruit_SSD1306 ecran_oled(nombreDePixelsEnLargeurEcranOLED, nombreDePixelsEnHauteurEcranOLED, &Wire, brocheResetOLED);


// Autres variables
int ligne_selectionnee_dans_menu = 1;               // Contiendra le numéro de ligne sélectionné dans le menu de l'écran OLED (1=Tmin, 2=Tmax, et 3=Réinitialiser)
bool touches_haut_bas_actives = true;               // Booléen qui dira si les touches haut/bas doivent être actives ou non (selon si on navigue dans le menu, ou si on modifie une valeur)
bool touches_gauche_droite_actives = false;         // Booléen qui dira si les touches gauche/droite doivent être actives ou non (selon si on navigue dans le menu, ou si on modifie une valeur)
bool affichage_menu_principal = true;               // Booléen qui dira si on affiche le menu principal ou l'écran de réinitialisation
bool annuler_reinitialisation = true;               // Booléen qui dire si on a sélectionné l'option "NON" ou "OUI", sur la page "réinitialiser les valeurs"
int valeurMinCouranteServo = 0;                     // Valeur minimale courante pour l'impulsion servo, servant de base au PWM
int valeurMaxCouranteServo = 0;                     // Valeur maximale courante pour l'impulsion servo, servant de base au PWM
int valeurPrecendentePotentiometre = 0;             // Mémorisation de la précédente valeur du potentiomètre, pour ne modifier le signal PWM que si nécessaire


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


    // Récupère les valeurs sauvegardées en EEPROM (ou les initialise, si elles sont absentes)
    recupereValeursEnEEPROM();
    

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

    // Petite pause, avant d'attaquer la boucle loop
    delay(500);

}


// ==================================
// Fonction : recupereValeursEnEEPROM
// ==================================
void recupereValeursEnEEPROM() {

  Serial.println("");
  Serial.println(F("Récupératon des valeurs sauvegardées en EEPROM"));

  // Pour commencer, on lit les 5 premiers octets de la mémoire EEPROM. Pour rappel, ils doivent contenir :
  //      - sur 2 octets : la valeur min de l'impulsion servo sauvegardée
  //      - sur 2 octets : la valeur max de l'impulsion servo sauvegardée
  //      - et sur 1 octet : un code de vérification nous disant si cette application a été tournée sur cet arduino ou non
  //        (si l'arduino est "neuf", on aura la valeur 255 au lieu de la valeur attendue, ce qui nous indiquera qu'il faut initialiser cette mémoire)
  
  int valeurMinImpulsionServoLueEnEEPROM = litValeurIntEnEEPROM(adresseMemoireValeurBasseImpulsionServo);
  int valeurMaxImpulsionServoLueEnEEPROM = litValeurIntEnEEPROM(adresseMemoireValeurHauteImpulsionServo);
  int valeurCodeDeVerificationLuEnEEPROM = litValeurIntEnEEPROM(adresseMemoireCodeArduinoNeufOuPas);

  // 2 cas possibles : le code de vérification est bon, ou pas !
  if(valeurCodeDeVerificationLuEnEEPROM == valeurParticulierePourDireSiArduinoNeufOuPas) {
    // Le code correspond, alors on stocke ces valeurs dans nos variables globales (déclarées tout en haut de ce programme)
    valeurMinCouranteServo = valeurMinImpulsionServoLueEnEEPROM;
    valeurMaxCouranteServo = valeurMaxImpulsionServoLueEnEEPROM;
    Serial.print(F("--> valeurs récupérées avec succès (min="));
    Serial.print(valeurMinImpulsionServoLueEnEEPROM);
    Serial.print(F("/max="));
    Serial.print(valeurMaxImpulsionServoLueEnEEPROM);
    Serial.println(F(")"));
  } else {
    // Le code ne correspond pas, alors on stocke ces valeurs dans les variables globales
    valeurMinCouranteServo = valeurDefautSeuilBasImpulsionServo;
    valeurMaxCouranteServo = valeurDefautSeuilHautImpulsionServo;
    // et on initialise la mémoire EEPROM avec
    ecritValeurIntEnEEPROM(adresseMemoireValeurBasseImpulsionServo, valeurDefautSeuilBasImpulsionServo);
    ecritValeurIntEnEEPROM(adresseMemoireValeurHauteImpulsionServo, valeurDefautSeuilHautImpulsionServo);
    ecritValeurIntEnEEPROM(adresseMemoireCodeArduinoNeufOuPas, valeurParticulierePourDireSiArduinoNeufOuPas);
    Serial.println(F("--> valeurs initialisées (1ère fois que ce programme est utilisé sur cet arduino)"));
  }
  Serial.println("");
}


// ===============================
// Fonction : litValeurIntEnEEPROM
// ===============================
int litValeurIntEnEEPROM(int adresse)
{
  // Une valeur "int" est de type 16 bits ; elle prend donc 2 octets
  byte octet1 = EEPROM.read(adresse);               // 8 bits de poids fort en premier
  byte octet2 = EEPROM.read(adresse + 1);           // 8 bits de poids faible ensuite
  return (octet1 << 8) + octet2;
}

// =================================
// Fonction : ecritValeurIntEnEEPROM
// =================================
void ecritValeurIntEnEEPROM(int adresse, int valeur)
{
  // Une valeur "int" est de type 16 bits ; elle prend donc 2 octets
  EEPROM.write(adresse, valeur >> 8);               // 8 bits de poids fort d'abord
  EEPROM.write(adresse + 1, valeur & 0xFF);         // 8 bits de poids faible après
}



// ================================================
// Fonction : contruitEtRafraichiAffichageEcranOLED
// ================================================
void contruitEtRafraichiAffichageEcranOLED() {

  // Effaçage de la mémoire tampon de l'écran OLED
  ecran_oled.clearDisplay();                           

  // Titre
  ecran_oled.setTextColor(WHITE);
  ecran_oled.setTextSize(2);
  ecran_oled.setCursor(10, 0);
  ecran_oled.println("SERVOTEST");

  if(affichage_menu_principal) {

    // ==== Affichage "MENU PRINCIPAL"

    // Ligne #1 du menu
    ecran_oled.setTextSize(1);
    ecran_oled.setCursor(10, 25);  ecran_oled.println("T(min) :");
    ecran_oled.setCursor(99, 25);  ecran_oled.println("us");
    if(ligne_selectionnee_dans_menu == 1 && touches_gauche_droite_actives == true) {
      ecran_oled.setTextColor(BLACK, WHITE);
    }
    ecran_oled.setCursor(70, 25);  ecran_oled.println(valeurMinCouranteServo);
    
    // Ligne #2 du menu
    ecran_oled.setTextColor(WHITE);
    ecran_oled.setCursor(10, 35);  ecran_oled.println("T(max) :");
    ecran_oled.setCursor(99, 35);  ecran_oled.println("us");
    if(ligne_selectionnee_dans_menu == 2 && touches_gauche_droite_actives == true) {
      ecran_oled.setTextColor(BLACK, WHITE);
    }
    ecran_oled.setCursor(70, 35);  ecran_oled.println(valeurMaxCouranteServo);
  
    // Ligne #3 du menu 
    ecran_oled.setTextColor(WHITE);
    ecran_oled.setCursor(10, 55);
    ecran_oled.println("Reinitialiser");
  
    // Curseur (symbole ">")
    switch(ligne_selectionnee_dans_menu) {
      case 1:
        ecran_oled.setCursor(0, 25);
        break;
      case 2:
        ecran_oled.setCursor(0, 35);
        break;
      case 3:
        ecran_oled.setCursor(0, 55);
        break;
      default:
        ecran_oled.setCursor(0, 85);    // En dehors de l'écran, donc
        break;
    }
    ecran_oled.println(">");
  } else {

    // ==== Affichage "ÉCRAN DE RÉINITIALISATION VALEURS"

    ecran_oled.setTextSize(1);
    ecran_oled.setTextColor(WHITE);
    ecran_oled.setCursor(11, 25);  ecran_oled.println("Reinitialiser les");
    ecran_oled.setCursor(3, 35);  ecran_oled.println("valeurs par defaut ?");
    if(annuler_reinitialisation) {
      ecran_oled.setTextColor(BLACK, WHITE);
      ecran_oled.setCursor(11, 55);  ecran_oled.println(" NON ");
      ecran_oled.setTextColor(WHITE);
      ecran_oled.setCursor(86, 55);  ecran_oled.println(" OUI ");
    } else {
      ecran_oled.setTextColor(WHITE);
      ecran_oled.setCursor(11, 55);  ecran_oled.println(" NON ");
      ecran_oled.setTextColor(BLACK, WHITE);
      ecran_oled.setCursor(86, 55);  ecran_oled.println(" OUI ");
    }
  }

  // Affichage (transfert de la mémoire tampon à l'écran OLED)
  ecran_oled.display();
}

// ==================================
// genereEtAjusteSignalPWMservomoteur
// ==================================
void genereEtAjusteSignalPWMservomoteur() {

  // Ajuste PWM en sortie, en fonction de la valeur du potentiomètre, et des bornes hautes/basses paramétrées
  int valPotentiometre = analogRead(pinArduinoRaccordeeAuPotentiometre);    // Valeur 10 bits (0..1023)
  int valArrondiePot = ((int)(valPotentiometre/25)) * 25;
  if (valArrondiePot != valeurPrecendentePotentiometre) {
      Serial.print(F("Valeur arrondie du potentiomètre = "));
      Serial.print(valArrondiePot);
      Serial.println(F("/1000"));   // 1000 et non 1023, du fait de l'arrondi sans virgule /25 * 25 ci-dessus
      int dureeImpulsionPWM = map(valArrondiePot, 0, 1000, valeurMinCouranteServo, valeurMaxCouranteServo);
      servomoteur.writeMicroseconds(dureeImpulsionPWM);
      Serial.print(F("Durée impulsion haute PWM = "));
      Serial.print(dureeImpulsionPWM);
      Serial.println(F(" µs"));
      valeurPrecendentePotentiometre = valArrondiePot;
  }

}


// =================
// Boucle principale
// =================
void loop() {

  // On lit l'état du bouton du centre (noté "OK")
  bool etatBoutonNavigationCentre = !digitalRead(pinArduinoRaccordeeAuBoutonDuCentre);  // Niveau HAUT si inactif ; niveau BAS si appuyé

  // On regarde si les touches HAUT/BAS sont actives (qui signifie qu'on navigue dans le "menu principal")
  if(touches_haut_bas_actives) {

    // On récupère l'état des touches haut/bas
    bool etatBoutonNavigationHaut = !digitalRead(pinArduinoRaccordeeAuBoutonDuHaut);    // Niveau HAUT si inactif ; niveau BAS si appuyé
    bool etatBoutonNavigationBas = !digitalRead(pinArduinoRaccordeeAuBoutonDuBas);

    // Si appui vers le haut, alors on remonte dans le menu (on "décrémente" le numéro de ligne sélectionné, donc)
    if(etatBoutonNavigationHaut) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDuHaut));                           // Attente qu'il repasse au niveau haut
      ligne_selectionnee_dans_menu = ligne_selectionnee_dans_menu - 1;
      if(ligne_selectionnee_dans_menu < 1) {
        ligne_selectionnee_dans_menu = 1;
      }
      delay(20);      // Anti-rebond "logiciel" basique
    }

    // Si appui vers le bas, alors on descend dans le menu (on "incrémente" le numéro de ligne sélectionné, donc)
    if(etatBoutonNavigationBas) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDuBas));                            // Attente qu'il repasse au niveau haut
      ligne_selectionnee_dans_menu = ligne_selectionnee_dans_menu + 1;
      if(ligne_selectionnee_dans_menu > nombreDeLigneMaxDansMenu) {
        ligne_selectionnee_dans_menu = nombreDeLigneMaxDansMenu;
      }
      delay(20);      // Anti-rebond "logiciel" basique
    }
  }

  if(touches_gauche_droite_actives) {

    // On récupère l'état des touches gauche/droite
    bool etatBoutonNavigationGauche = !digitalRead(pinArduinoRaccordeeAuBoutonDeGauche);    // Niveau HAUT si inactif ; niveau BAS si appuyé
    bool etatBoutonNavigationDroite = !digitalRead(pinArduinoRaccordeeAuBoutonDeDroite);

    // Si appui bouton de gauche
    if(etatBoutonNavigationGauche) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDeGauche));                             // Attente qu'il repasse au niveau haut
      if(ligne_selectionnee_dans_menu == 1) {
        valeurMinCouranteServo = valeurMinCouranteServo - pasDeModificationDelaiImpulsionServo;
        if(valeurMinCouranteServo < valeurMinSeuilBasImpulsionServo) {
          valeurMinCouranteServo = valeurMinSeuilBasImpulsionServo;
        }
      }
      if(ligne_selectionnee_dans_menu == 2) {
        valeurMaxCouranteServo = valeurMaxCouranteServo - pasDeModificationDelaiImpulsionServo;
        if(valeurMaxCouranteServo < valeurMinSeuilHautImpulsionServo) {
          valeurMaxCouranteServo = valeurMinSeuilHautImpulsionServo;
        }
      }
      if(ligne_selectionnee_dans_menu == 3) {
        annuler_reinitialisation = true;
      }
      delay(20);      // Anti-rebond "logiciel" basique
    }

    // Si appui bouton de droite
    if(etatBoutonNavigationDroite) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDeDroite));                             // Attente qu'il repasse au niveau haut
      if(ligne_selectionnee_dans_menu == 1) {
        valeurMinCouranteServo = valeurMinCouranteServo + pasDeModificationDelaiImpulsionServo;
        if(valeurMinCouranteServo > valeurMaxSeuilBasImpulsionServo) {
          valeurMinCouranteServo = valeurMaxSeuilBasImpulsionServo;
        }
      }
      if(ligne_selectionnee_dans_menu == 2) {
        valeurMaxCouranteServo = valeurMaxCouranteServo + pasDeModificationDelaiImpulsionServo;
        if(valeurMaxCouranteServo > valeurMaxSeuilHautImpulsionServo) {
          valeurMaxCouranteServo = valeurMaxSeuilHautImpulsionServo;
        }
      }
      if(ligne_selectionnee_dans_menu == 3) {
        annuler_reinitialisation = false;
      }
      delay(20);      // Anti-rebond "logiciel" basique
    }
  }

  // Si appui sur le bouton du centre ("OK")
  if(etatBoutonNavigationCentre) {
    
    if(touches_haut_bas_actives) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDuCentre));                         // Attente qu'il repasse au niveau haut
      touches_gauche_droite_actives = true;
      touches_haut_bas_actives = false;
      if(ligne_selectionnee_dans_menu == 3) {
        affichage_menu_principal = false;
        annuler_reinitialisation = true;
      }
      delay(20);      // Anti-rebond "logiciel" basique
    } else if(touches_gauche_droite_actives) {
      while(!digitalRead(pinArduinoRaccordeeAuBoutonDuCentre));                         // Attente qu'il repasse au niveau haut
      touches_gauche_droite_actives = false;
      touches_haut_bas_actives = true;
      if(ligne_selectionnee_dans_menu == 1) {
        ecritValeurIntEnEEPROM(adresseMemoireValeurBasseImpulsionServo, valeurMinCouranteServo);
      }
      if(ligne_selectionnee_dans_menu == 2) {
        ecritValeurIntEnEEPROM(adresseMemoireValeurHauteImpulsionServo, valeurMaxCouranteServo);
      }
      if(ligne_selectionnee_dans_menu == 3) {
        if(annuler_reinitialisation == false) {
          // Si la réinitialisation a été demandée, alors on réinitialise les valeurs par défaut (variables globales + EEPROM)
          ecritValeurIntEnEEPROM(adresseMemoireValeurBasseImpulsionServo, valeurDefautSeuilBasImpulsionServo);
          ecritValeurIntEnEEPROM(adresseMemoireValeurHauteImpulsionServo, valeurDefautSeuilHautImpulsionServo);
          valeurMinCouranteServo = valeurDefautSeuilBasImpulsionServo;
          valeurMaxCouranteServo = valeurDefautSeuilHautImpulsionServo;
          ligne_selectionnee_dans_menu = 1;     // On remet le "curseur" sur la 1ère ligne du menu principal       
        }
        affichage_menu_principal = true;
      }
      valeurPrecendentePotentiometre = -1;    // On force le recalcul du signal PWM, avec ces nouvelles valeurs
      delay(20);      // Anti-rebond "logiciel" basique
    }
  } 
   
  // Contruit/rafraîchi affichage écran OLED
  contruitEtRafraichiAffichageEcranOLED();

  // Génère/ajuste le signal PWM en sortie (à destination du servomoteur)
  genereEtAjusteSignalPWMservomoteur();



  // Puis on reboucle … à l'infini !
}
