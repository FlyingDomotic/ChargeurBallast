#define CODE_VERSION "V26.1.7-1"

/*

Contrôle d'un chargeur de ballast pour train miniature

	Ce code contrôle un chargeur de ballast dans des wagons adéquats.

	Il est basé sur une trémie commandée par 2 relais à impulsion.

	Le premier relais ouvre la trémie alors que le second la ferme.

	Un rail a été équipé de 10 ILS régulièrement espacés, afin de détecter l'aimant placé sous chaque wagon.

	Lorsqu'un wagon passe au dessus de l'ILS d'ouverture (paramétrable), une impulsion (de durée réglable)
		est envoyée au relais d'ouverture. La même chose est faite lorsque l'ILS de fermeture est
		activé. De plus, la trémie est automatiquement fermée après un temps paramétrable, afin d'éviter
		de vider son contenu sur la voie en cas de souci (train bloqué, problème d'ILS, ...)

	A la fin d'un cycle d'ouverture/fermeture, l'Arduino affiche l'écart entre la vitesse réelle de
		chargement et la vitesse optimale. Si on modifie la vitesse du train en proportion, on aura
		un chargement parfait.

	Les réglages sont envoyés à l'Arduino au travers de sa liaison série. Ce même moyen est utilisé pour
		envoyer les messages à l'utilisateur.

	Les réglages sont mémorisés dans l'EEPROM de l'Arduino afin d'être disponibles après son redémarrage.
	
Hardware Arduino Nano:
	- 10 entrées ILS
	- 2 sorties relais
	- interface série pour paramétrage :
		- définition numéro ILS activation trémie(A1 à A10)
		- définition numéro ILS désactivation trémie (D1 à D10)
		- définition durée impulsion en 1/100eme de seconde (T1 à T99)
		- définition durée remplissage en 1/10eme de seconde (R1 à R99)
		- sauvegarde paramètres en EEPROM
		- affichage état des ILS (E)
		- affichage aide (?)
		- commande d'ouverture de trémie (O)
		- commande de fermeture (F)
        - (ré)initialisation (INIT)
        - fonction marche/arrêt (M)
        - bascule debug (P)

Principe utilisé :
	- le temps de remplissage d'un wagon est fixe, lié à la section de la trémie
	- on commence à remplir en passant sur un ILS
	- on termine le remplissage en passant sur un autre ILS
	- la vitesse du train doit être telle que le temps entre les 2 ILS soit celui du remplissage

Paramétrage :
    - on cherche la durée d'impulsion nécessaire pour ouvrir ou fermer la trémie à coup sur, sans faire chauffer les bobines,
        en augmentant/réduisant la valeur du paramètre T et utilisant les commandes O et F pour ouvrir/fermer la trémie
	- on mesure le temps de remplissage d'un wagon au chrono, avec avancée du wagon à la main (paramètre R, valeur 1 à 99, en 1/10e de seconde)
	- on repère le numéro de l'ILS début de remplissage  (paramètre A, valeur 1 à 10)
	- on repère le numéro de l'ILS fin de remplissage  (paramètre D, valeur 1 à 10)
	- on ajuste la vitesse de la loco pour que le temps entre les 2 ILS soit celui de remplissage, aidé par le retour de l'Arduino
		sur l'écart avec la vitesse idéale (ajouter le pourcentage donné par l'Arduino à la vitesse courante de la loco pour être parfait)

Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré A, valeur 1 à 10
	- déclenche la fermeture sur la détection de l'ILS paramétré D, valeur 1 à 10
		ou l'expiration du temps de remplissage paramétré R, valeur 1 à 99 en 1/10e de seconde (pour éviter un débordement)
	- la durée du déclenchement des bobines est fixée en 1/100 s paramètre T, valeur 1 à 99)
	- affiche en % la différence entre la durée écoulée entre A et D et celle de R
	- répond aux commandes sur le port série

Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007

*/

// Includes
#include "Arduino.h"
#include "EEPROM.h"

//  Parameters
uint8_t activationIls;                                              // A: ILS number to activate filling
uint8_t deactivationIls;                                            // D: ILS number to deactivate filling
uint8_t fillingTime;                                                // R: time (0.1s) to fill wagon
uint8_t pulseTime;                                                  // T: time (0.01s) to send current to relay
bool isActive = false;                                              // When active flag is true, relays are triggered by ILS
bool inDebug = false;                                               // Print debug message when true
bool displayIls = false;                                            // When set, continously ILS state

#define FILLING_MULTIPLIER 100                                      // Multiplier to get fillingTime in ms
#define PULSE_MULTIPLIER 10                                         // Multiplier to get pulseTime in ms
#define MAGIC_NUMBER 59                                             // EEPROM magic byte
#define BUFFER_LENGHT 50                                            // Serial input buffer length
#define ILS_CLOSED LOW                                              // State read when ILS is closed
#define RELAY_CLOSED LOW                                            // State to write to close relay
#define RELAY_OPENED HIGH                                           // State to write to open relay
#define DISPLAY_ILS_TIME 100                                        // Display ILS state every xxx ms

uint8_t ilsPinMapping[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};         // Maps ILS number to Arduino PIN number
uint8_t relayPinMapping[] = {12, 13};                               // Maps relay number to Arduino PIN number

// Data
unsigned long activationStartTime = 0;                              // Activation ILS trigger time (0 if none)
unsigned long deactivationStartTime = 0;                            // Deactivation ILS trigger time (0 if none)
unsigned long fillingStartTime = 0;                                 // Filling start time (0 if none)
unsigned long fillingEndTime = 0;                                   // Filling end time (0 if none)
unsigned long relayPulseTime = 0;                                   // Relay plusing start time
unsigned long lastIlsDisplay = 0;                                   // Last time we displayed ILS
char inputBuffer[BUFFER_LENGHT];                                    // Serial input buffer
uint8_t bufferLen = 0;                                              // Used chars in input buffer

// Routines and functions

void displayStatus(void);                                           // Display current status
void loadSettings(void);                                            // Load settings from EEPROM
void saveSettings(void);                                            // Save settings to EEPROM (only if modified)
void initSettings(void);                                            // Init settings to default values
void resetInputBuffer(void);                                        // Reset serial input buffer
void initIO(void);                                                  // Init IO pins
void workWithSerial(void);                                          // Work with Serial input
void startFilling(void);                                            // Start filling
void stopFilling(void);                                             // Stop filling
void displayIlsState(void);                                         // Display all ILS state
void printHelp(void);                                               // Print help message
void toggleRun(void);                                               // Toggle run flag
void toggleDebug(void);                                             // Toggle  debug flag
void reinitAll(void);                                               // Reinitialize all settings
void executeCommand(void);                                          // Execute command read on serial input (a-z and 0-9)
void setup(void);                                                   // Setup
void loop(void);                                                    // Main loop

// Display current status
void displayStatus(void) {
    Serial.print(F("A"));
    Serial.print(activationIls);
    Serial.print(F(" D"));
    Serial.print(deactivationIls);
    Serial.print(F(" R"));
    Serial.print(fillingTime);
    Serial.print(F(" T"));
    Serial.print(pulseTime);
    if (inDebug) Serial.print(F(", debug"));
    Serial.println(isActive ? F(", en marche") : F(", à l'arrêt"));
}

// Load settings from EEPROM
void loadSettings(void) {
    if (EEPROM.read(0) != MAGIC_NUMBER) {                           // Is first byte equal to magic cnumber?
        Serial.print(F("Magic est "));
        Serial.print(EEPROM.read(0));
        Serial.print(F(", pas "));
        Serial.print(MAGIC_NUMBER);
        Serial.println(F("!"));
        return;
    }
    activationIls = EEPROM.read(1);                                 // A: ILS number to activate filling
    deactivationIls = EEPROM.read(2);                               // D: ILS number to deactivate filling
    fillingTime = EEPROM.read(3);                                   // R: time (0.1s) to fill wagon
    pulseTime = EEPROM.read(4);                                     // T: time (0.01s) to send current to relay
    isActive = EEPROM.read(5);                                      // When active flag is true, relays are triggered by ILS
    inDebug = EEPROM.read(6);                                       // Debug flag
}

// Save settings to EEPROM (only if modified)
void saveSettings(void) {
    EEPROM.update(0, MAGIC_NUMBER);                                 // First byte = magic number
    EEPROM.update(1, activationIls);                                // A: ILS number to activate filling
    EEPROM.update(2, deactivationIls);                              // D: ILS number to deactivate filling
    EEPROM.update(3, fillingTime);                                  // R: time (0.1s) to fill wagon
    EEPROM.update(4, pulseTime);                                    // T: time (0.01s) to send current to relay
    EEPROM.update(5, isActive);                                     // When active flag is true, relays are triggered by ILS
    EEPROM.update(6, inDebug);                                      // Debug flag
}

// Init settings to default values
void initSettings(void) {
    activationIls = 0;                                              // A: ILS number to activate filling
    deactivationIls = 0;                                            // D: ILS number to deactivate filling
    fillingTime = 0;                                                // R: time (0.1s) to fill wagon
    pulseTime = 20;                                                 // T: time (0.01s) to send current to relay
    isActive = false;                                               // When active flag is true, relays are triggered by ILS
}

// Reset serial input buffer
void resetInputBuffer(void) {
    memset(inputBuffer, 0, sizeof(inputBuffer));
    bufferLen = 0;
}

// Init IO pins
void initIO(void) {
    // Set ILS PIN to input with pullup resistor
    for (uint8_t i = 0; i < 10; i++) {
        pinMode(ilsPinMapping[i], INPUT_PULLUP);
    }
    // Set relay PIN to output, init level = opened
    for (uint8_t i = 0; i < 2; i++) {
        digitalWrite(relayPinMapping[i], RELAY_OPENED);
        pinMode(relayPinMapping[i], OUTPUT);
    }
}

// Work with Serial input
void workWithSerial(void) {
    // Reset display ILS flag
    if (displayIls) {
        Serial.println(F(""));
        displayIls = false;
    }
    // Read serial input
    while (Serial.available()) {
        // Read one character
        char c = Serial.read();
        // Is this a return ?
        if (c == 13) {
            executeCommand();
            resetInputBuffer();
        } else if (c) {
            // Non null character
            // Convert uppercases to lowercase
            if (c >= 'A' && c <= 'Z') {
                c += 32;
            }
            // Keep only "a" to "z" and "0" to "9"
            if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
                if (bufferLen >= BUFFER_LENGHT - 1) {
                    Serial.println(F("Buffer plein - Reset!"));
                    resetInputBuffer();
                }
                inputBuffer[bufferLen++] = c;
            }
        }
    }
}

// Start filling
void startFilling(void) {
    unsigned long now = millis();
    if (inDebug) {Serial.print(F("Début chargement à ")); Serial.println(now);}
    digitalWrite(relayPinMapping[1], RELAY_OPENED);
    digitalWrite(relayPinMapping[0], RELAY_CLOSED);
    fillingStartTime = now;
    relayPulseTime = now;
}

// Stop filling
void stopFilling(void) {
    unsigned long now = millis();
    if (inDebug) {Serial.print(F("Fin chargement à ")); Serial.print(now); Serial.print(F(", durée "));Serial.println(now - fillingStartTime);}
    digitalWrite(relayPinMapping[0], RELAY_OPENED);
    digitalWrite(relayPinMapping[1], RELAY_CLOSED);
    fillingEndTime = now;
    relayPulseTime = now;
}

// Display all ILS state
void displayIlsState(void) {
    displayIls = true;
}

// Print help message
void printHelp(void) {
    Serial.println(F("A1-10: numéro ILS activation"));
    Serial.println(F("D1-10: numéro ILS désactivation"));
    Serial.println(F("R1-99: durée remplissage (0,1 s)"));
    Serial.println(F("T1-99: durée impulsion relais (0,01 s)"));
    Serial.println(F("O: ouvre trémie"));
    Serial.println(F("F: ferme trémie"));
    Serial.println(F("E: état ILS"));
    Serial.println(F("M: bascule l'état marche/arrêt"));
    Serial.println(F("P: bascule l'état debug"));
    Serial.println(F("init: (ré)initialise tout"));
    Serial.println(F("?: cette aide"));
}

// Toggle run flag
void toggleRun(void){
    isActive = !isActive;
    saveSettings();
}

// Toggle  debug flag
void toggleDebug(void){
    inDebug = !inDebug;
    saveSettings();
}

// Reinitialize all settings
void reinitAll(void) {
    stopFilling();
    initSettings();
    saveSettings();
}

// Execute command read on serial input (a-z and 0-9)
void executeCommand(void) {
    Serial.print(F("Reçu: "));
    Serial.println(inputBuffer);
    if (!strcmp(inputBuffer, "init")) {
        reinitAll();
    } else if (!strcmp(inputBuffer, "o")) {
        startFilling();
    } else if (!strcmp(inputBuffer, "f")) {
        stopFilling();
    } else if (!strcmp(inputBuffer, "e")) {
        displayIlsState();
    } else if (!strcmp(inputBuffer, "m")) {
        toggleRun();
    } else if (!strcmp(inputBuffer, "p")) {
        toggleDebug();
    } else if (inputBuffer[0] == 'a') {
        if (strlen(inputBuffer) >= 2 && strlen(inputBuffer) <= 3) {
            inputBuffer[0] = '0';
            uint16_t value = atoi(inputBuffer);
            if (value >= 1 && value <= 10) {
                activationIls = value;
                saveSettings();
            } else {
                Serial.println(F("A1-10 seulement!"));
            }
        } else {
            Serial.println(F("Nombre incorrect!"));
        }
    } else if (inputBuffer[0] == 'd') {
        if (strlen(inputBuffer) >= 2 && strlen(inputBuffer) <= 3) {
            inputBuffer[0] = '0';
            uint16_t value = atoi(inputBuffer);
            if (value >= 1 && value <= 10) {
                deactivationIls = value;
                saveSettings();
            } else {
                Serial.println(F("D1-10 seulement!")); 
            }
        } else {
            Serial.println(F("Nombre incorrect!"));
        }
    } else if (inputBuffer[0] == 'r') {
        if (strlen(inputBuffer) >= 2 && strlen(inputBuffer) <= 3) {
            inputBuffer[0] = '0';
            uint16_t value = atoi(inputBuffer);
            if (value >= 1 && value <= 99) {
                fillingTime = value;
                saveSettings();
            } else {
                Serial.println(F("R1-99 seulement!"));
            }
        } else {
            Serial.println(F("Nombre incorrect!"));
        }
    } else if (inputBuffer[0] == 't') {
        if (strlen(inputBuffer) >= 2 && strlen(inputBuffer) <= 3) {
            inputBuffer[0] = '0';
            uint16_t value = atoi(inputBuffer);
            if (value >= 1 && value <= 99) {
                pulseTime = value;
                saveSettings();
            } else {
                Serial.println(F("R1-99 seulement!"));
            }
        } else {
            Serial.println(F("Nombre incorrect!"));
        }
    } else {
        printHelp();
    }
    displayStatus();
}

// Setup
void setup(void){
    initIO();
    Serial.begin(115200);
    Serial.println();
    Serial.print(F("Chargeur ballast "));
    Serial.print(CODE_VERSION);
    Serial.println(F(" lancé..."));
    resetInputBuffer();
    initSettings();
    loadSettings();
    stopFilling();
    displayStatus();
}

// Main loop
void loop(void){
    unsigned long now = millis();

    // Do we need to close any relay after pulse?
    if (relayPulseTime != 0 && ((now - relayPulseTime) >= (pulseTime * PULSE_MULTIPLIER))) {
        if (inDebug) {Serial.print(F("Fin impulsion à ")); Serial.print(now); Serial.print(F(", durée "));Serial.println(now - relayPulseTime);}
        for (uint8_t i = 0; i < 2; i++) {
            digitalWrite(relayPinMapping[i], RELAY_OPENED);
        }
        relayPulseTime = 0;        
    }

    // Do we need to stop filling after a too long time opened?
    if (fillingEndTime == 0 && ((now - fillingStartTime) >= (fillingTime * FILLING_MULTIPLIER))) {
        stopFilling();
    };

    // Do job if we're active
    if (isActive) {
        // Check for activation ILS
        if ((activationStartTime == 0) && (digitalRead(ilsPinMapping[activationIls]) == ILS_CLOSED)) {
            if (inDebug) {Serial.println(F("Début ILS à ")); Serial.println(now);}
            activationStartTime = now;
            startFilling();
        }
        // Check for deactivation ILS
        if ((activationStartTime != 0) && (deactivationStartTime == 0) && (digitalRead(ilsPinMapping[deactivationIls]) == ILS_CLOSED)) {
            deactivationIls = now;
            stopFilling();
            // Compute between ILS duration (max 65 seconds)
            uint16_t ilsDuration = deactivationStartTime - activationStartTime;
            if (inDebug) {Serial.println(F("Fin ILS à ")); Serial.print(now); Serial.print(F(", durée ")); Serial.println(ilsDuration);}
            uint16_t idealDuration = fillingTime * FILLING_MULTIPLIER;
            // Avoid error if fillingTime is zero (not defined)
            if (idealDuration) {
                // Compute percent to perfect speed
                long percentToPerfectSpeed = (100 * (idealDuration - ilsDuration)) / idealDuration;
                Serial.print(F("Corriger vitesse de "));
                // Force a "+" if positive
                if (percentToPerfectSpeed > 0) {
                    Serial.print(F("+"));
                }
                Serial.print(percentToPerfectSpeed);
                Serial.println(F("%"));
            }
            activationStartTime = 0;
            deactivationStartTime = 0;
            fillingStartTime = 0;
            fillingEndTime = 0;
        }
    }

    // Scan serial for input
    if (Serial.available()) {
        workWithSerial();
    }

    // Display ILS state if needed
    if (displayIls) {
        if ((millis() - lastIlsDisplay) > DISPLAY_ILS_TIME) {
            Serial.print (F("\rILS : "));
            for (uint8_t i = 0; i < 10; i++) {
                if (digitalRead(ilsPinMapping[i]) == ILS_CLOSED) {
                    Serial.print(i+1);
                } else {
                    Serial.print(F("-"));
                }
                Serial.print(F(" "));
            }
            Serial.print(F(" "));
            lastIlsDisplay = millis();
        }
    }
}
