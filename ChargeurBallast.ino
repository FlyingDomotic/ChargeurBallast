#define CODE_VERSION "V26.1.12-2"

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

	Il arrive que le granulat bloque la sortie de la trémie. Pour contrer ce problème, on peut définir
        des cycles fermetures/ouvertures de la trémie, les vibration des relais de commande de la trappe
        décoinçant le granulat. Ces cycles peuvent être paramétrés si besoin.
    
    Les réglages sont envoyés à l'Arduino au travers de sa liaison série. Ce même moyen est utilisé pour
		envoyer les messages à l'utilisateur.

	Les réglages sont mémorisés dans l'EEPROM de l'Arduino afin d'être disponibles après son redémarrage.
	
Hardware Arduino Nano:
	- 10 entrées ILS
	- 2 sorties relais
	- interface série pour paramétrage :
		- définition numéro ILS activation trémie(ILO1 à ILO10)
		- définition numéro ILS désactivation trémie (ILF1 à IFL10)
		- définition durée impulsion en 1/1000eme de seconde (DRW1 à DRW999)
		- définition durée remplissage en 1/1000eme de seconde (DIR1 à DIR9999)
        - définition de durée d'attente trémie ouverte avant fermeture (AOV0 à AV999) (vibrations)
        - définition de la durée de l'impulsion de fermeture (IFV0 à IFV999) (vibrations)
        - définition de la durée d'attente trémie fermée avant ouverture (AFV0 à AFV999) (vibrations)
        - définition de la durée de l'impulsion d'ouverture (IOV1 à IOV99) (vibrations)
		- affichage état des ILS (EI)
		- affichage aide (?)
		- commande d'ouverture de trémie (OT)
		- commande de fermeture (FT)
        - (ré)initialisation (INIT)
        - fonction marche/arrêt (M)
        - bascule debug (BD)
        - sauvegarde paramètres en EEPROM

Principe utilisé :
	- le temps de remplissage d'un wagon est fixe, lié à la section de la trémie
	- on commence à remplir en passant sur un ILS
	- on termine le remplissage en passant sur un autre ILS
	- la vitesse du train doit être telle que le temps entre les 2 ILS soit celui du remplissage

Paramétrage :
    - on cherche la durée d'impulsion nécessaire pour ouvrir ou fermer la trémie à coup sur, sans faire chauffer les bobines,
        en augmentant/réduisant la valeur du paramètre DIR et utilisant les commandes OT et FT pour ouvrir/fermer la trémie
	- on mesure le temps de remplissage d'un wagon au chrono, avec avancée du wagon à la main (paramètre DRW, valeur 1 à 9999, en millisecondes)
	- on repère le numéro de l'ILS début de remplissage  (paramètre ILO, valeur 1 à 10)
	- on repère le numéro de l'ILS fin de remplissage  (paramètre ILF, valeur 1 à 10)
	- on ajuste la vitesse de la loco pour que le temps entre les 2 ILS soit celui de remplissage, aidé par le retour de l'Arduino
		sur l'écart avec la vitesse idéale (ajouter le pourcentage donné par l'Arduino à la vitesse courante de la loco pour être parfait)
    - si on souhaite utiliser les vibrations :
        - l'idée est d'utiliser les vibrations des bobines de fermeture et d'ouverture pour décoincer le granulat.
        - tant que la trémie doit être ouverte (fixé par DIR), on active le cycle suivant :
            - la trémie reste ouverte AOV ms
            - on envoie une impulsion de fermeture d'IFV ms (qui peut être zéro ou très courte)
            - la trémie reste fermée AFV ms (qui peut être zéro pour réouvrir immédiatement)
            - on envoie une impulsion d'ouverture d'IOV ms
        - tatonner pour voir quel est le meilleur compromis selon le granulat
        - on désactive les vibrations en mettant IFV et IOV à zéro

Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré ILO, valeur 1 à 10
	- déclenche la fermeture sur la détection de l'ILS paramétré ILF, valeur 1 à 10
		ou l'expiration du temps de remplissage paramétré DRW, valeur 1 à 9999 en millisecondes
	- la durée du déclenchement des bobines est fixée par le paramètre DIR, valeur 1 à 999 en millisecondes
	- affiche en % la différence entre la durée écoulée entre ILO et ILF et celle de DRW
    - on gère les vibrations pendant la durée de remplissage du wagon si besoin
	- répond aux commandes sur le port série

Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007

*/

// Includes
#include "Arduino.h"
#include "EEPROM.h"

// Command and text

#define ILSOPEN_COMMAND "ILO"
#define ILSCLOSE_COMMAND "ILF"
#define RELAYPULSEDURATION_COMMAND "DIR"
#define WAGONFILLDURATION_COMMAND "DRW"
#define START_COMMAND "M"
#define STOP_COMMAND "A"
#define ILSSTATE_COMMAND "EI"
#define OPENVIBRATIONPULSE_COMMAND "IOV"
#define CLOSEVIBRATIONPULSE_COMMAND "IFV"
#define OPENVIBRATIONWAIT_COMMAND "AOV"
#define CLOSEVIBRATIONWAIT_COMMAND "AFV"
#define OPENRELAY_COMMAND "OT"
#define CLOSERELAY_COMMAND "FT"
#define DEBUGTOGGLE_COMMAND "BD"
#define INIT_COMMAND "INIT"

//  Parameters

#define MAGIC_NUMBER 59                                             // EEPROM magic byte
#define EEPROM_VERSION 2                                            // EEPROM version
#define BUFFER_LENGHT 50                                            // Serial input buffer length
#define ILS_CLOSED LOW                                              // State read when ILS is closed
#define RELAY_CLOSED LOW                                            // State to write to close relay
#define RELAY_OPENED HIGH                                           // State to write to open relay
#define DISPLAY_ILS_TIME 100                                        // Display ILS state every xxx ms
#define OPEN_RELAY 0                                                // Index of open relay into relayPinMapping
#define CLOSE_RELAY 1                                               // Index of close relay into relayPinMapping

// EEPROM data (current version)
struct eepromData_s {
    uint8_t magicNumber;                                            // Magic number
    uint8_t version;                                                // Structure version
    uint8_t activationIls;                                          // ILS number to activate filling
    uint8_t deactivationIls;                                        // ILS number to deactivate filling
    uint16_t fillingTime;                                           // Time (0.001s) to fill wagon
    uint16_t pulseTime;                                             // Time (0.001s) to send current to relay
    bool isActive;                                                  // When active flag is true, relays are triggered by ILS
    bool inDebug;                                                   // Print debug message when true
    uint16_t openVibrationWait;                                     // Time to wait in open state for vibrations
    uint16_t closeVibrationWait;                                    // Time to wait in closed state for vibrations
    uint16_t openVibrationPulse;                                    // Open pulse duration for vibration
    uint16_t closeVibrationPulse;                                   // Close pulse duration for vibration
};

// EEPROM data (V1 version)
struct eepromDataV1_s {
    uint8_t magicNumber;                                            // Magic number
    uint8_t version;                                                // Structure version
    uint8_t activationIls;                                          // ILS number to activate filling
    uint8_t deactivationIls;                                        // ILS number to deactivate filling
    uint16_t fillingTime;                                           // Time (0.001s) to fill wagon
    uint16_t pulseTime;                                             // Time (0.001s) to send current to relay
    bool isActive;                                                  // When active flag is true, relays are triggered by ILS
    bool inDebug;                                                   // Print debug message when true
};

bool displayIls = false;                                            // When set, continously display ILS state
uint8_t ilsPinMapping[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};         // Maps ILS number to Arduino PIN number
uint8_t relayPinMapping[] = {12, 13};                               // Maps relay number to Arduino PIN number

// Vibration states
typedef enum {
    vibrationOff = 0,
    waitOpened,
    pulseClose,
    waitClosed,
    pulseOpen
} vibrationState_t;

// Data
unsigned long fillingStartTime = 0;                                 // Filling start time (0 if none)
unsigned long fillingEndTime = 0;                                   // Filling end time (0 if none)
unsigned long relayPulseTime = 0;                                   // Relay plusing start time
unsigned long lastIlsDisplay = 0;                                   // Last time we displayed ILS
unsigned long vibrationTime = 0;                                    // Last vibration change time
unsigned long vibrationDuration = 0;                                // duration before next step
uint16_t commandValue;                                              // Value extracted from command
uint8_t bufferLen = 0;                                              // Used chars in input buffer
char inputBuffer[BUFFER_LENGHT];                                    // Serial input buffer
vibrationState_t vibrationState = vibrationOff;                     // Where we are in vibration
eepromData_s data;                                                  // Data stored to/read from EEPROM

// Routines and functions

void displayStatus(void);                                           // Display current status
void loadSettings(void);                                            // Load settings from EEPROM
void saveSettings(void);                                            // Save settings to EEPROM (only if modified)
void initSettings(void);                                            // Init settings to default values
void resetInputBuffer(void);                                        // Reset serial input buffer
void initIO(void);                                                  // Init IO pins
void workWithSerial(void);                                          // Work with Serial input
bool isCommand(char* inputBuffer, char* commandToCheck);            // Check command without value
bool isCommandValue(char* inputBuffer, char* commandToCheck, uint16_t minValue, uint16_t maxValue); // Check command with value
void startFilling(void);                                            // Start filling
void stopFilling(void);                                             // Stop filling
void setVibrationNextStep(void);                                    // Set next step in vibration sequence
void displayIlsState(void);                                         // Display all ILS state
void printHelp(void);                                               // Print help message
void toggleDebug(void);                                             // Toggle  debug flag
void reinitAll(void);                                               // Reinitialize all settings
void executeCommand(void);                                          // Execute command read on serial input (a-z and 0-9)
void setup(void);                                                   // Setup
void loop(void);                                                    // Main loop

// Display current status
void displayStatus(void) {
    Serial.print(F(ILSOPEN_COMMAND));
    Serial.print(data.activationIls);
    Serial.print(F(" "));
    Serial.print(F(ILSCLOSE_COMMAND));
    Serial.print(data.deactivationIls);
    Serial.print(F(" "));
    Serial.print(F(WAGONFILLDURATION_COMMAND));
    Serial.print(data.fillingTime);
    Serial.print(F(" "));
    Serial.print(F(RELAYPULSEDURATION_COMMAND));
    Serial.print(data.pulseTime);
    if (data.openVibrationWait) {
        Serial.print(F(" "));
        Serial.print(F(OPENVIBRATIONWAIT_COMMAND));
        Serial.print(data.openVibrationWait);
    }
    if (data.closeVibrationPulse) {
        Serial.print(F(" "));
        Serial.print(F(CLOSEVIBRATIONPULSE_COMMAND));
        Serial.print(data.closeVibrationPulse);
    }
    if (data.closeVibrationWait) {
        Serial.print(F(" "));
        Serial.print(F(CLOSEVIBRATIONWAIT_COMMAND));
        Serial.print(data.closeVibrationWait);
    }
    if (data.openVibrationPulse) {
        Serial.print(F(" "));
        Serial.print(F(OPENVIBRATIONPULSE_COMMAND));
        Serial.print(data.openVibrationPulse);
    }
    if (data.inDebug) Serial.print(F(", déverminage"));
    Serial.println(data.isActive ? F(", en marche") : F(", à l'arrêt"));
}

// Load settings from EEPROM
void loadSettings(void) {
    initSettings();                                                 // Init data structure
    if (EEPROM.read(0) != MAGIC_NUMBER) {                           // Is first byte equal to magic number?
        Serial.print(F("Magic est "));
        Serial.print(EEPROM.read(0));
        Serial.print(F(", pas "));
        Serial.print(MAGIC_NUMBER);
        Serial.println(F("!"));
        return;
    }

    uint8_t version = EEPROM.read(1);                               // Get version
    if (version == 1) {
        eepromDataV1_s v1Data;                                      // V1 data structure
        EEPROM.get(0, v1Data);                                      // Load EEPROM V1 structure
        data.activationIls = v1Data.activationIls;
        data.deactivationIls = v1Data.deactivationIls;
        data.fillingTime = v1Data.fillingTime;
        data.pulseTime = v1Data.pulseTime;
        data.isActive = v1Data.isActive;
        data.inDebug = v1Data.inDebug;
    } else if (version == 2) {
        EEPROM.get(0, data);                                        // Load EEPROM V2 structure
    } else {
        Serial.print(F("Version est "));
        Serial.print(version);
        Serial.print(F(", pas "));
        Serial.print(EEPROM_VERSION);
        Serial.println(F("!"));
        return;
    }
}

// Save settings to EEPROM (only if modified)
void saveSettings(void) {
    data.magicNumber = MAGIC_NUMBER;                                // Force magic number
    data.version = EEPROM_VERSION;                                  // ... and version
    eepromData_s savedData;                                         // Current EEPROM data
    EEPROM.get(0, savedData);                                       // Get saved data
    if (memcmp(&data, &savedData, sizeof(data))) {                  // Compare full buffers
        EEPROM.put(0, data);                                        // Store structure
    }
}

// Init settings to default values
void initSettings(void) {
    data.activationIls = 0;                                         // ILS number to activate filling
    data.deactivationIls = 0;                                       // ILS number to deactivate filling
    data.fillingTime = 0;                                           // Time to fill wagon
    data.pulseTime = 100;                                           // Time to send current to relay
    data.isActive = false;                                          // When active flag is true, relays are triggered by ILS
    data.inDebug = false;                                           // Print debug message when true
    data.openVibrationWait = 0;                                     // Time to wait in open state for vibrations
    data.closeVibrationWait = 0;                                    // Time to wait in closed state for vibrations
    data.openVibrationPulse = 0;                                    // Open pulse duration for vibration
    data.closeVibrationPulse = 0;                                   // Close pulse duration for vibration
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
        // Is this a return?
        if (c == 13) {
            executeCommand();
            resetInputBuffer();
        } else if (c) {
            // Non null character
            // Keep only "A" to "Z", "a" to "z" and "0" to "9"
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
                if (bufferLen >= BUFFER_LENGHT - 1) {
                    Serial.println(F("Buffer plein - Reset!"));
                    resetInputBuffer();
                }
                inputBuffer[bufferLen++] = c;
            }
        }
    }
}

// Check command without value
bool isCommand(char* inputBuffer, char* commandToCheck) {
    return !strcasecmp(inputBuffer, commandToCheck);
}

// Check command with value
bool isCommandValue(char* inputBuffer, char* commandToCheck, uint16_t minValue, uint16_t maxValue) {
    if (strncmp(inputBuffer, commandToCheck, strlen(commandToCheck))) {
        return false;
    }
    commandValue = atoi(&inputBuffer[strlen(commandToCheck)]);      // Convert given value
    if (commandValue < minValue || commandValue > maxValue) {       // Check limits
        Serial.print(commandValue);
        Serial.print(F(" hors limites "));
        Serial.print(minValue);
        Serial.print(F("-"));
        Serial.print(maxValue);
        Serial.print(F(" pour la commande "));
        Serial.println(commandToCheck);
        return false;
    }
    return true;
}

// Start filling
void startFilling(void) {
    unsigned long now = millis();
    if (data.inDebug) {
        Serial.print(F("Début chargement à "));
        Serial.println(now);
    }
    digitalWrite(relayPinMapping[CLOSE_RELAY], RELAY_OPENED);
    digitalWrite(relayPinMapping[OPEN_RELAY], RELAY_CLOSED);
    fillingStartTime = now;
    relayPulseTime = now;
    // Start vibration if needed
    if (data.closeVibrationPulse || data.openVibrationPulse) {
        vibrationTime = now;
        vibrationDuration = data.openVibrationWait;
        vibrationState = waitOpened;
        if (data.inDebug) {
            Serial.println(F("Début attente ouvert"));
        }
    }
}

// Stop filling
void stopFilling(void) {
    unsigned long now = millis();
    if (data.inDebug) {
        Serial.print(F("Fin chargement à "));
        Serial.print(now);
        Serial.print(F(", durée "));
        Serial.println(now - fillingStartTime);
    }
    digitalWrite(relayPinMapping[OPEN_RELAY], RELAY_OPENED);
    digitalWrite(relayPinMapping[CLOSE_RELAY], RELAY_CLOSED);
    fillingEndTime = now;
    relayPulseTime = now;
    // Stop vibration
    vibrationState = vibrationOff;
}

void setVibrationNextStep(void) {
    unsigned long now = millis();
    if (vibrationState == waitOpened) {                             // We're at end of wait opened
        if (data.closeVibrationPulse) {                             // Do we have a close pulse
            if (data.inDebug) {
                Serial.print(F("Fin attente ouvert à "));
                Serial.print(now);
                Serial.print(F(", durée "));
                Serial.println(now - vibrationTime);
            }
            vibrationTime = now;                                    // Yes, save time
            vibrationDuration = data.closeVibrationPulse;           // ... and duration
            vibrationState = pulseClose;                            // Update state
            digitalWrite(relayPinMapping[OPEN_RELAY], RELAY_OPENED);// Release open relay
            digitalWrite(relayPinMapping[CLOSE_RELAY], RELAY_CLOSED);// Set close relay
            return;
        }
        vibrationState = waitClosed;                                // Skip pulse close and set end of waitClosed
    }
    
    if (vibrationState == pulseClose) {
        if (data.inDebug) {
            Serial.print(F("Fin impulsion fermeture à "));
            Serial.print(now);
            Serial.print(F(", durée "));
            Serial.println(now - vibrationTime);
        }
        digitalWrite(relayPinMapping[CLOSE_RELAY], RELAY_OPENED);   // Release close relay
        vibrationTime = now;                                        // Save time
        vibrationDuration = data.closeVibrationWait;                // ... and duration
        vibrationState = waitClosed;                                // Update state
    }

    if (vibrationState == waitClosed) {
        if (data.openVibrationPulse) {                              // Do we have an open pulse
            if (data.inDebug) {
                Serial.print(F("Fin attente fermé à "));
                Serial.print(now);
                Serial.print(F(", durée "));
                Serial.println(now - vibrationTime);
            }
            vibrationTime = now;                                    // Yes, save time
            vibrationDuration = data.openVibrationPulse;            // ... and duration
            vibrationState = pulseOpen;                             // Update state
            digitalWrite(relayPinMapping[CLOSE_RELAY], RELAY_OPENED);// Release close relay
            digitalWrite(relayPinMapping[OPEN_RELAY], RELAY_CLOSED);// Set open relay
            return;
        }
        vibrationState = waitOpened;                                // Skip pulse open and set end of waitOpen

    }
    if (vibrationState == pulseOpen) {
        if (data.inDebug) {
            Serial.print(F("Fin impulsion ouverture à "));
            Serial.print(now);
            Serial.print(F(", durée "));
            Serial.println(now - vibrationTime);
        }
        digitalWrite(relayPinMapping[OPEN_RELAY], RELAY_OPENED);    // Release open relay
        vibrationTime = now;                                        // Save time
        vibrationDuration = data.openVibrationWait;                 // ... and duration
        vibrationState = waitOpened;                                // Update state
    }
}

// Display all ILS state
void displayIlsState(void) {
    displayIls = true;
}

// Print help message
void printHelp(void) {
    Serial.print(F(ILSOPEN_COMMAND)); Serial.println(F("1-10 : ILS ouverture (numéro)"));
    Serial.print(F(ILSCLOSE_COMMAND)); Serial.println(F("1-10 : ILS fermeture numéro)"));
    Serial.print(F(RELAYPULSEDURATION_COMMAND)); Serial.println(F("1-999 : Durée impulsion relai (ms)"));
    Serial.print(F(WAGONFILLDURATION_COMMAND)); Serial.println(F("1-9999 : Durée remplissage wagon (ms)"));
    Serial.print(F(START_COMMAND)); Serial.println(F(" : Marche"));
    Serial.print(F(STOP_COMMAND)); Serial.println(F(" : Arrêt"));
    Serial.print(F(ILSSTATE_COMMAND)); Serial.println(F(" : Etat ILS"));
    Serial.print(F(OPENVIBRATIONPULSE_COMMAND)); Serial.println(F("0-9999 : Impulsion ouverture vibration (ms)"));
    Serial.print(F(CLOSEVIBRATIONPULSE_COMMAND)); Serial.println(F("0-9999 : Impulsion fermeture vibration (ms)"));
    Serial.print(F(OPENVIBRATIONWAIT_COMMAND)); Serial.println(F("0-9999 : Attente ouverture vibration (ms)"));
    Serial.print(F(CLOSEVIBRATIONWAIT_COMMAND)); Serial.println(F("0-9999 : Attente fermeture vibration (ms)"));
    Serial.print(F(OPENRELAY_COMMAND)); Serial.println(F(" : Ouverture trémie"));
    Serial.print(F(CLOSERELAY_COMMAND)); Serial.println(F(" : Fermeture trémie"));
    Serial.print(F(DEBUGTOGGLE_COMMAND)); Serial.println(F(" : Bascule déverminage"));
    Serial.print(F(INIT_COMMAND)); Serial.println(F(" : Initialisation globale"));
}

// Toggle  debug flag
void toggleDebug(void){
    data.inDebug = !data.inDebug;
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
    if (isCommand(inputBuffer, (char*) INIT_COMMAND)) {
        reinitAll();
    } else if (isCommand(inputBuffer, (char*) OPENRELAY_COMMAND)) {
        startFilling();
    } else if (isCommand(inputBuffer, (char*) CLOSERELAY_COMMAND)) {
        stopFilling();
    } else if (isCommand(inputBuffer, (char*) ILSSTATE_COMMAND)) {
        displayIlsState();
    } else if (isCommand(inputBuffer, (char*) START_COMMAND)) {
        data.isActive = true;
    } else if (isCommand(inputBuffer, (char*) STOP_COMMAND)) {
        data.isActive = false;
        if (fillingStartTime) {
            stopFilling();
        }
    } else if (isCommand(inputBuffer, (char*) DEBUGTOGGLE_COMMAND)) {
        toggleDebug();
    } else if (isCommandValue(inputBuffer, (char*) ILSOPEN_COMMAND, 1, 10)) { 
        data.activationIls = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) ILSCLOSE_COMMAND, 1, 10)) { 
        data.deactivationIls = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) WAGONFILLDURATION_COMMAND, 1, 9999)) {
        data.fillingTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) RELAYPULSEDURATION_COMMAND, 1, 999)) {
        data.pulseTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) OPENVIBRATIONWAIT_COMMAND, 1, 9999)) {
        data.openVibrationWait = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) CLOSEVIBRATIONWAIT_COMMAND, 1, 9999)) {
        data.closeVibrationWait = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) OPENVIBRATIONPULSE_COMMAND, 1, 9999)) {
        data.openVibrationPulse = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) CLOSEVIBRATIONPULSE_COMMAND, 1, 9999)) {
        data.closeVibrationPulse = commandValue;
        saveSettings();
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
    if (relayPulseTime && ((now - relayPulseTime) >= data.pulseTime)) {
        if (data.inDebug) {
            Serial.print(F("Fin impulsion à "));
            Serial.print(now);
            Serial.print(F(", durée "));
            Serial.println(now - relayPulseTime);
        }
        for (uint8_t i = 0; i < 2; i++) {
            digitalWrite(relayPinMapping[i], RELAY_OPENED);
        }
        relayPulseTime = 0;        
    }

    // Do we need to stop filling after a too long time opened?
    if (!fillingEndTime && fillingStartTime && ((now - fillingStartTime) >= data.fillingTime)) {
        Serial.print(F("Fermeture forcée après "));
        Serial.print(data.fillingTime);
        Serial.println(F(" ms"));
        stopFilling();
    };

    // Do job if we're active
    if (data.isActive) {
        // Check for activation ILS
        if (!fillingStartTime && (digitalRead(ilsPinMapping[data.activationIls]) == ILS_CLOSED)) {
            if (data.inDebug) {
                Serial.println(F("Début ILS à "));
                Serial.println(now);
            }
            startFilling();
        }
        // Check for deactivation ILS
        if ((fillingStartTime) && !fillingEndTime && (digitalRead(ilsPinMapping[data.deactivationIls]) == ILS_CLOSED)) {
            stopFilling();
            // Compute between ILS duration (max 65 seconds)
            uint16_t ilsDuration = fillingEndTime - fillingStartTime;
            if (data.inDebug) {
                Serial.println(F("Fin ILS à "));
                Serial.print(now);
                Serial.print(F(", durée "));
                Serial.println(ilsDuration);
            }
            uint16_t idealDuration = data.fillingTime;
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
            fillingStartTime = 0;
            fillingEndTime = 0;
        }
    }

    // Is vibration active?
    if (vibrationState != vibrationOff && ((now - vibrationTime) >= vibrationDuration)) {
        setVibrationNextStep();
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
