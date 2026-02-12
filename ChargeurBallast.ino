#define CODE_VERSION "V26.2.11-1"

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

	Il arrive que le granulat bloque la sortie de la trémie. Pour contrer ce problème, on peut ajouter
        un vibreur qui sera alimenté lorsque la trémie s'ouvre jusqu'à expiration d'un temps donné
        après la fermeture de la trémie. De plus, il est possible d'ajouter un ILS supplémentaire sur la
        zone de déchargement, qui activera également le vibreur pendant un temps donné, afin de faciliter
        la descente du ballast dans la trémie.
    
    Les réglages sont envoyés à l'Arduino au travers de sa liaison série. Ce même moyen est utilisé pour
		envoyer les messages à l'utilisateur.

	Les réglages sont mémorisés dans l'EEPROM de l'Arduino afin d'être disponibles après son redémarrage.
	
Hardware Arduino Nano:
	- 11 entrées ILS (10 pour l'avancement du wagon lors du chargement, 1 pour la détection du déchargement)
	- 3 sorties relais (1 pour l'ouverture de la trémie, 1 pour sa fermeture, 1 pour le vibreur)
	- interface série pour paramétrage :
		- définition numéro ILS activation trémie
		- définition numéro ILS désactivation trémie
		- définition durée impulsion en 1/1000eme de seconde
		- définition durée remplissage en 1/1000eme de seconde
        - définition du délai d'arrêt des vibrations au remplissage
        - définition du délai d'arrêt des vibrations au vidage
		- affichage état des ILS
		- affichage aide
		- commande d'ouverture de trémie
		- commande de fermeture
        - (ré)initialisation
        - fonction marche/arrêt
        - bascule debug
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
        - on active le vibreur à l'ouverture de la trémie, on l'arrête ferme après un délai à la fermeture
        - on active le vibreur au passage sur l'ILS de vidage, on l'arrête après un autre délai à la fermeture

Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré
	- déclenche la fermeture sur la détection de l'ILS paramétré
		ou l'expiration du temps de remplissage paramétré
	- la durée du déclenchement des bobines est fixée par le paramètre paramétré
	- affiche en % la différence entre la durée écoulée entre les ILS d'ouverture et de fermeture et la durée de remplissage du wagon
    - on gère les vibrations pendant la durée de remplissage (et de vidage) du wagon si besoin
	- répond aux commandes sur le port série

Commandes :
    - IO1-10 : ILS ouverture (numéro)
    - IF1-10 : ILS fermeture (numéro)
    - DI1-999 : Durée impulsion relai (ms)
    - DR1-9999 : Durée remplissage wagon (ms)
    - RR0-9999 : Retard remplissage vibrations (ms)
    - RV0-9999 : Retard vidage vibrations (ms)
    - M : Marche
    - A : Arrêt
    - E : Etat ILS
    - O : Ouverture trémie
    - F : Fermeture trémie
    - D : Bascule déverminage
    - AV : Affiche variables
    - INIT : Initialisation globale

Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007

*/

// Includes
#include "Arduino.h"
#include "EEPROM.h"

// Command and text

#define ILS_OPEN_COMMAND "IO"
#define ILS_CLOSE_COMMAND "IF"
#define RELAY_PULSE_DURATION_COMMAND "DI"
#define WAGON_FILL_DURATION_COMMAND "DR"
#define LOAD_DELAY_COMMAND "RR"
#define UNLOAD_DELAY_COMMAND "RV"
#define START_COMMAND "M"
#define STOP_COMMAND "A"
#define ILS_STATE_COMMAND "E"
#define OPEN_RELAY_COMMAND "O"
#define CLOSE_RELAY_COMMAND "F"
#define DEBUG_TOGGLE_COMMAND "D"
#define INIT_COMMAND "INIT"
#define DISPLAY_VARIABLES_COMMAND "AV"


//  Parameters

#define MAGIC_NUMBER 59                                             // EEPROM magic byte
#define EEPROM_VERSION 3                                            // EEPROM version
#define BUFFER_LENGHT 50                                            // Serial input buffer length
#define ILS_CLOSED LOW                                              // State read when ILS is closed
#define RELAY_CLOSED LOW                                            // State to write to close relay
#define RELAY_OPENED HIGH                                           // State to write to open relay
#define DISPLAY_ILS_TIME 100                                        // Display ILS state every xxx ms
#define OPEN_RELAY 0                                                // Index of open relay into relayPinMapping
#define CLOSE_RELAY 1                                               // Index of close relay into relayPinMapping
#define VIBRATION_RELAY 2                                           // Index of vibration relay into relayPinMapping
#define DISPLAY_KEYBOARD_INPUT                                      // Display each character read on keyboard

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
    uint16_t loadDelay;                                             // Duration (ms) to keep vibrations after load (here even if VIBRATION_RELAY not set)
    uint16_t unloadDelay;                                           // Duration (ms) to keep vibrations after unload (here even if VIBRATION_RELAY not set)
};

// EEPROM data (V2 version, not used anymore but converted to new version if needed)
struct eepromDataV2_s {
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

// EEPROM data (V1 version, not used anymore but converted to new version if needed))
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
uint8_t ilsPinMapping[] = {4, 5, 6, 7, A5, A4, A3, A2, A1, A0, 2};  // Maps ILS number to Arduino PIN number
uint8_t relayPinMapping[] = {12, 11, 10, 9};                        // Maps relay number to Arduino PIN number
uint8_t relayState[] = {0, 0, 0, 0};                                // Relay current state

// Data
unsigned long fillingStartTime = 0;                                 // Filling start time (0 if none)
unsigned long fillingEndTime = 0;                                   // Filling end time (0 if none)
unsigned long relayPulseTime = 0;                                   // Relay plusing start time
unsigned long lastIlsDisplay = 0;                                   // Last time we displayed ILS
#ifdef VIBRATION_RELAY
    unsigned long vibrationLoadTime = 0;                            // Last vibration load time
    unsigned long vibrationUnloadTime = 0;                          // Last vibration unload time
#endif
uint16_t commandValue;                                              // Value extracted from command
uint8_t bufferLen = 0;                                              // Used chars in input buffer
char inputBuffer[BUFFER_LENGHT];                                    // Serial input buffer
bool doorOpened = false;                                            // Is door opened?
eepromData_s data;                                                  // Data stored to/read from EEPROM

// Routines and functions

unsigned long getMillis(void);                                      // Millis() implementation never returning 0
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
void stopFilling(unsigned long now);                                // Stop filling
#ifdef VIBRATION_RELAY
    void startUnloading(unsigned long now);                                      // Start unloading process
    void startVibration(void);                                      // Start vibration relay
    void stopVibration(void);                                       // Stop vibration relay
#endif
void displayIlsState(void);                                         // Display all ILS state
void printHelp(void);                                               // Print help message
void toggleDebug(void);                                             // Toggle  debug flag
void reinitAll(void);                                               // Reinitialize all settings
void executeCommand(void);                                          // Execute command read on serial input (a-z and 0-9)
void displayVariables(void);                                        // Display all variables (debug)
void setRelay(uint8_t pin, uint8_t state);                          // Change a relay state
void setup(void);                                                   // Setup
void loop(void);                                                    // Main loop

// millis() implementation never returning 0
unsigned long getMillis(void){
    unsigned long now = millis();
    if (!now) now++;                                                // Avoid lillis() un setup returning 0 (or at overflow)
    return now;
}

// Display current status
void displayStatus(void) {
    Serial.print(F(ILS_OPEN_COMMAND));
    Serial.print(data.activationIls);
    Serial.print(F(" "));
    Serial.print(F(ILS_CLOSE_COMMAND));
    Serial.print(data.deactivationIls);
    Serial.print(F(" "));
    Serial.print(F(RELAY_PULSE_DURATION_COMMAND));
    Serial.print(data.pulseTime);
    Serial.print(F(" "));
    Serial.print(F(WAGON_FILL_DURATION_COMMAND));
    Serial.print(data.fillingTime);
    #ifdef VIBRATION_RELAY
        if (data.loadDelay) {
            Serial.print(F(" "));
            Serial.print(F(LOAD_DELAY_COMMAND));
            Serial.print(data.loadDelay);
        }
        if (data.unloadDelay) {
            Serial.print(F(" "));
            Serial.print(F(UNLOAD_DELAY_COMMAND));
            Serial.print(data.unloadDelay);
        }
    #endif
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
        eepromDataV2_s v2Data;                                      // V2 data structure
        EEPROM.get(0, v2Data);                                      // Load EEPROM V2 structure
        data.activationIls = v2Data.activationIls;
        data.deactivationIls = v2Data.deactivationIls;
        data.fillingTime = v2Data.fillingTime;
        data.pulseTime = v2Data.pulseTime;
        data.isActive = v2Data.isActive;
        data.inDebug = v2Data.inDebug;
    } else if (version == 3) {
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
    data.loadDelay = 0;                                             // Keep vibrations after load
    data.unloadDelay = 0;                                           // Keep vibrations after unload
}

// Reset serial input buffer
void resetInputBuffer(void) {
    memset(inputBuffer, 0, sizeof(inputBuffer));
    bufferLen = 0;
}

// Init IO pins
void initIO(void) {
    // Set ILS PIN to input with pullup resistor
    for (uint8_t i = 0; i < 11; i++) {
        pinMode(ilsPinMapping[i], INPUT_PULLUP);
    }
    // Set relay PIN to output, init level = opened
    for (uint8_t i = 0; i < 4; i++) {
        setRelay(i, RELAY_OPENED);
        pinMode(relayPinMapping[i], OUTPUT);
    }
}

// Work with Serial input
void workWithSerial(void) {
    // Reset display ILS flag
    if (displayIls) {
        Serial.println();
        displayIls = false;
    }
    // Read serial input
    while (Serial.available()) {
        // Read one character
        char c = Serial.read();
        // Is this a return?
        if (c == 13) {
            Serial.print(c);
            executeCommand();
            resetInputBuffer();
        } else if (c) {
            // Keep only "A" to "Z", "a" to "z" and "0" to "9"
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
                if (bufferLen >= BUFFER_LENGHT - 1) {
                    Serial.println(F("Buffer plein - Reset!"));
                    resetInputBuffer();
                }
                inputBuffer[bufferLen++] = c;
            } else if (c == 8) {                                  // Is this <backspace> character?
                if (bufferLen) {                                    // Is buffer not empty?
                    bufferLen--;                                    // Reduce buffer len
                    inputBuffer[bufferLen] = 0;                     // Remove charcater
                }
            }
        }
    }
    #ifdef DISPLAY_KEYBOARD_INPUT
        Serial.print(F("\r"));
        Serial.print(inputBuffer);
        Serial.print(F("  "));
        Serial.print(F("\r"));
        Serial.print(inputBuffer);
    #endif
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
    unsigned long now = getMillis();
    if (data.inDebug) {
        Serial.print(F("Début chargement à "));
        Serial.println(now);
    }
    setRelay(CLOSE_RELAY, RELAY_OPENED);
    setRelay(OPEN_RELAY, RELAY_CLOSED);
    fillingStartTime = now;
    fillingEndTime = 0;
    relayPulseTime = now;
    startVibration();
}

// Stop filling
void stopFilling(unsigned long now) {
    if (data.inDebug) {
        Serial.print(F("Fin chargement à "));
        Serial.print(now);
        Serial.print(F(", durée "));
        Serial.println(now - fillingStartTime);
    }
    setRelay(OPEN_RELAY, RELAY_OPENED);
    setRelay(CLOSE_RELAY, RELAY_CLOSED);
    fillingEndTime = now;
    fillingStartTime = 0;
    relayPulseTime = now;
    #ifdef VIBRATION_RELAY
        // Keep vibrations after load
        vibrationLoadTime = now;
    #endif
}

#ifdef VIBRATION_RELAY
    // Start unloading process
    void startUnloading(unsigned long now) {
        if (!vibrationUnloadTime) {
            if (data.inDebug) {
                Serial.println(F("Début déchargement"));
            }
            startVibration();
        }
        vibrationUnloadTime = now;
    }

    // Start vibration relay
    void startVibration(void) {
        if (data.inDebug) {
            Serial.print(F("Début vibrations à "));
            Serial.println(getMillis());
        }
        setRelay(VIBRATION_RELAY, RELAY_CLOSED);
    }

    // Stop vibration relay
    void stopVibration(void) {
        if (data.inDebug) {
            Serial.print(F("Fin vibrations à "));
            Serial.println(getMillis());
        }
        setRelay(VIBRATION_RELAY, RELAY_OPENED);
        vibrationLoadTime = 0;
        vibrationUnloadTime = 0;
    }
#endif

// Display all ILS state
void displayIlsState(void) {
    displayIls = true;
}

// Print help message
void printHelp(void) {
    Serial.print(F(ILS_OPEN_COMMAND)); Serial.println(F("1-10 : ILS ouverture (numéro)"));
    Serial.print(F(ILS_CLOSE_COMMAND)); Serial.println(F("1-10 : ILS fermeture( numéro)"));
    Serial.print(F(RELAY_PULSE_DURATION_COMMAND)); Serial.println(F("1-999 : Durée impulsion relai (ms)"));
    Serial.print(F(WAGON_FILL_DURATION_COMMAND)); Serial.println(F("1-9999 : Durée remplissage wagon (ms)"));
    #ifdef VIBRATION_RELAY
        Serial.print(F(LOAD_DELAY_COMMAND)); Serial.println(F("0-9999 : Retard remplissage vibrations (ms)"));
        Serial.print(F(UNLOAD_DELAY_COMMAND)); Serial.println(F("0-9999 : Retard vidage vibrations (ms)"));
    #endif
    Serial.print(F(START_COMMAND)); Serial.println(F(" : Marche"));
    Serial.print(F(STOP_COMMAND)); Serial.println(F(" : Arrêt"));
    Serial.print(F(ILS_STATE_COMMAND)); Serial.println(F(" : Etat ILS"));
    Serial.print(F(OPEN_RELAY_COMMAND)); Serial.println(F(" : Ouverture trémie"));
    Serial.print(F(CLOSE_RELAY_COMMAND)); Serial.println(F(" : Fermeture trémie"));
    Serial.print(F(DEBUG_TOGGLE_COMMAND)); Serial.println(F(" : Bascule déverminage"));
    Serial.print(F(DISPLAY_VARIABLES_COMMAND)); Serial.println(F(" : Affiche variables"));
    Serial.print(F(INIT_COMMAND)); Serial.println(F(" : Initialisation globale"));
}

// Toggle  debug flag
void toggleDebug(void){
    data.inDebug = !data.inDebug;
    saveSettings();
}

// Reinitialize all settings
void reinitAll(void) {
    stopFilling(getMillis());
    initSettings();
    saveSettings();
}

// Execute command read on serial input (a-z and 0-9)
void executeCommand(void) {
    #ifndef DISPLAY_KEYBOARD_INPUT
        Serial.println(F(""));
        Serial.print(F("Reçu: "));
    #endif
    Serial.println(inputBuffer);
    if (isCommand(inputBuffer, (char*) INIT_COMMAND)) {
        reinitAll();
    } else if (isCommand(inputBuffer, (char*) OPEN_RELAY_COMMAND)) {
        startFilling();
    } else if (isCommand(inputBuffer, (char*) CLOSE_RELAY_COMMAND)) {
        stopFilling(getMillis());
    } else if (isCommand(inputBuffer, (char*) ILS_STATE_COMMAND)) {
        displayIlsState();
    } else if (isCommand(inputBuffer, (char*) START_COMMAND)) {
        data.isActive = true;
    } else if (isCommand(inputBuffer, (char*) STOP_COMMAND)) {
        data.isActive = false;
        if (fillingStartTime) {
            stopFilling(getMillis());
        }
    } else if (isCommand(inputBuffer, (char*) DEBUG_TOGGLE_COMMAND)) {
        toggleDebug();
    } else if (isCommand(inputBuffer, (char*) DISPLAY_VARIABLES_COMMAND)) {
        displayVariables();
    } else if (isCommandValue(inputBuffer, (char*) ILS_OPEN_COMMAND, 1, 10)) { 
        data.activationIls = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) ILS_CLOSE_COMMAND, 1, 10)) { 
        data.deactivationIls = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) WAGON_FILL_DURATION_COMMAND, 1, 9999)) {
        data.fillingTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) RELAY_PULSE_DURATION_COMMAND, 1, 999)) {
        data.pulseTime = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) LOAD_DELAY_COMMAND, 1, 9999)) {
        data.loadDelay = commandValue;
        saveSettings();
    } else if (isCommandValue(inputBuffer, (char*) UNLOAD_DELAY_COMMAND, 1, 9999)) {
        data.unloadDelay = commandValue;
        saveSettings();
    } else {
        if (inputBuffer[0]) {
            printHelp();
        }
    }
    displayStatus();
}

// Display all variables (debug)
void displayVariables(void) {
    Serial.print(F("millis()=")); Serial.println(millis());
    Serial.print(F("fillingStartTime=")); Serial.println(fillingStartTime);
    Serial.print(F("fillingEndTime=")); Serial.println(fillingEndTime);
    Serial.print(F("relayPulseTime=")); Serial.println(relayPulseTime);
    Serial.print(F("lastIlsDisplay=")); Serial.println(lastIlsDisplay);
    #ifdef VIBRATION_RELAY
        Serial.print(F("vibrationLoadTime=")); Serial.println(vibrationLoadTime);
        Serial.print(F("vibrationUnloadTime=")); Serial.println(vibrationUnloadTime);
    #endif
    Serial.print(F("doorOpened=")); Serial.println(doorOpened);
    Serial.println(F("ILS: 1234567890D"));
    Serial.print(F(  "     "));
    for (uint8_t i = 0; i<11; i++) {
        if (digitalRead(ilsPinMapping[i]) == ILS_CLOSED) {
            Serial.print(F("X"));
        } else {
            Serial.print(F("-"));
        }
    }
    Serial.println();
    Serial.println(F("Relai : OFV"));
    Serial.print(F(  "        "));
    for (uint8_t i = 0; i<3; i++) {
        if (relayState[i] == RELAY_CLOSED) {
            Serial.print(F("X"));
        } else {
            Serial.print(F("-"));
        }
    }
    Serial.println();
}

// Change a relay state
void setRelay(uint8_t index, uint8_t state){
    digitalWrite(relayPinMapping[index], state);
    relayState[index] = state;
    // Set door opened depnding on states
    if (index == OPEN_RELAY && state) doorOpened = true;
    if (index == CLOSE_RELAY && state) doorOpened = false;
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
    stopFilling(getMillis());
    displayStatus();
}

// Main loop
void loop(void){
    unsigned long now = getMillis();

    // Do we need to close any relay after pulse?
    if (relayPulseTime && ((now - relayPulseTime) >= data.pulseTime)) {
        if (data.inDebug) {
            Serial.print(F("Fin impulsion à "));
            Serial.print(now);
            Serial.print(F(", durée "));
            Serial.println(now - relayPulseTime);
        }
        for (uint8_t i = 0; i < 2; i++) {
            setRelay(i, RELAY_OPENED);
        }
        relayPulseTime = 0;        
    }

    // Do we need to stop filling after a too long time opened?
    if (!fillingEndTime && fillingStartTime && ((now - fillingStartTime) >= data.fillingTime)) {
        Serial.print(F("Fermeture forcée à "));
        Serial.print(now);
        Serial.print(F(" après "));
        Serial.print(data.fillingTime);
        Serial.println(F(" ms"));
        stopFilling(now);
    };

    // Do job if we're active
    if (data.isActive) {
        // Check for activation ILS
        if (!fillingStartTime && (digitalRead(ilsPinMapping[data.activationIls]) == ILS_CLOSED)) {
            if (data.inDebug) {
                Serial.print(F("Début ILS à "));
                Serial.println(now);
            }
            startFilling();
        }
        // Check for deactivation ILS
        if ((fillingStartTime) && !fillingEndTime && (digitalRead(ilsPinMapping[data.deactivationIls]) == ILS_CLOSED)) {
            stopFilling(now);
            // Compute between ILS duration (max 65 seconds)
            uint16_t ilsDuration = fillingEndTime - fillingStartTime;
            if (data.inDebug) {
                Serial.print(F("Fin ILS à "));
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
        #ifdef VIBRATION_RELAY
            // Check for unload ILS
            if (digitalRead(ilsPinMapping[10]) == ILS_CLOSED) {
                startUnloading(now);
            }
        #endif
    }

    #ifdef VIBRATION_RELAY
        if (((!vibrationLoadTime) || ((now - vibrationLoadTime) >= data.loadDelay))
                && ((!vibrationUnloadTime) || ((now - vibrationUnloadTime) >= data.unloadDelay))) {
            if (vibrationLoadTime || vibrationUnloadTime) {
                stopVibration();
            }
        }
    #endif
    // Scan serial for input
    if (Serial.available()) {
        workWithSerial();
    }

    // Display ILS state if needed
    if (displayIls) {
        if ((getMillis() - lastIlsDisplay) > DISPLAY_ILS_TIME) {
            Serial.print (F("\rILS : "));
            for (uint8_t i = 0; i < 10; i++) {
                if (digitalRead(ilsPinMapping[i]) == ILS_CLOSED) {
                    Serial.print(i+1);
                } else {
                    Serial.print(F("-"));
                    if (i == 9) Serial.print(F(" "));
                }
                Serial.print(F(" "));
            }
            #ifdef VIBRATION_RELAY  
                // Display unload vibrations ILS
                if (digitalRead(ilsPinMapping[10]) == ILS_CLOSED) {
                    Serial.print(F("D"));
                } else {
                    Serial.print(F("-"));
                }
            #endif
            Serial.print(F(" "));
            lastIlsDisplay = getMillis();
        }
    }
}
