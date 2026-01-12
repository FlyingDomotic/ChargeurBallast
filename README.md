# Contrôle d'un chargeur de ballast pour train miniature

Ce code contrôle un chargeur de ballast dans des wagons adéquats.

Il est basé sur une trémie commandée par 2 relais à impulsion.

Le premier relais ouvre la trémie alors que le second la ferme.

Un rail a été équipé de 10 ILS régulièrement espacés, afin de détecter l'aimant placé sous les wagons.

Lorsqu'un wagon passe au dessus de l'ILS d'ouverture (paramétrable), une impulsion (de durée réglable) est envoyée au relais d'ouverture. La même chose est faite lorsque l'ILS de fermeture est activé. De plus, la trémie est automatiquement fermée après un temps paramétrable, afin d'éviter de vider son contenu sur la voie en cas de souci (train bloqué, problème d'ILS, ...)

A la fin d'un cycle d'ouverture/fermeture, l'Arduino affiche l'écart entre la vitesse réelle de chargement et la vitesse optimale. Si on modifie la vitesse du train en proportion, on aura un chargement parfait.

Les réglages sont envoyés à l'Arduino au travers de sa liaison série. Ce même moyen est utilisé pour envoyer les messages à l'utilisateur.

Les réglages sont mémorisés dans l'EEPROM de l'Arduino afin d'être disponibles après son redémarrage.
	
# Hardware Arduino Nano:
	- 10 entrées ILS
	- 2 sorties relais
	- interface série pour paramétrage :
		- définition numéro ILS activation trémie(IlO1 à ILO10)
		- définition numéro ILS désactivation trémie (ILF1 à ILF10)
		- définition durée impulsion relai en 1/1000eme de seconde (DIR1 à T999)
		- définition durée remplissage wagon en 1/1000eme de seconde (DRW1 à R9999)
		- affichage état des ILS (EI)
		- affichage aide (?)
		- commande d'ouverture de trémie (OT)
		- commande de fermeture (FT)
        - (ré)initialisation (INIT)
        - fonction marche (M)
        - fonction arrêt (A)
        - bascule déverminage (BD)
		- sauvegarde automatique paramètres en EEPROM

# Principe utilisé :
	- le temps de remplissage d'un wagon est fixe, lié à la section de la trémie
	- on commence à remplir en passant sur un ILS
	- on termine le remplissage en passant sur un autre ILS
	- la vitesse du train doit être telle que le temps entre les 2 ILS soit celui du remplissage

# Paramétrage :
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

# Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré A, valeur 1 à 10
	- déclenche la fermeture sur la détection de l'ILS paramétré D, valeur 1 à 10
		ou l'expiration du temps de remplissage paramétré R, valeur 1 à 9999 en millisecondes
	- la durée du déclenchement des bobines est fixée en 1/100 s paramètre T, valeur 1 à 99)
	- affiche en % la différence entre la durée écoulée entre A et D et celle de R
    - on gère les vibrations pendant la durée de remplissage du wagon si besoin
	- répond aux commandes sur le port série

# Liste des commandes supportées
    - ILO1-10 : ILS ouverture (numéro)
    - ILF1-10 : ILS fermeture (numéro)
    - DIR1-999 : Durée impulsion relai (ms)
    - DRW1-9999 : Durée remplissage wagon (ms)
    - M : Marche
    - A : Arrêt
    - EI : Etat ILS
    - IOV0-9999 : Impulsion ouverture vibration (ms)
    - IFV0-9999 : Impulsion fermeture vibration (ms)
    - AOV0-9999 : Attente ouverture vibration (ms)
    - AFV0-9999 : Attente fermeture vibration (ms)
    - OT : Ouverture trémie
    - FT : Fermeture trémie
    - BD : Bascule déverminage
    - ?: cette aide


Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007
