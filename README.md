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
		- définition numéro ILS activation trémie(A1 à A10)
		- définition numéro ILS désactivation trémie (D1 à D10)
		- définition durée impulsion en 1/1000eme de seconde (T1 à T999)
		- définition durée remplissage en 1/1000eme de seconde (R1 à R9999)
		- sauvegarde paramètres en EEPROM
		- affichage état des ILS (E)
		- affichage aide (?)
		- commande d'ouverture de trémie (O)
		- commande de fermeture (F)
        - (ré)initialisation (INIT)
        - fonction marche/arrêt (M)
        - bascule debug (P)

# Principe utilisé :
	- le temps de remplissage d'un wagon est fixe, lié à la section de la trémie
	- on commence à remplir en passant sur un ILS
	- on termine le remplissage en passant sur un autre ILS
	- la vitesse du train doit être telle que le temps entre les 2 ILS soit celui du remplissage

# Paramétrage :
    - on cherche la durée d'impulsion nécessaire pour ouvrir ou fermer la trémie à coup sur, sans faire chauffer les bobines,
        en augmentant/réduisant la valeur du paramètre T et utilisant les commandes O et F pour ouvrir/fermer la trémie
	- on mesure le temps de remplissage d'un wagon au chrono, avec avancée du wagon à la main (paramètre R, valeur 1 à 9999, en millisecondes)
	- on repère le numéro de l'ILS début de remplissage  (paramètre A, valeur 1 à 10)
	- on repère le numéro de l'ILS fin de remplissage  (paramètre D, valeur 1 à 10)
	- on ajuste la vitesse de la loco pour que le temps entre les 2 ILS soit celui de remplissage, aidé par le retour de l'Arduino
		sur l'écart avec la vitesse idéale (ajouter le pourcentage donné par l'Arduino à la vitesse courante de la loco pour être parfait)

# Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré A, valeur 1 à 10
	- déclenche la fermeture sur la détection de l'ILS paramétré D, valeur 1 à 10
		ou l'expiration du temps de remplissage paramétré R, valeur 1 à 9999 en millisecondes
	- la durée du déclenchement des bobines est fixée en 1/100 s paramètre T, valeur 1 à 99)
	- affiche en % la différence entre la durée écoulée entre A et D et celle de R
	- répond aux commandes sur le port série

# Liste des commandes supportées
    - A1-10: numéro ILS activation
    - D1-10: numéro ILS désactivation
    - R1-99: durée remplissage (0,1 s)
    - T1-99: durée impulsion relais (0,01 s)
    - O: ouvre trémie
    - F: ferme trémie
    - E: état ILS
    - M: bascule l'état marche/arrêt
    - P: bascule l'état debug
    - init: (ré)initialise tout
    - ?: cette aide


Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007
