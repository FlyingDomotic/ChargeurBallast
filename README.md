# Contrôle d'un chargeur de ballast pour train miniature

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
ajouter vibreur qui sera alimenté lorsque la trémie s'ouvre jusqu'à expiration d'un temps donné
après la fermeture de la trémie. De plus, il est possible d'ajouter un ILS supplémentaire sur la
zone de déchargement, qui activera le vibreur pendant un temps donné, afin de faciliter la descente
du ballast dans la trémie.

Les réglages sont envoyés à l'Arduino au travers de sa liaison série. Ce même moyen est utilisé pour
envoyer les messages à l'utilisateur.

Les réglages sont mémorisés dans l'EEPROM de l'Arduino afin d'être disponibles après son redémarrage.

	
# Hardware Arduino Nano:
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
        - on active le vibreur à l'ouverture de la trémie, on l'arrête ferme après un délai à la fermeture
        - on active le vibreur au passage sur l'ILS de vidage, on l'arrête après un autre délai à la fermeture

# Code :
	- ferme la trémie au lancement
	- déclenche l'ouverture sur la détection de l'ILS paramétré
	- déclenche la fermeture sur la détection de l'ILS paramétré
		ou l'expiration du temps de remplissage paramétré
	- la durée du déclenchement des bobines est fixée par le paramètre paramétré
	- affiche en % la différence entre la durée écoulée entre les ILS d'ouverture et de fermeture et la durée de remplissage du wagon
    - on gère les vibrations pendant la durée de remplissage (et de vidage) du wagon si besoin
	- répond aux commandes sur le port série

# Liste des commandes supportées
    - IO1-10 : ILS ouverture (numéro)
    - IF1-10 : ILS fermeture( numéro)
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
    - INIT : Initialisation globale

Auteur : Flying Domotic, Février 2025, pour le FabLab
Licence: GNU GENERAL PUBLIC LICENSE - Version 3, 29 June 2007
