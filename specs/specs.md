# Specifications

## 1. Cas d'utilisation

On décide d'enregistrer un trajet : on passe le drone en mode "controle manuel" et enregistre le trajet.

Une fois que c'est fait, on peut lancer un trajet prédéfini.

Le drone va faire son trajet et faire les analyses d'image à la volée. S'il n'y a aucun problème en route, 
tout va bien, on rentre à la maison comme si de rien n'était. Sinon :

 - Une alete est lancée
 - La position du drone est sauvegardée en tant que point d'intérêt
 - Le drone s'arrête et attend le choix de l'opérateur
 - (L'opérateur peut voir ce que filme le drone en direct sur l'application)

Il y a donc deux choix possibles à ce moment :
1. Aucun problème, on continue le trajet prévu
    - Le point d'intérêt est défini à "non important" -> ne pas déclencher d'alerte à ce point lors du prochain passage ?
2. On veut investiguer un peu plus 
    - On repasse temporairement en mode manuel 
    - Ce point d'intérêt est sauvegardé comme important
    - L'opérateur contrôle le drone pour faire un tour et enregistrer de nouvelles images
    - Une fois qu'il a fini, le drone retourne en mode "contrôle automatique" et continue son trajet programmé la ou l'avait laissé

Lors des prochains lancements de ce trajet, l'opérateur pourra :
1. Indiquer au drone qu'il doit faire ce trajet comme s'il ne l'avait jamais fait (pas sûr de ça) 
2. Indiquer au drone quels points d'intérêt importants à approfondir (i.e, faire le trajet alternatif) quoi qu'il se passe
et quels points à ommettre 
 - Ne pas lancer d'alerte sur les points omis, même s'il y a un problème. On sauvegarde tout de même l'anomalie
3. Laisser le drone choisir les points d'intérêt à approfondir (s'il y en a)
    
Il n'est pas impossible que, lorsqu'un point d'intérêt est approfondi (trajet alternatif), il puisse y avoir une nouvelle anomalie.
Il faudrait alors laisser les mêmes deux choix à l'opérateur, engendrant donc un nouveau point d'intérêt ainsi que (si l'opérateur en décide ainsi) d'un nouveau trajet alternatif. Il faut alors prévoir d'avoir des trajets alternatifs de trajets alternatifs (dans une sorte de hiérarchie. 

Il serait aussi bien de pouvoir laisser à l'opérateur la possibilité d'avoir une vue globale sur un trajet (un mode "historique") avec :
 - Des statistiques globales (durée du trajet, nombre de fois où le trajet à été lancé)
 - Une vue sur le trajet et ses embranchements
 - Tous les points d'intérêt ainsi que la fréquence ou il y a eu des anomalies trouvées

## 2. Contraintes et qualifications

### 2.1. Drone et télémétrie

La télémétrie disponible est une télémétrie très longue distance : rayon d'émission / reception de près de 40km (à confirmer)

### 2.2. Caméra

La caméra utilisée (Foxeer CAT 2) envoie des images de résolution 640x480 (4:3) via un transmetteur vidéo (TS852 32CH 5.8G) aux fréquences et canaux d'émission réglables. La reception se fait via un Skydroid 5.8G OTG Receiver, recepteur que l'on peut connecter via usb-c sur un ordinateur fonctionnant comme un carte d'acquisition vidéo.

La taille des images dépend des informations sur celle-ci, sur 3 images prises (avec un taux de compression défini à 3), on a enregistré
des tailles de 253, 263 et 283 kilo-octets (plus de tests de contraintes sont à prévoir)

### 2.3 Appli Mobile

Une application mobile doit permettre de réaliser l'ensemble des opérations sur le drone, à savoir le pilotage manuel, le lancement des trajets automatiques ainsi que la consultation de l'historique, le tout sur un téléphone tournant sous Android.

La communication avec le serveur central se fait via une communcation UDP et sous un format JSON. La communication se réalise toujours avec un format REQUÊTE -> REPONSE et chaque transmission doit recevoir sa réponse associée dans un délai de 100ms (hormis la requête MANUAL_CONTROL) afin de garantir une bonne réactivité sur le pilotage du drone ainsi qu'une ergonomie correcte lors de la navigation entre les fenêtres.
