# ProjetSynthese
Projet de synthèse sur le projet "Drone intelligent".

Ce projet requiert une version de C++17 ou plus afin de pouvoir être compilé

## Installation des dépedances

Allez dans le dossier racine du dépôt et lancez le script suivant :

> $ ./scripts/install_and_configure.sh

Si il y a des erreurs dans ce script, essayez de lancer celui-ci :

> $ ./scripts/subScripts/install_blaar_libs_ubuntu.sh

## Communication via USB (FAIRE A CHAQUE DEMARRAGE POUR L'INSTANT)

Pour pouvoir lancer l'application, il faut avoir les autorisations pour communiquer en usb.

- $ ls -l /dev/ttyACM0

Cela vous permettra de voir si l'appareil est connecté sur ce device, sinon on va devoir en parler...

- $ sudo chmod 666 /dev/ttyACM0

- $ sudo adduser [votre pseudo] dialout

Et normalement la connexion devrait marcher maintenant

## Compiler Com_server

Vous pouvez lancer la compilation de Com_server facilement en lançant le script : 

> $ ./scripts/compile_com_server.sh

Les executables se créent alors dans le dossier "bin" du projet
