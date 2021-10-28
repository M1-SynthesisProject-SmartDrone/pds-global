## Installation des librairies cyu

### Dépendances simples

Avant toute chose, utilisez cette commande :

$ sudo apt-get install cmake build-essential colordiff git doxygen -y

Il faut d'abord installer avec apt les librairies suivantes : 
- librapidxml-dev
- libncurses5-dev

### MAVSDK

1. $ git clone https://github.com/mavlink/MAVSDK.git
2. $ cd MAVSDK
3. $ git submodule update --init --recursive
4. $ cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H.
5. $ cmake --build build/default

### blaar

Il faut ensuite installer les librairies blaar de cyu dont on a besoin : 
lancez tout simplement le script prévu à cet effet (PAS DANS CE REPOS) ! 

> $ ./scripts/install_blaar_libs_ubuntu.sh

On peut ensuite installer la vraie librairie de communication : nous utiliserons la version qui est dans ce dépôt (voir dans **"cyu/Com_server"**).

> Le .gitignore devrait éviter de commiter les fichiers générés, mais faites attention tout de même !

1. Aller dans le projet **Com_server** qui est dans le repos
2. $ mkdir build
3. $ cd build
4. $ cmake ..
5. $ make

Les executables se créent alors dans le dossier "bin" du projet