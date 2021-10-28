#!/bin/bash

# Use this script and (normally) all the wanted libraries will 
# be installed and the project will be ready to be compiled
# with the script "./compile_com_server.sh".
#
# This script is meant to be used while in the root of the repository
#
# Author : Aldric Vitali Silvestre

# Ask for sudo rights and relaunch script as sudo if not the case
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

set -o errexit  #Exit on first error
set -o nounset  #Error if a variable is used but nont defined
set -o pipefail #Error if a pipe fail

# Before, check if we are in the right dir
if [ "$(basename $(pwd))" != "ProjetSynthese" ]; then
    echo "You are not in the right folder ! Go to the repository 'ProjetSynthese' and relaunch the script."
    exit 1
fi

echo "============================================"
echo "=           Install and configure          ="
echo "============================================"
echo

# Step 1 : install all necessary libraries
echo "==========================="
echo "=     Install packages    ="
echo "==========================="
sudo apt-get --allow-unauthenticated update -y
sudo apt-get install xboxdrv librapidxml-dev vim libncurses5-dev libsdl2-ttf-dev -y
sudo apt-get install build-essential colordiff git g++ cmake ninja-build doxygen lsof libpng-dev libjpeg-dev gnuplot fftw3-dev libsndfile1-dev libasound2-dev libv4l-dev libgtk-3-dev -y
sudo apt-get install postgresql postgresql-contrib postgresql-server-dev-12 -y

echo
echo "==========================="
echo "=     Install cxxopts     ="
echo "==========================="
echo
git clone --depth 1 --branch v3.0.0 https://github.com/jarro2783/cxxopts
cd cxxopts
sudo cmake .
sudo make install
cd ..
sudo rm -r cxxopts

# Step 2 : libpqxx
echo
echo "==========================="
echo "=     Install libpqxx     ="
echo "==========================="
echo
git clone https://github.com/jtv/libpqxx.git
cd libpqxx
sudo sh ./configure
sudo make
sudo make install
cd ..
sudo rm -r libpqxx

# Step 3 : MAVSDK
echo
echo "==========================="
echo "=      Install MAVSDK     ="
echo "==========================="
echo
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git submodule update --init --recursive
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H.
cmake --build build/default
cd ..
sudo rm -r 

# Step 4 : install blc libraries
echo
echo "==========================="
echo "=       Install blc       ="
echo "==========================="
echo
git clone https://git.cyu.fr/blaar/blc.git
mkdir blc_build && cd blc_build  && cmake ../blc
make
sudo make install
cd ..
sudo rm -r blc_build blc

# WARN : libs folders should be removed, but be still careful
# sudo sh ./scripts/subScripts/install_blaar_libs_ubuntu.sh

# # Step 5 : create makefiles and build binaries for the Com_server
# echo
./scripts/compile_com_server.sh

echo "==== Update libraries config ===="
sudo ldconfig


echo "============================================"
echo "=                  Ended !                 ="
echo "============================================"
echo "Now, go launch the app into the 'bin' folder !"
