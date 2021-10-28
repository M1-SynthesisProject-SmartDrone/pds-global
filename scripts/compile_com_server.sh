#!/bin/bash

# A very little script that will create cmake files and use them to 
# create the binaries
#
# Author : Aldric Vitali Silvestre
#
# WARN : you must be in the root repository folder before trying to launch
# this script

# Before, check if we are in the right dir
if [ "$(basename $(pwd))" != "ProjetSynthese" ]; then
    echo "You are not in the right folder ! Go to the repository 'ProjetSynthese' and relaunch the script."
    exit 1
fi

echo "==========================="
echo "=     Build Com_server    ="
echo "==========================="
echo

cd cyu/Com_server

echo "Start building"
sudo mkdir build
cd build
cmake ..
echo "Start compiling"
make

cd ../..

echo "==========================="
echo "=          Done !         ="
echo "==========================="
