#!/bin/bash
#
# This script will clone all the blc libraries needed for the Com_server
# project to compile and launch.
# We need sudo rights in order to process properly.
# A lot of folder will be created, and no one will be erased after
#
# Author : Aldric Vitali Silvestre


# If want to remove all folders (downloaded and build) after installation, set this to true
do_remove_folders=true

set -o errexit  #Exit on first error
set -o nounset  #Error if a variable is used but nont defined
# set -o pipefail #Error if a pipe fail

# Ask for sudo rights and relaunch script as sudo if not the case
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

# The associative array with :
#   - key = name of the library
#   - value = link to the wanted repos

# WARN : don't forget to rename also in the libs_index array !!
declare -A libs=(
    [blc_core]=https://git.cyu.fr/blaar/blibs/blc_core.git
    [blc_channel]=https://git.cyu.fr/blaar/blibs/blc_channel.git
    [blc_image]=https://git.cyu.fr/blaar/blibs/blc_image.git
    [blc_network]=https://git.cyu.fr/blaar/blibs/blc_network.git
    [blc_program]=https://git.cyu.fr/blaar/blibs/blc_program.git
    [blc_process]=https://git.cyu.fr/blaar/blibs/blc_process.git
)

# An array to grant order of associative array
# blc must be the first to be intalled, as the others need him
declare -a libs_index=(blc_core blc_channel blc_image blc_network blc_program blc_process)

echo "==========================="
echo "=  Install blc libraries  ="
echo "==========================="
echo

echo
echo "==== Install required packages ===="
echo
sudo apt-get update -y
sudo apt-get install git g++ cmake ninja-build doxygen lsof libpng-dev libjpeg-dev gnuplot fftw3-dev libsndfile1-dev libasound2-dev libv4l-dev libgtk-3-dev -y

echo
echo "==== Start installations ===="
echo

# For each lib name, get the lib url, clone it and install it
for name in "${libs_index[@]}";
do
    echo "** Install $name **"
    echo
    url=${libs[$name]}

    # The build will be in another folder
    name_build_folder="${name}_build"

    echo "Check if folder already exists"
    # If folders are already present, stop all
    [ -d "${name}" ] && { echo "${name} directory already exist. Remove it or change the current install directory."; exit 1; }
    [ -d  "${name_build_folder}" ] && { echo "${name_build_folder} directory already exist. Remove it or change the current install directory."; exit 1; }

    echo "Clone ${name}"
    git clone $url

    echo "Create folder for ${name_build_folder} and start cmake"
    sudo mkdir $name_build_folder && cd $name_build_folder && cmake "../${name}"
    echo "Make and install lib"
    make
    sudo make install
    # Don't forget to return back to the main folder
    cd ..
    if $do_remove_folders; then
        echo "Remove folders ${name} and ${name_build_folder}"
        sudo rm -r "${name}" "${name_build_folder}"
    fi
    echo
done
echo "Refresh libs cache"
sudo ldconfig
echo "Done !"
echo