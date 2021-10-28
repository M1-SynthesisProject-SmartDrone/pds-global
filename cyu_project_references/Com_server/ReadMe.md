[![Awesome Badges](https://img.shields.io/badge/Laboratory-Etis-purple.svg)](https://www-etis.ensea.fr)
[![Awesome Badges](https://img.shields.io/badge/Team-Neurocyber-red.svg)](https://www-etis.ensea.fr)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](sylvain.colomer.pro@gmail.com)

# drone-controlServer  #

To install
> cmake .
> make

## Introduction ##

The pixhawk service program is a simple application that allow user to communicate though Mavlink to a pixhawk flying controler. It use Blc channels, a lib to use shared memory make by a collegue, Arnaud Blanchard (See blaar lib).


## Application structure ##

The application is a C++ service use to send messages on a mavlink drone. It operation is simple : 
* The drone upon ignition sends data called heartbeat
* The system watch over available drone with a DataListenerThread. It's a thread that listen data.
* If a drone is available, the system advertise user an begin to display heartbeat information

The project word in real time with multiple thread : 
* Serial port thread in reading and writing 
* IHM thread
* Joystick thread
* Main thread

![Image of structure application](./doc/images/program.svg)


### Language ###

C++

### Dependencies ###

This project use :
* https://promethe.u-cergy.fr/blaar/blaar -> important
* librapidxml-dev 1.13-1
* libncurses5-dev


### Organisation ###
The project is organized in different specific folders : 
* bin : application exe folder
* build: cmake folder 
* data : explicit
* include : all header of the application
* lib : all lib use in the project like Mavlink v2.0
* log : classic
* src : all the source file of the project
* test : unit test use by the system

## How to ? ##

#  see blc_channels

cd $HOME/blaar
./run.sh gnuplot/o_gnuplot /pixhawk.control.arm
./run.sh gnuplot/o_gnuplot /pixhawk.control.motors -m-1000 -M1000

### Resolve serial port problem ?

http://tvaira.free.fr/bts-sn/activites/preparation-ccf-e52/activite-port-serie-usb.html

### Compilate the project ###


Don't lose your time with greedy IDE :P. Use Cmake in the build folder.
 
 ```
 mkdir build
 cd build
 cmake ..
 make

 ```
 
 ### Execute tests ###
 
No unitary test support was made but the system was prepared. It's only a simple option on cmake.

## Other ##
Some links are interresting to consult to contributre to this program.

* https://www.google.com/search?client=ubuntu&channel=fs&q=mavlink+mode+guided&ie=utf-8&oe=utf-8
* https://gitter.im/dronekit/dronekit-python/archives/2017/06/04
* ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html -> tone
* https://dev.px4.io/en/qgc/video_streaming.html 
* https://dev.px4.io/en/advanced/parameter_reference.html -> pixhawk flight stack
* https://msgpack.org/ -> format
* https://capnproto.org/ -> format
* https://docs.qgroundcontrol.com/en/SetupView/Joystick.html JOYSTICK

### Wiki ###


### Author ###
Sylvain Colomer into the Laboratory Etis, University of Cergy-Pontoise
Don't hesitate to contact me trough gitLab !

