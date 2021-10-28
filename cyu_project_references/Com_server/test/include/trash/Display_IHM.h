/**
 * @brief Tool class which convey to open a serial port
 * @file serial_port.cpp
 * 
 * @author Sylvain Colomer
 * @date 19/04/19
 * @version 1.1 
 */

#ifndef DISPLAY_Display_IHM_H
#define DISPLAY_Display_IHM_H

#include <cstdlib>
#include <iostream>
#include <stdio.h>  
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <memory>


/**
 * IHM Class. Interface use to make an abstraction between the interface system and the data class. 
 */
class Display_IHM {

private:

	int screen_widht=1000, screen_height=500;

public:
	
	Display_IHM();

	~Display_IHM();

	void printMessage(std::string message);

	void printLog(std::string message);


};

#endif // SERIAL_PORT_H_


