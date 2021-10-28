/**
 * @brief Tool class which convey to open a serial port
 * @file serial_port.cpp
 * 
 * @author Sylvain ColomerJoystickModeAttitude
 * @date 19/04/19
 * @version 1.1 
 */

#ifndef DRONE_H
#define DRONE_H


#include <common/mavlink.h>

#include <iostream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <mutex>
#include <sys/time.h>

/**
 * define about drone mode
 */
enum Drone_mode{
	DRONE_OFF,
	DRONE_MANUAL_DIRECT // Mode manual with direct control of the motors
};

// State 

/**
 * Drone classes
 */
class Drone
{

public:

	//Drone state var
	int drone_state= -1;

	//Drone last message timestamp
	int last_timestamp = 0;

	//Drone system var
	uint8_t system_id=-1;
	uint8_t component_id=-1;
	uint8_t autopilot_id=-1;

	// Var use in remote mode to control the drone. 
	// Warning: if the value are highter than 0 before, it will be dangerous because the drone will make an hight jump
	int current_x=0;
	int current_y=0;
	int current_z=0;
	int current_r=0;

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//	Drone();

//	~Drone();


};



#endif // SERIAL_PORT_H_


