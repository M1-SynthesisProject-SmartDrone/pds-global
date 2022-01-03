/**
 * @brief Main loop of the system
 * 
 * @author Sylvain Colomer
 * @date 19/04/19
 * @version 1.1 
 */

#ifndef ENGINE_MAINLOOP_H_
#define ENGINE_MAINLOOP_H_

#include <iostream>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <stack> 

#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>


//#include <loguru.hpp>

#include "blc_channel.h"
#include "blc_program.h"

#include "../include/Com_WifiPort.h"
#include "../include/Com_SerialPort.h"
#include "../include/Thread_SerialPort.h"
#include "../include/DataListener_Keyboard.h"
#include "../include/Display_MainFrame.h"
#include "../include/Data_Bus.h"
#include "../include/Data_Drone.h"
#include "../include/Com_Mavlink.h"
#include "../include/Constants.h"

/**
 * Engine classes
 */
class Engine{

private:

    // LOOP VAR
    /**
     * timeval that save the begin of the running method
     */
    struct timeval begin;
    /**
     * timeval use to control the period of the real time thread
     */
    struct timeval front_checkpoint;
    /**
     * timeval use to control the period of the real time thread
     */
    struct timeval end_checkpoint;

    /**
     * Represent the period between all task in ms
     */
    int task_period = 1000;
    /**
     * Var use to define when a task is overcome is execution delay
     */
    int task_deadline = 1000; //ms

    /**
     * Mutex of the run flag
     */
    std::mutex *runFlag_mutex;
    /**
     * Run flag : convey to loop the system
     */
    volatile bool runFlag = true;

	//%% EVENTS
	/**
	 * Main state of the Engine
	 */
	int state_engine_com=ENGINE_COM_INIT_1;
	int state_engine_drones = ENGINE_DRONES_INIT;
	/*int state_engine_com = ENGINE_COM_WORK;
	int state_engine_drones = ENGINE_DRONES_INIT;*/

	//%% EVENT LIST
	std::stack <int> events;
	std::mutex *events_mutex;
	
	//%% OBJECT
	//IHM
	std::shared_ptr<MainFrame> MainFrame_1;
	//Interface
	std::shared_ptr<Serial_Port> serial_Port_1;
	//Listener
	std::shared_ptr<Thread_SerialPort> dataListener_SerialPort_1;
	std::shared_ptr<DataListener_Keyboard> dataListener_Keyboard_1;
	//Data Bus
	std::shared_ptr<Bus> data_Bus_1;
	std::map<int, Drone> drones; // Drone list
	//Tools
	std::shared_ptr<MavlinkTools> mavlink1 = std::shared_ptr<MavlinkTools>(new MavlinkTools());

	//%% BUFFERS
	std::string buffer1="";
	struct dirent * dp;
	DIR* dirp;

	//%% PARAMETERS
	std::string device_path = "ttyUSB0"; //ttyACM0
	int device_baudrate = 57600;

	
public:
	
	Engine();

	virtual ~Engine() = default;

	bool test_communication();

	void main_loop();
	void run();
	void print_title();

	//GETTERS AND SETTERS
	std::shared_ptr<MainFrame> getMainFrame_1()
	{
		return MainFrame_1;
	}

	void add_event(int event){
		events_mutex->lock();
		events.push(event);
		events_mutex->unlock();
	}

	void stop();
	void stop_force();
	void join();
	bool isRunFlag() ;

};

#endif // ENGINE_MAINLOOP_H_


