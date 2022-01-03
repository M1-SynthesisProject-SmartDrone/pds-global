/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 */
 
#include <common/mavlink.h>

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

#include "../include/Serial_port.h"
#include "../include/Drone.h"

#include "../include/Serial_Port_ReadingThread.h"



mavlink_command_long_t mavlink_newCommand(){
    mavlink_command_long_t com;
    com.target_system = 0;
    com.target_component = 0.;
    com.command = 0;
    com.confirmation = false;
    com.param1=0; 
    com.param2=0; 
    com.param3=0; 
    com.param4=0; 
    com.param5=0; 
    com.param6=0; 
    com.param7=0;
    return com;
}

void automatic_initialisation(std::shared_ptr<Serial_Port> serial1, std::shared_ptr<Drone> drone1, uint limit)
{
    mavlink_message_t mavlink_message;
    bool flag_test = true;
    bool flag_success = true;
    uint counter=0;
    while(flag_test)
    {
        std::cout<<"-> Wait for drone heartbeat"<<std::endl;

        
        flag_success = serial1.get()->read_message(mavlink_message);
        if(flag_success) // Test about heatbeat
        {
            std::cout<<"-> Succes !"<<std::endl;
            std::cout<<"-> Extraction of drone data !"<<std::endl;

            drone1.get()->system_id = mavlink_message.sysid;
            drone1.get()->component_id = mavlink_message.compid;

            printf("--> sysid: %d\n", mavlink_message.sysid); // System ID
            printf("--> compid: %d\n", mavlink_message.compid); // Component ID

            
            flag_test=false;
        }
        
        if(counter >= limit) //Computation limit test
        {
            std::cout<<"Error: no drone detected"<<std::endl;
            exit(1);
        }
        counter++;
        usleep(1000);
    }       
}


/**
 * Only the main exe of the programm
 * @return
 */
int main(int argc, char* argv[]){

    // General var and initialisation
    bool flag_run = true; //Flag use for the main loop of the system
    std::string buffer_command = "";
    mavlink_message_t message;
    std::string device_path = "/dev/ttyUSB0";
    int device_baudrate = 57600;
    
    // General object
    std::shared_ptr<Drone> drone1;
    std::shared_ptr<Serial_Port> serial1;
    std::shared_ptr<Serial_Port_ReadingThread> readingThread;

    // BEGIN
    std::cout << "Pixhawk server" <<std::endl;

    //Object creation
    drone1 = std::shared_ptr<Drone>(new Drone());
    serial1 = std::shared_ptr<Serial_Port>(new Serial_Port(device_path.c_str(), device_baudrate));
    readingThread = std::shared_ptr<Serial_Port_ReadingThread>(new Serial_Port_ReadingThread(1000, 200, serial1, drone1));

    //Openning of the serial port
    std::cout<<"1. Serial port "<<device_path<<" oppening"<<std::endl;
    try
    {
        serial1.get()->open_serial();
    }
    catch(int e) 
    {
        std::cout<<"Serial port error"<<std::endl;
        return 1;
    }

    //Detecting device and pulling drone information
    std::cout<<"2. Update of drone data"<<std::endl;
    automatic_initialisation(serial1, drone1, 10000);

    // test of arm
    std::cout<<"Waiting for shutdown command 'q'" <<std::endl;

    while(flag_run){

        std::cin >> buffer_command;

        if(buffer_command=="a")
        {
            std::cout<<"Arming drone"<<std::endl;
            // Encode
            mavlink_command_long_t command=mavlink_newCommand();
            command.target_system = 1;
            command.target_component = 1;
            command.command = MAV_CMD_COMPONENT_ARM_DISARM;
            command.confirmation = false;
            command.confirmation     = 0;
            command.param1           = 1;

            mavlink_msg_command_long_encode(255, drone1.get()->component_id, &message, &command);
            serial1.get()->write_message(message);
            usleep(1000);
        }
        else if(buffer_command=="!a")
        {
            std::cout<<"Un-arming drone"<<std::endl;
            // Encode
            mavlink_command_long_t command=mavlink_newCommand();
            command.target_system = 1;
            command.target_component = 1;
            command.command = MAV_CMD_COMPONENT_ARM_DISARM;
            command.confirmation = false;
            command.confirmation     = 0;
            command.param1           = 0;

            mavlink_msg_command_long_encode(255, drone1.get()->component_id, &message, &command);
            serial1.get()->write_message(message);
            usleep(1000);
        }
        else if(buffer_command=="m")
        {

            std::cout<<"Set mode guided"<<std::endl;
            mavlink_command_long_t command=mavlink_newCommand();
            command.target_system = 1;
            command.target_component = 1;
            command.command          = MAV_CMD_NAV_GUIDED_ENABLE;
            command.confirmation     = true;
            command.param1           = 1; // flag >0.5 => start, <0.5 => stop

	        mavlink_msg_command_long_encode(255, drone1.get()->component_id, &message, &command);
            serial1.get()->write_message(message);
            usleep(1000);
        }
        else if(buffer_command=="!m")
        {

            std::cout<<"Outset mode guided"<<std::endl;
            mavlink_command_long_t command=mavlink_newCommand();
            command.target_system = 1;
            command.target_component = 1;
            command.command          = MAV_CMD_NAV_GUIDED_ENABLE;
            command.confirmation     = true;
            command.param1           = 0; // flag >0.5 => start, <0.5 => stop

	        mavlink_msg_command_long_encode(255, drone1.get()->component_id, &message, &command);
            serial1.get()->write_message(message);
            usleep(1000);
        }
        else if(buffer_command=="t")
        {
            std::cout<<"Play tone"<<std::endl;

            mavlink_msg_play_tune_pack(255, drone1.get()->component_id, &message, 1, 1 , "AAAA", "");

            serial1.get()->write_message(message);
            usleep(1000);
        }
        else if(buffer_command=="q")
        {
            std::cout<< "Command: exit from system" << std::endl;
            flag_run = false;
        }
        else
        {
            std::cout<< "Error: don't understand what you say" << std::endl;
        }
    }

    serial1.get()->close_serial();

    return 0;
}
