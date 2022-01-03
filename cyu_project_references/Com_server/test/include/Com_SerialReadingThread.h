/**
 * @author Sylvain Colomer
 * @date 17/08/18.
 */

#ifndef SERIAL_PORT_READINGTHREAD_H
#define SERIAL_PORT_READINGTHREAD_H

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <mutex>
#include <sys/time.h>
//#include <mavlink_msg_compassmot_status>







#include "Abstract_ThreadClass.h"
#include "Data_Drone.h"

#include "blc_channel.h"
#include "blc_program.h"

class Serial_Port_ReadingThread : public Abstract_ThreadClass 
{
protected: 
    /**
     * Serial port use with the drone
     */
    std::shared_ptr<Serial_Port> serial1;
    
    /**
     * Drone buffer
     */
    //std::shared_ptr<Drone> pt_drone;

    /**
     * Mavlink message dictionnary
     */
    std::map<int, std::string> mavlink_dic;


public:
    
    Serial_Port_ReadingThread(int task_period, int task_deadline);
    ~Serial_Port_ReadingThread();

    int task_period = 500;

    void run();

    uint64_t get_time_usec();
    
    /**
     * Reading function of mavlink message and diffusion by shared memory
     */
    void read_messages(mavlink_message_t message);
    int testAck(mavlink_command_ack_t *ack, std::string message);
    void handle_command_ack(mavlink_command_ack_t *ack);
};

#endif //VEHICLEWATCHER_Abstract_ThreadClass_H
