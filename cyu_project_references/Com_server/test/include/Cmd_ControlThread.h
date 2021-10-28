/**
 * @author Sylvain Colomer
 * @date 17/08/18.
 */

#ifndef SERIAL_PORT_WRITINGTHREAD_H
#define SERIAL_PORT_WRITINGTHREAD_H

#include <iostream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <mutex>
#include <sys/time.h>

#include "Abstract_ThreadClass.h"
#include "Drone.h"
#include "Display_IHM.h"

#include "blc_channel.h"
#include "blc_program.h"

class Serial_Port_WritingThread : public Abstract_ThreadClass {

protected: 

    /**
     * Drone buffer
     */
    std::shared_ptr<Drone> drone1;

    // Different buffer use for get the data
    int time_stamps;

    //Channels use to arm the drone
    std::shared_ptr<blc_channel> blc_control_arm; // 1=on, 0=off

    //Channels use to control manually the drone
    std::shared_ptr<blc_channel> blc_control_remote_vector; //x, y, z, r


public:

    /**
     * Default constructor of the class. Need to have the 
     */
    Serial_Port_WritingThread(int task_period, int task_deadline, std::shared_ptr<Drone> drone1);

    ~Serial_Port_WritingThread();

    void run();

    /**
     * Direct writing method command
     */
    int main_loop();

};

#endif //VEHICLEWATCHER_Abstract_ThreadClass_H
