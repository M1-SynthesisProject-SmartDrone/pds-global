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
#include "Data_Drone.h"

#include "blc_channel.h"
#include "blc_program.h"

class Serial_Port_WritingThread : public Abstract_ThreadClass 
{
protected: 

    /**
     * Drone buffer
     */
    //std::shared_ptr<Drone> drone1;

    // Different buffer use for get the data
    int time_stamps;

public:

    int task_period = 250;

    /**
     * Default constructor of the class. Need to have the 
     */
    Serial_Port_WritingThread(int task_period, int task_deadline);
    ~Serial_Port_WritingThread();

    void run();
    
    void set_task_period(int period);

    /**
     * Direct writing method command
     */
    int main_loop();
};

#endif //VEHICLEWATCHER_Abstract_ThreadClass_H
