/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 */

#include "../include/Cmd_ControlThread.h"
#include "../include/global_variables.h"

Serial_Port_WritingThread::Serial_Port_WritingThread(int task_period, int task_deadline):
    Abstract_ThreadClass(task_period, task_deadline)
{
 //   Serial_Port_WritingThread::drone1 = drone1;

    //Channels use to arm the drone
    blc_control_arm= std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.control.arm", BLC_CHANNEL_READ, 'IN16', 'NDEF', 1, 1)); // 1=on, 0=off

    //Channels use to control manually the drone
    blc_control_remote_vector = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.control.remoteVectors", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 4));
}

Serial_Port_WritingThread::~Serial_Port_WritingThread()
{

}


int Serial_Port_WritingThread::main_loop()
{

    //Main loop of the system
    switch (drone1.mode==DRONE_OFF)
    {
        
    }
    // We take care only for difference in mode control
    /*if(blc_message_arm.ints16[0] ==1){

    }*/

    return 0;
}


void Serial_Port_WritingThread::run()
{
    long long currentThreadDelay;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    currentState = LifeCoreState::RUN;

    while(isRunFlag())
    {
        usleep(task_period);

        gettimeofday(&end_checkpoint, 0);
        currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);

        if (currentThreadDelay > task_period )
        {
            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {
                currentState = LifeCoreState::DEADLINE_EXCEEDED;
            }
            else 
            {  //PG: strange code (check older version)
                currentState = LifeCoreState::RUN;
                main_loop(); // PG: strange name?
            }
        }
    }
}

