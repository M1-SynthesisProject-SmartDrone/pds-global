#include "../include/Engine_mainLoop.h"

#include <iostream>
#include <string>

using namespace std;

Engine::Engine()
{
    //1. Init interface. don't use real time context
    MainFrame_1 = std::shared_ptr<MainFrame>(new MainFrame(100000, 100000));
    MainFrame_1.get()->repaint();
    
    //2. Initialisation
    MainFrame_1.get()->setState("INIT");
    MainFrame_1.get()->add_log("Welcome to pixhawk server", NCURSES_TEXT_GREEN);
    MainFrame_1.get()->repaint();
    sleep(1);
    MainFrame_1.get()->add_log("Initialisation", NCURSES_TEXT_BLUE);
    MainFrame_1.get()->repaint();

    //3. Init other objects
    MainFrame_1.get()->add_log("-> Object", NCURSES_TEXT_GREEN);
    MainFrame_1.get()->repaint();
    dataListener_Keyboard_1 = std::shared_ptr<DataListener_Keyboard>(new DataListener_Keyboard(10000, 1000));
    serial_Port_1 = std::shared_ptr<Serial_Port>(new Serial_Port(device_path, device_baudrate)); // Default creation
    dataListener_SerialPort_1 = std::shared_ptr<Thread_SerialPort>(new Thread_SerialPort(1000, 1000, serial_Port_1)); // Default creation too    

    //4. Init bus
    MainFrame_1.get()->add_log("-> Communication Bus", NCURSES_TEXT_GREEN);
    MainFrame_1.get()->repaint();
    Bus::getInstanceOf().initBus(*this, MainFrame_1, serial_Port_1, mavlink1, drones);
    
    //5. Init thread
    MainFrame_1.get()->add_log("-> Thread", NCURSES_TEXT_GREEN);
    MainFrame_1.get()->repaint();
    dataListener_Keyboard_1.get()->start();
    //MainFrame_1.get()->start();

    //x. Launch core
    MainFrame_1.get()->setState("WORK");
    MainFrame_1.get()->add_log("-> Launching main core", NCURSES_TEXT_GREEN);
    events_mutex = new std::mutex(); //mutex init runFlag_mutex
    runFlag_mutex = new std::mutex(); // Mutex init
    MainFrame_1.get()->repaint();

    run();
}

void Engine::run()
{
    long long currentThreadDelay;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    while(runFlag)
    {
        usleep(task_period);
        gettimeofday(&end_checkpoint, 0);
        currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);

        if (currentThreadDelay > task_period )
        {
            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {

            }
            else
            {
                main_loop();   
            }
        }
    }

    //%% CLEANING %%
    serial_Port_1.get()->close_serial();
    dataListener_Keyboard_1.get()->stop();
    dataListener_Keyboard_1.get()->join();
    MainFrame_1.get()->stop();
    MainFrame_1.get()->join();
}


/**
 * Main loop of the engine
 */
void Engine::main_loop()
{
    //EVENTS LOOP -> don't work
    events_mutex->lock();
    if(!events.empty())
    {
        switch (events.top())
        {
            case EVENT_DEMO :
                MainFrame_1.get()->add_log("Demo command received", NCURSES_TEXT_GREEN);
                MainFrame_1.get()->repaint();
                break;

            case EVENT_EXIT :
                MainFrame_1.get()->add_log("Validation of exit command", NCURSES_TEXT_RED);
                MainFrame_1.get()->repaint();
                sleep(1);
                runFlag = false;
                break;
        }
        events.pop();
    }
    events_mutex->unlock();

    // ENGINE COMMUNICATION AUTOMATE -> Set always communication working 
    if(state_engine_com == ENGINE_COM_INIT_1) //We seek a comm system
    {       
        bool flag_result = false;

        MainFrame_1.get()->add_log("Research of communication dongle", NCURSES_TEXT_BLUE);

        dirp = opendir("/dev");
        while ((dp = readdir(dirp)) != NULL) 
        {
            if(std::string(dp->d_name).compare(device_path) == 0)
            {
                MainFrame_1.get()->add_log("-> Communication system found", NCURSES_TEXT_GREEN);
                MainFrame_1.get()->repaint();
                flag_result = true;
            }

            if(flag_result) // Comm system found
            {
                state_engine_com = ENGINE_COM_INIT_2;
            }
            else  //No comm
            {
                MainFrame_1.get()->add_log("-> No communication system found.", NCURSES_TEXT_RED);
                MainFrame_1.get()->repaint();
                usleep(100000);
            }
        }
    }
    else if(state_engine_com == ENGINE_COM_INIT_2) //Openning a serial connection 
    { 
        MainFrame_1.get()->add_log("Openning a serial connection "+device_path+" with "+std::to_string(device_baudrate)+" bauds", NCURSES_TEXT_BLUE);
        serial_Port_1.get()->setUartName("/dev/"+device_path);
        serial_Port_1.get()->setBaudrate(device_baudrate);            
        
        if(serial_Port_1.get()->open_serial()==0)
        { 
            //DLOG_F(INFO, "Serial port open");
            MainFrame_1.get()->add_log("-> Serial port open", NCURSES_TEXT_GREEN);
            MainFrame_1.get()->add_log("Research a drone", NCURSES_TEXT_BLUE);
            MainFrame_1.get()->repaint();
            state_engine_com = ENGINE_COM_WORK;
            
        }else
        {
            //DLOG_F(ERROR, "Serial port wan't be open");
            MainFrame_1.get()->add_log("-> Serial port wan't be open. Retry", NCURSES_TEXT_RED);
            MainFrame_1.get()->repaint();
            usleep(100000);
        }
    }
    else if(state_engine_com == ENGINE_COM_WORK)
    {
        // TEST COMMUNICATION
        /*if(test_communication() == false)
        {
            state_engine_com = ENGINE_COM_ERROR;
        }*/
    }
    else if(state_engine_com == ENGINE_COM_ERROR)
    {
    }
    /*char mon_message[245];
	sprintf(mon_message, "engine com = %d drone = %d \n", state_engine_com, state_engine_drones);
	MainFrame_1.get()->add_log(mon_message, NCURSES_TEXT_GREEN);
    MainFrame_1.get()->repaint();*/
   
    if(state_engine_com == ENGINE_COM_WORK && state_engine_drones == ENGINE_DRONES_INIT) // We can only enter in engine drone state if communiation work // we only work on the first drone
    {
        mavlink_message_t mavlink_message;

        // the framerate of this loop need to be high to found a drone
        if (mavlink1.get()->read_message_serial(mavlink_message, serial_Port_1)) 
        {
            MainFrame_1.get()->add_log("-> Drone found ", NCURSES_TEXT_GREEN);
            MainFrame_1.get()->repaint();

            drones.insert( {mavlink_message.sysid, Drone() } );
            drones[mavlink_message.sysid].system_id = mavlink_message.sysid;
            drones[mavlink_message.sysid].component_id = mavlink_message.compid;

            //Launch data listener and worker
            MainFrame_1.get()->add_log("-> Launch serial port Listener", NCURSES_TEXT_GREEN);
            MainFrame_1.get()->repaint();
            dataListener_SerialPort_1.get()->start();
   //         cout<<"fin init \n";

            state_engine_drones = ENGINE_DRONES_WORK;
        }
    }
    else if(state_engine_com == ENGINE_COM_WORK && state_engine_drones == ENGINE_DRONES_WORK) 
    {
    }
}

/**
 * Return a flag that indicate an hardware deconnexion
 * To test
 */
bool Engine::test_communication()
{
    dirp = opendir("/dev");
    while ((dp = readdir(dirp)) != NULL) 
    {
        if(std::string(dp->d_name).compare(device_path) == 0){ // Si on trouve le port dans le /dev
            return false;
        }
    }
    return true;
}


void Engine::stop()
{
   // DLOG_F(INFO, "Stop one thread");
   cout<<"Stop one thread\n";
    runFlag_mutex->lock();
    runFlag = runFlag = false;
    runFlag_mutex->unlock();
}

void Engine::stop_force()
{
    cout<<"Force Stop one thread\n";
    runFlag = false;
}

bool Engine::isRunFlag() 
{
    return runFlag;
}



// SNIPPET CODE FOR ENGINE_COM_ERROR
/*
if(flag_lost==true && state_communication!=STATE_COM_ERROR){ // test 1: if we detect a deconnexion
    MainFrame_1.get()->add_log("Communication system lost", NCURSES_TEXT_RED);
    MainFrame_1.get()->setCommunicationType(STATE_COM_ERROR);

    MainFrame_1.get()->add_log("Close serial connection", NCURSES_TEXT_RED);
    serial_Port_1.get()->close_serial();

    state_communication = STATE_COM_ERROR;
    MainFrame_1.get()->repaint();
} 
else if(flag_lost==false && state_communication==STATE_COM_ERROR) // test 2: if we detect a reconnexion
{ 
    MainFrame_1.get()->add_log("Communication system retablished", NCURSES_TEXT_GREEN);
    MainFrame_1.get()->setCommunicationType(STATE_COM_WORK);

    MainFrame_1.get()->add_log("Open serial port", NCURSES_TEXT_BLUE);
    if(serial_Port_1.get()->open_serial()==0)
    { 
        DLOG_F(INFO, "Serial port open");
        MainFrame_1.get()->add_log("Serial port open", NCURSES_TEXT_GREEN);
        
    }else{
        DLOG_F(ERROR, "Serial port wan't be open");
        MainFrame_1.get()->add_log("Serial port wan't be open. Retry", NCURSES_TEXT_RED);
    }

    state_communication = ENGINE_COM_WORK;
    MainFrame_1.get()->repaint();
}    
*/ 

int main()
{
   Engine engine;
   return 1;
}
