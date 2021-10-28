/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 */

 
#include <iostream>
#include <string>

using namespace std;

#include "../include/DataListener_Keyboard.h"
#include "../include/Engine_mainLoop.h"

DataListener_Keyboard::DataListener_Keyboard(int task_period, int task_deadline):
    Abstract_ThreadClass(task_period, task_deadline)
{
}

DataListener_Keyboard::~DataListener_Keyboard()
{

}

void DataListener_Keyboard::run()
{
    long long currentThreadDelay;
    //bool success=false;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

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
                cout<<"keayboard delay issue :\n";
            }
            else
            {
               cout<<"clavier :\n";
                getch_value = getch();
                
                switch(getch_value)  // the real value
                {
                    // Refresh IHM
                    case 'r' :
                        //DLOG_F(INFO, "Command refresh");
                        Bus::getInstanceOf().getMainFrame().get()->add_log("Refresh command", NCURSES_TEXT_GREEN);
                        Bus::getInstanceOf().getMainFrame().get()->repaint();
                        //Bus::getInstanceOf().getEngine()->add_event(EVENT_REPAINT);
                        break;
                    
                    // Demonstration
                    case 'd' : //Not protected -> very dangerous during controled fly
                        //DLOG_F(INFO, "Demo");
                        Bus::getInstanceOf().getMainFrame().get()->add_log("Demo command: warning only for test", NCURSES_TEXT_RED);
                        Bus::getInstanceOf().getMainFrame().get()->repaint();

                        //Bus::getInstanceOf().getSerialPort().get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_arm(0.8));
                        sleep(1);
                        
                        //Bus::getInstanceOf().getSerialPort().get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_arm(0.2));

                        Bus::getInstanceOf().getMainFrame().get()->add_log("Demo command: succes", NCURSES_TEXT_RED);
                        Bus::getInstanceOf().getMainFrame().get()->repaint();

                        //Bus::getInstanceOf().getEngine()->add_event(EVENT_DEMO);
                        break;

                    // Quit programm
                    case 'q' : // ECHAP
                        //DLOG_F(INFO, "Command exit");
                        Bus::getInstanceOf().getMainFrame().get()->add_log("Exit command", NCURSES_TEXT_RED);
                        Bus::getInstanceOf().getMainFrame().get()->repaint();
                        Bus::getInstanceOf().getEngine()->stop();
                        //Bus::getInstanceOf().getEngine()->add_event(EVENT_EXIT);
                        break;
                } 
            }
        }
    }
}

