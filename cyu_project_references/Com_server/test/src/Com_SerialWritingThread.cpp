/**
 *  @author Sylvain Colomer, P. Gaussier
 *  @date 18/04/19.
 */

#include <iostream>
#include <string>

using namespace std;

#include "../include/Com_SerialWritingThread.h"
#include "../include/global_variables.h"



//#define DISPLAY


Serial_Port_WritingThread::Serial_Port_WritingThread(int task_period, int task_deadline):
    Abstract_ThreadClass(task_period, task_deadline)
{
//    Serial_Port_WritingThread::drone1 = drone1;

}

Serial_Port_WritingThread::~Serial_Port_WritingThread()
{

}

void Serial_Port_WritingThread::set_task_period(int period)
{
	cout<<"\n\n\n                               >>>>>>>>>>>>>>>>>                                    set_task_period = "<<period<<endl;
	task_deadline=period;

}

string string_currentState[]={"TO_INIT", "INIT", "READY", "RUN", "STOP", "QUIT", "PROBLEM", "DEADLINE_EXCEEDED"};

void Serial_Port_WritingThread::run()
{
    long long currentThreadDelay;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    currentState = LifeCoreState::RUN;
 //   cout<<__FUNCTION__<<endl;
    while(isRunFlag())
    {
	//	cout<<"write task period = "<<task_period<<endl;
        usleep(task_period);

        gettimeofday(&end_checkpoint, 0);
        currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);
  //      cout<<__FUNCTION__<<" wake up "<<endl;

        if (currentThreadDelay > task_period )
        {
            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {
                currentState = LifeCoreState::DEADLINE_EXCEEDED;
            }
            else 
            {
                currentState = LifeCoreState::RUN;
                main_loop();
                
            }
        }
  //      cout<<__FUNCTION__<<" end currentState = "<<currentState<<" "<<string_currentState[currentState]<<endl;
        
    }
}
	int i = 0;

int Serial_Port_WritingThread::main_loop()
{
    std::string buffer1="";
   
    //cout<<"__FUNCTION__"<<endl;

    if(blc_control_commands.floats[0]>0.5 && drone1.motors == UNARM)
    {
		//drone1.command_setMode(DRONE_OFF);		 
		cout<<" command_arm 1\n";
        drone1.command_arm(1);
        drone1.motors = ARM;
    }
    if(blc_control_commands.floats[1]>0.5 && drone1.motors == ARM)
    {
        cout<<" command_arm 0 \n";
        drone1.command_arm(0);
        drone1.motors = UNARM;
    }
   
    if(blc_control_commands.floats[2]>0.5 && drone1.motors == ARM)
    {
        cout<<" takeoff 0 \n";
        drone1.take_off();
        
      
    }
    
       if(blc_control_commands.floats[3]>0.5 && drone1.motors == ARM)
    {
        cout<<" landing 0 \n";
        drone1.landing();
    }
     if(blc_control_commands.floats[4]>0.5 && drone1.motors == ARM)
    {
        cout<<" Return to llaunch0 \n";
        drone1.return_to_launch(1);
    }
    
     if(blc_control_commands.floats[5]>0.5 && drone1.motors == ARM)
    {
        cout <<"UPPPPP 1\n";
		drone1.waypoint(0,0,5);
    }
    
       if(blc_control_commands.floats[6]>0.5 && drone1.motors == ARM)
    {
        cout <<"DOWNNN 1\n";
		drone1.waypoint(0,0,3);
    }
    

	if(blc_control_motors.floats[0]>0.5 && drone1.motors == ARM)
    {
		
		cout <<"move_to	_right 1\n";
		drone1.move_drone_to(0.5,0,0); //move right 0.5m/s
	}
	
	if(blc_control_motors.floats[1]>0.5 && drone1.motors == ARM)
    {	
		cout <<"move_to	left 1\n";
		drone1.move_drone_to(-0.5,0,0); //move left 0.5m/s
	}
	
		if(blc_control_motors.floats[2]>0.5 && drone1.motors == ARM)
    {	
		cout <<"move_forward 1\n";
		drone1.move_drone_to(0,0.5,0);
	}
	
		
		if(blc_control_motors.floats[3]>0.5 && drone1.motors == ARM)
    {	
		cout <<"move_backward 1\n";
		drone1.move_drone_to(0,-0.5,0);
	} 
	
	if(blc_control_motors.floats[4]>0.5 && drone1.motors == ARM)
    {
		cout <<"TURN RIGHT 1\n";
		drone1.command_directControl(0,0,0,1000); 
	}
	if(blc_control_motors.floats[5]<0.5 && drone1.motors == ARM)
    {
		cout <<"TURN LEFT 1\n";
		drone1.command_directControl(0,0,0,-1000); 
	} 

	/*	if(blc_control_commands.floats[7]>0.5 && drone1.motors == ARM)
    {
		cout <<"UP 1\n";
		drone1.command_directControl(0,0,0,1000); 
	}
	if(blc_control_commands.floats[8]<0.5 && drone1.motors == ARM)
    {
		cout <<"DOWN 1\n";
		drone1.command_directControl(0,0,0,-1000); 
	} */
/*	
	if(blc_control_commands.floats[7]>0.5 && drone1.motors == ARM)
    {
		cout <<"up 1\n";
		float u = 0.01;
		u = u + 0.01;
		drone1.waypointupdown(u);  
	}
	
	if(blc_control_commands.floats[8]>0.5 && drone1.motors == ARM)
    {
		cout <<"down 1\n";
		float d =0.01;
		d = d -0.01;
		drone1.waypointupdown(d);  
	}
	
	*/
	
	//drone1.return_to_launch(1);  // retour au position initiale 
	
 /*   float offset=0.05;
    if(blc_control_motors.floats[0]>0.5+offset || blc_control_motors.floats[1]>0.5+offset || blc_control_motors.floats[2]>0.5+offset || blc_control_motors.floats[3]>0.5+offset)
    {
       cout<<" drone1.command_directControl \n";
      drone1.command_directControl(blc_control_motors.floats[1],blc_control_motors.floats[0],blc_control_motors.floats[3],blc_control_motors.floats[2]);
    }
   else if(blc_control_motors.floats[0]<0.5-offset || blc_control_motors.floats[1]<0.5-offset || blc_control_motors.floats[2]<0.5-offset || blc_control_motors.floats[3]<0.5-offset)
    {
       cout<<" drone1.command_directControl \n";
     drone1.command_directControl(blc_control_motors.floats[1],blc_control_motors.floats[0],blc_control_motors.floats[3],blc_control_motors.floats[2]);
    }
  
			
	if(blc_control_motors.floats[0]>0.5+offset) // right command
   { 
         //printf("mémoire partagée : demande motor 0 \n"); 
         //blc_control_commands.floats[1] = 0.0;S
         drone1.command_right(1);				
   }	
   else if(blc_control_motors.floats[0]<0.5-offset)// left command
   { 
         //printf("mémoire partagée : demande motor < 0,5 \n"); 
         //blc_control_commands.floats[1] = 0.0;S
         drone1.command_left(1);
   }
			
//	while (blc_control_motors.floats[1]<0.5) //PB si on a un while ici PG !
//   {
		if(blc_control_motors.floats[1]>0.5+offset) // up/down axis command
         { 
						//printf("mémoire partagée : demande motors 1 > 0,5 \n"); 
						//blc_control_commands.floats[1] = 0.0;S
               drone1.command_up(1);
					//	action.set_takeoff_altitude(3.0);
         }
			else if(blc_control_motors.floats[1]<0.5-offset)  // PG : ????
         { 
						//printf("mémoire partagée : demande motor 1 < 0,5 \n"); 
						//blc_control_commands.floats[1] = 0.0;S
               drone1.command_down(1);
         }
					
//		}
	
			
	if(blc_control_motors.floats[3]>0.5) //pow command
   {
         //printf("mémoire partagée : demande d'augmentation de la puissance \n");
         //blc_control_commands.floats[0] = 0.0;
         drone1.command_pow(blc_control_motors.floats[3]);
				
   }

    // COMMAND FOR CONTROL THE DRONE. THIS PART OF THE THREAD NEED TO BE CHANGE BEFORE ALL APPLICATION
    // if(1){
    //     drone1.command_directControl(blc_control_motors.floats[1],blc_control_motors.floats[0],blc_control_motors.floats[3],blc_control_motors.floats[2]);
    // }
//}
    
    buffer1 = "Arm: "+std::to_string(arm);
    Display_IHM::getInstanceOf().printData(buffer1, 18, 1);*/

    /*switch (drone1.mode){
        case DRONE_OFF:        
            
        break;

        case DRONE_MANUAL_DIRECT:        
            
        break;
    */    
    
    blc_control_arm;
    blc_control_remote_vector;
    return 0;
}

