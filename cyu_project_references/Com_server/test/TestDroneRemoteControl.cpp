/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 * 
 * https://www.tldp.org/HOWTO/html_single/Text-Terminal-HOWTO/#colors
 * https://www.tldp.org/HOWTO/html_single/Text-Terminal-HOWTO/#colors
 * https://en.wikipedia.org/wiki/ANSI_escape_code
 * https://fr.wikipedia.org/wiki/Curses
 * http://www.termsys.demon.co.uk/vtansi.htm
 */

#include "include/PixhawkServer.h"
#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <thread>

//include "Com_server/action.h"



using namespace std;
using std::chrono::seconds;
using std::this_thread::sleep_for;

char device_path[255] = "/dev/ttyUSB2";
int device_baudrate = 57600;
//Drone drone1(device_path, device_baudrate);
Drone drone1;


    // General object
    shared_ptr<Serial_Port_ReadingThread> readingThread;
    shared_ptr<Serial_Port_WritingThread> writingThread;

ofstream MyFile;
ofstream MyFilew;
ofstream compassx;
ofstream compassy;

blc_channel blc_control_arm("/pixhawk.control.arm", BLC_CHANNEL_READ, 'IN16', 'NDEF', 1, 1); // 1=on, 0=off;
blc_channel blc_control_remote_vector("/pixhawk.control.remoteVectors", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 4);
    //Channels use to control manually the drone  
blc_channel blc_control_motors("/pixhawk.control.motors", BLC_CHANNEL_READ, 'FL32', 'NDEF', 1, 6); //x, y, z, r

    //Channels use to command the drone
blc_channel blc_control_commands("/pixhawk.control.commands", BLC_CHANNEL_READ, 'FL32', 'NDEF', 1, 7); // 1=on, 0=off
blc_channel blc_highres_imu("/pixhawk.sensors.imu", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 9);
	//Channel use to know altitude of drone
blc_channel blc_attitude("/pixhawk.sensors.attitude", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 7);
    //Channel use to know GPS values of drone
blc_channel blc_local_position_ned("/pixhawk.sensors.local_position_ned", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 6);
blc_channel blc_global_position("/pixhawk.sensors.global_position", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 3);

    //Different blc channel use to publish data
blc_channel blc_heartbeat; //Hearbeat message  PG: PB ????? pas d'init  pour l'instant ????
blc_channel blc_test_detection("/pixhawk.test.detection", BLC_CHANNEL_READ, 'IN16', 'NDEF', 1, 7);
int j = 0;

void command_loop()
{
	int ret; 
    mavlink_message_t message;
    string buffer_command = "";

	writingThread.get()->set_task_period(100000);
    cout<<"Enter command : \n";
    cin >> buffer_command;


/*_____________SET_MODE____________*/

//drone1.make_command_msg_rate(1);

//drone1.make_mode(1);
/*___________________________________*/

	
    if(buffer_command=="a")
    {
		cout<<"arm \n";
        drone1.command_arm(1);
        //drone1.make_mode(193);

    }
    else if(buffer_command=="!a")
    {
        
        drone1.command_arm(0);
    }
    else if(buffer_command=="h"){
	cout<<"HOME_POSITION \n";
	drone1.request_home_position(1,1,1);
	}
	 else if(buffer_command=="e"){
	cout<<"X_POSITION \n";
	drone1.get_home_x(1);
	}
    else if(buffer_command=="o")
    {
		cout<<"current_consumed :"<<drone1.battery_status.current_consumed<<endl;
	cout<<"=========================================="<<endl;
	cout<<"energy_consumed :"<<drone1.battery_status.energy_consumed<<endl;
	cout<<"=========================================="<<endl;
	cout<<"temperature :"<<drone1.battery_status.temperature<<endl;
	cout<<"=========================================="<<endl;
	cout<<"voltages :"<<drone1.battery_status.voltages[10]<<endl;
	cout<<"=========================================="<<endl;
	cout<<"current_battery :"<<drone1.battery_status.current_battery<<endl;
	cout<<"=========================================="<<endl;
	cout<<"battery_function :"<<drone1.battery_status.battery_function<<endl;
	cout<<"=========================================="<<endl;
	cout<<"type :"<<drone1.battery_status.type<<endl;
	cout<<"=========================================="<<endl;
	cout<<"battery_remaining :"<<drone1.battery_status.battery_remaining<<endl;
	cout<<"=========================================="<<endl;
	cout<<"time_remaining :"<<drone1.battery_status.time_remaining<<endl;
	cout<<"=========================================="<<endl;
	cout<<"charge_state :"<<drone1.battery_status.charge_state<<endl;

	}
    else if(buffer_command=="m")
    {
        
        drone1.followhold();
    }
    else if(buffer_command=="p")
    {
		drone1.go_to();
		
	}
    else if(buffer_command=="f")
    {
		drone1.take_off();
       cout<<"fly \n";
   ret = drone1.take_off();
      cout<<"fly ret = "<<ret<<endl;
      cout<<"test"<<drone1.altitude.altitude_local<<endl;
      //cout<<"pos"<<altitude.altitude_local<<endl;
	}
    else if(buffer_command=="g")
    {
		cout<<"===================CORRDONNEES===================="<<endl;
		cout<<"HOME_LATITUDE_DEG"<<drone1.home_position.latitude<<endl;
		cout<<"_______________________"<<endl;
		cout<<"HOME_LONGITUDE_DEG"<<drone1.home_position.longitude<<endl;
		cout<<"_______________________"<<endl;
		cout<<"HOME_ALTITUDE_DEG"<<drone1.home_position.altitude<<endl;
		cout<<"_______________________"<<endl;
		cout<<"HOME_X_metres"<<drone1.home_position.x<<endl;
		cout<<"_______________________"<<endl;
		cout<<"HOME_Y_metres"<<drone1.home_position.y<<endl;
		cout<<"_______________________"<<endl;
		cout<<"HOME_Z_metres"<<drone1.home_position.z<<endl;
		cout<<"_______________________"<<endl;
		cout<<"BOUSSOLE"<<drone1.highres_imu.xmag<<endl;
	} 
	
    else if(buffer_command=="!f")
    {
        //drone1.take(0);
    }

    else  if(buffer_command=="i")
    { 
       cout<<"return \n";
        drone1.return_to_launch(1);
    }
    else if(buffer_command=="!i")
    {
        
        drone1.return_to_launch(0);
    }
    else  if(buffer_command=="l")
    {
       cout<<"landing \n";
        drone1.landing();
    }
 
     /*if(buffer_command=="u")
    {
       cout<<"takeoff \n";
        drone1.command_up(1);
    }
    else if(buffer_command=="!u")
    {
        
        drone1.command_up(0);
    }
    */


   else if(buffer_command=="r")
    {
       cout<<"right \n";
        drone1.command_right(1);
    }
    else if(buffer_command=="!r")
    {
        
        drone1.command_right(0);
    }
    
    else if(buffer_command=="k")
    {
       cout<<"kill \n";
        drone1.command_kill(1);
    }
    else if(buffer_command=="!k")
    {
        drone1.command_kill(0);
    }
    else if(buffer_command=="j")
    {
        drone1.move_drone_to(1,0,0);
    }
    else if(buffer_command=="y"){
		drone1.move_to(1,1,1);
	}
	
	
    else if(buffer_command=="z")
    {
        cout<<"motor 1 \n";
        drone1.move_drone_to(0,0.01,0);  //move forward 0.01m/s
    }
    else if(buffer_command=="s")
    {
        cout<<"motor 1 \n";
        drone1.move_drone_to(0,-0.01,0);  //move backward 0.01m/s
    }
    else if(buffer_command=="d")
    {
        drone1.move_drone_to(0.01,0,0);  //move right 0.01m/s
    }
    else if(buffer_command=="q")
    {
        drone1.move_drone_to(-0.01,0,0);  //move left 0.01m/s
    }
    else if(buffer_command=="w")
    {
        drone1.move_drone_to(0,0,1);  //UP for 0.01m/s
    }
    else if(buffer_command=="x")
    {
         drone1.move_drone_to(0,0,-0.01);  //DOWN for 0.01m/s
    }
    else if(buffer_command=="c")
    {
        drone1.command_directControl(0,0,0,1000);
    }
    else if(buffer_command=="v")
    {
        drone1.command_directControl(0,0,0,-1000);
    }
    else if(buffer_command=="!m")
    {
        drone1.command_directControl(0,0,0,0);
    }
    else if(buffer_command=="tone")
    {
       cout<<"Tone \n";
        //Display_IHM::getInstanceOf().printLog("PLAY TONE");
        mavlink_msg_play_tune_pack(255, drone1.component_id, &message, 1, 1 , "AAAA", "");
        drone1.write_message(message);
    }
    /*else if (buffer_command=="g")
    { 
		 drone1.command_setModeGuided(1);
	}
    else if (buffer_command=="!g")
    { 
		 drone1.command_setModeGuided(0);
	}
	*/
    else
    {
       cout<<"command not recognized\n";
        //Display_IHM::getInstanceOf().printLog("Error: what ?");
    }

    //PARAM_REQUEST_READ
}




// else if(buffer_command=="1") // SET_ATTITUDE_TARGET
// {
//     Display_IHM::getInstanceOf().printLog("U");
//     mavlink_setAttitudeTarget(&message, *drone1.get(), 800, 0, 0, 0);
//     drone1.get()->write_message(message);
// }
// else if(buffer_command=="2") // SET_ATTITUDE_TARGET
// {
//     Display_IHM::getInstanceOf().printLog("");
//     mavlink_setAttitudeTarget(&message, *drone1.get(), 0, 800, 0, 0);
//     drone1.get()->write_message(message);
// }
// else if(buffer_command=="3") // SET_ATTITUDE_TARGET
// {
//     Display_IHM::getInstanceOf().printLog("");
//     mavlink_setAttitudeTarget(&message, *drone1.get(), 0, 0, 800, 0);
//     drone1.get()->write_message(message);
// }
// else if(buffer_command=="4") // SET_ATTITUDE_TARGET
// {
//     Display_IHM::getInstanceOf().printLog("");
//     mavlink_setAttitudeTarget(&message, *drone1.get(), 0, 0, 0, 800);
//     drone1.get()->write_message(message);
// }

/**
 * @fn void signalHandler(int number)
 * @brief Handles received signals. Triggered when a signal is received.
 *
 * @param number The signal code.
 **/
void signalHandler(int number) 
{
   switch(number) 
   {
   case SIGINT :
      printf("SIGINT caught\n"); //CTRL+C 
      exit(0);
      //TODO handle SIGINT reception situation
      break;

   case SIGTERM :
      printf("SIGTERM caught\n"); //kill (term command)
      //TODO handle SIGTERM reception situation
      exit(0);
      break;

   case SIGFPE :
      printf("SIGFPE caught\n"); //Floating-Point arithmetic Exception
      //TODO handle SIGFPE reception situation
      exit(0); //to prevent infinite loop
      
      break;

   default :
      printf("Signal number : %d caught\n", number);
   }
}

/**
 * Only the main exe of the programm
 * @return
 */
int main(int argc, char *argv[])
{
    // General var
  //  bool flag_run = true; //Flag use for the main loop of the system

   struct sigaction action;
   if(argc>1)  {strcpy(device_path ,argv[1]); printf("port = %s \n",device_path);}
  
   //signal handler initialisation
   sigfillset(&action.sa_mask);
   action.sa_handler = signalHandler;
   action.sa_flags = 0;

   //Make the list of all SIG to catch (will not be caught if not listed here, the list is not definitive and all are not necessarily usefull)
   if(sigaction(SIGINT, &action, NULL) != 0) printf("SIGINT couldn't be attached\n"); //ctrl+C
   if(sigaction(SIGTERM, &action, NULL) != 0) printf("SIGTERM couldn't be attached\n"); //kill [PID]
   if(sigaction(SIGFPE, &action, NULL) != 0) printf("SIGFPE couldn't be attached\n"); //integer divided by 0 
   drone1.open(device_path, device_baudrate);

    string buffer1="";

//    shared_ptr<Display_IHM> ihm;

   // drone1 = shared_ptr<Drone>(new Drone(device_path, device_baudrate));
  //  drone1 = new Drone(device_path, device_baudrate);
	
				     
//		ofstream MyFile("mesures.txt");
		MyFile.open("mesures.txt");    
		MyFilew.open("tcdrain.txt");    
		compassx.open("compassx.txt");
		compassy.open("compassy.txt");

   writingThread = shared_ptr<Serial_Port_WritingThread>(new Serial_Port_WritingThread(1000000, 200));  //beug ne marche pas 
																						
   readingThread = shared_ptr<Serial_Port_ReadingThread>(new Serial_Port_ReadingThread(10000, 200)); //beug ne marche pas 
																						

    cout<<"Welcome to pixhawk server"<<endl;
    cout<<"Init communicationn"<<endl;
    cout<<"-> Open  " <<device_path<<endl;
    if(drone1.init_communication()==0)
    { 
        cout<<"-> Succes"<<endl;
    }
    else
    {
        cout<<"-> Fail, exiting now"<<endl;
        sleep(2);
        exit(1);
    }
    
    cout<<"Pull parameters"<<endl;
    if(drone1.init_parameters(10000)==0)
    { 
        cout<<"-> Succes"<<endl;
    }
    else
    {
        cout<<"-> Fail, exiting"<<endl;
        sleep(2); 
        exit(1);
    }

    // Display
    //Display_IHM::getInstanceOf().displayDroneState(drone1);
    //Display_IHM::getInstanceOf().displayMotorsState(drone1);

    //Display_IHM::getInstanceOf().printLog("Begin");   
    
   readingThread.get()->start();
    writingThread.get()->start();
    sleep(1);
    
    //cout<<"Arm"<<endl;
    //drone1.command_arm(1);
    //sleep(1);

    //cout<<"test motors"<<endl;
    //drone1.command_directControl(0,1000,0,0);
    //sleep(1);

//    cout<<"Unarm"<<endl;    
//    drone1.get()->command_arm(0);
//    sleep(1);

//    readingThread.get()->stop();
//    writingThread.get()->stop();

   while(true)
   {
      command_loop();
   }
		MyFile.close();
		MyFilew.close();
		compassx.close();
		compassy.close();
    exit(0);
}
