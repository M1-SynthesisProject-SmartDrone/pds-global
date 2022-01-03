/**
 *  @author Sylvain Colomer, Alexis Constant, P. Gaussier
 *  @date 19/11/19.
 *  trash-put /dev/shm/file
 */
 
#include <iostream>
#include <string>

using namespace std;

#include "../include/Thread_SerialPort.h"
#include "../include/global_variables.h"

//#define DISPLAY

//Channels use to control manually the drone
blc_channel  blc_control_motors("/pixhawk.control.motors", BLC_CHANNEL_READ, 'FL32', 'NDEF', 1, 4); //x, y, z, r
//Channels use to command the drone
blc_channel  blc_control_commands("/pixhawk.control.commands", BLC_CHANNEL_READ, 'FL32', 'NDEF', 1, 4); // 1=on, 0=off

blc_channel  blc_drone_data("/pixhawk.control.data", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 12); // 1=on, 0=off
    //Roll,Pitch,Yaw (3), xAcc, yAcc, zAcc (3), Bat, Heading, Heartbeat (3), Lat, Lon, Alt (3),  => 4x3=12 
    // battery_status.battery_remaining
    // altitude.altitude_relative
    // attitude.pitch*180/PI
    // attitude.roll*180/PI
    // attitude.yaw*180/PI
    // vfr_hud.heading
    // highres_imu.xgyro
    // highres_imu.ygyro
    // highres_imu.zgyro

    //Create or reopen an asynchrone channel "/channel_example" sending data of type int8/char in format undefined format of one dimension (vector) of SIZE.
/*    channel.create_or_open("/pixhawks.control.data", BLC_CHANNEL_WRITE, 'UIN8', 'TEXT',  1, SIZE);*/

Thread_SerialPort::Thread_SerialPort(int task_period, int task_deadline, std::shared_ptr<Serial_Port> serial_Port_1):
    Abstract_ThreadClass(task_period, task_deadline)
{
    Thread_SerialPort::serial_Port_1 = serial_Port_1;
    
    cout<<"init shared memory\n";
    blc_drone_data.reset(); //Clear the memory 

}

Thread_SerialPort::~Thread_SerialPort()
{

}

void Thread_SerialPort::run()
{
   cout<<"start Thread_SErialPort run()\n";
    long long currentThreadDelay;
    bool success=false;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    while(isRunFlag())
    {
        state = THREAD_STATE_WORK_SLEEP;
        usleep(task_period);

        gettimeofday(&end_checkpoint, 0);
        currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);

        if (currentThreadDelay > task_period )
        {
            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {
                state = THREAD_STATE_WORK_DEADLINE;
            }
            else 
            {
                state = THREAD_STATE_WORK_TASK;
              //  cout<<"R/W drone \n ";

                write_messages();
              //  cout<<"end W \n";

                //Task1: read message
                if(mavlink1.get()->read_message_serial(message, serial_Port_1))
                {
                    read_messages();
                }
              //  cout<<"end R/W \n";
            }
        }
    }
}

void Thread_SerialPort::write_messages()
{
    if(blc_control_commands.floats[0]>0.5) //arm command
    {
				//printf("mémoire partagée : demande armement \n");
				//blc_control_commands.floats[0] = 0.0;
  //      blc_control_commands.floats[0] = 0.0;
        serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_arm(1));
								
    }
    	
    if(blc_control_commands.floats[1]>0.5) // arm unarm command
    {
						//printf("mémoire partagée : demande desarmement \n");
						//blc_control_commands.floats[0] = 0.0;
    //    blc_control_commands.floats[1] = 0.0;
        serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_arm(1));
    }
			
	if(blc_control_motors.floats[0]>0.5) // right command
   { 
         //printf("mémoire partagée : demande motor 0 \n"); 
         //blc_control_commands.floats[1] = 0.0;S
         serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_right(1));
					
   }
			
   else // left command
   { 
         //printf("mémoire partagée : demande motor < 0,5 \n"); 
         //blc_control_commands.floats[1] = 0.0;S
         serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_left(1));
   }
			
//	while (blc_control_motors.floats[1]<0.5) //PB si on a un while ici PG !
//   {
			if(blc_control_motors.floats[1]>0.5) // up/down axis command
         { 
						//printf("mémoire partagée : demande motors 1 > 0,5 \n"); 
						//blc_control_commands.floats[1] = 0.0;S
               serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_up(1));
					//	action.set_takeoff_altitude(3.0);
			
         }
			else  // PG : ????
         { 
						//printf("mémoire partagée : demande motor 1 < 0,5 \n"); 
						//blc_control_commands.floats[1] = 0.0;S
               serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_down(1));
         }
					
//		}
	
			
	if(blc_control_motors.floats[3]>0.5) //pow command
   {
         //printf("mémoire partagée : demande d'augmentation de la puissance \n");
         //blc_control_commands.floats[0] = 0.0;
         serial_Port_1.get()->write_message(Bus::getInstanceOf().getMavlinkTool().get()->command_pow(1));
				
   }

    // COMMAND FOR CONTROL THE DRONE. THIS PART OF THE THREAD NEED TO BE CHANGE BEFORE ALL APPLICATION
    // if(1){
    //     drone1.command_directControl(blc_control_motors.floats[1],blc_control_motors.floats[0],blc_control_motors.floats[3],blc_control_motors.floats[2]);
    // }
}


void Thread_SerialPort::read_messages()
{
    string buffer1="";
    Bus::getInstanceOf().event_newMessage(message);
 //   cout<< "LOG "<<message.msgid<<"\n";

    switch (message.msgid)
    {
        //SENSORS HANG
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            Bus::getInstanceOf().update(heartbeat);
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_msg_sys_status_decode(&message, &sys_status);
            Bus::getInstanceOf().update(sys_status);
            
            buffer1 = "Error: "+std::to_string(sys_status.errors_count1)+" "+std::to_string(sys_status.errors_count2)+" "+std::to_string(sys_status.errors_count3)+" "+std::to_string(sys_status.errors_count4);
            //Display_IHM::getInstanceOf().printData(buffer1, 9, 1);
            buffer1 = "Voltage: "+std::to_string(sys_status.voltage_battery);
            //Display_IHM::getInstanceOf().printData(buffer1, 10, 1);*/
            break;
        }

        //ACCELEROMETERS
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            mavlink_msg_highres_imu_decode(&message, &highres_imu);
            //mavlink_msg_highres_imu_decode(&message, &drone1.highres_imu);
         //   Bus::getInstanceOf().update(highres_imu);

            buffer1 = "Acc: "+std::to_string(highres_imu.xacc)+" "+std::to_string(highres_imu.yacc)+" "+std::to_string(highres_imu.zacc);
            //Display_IHM::getInstanceOf().printData(buffer1, 11, 1);
            buffer1 = "Gyro: "+std::to_string(highres_imu.xgyro)+" "+std::to_string(highres_imu.ygyro)+" "+std::to_string(highres_imu.zgyro);
            //Display_IHM::getInstanceOf().printData(buffer1, 12, 1);
            buffer1 = "Mag: "+std::to_string(highres_imu.xmag)+" "+std::to_string(highres_imu.ymag)+" "+std::to_string(highres_imu.zmag);
            //Display_IHM::getInstanceOf().printData(buffer1, 13, 1);
            break;
        }

        //ACK COMMAND
        /*case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_msg_command_ack_decode(&message, &drone1.ack);
            handle_command_ack(&drone1.ack);
            break;
        }



        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            mavlink_msg_battery_status_decode(&message, &drone1.battery_status);
            break;
        }

        case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            mavlink_msg_radio_status_decode(&message, &drone1.radio_status);
            break;
        }*/



        //GPS
     /*   case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_msg_local_position_ned_decode(&message, &drone1.local_position_ned);

            buffer1 = "Pos_coord: "+std::to_string(drone1.local_position_ned.x)+" "+std::to_string(drone1.local_position_ned.y)+" "+std::to_string(drone1.local_position_ned.z);
            //Display_IHM::getInstanceOf().printData(buffer1, 14, 1);
            buffer1 = "Pos_speed: "+std::to_string(drone1.local_position_ned.vx)+" "+std::to_string(drone1.local_position_ned.vy)+" "+std::to_string(drone1.local_position_ned.vz);
            //Display_IHM::getInstanceOf().printData(buffer1, 15, 1);
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            mavlink_msg_position_target_local_ned_decode(&message, &drone1.position_target_local_ned);

            buffer1 = "Pos_coord_t: "+std::to_string(drone1.position_target_local_ned.x)+" "+std::to_string(drone1.position_target_local_ned.y)+" "+std::to_string(drone1.position_target_local_ned.z);
            // Display_IHM::getInstanceOf().printData(buffer1, 10, 1);
            buffer1 = "Pos_speed_t: "+std::to_string(drone1.position_target_local_ned.vx)+" "+std::to_string(drone1.position_target_local_ned.vy)+" "+std::to_string(drone1.position_target_local_ned.vz);
            // Display_IHM::getInstanceOf().printData(buffer1, 11, 1);
            break;
        }*/

        /*case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:{
            mavlink_msg_local_position_ned_system_global_offset_decode(&message, &local_position_ned_system_global_offset);
            Bus::getInstanceOf().update(local_position_ned_system_global_offset);
            break;
        }*/
        
        /*case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_msg_global_position_int_decode(&message, &global_position_int);
            Bus::getInstanceOf().update(global_position_int);
            // mavlink_msg_global_position_int_decode(&message, &drone1.global_position_int);
            // buffer1 = "GPS: "+std::to_string(drone1.global_position_int.lat)+" "+std::to_string(drone1.global_position_int.lon)+" "+std::to_string(drone1.global_position_int.alt);
            // Display_IHM::getInstanceOf().printData(buffer1, 12, 1);
            break;
        }*/

        /*case MAVLINK_MSG_ID_GPS_STATUS:{
            mavlink_msg_gps_status_decode(&message, &gps_status);
            Bus::getInstanceOf().update(gps_status);
            break;
        }*/

        /*
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_msg_gps_raw_int_decode(&message, &gps_raw_int);
            Bus::getInstanceOf().update(gps_raw_int);
            break;
        }*/

        //Attitude function
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            //mavlink_msg_attitude_decode(&message, &drone1.attitude);
            mavlink_msg_attitude_decode(&message, &attitude);
            Bus::getInstanceOf().update(attitude);
            
            buffer1 = "Attitude_t: "+std::to_string(attitude.time_boot_ms);
            // Display_IHM::getInstanceOf().printData(buffer1, 13, 1);
             buffer1 = "Attitude_p: "+std::to_string(attitude.roll)+" "+std::to_string(attitude.pitch)+" "+std::to_string(attitude.yaw);
            // Display_IHM::getInstanceOf().printData(buffer1, 14, 1);
             buffer1 = "Attitude_s: "+std::to_string(attitude.rollspeed)+" "+std::to_string(attitude.pitchspeed)+" "+std::to_string(attitude.yawspeed);
            // Display_IHM::getInstanceOf().printData(buffer1, 15, 1);
            
            break;
        }
        /*
        case MAVLINK_MSG_ID_VFR_HUD:{
            mavlink_msg_vfr_hud_decode(&message, &vfr_hud);
            Bus::getInstanceOf().update(vfr_hud);
            break;
        }
        
        case MAVLINK_MSG_ID_FLIGHT_INFORMATION:{
            mavlink_msg_flight_information_decode(&message, &flight_information);
            Bus::getInstanceOf().update(flight_information);
            break;
        }

        case MAVLINK_MSG_ID_MANUAL_CONTROL:{
            mavlink_msg_manual_control_decode(&message, &manual_control);
            Bus::getInstanceOf().update(manual_control);
            break;
        }
        */
        /*case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        {
            mavlink_msg_attitude_target_decode(&message, &drone1.attitude_target);

             buffer1 = "Attitude_target: "+std::to_string(drone1.attitude_target.time_boot_ms);
            //Display_IHM::getInstanceOf().printData(buffer1, 16, 1);
             buffer1 = "Attitude_target_q: "+std::to_string(drone1.attitude_target.q[0])+" "+std::to_string(drone1.attitude_target.q[1])+" "+std::to_string(drone1.attitude_target.q[2])+" "+std::to_string(drone1.attitude_target.q[3]);
            // Display_IHM::getInstanceOf().printData(buffer1, 17, 1);
             buffer1 = "Attitude_tar_r: "+std::to_string(drone1.attitude_target.body_roll_rate)+" "+std::to_string(drone1.attitude_target.body_pitch_rate)+" "+std::to_string(drone1.attitude_target.body_yaw_rate);
            // Display_IHM::getInstanceOf().printData(buffer1, 18, 1);
             buffer1 = "Attitude_tar_m: "+std::to_string(drone1.attitude_target.type_mask);
            // Display_IHM::getInstanceOf().printData(buffer1, 19, 1);
            break;
        }*/

        default:
        {
            printf("Warning, did not handle message id %i\n",message.msgid);
            break;
        }
    }
    cout<<"Thread_SerialPort "<<buffer1<<endl;

} 

int Thread_SerialPort::testAck(mavlink_command_ack_t *ack, std::string message)
{
    std::string buffer1="-> Ack-";
    buffer1+=message;

    if (ack->result == MAV_RESULT_ACCEPTED) 
    {
        buffer1+="-succes";
        //Display_Interface::Instance_of().print_Log(buffer1);
        return 0;
    } else 
    {
        buffer1+="-fail";
        //Display_Interface::Instance_of().print_Log(buffer1);
        return 1;
    }
}

void Thread_SerialPort::handle_command_ack(mavlink_command_ack_t *ack)
{
    switch (ack->command) 
    {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if(testAck(ack, "Arm_desarm") == 1) //if command is succeed
            {
                //if(drone1.motors == ARM){
                //    drone1.motors = UNARM;
                //}else if(drone1.motors == UNARM){
                //    drone1.motors = ARM;
                //}
            }
            //motors
            break;
        case MAV_CMD_NAV_GUIDED_ENABLE:
            testAck(ack, "navGuided");
            break;
        case MAVLINK_MSG_ID_PLAY_TUNE:
            testAck(ack, "Tone");
            break;
        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
            testAck(ack, "Attitude");
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            testAck(ack, "Manual control");
            break;
        default:
            testAck(ack, "Unknow");
            break;
    }    
}
