/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 *  trash-put /dev/shm/file
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>	
#include <cmath>

#include <sys/time.h>



using namespace std;

#include "../include/Com_SerialReadingThread.h"
#include "../include/global_variables.h"

extern ofstream MyFile;
extern ofstream compassx;
extern ofstream compassy;



//#define DISPLAY

double t_old = -1.;

Serial_Port_ReadingThread::Serial_Port_ReadingThread(int task_period, int task_deadline):
    Abstract_ThreadClass(task_period, task_deadline)
{
    //Init object
   // Serial_Port_ReadingThread::drone1 = drone1;

    //Init dictionnary
    //mavlink_dic
    //ifstream file1("test.txt", ios::in); 
    //std::string buffer1;
    /*if(file1)
    {
        
        while(getline(file1, buffer1))
        {
            std::cout << line << std::endl;
        }
    }
    else{
        std::cout<< "Error : unable to open file" << std::endl;
        exit(1);
    }
    */

    //Creation of blc channels
    //Example: blc_heartbeat = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.heartbeat", BLC_CHANNEL_WRITE, 'IN16', 'NDEF', 1, 6));

    //blc_local_position_ned = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.local_position_ned", BLC_CHANNEL_WRITE, 'IN16', 'NDEF', 1, 6));
    //blc_global_position_int = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.global_position_int", BLC_CHANNEL_WRITE, 'IN16', 'NDEF', 1, 6));
    // blc_position_target_local_ned = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.position_target_local_ned", BLC_CHANNEL_WRITE, 'IN16', 'NDEF', 1, 6));
    // blc_position_target_global_int = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.position_target_global_int", BLC_CHANNEL_WRITE, 'IN16', 'NDEF', 1, 6));
  //  blc_highres_imu = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.highres_imu", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 9));
    // blc_attitude = std::shared_ptr<blc_channel>(new blc_channel("/pixhawk.sensors.attitude", BLC_CHANNEL_WRITE, 'FL32', 'NDEF', 1, 9));

}

Serial_Port_ReadingThread::~Serial_Port_ReadingThread()
{

}

void Serial_Port_ReadingThread::run()
{
    //long long currentThreadDelay;
    bool success=false;

    gettimeofday(&begin, 0);
    gettimeofday(&front_checkpoint, 0);

    currentState = LifeCoreState::RUN;

    while(isRunFlag())
    {
        usleep(task_period);

        gettimeofday(&end_checkpoint, 0);
      //  currentThreadDelay=(end_checkpoint.tv_sec-front_checkpoint.tv_sec) * 1000000L + (end_checkpoint.tv_usec-front_checkpoint.tv_usec);
        
 /*       if (currentThreadDelay > task_period )
        {
            cout<<"ReadingThread delay = "<<currentThreadDelay<<endl;

            gettimeofday(&front_checkpoint, 0);

            if (currentThreadDelay > task_period + task_deadline)
            {
                currentState = LifeCoreState::DEADLINE_EXCEEDED;
            }
            else 
            {
                currentState = LifeCoreState::RUN;
                mavlink_message_t message;
                success = drone1.read_message(message);
                if(success)
                {
                    read_messages(message);
                }
            }
        }*/
        
    //    cout<<"ReadingThread delay = "<<currentThreadDelay<<endl;

               currentState = LifeCoreState::RUN;
                mavlink_message_t message;
                success = drone1.read_message(message);
                if(success)
                {
//cout<<__FUNCTION__<<" read_message call \n";
                    read_messages(message);
                }
    }
}




/*MAVLINK_MSG_ID_HEARTBEAT 0
MAVLINK_MSG_ID_SYS_STATUS 1
MAVLINK_MSG_ID_SYSTEM_TIME 2
MAVLINK_MSG_ID_PING 4
MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL 5
MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK 6
MAVLINK_MSG_ID_AUTH_KEY 7
MAVLINK_MSG_ID_SET_MODE 11
MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
MAVLINK_MSG_ID_PARAM_REQUEST_LIST 21
MAVLINK_MSG_ID_PARAM_VALUE 22
MAVLINK_MSG_ID_PARAM_SET 23
MAVLINK_MSG_ID_GPS_RAW_INT 24
MAVLINK_MSG_ID_GPS_STATUS 25
MAVLINK_MSG_ID_SCALED_IMU 26
MAVLINK_MSG_ID_RAW_IMU 27
MAVLINK_MSG_ID_RAW_PRESSURE 28
MAVLINK_MSG_ID_SCALED_PRESSURE 29
MAVLINK_MSG_ID_ATTITUDE 30
MAVLINK_MSG_ID_ATTITUDE_QUATERNION 31
MAVLINK_MSG_ID_LOCAL_POSITION_NED 32
MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
MAVLINK_MSG_ID_RC_CHANNELS_SCALED 34
MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36
MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 37
MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST 38
MAVLINK_MSG_ID_MISSION_ITEM 39
MAVLINK_MSG_ID_MISSION_REQUEST 40
MAVLINK_MSG_ID_MISSION_SET_CURRENT 41
MAVLINK_MSG_ID_MISSION_CURRENT 42
MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43
MAVLINK_MSG_ID_MISSION_COUNT 44
MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45
MAVLINK_MSG_ID_MISSION_ITEM_REACHED 46
MAVLINK_MSG_ID_MISSION_ACK 47
MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN 48
MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN 49
MAVLINK_MSG_ID_PARAM_MAP_RC 50
MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA 54
MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA 55
MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV 61
MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62
MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV 63
MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV 64
MAVLINK_MSG_ID_RC_CHANNELS 65
MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
MAVLINK_MSG_ID_DATA_STREAM 67
MAVLINK_MSG_ID_MANUAL_CONTROL 69
MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70
MAVLINK_MSG_ID_MISSION_ITEM_INT 73
MAVLINK_MSG_ID_VFR_HUD 74
MAVLINK_MSG_ID_COMMAND_INT 75
MAVLINK_MSG_ID_COMMAND_LONG 76
MAVLINK_MSG_ID_COMMAND_ACK 77
MAVLINK_MSG_ID_MANUAL_SETPOINT 81
MAVLINK_MSG_ID_SET_ATTITUDE_TARGET 82
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84
MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED 85
MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT 86
MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT 87
MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET 89
MAVLINK_MSG_ID_HIL_STATE 90
MAVLINK_MSG_ID_HIL_CONTROLS 91
MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW 92
MAVLINK_MSG_ID_OPTICAL_FLOW 100
MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE 101
MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE 102
MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE 103
MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104
MAVLINK_MSG_ID_HIGHRES_IMU 105
MAVLINK_MSG_ID_OPTICAL_FLOW_RAD 106
MAVLINK_MSG_ID_HIL_SENSOR 107
MAVLINK_MSG_ID_SIM_STATE 108
MAVLINK_MSG_ID_RADIO_STATUS 109
MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL 110
MAVLINK_MSG_ID_TIMESYNC 111
MAVLINK_MSG_ID_CAMERA_TRIGGER 112
MAVLINK_MSG_ID_HIL_GPS 113
MAVLINK_MSG_ID_HIL_OPTICAL_FLOW 114
MAVLINK_MSG_ID_HIL_STATE_QUATERNION 115
MAVLINK_MSG_ID_SCALED_IMU2 116
MAVLINK_MSG_ID_LOG_REQUEST_LIST 117
MAVLINK_MSG_ID_LOG_ENTRY 118
MAVLINK_MSG_ID_LOG_REQUEST_DATA 119
MAVLINK_MSG_ID_LOG_DATA 120
MAVLINK_MSG_ID_LOG_ERASE 121
MAVLINK_MSG_ID_LOG_REQUEST_END 122
MAVLINK_MSG_ID_GPS_INJECT_DATA 123
MAVLINK_MSG_ID_GPS2_RAW 124
MAVLINK_MSG_ID_POWER_STATUS 125
MAVLINK_MSG_ID_SERIAL_CONTROL 126
MAVLINK_MSG_ID_GPS_RTK 127
MAVLINK_MSG_ID_GPS2_RTK 128
MAVLINK_MSG_ID_SCALED_IMU3 129
MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE 130
MAVLINK_MSG_ID_ENCAPSULATED_DATA 131
MAVLINK_MSG_ID_DISTANCE_SENSOR 132
MAVLINK_MSG_ID_TERRAIN_REQUEST 133
MAVLINK_MSG_ID_TERRAIN_DATA 134
MAVLINK_MSG_ID_TERRAIN_CHECK 135
MAVLINK_MSG_ID_TERRAIN_REPORT 136
MAVLINK_MSG_ID_SCALED_PRESSURE2 137
MAVLINK_MSG_ID_ATT_POS_MOCAP 138
MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET 139
MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET 140
MAVLINK_MSG_ID_ALTITUDE 141
MAVLINK_MSG_ID_RESOURCE_REQUEST 142
MAVLINK_MSG_ID_SCALED_PRESSURE3 143
MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE 146
MAVLINK_MSG_ID_BATTERY_STATUS 147
MAVLINK_MSG_ID_AUTOPILOT_VERSION 148
MAVLINK_MSG_ID_LANDING_TARGET 149
MAVLINK_MSG_ID_SENSOR_OFFSETS 150
MAVLINK_MSG_ID_SET_MAG_OFFSETS 151
MAVLINK_MSG_ID_MEMINFO 152
MAVLINK_MSG_ID_AP_ADC 153
MAVLINK_MSG_ID_DIGICAM_CONFIGURE 154
MAVLINK_MSG_ID_DIGICAM_CONTROL 155
MAVLINK_MSG_ID_MOUNT_CONFIGURE 156
MAVLINK_MSG_ID_MOUNT_CONTROL 157
MAVLINK_MSG_ID_MOUNT_STATUS 158
MAVLINK_MSG_ID_FENCE_POINT 160
MAVLINK_MSG_ID_FENCE_FETCH_POINT 161
MAVLINK_MSG_ID_FENCE_STATUS 162
MAVLINK_MSG_ID_AHRS 163
MAVLINK_MSG_ID_SIMSTATE 164
MAVLINK_MSG_ID_HWSTATUS 165
MAVLINK_MSG_ID_RADIO 166
MAVLINK_MSG_ID_LIMITS_STATUS 167
MAVLINK_MSG_ID_WIND 168
MAVLINK_MSG_ID_DATA16 169
MAVLINK_MSG_ID_DATA32 170
MAVLINK_MSG_ID_DATA64 171
MAVLINK_MSG_ID_DATA96 172
MAVLINK_MSG_ID_RANGEFINDER 173
MAVLINK_MSG_ID_AIRSPEED_AUTOCAL 174
MAVLINK_MSG_ID_RALLY_POINT 175
MAVLINK_MSG_ID_RALLY_FETCH_POINT 176
MAVLINK_MSG_ID_COMPASSMOT_STATUS 177
MAVLINK_MSG_ID_AHRS2 178
MAVLINK_MSG_ID_CAMERA_STATUS 179
MAVLINK_MSG_ID_CAMERA_FEEDBACK 180
MAVLINK_MSG_ID_BATTERY2 181
MAVLINK_MSG_ID_AHRS3 182
MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST 183
MAVLINK_MSG_ID_LED_CONTROL 186
MAVLINK_MSG_ID_MAG_CAL_PROGRESS 191
MAVLINK_MSG_ID_MAG_CAL_REPORT 192
MAVLINK_MSG_ID_EKF_STATUS_REPORT 193
MAVLINK_MSG_ID_PID_TUNING 194
MAVLINK_MSG_ID_GIMBAL_REPORT 200
MAVLINK_MSG_ID_GIMBAL_CONTROL 201
MAVLINK_MSG_ID_GIMBAL_RESET 202
MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS 203
MAVLINK_MSG_ID_GIMBAL_SET_HOME_OFFSETS 204
MAVLINK_MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT 205
MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS 206
MAVLINK_MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED 207
MAVLINK_MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG 208
MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS 209
MAVLINK_MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS 210
MAVLINK_MSG_ID_GOPRO_POWER_ON 215
MAVLINK_MSG_ID_GOPRO_POWER_OFF 216
MAVLINK_MSG_ID_GOPRO_COMMAND 217
MAVLINK_MSG_ID_GOPRO_RESPONSE 218
MAVLINK_MSG_ID_RPM 226
MAVLINK_MSG_ID_VIBRATION 241
MAVLINK_MSG_ID_HOME_POSITION 242
MAVLINK_MSG_ID_SET_HOME_POSITION 243
MAVLINK_MSG_ID_MESSAGE_INTERVAL 244
MAVLINK_MSG_ID_EXTENDED_SYS_STATE 245
MAVLINK_MSG_ID_ADSB_VEHICLE 246
MAVLINK_MSG_ID_V2_EXTENSION 248
MAVLINK_MSG_ID_MEMORY_VECT 249
MAVLINK_MSG_ID_DEBUG_VECT 250
MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251
MAVLINK_MSG_ID_NAMED_VALUE_INT 252
MAVLINK_MSG_ID_STATUSTEXT 253
MAVLINK_MSG_ID_DEBUG 254
MAVLINK_MSG_ID_EXTENDED_MESSAGE 255
*/

	static const struct { const char *name; uint32_t msgid; } mavlink_message_names[] = MAVLINK_MESSAGE_NAMES;

void Serial_Port_ReadingThread::read_messages(mavlink_message_t message)
{
	struct timeval t;
	double t_current, dt;
    std::string buffer1="";
    

    // Handle current timestamp
    drone1.timestamps = get_time_usec();
    buffer1 = "Time: "+std::to_string(drone1.timestamps);
    //Display_IHM::getInstanceOf().printData(buffer1, 7, 1);

    //Display_IHM::getInstanceOf().printLog(std::to_string(message.msgid));
    
   // cout<<"read_message "<<buffer1<<endl;
    // cout<<"  msgid = "<<message.msgid<<endl;
    for(unsigned int i=0;i<sizeof(mavlink_message_names);i++)
    {
      // cout<<"i = "<<i<<" "<<mavlink_message_names[i].msgid<<" "<<mavlink_message_names[i].name<<endl;
       if(mavlink_message_names[i].msgid==message.msgid)
       {
       //   cout<<" message name = "<<mavlink_message_names[i].name<<endl;
          break;
       }
    }
    
    switch (message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            //Display_IHM::getInstanceOf().printLog("heartbeat");
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&message, &drone1.heartbeat);
            buffer1 = 
                "Hearthbeat:"+std::to_string(drone1.heartbeat.type)+
                " "+std::to_string(drone1.heartbeat.autopilot)+
                " "+std::to_string(drone1.heartbeat.base_mode)+
                " "+std::to_string(drone1.heartbeat.custom_mode)+
                " "+std::to_string(drone1.heartbeat.system_status)+
                " "+std::to_string(drone1.heartbeat.mavlink_version);
   
            hb=drone1.heartbeat;
            printf("Heartbeat Message  (%d)  : ",message.msgid);
            printf("Type:  %d  ", hb.type);
            printf("Autopilot type: %d ", hb.autopilot);
            printf("System mode:  %d ", hb.base_mode);
 //           printf("Custom mode: %ald ", hb.custom_mode);
            printf("System status:  %d ", hb.system_status);
            printf("Mavlink version: %d \n", hb.mavlink_version);
                
            cout<<buffer1<<endl;     

            //Display_IHM::getInstanceOf().printData(buffer1, 8, 1);

            break;
        }

        //ACK COMMAND
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_msg_command_ack_decode(&message, &drone1.ack);
            handle_command_ack(&drone1.ack);
            
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_msg_sys_status_decode(&message, &drone1.sys_status);
            buffer1 = "Error: "+std::to_string(drone1.sys_status.errors_count1)+" "+std::to_string(drone1.sys_status.errors_count2)+" "+std::to_string(drone1.sys_status.errors_count3)+" "+std::to_string(drone1.sys_status.errors_count4);
            //Display_IHM::getInstanceOf().printData(buffer1, 9, 1);
            buffer1 = "Voltage: "+std::to_string(drone1.sys_status.voltage_battery);
            //Display_IHM::getInstanceOf().printData(buffer1, 10, 1);
      
             float volt = drone1.sys_status.voltage_battery / 1000.0f;	// mV
             float curr = drone1.sys_status.current_battery / 100.0f;	// 10 mA or -1
            float rem = drone1.sys_status.battery_remaining / 100.0f;	// or -1
                  
            cout<<buffer1<<endl;
            printf("volt = %f, curr = %f , remaining = %f \n",volt,curr,rem);
            
            break;
        }

        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            mavlink_msg_battery_status_decode(&message, &drone1.battery_status);
            printf("MAVLINK_MSG_ID_BATTERY_STATUS   \n");
            break;
        }

        case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            mavlink_msg_radio_status_decode(&message, &drone1.radio_status);
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD 	: //74
        {
            printf("MAVLINK_MSG_ID_VFR_HUD 	???\n");
            break;
        }
        //ACCELEROMETERS
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            mavlink_msg_highres_imu_decode(&message, &drone1.highres_imu);

            buffer1 = buffer1+" Acc: "+std::to_string(drone1.highres_imu.xacc)+" "+std::to_string(drone1.highres_imu.yacc)+" "+std::to_string(drone1.highres_imu.zacc)+"\n";
            //Display_IHM::getInstanceOf().printData(buffer1, 11, 1);
            buffer1 = buffer1+" Gyro: "+std::to_string(drone1.highres_imu.xgyro)+" "+std::to_string(drone1.highres_imu.ygyro)+" "+std::to_string(drone1.highres_imu.zgyro)+"\n";
            //Display_IHM::getInstanceOf().printData(buffer1, 12, 1);
            buffer1 = buffer1+" Mag: "+std::to_string(drone1.highres_imu.xmag)+" "+std::to_string(drone1.highres_imu.ymag)+" "+std::to_string(drone1.highres_imu.zmag)+"\n";
            //Display_IHM::getInstanceOf().printData(buffer1, 13, 1);

            blc_highres_imu.floats[0] = drone1.highres_imu.xacc;
            blc_highres_imu.floats[1] = drone1.highres_imu.yacc;
            blc_highres_imu.floats[2] = drone1.highres_imu.zacc;
            blc_highres_imu.floats[3] = drone1.highres_imu.xgyro;
            blc_highres_imu.floats[4] = drone1.highres_imu.ygyro;
            blc_highres_imu.floats[5] = drone1.highres_imu.zgyro;
            blc_highres_imu.floats[6] = drone1.highres_imu.xmag;
            blc_highres_imu.floats[7] = drone1.highres_imu.ymag;
            blc_highres_imu.floats[8] = drone1.highres_imu.zmag;
            
            double xmag = drone1.highres_imu.xmag;
            double ymag = drone1.highres_imu.ymag;
            double angle;
            if ( xmag>=-0.7 && xmag<=0.7 ) {
				angle = acos(xmag);
				cout << "acos(xmag) = " << angle << " radians" << endl;
				cout << "acos(x) = " << angle*180/3.1415 << " degrees" << endl;
			}
			else if ( ymag>=-0.7 && ymag<=0.7 )
			{	
				angle = acos(ymag);
				cout << "acos(ymag) = " << angle << " radians" << endl;
				cout << "acos(x) = " << angle*180/3.1415 << " degrees" << endl;
			}
				
			
            cout << "xmag : " << drone1.highres_imu.xmag << "ymag :" << drone1.highres_imu.ymag << "zmag :" << drone1.highres_imu.zmag << endl;
            compassx << drone1.highres_imu.xmag << endl;
			compassy << drone1.highres_imu.ymag << endl;
			cout << "buffer 1  mag" << buffer1 << endl;

            cout<<"Com_SerialReadingThread "<<buffer1<<endl;
            break;
        }
        
  /*    case MAVLINK_MSG_ID_COMPASSMOT_STATUS:
        
			mavlink_compassmot_status_decode(&message, &drone1.compassmot_status);

            buffer1 = buffer1+" CompensationX: "+std::to_string(drone1.compassmot_status.CompensationX)+" CompensationY:"+std::to_string(drone1.compassmot_status.CompensationY)+" CompensationZ :"+std::to_string(drone1.compassmot_status.CompensationZ)+"\n";
          
            blc_compassmot_status.floats[0] = drone1.compassmot_status.CompensationX;
            blc_compassmot_status.floats[1] = drone1.compassmot_status.CompensationY;
            blc_compassmot_status.floats[2] = drone1.compassmot_status.CompensationZ;
      
       //     cout<<"Com_SerialReadingThread "<<buffer1<<endl;
            break;
		*/
		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
		//	cout << "MAVLINK_MSG_ID_GPS_RAW_INT:" << endl;
			mavlink_msg_gps_raw_int_decode(&message, &drone1.gps_raw_int);
	//		current_messages.time_stamps.gps_raw_int = get_time_usec();
	//		this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;
	/*		cout <<"\tlat: " << drone1.gps_raw_int.lat <<"\tlon: " << drone1.gps_raw_int.lon <<"\talt: " << drone1.gps_raw_int.alt <<endl;
			cout <<"\teph: " << drone1.gps_raw_int.eph <<"\tepv: " << drone1.gps_raw_int.epv <<endl;
			cout <<"\tvel: " << drone1.gps_raw_int.vel <<"\tcog: " << drone1.gps_raw_int.cog <<endl;
			cout <<"\tfix: " << (int)drone1.gps_raw_int.fix_type <<"\tsat: " << (int)drone1.gps_raw_int.satellites_visible <<endl;*/
			
			if( drone1.motors == UNARM) { drone1.departure_alt = drone1.gps_raw_int.alt; cout<<"un_armed alt set to "<<drone1.departure_alt<<endl; }
			break;
		}

        //GPS
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_msg_local_position_ned_decode(&message, &drone1.local_position_ned);

            buffer1 =  buffer1 + "\n Pos_coord: "+std::to_string(drone1.local_position_ned.x)+" "+std::to_string(drone1.local_position_ned.y)+" "+std::to_string(drone1.local_position_ned.z)+"\n";
            //Display_IHM::getInstanceOf().printData(buffer1, 14, 1);
            buffer1 = "Pos_speed: "+std::to_string(drone1.local_position_ned.vx)+" "+std::to_string(drone1.local_position_ned.vy)+" "+std::to_string(drone1.local_position_ned.vz)+"\n";
            //Display_IHM::getInstanceOf().printData(buffer1, 15, 1);
       //      cout<<buffer1<<endl;

            
            blc_local_position_ned.floats[0] = drone1.local_position_ned.x;
            blc_local_position_ned.floats[1] = drone1.local_position_ned.y;
            blc_local_position_ned.floats[2] = drone1.local_position_ned.z;
            blc_local_position_ned.floats[3] = drone1.local_position_ned.vx;
            blc_local_position_ned.floats[4] = drone1.local_position_ned.vy;
            blc_local_position_ned.floats[5] = drone1.local_position_ned.vz;
            
            
            break;
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            mavlink_msg_position_target_local_ned_decode(&message, &drone1.position_target_local_ned);

            buffer1 = "Pos_coord_t: "+std::to_string(drone1.position_target_local_ned.x)+" "+std::to_string(drone1.position_target_local_ned.y)+" "+std::to_string(drone1.position_target_local_ned.z);
         //   cout<<buffer1<<endl;

         /*   Display_IHM::getInstanceOf().printData(buffer1, 10, 1);
            buffer1 = "Pos_speed_t: "+std::to_string(drone1.position_target_local_ned.vx)+" "+std::to_string(drone1.position_target_local_ned.vy)+" "+std::to_string(drone1.position_target_local_ned.vz);
            Display_IHM::getInstanceOf().printData(buffer1, 11, 1);*/
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_msg_global_position_int_decode(&message, &drone1.global_position_int);
            buffer1 = "GPS: "+std::to_string(drone1.global_position_int.lat)+" "+std::to_string(drone1.global_position_int.lon)+" "+std::to_string(drone1.global_position_int.alt);
      //     cout<<buffer1<<endl;

            //Display_IHM::getInstanceOf().printData(buffer1, 12, 1);*/
            
            blc_global_position.floats[0] = drone1.global_position_int.lat;
            blc_global_position.floats[1] = drone1.global_position_int.lon;
            blc_global_position.floats[2] = drone1.global_position_int.alt;
            cout<<buffer1<<endl;
            
            break;
        }

        //Attitude function
        
        case MAVLINK_MSG_ID_ATTITUDE:
        {
			gettimeofday(&t, NULL);
			 t_current = t.tv_sec+t.tv_usec*1e-6; 
			 
	//		cout<<"t_old = "<<t_old<<endl;
		
			if(t_old>0.)
			{
				dt=t_current-t_old;
		//		cout<<"\n \n >>>>  period  msg_id_attitude "<<dt<<"\n\n\n";
			}
			t_old=t_current;
		//	cout<<"t_old = "<<t_old<<endl;
			
		   MyFile << dt << endl;

            mavlink_msg_attitude_decode(&message, &drone1.attitude);
            
            buffer1 = buffer1 + "\n Attitude_time: "+to_string(drone1.attitude.time_boot_ms)+"\n";
           // Display_IHM::getInstanceOf().printData(buffer1, 13, 1);
            buffer1 = buffer1 + " Attitude_p (roll, pitch, yaw): "+to_string(drone1.attitude.roll)+" "+to_string(drone1.attitude.pitch)+" "+std::to_string(drone1.attitude.yaw)+"\n";
           // Display_IHM::getInstanceOf().printData(buffer1, 14, 1);
            buffer1 = buffer1 + " Attitude_s: "+to_string(drone1.attitude.rollspeed)+" "+to_string(drone1.attitude.pitchspeed)+" "+to_string(drone1.attitude.yawspeed)+"\n";
          //  Display_IHM::getInstanceOf().printData(buffer1, 15, 1);*/
       //    cout<<"MAVLINK_MSG_ID_ATTITUDE     : \n"<<buffer1<<endl;
           
           blc_attitude.floats[0] = drone1.attitude.time_boot_ms;
           blc_attitude.floats[1] = drone1.attitude.roll;
           blc_attitude.floats[2] = drone1.attitude.pitch;
           blc_attitude.floats[3] = drone1.attitude.yaw;
           blc_attitude.floats[4] = drone1.attitude.rollspeed;
           blc_attitude.floats[5] = drone1.attitude.pitchspeed;
           blc_attitude.floats[6] = drone1.attitude.yawspeed;
           
           /*blc_attitude.floats[0] = 5.24;
           blc_attitude.floats[1] = 4.65;
           blc_attitude.floats[2] = 0.45;
           blc_attitude.floats[3] = 8.15;
           blc_attitude.floats[4] = 2.59;
           blc_attitude.floats[5] = 1.57;
           blc_attitude.floats[6] = 1.20;*/
                    
          break;
        }
	
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
        {
            mavlink_msg_attitude_target_decode(&message, &drone1.attitude_target);

            /*buffer1 = "Attitude_target: "+std::to_string(drone1.attitude_target.time_boot_ms);
            Display_IHM::getInstanceOf().printData(buffer1, 16, 1);
            buffer1 = "Attitude_target_q: "+std::to_string(drone1.attitude_target.q[0])+" "+std::to_string(drone1.attitude_target.q[1])+" "+std::to_string(drone1.attitude_target.q[2])+" "+std::to_string(drone1.attitude_target.q[3]);
            Display_IHM::getInstanceOf().printData(buffer1, 17, 1);
            buffer1 = "Attitude_tar_r: "+std::to_string(drone1.attitude_target.body_roll_rate)+" "+std::to_string(drone1.attitude_target.body_pitch_rate)+" "+std::to_string(drone1.attitude_target.body_yaw_rate);
            Display_IHM::getInstanceOf().printData(buffer1, 18, 1);
            buffer1 = "Attitude_tar_m: "+std::to_string(drone1.attitude_target.type_mask);
            Display_IHM::getInstanceOf().printData(buffer1, 19, 1);*/
            
            break;
        }
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
           printf("MAVLINK_MSG_ID_SERVO_OUTPUT_RAW ???\n");
            break;
        }
       case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        {
           printf("MAVLINK_MSG_ID_EXTENDED_SYS_STATE \n");
           mavlink_msg_extended_sys_state_decode(&message, &drone1.extended_sys_state);
    
           printf("vtol_state %d,  landed_state %d \n", drone1.extended_sys_state.vtol_state,drone1.extended_sys_state.landed_state);
            break;
        }
        case MAVLINK_MSG_ID_ALTITUDE:
        {
			printf("MAVLINK_MSG_ID_ALTITUDE  \n");
			mavlink_altitude_t* altitude;
			mavlink_msg_altitude_decode(&message, &drone1.altitude);
			altitude= &drone1.altitude;

			printf("altitude_local  %f altitude_relative %f altitude_terrain %f  bottom_clearance %f\n",altitude->altitude_local , altitude->altitude_relative, altitude->altitude_terrain,altitude->bottom_clearance );    
			break;
        }
  /*      case MAVLINK_MSG_ID_HOME_POSITION:
        {
			printf("MAVLINK_MSG_ID_HOME_POSITION  \n");
			//mavlink_home_position_t* home_position;
			mavlink_msg_home_position_decode(&message, &drone1.home_position);
			//home_position= &drone1.home_position;
			printf("latitude %d, longitude %d, altitude %d \n", drone1.home_position.latitude, drone1.home_position.longitude, drone1.home_position.altitude);
			break;
        }                */
       case MAVLINK_MSG_ID_VIBRATION:
        {
      //     printf("MAVLINK_MSG_ID_VIBRATION ???\n");
            break;
        }   
       case MAVLINK_MSG_ID_PING:
        {
           printf("MAVLINK_MSG_ID_PING ???\n");
            break;
        }   
        
 	   case MAVLINK_MSG_ID_COMMAND_LONG:
		{
			 printf("MAVLINK_MSG_ID_COMMAND_LONG \n"); 
              mavlink_msg_command_long_decode(&message, &drone1.command_long);

			printf(" %f %f %f %f %f %f %f \n", drone1.command_long.param1, drone1.command_long.param2 ,  drone1.command_long.param3 , drone1.command_long.param4 , drone1.command_long.param5 , drone1.command_long.param6 ,  drone1.command_long.param7 );
			// command_long->command = mavlink_msg_command_long_get_command(msg);
			printf("target_system= %d , target_component= %d , confirm = %d \n", drone1.command_long.target_system ,  drone1.command_long.target_component , drone1.command_long.confirmation);
			break;
		}
        default:
        {
            printf("=======================\n\n      Warning, did not handle message id %i\n======================\n\n",message.msgid);
            break;
        }


    }
 //  cout<<"End function : "<<buffer1<<endl;
} 


uint64_t Serial_Port_ReadingThread::get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

int Serial_Port_ReadingThread::testAck(mavlink_command_ack_t *ack, std::string message)
{
    string buffer1="-> Ack- ";
    buffer1+=message;

    if (ack->result == MAV_RESULT_ACCEPTED) {
        buffer1+="-succes";
        //Display_IHM::getInstanceOf().printLog(buffer1);
        return 0;
    } else {
        buffer1+="-fail";
        //Display_IHM::getInstanceOf().printLog(buffer1);
        return 1;
    }
    cout<<"testAck "<<buffer1<<endl;
}

void Serial_Port_ReadingThread::handle_command_ack(mavlink_command_ack_t *ack)
{
    switch (ack->command) 
    {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            if(testAck(ack, "Arm_desarm") == 1)
            { //if command is succeed
                if(drone1.motors == ARM){
                    drone1.motors = UNARM;
                } else if(drone1.motors == UNARM){
                    drone1.motors = ARM;
                }
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
    ////Display_IHM::getInstanceOf().displayDroneState(drone1);
    
}

