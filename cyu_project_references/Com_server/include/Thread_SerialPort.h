/**
 * @author Sylvain Colomer, Alexis
 * @date 19/11/19.
 */

#ifndef Thread_SerialPort_H
#define Thread_SerialPort_H

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

#include "blc_channel.h"
#include "blc_program.h"

#include "Abstract_ThreadClass.h"
#include "../include/Com_SerialPort.h"
#include "../include/Com_Mavlink.h"

#include "../include/Data_Bus.h"
#include "Data_Drone.h"


class Thread_SerialPort : public Abstract_ThreadClass 
{
private:
    // Loop buffer
    mavlink_message_t message;

	//Drone buffers corresponding to input var. Very usefull for GPS navigation 
	mavlink_heartbeat_t heartbeat; //Heartbeat. gather Id and others
	mavlink_command_ack_t ack; //Ack command: one of the most usefull parameters. it convey to read return of command
	mavlink_sys_status_t sys_status;
    mavlink_ping_t ping;
    mavlink_param_value_t param_value;
    mavlink_servo_output_raw_t servo_output_raw;
	mavlink_battery_status_t battery_status;
    mavlink_autopilot_version_t autopilot_version;
    mavlink_estimator_status_t estimator_status;
    mavlink_vibration_t vibration;
    mavlink_extended_sys_state_t extended_sys_state;
	mavlink_radio_status_t radio_status;
    mavlink_highres_imu_t highres_imu; //Accelerometers of the system
	mavlink_altitude_t altitude; //Altitude data
	mavlink_attitude_t attitude; //Attitude of the drone use to manual control and target attitude
    mavlink_attitude_target_t attitude_target; //Attitude of the drone use to manual control and target attitude
	mavlink_global_position_int_t global_position_int;
    mavlink_local_position_ned_t local_position_ned;
    mavlink_position_target_local_ned_t position_target_local_ned;
    mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset;
    //
    mavlink_gps_status_t gps_status;    // Satellite used
    mavlink_flight_information_t flight_information;    // Flight time
    mavlink_vfr_hud_t vfr_hud;  // Heading
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_manual_control_t manual_control;    // X-axis; Y-axis and Z-axis
    /*mavlink_sys_status_t battery_status;    // Remaining battery
    mavlink_global_vision_position_estimate_t global_vision_position_estimate;*/   // Attitude of the drone

protected: 
    
    /**
     * Target serial port 
     */
    std::shared_ptr<Serial_Port> serial_Port_1;

    /**
     * 
     */
    std::shared_ptr<MavlinkTools> mavlink1 = std::shared_ptr<MavlinkTools>(new MavlinkTools());

    /**
     * Mavlink message dictionnary
     */
    std::map<int, std::string> mavlink_dic;

public:
    
    
    Thread_SerialPort(int task_period, int task_deadline, std::shared_ptr<Serial_Port> serial_Port_1);

    ~Thread_SerialPort();

    void run();

    uint64_t get_time_usec();

    /**
     * Writing function of the serial port. Read Blc channels and execute associated command
     */
    void write_messages();
    
    /**
     * Reading function of mavlink message and diffusion by shared memory
     */
    void read_messages();

    int testAck(mavlink_command_ack_t *ack, std::string message);

    void handle_command_ack(mavlink_command_ack_t *ack);
};

#endif
