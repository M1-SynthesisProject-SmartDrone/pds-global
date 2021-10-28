/**
 * Class called Bus that allow all class of system to communicate each other
 * 
 * @author Sylvain, Alexis
 * @date 19/11/19
 * @version 1.1 
 */

#ifndef BUS_H_
#define BUS_H_

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

#include <common/mavlink.h>

#include "../include/Com_WifiPort.h"
#include "../include/Com_SerialPort.h"
#include "../include/Thread_SerialPort.h"
#include "../include/DataListener_Keyboard.h"
#include "../include/Display_MainFrame.h"
#include "../include/Data_Bus.h"
#include "../include/Data_Drone.h"
#include "../include/Constants.h"

// Forward declaration for recursive include
class Engine;

/**
 * Singleton
 */
class Bus{

private:

    static Bus instanceOf;

    //%% Adress of all object
    Engine *main_engine;
    std::shared_ptr<MainFrame> mainFrame_1;
    std::shared_ptr<Serial_Port> serial_Port_1;
    std::shared_ptr<MavlinkTools> mavlink1;
    std::map<int, Drone> drones;

    Bus();
    ~Bus();

public: 

    /**
     * Method that initialize Bus adress
     */
    void initBus(Engine main_engine, std::shared_ptr<MainFrame> mainFrame_1, std::shared_ptr<Serial_Port> serial_Port_1, std::shared_ptr<MavlinkTools> mavlink1, std::map<int, Drone> drones);

    void event_newMessage(mavlink_message_t message);

    void update_threadState(std::string targetThread, int threadState);
    void update(mavlink_heartbeat_t heartbeat); //Heartbeat. gather Id and other
	void update(mavlink_command_ack_t ack); //Ack command: one of the most usefull parameters. it convey to read return of command
	void update(mavlink_sys_status_t sys_status);
    void update(mavlink_ping_t ping);
    void update(mavlink_param_value_t param_value);
    void update(mavlink_servo_output_raw_t servo_output_raw);
	void update(mavlink_battery_status_t battery_status);
    void update(mavlink_autopilot_version_t autopilot_version);
    void update(mavlink_estimator_status_t estimator_status);
    void update(mavlink_vibration_t vibration);
    void update(mavlink_extended_sys_state_t extended_sys_state);
	void update(mavlink_radio_status_t radio_status);
    void update(mavlink_highres_imu_t highres_imu); //Accelerometers of the system
	void update(mavlink_altitude_t altitude); //Altitude data
	void update(mavlink_attitude_t attitude); //Attitude of the drone use to manual control and target attitude
    void update(mavlink_attitude_target_t attitude_target); //Attitude of the drone use to manual control and target attitude
	void update(mavlink_global_position_int_t global_position_int);
    void update(mavlink_local_position_ned_t local_position_ned);
    void update(mavlink_position_target_local_ned_t position_target_local_ned);
    void update(mavlink_flight_information_t flight_information); // Flying time of the drone
    void update(mavlink_gps_status_t gps_status);   // Satellites visible
    void update(mavlink_gps_status_t* gps_status);   // If satellite is used
    void update(mavlink_global_vision_position_estimate_t global_vision_position_estimate); // Attitude of the drone
    void update(mavlink_attitude_t* attitude);    // Attitude of the drone
    void update(mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset);
    void update(mavlink_vfr_hud_t vfr_hud); // Heading of the drone
    void update(mavlink_gps_raw_int_t gps_raw_int); // Satellites visible
    void update(mavlink_manual_control_t manual_control); // X-axis, Y-axis and Z-axis

    uint64_t get_time_usec();

    static Bus& getInstanceOf(){
        return instanceOf;
    }   

    Engine* getEngine(){
        return main_engine;
    }

    std::shared_ptr<MainFrame> getMainFrame(){
        return mainFrame_1;
    }

    std::shared_ptr<Serial_Port> getSerialPort(){
        return serial_Port_1;
    }

    std::shared_ptr<MavlinkTools>  getMavlinkTool(){
        return mavlink1;
    }

    

};

#endif // BUS_H_


