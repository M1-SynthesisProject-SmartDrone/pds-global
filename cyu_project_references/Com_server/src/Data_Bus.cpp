/**
 * @brief Class that model a drone
 * 
 * @author Sylvain Colomer, Alexis
 * @date 19/11/19
 * @version 1.0
 */

#include "../include/Data_Bus.h"
#include "../include/Engine_mainLoop.h"

#define PI 3.141592564

//Creation of the pbject
Bus Bus::instanceOf=Bus();


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTRUCTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Bus::Bus(){}
Bus::~Bus(){}

void Bus::initBus(Engine main_engine, std::shared_ptr<MainFrame> mainFrame_1, std::shared_ptr<Serial_Port> serial_Port_1, std::shared_ptr<MavlinkTools> mavlink1, std::map<int, Drone> drones)

{
    Bus::main_engine = &main_engine;
    Bus::mainFrame_1 = mainFrame_1;
    Bus::serial_Port_1 = serial_Port_1;
    Bus::mavlink1 = mavlink1;
    Bus::drones=drones;
}

void Bus::event_newMessage(mavlink_message_t message)
{
    int currentTime = get_time_usec();
    float freq = 0;

    freq = 1.0/((float)currentTime - (float)drones[message.sysid].last_timestamp);
    mainFrame_1.get()->setDetail("1 - Heartbeat", std::to_string(freq) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();

    //Refresh last time stamp
    drones[message.sysid].last_timestamp = currentTime;
}


void Bus::update(mavlink_heartbeat_t heartbeat)
{    
    // ID
    mainFrame_1.get()->setDetail("0 - Id", std::to_string(heartbeat.autopilot) , NCURSES_TEXT_DEFAULT); 

    mainFrame_1.get()->refresh_details();

    /*buffer1 = 
        "Hearthbeat:"+std::to_string(drone1.heartbeat.type)+
        " "+std::to_string(drone1.heartbeat.autopilot)+
        " "+std::to_string(drone1.heartbeat.base_mode)+
        " "+std::to_string(drone1.heartbeat.custom_mode)+
        " "+std::to_string(drone1.heartbeat.system_status)+
        " "+std::to_string(drone1.heartbeat.mavlink_version);*/
}

void Bus::update(mavlink_sys_status_t battery_status)
{      // Shows the remaining battery level
    if (battery_status.battery_remaining > 20)
    {
        mainFrame_1.get()->setDetail("3 - Battery", std::to_string(battery_status.battery_remaining)+"%" , NCURSES_TEXT_DEFAULT);
    }
    else if(battery_status.battery_remaining < 0)
    {
        mainFrame_1.get()->setDetail("3 - Battery", "Disconnected" , NCURSES_TEXT_RED);
        //mainFrame_1.get()->setDetail("Battery", std::to_string(battery_status.battery_remaining)+"%" , NCURSES_TEXT_RED);
    }
    else
    {
        mainFrame_1.get()->setDetail("3 - Battery", std::to_string(battery_status.battery_remaining)+"%" , NCURSES_TEXT_ORANGE);
    }
    mainFrame_1.get()->refresh_details();
}

/*void Bus::update(mavlink_flight_information_t flight_information){      // Shows the flight time
    if((flight_information.time_boot_ms*1000) >= 60){
        if((flight_information.time_boot_ms*1000)%60 < 10){
            mainFrame_1.get()->setDetail("4 - Flight time", std::to_string((flight_information.time_boot_ms)*1000*60)+":0"+std::to_string((flight_information.time_boot_ms)*1000%60) , NCURSES_TEXT_DEFAULT);
        }
        else{
            mainFrame_1.get()->setDetail("4 - Flight time", std::to_string((flight_information.time_boot_ms)*1000*60)+":"+std::to_string((flight_information.time_boot_ms)*1000%60) , NCURSES_TEXT_DEFAULT);
        }
    }
    else{
        mainFrame_1.get()->setDetail("4 - Flight time", std::to_string((flight_information.time_boot_ms)*1000)+"s" , NCURSES_TEXT_DEFAULT);
    }
    mainFrame_1.get()->refresh_details();
}*/

void Bus::update(mavlink_flight_information_t flight_information)
{      // Shows the flight id
    mainFrame_1.get()->setDetail("4 - Flight time", std::to_string(flight_information.flight_uuid) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}

void Bus::update(mavlink_altitude_t altitude)
{      // Shows altitude data
    if(altitude.altitude_relative == 0)
    {
        mainFrame_1.get()->setDetail("5 - Altitude", "Unknown" , NCURSES_TEXT_RED);
    }
    else
    {
        mainFrame_1.get()->setDetail("5 - Altitude", std::to_string(altitude.altitude_relative)+"m" , NCURSES_TEXT_DEFAULT);
    }
    mainFrame_1.get()->refresh_details();
}

void Bus::update(mavlink_gps_raw_int_t gps_raw_int)
{  // Shows if satellite is used
    if((gps_raw_int.satellites_visible) > 0)
    {
        mainFrame_1.get()->setDetail("2 - Satellite", std::to_string(gps_raw_int.satellites_visible)+" used" , NCURSES_TEXT_DEFAULT);
    }
    else
    {
        mainFrame_1.get()->setDetail("2 - Satellite", "Not used" , NCURSES_TEXT_RED);
    }
    mainFrame_1.get()->refresh_details();
}

void Bus::update(mavlink_attitude_t attitude)
{    // Shows attitude of the drone
    mainFrame_1.get()->setDetail("7 - Pitch", std::to_string(attitude.pitch*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("8 - Roll", std::to_string(attitude.roll*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("9 - Yaw", std::to_string(attitude.yaw*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}

void Bus::update(mavlink_vfr_hud_t vfr_hud)
{
    mainFrame_1.get()->setDetail("6 - Heading", std::to_string(vfr_hud.heading)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}

void Bus::update(mavlink_manual_control_t manual_control)
{
    mainFrame_1.get()->setDetail("A - X-axis order", std::to_string(manual_control.x) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("B - Y-axis order", std::to_string(manual_control.y) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("C - Z-axis order", std::to_string(manual_control.z) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("D - Rotation order", std::to_string(manual_control.r) , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}

/*void Bus::update(mavlink_highres_imu_t highres_imu){    // Shows attitude of the drone
    //mainFrame_1.get()->setDetail("7 - Pitch", std::to_string((attitude.pitch)*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->setDetail("7 - Pitch", std::to_string(highres_imu.xgyro)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    //mainFrame_1.get()->setDetail("8 - Roll", std::to_string((attitude.roll)*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->setDetail("8 - Roll", std::to_string((highres_imu.ygyro)*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    //mainFrame_1.get()->setDetail("9 - Yaw", std::to_string((attitude.yaw)*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->setDetail("9 - Yaw", std::to_string((highres_imu.zgyro)*180/PI)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}*/

/*void Bus::update(mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset){
    mainFrame_1.get()->setDetail("7 - Pitch", std::to_string(local_position_ned_system_global_offset.pitch)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("8 - Roll", std::to_string(local_position_ned_system_global_offset.roll)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
    mainFrame_1.get()->setDetail("9 - Yaw", std::to_string(local_position_ned_system_global_offset.yaw)+"°" , NCURSES_TEXT_DEFAULT);
    mainFrame_1.get()->refresh_details();
}*/

void Bus::update_threadState(std::string targetThread, int threadState)
{
    mainFrame_1.get()->setElement_ThreadState(targetThread, STATE_THREAD_WORK);
    mainFrame_1.get()->repaint();
}

uint64_t Bus::get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

