
// global variables to access the drone data and use the different shared memories (blc)

extern Drone drone1;

extern blc_channel blc_control_arm; // 1=on, 0=off;
extern blc_channel blc_control_remote_vector;
    //Channels use to control manually the drone
extern blc_channel blc_control_motors; //x, y, z, r

    //Channels use to command the drone
extern blc_channel blc_control_commands; // 1=on, 0=off
extern blc_channel blc_highres_imu;

    //Different blc channel use to publish data
extern blc_channel blc_heartbeat; //Hearbeat message

extern blc_channel blc_attitude;
extern blc_channel blc_local_position_ned;
extern blc_channel blc_global_position;

	//only for test
extern blc_channel blc_test_detection;
