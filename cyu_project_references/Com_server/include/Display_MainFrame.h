/**
 * @brief Class use to display a real time SDL frame
 * 
 * @author Sylvain Colomer, Alexis Constant
 * @date 19/11/19
 * @version 1.1 
 */

#ifndef DISPLAY_THREADNCURSES_H_
#define DISPLAY_THREADNCURSES_H_

#include <iostream>
#include <iterator>
#include <algorithm>
#include <list>

#include <cstdlib>
#include <cstring>
#include <stdio.h>  
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <memory>
#include <chrono>
#include <ctime>  
#include <map>

#include <csignal>
#include <sys/signal.h>

#include <ncurses.h>
#include <panel.h>
#include <sys/ioctl.h>

#include <list>
#include <queue>
#include <utility>
	

//#include <loguru.hpp>

#include "Abstract_ThreadClass.h"
#include "Display_Panel.h"
#include "Constants.h"


/**
 * Drone classes
 */
class MainFrame : public Abstract_ThreadClass {

private:

	// NCURSES ATTRIBUTES
	winsize window_size;
	Ncurses_Panel *panel_header;
	Ncurses_Panel *panel_state;
	Ncurses_Panel *panel_thread;
	Ncurses_Panel *panel_comm;
	Ncurses_Panel *panel_drones;
	Ncurses_Panel *panel_details;
	Ncurses_Panel *panel_log;
	
	// INTERNAL DATA
	//log
	std::vector< std::pair<std::string, int> > log_list;
	//details : key, value, color
	std::map< std::string, std::pair<std::string, int> > details_list = {		// Displays parameters in alphabetic order
		{"0 - Id", std::make_pair("", 0)},
		{"1 - Heartbeat", std::make_pair("", 0)},
		{"2 - Satellite", std::make_pair("", 0)},	// Adding satellite use
		{"3 - Battery", std::make_pair("", 0)},		// Adding remaining battery level
		{"4 - Flight time", std::make_pair("", 0)},	// Adding flight time
		{"5 - Altitude", std::make_pair("", 0)},		// Adding altitude data
		{"6 - Heading", std::make_pair("", 0)},	// Adding compass
		{"7 - Pitch", std::make_pair("", 0)},	// Adding drone's pitch
		{"8 - Roll", std::make_pair("", 0)},	// Adding drone's roll
		{"9 - Yaw", std::make_pair("", 0)},	// Adding drone's yaw
		{"A - X-axis order", std::make_pair("", 0)},	// Adding control's X-axis
		{"B - Y-axis order", std::make_pair("", 0)},	// Adding control's Y-axis
		{"C - Z-axis order", std::make_pair("", 0)},	// Adding control's Z-axis
		{"D - Rotation order", std::make_pair("", 0)},	// Adding control's rotation
	};
	//comm
	int communication_type = 0;
	//state
	std::string state = "VOID";
	//thread
	std::map<std::string, int> thread_state = {
		{"Engine",STATE_THREAD_VOID},
		{"Keyboard",STATE_THREAD_VOID},
		{"Server_in",STATE_THREAD_VOID},
		{"Server_out",STATE_THREAD_VOID}
	};
	
public:
	
	
	MainFrame(int task_period, int task_deadline);

	~MainFrame();

	void run() override;

	void repaint();
	
	/**
	 * paint_all all element function
	 */
	void paint_all();

	void paint_comm();
	void paint_state();
	void paint_threads();

	void paint_log();
	void add_log(std::string text, int color);
	void refresh_log();

	void paint_details();
	void setDetail(std::string key, std::string value, int level);
	void refresh_details();

	void resize_all();
	void resize();

	/**
	 * repaint_all only changing element ?s
	 */
	void refresh_all();

	void clear_all();

	/**
	 * function that convey to push log on the queue log_lists
	 */
	void write_line(std::string text, WINDOW *window, int lines, int cols , int color);

	void erase_line(WINDOW *window, int lines);

	void quit_menu();

	int colornum(int fg, int bg);



	// GETTERS AND SETTERS
	void setState(std::string value){
		state = value;
	}
	int getCommunicationType();
	void setCommunicationType(int communication_type);
	std::map<std::string, int> getThreadState(){
		return thread_state;
	}
	void setThreadState(std::map<std::string, int> new_thread_state);
	void setElement_ThreadState(std::string key, int value);

};

#endif // SERIAL_PORT_H_


