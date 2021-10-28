/**
 * @brief Class use to display a real time SDL frame
 * 
 * @author Sylvain Colomer
 * @date 19/04/19
 * @version 1.1 
 */

#ifndef DISPLAY_PANEL_H
#define DISPLAY_PANEL_H

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

#include "Constants.h"



class Ncurses_Panel
{

public : 
	WINDOW *window;
	WINDOW *panel;
	int x;
	int y;
	int lines;
	int cols;

	Ncurses_Panel(int x, int y, int lines, int cols);

	~Ncurses_Panel();

	void updateSize(int x, int y, int lines, int cols);
};


#endif //DISPLAY_PANEL_H


