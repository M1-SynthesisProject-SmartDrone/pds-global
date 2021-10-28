/**
 * @author Sylvain Colomer
 * @date 17/08/18.
 */

#ifndef DataListener_keyboard_H
#define DataListener_keyboard_H

#include <iostream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <mutex>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>

#include <ncurses.h>
#include <panel.h>
#include <sys/ioctl.h>

//#include <loguru.hpp>

#include "Abstract_ThreadClass.h"

class DataListener_Keyboard : public Abstract_ThreadClass {

private:
    int getch_value = 0;

public:
    
    DataListener_Keyboard(int task_period, int task_deadline);

    ~DataListener_Keyboard();

    void run();


};

#endif
