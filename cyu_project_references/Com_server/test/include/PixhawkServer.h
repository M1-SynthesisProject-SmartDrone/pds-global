/**
 *  @author Sylvain Colomer
 *  @date 18/04/19.
 */

#ifndef PIXHAWK_SERVER_H
#define PIXHAWK_SERVER_H

#include <common/mavlink.h>

#include <cstdlib>
#include <iostream>
#include <stdio.h>  
#include <unistd.h> 
#include <fcntl.h>  
#include <time.h>
#include <sys/time.h>

#include "blc_channel.h"
#include "blc_program.h"

#include "Data_Drone.h"

#include "Com_SerialReadingThread.h"
#include "Com_SerialWritingThread.h"
//#include "Display_IHM.h"



void command_loop();

#endif 
