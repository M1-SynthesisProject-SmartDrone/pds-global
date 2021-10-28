# Root Cmake. Use to link the different libraries
#
# @author  sylvain Colomer, P. Gaussier
# @version 1.0


#########################################################################################################
###                                       GENERAL CONFIG                                               ##
#########################################################################################################

message("%%% PROJECT DRONE-FLY-MONITOR")
message("\n%% GENERAL CONFIGURATION")

# GENERAL SETTINGS
project(PixhawkProject CXX)
cmake_minimum_required(VERSION 3.2)
set(VERSION 1.0.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# OUTPUT DIR SETTINGS
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# COMPILER OPTIONS
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG   " -g3 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE " -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}") # Warning : option are important for some lib


########################################################################################################
###                                       LIBRARY CONFIG                                              ##
########################################################################################################

message("\n%% EXTERNALS libs configuration")

message("- Checking : pkg")
find_package(PkgConfig REQUIRED)
if(PkgConfig_FOUND)
    message("-> succes " )
endif()


message("- Checking : mavlink")
if(EXISTS ${CMAKE_SOURCE_DIR}/lib/mavlink)
    message("-> succes " )
else()
    message("Fail: You must install mavlink")
endif()
set(MAVLINK_LIB -I/lib/mavlink)
include_directories(./lib/mavlink)


#message("- Checking : Loguru")
#if(EXISTS ${CMAKE_SOURCE_DIR}/lib/loguru)
#    message("-> succes " )
#else()
#    message("Fail: You must add loguru log system in lib folder")
#endif()

#include_directories(./lib/loguru)

message("- Checking : Blc channels")
find_package(blc_channel REQUIRED)
find_package(blc_program REQUIRED)
include_directories( ${BL_INCLUDE_DIRS} )
add_definitions( ${BL_DEFINITIONS} )


message("- Checking : Thread")
find_package(Threads)

message("- Checking : Curses")
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


######################################################################################################
###                                            LINKS                                                ##
######################################################################################################

link_libraries(
    ${BL_LIBRARIES} 
    ${MAVLINK_LIB}
    ${CURSES_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    dl
    panel 
    ncurses
)



######################################################################################################
###                                         APPLICATION                                             ##
######################################################################################################

message("\n%% APPLICATION GENERATION")

# %% APP 1
add_executable(
    mavlink_server

    src/Display_MainFrame.cpp
    src/Display_Panel.cpp

    src/Com_SerialPort.cpp
    src/Com_WifiPort.cpp

    src/Thread_SerialPort.cpp
    src/Engine_mainLoop.cpp

    src/DataListener_Keyboard.cpp

    src/Com_Mavlink.cpp

    src/Data_Bus.cpp

    src/Abstract_ThreadClass.cpp
)

# Test trajectory launch
add_executable(TestDrone
    test/TestDroneRemoteControl.cpp 
    test/include/PixhawkServer.h
    test/src/Data_Drone.cpp 
    test/include/Data_Drone.h

    test/src/Com_SerialReadingThread.cpp 
    test/include/Com_SerialReadingThread.h
    test/src/Com_SerialWritingThread.cpp 
    test/include/Com_SerialWritingThread.h

    test/src/Com_Serial.cpp
    test/src/Abstract_ThreadClass.cpp
)





