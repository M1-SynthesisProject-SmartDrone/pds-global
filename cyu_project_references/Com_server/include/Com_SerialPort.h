/**
 * @brief Tool class which convey to open a serial port
 * @file serial_port.cpp
 * 
 * @author Sylvain Colomer
 * @date 19/04/19
 * @version 1.1 
 */

#ifndef COMM_SERIALPORT_H_
#define COMM_SERIALPORT_H_

// The following two non-standard baudrates should have been defined by the system
// If not, just fallback to number
#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// Status flags
#define SERIAL_PORT_OPEN   1
#define SERIAL_PORT_CLOSED 0
#define SERIAL_PORT_ERROR -1

#include <common/mavlink.h>

#include <cstdlib>
#include <stdio.h>  
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <string>

/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class Serial_Port
{

public:
	bool debug;
	std::string uart_name;
	int  baudrate;
	int  status;

public:

	Serial_Port();
	Serial_Port(std::string uart_name, int baudrate);
	~Serial_Port();

	void initialize_defaults();
	
	int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	int open_serial();
	void close_serial();

	void start();
	void stop();

	void handle_quit();

	void setUartName (std::string new_uart_name)
   {
		uart_name = new_uart_name;
	}

	void setBaudrate (int new_baudrate)
   {
		baudrate = new_baudrate;
	}

private:

	int  fd;
	mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	int  _open_port(const char* port);
	bool _setup_port(int baud);
	int  _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);
};



#endif // COMM_SERIALPORT_H_


