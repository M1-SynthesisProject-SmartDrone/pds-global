/**
 * @brief Tool class which convey to open a serial port
 * @file serial_port.cpp
 * 
 * @author Sylvain Colomer, Raphael Bergoin, Clement Bout
 * @date 10/2018
 * @version 1.1 
 */

#include "../include/Com_WifiPort.h"

Wifi_Port::Wifi_Port()
{
	initialize_defaults(&sock, &locAddr, &targetAddr, local_port, target_ip);
}

Wifi_Port::~Wifi_Port()
{
    close(sock);
}

/**
 * @brief      	Init the socket to receive datagram and support UDP protocol
 *
 * @param		sock	the socket
 * @param		locAddr		the structure of the local socket
 * @param		targetAddr	the structure of the target socket
 * @param		local_port	the listening port
 * @param		target_ip	the IP adress of the target
 *			   
 * @return     -1 if there is a problem, 0 else
 */
int Wifi_Port::initialize_defaults(int *sock, struct sockaddr_in *locAddr, struct sockaddr_in *targetAddr, int local_port, char *target_ip)
{	
	struct sockaddr_in possibleTarget;						// The possible socket which will received our messages
  	socklen_t possibleTargetLen = sizeof(possibleTarget);
	uint8_t buf[BUFFER_LENGTH];
	
	int timeout = 10;							//Time in second for waiting an answer from the server
  	time_t currentTime;
  	time_t startTime = time(&currentTime);
  	double timeLeft = 0;
	
	*sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	locAddr->sin_family = AF_INET;
	locAddr->sin_addr.s_addr = INADDR_ANY;
	locAddr->sin_port = htons(local_port);
	memset (locAddr->sin_zero, 0, sizeof(locAddr->sin_zero));
	
	if (-1 == bind(*sock,(struct sockaddr *)locAddr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		close(*sock);
    	return -1;
	}
	
	/* Initialization listenning done */
	printf("INIT listenning :\nUDPin: 0.0.0.0:%d\n", ntohs(locAddr->sin_port));
	
	/* Attempt to make it non blocking */
	#if (defined __QNX__) | (defined __QNXNTO__)
	if (fcntl(*sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	#else
	if (fcntl(*sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
	#endif
	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(*sock);
    	return -1;
	}
	
  	/* While we don't find a packet which can indicate sending port we read all messages received on our port */
	while (recvfrom(*sock, buf, sizeof(buf), 0, (struct sockaddr*)(&possibleTarget), &possibleTargetLen)<=0 || possibleTarget.sin_addr.s_addr != inet_addr(target_ip)) 
	{
		memset(buf,0,256);
		time(&currentTime);
		timeLeft = difftime(currentTime, startTime);
		if (timeLeft > timeout)
		{
			perror("Connection time out");
			close(*sock);
			return -1;	
		}
	}

	/* Init the socket to send datagram and support UDP protocol */
	targetAddr->sin_family = AF_INET;
	targetAddr->sin_addr.s_addr = inet_addr(target_ip);
	targetAddr->sin_port = possibleTarget.sin_port;
	memset (targetAddr->sin_zero, 0, sizeof(targetAddr->sin_zero));

	/* Initialization sending done */
	printf("INIT target : UDPout : %s:%d\n",target_ip,ntohs(targetAddr->sin_port));

	return 0;
}

int Wifi_Port::read_message(mavlink_message_t *message)
{
    unsigned int temp = 0;

	ssize_t recsize;
	socklen_t fromlen;

	mavlink_status_t status;
	mavlink_channel_t chan = MAVLINK_COMM_0;

    memset(buf, 0, BUFFER_LENGTH);

    recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&targetAddr, &fromlen);	// reception

    /* Something received */
    if (recsize > 0)
    {
        printf("Bytes Received : %d\n", (int)recsize);	//Size
        /* For each part of the tram */
        for (int i = 0; i < recsize; ++i)
        {
            temp = buf[i];
            printf("%02x ", (unsigned char)temp); //Field of the tram in hexadecimal
            
            /* Parse the tram in order to get a mavlink message */
            if (mavlink_parse_char(chan, buf[i], message, &status))
            {
                /* Information about the packet received */
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n\n", message->sysid, message->compid, message->len, message->msgid);
				return 1;
            }
        }
    }

	return 0;
}

int Wifi_Port::write_message(const mavlink_message_t *message)
{
    len = mavlink_msg_to_send_buffer(buf, message);
    
    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&targetAddr, sizeof(struct sockaddr_in));
    if (bytes_sent==-1) 
    {
        //perror("Error Sending Heartbeat\n"); // If there is an error
		return 1;
    }
    memset(buf, 0, BUFFER_LENGTH);

	return 0;
}
