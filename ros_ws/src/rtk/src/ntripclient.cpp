/*
   NTRIP client for POSIX.
   $Id: ntripclient.c,v 1.51 2009/09/11 09:49:19 stoecker Exp $
   Copyright (C) 2003-2008 by Dirk Stöcker <soft@dstoecker.de>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   or read http://www.gnu.org/licenses/gpl.txt

 */

/**
 *
 *	@brief This contains the client code for running NTRIP corrections services with an RTK capable device.
 *			
 *	Currently supports Raspberry Pi 3b. This device uses two USB ports for NMEA read and RTCM write. It also
 *	uses on-board UART on Pi to read/write to bluetooth device for companion iOS app.
 *
 *	@author Luke Armbruster
 *
 *	@bug No known bugs.
 */



#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string>
#include <time.h>
#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "../include/NMEAData.h"
#include "rtk/HeadingSpeed.h"
#include "../include/serial.h"

typedef int sockettype;
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define closesocket(sock)       close(sock)
#define ALARMTIME   (2*60)
#define myperror perror

/* The string, which is send as agent in HTTP request */
#define AGENTSTRING "NTRIP NtripClientPOSIX"
#define TIME_RESOLUTION 125

#define MAXDATASIZE 3000 /* max number of bytes we can get at once */

/* CVS revision and version */
static char revisionstr[] = "$Revision: 1.51 $";
static char datestr[]     = "$Date: 2018/12/12 09:49:19 $";

enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

/* */
std::mutex input_buf_lock;
static char input_buf[MAXDATASIZE * 2];
static char buf_to_send[MAXDATASIZE * 2];
static void handleIPConnection(sockettype sockfd);
static int numbytes = 0;

struct Args
{
	const char *server;
	const char *port;
	const char *user;
	const char *proxyhost;
	const char *proxyport;
	const char *password;
	const char *nmea;
	const char *data;
	int         bitrate;
	int         mode;

	int         udpport;
	int         initudp;
	enum SerialBaud baud;
	enum SerialDatabits databits;
	enum SerialStopbits stopbits;
	enum SerialParity parity;
	enum SerialProtocol protocol;
	const char *serdevice;
	const char *serlogfile;
	const char *nmea_usb;
	const char *rtcm_usb;
};

/* option parsing */
#ifdef NO_LONG_OPTS
#define LONG_OPT(a)
#else
#define LONG_OPT(a) a
static struct option opts[] = {
	{ "bitrate",    no_argument,       0, 'b'},
	{ "data",       required_argument, 0, 'd'}, /* compatibility */
	{ "mountpoint", required_argument, 0, 'm'},
	{ "initudp",    no_argument,       0, 'I'},
	{ "udpport",    required_argument, 0, 'P'},
	{ "server",     required_argument, 0, 's'},
	{ "password",   required_argument, 0, 'p'},
	{ "port",       required_argument, 0, 'r'},
	{ "proxyport",  required_argument, 0, 'R'},
	{ "proxyhost",  required_argument, 0, 'S'},
	{ "user",       required_argument, 0, 'u'},
	{ "nmea",       required_argument, 0, 'n'},
	{ "mode",       required_argument, 0, 'M'},
	{ "serdevice",  required_argument, 0, 'D'},
	{ "baud",       required_argument, 0, 'B'},
	{ "stopbits",   required_argument, 0, 'T'},
	{ "protocol",   required_argument, 0, 'C'},
	{ "parity",     required_argument, 0, 'Y'},
	{ "databits",   required_argument, 0, 'A'},
	{ "serlogfile", required_argument, 0, 'l'},
	{0,0,0,0}};
#endif
#define ARGOPT "-d:m:bhp:r:s:u:n:S:R:M:IP:D:B:T:C:Y:A:l:t:e:"

int stop = 0;
int sigstop = 0;
#ifdef __GNUC__
	static __attribute__ ((noreturn)) void sighandler_alarm(
			int sig __attribute__((__unused__)))
#else /* __GNUC__ */
static void sighandler_alarm(int sig)
#endif /* __GNUC__ */
{
	if(!sigstop)
		fprintf(stderr, "ERROR: more than %d seconds no activity\n", ALARMTIME);
	else
		fprintf(stderr, "ERROR: user break\n");
	exit(1);
}

#ifdef __GNUC__
static void sighandler_int(int sig) 
{
	fprintf(stderr, "ERROR: user break\n");
	exit(1);

}
#else /* __GNUC__ */
static void sighandler_alarm(int sig)
{
	sigstop = 1;
	alarm(2);
	stop = 1;
}
#endif /* __GNUC__ */

static void setargs(struct Args *args)
{
	args->server = "108.59.49.226";
	args->port = "10000";
	args->user = "lukea1";
	args->password = "lukea1";
	args->nmea = 0;
	args->data = "RTCM3_MAX";
	args->bitrate = 0;
	args->proxyhost = 0;
	args->proxyport = "2101";
	args->mode = NTRIP1;
	args->initudp = 0;
	args->udpport = 0;
	args->protocol = SPAPROTOCOL_NONE;
	args->parity = SPAPARITY_NONE;
	args->stopbits = SPASTOPBITS_1;
	args->databits = SPADATABITS_8;
	args->baud = SPABAUD_57600;
	args->serdevice = 0;
	args->serlogfile = 0;
}

static const char encodingTable [64] = {
	'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
	'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
	'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
	'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
static int encode(char *buf, int size, const char *user, const char *pwd)
{
	unsigned char inbuf[3];
	char *out = buf;
	int i, sep = 0, fill = 0, bytes = 0;

	while(*user || *pwd)
	{
		i = 0;
		while(i < 3 && *user) inbuf[i++] = *(user++);
		if(i < 3 && !sep)    {inbuf[i++] = ':'; ++sep; }
		while(i < 3 && *pwd)  inbuf[i++] = *(pwd++);
		while(i < 3)         {inbuf[i++] = 0; ++fill; }
		if(out-buf < size-1)
			*(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
		if(out-buf < size-1)
			*(out++) = encodingTable[((inbuf [0] & 0x03) << 4)
				| ((inbuf [1] & 0xF0) >> 4)];
		if(out-buf < size-1)
		{
			if(fill == 2)
				*(out++) = '=';
			else
				*(out++) = encodingTable[((inbuf [1] & 0x0F) << 2)
					| ((inbuf [2] & 0xC0) >> 6)];
		}
		if(out-buf < size-1)
		{
			if(fill >= 1)
				*(out++) = '=';
			else
				*(out++) = encodingTable[inbuf [2] & 0x3F];
		}
		bytes += 4;
	}
	if(out-buf < size)
		*out = 0;
	return bytes;
}

static struct serial rtk_uart; // Serial connection (read/write at 57600)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtk_gps_node");
	ros::NodeHandle nh;
	ros::Publisher gpgga_pub = nh.advertise<sensor_msgs::NavSatFix>("rtk_gpgga", 1000);
	ros::Publisher gpvtg_pub = nh.advertise<rtk::HeadingSpeed>("rtk_gpvtg", 1000);

	while(ros::ok()) {
		struct Args args;

		setbuf(stdout, 0);
		setbuf(stdin, 0);
		setbuf(stderr, 0);
		signal(SIGALRM,sighandler_alarm);
		signal(SIGINT,sighandler_int);
		alarm(ALARMTIME);
		setargs(&args);

		FILE *ser = 0;
		char nmeabuffer[200] = "$GPGGA,"; // Start string
		size_t nmeabufpos = 0;
		size_t nmeastarpos = 0;
		int sleeptime = 0;
		const char *rtk_uart_conn = SerialInit(&rtk_uart, "/dev/gps", SPABAUD_57600,
				args.stopbits, args.protocol, args.parity, args.databits, 1);
		
		if(rtk_uart_conn)
		{
			printf("Not connected to RTK device\n");
		}
		do {
			sockettype sockfd = 0;
			struct sockaddr_in their_addr;
			struct hostent *he;
			const char* server;
			const char* port;
			char* b;
			long in_i;
			long out_i;
			char output_buf[MAXDATASIZE];

			char read_buf[MAXDATASIZE];
			int readBytes = 0;

			server = args.server;
			printf("Server: %s\n", server);
			port = args.port;
			printf("Port: %s\n", port);
			memset(&their_addr, 0, sizeof(struct sockaddr_in));
			printf("Waiting for data...\n");
			while(true) { // Wait until a message is received from the GPS
				NMEAData::gpgga_mu.lock();
				if(NMEAData::gpgga_msg != "") {
					std::cout << "**" << NMEAData::gpgga_msg << "**" << std::endl;
					NMEAData::gpgga_mu.unlock();
					break;
				}
				NMEAData::gpgga_mu.unlock();

				read_buf[0] = 0;
				readBytes = SerialRead(&rtk_uart, read_buf, MAXDATASIZE);	
				if(readBytes) { // Read some data. Check to see if it contains the GPGGA message
					alarm(ALARMTIME); // Data has been received, reset the alarm
					NMEAData::parseNMEA(read_buf, readBytes, gpgga_pub, gpvtg_pub);
				}
				NMEAData::gpgga_mu.lock();
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				if(NMEAData::gpgga_msg.size() < 36 || (NMEAData::gpgga_msg.substr(17, 4) == "0000" && NMEAData::gpgga_msg.substr(30, 5) == "00000")) { // No good yet (latitude and longitude are both 0)
					std::cout << NMEAData::gpgga_msg << std::endl;
					std::cout << "Data not valid yet" << std::endl;
					NMEAData::gpgga_msg = "";
					readBytes = 0;
				}
				NMEAData::gpgga_mu.unlock();

			}
			std::cout << "Obtained valid first GPGGA message to send to caster" << std::endl;
			//	
			if((out_i = strtol(port, &b, 10)) && (!b || !*b)) { 
				their_addr.sin_port = htons(out_i);
			}
			//
			if(!(he = gethostbyname(server))) {
				printf("Couldn't find server\n");	
			}
			else if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				printf("Socket error\n");
			}
			else {
				their_addr.sin_family = AF_INET;
				their_addr.sin_addr = *((struct in_addr *)he->h_addr);
			}
			//		
			if(connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
				printf("Could not connect on socket\n");
			}
			else {
				printf("Connected on socket\n");
			}
			//
			const char *nmeahead = (args.nmea && args.mode == HTTP) ? args.nmea : 0;
			printf("Arg data: %s\n", args.data);
			out_i=snprintf(output_buf, MAXDATASIZE-40, /* leave some space for login */
					"GET %s%s%s%s/%s HTTP/1.1\r\n"
					"Host: %s\r\n%s"
					"User-Agent: %s/%s\r\n"
					"%s%s%s"
					"Connection: close%s"
					, "", "",
					"", "",
					args.data, args.server,
					args.mode == NTRIP1 ? "" : "Ntrip-Version: Ntrip/2.0\r\n",
					AGENTSTRING, revisionstr,
					nmeahead ? "Ntrip-GGA: " : "", nmeahead ? nmeahead : "",
					nmeahead ? "\r\n" : "",
					//"Ntrip-GGA: ", "\r\n",
					(*args.user || *args.password) ? "\r\nAuthorization: Basic " : "");
			printf("%s\n", output_buf);
			out_i += encode(output_buf + out_i, MAXDATASIZE - out_i - 4, args.user, args.password); // -4 is to save room for carriage returns and line feeds
			output_buf[out_i++] = '\r';
			output_buf[out_i++] = '\n';
			output_buf[out_i++] = '\r';
			output_buf[out_i++] = '\n';
			//
			if(send(sockfd, output_buf, (size_t)out_i, 0) != out_i) {
				printf("Issue writing on socket\n");
			}
			else if(args.data) { // Specific stream
				printf("Getting data from NTRIP stream:\n");

				numbytes = recv(sockfd, input_buf, MAXDATASIZE-1, 0);

				if(numbytes > 17 && strstr(input_buf, "ICY 200 OK")) { // Caster responded properly
					printf("Proper stream connected. Caster expects data.\n");
					NMEAData::gpgga_mu.lock();
					if(send(sockfd, NMEAData::gpgga_msg.c_str(), NMEAData::gpgga_msg.size(), 0) != NMEAData::gpgga_msg.size()) {
						printf("Issue writing on socket");
						NMEAData::gpgga_mu.unlock();
						break;
					}
					else {
						std::cout << NMEAData::gpgga_msg << std::endl;
						printf("GPGGA message written on socket\n");
					}
					NMEAData::gpgga_mu.unlock();
				}
				else {
					printf("Could not connect to caster source table");
					break;
				}


				auto start = std::chrono::system_clock::now();
				auto cur_time = start;
				std::chrono::duration<double> elapsed_time = std::chrono::duration<double>(10.0f); // Initially send message to caster

				std::thread ntrip_thread(handleIPConnection, sockfd); // Launch thread to handle communication with NTRIP server
				numbytes = 0;

				while(true) { // Read Caster data on socket continuously. Send GPGGA message every 5s.
					if(elapsed_time.count() >= 5.0f) { // 5 second passed, send caster most recent GPGGA message
						NMEAData::gpgga_mu.lock();
						if(send(sockfd, NMEAData::gpgga_msg.c_str(), NMEAData::gpgga_msg.size(), 0) != NMEAData::gpgga_msg.size()) {
							printf("Issue writing on socket in main loop");
							break; // Reconnect to caster 
						}
						NMEAData::gpgga_mu.unlock();
						start = std::chrono::system_clock::now();
					}

					/*
					if(numbytes > 0) { // This should be the full blown RTCM3 message
						alarm(ALARMTIME); // Data has been received, reset the alarm
						int bytes_out = numbytes;
						memcpy(buf_to_send, input_buf, numbytes);
						numbytes = 0; // All buffer data was written
						SerialWrite(&rtk_uart, input_buf, bytes_out);	
					}
					*/
					memset(read_buf, '\0', sizeof(read_buf)); // Clear the buffer
                    input_buf_lock.lock();
					readBytes = SerialRead(&rtk_uart, read_buf, MAXDATASIZE); // Receive any data from the UART hardware buffer
                    input_buf_lock.unlock();

					NMEAData::parseNMEA(read_buf, readBytes, gpgga_pub, gpvtg_pub); // Check the buffer for the relevant data

					cur_time = std::chrono::system_clock::now();
					elapsed_time = cur_time - start;
					//std::cout << elapsed_time.count() << std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(900));
				}
				ntrip_thread.join();
			}
			printf("Done\n");
			//
			sleep(1);
		} while(true);
		return 0;
	}
}

static void handleIPConnection(sockettype sockfd) {
	std::chrono::duration<double> debug_time = std::chrono::duration<double>(0.0f);
	char ip_recv_buf[MAXDATASIZE];
	int ip_recv_bytes = 0;

	while(true) {
		ip_recv_bytes = recv(sockfd, ip_recv_buf, MAXDATASIZE-1, 0); // Receive any RTCM data from caster
		if(ip_recv_bytes + numbytes > MAXDATASIZE) {
			ROS_ERROR("OVERRAN NTRIP BUFFER");
		}
		input_buf_lock.lock();
		memcpy(input_buf + numbytes, ip_recv_buf, ip_recv_bytes);
		numbytes += ip_recv_bytes;
		if(numbytes > 0) { // This should be the full blown RTCM3 message
			alarm(ALARMTIME); // Data has been received, reset the alarm
			SerialWrite(&rtk_uart, ip_recv_buf, numbytes);	
			numbytes = 0; // All buffer data was written
		}
		input_buf_lock.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

}
