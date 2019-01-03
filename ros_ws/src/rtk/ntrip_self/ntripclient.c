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

	Program last modified by: Luke Armbruster
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
#include "include/ntrip_helper.h"

#include "serial.c"

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

#define MAXDATASIZE 1500 /* max number of bytes we can get at once */

/* CVS revision and version */
static char revisionstr[] = "$Revision: 1.51 $";
static char datestr[]     = "$Date: 2009/09/11 09:49:19 $";

enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

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
static void sighandler_int(int sig __attribute__((__unused__)))
#else /* __GNUC__ */
static void sighandler_alarm(int sig)
#endif /* __GNUC__ */
{
	sigstop = 1;
	alarm(2);
	stop = 1;
}

static const char *encodeurl(const char *req)
{
	std::string my_h = "0123456789abcdef";
	const char *h = my_h.c_str();
	static char buf[128];
	char *urlenc = buf;
	char *bufend = buf + sizeof(buf) - 3;

	while(*req && urlenc < bufend)
	{
		if(isalnum(*req)
				|| *req == '-' || *req == '_' || *req == '.')
			*urlenc++ = *req++;
		else
		{
			*urlenc++ = '%';
			*urlenc++ = h[*req >> 4];
			*urlenc++ = h[*req & 0x0f];
			req++;
		}
	}
	*urlenc = 0;
	return buf;
}


static int getargs(int argc, char **argv, struct Args *args)
{
	int res = 1;
	int getoptr;
	char *a;
	int i = 0;

	args->server = "www.euref-ip.net";
	args->port = "2101";
	args->user = "";
	args->password = "";
	args->nmea = 0;
	args->data = 0;
	args->bitrate = 0;
	args->proxyhost = 0;
	args->proxyport = "2101";
	args->mode = AUTO;
	args->initudp = 0;
	args->udpport = 0;
	args->protocol = SPAPROTOCOL_NONE;
	args->parity = SPAPARITY_NONE;
	args->stopbits = SPASTOPBITS_1;
	args->databits = SPADATABITS_8;
	args->baud = SPABAUD_57600;
	args->serdevice = 0;
	args->serlogfile = 0;
	args->nmea_usb = "/dev/ttyUSB0";
	args->rtcm_usb = "/dev/ttyUSB1";

	do
	{
#ifdef NO_LONG_OPTS
		switch((getoptr = getopt(argc, argv, ARGOPT)))
#else
			switch((getoptr = getopt_long(argc, argv, ARGOPT, opts, 0)))
#endif
			{
				case 's': args->server = optarg; break;
				case 'u': args->user = optarg; break;
				case 'p': args->password = optarg; break;
				case 'd': /* legacy option, may get removed in future */
						  fprintf(stderr, "Option -d or --data is deprecated. Use -m instead.\n");
				case 'm':
						  if(optarg && *optarg == '?')
							  args->data = encodeurl(optarg);
						  else
							  args->data = optarg;
						  break;
				case 'B':
						  {
							  int i = strtol(optarg, 0, 10);

							  switch(i)
							  {
								  case 50: args->baud = SPABAUD_50; break;
								  case 110: args->baud = SPABAUD_110; break;
								  case 300: args->baud = SPABAUD_300; break;
								  case 600: args->baud = SPABAUD_600; break;
								  case 1200: args->baud = SPABAUD_1200; break;
								  case 2400: args->baud = SPABAUD_2400; break;
								  case 4800: args->baud = SPABAUD_4800; break;
								  case 9600: args->baud = SPABAUD_9600; break;
								  case 19200: args->baud = SPABAUD_19200; break;
								  case 38400: args->baud = SPABAUD_38400; break;
								  case 57600: args->baud = SPABAUD_57600; break;
								  case 115200: args->baud = SPABAUD_115200; break;
								  default:
											   fprintf(stderr, "Baudrate '%s' unknown\n", optarg);
											   res = 0;
											   break;
							  }
						  }
						  break;
				case 'T':
						  if(!strcmp(optarg, "1")) args->stopbits = SPASTOPBITS_1;
						  else if(!strcmp(optarg, "2")) args->stopbits = SPASTOPBITS_2;
						  else
						  {
							  fprintf(stderr, "Stopbits '%s' unknown\n", optarg);
							  res = 0;
						  }
						  break;
				case 'A':
						  if(!strcmp(optarg, "5")) args->databits = SPADATABITS_5;
						  else if(!strcmp(optarg, "6")) args->databits = SPADATABITS_6;
						  else if(!strcmp(optarg, "7")) args->databits = SPADATABITS_7;
						  else if(!strcmp(optarg, "8")) args->databits = SPADATABITS_8;
						  else
						  {
							  fprintf(stderr, "Databits '%s' unknown\n", optarg);
							  res = 0;
						  }
						  break;
				case 'C':
						  {
							  int i = 0;
							  args->protocol = SerialGetProtocol(optarg, &i);
							  if(!i)
							  {
								  fprintf(stderr, "Protocol '%s' unknown\n", optarg);
								  res = 0;
							  }
						  }
						  break;
				case 'Y':
						  {
							  int i = 0;
							  args->parity = SerialGetParity(optarg, &i);
							  if(!i)
							  {
								  fprintf(stderr, "Parity '%s' unknown\n", optarg);
								  res = 0;
							  }
						  }
						  break;
				case 'D': args->serdevice = optarg; break;
				case 'l': args->serlogfile = optarg; break;
				case 'I': args->initudp = 1; break;
				case 'P': args->udpport = strtol(optarg, 0, 10); break;
				case 'n': args->nmea = optarg; break;
				case 'b': args->bitrate = 1; break;
				case 'r': args->port = optarg; break;
				case 'S': args->proxyhost = optarg; break;
				case 'R': args->proxyport = optarg; break;
				case 'M':
						  args->mode = 0;
						  if (!strcmp(optarg,"n") || !strcmp(optarg,"ntrip1"))
							  args->mode = NTRIP1;
						  else if(!strcmp(optarg,"h") || !strcmp(optarg,"http"))
							  args->mode = HTTP;
						  else if(!strcmp(optarg,"r") || !strcmp(optarg,"rtsp"))
							  args->mode = RTSP;
						  else if(!strcmp(optarg,"u") || !strcmp(optarg,"udp"))
							  args->mode = UDP;
						  else if(!strcmp(optarg,"a") || !strcmp(optarg,"auto"))
							  args->mode = AUTO;
						  else args->mode = atoi(optarg);
						  if((args->mode == 0) || (args->mode >= END))
						  {
							  fprintf(stderr, "Mode %s unknown\n", optarg);
							  res = 0;
						  }
						  break;
				case 'e': args->nmea_usb = optarg; break; // NMEA
				case 't': args->rtcm_usb = optarg; break; // RTCM
				case -1: break;
			}
	} while(getoptr != -1 && res);

	for(a = revisionstr+11; *a && *a != ' '; ++a)
		revisionstr[i++] = *a;
	revisionstr[i] = 0;
	datestr[0] = datestr[7];
	datestr[1] = datestr[8];
	datestr[2] = datestr[9];
	datestr[3] = datestr[10];
	datestr[5] = datestr[12];
	datestr[6] = datestr[13];
	datestr[8] = datestr[15];
	datestr[9] = datestr[16];
	datestr[4] = datestr[7] = '-';
	datestr[10] = 0;

	return res;
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

static int isReading = 1;
static int isWriting = 1;
static int btConnected = 1;

int main(int argc, char **argv)
{
	struct Args args;

	setbuf(stdout, 0);
	setbuf(stdin, 0);
	setbuf(stderr, 0);
#ifndef WINDOWSVERSION
	signal(SIGALRM,sighandler_alarm);
	signal(SIGINT,sighandler_int);
	alarm(ALARMTIME);
#else
	WSADATA wsaData;
	if(WSAStartup(MAKEWORD(1,1),&wsaData))
	{
		fprintf(stderr, "Could not init network access.\n");
		return 20;
	}
#endif

	//sleep(45); // Give the GPS time to warm up

	if(getargs(argc, argv, &args))
	{
		struct serial rtcm_out; // RTCM data out of Pi (write at 57600)
		struct serial nmea_in; // NMEA data into Pi (read at 115200)
		struct serial bt_serial; // Bluetooth connection (read/write at 115200)
		FILE *ser = 0;
		char nmeabuffer[200] = "$GPGGA,"; /* our start string */
		size_t nmeabufpos = 0;
		size_t nmeastarpos = 0;
		int sleeptime = 0;
		const char *rtk_rtcm_conn = SerialInit(&rtcm_out, args.rtcm_usb, args.baud,
				args.stopbits, args.protocol, args.parity, args.databits, 1);

		const char *rtk_nmea_conn = SerialInit(&nmea_in, args.nmea_usb, SPABAUD_115200,
				args.stopbits, args.protocol, args.parity, args.databits, 1);

		const char *bt_conn = SerialInit(&bt_serial, "/dev/serial0", SPABAUD_115200,
				args.stopbits, args.protocol, args.parity, args.databits, 1);
		std::thread bt_thread(btHandler, bt_conn, &bt_serial);
		//std::thread nmea_thread(nmeaHandler, rtk_nmea_conn, &nmea_in);
		if(rtk_rtcm_conn)
		{
			printf("Not writing data to RTK device\n");
			isWriting = 0;
		}
		if(rtk_nmea_conn) {
			printf("Not reading data from RTK device\n");
			isReading = 0;
		}
		if(bt_conn) {
			printf("Not connected to bluetooth device\n");
			btConnected = 0;
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
			int numbytes;
			char output_buf[MAXDATASIZE];
			char input_buf[MAXDATASIZE];

			server = args.server;
			printf("Server: %s\n", server);
			port = args.port;
			printf("Port: %s\n", port);
			memset(&their_addr, 0, sizeof(struct sockaddr_in));

			std::string gpgga_msg; // Contains GPGGA message received from GPS
			//
			if(isReading) { // Connected to GPS
				printf("Waiting for data...\n");
				while(1) { // Wait until a message is received from the GPS
					//gpgga_mu.lock();
					if(gpgga_msg != "") {
						//gpgga_mu.unlock();
						break;
					}
					//gpgga_mu.unlock();

					char read_buf[MAXDATASIZE];
					int readBytes = SerialRead(&nmea_in, read_buf, MAXDATASIZE);	
					//gpgga_mu.lock();
					if(readBytes) { // Read some data. Check to see if it contains the GPGGA message
						alarm(ALARMTIME); // Data has been received, reset the alarm
						gpgga_msg = getGPGGA(read_buf, readBytes);
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					if(gpgga_msg[20] == '0') { // No good yet (latitude and longitude are both 0)
						std::cout << "Data not valid yet" << std::endl;
						gpgga_msg = "";
					}
					//gpgga_mu.unlock();

				}
				gpgga_ready.store(true);
				std::cout << "Obtained valid first GPGGA message to send to caster" << std::endl;
			}
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
			if(!args.data) { // No specific stream supplied. Showing entire source table.
				out_i = snprintf(output_buf, MAXDATASIZE, "GET %s%s%s%s/ HTTP/1.1\r\n" "Host: %s\r\n%s" "User-Agent: %s/%s\r\n" "User-Agent: %s/%s\r\n" "Connection: close\r\n" "\r\n", "", "", args.server, "", AGENTSTRING, revisionstr); // Write this to the sized input buffer
			}	
			else { // Supplying a specific stream
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
						(*args.user || *args.password) ? "\r\nAuthorization: Basic " : "");
				printf("%s\n", output_buf);
				out_i += encode(output_buf + out_i, MAXDATASIZE - out_i - 4, args.user, args.password); // -4 is to save room for carriage returns and line feeds
				output_buf[out_i++] = '\r';
				output_buf[out_i++] = '\n';
				output_buf[out_i++] = '\r';
				output_buf[out_i++] = '\n';
			}
			//
			if(send(sockfd, output_buf, (size_t)out_i, 0) != out_i) {
				printf("Issue writing on socket\n");
			}
			else if(args.data) { // Specific stream
				printf("Getting data from NTRIP stream:\n");

				numbytes = recv(sockfd, input_buf, MAXDATASIZE-1, 0);

				if(numbytes > 17 && strstr(input_buf, "ICY 200 OK")) {
					printf("Proper stream connected. Caster expects data.\n");
					numbytes = 0;	
					//gpgga_mu.lock();
					if(send(sockfd, gpgga_msg.c_str(), gpgga_msg.size(), 0) != gpgga_msg.size()) {
						printf("Issue writing on socket");
						//gpgga_mu.unlock();
						break;
					}
					else {
						std::cout << gpgga_msg << std::endl;
						printf("GPGGA message written on socket\n");
					}
					//gpgga_mu.unlock();
				}
				else {
					printf("Could not connect to caster source table");
					break;
				}
				
				auto start = std::chrono::system_clock::now();
				char read_buf[MAXDATASIZE];
				int readBytes = 0;
				while(true) { // Read Caster data on socket and send GPGGA message every 1s
					auto cur_time = std::chrono::system_clock::now();
					std::chrono::duration<double> elapsed_time = cur_time - start;
					if(elapsed_time.count() >= 1.0f) { // 1 second passed, send caster most recent GPGGA message
						//gpgga_mu.lock();
						if(gpgga_msg[20] == '0') { // Lost signal 
							std::cout << "Lost signal" << std::endl;
							//gpgga_mu.unlock();
							break;
						}
						if(send(sockfd, gpgga_msg.c_str(), gpgga_msg.size(), 0) != gpgga_msg.size()) {
							printf("Issue writing on socket");
						}
						//gpgga_mu.unlock();
						start = cur_time;
					}
					numbytes = recv(sockfd, input_buf, MAXDATASIZE-1, 0);
					unsigned int i = 0;
					if(numbytes > 0) { // This should be the full blown RTCM3 message
						alarm(ALARMTIME); // Data has been received, reset the alarm
						SerialWrite(&rtcm_out, input_buf, numbytes);	
						read_buf[0] = 0;
						readBytes = SerialRead(&nmea_in, read_buf, MAXDATASIZE);	
						if(readBytes < MAXDATASIZE) {
							read_buf[readBytes] = 0;
						}
						read_buf[MAXDATASIZE - 1] = 0;
						if(readBytes > 100) {
							//gpgga_mu.lock();
							std::cout << read_buf << std::endl << std::endl;
							std::cout << readBytes << std::endl;
							std::string new_gpgga = getGPGGA(read_buf, readBytes);		
							SerialWrite(&bt_serial, gpgga_msg.c_str(), gpgga_msg.size());
							if(new_gpgga != "" && new_gpgga[20] != '0') {
								gpgga_msg = new_gpgga;
								gpgga_ready.store(true);
							}
							std::cout << gpgga_msg << std::endl;
							//gpgga_mu.unlock();
						}
						readBytes = 0;
						//std::this_thread::sleep_for(std::chrono::milliseconds(100));
					}
				}
			}
			printf("Done\n");
			//
			sleep(1);
		} while(1);
		bt_thread.join();
		//nmea_thread.join();
	}
	return 0;
}
