/* ========================================================================== *
*																			 *
*	lidar.c																  *
*																			 *
*	Description															  *
*	Driver for the Hokuyo URG-04LX-UG01. Special thanks to the Mobile		*
*	Robot Programming Toolkit, upon whose code this is based.				*
*  ========================================================================== */

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>

#define CMD "MS0044072501001\x0A"
#define HOST "127.0.0.1"
#define PORT 5560

typedef unsigned short int u16;

int encode(const char code[], int byte);
int setup_serial();
void run_scan(int port, char *scan, char *header);
char *buf;

int main(int argc, char *argv[]){
	int port, n, res, running;
	char *header = calloc(32, sizeof(char));
	char *scan;
	
	int sock;
	struct sockaddr_in *client = malloc(sizeof(struct sockaddr_in));
	
	//Allocate some space for a buffer
	buf = calloc(4096,sizeof(char));
	scan = buf;
	
	//Set up Serial Port
	port = setup_serial();
	printf("Serial Port Open\n");
	
	//Set up UDP Port
	if(argc>1){
		sock = setup_udp(client, argv[1], PORT);
		printf("UDP Port Open: %s\n", argv[1]);
	}
	else{
		sock = setup_udp(client, HOST, PORT);
		printf("UDP Port Open: %s\n", HOST);
	}
	
	//Main loop
	running = 1;
	int sent;
	printf("Starting LIDAR Scans\n");
	while(running){
		//Get scan
		run_scan(port, scan, header);
		//Send scan
		sent = sendto(sock, scan, strlen(scan), 0, (struct sockaddr*)client, sizeof(*client));
		if(sent<0)
			printf("Error Sending LIDAR Data\n");
		else
			printf("Sent %d bytes", sent);
	}
}

void run_scan(int port, char *scan, char *header){
	int run, res;
	
	buf = scan;
	if (write(port, CMD, 16) < 0){
		printf("Failed to write command (command = \"%s\", port = %d)!\n", CMD, port);
		perror("");
		exit(1);
	}
	else{
		run=1;
		//Get first packet (no scan)
		while(run){
			res = read(port, buf, 32);
			if(res == 0){
				continue;
			}
			buf[res]=0;
			printf(buf);
			//Check for end of packet
			if(strcmp(buf+19, "\n\n")==0){
				run=0;
			}
		}
		
		run = 1;
		//Get second packet (scan)
		//Get Header
		while(run){
			res = read(port, header, 26);
			run = ! res;
			buf[res]=0;
			printf(buf);
		}
		
		//Get Scan
		run = 1;
		while(run){
			res = read(port, buf, 1024);
			printf("%d bytes read",res);
			if(res == 0){
				continue;
			}
			buf += res;
			
			//Check for end of scan
			if(strcmp(buf-2, "\n\n")==0){
				run=0;
				buf[res] = 0;
			}
		}
		 printf("%s\n", scan);
	}
}

int encode(const char code[], int byte) {

  int value = 0;
  int i;

  for (i = 0; i < byte; ++i) {
	value <<= 6;
	value &= ~0x3f;
	value |= code[i] - 0x30;
  }
  return value;
}

int setup_serial(){
	int port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (port == -1){
		//Could not open port
		perror("Unabled to open /dev/ttyACM0\n");
		exit(1);
	}
	else
		fcntl(port, F_SETFL, 0);

	// Start assembling the new port settings.
	struct termios port_settings;
	memset( &port_settings,0x0,sizeof( port_settings ) ) ;

	// Enable the receiver (CREAD) and ignore modem control lines
	// (CLOCAL).
	port_settings.c_cflag |= CREAD | CLOCAL ;

	// Set the VMIN and VTIME parameters to zero by default. VMIN is
	// the minimum number of characters for non-canonical read and
	// VTIME is the timeout in deciseconds for non-canonical
	// read. Setting both of these parameters to zero implies that a
	// read will return immediately only giving the currently
	// available characters.
	port_settings.c_cc[ VMIN  ] = 0 ;
	port_settings.c_cc[ VTIME ] = 0 ;

	//Flush the input buffer associated with the port.
	if ( tcflush( port,TCIFLUSH ) < 0 ){
		perror("Cannot flush serial port\n");
		exit(1);
	}

	//Write the new settings to the port.
	if ( tcsetattr( port, TCSANOW, &port_settings ) < 0 ){
		perror("Cannot set the new config to the serial port\n") ;
		exit(1);
	}
	// Do NOT block on read.
	fcntl(port, F_SETFL, FNDELAY);
	return port;
}

int setup_udp(struct sockaddr_in *client, char *address, int port){
	int sock;
		
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
		perror("socket");
		exit(1);
	}

    memset(client, 0, sizeof(*client)); //Clear struct
    
	(*client).sin_family = AF_INET;
	(*client).sin_port = htons(PORT);
	(*client).sin_addr.s_addr = inet_addr(address);
	
	return sock;
}
