#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>

#include "/home/ed/ardrone_toolchain/include/linux/i2c-dev.h"

#define ADDRESS 0x70
#define INCH 80
#define CM 81
#define MS 82
#define RANGE_REG 2
#define CMD_REG 0

int file;
char* cmd_buf;

int ultrasonic_open();
unsigned int range(char mode);

int main(int argc, char *argv[]){
	ultrasonic_open();	//Open i2c line
	while(1)printf("Range: %d\n",range(CM)); //Print range constantly forever
	return 0;
}

//Open the i2c line
int ultrasonic_open(){
	file = open( "/dev/i2c-0", O_RDWR );
	if( ioctl(file, I2C_SLAVE, ADDRESS)<0){
		fprintf(stderr, "Couldn't set slave address: %m\n");
		return 2;
	}
}

//Get the range
unsigned int range(char mode){
	unsigned int range;
	//Write to address 0 (command address)
	//Modes: 0x80 (inch), 0x81 (cm), 0x82 (millisecond)
	i2c_smbus_write_byte_data(file, CMD_REG, mode);
	//printf("Sent Command\n");
	//Get the range (keep trying until it works; the EEPROM is not readable until the ranging finishes)
	do{
		range = i2c_smbus_read_word_data(file,RANGE_REG);
	}while(range==0xffffffff);
	return range >> 8;
}
