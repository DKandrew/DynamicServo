#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART, we need them for wirte() function
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 

#define switch 1
#define SWITCH_ON 1	 // SWITCH_ON = RX enable, RPi in receiving mode. 
#define SWITCH_OFF 0   // SWITCH_OFF = TX enable, RPi in transmitting mode. 
									   
using namespace std;

int writeData(int fd, char *data, int dataLen){
	digitalWrite(switch, SWITCH_OFF);
	delayMicroseconds(100);				//Need delay some time before the GPIO switch is truely turned off. 
										//I guess 20us is the limited time (reading from oscilloscope)
	int result = write(fd, data, dataLen);
	delayMicroseconds(80*dataLen);		// After sending the message, need delay some time before the GPIO switch is truely turned on.
										//I guess 400us is the limited time for transimitting 5 bytes (by tried and error)
	digitalWrite(switch, SWITCH_ON);
	return result;
}

//int readData(){
	
//}

//To compile: g++ UART_Comm.cpp -Wall -o uart -lwiringPi
int main(){
	// UART Setup
	int baud = 115200;	//baud rate
	int fd = serialOpen("/dev/ttyS0",baud);
	if(( fd < 0 )){
		std::cout << "Open file failed" << std::endl;
		return 0;
	}
	
	//Initialize GPIO switch
	wiringPiSetup();
	pinMode(switch, OUTPUT);
	digitalWrite(switch, SWITCH_ON);
	
	// UART test
	while(1){
		
		char data_2config[5] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
		writeData(fd, data_2config, 5);
		delay(5);
	}
}
