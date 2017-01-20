#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 

#define SWITCH 1		// This is corresponding to GPIO 18. In wiringPi it is 1. 
#define SWITCH_ON 1	 	// SWITCH_ON = TX enable, RPi in transmitting mode. 
#define SWITCH_OFF 0   	// SWITCH_OFF = RX enable, RPi in receiving mode. 

using namespace std;

int writeData(int fd, char *data, int dataLen){
	int result = write(fd, data, dataLen);
	return result;
}

//To compile: g++ UART_Comm.cpp -o uart -lwiringPi 
int main(){
	int baud = 1000000;	//baud rate
	int fd = serialOpen("/dev/ttyS0",baud);
	if(( fd < 0 )){
		std::cout << "Open file failed" << std::endl;
		return 0;
	}
	//Initialize GPIO switch
	wiringPiSetup();
	pinMode(SWITCH, OUTPUT);
	digitalWrite(SWITCH, SWITCH_ON);

	char data_2config[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
	
	while(1){
		int dataLen = 6;
		
		// Enable wirte
		digitalWrite(SWITCH, SWITCH_ON);
		delayMicroseconds(100);			//Delay
		
		writeData(fd, data_2config, 6);	//Write
		delayMicroseconds(100);			//Delay
		
		// Disable wirte
		digitalWrite(SWITCH, SWITCH_OFF);
		
		// Read Status Packet 
		int fa=0; 				//Flags for detecting FF signal
		char temp[1];
		read(fd, (void*)temp, 1);
		if(temp[0] == 0xFF){
			fa = 1;
		}
		if(fa){
			int i = 0;
			int dataLen = 5;
			char data[dataLen];
			while(i<dataLen){
				read(fd, (void*)temp, 1);
				data[i] = temp[0];
				i++;
			}
			printf("0xff detected, next: ");
			i = 0;
			while(i<dataLen){
				printf("%x ", data[i]);
				i++;
			}
			printf("\n");
		}

		delay(1);
	}
	serialClose(fd);
}
