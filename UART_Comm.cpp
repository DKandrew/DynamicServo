#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 

#define SWITCH 1
#define SWITCH_ON 0	 // SWITCH_ON = RX enable, RPi in receiving mode. 
#define SWITCH_OFF 1   // SWITCH_OFF = TX enable, RPi in transmitting mode. 

using namespace std;

int writeData(int fd, char *data, int dataLen){
	//printf("Check 2");
	
	
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
	
	//char data_reset[5] = {0xFA, 0xFF, 0x40, 0x00, 0xC1}; 
	//writeData(fd, data_reset, 5);
    //char data_resetAKL[5] = {0xFA, 0xFF, 0x3F, 0x00, 0xC2};
	//writeData(fd, data_resetAKL, 5);
	char data_2config[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
	writeData(fd, data_2config, 6);
	
	
	while(1){
		//printf("Check 1");
		int dataLen = 6;
		
		// Enable wirte
		digitalWrite(SWITCH, SWITCH_OFF);
		delayMicroseconds(100);
		//This code is used to capture signal in oscilloscope.
		writeData(fd, data_2config, 6);
		delayMicroseconds(100);		
		
		// Disable wirte
		digitalWrite(SWITCH, SWITCH_ON);
		
		//Flags for detecting FA, FF signal
		int fa=0; 
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
			
		//// Read
		//int i; 
		//char temp[0];
		//dataLen = 6;
		//char data[dataLen];
		//while(i<dataLen){
			//read(fd, (void*)temp, 1);
			//data[i] = temp[0];
			//i++;
		//}
		//printf("%x", data[2]);
		delay(1);
		
		
		
		
		//writeData(fd, data_setConfig, 17);
		//delay(10);
		//writeData(fd, data_2meas, 5);
		//delay(20);
		
		/*
		//Flags for detecting FA, FF signal
		int fa=0; 
		char temp[1];
		read(fd, (void*)temp, 1);
		if(temp[0] == 0xFA){
			fa = 1;
		}
		
		//If we find a sequence of FA FF, we find the starting of signal. Start reading
		if(fa){ 
			int i = 0;
			int dataLen = 49;
			char data[dataLen];
			while(i<dataLen){
				read(fd, (void*)temp, 1);
				data[i] = temp[0];
				i++;
			}
			
			long roll_angle = ((long)data[6]<<24) | ((long)data[7]<<16) | ((long)data[8]<<8) | (long)data[9];
			long pitch_angle  = ((long)data[10]<<24) | ((long)data[11]<<16) | ((long)data[12]<<8) | (long)data[13];
			long yaw_angle = ((long)data[14]<<24) | ((long)data[15]<<16) | ((long)data[16]<<8) | (long)data[17];
			
			long acc_x = ((long)data[21]<<24) | ((long)data[22]<<16) | ((long)data[23]<<8) | (long)data[24];
			long acc_y = ((long)data[25]<<24) | ((long)data[26]<<16) | ((long)data[27]<<8) | (long)data[28];
			long acc_z = ((long)data[29]<<24) | ((long)data[30]<<16) | ((long)data[31]<<8) | (long)data[32];
	
			long w1 = ((long)data[36]<<24) | ((long)data[37]<<16) | ((long)data[38]<<8) | (long)data[39];
			long w2 = ((long)data[40]<<24) | ((long)data[41]<<16) | ((long)data[42]<<8) | (long)data[43];
			long w3 = ((long)data[44]<<24) | ((long)data[45]<<16) | ((long)data[46]<<8) | (long)data[47];
			
			float x =  *((float*)&roll_angle);
			float y =  *((float*)&pitch_angle);
			float z =  *((float*)&yaw_angle);
			
			float ax = *((float*)&acc_x);
			float ay = *((float*)&acc_y);
			float az = *((float*)&acc_z);
			
			float wx = *((float*)&w1);
			float wy = *((float*)&w2);
			float wz = *((float*)&w3);
		
			printf("x: %f, y: %f, z: %f | ax: %f, ay: %f, az: %f | wx: %f, wy: %f, wz: %f \n", x, y, z, ax, ay, az, wx, wy, wz); 
			
		}
		*/
	}
	serialClose(fd);
}




//#include <iostream>
//#include <wiringPi.h>
//#include <wiringSerial.h>
//#include <cstdio>
//// Below are for UART, we need them for wirte() function
//#include <unistd.h>
//#include <fcntl.h>
//#include <termios.h> 

//#define SWITCH 1
//#define SWITCH_ON 1	 // SWITCH_ON = RX enable, RPi in receiving mode. 
//#define SWITCH_OFF 0   // SWITCH_OFF = TX enable, RPi in transmitting mode. 
									   
//using namespace std;

//int readData(int fd){
	///*
	//struct termios options;
	//tcgetattr(fd, &options);
	//options.c_cflag |= CREAD;
	//tcsetattr(fd, TCSANOW, &options);
	//*/
	
	////char receivedData[256];
	////int len = read(fd, &receivedData, 255);
	////printf("%d bytes \n", len);
	////if (len != -1)
	////	printf("%x", receivedData[0]);
		
	//char x[2];
	//int len = read (fd, &x, 2);
	//printf("%x %x \n", x[0],x[1]);
	//return len;
//}


//// This function will perform a data writting/transmitting 
//int writeData(int fd, char *data, int dataLen){
	
	//printf("Check 2");
	//// Enable wirte
	//digitalWrite(SWITCH, SWITCH_OFF);
	//delayMicroseconds(100);				//Need delay some time before the GPIO switch is truely turned off. 
										////I guess 20us is the limited time (reading from oscilloscope)
	//int result = write(fd, data, dataLen);
	//delayMicroseconds(50*dataLen);		// After sending the message, need delay some time before the GPIO switch is truely turned on.
										////I guess 400us is the limited time for transimitting 5 bytes (by tried and error)
	//// Disable wirte
	//digitalWrite(SWITCH, SWITCH_ON);
	
	//// Read 
	////delayMicroseconds(2000);	
	////char receivedData[256];
	////int len = read(fd, &receivedData, 255);
	
	////char x;
	////int len = read (fd, &x, 1);
	////printf("%x \n", x);
	
	////int len = serialGetchar(fd);
	////delay(1);
	////printf("%d bytes \n", len);
	////printf("%x",(long)receivedData[2]);
	
	//readData(fd);
	//return result;
//}


////To compile: g++ UART_Comm.cpp -o uart -lwiringPi
//int main(){
	//// UART Setup
	//// Note: wiringPi does not support 1M Baud Rate, so I have to re-write a serialSetup function by myself.
	////		 Following the code in http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	//// 		 Now this "uartSerialSetup" can support 1M Baud Rate, it is also capable of setting up more custormized specification such as parity. 
	
	//int baud = 1000000;
	//int fd = serialOpen("/dev/ttyS0",baud);
	//if(( fd < 0 )){
		//std::cout << "Open file failed" << std::endl;
		//return 0;
	//}
	//printf("%d\n", fd);
	
	////Initialize GPIO switch
	//wiringPiSetup();
	//pinMode(SWITCH, OUTPUT);
	//digitalWrite(SWITCH, SWITCH_ON);
	
	//// UART test
	//int i = 0;
	//while(1){
		///*
		//char data_2config[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC};
		//writeData(fd, data_2config, 8);
		//delay(1);
		//*/
		
		//printf("Check 1");
		//char data_2config[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
		//writeData(fd, data_2config, 6);
		//delay(1);
		
		
		
	//}
//}

