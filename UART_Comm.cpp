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

//important AX-12 constants
/////////////////////////////////////////////////////////// EEPROM AREA
#define    AX_MODEL_NUMBER_L 0x00
#define    AX_MODEL_NUMBER_H 0x01
#define    AX_VERSION 0x02
#define    AX_ID 0x03
#define    AX_BAUD_RATE 0x04
#define    AX_RETURN_DELAY_TIME 0x05
#define    AX_CW_ANGLE_LIMIT_L 0x06
#define    AX_CW_ANGLE_LIMIT_H 0x07
#define    AX_CCW_ANGLE_LIMIT_L 0x08
#define    AX_CCW_ANGLE_LIMIT_H 0x09
#define    AX_SYSTEM_DATA2 0x0A  //????
#define    AX_HIGHEST_LIMIT_TEMPERATURE 0x0B
#define    AX_LOWEST_LIMIT_VOLTAGE 0x0C
#define    AX_HIGHEST_LIMIT_VOLTAGE 0x0D
#define    AX_MAX_TORQUE_L 0x0E
#define    AX_MAX_TORQUE_H 0x0F
#define    AX_RETURN_LEVEL 0x10
#define    AX_ALARM_LED 0x11
#define    AX_ALARM_SHUTDOWN 0x12
#define    AX_OPERATING_MODE 0x13
#define    AX_DOWN_CALIBRATION_L 0x14
#define    AX_DOWN_CALIBRATION_H 0x15
#define    AX_UP_CALIBRATION_L 0x16
#define    AX_UP_CALIBRATION_H 0x17


////////////////////////////////////////////////////////////// RAM AREA
#define    AX_TORQUE_ENABLE 0x18
#define    AX_LED_ENABLE 0x19
#define    AX_CW_COMPLIANCE_MARGIN 0x1A
#define    AX_CCW_COMPLIANCE_MARGIN 0x1B
#define    AX_CW_COMPLIANCE_SLOPE 0x1C
#define    AX_CCW_COMPLIANCE_SLOPE 0x1D
#define    AX_GOAL_POSITION_L 0x1E
#define    AX_GOAL_POSITION_H 0x1F
#define    AX_MOVE_SPEED_L 0x20
#define    AX_MOVE_SPEED_H 0x21
#define    AX_TORQUE_LIMIT_L 0x22
#define    AX_TORQUE_LIMIT_H 0x23
#define    AX_PRESENT_POSITION_L 0x24
#define    AX_PRESENT_POSITION_H 0x25
#define    AX_PRESENT_SPEED_L 0x26
#define    AX_PRESENT_SPEED_H 0x27
#define    AX_PRESENT_LOAD_L 0x28
#define    AX_PRESENT_LOAD_H 0x29
#define    AX_PRESENT_VOLTAGE 0x2A
#define    AX_PRESENT_TEMPERATURE 0x2B
#define    AX_REGISTERED_INSTRUCTION 0x2C
#define    AX_PAUSE_TIME 0x2D
#define    AX_MOVING 0x2E
#define    AX_LOCK 0x2F
#define    AX_PUNCH_L 0x30
#define    AX_PUNCH_H 0x31

/////////////////////////////////////////////////////////////// Status Return Levels
#define    AX_RETURN_NONE 0x0
#define    AX_RETURN_READ 0x1
#define    AX_RETURN_ALL 0x2

/////////////////////////////////////////////////////////////// Instruction Set
#define    AX_PING 0x1
#define    AX_READ_DATA 0x2
#define    AX_WRITE_DATA 0x3
#define    AX_REG_WRITE 0x4
#define    AX_ACTION 0x5
#define    AX_RESET 0x6
#define    AX_SYNC_WRITE 0x83

/////////////////////////////////////////////////////////////// Lengths
#define    AX_RESET_LENGTH 2
#define    AX_ACTION_LENGTH 2
#define    AX_ID_LENGTH 4
#define    AX_LR_LENGTH 4
#define    AX_SRL_LENGTH 4
#define    AX_RDT_LENGTH 4
#define    AX_LEDALARM_LENGTH 4
#define    AX_SHUTDOWNALARM_LENGTH 4
#define    AX_TL_LENGTH 4
#define    AX_VL_LENGTH 6
#define    AX_AL_LENGTH 7
#define    AX_CM_LENGTH 6
#define    AX_CS_LENGTH 5
#define    AX_COMPLIANCE_LENGTH 7
#define    AX_CCW_CW_LENGTH 8
#define    AX_BD_LENGTH 4
#define    AX_TEM_LENGTH 4
#define    AX_MOVING_LENGTH 4
#define    AX_RWS_LENGTH 4
#define    AX_VOLT_LENGTH 4
#define    AX_LOAD_LENGTH 4
#define    AX_LED_LENGTH 4
#define    AX_TORQUE_LENGTH 4
#define    AX_POS_LENGTH 4
#define    AX_GOAL_LENGTH 5
#define    AX_MT_LENGTH 5
#define    AX_PUNCH_LENGTH 5
#define    AX_SPEED_LENGTH 5
#define    AX_GOAL_SP_LENGTH 7

/////////////////////////////////////////////////////////////// Specials
#define    AX_BYTE_READ 0x1
#define    AX_INT_READ 0x2
#define    AX_ACTION_CHECKSUM 0xFA
#define    AX_BROADCAST_ID 0xFE
#define    AX_START 0xFF
#define    AX_CCW_AL_L 0xFF
#define    AX_CCW_AL_H 0x3
#define    AX_LOCK_VALUE 0x1


using namespace std;

int modeFlag = 0; 		// if modeFlag == 0, it is in Wheel mode; if modeFlag == 1 it is in Joint mode


void printData(char data[], int len){
	for(int i=0; i<len; i++){
		printf("%x ", data[i]);
	}
	printf("\n");
}

int writeData(int fd, char *data, int dataLen){
	// Enable wirte
	digitalWrite(SWITCH, SWITCH_ON);
	delayMicroseconds(100);			//Delay
	
	int result = write(fd, data, dataLen);	//Write
	delayMicroseconds(100);			//Delay	
	// Disable wirte
	digitalWrite(SWITCH, SWITCH_OFF);
	
	return result;
}

// Wheel Mode is the mode where servo can have torque control
void setToWheelMode(int fd, int id){
	int len = 7;					// Data length is 5
	int checksum = ~(id + len + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L) & 0xff;  // Set both cwLimit and ccwLimit to zero
	int dataLen = len + 4;			// 4 represnets the length of FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, 0, 0, 0, 0, checksum};
	writeData(fd, data, dataLen);
	modeFlag = 0;		// Update modeFlag
	
	//For Debugging
	//printf("Checksum: %d \n", checksum);
	//printData(data, dataLen);
}


/* Joint Mode is the mode where servo can have position control.
   cwLimit is the min value of Goal position. ccwLimit is the max value of Goal position.
*/
void setToJointMode(int fd, int id, int cwLimit, int ccwLimit){
	if (cwLimit < 0) cwLimit = 0;		// min value of cwLimit is 0
	if (ccwLimit > 1023) ccwLimit = ccwLimit % 1024; // max value of ccwLimit is 1023
	
	// Set the lower & upper bytes of cwLimit and ccwLimit
	int cwLimit_l = cwLimit & 0xff;
	int cwLimit_h = cwLimit >> 8;
	int ccwLimit_l = ccwLimit & 0xff;
	int ccwLimit_h = ccwLimit >> 8;
	
	int len = 7;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + cwLimit_l + cwLimit_h + ccwLimit_l + ccwLimit_h) & 0xff; 
	int dataLen = len + 4;			// 4 represnets the length of FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, cwLimit_l, cwLimit_h, ccwLimit_l, ccwLimit_h, checksum};
	writeData(fd, data, dataLen);
	modeFlag = 1;		// Update modeFlag
	
	//For Debugging
	//printf("Checksum: %d, cw: %d, ccw: %d\n", checksum, cwLimit, ccwLimit);
	//printData(data, dataLen);
	return ;
}

void setPosition(int fd, int id, int position){
	//position = position % 1024;		// The range of AX servo position is from 0 (0x0) to 1023(0x3ff). 
	int posi_l = position & 0xff;	// Lower 8 bits of position 
	int posi_h = position >> 8;		// Upper 8 bits of position 
	int len = 5;					// Data length is 5
	int checksum = ~(id + len + AX_WRITE_DATA + AX_GOAL_POSITION_L + posi_l + posi_h) & 0xff; // First NOT the sum, then truncate it to 1 bytes, i.e. AND 0xFF
	int dataLen = 9; 		// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_GOAL_POSITION_L, posi_l, posi_h, checksum};
	writeData(fd, data, dataLen);

}

// This function will set the torque servo produces, i.e. Torque control
void setTorqueLimit(int fd, int id, int torque){
	//torque = torque % 1024;					// The range of torque is from 0 (0x0) to 1023(0x3ff). 
	int torque_limit_l = torque & 0xff;		// Lower 8 bits of torque limit
	int torque_limit_h = torque >> 8;		// Upper 8 bits of torque limit
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_TORQUE_LIMIT_L + torque_limit_l + torque_limit_h) & 0xff;
	int dataLen = 9; 		// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_TORQUE_LIMIT_L, torque_limit_l, torque_limit_h, checksum};
	writeData(fd, data, dataLen);
	
}

/*	This function will set the moving speed of the servo. Speed in Wheel mode and Joint mode are different
 * 	In the Joint mode, the range of speed is from 0~1023(0x3ff). The unit is about 0.111 rpm. At 1023, the (max) speed is 114 rpm
 * 	In the Wheel mode, the range of speed is from 0~2047(0x7ff). Range 0~1023 is rotating in CCW directions; Range 1024~2047 is rotating in CW directions; 
 */
void setMovingSpeed(int fd, int id, int speed){
	if(modeFlag == 0) speed %= 2048;		// If in wheel mode, speed limit is 2047.
	if(modeFlag == 1) speed %= 1024;		// If in wheel mode, speed limit is 1023.
	
	int moving_speed_l = speed & 0xff; 		// Lower 8 bits of moving speed
	int moving_speed_h = speed >> 8;		// Upper 8 bits of moving speed
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_MOVE_SPEED_L + moving_speed_l + moving_speed_h) & 0xff;
	int dataLen = 9; 			// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_MOVE_SPEED_L, moving_speed_l, moving_speed_h, checksum};
	writeData(fd, data, dataLen);
}


void test1(int fd, int posi){
	//posi = (posi + 1023)%1024;
		if(posi == 0) posi = 1023;
		else posi = 0;
		setPosition(fd, 1, posi);
		
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

		delay(1000);
		
}



//To compile: g++ UART_Comm.cpp -o uart -lwiringPi 
int main(){
	int baud = 1000000;				//wiringPi does not support 1M baud rate originally. You have to add this into the source code "wiringSerial.h" and re-build the wiringPi.
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
	
	int posi = 0;
				
	
	while(1){
		posi = (posi + 100) % 1023;
		//setToWheelMode(fd, 1);
		//setToJointMode(fd, 1, 30, 512);
		//setPosition(fd, 1, posi);
		//setTorqueLimit(fd, 1, posi);			//Note: speed and torque control are independent to each other
		//setMovingSpeed(fd, 1, 1024);
		Need to implement setMaxTorque func
		
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
		delay(1000);
	}
	serialClose(fd);
}




/*
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
		* 
*/
