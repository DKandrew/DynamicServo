#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 
//
#include <string>

using namespace std; 

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

int modeFlag = 0; 		// if modeFlag == 0, it is in Wheel mode; if modeFlag == 1 it is in Joint mode
int DEBUG = 1;			// Debug flag
int READ_DELAY_TIME = 200;	// By default, the servo delay 0.5 msec to send the status packet after it receives inst packet. 
							// But the acutal delay time can be modified if you changed the Return Delay Time register
							// 100 is picked by experiment. 50 will fail.
int currPos = 0;
int isFail = 0;
int expect = 0;

void printData(char data[], int len){
	for(int i=0; i<len; i++){
		printf("%x ", data[i]);
	}
	printf("\n");
}

/*	This function will write data into the dynamicxel servo. 
 * 	There are some delay after the swtich is turned on and when write() is called.
 * 	The time is manually picked. The delay time after write() cannot be zero otherwise we fail to read status packet from servo.
 */
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

/* This function will read the data from servo into buffer. Because it uses the first 0xff in the status packet, 
 * The returned buffer will only the rest of the information (i.e. ignored the first 0xff)
 * buffer is pre-allocated. User should use delete [] after calling this function. len is the total length of the buffer. 
 */
void readData(int fd, char* buffer, int len){
	// Read Status Packet 
	int fa=0; 				//Flags for detecting FF signal
	char temp[2];
	
	// Read the first byte and check if it is 0xFF
	read(fd, (void*)temp, 2);
	if(temp[0] == 0xFF && temp[1] == 0xFF){
		fa = 1;
	}
	
	if(fa){
		buffer[0] = temp[0];
		buffer[1] = temp[1];
		
		int i = 0;
		int dataLen = len - 3;
		read(fd, (void*)(buffer + 2), dataLen);
		
		//Debug
		if(DEBUG == 1){
			printf("0xff, oxff detected, next: ");
			i = 0;
			while(i<dataLen){
				printf("%x ", buffer[i+2]);
				i++;
			}
			printf("\n");
		}
		
		// Add '\0' at the last byte of buffer
		buffer[len - 1] = '\0';
	}
}

int readRegister(int fd, int id, int inst){
	int rpl = 0;	// Read Parameter Length
	switch (inst){
		case AX_MODEL_NUMBER_L: rpl = 2; break;
		case AX_VERSION: 		rpl = 1; break;
		case AX_ID: 			rpl = 1; break;
		case AX_BAUD_RATE: 		rpl = 1; break;
		case AX_RETURN_DELAY_TIME: 	rpl = 1; break;
		case AX_CW_ANGLE_LIMIT_L: 	rpl = 2; break;
		case AX_CCW_ANGLE_LIMIT_L: 	rpl = 2; break;
		case AX_HIGHEST_LIMIT_TEMPERATURE: 	rpl = 1; break;
		case AX_LOWEST_LIMIT_VOLTAGE: 		rpl = 1; break;
		case AX_HIGHEST_LIMIT_VOLTAGE: 		rpl = 1; break;
		case AX_MAX_TORQUE_L: 	rpl = 2; break;
		case AX_RETURN_LEVEL: 	rpl = 1; break;
		case AX_ALARM_LED: 		rpl = 1; break;
		case AX_ALARM_SHUTDOWN: rpl = 1; break;
		case AX_TORQUE_ENABLE: 	rpl = 1; break;
		case AX_LED_ENABLE: 	rpl = 1; break;
		case AX_CW_COMPLIANCE_MARGIN: 	rpl = 1; break;
		case AX_CCW_COMPLIANCE_MARGIN: 	rpl = 1; break;
		case AX_CW_COMPLIANCE_SLOPE: 	rpl = 1; break;
		case AX_CCW_COMPLIANCE_SLOPE: 	rpl = 1; break;
		case AX_GOAL_POSITION_L: 	rpl = 2; break;
		case AX_MOVE_SPEED_L: 		rpl = 2; break;
		case AX_TORQUE_LIMIT_L: 	rpl = 2; break;
		case AX_PRESENT_POSITION_L: rpl = 2; break;
		case AX_PRESENT_SPEED_L: 	rpl = 2; break;
		case AX_PRESENT_LOAD_L: 	rpl = 2; break;
		case AX_PRESENT_VOLTAGE: 	rpl = 1; break;
		case AX_PRESENT_TEMPERATURE: 	rpl = 1; break;
		case AX_REGISTERED_INSTRUCTION: rpl = 1; break;
		case AX_MOVING: 	rpl = 1; break;
		case AX_LOCK: 		rpl = 1; break;
		case AX_PUNCH_L: 	rpl = 2; break;
		default: break;	
	}
	
	// Write data 
	int len = 4;
	int checksum = ~(id + len + AX_READ_DATA + inst + rpl) &  0xff;
	int dataLen = len + 4;
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_READ_DATA, inst, rpl, checksum};
	writeData(fd, data, dataLen);
	
	// Read Delay
	delayMicroseconds(READ_DELAY_TIME);	
	
	// Read data
	int readLen = 7 + rpl;		// Status packet will return "FF FF ID LEN ERROR [rpl] CHECKSUM \0" (\0 means the end of a string), so readlen should be 6 bytes (everything except parameters) + rpl
	char* buffer = new char[readLen];	
	readData(fd, buffer, readLen);		 
	int result = 0;						
	if(*buffer == 0xff){
		int index = 5; 		// Skip id, len, error in status packet which is 4 bytes in total.
		if (rpl > 1){
			int lowbyte = buffer[index];
			int highbyte = buffer[index + 1]; 
			result = highbyte << 8 | lowbyte;
		}else {
			result = buffer[index];
		}
		isFail = 0;
	}else{
		isFail = 1;//printf("Fail to read register.\n");
	}
	
	// Delete buffer
	delete [] buffer; 
	
	// Return result
	return result;
}


void torqueEnable(int fd, int id, int enable){
	int len = 4;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_TORQUE_ENABLE + enable) & 0xff;
	int dataLen = len + 4;
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_TORQUE_ENABLE, enable, checksum};
	writeData(fd, data, dataLen);
}

// Wheel Mode is the mode where servo can have torque control
void setToWheelMode(int fd, int id){
	int len = 7;					// Data length is 7
	int checksum = ~(id + len + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L) & 0xff;  // Set both cwLimit and ccwLimit to zero
	int dataLen = len + 4;			// 4 represnets the length of FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, 0, 0, 0, 0, checksum};
	writeData(fd, data, dataLen);
	//torqueEnable(fd, id, 1);
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
	if(modeFlag == 1) speed %= 1024;		// If in joint mode, speed limit is 1023.
	
	int moving_speed_l = speed & 0xff; 		// Lower 8 bits of moving speed
	int moving_speed_h = speed >> 8;		// Upper 8 bits of moving speed
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_MOVE_SPEED_L + moving_speed_l + moving_speed_h) & 0xff;
	int dataLen = 9; 			// Data length is 9 = len + 4. 4 represnets FF FF id len
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_MOVE_SPEED_L, moving_speed_l, moving_speed_h, checksum};
	writeData(fd, data, dataLen);
}

void setMaxTorque(int fd, int id, int torque){
	int torque_limit_l = torque & 0xff;		// Lower 8 bits of torque limit
	int torque_limit_h = torque >> 8;		// Upper 8 bits of torque limit
	int len = 5;
	int checksum = ~(id + len + AX_WRITE_DATA + AX_MAX_TORQUE_L + torque_limit_l + torque_limit_h) & 0xff;
	int dataLen = len + 4; 		
	char data[dataLen] = {0xFF, 0XFF, id, len, AX_WRITE_DATA, AX_MAX_TORQUE_L, torque_limit_l, torque_limit_h, checksum};
	writeData(fd, data, dataLen); 
}

int readPresentPosition(int fd, int id){
	return readRegister(fd, id, AX_PRESENT_POSITION_L);
}

void servoControl(int fd, int id, int CW_limit, int CCW_limit, int step_len, int speed){
	cout << "Start function with currPos: " << currPos << ", step_len" << step_len << endl;
	int target =  currPos + step_len;//+CW_limit);
	if(target > CCW_limit) target = target-CCW_limit+CW_limit;
	cout << "target: " << target << endl;
	setPosition(fd, id, target);
	setMovingSpeed(fd, id, speed);
	cout << "Finish task" << endl;
	expect = target;
}

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
	int id = 1;
	
	int CW_limit, CCW_limit, step_len, speed;
	int low_limit, hi_limit;
	do{
		cout << "Please input the lowest limit of the servo(from 0 to 300 degree): ";
		cin >> low_limit;
		cout << "Please input the highest limit of the servo(from 0 to 300 degree): ";
		cin >> hi_limit;
	}while(low_limit >= hi_limit);
	int singleAng = 1024/300;
	low_limit = (int)(low_limit * singleAng);
	hi_limit = (int)(hi_limit * singleAng);
	int isSame = 0;
	
	cout << "low_limit is " << low_limit << ", hi_limit is " << hi_limit << endl;
	cout << "Initialize to the low limit.........." << endl;
	setToJointMode(fd, id, low_limit, hi_limit);
	setPosition(fd, id, low_limit);
	setMovingSpeed(fd, id, 100);
	cout << "Finish initialize." << endl;
	expect = low_limit;
	
	while(1){
		int isContinue = 1;
		if(!isSame){
			cout << "Please input the step length you want(in degree): ";
			cin >> step_len;
			step_len *= singleAng;
			cout << "Please input the speed you want: ";
			cin >> speed;
		}
		currPos = readPresentPosition(fd, id);
		while(isFail == 1 || currPos < expect-3 || currPos > expect + 3){
			cout << ".";//cout << "Start reading" << endl;
			currPos = readPresentPosition(fd, id);
			cout << currPos << ", Expect: " << expect << endl;
			delay(200);
		}
		cout << "This is the current position." << currPos << endl;
		servoControl(fd, id, low_limit, hi_limit, step_len, speed);
		cout << "Do you want to continue with the same lo and hi limit of the servo (1 for continue, 0 for exit)? ";
		cin >> isContinue;
		if(!isContinue) break;
		else{
			cout << "Do you want to use the same step length and same speed (1 for yes, 0 for no)? ";
			cin >> isSame;
		}
		cout << "-----------------------------------------------------------------------------------" << endl;
	}
	return 0;
}
