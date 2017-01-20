#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART, we need them for wirte() function
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 

#define SWITCH 1
#define SWITCH_ON 1	 // SWITCH_ON = RX enable, RPi in receiving mode. 
#define SWITCH_OFF 0   // SWITCH_OFF = TX enable, RPi in transmitting mode. 
									   
using namespace std;

int readData(int fd){
	/*
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag |= CREAD;
	tcsetattr(fd, TCSANOW, &options);
	*/
	
	//char receivedData[256];
	//int len = read(fd, &receivedData, 255);
	//printf("%d bytes \n", len);
	//if (len != -1)
	//	printf("%x", receivedData[0]);
		
	char x[2];
	int len = read (fd, &x, 2);
	printf("%x %x \n", x[0],x[1]);
	return len;
}


// This function will perform a data writting/transmitting 
int writeData(int fd, char *data, int dataLen){
	/*
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag &= ~CREAD;			//Disable interrupt
	tcsetattr(fd, TCSANOW, &options);
	*/
	
	// Enable wirte
	digitalWrite(SWITCH, SWITCH_OFF);
	delayMicroseconds(100);				//Need delay some time before the GPIO switch is truely turned off. 
										//I guess 20us is the limited time (reading from oscilloscope)
	int result = write(fd, data, dataLen);
	delayMicroseconds(50*dataLen);		// After sending the message, need delay some time before the GPIO switch is truely turned on.
										//I guess 400us is the limited time for transimitting 5 bytes (by tried and error)
	// Disable wirte
	digitalWrite(SWITCH, SWITCH_ON);
	
	// Read 
	//delayMicroseconds(2000);	
	//char receivedData[256];
	//int len = read(fd, &receivedData, 255);
	
	//char x;
	//int len = read (fd, &x, 1);
	//printf("%x \n", x);
	
	//int len = serialGetchar(fd);
	//delay(1);
	//printf("%d bytes \n", len);
	//printf("%x",(long)receivedData[2]);
	
	readData(fd);
	return result;
}

// This function will initialize the uart serial port and return this corresponding filestream. 
int uartSerialSetup(char* device, int baud){
	
	//Following code are all copied based on http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	int uart0_filestream = -1;
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	//fcntl (uart0_filestream, F_SETFL, O_RDWR) ;
	
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	speed_t myBaud ;
	
	tcgetattr(uart0_filestream, &options);	//Get attribute
	switch (baud)
	{
	    case     50:	myBaud =     B50 ; break ;
	    case     75:	myBaud =     B75 ; break ;
	    case    110:	myBaud =    B110 ; break ;
	    case    134:	myBaud =    B134 ; break ;
	    case    150:	myBaud =    B150 ; break ;
	    case    200:	myBaud =    B200 ; break ;
	    case    300:	myBaud =    B300 ; break ;
	    case    600:	myBaud =    B600 ; break ;
	    case   1200:	myBaud =   B1200 ; break ;
	    case   1800:	myBaud =   B1800 ; break ;
	    case   2400:	myBaud =   B2400 ; break ;
	    case   4800:	myBaud =   B4800 ; break ;
	    case   9600:	myBaud =   B9600 ; break ;
	    case  19200:	myBaud =  B19200 ; break ;
	    case  38400:	myBaud =  B38400 ; break ;
	    case  57600:	myBaud =  B57600 ; break ;
	    case 115200:	myBaud = B115200 ; break ;
	    case 230400:	myBaud = B230400 ; break ;
	    case 1000000:	myBaud = B1000000 ; break ;
	
	    default:
	      return myBaud = B115200;
	}
	
	options.c_cflag = myBaud | CS8 | CLOCAL | CREAD;	
	
	// Set other stuffs
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	
	return uart0_filestream;
}


//To compile: g++ UART_Comm.cpp -o uart -lwiringPi
int main(){
	// UART Setup
	// Note: wiringPi does not support 1M Baud Rate, so I have to re-write a serialSetup function by myself.
	//		 Following the code in http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	// 		 Now this "uartSerialSetup" can support 1M Baud Rate, it is also capable of setting up more custormized specification such as parity. 
	
	/*
	int baud = 1000000;	//baud rate
	int fd = uartSerialSetup("/dev/ttyS0",baud);
	*/
	
	
	int baud = 1000000;
	int fd = serialOpen("/dev/ttyS0",baud);
	if(( fd < 0 )){
		std::cout << "Open file failed" << std::endl;
		return 0;
	}
	
	
	
	//Initialize GPIO switch
	wiringPiSetup();
	pinMode(SWITCH, OUTPUT);
	digitalWrite(SWITCH, SWITCH_ON);
	
	// UART test
	int i = 0;
	while(1){
		/*
		char data_2config[8] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC};
		writeData(fd, data_2config, 8);
		delay(1);
		*/
		
		
		char data_2config[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
		writeData(fd, data_2config, 6);
		delay(1);
		
		
	}
}


/*
// This function will initialize the uart serial port and return this corresponding filestream. 
int uartSerialSetup(char* device, int baud){
	
	//Following code are all copied based on http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	int uart0_filestream = -1;
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	//fcntl (uart0_filestream, F_SETFL, O_RDWR) ;
	
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	speed_t myBaud ;
	
	tcgetattr(uart0_filestream, &options);	//Get attribute
	switch (baud)
	{
	    case     50:	myBaud =     B50 ; break ;
	    case     75:	myBaud =     B75 ; break ;
	    case    110:	myBaud =    B110 ; break ;
	    case    134:	myBaud =    B134 ; break ;
	    case    150:	myBaud =    B150 ; break ;
	    case    200:	myBaud =    B200 ; break ;
	    case    300:	myBaud =    B300 ; break ;
	    case    600:	myBaud =    B600 ; break ;
	    case   1200:	myBaud =   B1200 ; break ;
	    case   1800:	myBaud =   B1800 ; break ;
	    case   2400:	myBaud =   B2400 ; break ;
	    case   4800:	myBaud =   B4800 ; break ;
	    case   9600:	myBaud =   B9600 ; break ;
	    case  19200:	myBaud =  B19200 ; break ;
	    case  38400:	myBaud =  B38400 ; break ;
	    case  57600:	myBaud =  B57600 ; break ;
	    case 115200:	myBaud = B115200 ; break ;
	    case 230400:	myBaud = B230400 ; break ;
	    case 1000000:	myBaud = B1000000 ; break ;
	
	    default:
	      return myBaud = B115200;
	}
	
	// Set baud rate
	switch(baud){
		case 9600:
			options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		
			break;
		case 115200:
			options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		
			break;
		case 1000000:
			options.c_cflag = B1000000 | CS8 | CLOCAL;		// We can not add CREAD because it enables interrupt of UART. Interrupt should be disable during servo communication 
			break;
		default:
			options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		
			break;
	}
	
	// Set other stuffs
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	
	return uart0_filestream;
}
*/

/*
// This function will initialize the uart serial port and return this corresponding filestream. 
int uartSerialSetup(char* device, int baud){
	
	//Following code are all copied based on http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
	int uart0_filestream = -1;
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open(device, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	speed_t myBaud ;
	
	tcgetattr(uart0_filestream, &options);	//Get attribute
	switch (baud)
	{
	    case     50:	myBaud =     B50 ; break ;
	    case     75:	myBaud =     B75 ; break ;
	    case    110:	myBaud =    B110 ; break ;
	    case    134:	myBaud =    B134 ; break ;
	    case    150:	myBaud =    B150 ; break ;
	    case    200:	myBaud =    B200 ; break ;
	    case    300:	myBaud =    B300 ; break ;
	    case    600:	myBaud =    B600 ; break ;
	    case   1200:	myBaud =   B1200 ; break ;
	    case   1800:	myBaud =   B1800 ; break ;
	    case   2400:	myBaud =   B2400 ; break ;
	    case   4800:	myBaud =   B4800 ; break ;
	    case   9600:	myBaud =   B9600 ; break ;
	    case  19200:	myBaud =  B19200 ; break ;
	    case  38400:	myBaud =  B38400 ; break ;
	    case  57600:	myBaud =  B57600 ; break ;
	    case 115200:	myBaud = B115200 ; break ;
	    case 230400:	myBaud = B230400 ; break ;
	    case 1000000:	myBaud = B1000000 ; break ;
	
	    default:
	      return myBaud = B115200;
	}
	
	cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;
    
	options.c_cflag |= (CLOCAL ) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

	tcsetattr (uart0_filestream, TCSANOW | TCSAFLUSH, &options) ;
	
	return uart0_filestream;
}
*/
