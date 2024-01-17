#include "serial.h"
#include <iostream>
//#include <fstream>

using namespace std;

int open_serial(char *dev_name, int baud, int vtime, int vmin)
{
    int fd;
    struct termios newtio;

    //open serial port
    fd = open(dev_name, O_RDWR | O_NOCTTY);
    //cout << "hahah" << endl;
    if(fd < 0)
    {
        //fail open
    	cout << "fail" << endl;
        return -1;
    }

    tcgetattr(fd,&newtio);

    // port configure
//    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;    // no-parity
    newtio.c_oflag = 0;

    newtio.c_cflag = CS8 | CLOCAL | CREAD;  // no-rts & no-cts
    newtio.c_cflag &= ~CSTOPB; //set stop bit to 1 

    switch(baud)
    {
		case 500000 : newtio.c_cflag |= B500000; break;
		case 250000 : newtio.c_cflag |= B230400; break;
		case 115200 : newtio.c_cflag |= B115200; break;
		case 57600  : newtio.c_cflag |= B57600; break;
        case 38400  : newtio.c_cflag |= B38400; break;
		case 9600   : newtio.c_cflag |= B9600; break;

		default     : newtio.c_cflag |= B115200; break;
    }

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
    newtio.c_cc[VMIN] = vmin;       // wait

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}
void close_serial(int fd)
{
    close(fd);
}




