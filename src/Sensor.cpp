/*
 * Sensor.cpp
 *
 *  Created on: 31 Jan 2024
 *      Author: william
 */

#include "Sensor.h"

Sensor::Sensor() {
	// TODO Auto-generated constructor stub
}

Sensor::~Sensor() {
	// TODO Auto-generated destructor stub
}

int Sensor::openSensor(const char* device){
    int uart = open(device, O_RDWR | O_NONBLOCK);
//	    int uart = open(device, O_WRONLY | O_NOCTTY );
    if (uart == -1) {
        std::cout << " failed to open " << device << std::endl;
        return uart;
    }

    struct termios options;
    tcgetattr(uart, &options);
    cfsetispeed(&options, B9600);  // Set input baud rate
    cfsetospeed(&options, B9600);  // Set output baud rate



    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    // Disable flow control
    options.c_cflag &= ~CRTSCTS;

    // Disable hang-up-on-close to avoid reset
    // toptions.c_cflag &= ~HUPCL;

    // Turn on READ & ignore ctrl lines
    options.c_cflag |= CREAD | CLOCAL;
    // Turn off s/w flow ctrl
    options.c_iflag &= ~(IXON | IXOFF | IXANY  | INLCR | ICRNL);

    // Make raw
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // Reference: http://unixwiz.net/techtips/termios-vmin-vtime.html
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 20;



    tcsetattr(uart, TCSANOW, &options);
    return uart;
}

bool Sensor::writeToUART(int _fd, const char* data, size_t size) {

    ssize_t written = write(_fd, data, size);
    if (written == -1 || static_cast<size_t>(written) != size) {
//    	std::cout << "failed to write to UART!" << std::endl;
        return false;
    }
//    std::cout << "UART: " << data << std::endl;
    return true;
}


int Sensor::writeReadByteFromUART(int _fd, char* _toWrite) {
	int _ret = -1;
    if(writeToUART(_fd, _toWrite, sizeof(_toWrite))){
    	_ret = readByteFromUARTWithTimeout(_fd);
    }
    return _ret;
}


//To thread
bool Sensor::askForSensorReading(){
//	const char request[] = "POS?\r";
	if(fd == -1){
		fd = openSensor((const char*) "/dev/ttyLP0");
	}
	sensorReading = writeReadByteFromUART(fd, "POS?\r");
    return true;
}



int Sensor::readByteFromUARTWithTimeout(int _fd) {
//    const char* uartDevice = device; // Change this to your UART device

    std::string uartMessage;

    fd_set read_fds, write_fds, except_fds;
	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	FD_ZERO(&except_fds);
	FD_SET(_fd, &read_fds);


	struct timeval timeout;
	timeout.tv_sec = 0; //timeout in seconds
	timeout.tv_usec = 53330;	//timeout == 1 frame == 0.333 millis * 1000	VERY IMPORTANT to get this right to grab the latest but no disrupt video loop ** now threaded, keep within time of preroll buffer

	char buffer[256];
	if (select(_fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1) {
		ssize_t readBytes = read(_fd, buffer, sizeof(buffer));


		if (readBytes > 0) {
//			std::cout << "Received bytes : " << static_cast<int>(readBytes) << std::endl;
			for(ssize_t  i = 0; i < readBytes; i ++){
//				std::cout << "Received val : " << static_cast<int>(buffer[i]) << std::endl;
			}
		}
		else {
//			std::cout << "No Message: " << static_cast<int>(readBytes) << std::endl;
			return -1;
		}
	}
	else {
//		std::cout << "Timed Out: " << std::endl;
		return -1;
	}

    return buffer[0] - 0;
}



void Sensor::loop(){
	fd = openSensor((const char*) "/dev/ttyLP0");
	while(true){
		mutex.lock();
		readingSensor = true;
		sensorReading = writeReadByteFromUART(fd, "POS?\r");
		readingSensor = false;
		usleep(loopTime);
		mutex.unlock();
	}
}

void SensorThreader::setup(){
	sensorThread.reset(new std::thread(&Sensor::loop, &_sensor));
}

int SensorThreader::getSensorValue(){
	if(_sensor.sensorReading != -1){
		sensorReading = _sensor.sensorReading;
	}
	return sensorReading;
}

//On Join
int Sensor::getSensorReading(){
	return sensorReading;
}

