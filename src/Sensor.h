/*
 * Sensor.h
 *
 *  Created on: 31 Jan 2024
 *      Author: william
 */
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <string.h>
#include <sys/select.h>
#include <cerrno>
#include <vector>
#include <thread>
#include <mutex>

#ifndef SENSOR_H_
#define SENSOR_H_

#define	O_NONBLOCK	0x0004	/* Non-blocking I/O.  */
#define	O_CREAT		0x0200	/* Create file if it doesn't exist.  */
#define	O_EXCL		0x0800	/* Fail if file already exists.  */
#define	O_TRUNC		0x0400	/* Truncate file to zero length.  */
#define	O_NOCTTY	0x8000	/* Don't assign a controlling terminal.  */
#define	O_ASYNC		0x0040	/* Send SIGIO to owner when data is ready.  */
#define	O_FSYNC		0x0080	/* Synchronous writes.  */

#define	O_RDONLY	0	/* Open read-only.  */
#define	O_WRONLY	1	/* Open write-only.  */
#define	O_RDWR		2	/* Open read/write.  */

/* File status flags for `open' and `fcntl'.  */
#define	O_APPEND	0x0008	/* Writes append to the file.  */
#define	O_NONBLOCK	0x0004	/* Non-blocking I/O.  */

class Sensor {
public:
	Sensor();
	virtual ~Sensor();

    bool writeToUART(int _fd, const char* data, size_t size);
    int writeReadByteFromUART(int _fd, char* _toWrite);
    int readByteFromUARTWithTimeout(int _fd);
    bool readFromUARTWithTimeout2(int _fd);
    bool readSensorReading();
    bool askForSensorReading();
    int getSensorReading();
    std::atomic<int> sensorValue();
    int openSensor(const char* device);
    int fd= -1;
    int sensorReading = -1;
    bool readingSensor = false;
    void  loop();
    unsigned int microsecond = 1000000;
    unsigned int loopTime = int(microsecond * 0.5);
    std::mutex mutex;

};

class SensorThreader{
public:
    bool readingSensor = false;
    int sensorReading = -1;
    void setup();
    int getSensorValue();
    Sensor _sensor;
    std::unique_ptr<std::thread> sensorThread = nullptr;
};

#endif /* SENSOR_H_ */
