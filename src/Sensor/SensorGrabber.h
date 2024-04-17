#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>

#include "vl53l5cx_api.h"

#define TOF_SPEED_FPS 15
#define N_WIDTH 8
#define KNT_FRAMES TOF_SPEED_FPS / 2
#define MAX_DIST 1500                 //1.5m  Culling
#define MIN_DIST 300

typedef struct {
	uint8_t distance;
	uint8_t status;
} measureData;

class TofImager {
public:
	uint8_t status, isAlive;
	VL53L5CX_Configuration Dev;
	uint8_t setup();
	uint8_t getValue();
	uint8_t loop(VL53L5CX_Configuration *p_dev);

	uint8_t gridSize = VL53L5CX_RESOLUTION_8X8;
	uint8_t gridWid = N_WIDTH;

	int positionToSend = -1;
	float errorNumber = -1;

	float sensorMesauements[N_WIDTH][N_WIDTH] = { { } };      //check for change
	float noiseSubtraction[N_WIDTH][N_WIDTH][KNT_FRAMES] = { { { } } }; //check for change
	float resultGrid[N_WIDTH][N_WIDTH] = { { } };             //check for change
	int highestPositionBuffer[KNT_FRAMES] = { int(-1) };
	float xScore[N_WIDTH] = { };
	float xAverage[N_WIDTH] = { };
	int positions[N_WIDTH * N_WIDTH] = { int(-1) };
	int positionsBuffer[N_WIDTH * N_WIDTH][KNT_FRAMES] = { { int(0) } };
	float xAbsDifference[N_WIDTH];

	std::mutex mutex;

};

class SensorMan {
public:
	bool readingSensor = false;
	int sensorReading = -1;
	void setup();
	int getPositionValue();
	void startSensors();
	TofImager _sensor;
	std::unique_ptr<std::thread> sensorThread = nullptr;
};

