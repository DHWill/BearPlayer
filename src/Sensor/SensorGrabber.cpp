#include "SensorGrabber.h"

int exit_main_loop = 0;

uint8_t TofImager::setup() {

	uint8_t status, isAlive;
	VL53L5CX_Configuration Dev;

	vl53l5cx_comms_init(&Dev.platform);
	Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

	gridSize = VL53L5CX_RESOLUTION_8X8;
	gridWid = sqrt(gridSize);

	status = vl53l5cx_is_alive(&Dev, &isAlive);
	status = vl53l5cx_init(&Dev);
	status = vl53l5cx_set_resolution(&Dev, gridSize);
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, TOF_SPEED_FPS);
//	status = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);

	if (!(status || !isAlive)) {
		status = loop(&Dev);
	} else {
		std::cout << "SensorSetupFailed" << std::endl;
	}

	return status;
}

uint8_t TofImager::loop(VL53L5CX_Configuration *p_dev) {
	uint8_t loop, isReady, status;
	status = vl53l5cx_start_ranging(p_dev);
	VL53L5CX_ResultsData Results; /* Results data from VL53L5CX */
	uint8_t frame = 0;

	while (1) {
		isReady = wait_for_dataready(&p_dev->platform);
		if (isReady) {

			vl53l5cx_get_ranging_data(p_dev, &Results);
//			std::cout << "frameNumber: " << int(p_dev->streamcount) << std::endl;

			int nMod = 0;
			for (int y = 0; y <= gridWid * (gridWid - 1); y += gridWid) {
				for (int x = gridWid - 1; x >= 0; x--) {
					int pos = x + y;
					uint8_t status =
							Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE
									* pos];
					int measurement = int(
							Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE
									* pos]);
					positions[nMod] = measurement;
					positionsBuffer[nMod][frame] = measurement;
					nMod++;
				}
			}

			for (int n = 0; n < N_WIDTH * N_WIDTH; n++) {
				int stickyScore = 0;
				for (int k = 0; k < KNT_FRAMES - 1; k++) {
					int frameToCheck = (frame + 1 + k) % KNT_FRAMES;
					if (positions[n] == positionsBuffer[n][frameToCheck]) {
						stickyScore++;
					}
				}
				if (stickyScore == KNT_FRAMES - 1) {
					// Serial.println("foundStick");
					positions[n] = 0;
				}
			}

			//Normalise & Distance Masking
			for (int x = 0; x < N_WIDTH; x++) {
				float averageX = 0;
				int nVals = 0;
				for (int y = 0; y < N_WIDTH; y++) {
					int gridVal = positions[x + y * N_WIDTH];
					if ((gridVal < MAX_DIST) && (gridVal > MIN_DIST)) {
						resultGrid[x][y] = float(gridVal)
								/ float(MAX_DIST - MIN_DIST);
						averageX += resultGrid[x][y];
						nVals++;
					} else {
						resultGrid[x][y] = errorNumber;
					}
				}
				xAverage[x] = averageX / float(nVals);
				xScore[x] = nVals;
			}

//	                Print Grid
//	                 for (int y = 0; y < N_WIDTH; y++) {
//	                   for (int x = 0; x < N_WIDTH; x++) {
//	                     printf("\t");
//	                     if (resultGrid[x][y] != errorNumber) {
//	                       printf("%3f", resultGrid[x][y]);
//	                     } else {
//	                    	printf("x");
//	                     }
//	                   }
//	                   printf("\n");
//	                 }
//
//	                 printf("\n");

//Get Averages
			float nNearest = 99.0;
			int nNearPos = 99;
			for (int x = 0; x < N_WIDTH; x++) {
				// Serial.print("\t");
				// Serial.print(String(xAverage[x]));
				if (xAverage[x] < nNearest) {
					nNearest = xAverage[x];
					nNearPos = x;
				}
			}
			// Serial.println();
			// Serial.println();

			//Get Height of Block
			int highestScore = 0;
			int highestScorePosition = 0;
			for (int x = 0; x < N_WIDTH; x++) {
				// Serial.print("\t");
				// Serial.print(String(xScore[x]));
				if (xScore[x] > highestScore) {
					highestScore = xScore[x];
					highestScorePosition = x;
				}
			}

			//Get abs Difference of block
			// Serial.println();
			for (int x = 0; x < N_WIDTH; x++) {
				float lowestAbsDiff = 99;
				if (xScore[x] == highestScore) {
					float absDiff = 0.0;
					for (int y = 0; y < N_WIDTH; y++) {
						absDiff += abs(xAverage[x] - resultGrid[x][y]);
					}
					xAbsDifference[x] = absDiff;
					// Serial.print("\t");
					// Serial.print(absDiff);
				} else {
					// Serial.print("\t");
				}
			}
			// Serial.println();

			//Get absolute difference between blocks
			float flattestComp = 99.0;
			int flattestPosition = -99;
			for (int x = 0; x < N_WIDTH; x++) {
				if (xScore[x] == highestScore) {
					if (xAbsDifference[x] < flattestComp) {
						flattestPosition = x;
						flattestComp = xAbsDifference[x];
					}
				}
			}
			// Serial.println("Absolute Potision: " + String(flattestPosition));

			//Get Position Average
			highestPositionBuffer[frame] = highestScorePosition;
			int positionAverage = -1;
			int checkedFrames = 0;
			for (int x = 0; x < KNT_FRAMES; x++) {
				if (highestPositionBuffer[x] != -1) {
					positionAverage += highestPositionBuffer[x];
					checkedFrames++;
				}
			}
			positionAverage /= checkedFrames;

			//Only require left middle right (012) out of 0123 1/2 = front
			if (positionAverage == 0 || positionAverage == 1
					|| positionAverage == 2) {
				positionAverage = 0;
			}
			if (positionAverage == 3 || positionAverage == 4) {
				positionAverage = 1;
			}
			if (positionAverage == 5 || positionAverage == 6
					|| positionAverage == 7) {
				positionAverage = 2;
			}

			if ((xScore[flattestPosition] < 6)) {
				positionAverage = -1;
			}
			mutex.lock();
			positionToSend = positionAverage;
			mutex.unlock();

//	     std::cout << positionToSend << std::endl;
			frame++;
			frame %= KNT_FRAMES;
		}

		WaitMs(&p_dev->platform, 5);
	}

	status = vl53l5cx_stop_ranging(p_dev);
	return status;
}

int SensorMan::getPositionValue() {
	return _sensor.positionToSend;
}

void SensorMan::startSensors() {
	sensorThread.reset(new std::thread(&TofImager::setup, &_sensor));
}

