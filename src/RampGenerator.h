/*
 * RampGenerator.h
 *
 *  Created on: 5 Mar 2024
 *      Author: william
 */

#ifndef RAMPGENERATOR_H_
#define RAMPGENERATOR_H_
#include <vector>
#include <math.h>

class RampGenerator {
public:
	RampGenerator();
	virtual ~RampGenerator();
	bool generateRamp(int rampSteps);
	float getRampValue(float position, bool direction);
	float getRampValue(bool direction);
	bool increment();
	bool reset();
	int rampResolution;
	std::vector<float> ramp;
	int nPos;

	bool loop(float destination);
	bool loopTwo(float destination);
	float vSpeed = 0.1;
	float posX, dirX, destX, velocity;
	float posXI = 0.0;

};

#endif /* RAMPGENERATOR_H_ */
