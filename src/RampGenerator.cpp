/*
 * RampGenerator.cpp
 *
 *  Created on: 5 Mar 2024
 *      Author: william
 */

#include "RampGenerator.h"

RampGenerator::RampGenerator() {
}

RampGenerator::~RampGenerator() {
	// TODO Auto-generated destructor stub
}
bool RampGenerator::generateRamp(int rampSteps) {
	ramp.clear();
	for (int x = 0; x < rampSteps; x++) {
		ramp.push_back(float(1. / (rampSteps - 1) * x));
	}
}
float RampGenerator::getRampValue(float position, bool direction) {
	return (direction) ? ramp[position] : ramp[(ramp.size() - 1) - position];
}
float RampGenerator::getRampValue(bool direction) {
	return (direction) ? ramp[nPos] : ramp[(ramp.size() - 1) - nPos];
}
bool RampGenerator::increment() {
	nPos < (ramp.size() - 1) ? nPos++ : nPos = 0;
	return !(nPos == 0);
}

bool RampGenerator::loop(float destination) {
	if (posXI != destination) {

		destX = destination;
		destX *= 2.;
		destX -= 1.;
		velocity = destX - posX;
		posX += (velocity * vSpeed);

		posXI = posX + 1;
		posXI /= 2;
	} else {
		return false;
	}

	if (abs(posXI - destination) < vSpeed) {
		posXI = destination;
		return true;
	}
	return (abs(posXI - destination) > vSpeed);
}

bool RampGenerator::loopTwo(float destination) {
	if (posXI != destination) {

		destX = destination;
		destX *= 2.;
		destX -= 1.;
		velocity = destX - posX;
		posX += (velocity * vSpeed);
		posXI = posX + 1;
		posXI /= 2;
	} else {
		return false;
	}

	if (abs(posXI - destination) < 0.002) {
		posXI = destination;
		return true;
	}
	return (abs(posXI - destination) > 0.002);
}

