#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "json.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <random>
#include <algorithm>
//#include <cstdlib.h>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
//#define JSON_IS_AMALGAMATION
#include "Sensor.h"


class StateMachine
{
public:

    class State{
        public:
            std::string name;
            int startTime;
            int endTime;
            int transitionFromParent;
            std::string parentAnimation;
            std::string parentState;
            int position;
            int targetPosition;
            bool isEarlyExit;
            bool isActive;
            std::vector<State> earlyExits;
            bool hasEarlyExits = false;


            bool operator < (const State& transition) const
            {
                return (transitionFromParent < transition.transitionFromParent);
            }
    };


    struct Segment {
    	std::string name;
    	int startTime;
    	int endTime;
    };

    StateMachine();
    bool parseFile(std::string _fileLocation);
    bool updateFrame(int _frame);
    void setTargetPosition(int position);
    State updateState();
    int randomPositionForTesting();
    void updateSegment();
    Segment currentSegment;
    void setIsActive(bool _isActive);
    bool getIsInit();
    bool init = true;


    int lastTargetPosition = -1;
    int currentPosition;
    int targetPosition;
    State currentState;
    State lastState;
    State earlyExitState;
    State earlyExitQueue;
    bool isExitingEarly = false;
    int currentTime;
    int earlyExitCount = 0;
    bool isExtendedState = false;
    bool isActive = false;
    Sensor sensor;
    std::vector<int> possiblePositions;
    int staleStateRoll;



    std::vector<State> readFile();
    std::vector<State> states;
    std::deque<State> tempEarlyExits;
    void getTempEarlyExits(std::vector<State> _earlyExits);

};

#endif // STATEMACHINE_H
