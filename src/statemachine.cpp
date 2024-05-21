#include "statemachine.h"
//using namespace std;
//#define JSON_IS_AMALGAMATION

StateMachine::StateMachine() {

}

int StateMachine::randomPositionForTesting() {
	std::random_device dev;
	std::mt19937 rng(dev());
	int nPositions = possiblePositions.size() -1;
	std::uniform_int_distribution<std::mt19937::result_type> dist6(0,nPositions); // distribution in range [1, 6]
	int randomNumber = dist6(rng);
	int randomPosToUse = possiblePositions[randomNumber];
	std::cout << "randomPosition generated" << randomPosToUse << std::endl;
	return randomPosToUse;
}

bool StateMachine::parseFile(std::string _fileLocation) {
	std::ifstream animations_file(_fileLocation.c_str(), std::ifstream::binary);
	Json::Value animations;
	Json::Reader reader;
	if (!reader.parse(animations_file, animations, false)) {
		std::cout << "json no good" << std::endl;
		return false;
	}
	std::vector<StateMachine::State> _states;
	std::vector<StateMachine::State> _earlyExits;
	for (Json::Value::ArrayIndex i = 0; i != animations.size(); i++) {
		State state;
		state.name = std::string(animations[i].get("name", "nan").asString());
		state.startTime = int(animations[i].get("startTime", "nan").asInt() - 1);	//-1
		state.endTime = int(animations[i].get("endTime", "nan").asInt() - 1);//inaccuracies in frame numbers?
		state.transitionFromParent = int(animations[i].get("transitionFromParent", "nan").asInt());//try-1
		state.parentAnimation = std::string(animations[i].get("parentAnimation", "nan").asString());
		state.parentState = std::string(animations[i].get("parentState", "nan").asString());
		state.position = int(animations[i].get("position", "nan").asInt());
		state.targetPosition = int(animations[i].get("targetPosition", "nan").asInt());
		state.isEarlyExit = bool(animations[i].get("isEarlyExit", "nan").asBool());
		state.isActive = bool(animations[i].get("isActive", "nan").asBool());

		if (state.isEarlyExit == false) {
			_states.push_back(state);
		} else {
			_earlyExits.push_back(state);
		}
	}

	for (size_t s = 0; s < _states.size(); s++) {
		std::vector<StateMachine::State> _sortEarlyExits;
		std::string name = _states[s].name;
		for (size_t e = 0; e < _earlyExits.size(); e++) {
			if (_earlyExits[e].parentAnimation == name) {
				_sortEarlyExits.push_back(_earlyExits[e]);
			}
		}
		std::sort(_earlyExits.begin(), _earlyExits.end());//sort early exits on time from start of clip

		for (size_t e = 0; e < _sortEarlyExits.size(); e++) {
			std::cout << "adding: " << _sortEarlyExits[e].name << "to: "
					<< _states[s].name << std::endl;
			_states[s].earlyExits.push_back(_sortEarlyExits[e]);
		}
		if (_states[s].earlyExits.size() > 0) {
			_states[s].hasEarlyExits = true;
		}
	}

	for (size_t e = 0; e < _states.size(); e++) {
		states.push_back(_states[e]);
	}

	possiblePositions.push_back(states[0].position);
	for (size_t e = 0; e < states.size(); e++) {
		bool isUnique = true;
		for (size_t j = 0; j < possiblePositions.size(); j++) {
			if (states[e].position == possiblePositions[j]) {
				isUnique = false;
			}
		}
		if (isUnique) {
			possiblePositions.push_back(states[e].position);
		}
	}
	for (size_t j = 0; j < possiblePositions.size(); j++) {
		std::cout << "possiblePositions: " << possiblePositions[j] << std::endl;
	}

	return true;
}

void StateMachine::setTargetPosition(int position) {
//	position = randomNumberForTesting();

//    targetPosition = possiblePositions[position % possiblePositions.size()];	//fix this, currently there are 2 positions for flowers: 3 for animals

	targetPosition = position;
	if ((tempEarlyExits.size() > 0)) {
		if (targetPosition != currentState.position) {
			std::cout << "QueuingEarlyExit: " << tempEarlyExits[0].name<< std::endl;
			lastTargetPosition = targetPosition;
			isExitingEarly = true;
		}
	}
}

void StateMachine::setIsActive(bool _isActive) {
	isActive = _isActive;
}

StateMachine::State StateMachine::updateState() {

	if(currentState.position == targetPosition){
		if(staleStateRoll > 3){
			setTargetPosition((randomPositionForTesting() + (possiblePositions.size() + 1)) % possiblePositions.size());		//This is for Gods and Titans randon number and dont get self
			staleStateRoll = 0;
		}
		else{
			staleStateRoll ++;
		}
	}

	earlyExitCount = 0;
	State _nextState = currentState; //needs had state or dequeue shuffler

	auto rd = std::random_device { };
	auto rng = std::default_random_engine { rd() };
	std::shuffle(std::begin(states), std::end(states), rng); //random shuffle and dont get last

	for (size_t i = 0; i < states.size(); i++) {
		State state = states[i];
		if (state.position == currentState.targetPosition) {
//        	_nextState = state;
			if (state.targetPosition == targetPosition) {
				_nextState = state;
				if (state.name != currentState.name) {
					_nextState = state;
					if (state.isActive == isActive) {
						_nextState = state;
						break;
					}
				}
			}
		}
	}
	return _nextState;
}

void StateMachine::getTempEarlyExits(std::vector<State> _earlyExits) {
	tempEarlyExits.clear();
	for (size_t i = 0; i < _earlyExits.size(); i++) {
		tempEarlyExits.push_back(_earlyExits[i]);
	}
}

bool StateMachine::getIsInit() {
	return init;
}

void StateMachine::updateSegment() {
	StateMachine::Segment _segment;

	if (init) {
		currentState = states[0];	//this is the starting animation
		lastTargetPosition = currentState.position;
		targetPosition = currentState.position;
		std::cout << "init2:" << currentState.name << "| EarlyExits0: " << tempEarlyExits.size() << std::endl;
		_segment.startTime = currentState.startTime;
		_segment.endTime = currentState.endTime;
		_segment.name = currentState.name;
		isExitingEarly = false;
		std::cout << "init :" << _segment.startTime << "end:" << _segment.endTime << std::endl;
		currentSegment = _segment;
		init = false;
		return;
//		return _segment;
	}

	if (isExitingEarly) {
		currentState = tempEarlyExits[0];
		getTempEarlyExits(currentState.earlyExits);
		std::cout << "NewState:" << currentState.name << "| EarlyExits0: " << tempEarlyExits.size() << std::endl;
		_segment.startTime = currentState.startTime;
		_segment.endTime = currentState.endTime;
		_segment.name = currentState.name;
		isExitingEarly = false;
		std::cout << "start:" << _segment.startTime << "end:" << _segment.endTime << std::endl;
		currentSegment = _segment;
		return;
//		return _segment;
	}

	if (tempEarlyExits.size() == size_t(0)) { //end of cycles, or no earlyExits in current clip
		currentState = updateState();
		getTempEarlyExits(currentState.earlyExits);
		std::cout << std::endl;
		std::cout << "NewState:" << currentState.name << "| EarlyExits1: "<< tempEarlyExits.size() << std::endl;
		_segment.startTime = currentState.startTime;
		_segment.endTime = currentState.endTime;
		_segment.name = currentState.name;

		if (tempEarlyExits.size() > size_t(0)) {//has early exits in selected clip
			_segment.startTime = currentState.startTime;
			_segment.endTime = tempEarlyExits[0].transitionFromParent;
			_segment.name = currentState.name;
			std::cout << currentState.name << "| EarlyExitsInThisClip: "<< tempEarlyExits.size() << std::endl;
//			tempEarlyExits.pop_front();		//without creates offset
		}
	} else {
		_segment.startTime = tempEarlyExits[0].transitionFromParent;// -1?notwork	+1 not work
		tempEarlyExits.pop_front();
		std::cout << currentState.name << "| EarlyExitsLeft: " << tempEarlyExits.size() << std::endl;

		if (tempEarlyExits.size() > size_t(0)) {		//playToEndOnLastCycle
			_segment.endTime = tempEarlyExits[0].transitionFromParent;//-1 not work
		} else {
			_segment.endTime = currentState.endTime;
			std::cout << "Playing:" << currentState.name << " to end"<< std::endl;
		}

		if (tempEarlyExits.size() > 0) {
			_segment.name = tempEarlyExits[0].parentAnimation;
		} else {
			_segment.name = currentState.name;
		}
//		tempEarlyExits.pop_front();
	}
	currentSegment = _segment;
	std::cout << "start:" << _segment.startTime << "end:" << _segment.endTime<< std::endl;
	return;
//	return _segment;
}

bool StateMachine::updateFrame(int _frame) {
	return true;
}

