#include "Climber.h"

//Will need to be controlled. On way up, will be like a single-stage elevator

const double DOWN_SPEED = 0.15;
const double UP_SPEED = 0.325;

const int CAN_TALON_CLIMBER;

const int UNITS_PER_ROT = 4096;

const double MAX_OUTPUT = 1.0;
const double MIN_OUTPUT = -1.0;

Climber::Climber() {

	canTalonClimber = new TalonSRX(CAN_TALON_CLIMBER);

}
