#ifndef CARRIAGE_H
#define CARRIAGE_H

#include "Elevator.h"

class Carriage : public Elevator {

public:

	Carriage(ElevatorMotionProfiler *elevator_profiler_) : Elevator(elevator_profiler_, { {  0, 0 }, { 0, 0 } }, { { 0, 0 }, { 0, 0 } }, 0, 0, 0, 0,
						DOWN_POS_CARR, MID_POS_CARR, HPS_POS_CARR, UP_POS_CARR, 0, 0, "CARR", -20, 0) {

	}

private:


};

#endif
