#ifndef CARRIAGE_H
#define CARRIAGE_H

#include "Elevator.h"

class Carriage : public Elevator {

public:

	Carriage(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_, bool is_carr) : Elevator(pdp, elevator_profiler_, true) {

	}

private:


};

#endif
