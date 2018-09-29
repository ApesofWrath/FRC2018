#ifndef MIDDLESTAGE_H
#define MIDDLESTAGE_H

#include "Elevator.h"

class MiddleStage : public Elevator {

public:

	MiddleStage(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_, bool is_carr) : Elevator(pdp, elevator_profiler_, false) {

	}

private:

};

#endif
