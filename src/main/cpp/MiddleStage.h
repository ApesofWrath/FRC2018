#ifndef MIDDLESTAGE_H
#define MIDDLESTAGE_H

#include "Elevator.h"

class MiddleStage : public Elevator {

public:

	MiddleStage(ElevatorMotionProfiler *elevator_profiler_) : Elevator(elevator_profiler_, { {  27.89, 4.12 }, { 25.90, 1.57 } }, { { 27.89, 4.12 }, { 22.11, 1.75 } },
							(20.0 / 1.0), 0.4, 0.0381, 0.75, DOWN_POS_MDS, MID_POS_MDS, HPS_POS_MDS, UP_POS_MDS, 2, 1, "MDS", -5, -2) {

	}

private:

};

#endif
