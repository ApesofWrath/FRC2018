#include "Elevator.h"

class Carriage : public Elevator {

public:

	const double DOWN_POS = 0.01;
	const double MID_POS = 0.01;
	const double HPS_POS = 0.01;
	const double UP_POS = 0.01;

	Carriage(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_) : Elevator(pdp, elevator_profiler_, true) {

	}

private:


};
