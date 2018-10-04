#ifndef CARRIAGE_H
#define CARRIAGE_H

#include "Elevator.h"

class Carriage : public Elevator {

public:

	Carriage(ElevatorMotionProfiler *elevator_profiler_) : Elevator(elevator_profiler_, { {  0, 0 }, { 0, 0 } }, { { 0, 0 }, { 0, 0 } },
						DOWN_POS_CARR, MID_POS_CARR, HPS_POS_CARR, UP_POS_CARR, (15.0 / 1.0), 0.4, 0.04191, 0.75, 0, 0, "CARR", 0, -1) {
							//ElevatorMotionProfiler *elevator_profiler_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_,
							//	double down_pos_, double mid_pos_, double hps_pos_, double up_pos_, double G_e_, double ff_percent_e_, double PULLEY_DIAMETER_,
							//	double friction_loss_, int TOP_HALL_, int BOT_HALL_, std::string elev_type_, int TALON_ID_1_, int TALON_ID_2_
	}

private:


};

#endif
