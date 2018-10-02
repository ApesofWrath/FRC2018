#ifndef MIDDLESTAGE_H
#define MIDDLESTAGE_H

#include "Elevator.h"

class MiddleStage : public Elevator {

public:

	MiddleStage(ElevatorMotionProfiler *elevator_profiler_) : Elevator(elevator_profiler_, { {  27.89, 4.12 }, { 0.0, 0.0 } }, { { 27.89, 4.12 }, { 0.0, 0.0 } },
							DOWN_POS_MDS, MID_POS_MDS, HPS_POS_MDS, UP_POS_MDS, (20.0 / 1.0), 0.4, 0.0381, 0.75, 2, 1, "MDS", 0, 0) {
								//ElevatorMotionProfiler *elevator_profiler_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_,
								//	double down_pos_, double mid_pos_, double hps_pos_, double up_pos_, double G_e_, double ff_percent_e_, double PULLEY_DIAMETER_,
								//	double friction_loss_, int TOP_HALL_, int BOT_HALL_, std::string elev_type_, int TALON_ID_1_, int TALON_ID_2_
	}

private:

};

#endif
