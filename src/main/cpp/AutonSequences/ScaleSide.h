/*
 * Scale.h
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSEQUENCES_SCALESIDE_H_
#define SRC_AUTONSEQUENCES_SCALESIDE_H_

#include "../Autonomous.h"

class ScaleSide : public Autonomous {
public:

	ScaleSide(DriveController *dc_, MiddleStage *mds_, Carriage *carr_, Intake *in_, AutonStateMachine *ausm) :
			Autonomous(dc_, mds_, carr_, in_, ausm) {

	}

	//robot ends up in a different spot after placing the first scale cube. it depends on if it started on the same side, or if it crossed over

	void GenerateSameScale(bool is_left, bool added_switch, bool added_scale); //only doing switch if the switch is on the same side as the scale (the side the robot starts on)
	void GenerateOppScale(bool left_start, bool added_switch, bool added_scale);

	void GenerateAddedSwitch(bool same_side, bool added_scale); //used for 2nd scale too
	void GenerateAddedScale(bool same_side);

	void RunStateMachineSameScaleSwitch(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground);
	void RunStateMachineOppScaleSwitch(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground);

	void RunStateMachineSameScaleScale(bool *place_scale_backwards, bool *get_cube_ground);
	void RunStateMachineOppScaleScale(bool *place_scale_backwards, bool *get_cube_ground);

	void RunStateMachineSameScale(bool *place_scale_backwards, bool *get_cube_ground);
	void RunStateMachineOppScale(bool *place_scale_backwards, bool *get_cube_ground);

	int length;

};

#endif /* SCALE_H_ */
