/*
 * Scale.h
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

//#ifndef SRC_AUTONSEQUENCES_SCALESIDE_H_
//#define SRC_AUTONSEQUENCES_SCALESIDE_H_

#include <Autonomous.h>

class ScaleSide : public Autonomous {
public:

	ScaleSide(DriveController *dc_, Elevator *el_, Intake *in_, AutonStateMachine *ausm) :
			Autonomous(dc_, el_, in_, ausm) {

	}

	void GenerateScale(bool left_start, bool switch_, bool left_switch, bool added_scale, bool left_added_scale);
	void GenerateCrossedScale(bool left_start, bool switch_, bool left_switch, bool added_scale, bool left_added_scale);

	void GenerateAddedSwitch(bool left_switch, bool added_scale, bool added_switch); //used for 2nd scale
	void GenerateAddedScale(bool left);

	void RunStateMachineScaleSwitch(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground);
	void RunStateMachineScaleScale(bool *place_scale_backwards, bool *get_cube_ground);
	void RunStateMachineScaleOnly(bool *place_scale_backwards, bool *get_cube_ground);

	int length;

};

//#endif /* SCALE_H_ */
