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

	ScaleSide(DriveController *dc_, Elevator *el_, Intake *in_) :
			Autonomous(dc_, el_, in_) {

	}

	void GenerateScale(bool left_scale, bool switch_, bool left_switch, bool added_scale, bool left_added_scale);

	//TODO: transition to these functions
//	void GenerateScale(bool left_scale);
//	void GenerateScale(bool left_scale, bool left_switch);
//	void GenerateScale(bool left_scale, bool left_scale);

	void GenerateAddedSwitch(bool left_switch, bool added_scale, bool added_switch); //used for 2nd scale
	void GenerateAddedScale(bool left);

	//TODO: 2-scale state machine function
	void RunStateMachineScaleSwitch(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground);
	void RunStateMachineScaleScale(bool *place_scale_backwards, bool *get_cube_ground);

	int length;

};

//#endif /* SCALE_H_ */
