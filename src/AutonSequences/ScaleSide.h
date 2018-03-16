/*
 * Scale.h
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>

class ScaleSide : public Autonomous {
public:

	ScaleSide(DriveController *dc_, Elevator *el_, Intake *in_) :
			Autonomous(dc_, el_, in_) {

	}

	void GenerateScale(bool left_scale, bool switch_, bool left_switch);

	//TODO: transition to these functions
	void GenerateScale(bool left_scale);
	void GenerateScale(bool left_scale, bool left_switch);
	void GenerateScale(bool left_scale, bool left_scale);

	void GenerateAddedSwitch(bool left);
	void GenerateAddedScale(bool left);

	//TODO: 2-scale state machine function
	void RunStateMachine(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground);

	int length;

};

//#endif /* SCALE_H_ */
