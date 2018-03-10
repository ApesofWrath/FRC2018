/*
 * Scale.h
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

//#ifndef SCALE_H_
//#define SCALE_H_

#include <Autonomous.h>

class Scale : public Autonomous {
public:

	Scale(DriveController *dc_, Elevator *el_, Intake *in_) :
			Autonomous(dc_, el_, in_) {

	}

	void GenerateScale(bool left_scale, bool switch_, bool left_switch);
	void GenerateAddedSwitch(bool left); //will return a vector that will be added to the GenerateScale

	void RunStateMachine(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground); //place_switch

	int length;

};

//#endif /* SCALE_H_ */
