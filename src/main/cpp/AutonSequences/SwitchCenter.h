/*
 * Switch.h
 *
 *  Created on: Feb 28, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSEQUENCES_SWITCH_CENTER_H_
#define SRC_AUTONSEQUENCES_SWITCH_CENTER_H_

#include "../Autonomous.h"

class SwitchCenter : public Autonomous {
public:

	int length;

	SwitchCenter(DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, AutonStateMachine *ausm) : Autonomous(dc, mds, carr, in, ausm) {

	}

	void GenerateSwitch(bool left, bool added_switch);
	void MoveToAddedSwitch(bool left); //back to starting position, for moving back on the x
	void GetAddedSwitch(bool left); //forward
	void BackUp(bool left); //back up first, so that we have forward room to move on the x, to place on the switch again
	void PlaceAddedSwitch(bool left);

	void RunStateMachine(bool *place_switch);

	void RunStateMachineTwo(bool *place_switch, bool *get_cube_ground);

};

#endif /* SRC_AUTONSEQUENCES_SWITCH_H_ */
