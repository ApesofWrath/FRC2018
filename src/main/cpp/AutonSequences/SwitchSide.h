/*
 * SwitchSide.h
 *
 *  Created on: Mar 8, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSEQUENCES_SWITCHSIDE_H_
#define SRC_AUTONSEQUENCES_SWITCHSIDE_H_

#include "../Autonomous.h"

class SwitchSide : public Autonomous {
public:

	SwitchSide(DriveController *dc, Elevator *el, Intake *in, AutonStateMachine *ausm) : Autonomous(dc, el, in, ausm) {

	}

	void GenerateSwitchSide(bool left, bool added_switch);

	void RunStateMachineSide(bool *place_switch);

	void ForwardSwitch(bool left, bool added_switch);

	void GetAddedSwitch(bool left);

	int length;

};

#endif /* SRC_AUTONSEQUENCES_SWITCH_H_ */
