/*
 * Switch.h
 *
 *  Created on: Feb 28, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSEQUENCES_SWITCH_CENTER_H_
#define SRC_AUTONSEQUENCES_SWITCH_CENTER_H_

#include <Autonomous.h>

class SwitchCenter : public Autonomous {
public:

	SwitchCenter(DriveController *dc, Elevator *el, Intake *in) : Autonomous(dc, el, in) {

	}

	void GenerateSwitch(bool left);

	void RunStateMachine(bool *place_switch);

	int length;

};

#endif /* SRC_AUTONSEQUENCES_SWITCH_H_ */
