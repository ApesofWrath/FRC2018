/*
 * Switch.h
 *
 *  Created on: Feb 28, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSEQUENCES_SWITCH_H_
#define SRC_AUTONSEQUENCES_SWITCH_H_

#include <Autonomous.h>

class Switch : public Autonomous {
public:

	Switch(DriveController *dc, Elevator *el, Intake *in) : Autonomous(dc, el, in) {

	}

	void GenerateSwitch(bool left);

	void RunStateMachine();

};

#endif /* SRC_AUTONSEQUENCES_SWITCH_H_ */
