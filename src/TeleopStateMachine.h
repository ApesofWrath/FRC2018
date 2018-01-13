/*
 * TeleopStateMachine.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#ifndef SRC_TELEOPSTATEMACHINE_H_
#define SRC_TELEOPSTATEMACHINE_H_

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <Intake.h>
#include <Elevator.h>
//#include <DriveController.h>

class TeleopStateMachine {
public:

	TeleopStateMachine(Elevator *elevator_, Intake *intake_);
	void StateMachine();

	Intake *intake;
	Elevator *elevator;

	void Initialize();

};

#endif /* SRC_TELEOPSTATEMACHINE_H_ */
