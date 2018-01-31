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
#include <iostream>
//#include <DriveController.h>

class TeleopStateMachine {
public:

	TeleopStateMachine(Elevator *elevator_, Intake *intake_);
	void StateMachine(bool wait_for_button, bool intake_spin_in,
			bool intake_spin_out, bool intake_spin_stop, bool get_cube_ground,
			bool get_cube_station, bool post_intake, bool raise_to_switch, bool raise_to_scale,
			bool intake_arm_up, bool intake_arm_down, bool elevator_up,
			bool elevator_down);

	Intake *intake;
	Elevator *elevator;

	void Initialize();

};

#endif /* SRC_TELEOPSTATEMACHINE_H_ */
