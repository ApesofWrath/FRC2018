/*
 * AutonStateMachine.h
 *
 *  Created on: Mar 14, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONSTATEMACHINE_H_
#define SRC_AUTONSTATEMACHINE_H_

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <Intake.h>
#include <Elevator.h>
#include <iostream>
#include <Timer.h>
#include <thread>
#include <chrono>
#include <DriveController.h>

class AutonStateMachine {
public:

	AutonStateMachine(Elevator *elevator_, Intake *intake_,
			DriveController *drive_controller);

	void StateMachineAuton(bool wait_for_button, bool intake_spin_in,
			bool intake_spin_out, bool intake_spin_slow, bool intake_spin_stop,
			bool get_cube_ground, bool get_cube_station, bool post_intake,
			bool raise_to_switch, bool raise_to_scale, bool intake_arm_up,
			bool intake_arm_mid, bool intake_arm_down, bool elevator_up,
			bool elevator_mid, bool elevator_down,
			bool raise_to_scale_backwards);

	void Initialize();

	void StartAutonStateMachineThread(bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_slow,
			bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch,
			bool *raise_to_scale, bool *intake_arm_up, bool *intake_arm_mid,
			bool *intake_arm_down, bool *elevator_up, bool *elevator_mid,
			bool *elevator_down, bool *raise_to_scale_backwards);
	static void AutonStateMachineWrapper(AutonStateMachine *auton_state_machine,
			bool *wait_for_button, bool *intake_spin_in, bool *intake_spin_out,
			bool *intake_spin_slow, bool *intake_spin_stop,
			bool *get_cube_ground, bool *get_cube_station, bool *post_intake,
			bool *raise_to_switch, bool *raise_to_scale, bool *intake_arm_up,
			bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up,
			bool *elevator_mid, bool *elevator_down,
			bool *raise_to_scale_backwards);
	void EndAutonStateMachineThread();

private:


};

#endif /* SRC_AUTONSTATEMACHINE_H_ */
