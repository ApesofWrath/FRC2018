/*
 * TaskManager.h
 *
 *  Created on: Mar 17, 2018
 *      Author: DriversStation
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <Intake.h>
#include <Elevator.h>
#include <iostream>
#include <Timer.h>
#include <thread>
#include <chrono>
#include <DriveController.h>
#include <TeleopStateMachine.h>
#include <AutonStateMachine.h>

class TaskManager {
public:

	TaskManager(TeleopStateMachine *tsm, AutonStateMachine *ausm, DriveController *dc, Elevator *el, Intake *in);

	void StartThread(bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
			bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up, bool *elevator_mid,
			bool *elevator_down,  bool *raise_to_scale_backwards, Joystick *JoyThrottle,
			Joystick *JoyWheel);
	static void ThreadWrapper(TaskManager *task_manager, Joystick *JoyThrottle,
			Joystick *JoyWheel, bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
			bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up, bool *elevator_mid,
			bool *elevator_down, bool *raise_to_scale_backwards);
	void EndThread();

};

#endif /* TASKMANAGER_H_ */
