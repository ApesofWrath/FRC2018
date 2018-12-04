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
#include "Intake.h"
#include "MiddleStage.h"
#include "Carriage.h"
#include "Climber.h"
#include <iostream>
#include <Timer.h>
#include <thread>
#include <chrono>
#include "DriveController.h"
#include "TeleopStateMachine.h"
#include "AutonStateMachine.h"

class TaskManager {
public:

	TaskManager(TeleopStateMachine *tsm, AutonStateMachine *ausm, DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, Climber *climb, double thread_time);

	double thread_time_step;

	void StartThread(bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_med, bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *pop_switch, bool *raise_to_scale_low, bool *raise_to_scale_mid, bool *raise_to_scale_high,
			bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down,bool *mds_up, bool *mds_mid, bool *mds_down, bool *open_intake, bool *close_intake, bool *carr_up,
			bool *carr_mid, bool *carr_down, bool *raise_to_scale_backwards, bool *climb_button, Joystick *JoyThrottle,
			Joystick *JoyWheel, Joystick *JoySlider, bool *is_heading);

	static void ThreadWrapper(TaskManager *task_manager, Joystick *JoyThrottle,
			Joystick *JoyWheel, Joystick *JoySlider, bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_med, bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *pop_switch, bool *raise_to_scale_low, bool *raise_to_scale_mid, bool *raise_to_scale_high,
			bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *mds_up, bool *mds_mid, bool *mds_down, bool *open_intake, bool *close_intake, bool *carr_up,
			bool *carr_mid, bool *carr_down, bool *raise_to_scale_backwards, bool *climb_button, bool *is_heading);

	void EndThread();

};

#endif /* TASKMANAGER_H_ */
