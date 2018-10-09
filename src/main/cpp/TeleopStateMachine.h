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
#include "Intake.h"
#include "MiddleStage.h"
#include "Climber.h"
#include "Carriage.h"
#include <iostream>
#include <Timer.h>
#include <thread>
#include <chrono>
#include "DriveController.h"

class TeleopStateMachine {
public:

	TeleopStateMachine(MiddleStage *mds_, Carriage *carr_, Intake *intake_, Climber *climber_, DriveController *drive_controller);

	void StateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_slow, bool intake_spin_med,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool pop_switch, bool raise_to_scale_low,
		bool raise_to_scale_mid, bool raise_to_scale_high, bool intake_arm_up,
		bool intake_arm_mid, bool intake_arm_down, bool mds_up, bool mds_mid, bool mds_down, bool open_intake, bool close_intake,
		bool carr_down, bool carr_mid, bool carr_up, bool raise_to_scale_backwards, Joystick *joySlider);

};

#endif /* SRC_TELEOPSTATEMACHINE_H_ */
