/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>

#define CORNELIUS 1

#if CORNELIUS

#else

#endif

bool has_started_shoot = false;

//The naming convention for the auton subclasses are PLACE TO SCORE and STARTING POSITION

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in, AutonStateMachine *ausm) { //TODO: can now use inherited objects, take out middleperson functions

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;
	auton_state_machine = ausm;

	time_step_auton = dc->time_step_drive;
}

bool Autonomous::StartedShoot() { //has_started_shoot never resets to false

	if ((intake_->intake_wheel_state == intake_->SLOW_STATE_H
			|| intake_->intake_wheel_state == intake_->OUT_STATE_H)) {
		has_started_shoot = true;
	}

	return has_started_shoot;

}
