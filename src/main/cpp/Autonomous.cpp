/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include "Autonomous.h"

bool has_started_shoot = false;

//The naming convention for the auton subclasses are PLACE TO SCORE and STARTING POSITION

Autonomous::Autonomous(DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, AutonStateMachine *ausm) { //TODO: can now use inherited objects, take out middleperson functions

	drive_controller = dc;
	mds_ = mds;
	carr_ = carr;
	intake_ = in;
	auton_state_machine = ausm;

	time_step_auton = drive_controller->time_step_drive;
}
