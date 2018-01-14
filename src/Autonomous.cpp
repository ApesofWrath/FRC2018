/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>

DriveController *drive_controller;
Elevator *elevator_;
Intake *intake_;

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;

}


