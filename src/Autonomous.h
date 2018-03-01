/*
 * Autonomous.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include <Elevator.h>
#include <Intake.h>
#include <fstream>
#include <vector>
#include <list>
#include <DriveController.h>
#include <pathfinder.h>
#include <ElevatorMotionProfiler.h>

class Autonomous {
public:

	DriveController *drive_controller;
	Elevator *elevator_;
	Intake *intake_;

	Autonomous(DriveController *dc, Elevator *el, Intake *in);
	void AutonStateMachine();

};

#endif /* SRC_AUTONOMOUS_H_ */

