/*
 * Autonomous.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include "MiddleStage.h"
#include "Carriage.h"
#include "Intake.h"
#include <fstream>
#include <vector>
#include <list>
#include "DriveController.h"
#include <pathfinder.h>
#include "ElevatorMotionProfiler.h"
#include <thread>
#include <chrono>
#include <Timer.h>
#include "AutonStateMachine.h"

class Autonomous {
public:

	DriveController *drive_controller;
	MiddleStage *mds_;
	Carriage *carr_;
	Intake *intake_;
	AutonStateMachine *auton_state_machine;

	Autonomous(DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, AutonStateMachine *ausm);

	double time_step_auton;

	std::vector<int> zeroing_indeces; //can also add the profile refs here

};

#endif /* SRC_AUTONOMOUS_H_ */
