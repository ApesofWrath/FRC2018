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
//#include <thread>
#include <DriveController.h>
#include <ElevatorMotionProfiler.h>

class Autonomous {
public:

	Autonomous(DriveController *dc, Elevator *el, Intake *in);
	void RunAuton();


};

#endif /* SRC_AUTONOMOUS_H_ */
