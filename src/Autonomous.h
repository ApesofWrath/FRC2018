/*
 * Autonomous.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include <MotionProfiler.h>
#include <Elevator.h>
#include <Intake.h>
#include <DriveController.h>

class Autonomous {
public:

	Autonomous(DriveController *dc, Elevator *el, Intake *in);


};

#endif /* SRC_AUTONOMOUS_H_ */
