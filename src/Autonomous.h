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
#include <thread>
#include <chrono>
#include <Timer.h>

class Autonomous {
public:

	Autonomous(DriveController *dc, Elevator *el, Intake *in);

	void FillProfile(std::vector<std::vector<double> > pathfinder_refs);

	double GetLeftPos();
	double GetRightPos();

};

#endif /* SRC_AUTONOMOUS_H_ */

