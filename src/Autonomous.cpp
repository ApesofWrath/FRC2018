/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>

#define CORNELIUS 0

#if CORNELIUS

#else

#endif

using namespace std::chrono;

const int NUM_POINTS = 1500; //every 10ms
const int NUM_INDEX = 10; //change this

double refs[NUM_POINTS][NUM_INDEX];

std::thread AutonStateMachineThread;

Timer *autonTimer = new Timer();

DriveController *drive_controller;
Elevator *elevator_;
Intake *intake_;

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;
}

void Autonomous::FillProfile(
		std::vector<std::vector<double> > pathfinder_refs) {

	drive_controller->SetRefs(pathfinder_refs);

}

double Autonomous::GetLeftPos() {

	return drive_controller->GetLeftPosition();

}

double Autonomous::GetRightPos() {

	return drive_controller->GetRightPosition();

}
