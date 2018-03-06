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

bool has_started_shoot = false;

std::thread AutonStateMachineThread;

Timer *autonTimer = new Timer();

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) { //TODO: can now use inherited objects, take out middleperson functions

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

bool Autonomous::IsLastDriveIndex() {

	return drive_controller->IsLastIndex();

}

int Autonomous::GetIndex() {

	return drive_controller->GetDriveIndex();

}

bool Autonomous::IsCubeReleased() {

	return intake_->ReleasedCube();

}

bool Autonomous::StartedShoot() { //has_started_shoot never resets to false

	if ((intake_->intake_wheel_state == intake_->SLOW_STATE_H
			|| intake_->intake_wheel_state == intake_->OUT_STATE_H)) {
		has_started_shoot = true;
	}
	if (has_started_shoot) {
		return true;
	} else {
		return false;
	}

}
