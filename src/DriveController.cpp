/*
 * DriveController.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: DriversStation
 */

#include <DriveController.h>


#define CORNELIUS 0

#if CORNELIUS
#else
#endif

using namespace std::chrono;

const int NUM_POINTS = 1500;
const int NUM_INDEX = 0; //?

//double drive_ref[NUM_INDEX]; is in mother

const int DRIVE_SLEEP_TIME = 0.00;
const double DRIVE_WAIT_TIME = 0.01; //seconds

//Timer *timerAuton = new Timer();

void DriveController::AutonWrapper(DriveController *driveController) {


}

void DriveController::StartAutonThread() {

	DriveController *dc = this;

	AutonThread = std::thread(&DriveController::AutonWrapper, dc);
	AutonThread.detach();

}

void DriveController::EndAutonThread() {

	AutonThread.~thread();

};
