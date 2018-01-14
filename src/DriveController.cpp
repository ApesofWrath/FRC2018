/*
 * DriveController.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: DriversStation
 */

#include <DriveController.h>

void DriveController::Switch() {

	solenoidLeft->Set(DoubleSolenoid::Value::kForward);
	solenoidRight->Set(DoubleSolenoid::Value::kForward);



}
