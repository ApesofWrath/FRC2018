/*
 * Elevator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: Apes of Wrath
 */

#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <DriveControllerMother.h>
#include <DoubleSolenoid.h>

class DriveController : public DriveControllerMother {
public:

	DriveController() : DriveControllerMother(23, 18, 30, 36, 29, 24, 21, 22) { //CHECK 29isright

	}

};

#endif /* SRC_DRIVE_H_ */
