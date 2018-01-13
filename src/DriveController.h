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

class DriveController : public DriveControllerMother {
public:

	DriveController() : DriveControllerMother(30, 33, 36, 18, -1, true) {

	}

};

#endif /* SRC_DRIVE_H_ */
