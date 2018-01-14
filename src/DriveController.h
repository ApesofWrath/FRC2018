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

	DriveController() : DriveControllerMother(30, 33, 36, 18, -1, true) {// change to int fl, int ml, int rl, int rr, int mr, int fr
			solenoidLeft = new DoubleSolenoid(0, 0, 1); //pcm can id, forward channel, reverse channel
			solenoidRight = new DoubleSolenoid(0, 0, 1);
	}

	DoubleSolenoid *solenoidLeft, *solenoidRight;

	void Switch();

};

#endif /* SRC_DRIVE_H_ */
