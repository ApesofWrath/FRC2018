/*
 * Elevator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: Apes of Wrath
 */

#ifndef SRC_DRIVECONTROLLER_H_
#define SRC_DRIVECONTROLLER_H_

#include <DriveControllerMother.h>

class DriveController : public DriveControllerMother {
public:

	//DriveController() : DriveControllerMother(18, 23, 30, 36, 22, 24, 21, 29, true) { //13 IS 29 //18 //22

	DriveController() : DriveControllerMother(18, 23, 30, 36, 24, 21, 22, 29, false) { //18, 23, 30, 36, 22, 24, 21, 29 //switch 22 and 24 //24 and 22 //WILL START IN HIGH


	}

};

#endif /* SRC_DRIVE_H_ */
