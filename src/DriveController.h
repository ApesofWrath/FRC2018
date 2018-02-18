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

	std::thread AutonThread;

	DriveController() : DriveControllerMother(18, 23, 30, 36, 22, 24, 21, 29, true) { //13 IS 29 //18 //22

	}

	static void AutonWrapper(DriveController *driveController);
	void StartAutonThread();
	void EndAutonThread();

};

#endif /* SRC_DRIVE_H_ */
