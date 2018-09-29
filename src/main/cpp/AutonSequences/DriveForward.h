/*
 * DriveForward.h
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#ifndef SRC_DRIVEFORWARD_H_
#define SRC_DRIVEFORWARD_H_

#include "../Autonomous.h"

class DriveForward : public Autonomous {
public:

	DriveForward(DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, AutonStateMachine *ausm) : Autonomous(dc, mds, carr, in, ausm) { //take objects from robot.cpp, place in drive_forward, put and initialize in autonomous

	}

	//int length;

	void GenerateForward(bool forward);

};

#endif /* SRC_DRIVEFORWARD_H_ */
