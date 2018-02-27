/*
 * DriveForward.h
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#ifndef SRC_DRIVEFORWARD_H_
#define SRC_DRIVEFORWARD_H_

//#include <WPILib.h>
//#include <ctre/Phoenix.h>
//#include <iostream>
#include <Autonomous.h>
//#include <DriveControllerMother.h>

class DriveForward : public Autonomous {
public:

	DriveForward(DriveController *dc, Elevator *el, Intake *in) : Autonomous(dc, el, in) { //take objects from robot.cpp, place in drive_forward, put and initialize in autonomous

	}

	// DriveForward();
//	virtual ~DriveForward();
	void Generate();
	void SetDriveRefs();
	bool FollowPath();
	void ConfigureEncoders();
	void StopFollowing();
private:
	//ALL IN FEET PLEASE!!!
	int POINT_LENGTH = 2;
	const double TIMESTEP = 0.02;
	const double MAX_VEL = 18;
	const double MAX_ACCEL = 12;
	const double MAX_JERK = 60;
	const int TICKS_PER_REV = 26214;
	const double WHEEL_CIRCUMFERENCE = 0.65449867893738;
//	const double K_P = 1;
//	const double K_I = 0.0;
//	const double K_D = .15;
//	const double K_V = .06; //.66;
//	const double K_A = 0.0856;
//	const double K_T = .35;
	double WHEELBASE_WIDTH;
	double WHEELBASE_LENGTH;
	TrajectoryCandidate candidate;
	Segment* trajectory;
	Segment* lTraj;
	Segment* rTraj;
//	Segment* blTraj;
//	Segment* brTraj;
	int length;
//	std::shared_ptr<std::vector<SwerveModule>> modules;
//	EncoderFollower* flFollower = (EncoderFollower*) malloc(
//			sizeof(EncoderFollower));
//	EncoderFollower* frFollower = (EncoderFollower*) malloc(
//			sizeof(EncoderFollower));
//	EncoderFollower* blFollower = (EncoderFollower*) malloc(
//			sizeof(EncoderFollower));
//	EncoderFollower* brFollower = (EncoderFollower*) malloc(
//			sizeof(EncoderFollower));
//	EncoderConfig flconfig;
//	EncoderConfig frconfig;
//	EncoderConfig blconfig;
//	EncoderConfig brconfig;
};

#endif /* SRC_DRIVEFORWARD_H_ */
