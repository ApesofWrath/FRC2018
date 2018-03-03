/*
 * DriveController.h
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */


#ifndef SRC_DRIVECONTROLLERMOTHER_H_
#define SRC_DRIVECONTROLLERMOTHER_H_

#include <iostream>
#include <WPILib.h>
#include <Joystick.h>
#include "AHRS.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <Timer.h>
#include "ctre/Phoenix.h"
#include <pathfinder.h>

class DriveControllerMother {
public:

	TalonSRX *canTalonLeft1, *canTalonLeft2, *canTalonLeft3, *canTalonLeft4, *canTalonRight1, *canTalonRight2,
			*canTalonRight3, *canTalonRight4, *canTalonKicker; //for 4 talons: 1 is front right, 2 is back right, 3 is front left, 4 is back left

	DoubleSolenoid *solenoid;

	std::thread DriveThread;
	//std::thread AutonThread;

	AHRS *ahrs;

	//needs the CAN IDs of all the talons and whether or not this is a west coast or HDrive train, input -1 if no kicker (or really whatever you want since it wont be used)
	DriveControllerMother(int fl, int fr, int rl, int rr, int k, bool is_wc, bool start_low); //for HDrive or West Coast with 4 total talons
	DriveControllerMother(int l1, int l2, int l3, int l4, int r1, int r2, int r3, int r4, bool start_low); //for West Coast with 8 total talons

	void ShiftUp();
	void ShiftDown();
	void SetGainsHigh();
	void SetGainsLow();
	void AutoShift();

	//Driving Operators
	void TeleopHDrive(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_fc); //creates velocity references set by joysticks, for HDrive Train
	void TeleopWCDrive(Joystick *JoyThrottle, Joystick *JoyWheel); //creates velocity references based on joysticks, for normal west coast drive train
	void AutonDrive(); //makes velocity references based on motion profiles
	void RotationController(Joystick *JoyWheel);
	void Controller(double ref_kick, double ref_right, double ref_left,
			double ref_yaw, double k_p_right, double k_p_left, double k_p_kick,
			double k_p_yaw, double k_d_yaw, double k_d_right, double k_d_left,
			double k_d_kick, double target_vel_left, double target_vel_right,
			double target_vel_kick); //The final controller, will take the references set by either teleop or auton drive function

	//Motor Functions
	void ZeroAll(bool stop_motors);
	void StopAll();
	void ZeroEncs();
	void ZeroYaw();
	void ZeroI();

	double GetLeftPosition();
	double GetRightPosition();

	void SetMaxRpm(double rpm);
	double GetMaxRpm();

	//Funstion to fill the profile points vector for autonomous
	void SetRefs(std::vector<std::vector<double>> profile);

	//Wrapper Functions
	static void DriveWrapper(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc, DriveControllerMother *driveController);
	//static void AutonWrapper(DriveControllerMother *driveController);

	//Auton functions for threads are in derived class

	//Thread Functions
	void StartDriveThreads(Joystick *JoyThrottle, Joystick *JoyWheel,
			bool *is_heading, bool *is_vision, bool *is_fc);
	void EndDriveThreads();
	//void StartAutonThreads();
	//void EndAutonThreads();

	//AutonThread functions for use in the daughter class
	void UpdateIndex();
	void ResetIndex();


};


#endif /* SRC_DRIVECONTROLLER_H_ */
