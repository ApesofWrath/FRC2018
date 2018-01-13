/*
 * DriveController.h
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */


#ifndef SRC_DRIVECONTROLLERMOTHER_H_
#define SRC_DRIVECONTROLLERMOTHER_H_

#include <WPILib.h>
#include <Joystick.h>
#include "AHRS.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <Timer.h>
#include "ctre/Phoenix.h"


class DriveControllerMother {
public:

	TalonSRX *canTalonFrontLeft, *canTalonFrontRight, *canTalonRearRight,
			*canTalonRearLeft, *canTalonKicker;

	AHRS *ahrs;

	//needs the CAN IDs of all the talons and whether or not this is a west coast or HDrive train, input -1 if no kicker (or really whatever you want since it wont be used)
	DriveControllerMother(int fl, int fr, int rl, int rr, int k, bool is_wc);

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
	void StopAll();
	void ZeroEncs();
	void ZeroI(bool StopMotors);

	//Funstion to fill the profile points vector for autonomous
	void SetRefs(std::vector<std::vector<double>> profile);

	//Wrapper Functions
	//virtual void AutonWrapper(DriveControllerMother *driveController) = 0; //needs to iterate through the profile and update drive_refs from auton_profile
	static void TeleopWrapper(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc, DriveControllerMother *driveController);


	//Thread Functions
	void StartTeleopThreads(Joystick *JoyThrottle, Joystick *JoyWheel,
			bool *is_heading, bool *is_vision, bool *is_fc);
	void StartAutonThreads();
	void EndTeleopThreads();
	void EndAutonThreads();

};


#endif /* SRC_DRIVECONTROLLER_H_ */
