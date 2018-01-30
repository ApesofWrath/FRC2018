/*
 * Intake.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */


#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <Timer.h>
#include <MotionProfiler.h>
#include <thread>
#include <chrono>

class Intake {
public:

	TalonSRX *talonIntake1, *talonIntake2, *talonIntakeArm;

	const int UP_STATE_H = 0;
	const int MID_STATE_H = 1;
	const int DOWN_STATE_H = 2;
	const int STOP_ARM_STATE_H = 3;
	int intake_arm_state = UP_STATE_H;

	const int STOP_WHEEL_STATE_H = 0;
	const int IN_STATE_H = 1;
	const int OUT_STATE_H = 2;
	int intake_wheel_state = STOP_WHEEL_STATE_H;

	Intake();

	void In();
	void Out();
	void StopWheels();
	void StopArm();

	int intake_index = 0;

	void SetIndex(int index);
	int GetIndex();

	void Rotate(double ref_intake[2][1]);
	double GetAngularVelocity();
	double GetAngularPosition();

	bool HaveCube();
	bool EncodersRunning();

	void IntakeWheelStateMachine();
	void IntakeArmStateMachine();

	void StartIntakeThread();
	void EndIntakeThread();
	static void IntakeWrapper(Intake *in, double *profile);



};

#endif /* SRC_INTAKE_H_ */
