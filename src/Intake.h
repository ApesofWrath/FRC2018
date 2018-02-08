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
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <list>
#include <DigitalInput.h>
#include <ElevatorMotionProfiler.h>

class Intake {
public:

	TalonSRX *talonIntake1, *talonIntake2, *talonIntakeArm;

	DigitalInput *hallEffectIntake; //for bottom //one top for elevator one bottom for elevator

	std::thread IntakeThread;

	bool is_arm_init = false; //is arm initialized

	const int UP_STATE_H = 0; //arm state machine
	const int MID_STATE_H = 1;
	const int DOWN_STATE_H = 2;
	const int STOP_ARM_STATE_H = 3;
	int intake_arm_state = UP_STATE_H;

	const int STOP_WHEEL_STATE_H = 0; //wheel state machine
	const int IN_STATE_H = 1;
	const int OUT_STATE_H = 2;
	int intake_wheel_state = STOP_WHEEL_STATE_H;

	Intake(PowerDistributionPanel *pdp, MotionProfiler *intake_profiler_);

	void In();
	void Out();
	void StopWheels();
	void StopArm();

	void Rotate(std::vector<std::vector<double> > ref_intake);
	double GetAngularVelocity();
	double GetAngularPosition();

	bool HaveCube();
	bool EncodersRunning();

	void SetVoltageIntake(double voltage_i);

	void ZeroEnc();
	void ManualArm(Joystick *joyOpArm);
	void ManualWheels(Joystick *joyOpWheels);

	void IntakeWheelStateMachine();
	void IntakeArmStateMachine();

	void StartIntakeThread();
	void EndIntakeThread();
	static void IntakeWrapper(Intake *in); //double *ref_in



};

#endif /* SRC_INTAKE_H_ */
