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
#include "IntakeMotionProfiler.h"
#include "Elevator.h"
#include <iostream>
#include <fstream>
#include "AlgLib/fasttransforms.h"
#include "AlgLib/ap.h"

class Intake {
public:

	TalonSRX *talonIntake1, *talonIntake2, *talonIntakeArm;
	DigitalInput *hallEffectIntake; //for bottom

	std::thread IntakeThread;

	std::ofstream currents_file;

	const double elevator_safety_position = 0.85;
	const double INTAKE_BACKWARDS_SOFT_LIMIT = 2.05;
	int zeroing_counter_i = 0;

	bool is_init_intake = false; //is arm initialized

	const int SLOW_SCALE = 0; //releasedcube()
	const int SWITCH = 1;
	const int BACK = 2;
	const int SLOW_BACK = 3;
	const int SCALE = 4;

	const int INIT_STATE_H = 0;
	const int UP_STATE_H = 1; //arm state machine
	const int MID_STATE_H = 2;
	const int DOWN_STATE_H = 3;
	const int STOP_ARM_STATE_H = 4;
	const int SWITCH_STATE_H = 5; //not used
	const int SWITCH_BACK_SHOT_STATE_H = 6; //new for the flippy back arm
	int intake_arm_state = INIT_STATE_H;

	const int STOP_WHEEL_STATE_H = 0; //wheel state machine
	const int IN_STATE_H = 1;
	const int OUT_STATE_H = 2;
	const int SLOW_STATE_H = 3;
	const int SLOW_SCALE_STATE_H = 4;
	const int POP_SWITCH_STATE_H = 5;
	int intake_wheel_state = STOP_WHEEL_STATE_H;

	const double DOWN_ANGLE = 0.0; //instead of changing the offset
	const double MID_ANGLE = 0.55;
	const double SWITCH_ANGLE = 1.0; //safety for elev
	const double UP_ANGLE = 1.35; //starting pos -> also the shooting position 1.3
	const double BACK_SHOT_ANGLE = 2.2; // 120 ish degrees for the flippy back arm shot 2.0

	Intake(PowerDistributionPanel *pdp, IntakeMotionProfiler *intake_profiler, Elevator *el_);

	void InitializeIntake();
	void SetStartingPos(double start);

	double FindMaximum(alglib::real_1d_array corr);

	void EnableCurrentLimits();
	void EnableIntakingCurrentLimits();
	void DisableCurrentLimits();

	void In();
	void Out();
	void Slow();
	void SlowScale();
	void StopWheels();
	void StopArm();
	void Pop();

	void Rotate();
	double GetAngularVelocity();
	double GetAngularPosition();

	bool IsAtBottomIntake();
	bool HaveCube();
	bool ReleasedCube(int shot_type);
	bool EncodersRunning();

	void SetVoltageIntake(double voltage_i);

	void SetZeroOffset();

	bool ZeroEnc();
	void ManualArm(Joystick *joyOpArm);
	void ManualWheels(Joystick *joyOpWheels);

	bool IsAtAngle(double target_ang);

	//std::vector<std::vector<double> > GetNextRef();

	void IntakeWheelStateMachine();
	void IntakeArmStateMachine();

	void StartIntakeThread();
	void EndIntakeThread();
	static void IntakeWrapper(Intake *in);

};

#endif /* SRC_INTAKE_H_ */
