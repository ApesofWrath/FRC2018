/*
 * Elevator.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */


#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include "ElevatorMotionProfiler.h"
#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <chrono>
#include <Timer.h>
#include <thread>

#define CORNELIUS_EL 0

#if CORNELIUS_EL
//down pos to 0.02
#else
//down pos to 0.00
#endif

class Elevator {
public:

	Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_);

	TalonSRX *talonElevator1, *talonElevator2;

	DigitalInput *hallEffectTop;
	DigitalInput *hallEffectBottom;

	std::thread ElevatorThread;

	PowerDistributionPanel *pdp_e;

	bool is_elevator_init = false;
	bool keep_elevator_up = false; //used in intake for safety

	double goal_vel_e = 0.0;

	int zeroing_counter_e = 0;

	const int INIT_STATE_E_H = 0;
	const int DOWN_STATE_E_H = 1;
	const int MID_STATE_E_H = 2;
	const int UP_STATE_E_H = 3;
	const int STOP_STATE_E_H = 4;
	const int SWITCH_STATE_E_H = 5;
	int elevator_state = INIT_STATE_E_H;

	const double DOWN_POS_E = 0.009; //lowest possible height is 0.008
	const double MID_POS_E = 0.668;
	const double SWITCH_POS_E = 0.5;
	const double UP_POS_E = 0.90; //reaches only 0.88 according to the elevator's own readings, but is outputting 1.4 volts  //0.89
	//el gains could be a bit more aggressive

	void InitializeElevator();

	void ElevatorStateMachine();
	void Move();
	void StopElevator();

	double GetVoltageElevator();
	void SetVoltageElevator(double elevator_voltage);

	void ManualElevator(Joystick *joyOpElev);

	void SetZeroOffsetElevator();

	double GetElevatorPosition();
	double GetElevatorVelocity();

	bool IsAtBottomElevator();
	bool IsAtTopElevator();
	bool ElevatorEncodersRunning();

	bool ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_);
	void EndElevatorThread();

};

#endif /* SRC_ELEVATOR_H_ */
