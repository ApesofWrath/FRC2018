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
#include <string>

class Elevator {

	double down_pos, mid_pos, hps_pos, up_pos;

public:

	Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_); //mds
	Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_, Elevator *mds_);

	TalonSRX *talonElevator1, *talonElevator2;

	DigitalInput *hallEffectTop, *hallEffectBottom;

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
	const int HPS_STATE_E_H = 5; //human player station
	int elevator_state = INIT_STATE_E_H;

	void InitializeElevator(); //should be able to initialize carr/ms at same time

	void ElevatorStateMachine(); //same for both
	void Move();
	void StopElevator();

	double GetVoltageElevator();

	void ManualElevator(Joystick *joyOpElev);

	double GetElevatorPosition();
	double GetElevatorVelocity();

	bool IsAtBottomElevator();
	bool IsAtTopElevator();
	bool ElevatorEncodersRunning();

	bool ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_);
	void EndElevatorThread();

private:

  void SetVoltage(double elevator_voltage);
	void SetZeroOffset();

};

#endif /* SRC_ELEVATOR_H_ */
