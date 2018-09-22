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
	bool is_carr_;

public:

	Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_, bool is_carr);

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

	const double DOWN_POS_MDS = 0.005;
	const double MID_POS_MDS = 0.668;
	const double HPS_POS_MDS = 0.5;
	const double UP_POS_MDS = 0.89;

	const double DOWN_POS_CARR = 0.01;
	const double MID_POS_CARR = 0.01;
	const double HPS_POS_CARR = 0.01;
	const double UP_POS_CARR = 0.01;

	const double SAFE_CARR_HEIGHT = 0.2; //TODO: actually determine this //carr height at which ms can start moving down

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
	bool IsAtPos(double target_pos);

	bool ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_);
	void EndElevatorThread();

private:

  void SetVoltage(double elevator_voltage);
	void SetZeroOffset();

};

#endif /* SRC_ELEVATOR_H_ */
