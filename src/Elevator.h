/*
 * Elevator.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */


#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include <ElevatorMotionProfiler.h>
#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <chrono>
#include <Timer.h>
#include <thread>

class Elevator {
public:

	Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_);

	TalonSRX *talonElevator1, *talonElevator2;

	DigitalInput *hallEffectTop;
	DigitalInput *hallEffectBottom;

	std::thread ElevatorThread;

	PowerDistributionPanel *pdp_e;

	bool is_elevator_init = false;

	double goal_vel_e = 0.0;

	int zeroing_counter_e = 0;

	const int INIT_STATE_E_H = 0;
	const int DOWN_STATE_E_H = 1;
	const int MID_STATE_E_H = 2;
	const int UP_STATE_E_H = 3;
	const int STOP_STATE_E_H = 4;
	const int SWITCH_STATE_E_H = 5;
	int elevator_state = INIT_STATE_E_H;

	const double DOWN_POS_E = 0.0; //starting pos
	const double MID_POS_E = 0.668;
	const double SWITCH_POS_E = 0.5;

	const double UP_POS_E = 0.89;

	void InitializeElevator();

	void ElevatorStateMachine();
	void Move(std::vector<std::vector<double> > ref_elevator);
	void StopElevator();

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
