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

	bool is_elevator_init = false;

	const int DOWN_STATE_E_H = 0;
	const int MID_STATE_E_H = 1;
	const int UP_STATE_E_H = 2;
	const int STOP_STATE_E_H = 3;
	int elevator_state = DOWN_STATE_E_H;

	void ElevatorStateMachine();
	void Move(std::vector<std::vector<double> > ref_elevator);
	void StopElevator();

	void SetVoltageElevator(double elevator_voltage);

	void ManualElevator(Joystick *joyOpElev);

	double GetElevatorPosition();
	double GetElevatorVelocity();

	bool ElevatorEncodersRunning();
	void ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_);
	void EndElevatorThread();

};

#endif /* SRC_ELEVATOR_H_ */
