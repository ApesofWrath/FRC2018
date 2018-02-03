/*
 * Elevator.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */


#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <chrono>
#include <Timer.h>
#include <thread>
#include <MotionProfiler.h>

class Elevator {
public:

	Elevator();

	MotionProfiler *elevator_profiler;

	TalonSRX *talonElevator1, *talonElevator2;

	std::thread IntakeThread;

	const int DOWN_STATE_H = 0;
	const int MID_STATE_H = 1;
	const int UP_STATE_H = 2;
	const int STOP_STATE_H = 3;
	int elevator_state = DOWN_STATE_H;

	void ElevatorStateMachine();
	void Move(double ref_elevator[2][1]);
	void StopElevator();

	double GetPosition();

	bool EncodersRunning();
	void ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_, MotionProfiler *elevator_profiler);
	void EndElevatorThread();

};

#endif /* SRC_ELEVATOR_H_ */
