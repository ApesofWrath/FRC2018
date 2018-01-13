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
#include <Timer.h>

class Elevator {
public:

	Elevator();

	TalonSRX *talonElevator1, talonElevator2;

	std::thread ElevatorThread;

	const int DOWN_STATE_H = 0;
	const int UP_STATE_H = 1;
	int elevator_state = 0;

	void ElevatorStateMachine();
	void Move(double ref);

	void StartElevatorThread();
	void ElevatorWrapper(Elevator *el, double *ref);
	void EndElevatorThread();

};

#endif /* SRC_ELEVATOR_H_ */
