/*
 * Elevator.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Elevator.h>
#include "ctre/Phoenix.h"

const int DOWN_STATE = 0;
const int UP_STATE = 1;
int elevator_state = 0;

const double ELEVATOR_SLEEP_TIME = 0.0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

double ref_;

const double DOWN_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

Timer *elevatorTimer = new Timer();

Elevator::Elevator() {

	talonElevator1 = new TalonSRX(0);
	talonElevator2 = new TalonSRX(0);

	talonElevator2->Set(ControlMode::Follower, 0); //TODO: figure out why this gives an error

}

void Elevator::Move(double ref) {

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE:
		ref_ = DOWN_ANGLE;
		break;

	case UP_STATE:
		ref_ = UP_ANGLE;
		break;

	}
}

void Elevator::StartElevatorThread() {

	Elevator *el = this;
	ElevatorThread = std::thread(&Elevator::ElevatorWrapper(), el, &ref_);
	ElevatorThread.detach();

}

void Elevator::ElevatorWrapper(Elevator *el, double *ref) {

	elevatorTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(ELEVATOR_SLEEP_TIME));

				if (elevatorTimer->HasPeriodPassed(ELEVATOR_WAIT_TIME)) {

					elevatorTimer->Reset();
					el->Move(*ref);

				}
		}
	}

}

void Elevator::EndElevatorThread() {

	ElevatorThread.~thread();

}
