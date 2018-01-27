/*
 * Elevator.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Elevator.h>
//#include "ctre/Phoenix.h"
#include <WPILib.h>

const int DOWN_STATE = 0;
const int UP_STATE = 1;
const int STOP_STATE = 2;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

double ref_elevator;

const double DOWN_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

Timer *elevatorTimer = new Timer();

Elevator::Elevator() {

	talonElevator1 = new TalonSRX(8);
	talonElevator2 = new TalonSRX(9);

	talonElevator2->Set(ControlMode::Follower, 8);

}

void Elevator::Move(double ref_elevator_) {

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE:
		SmartDashboard::PutString("ELEVATOR", "DOWN");
		ref_elevator = DOWN_ANGLE;
		break;

	case UP_STATE:
		SmartDashboard::PutString("ELEVATOR", "UP");
		ref_elevator = UP_ANGLE;
		break;

	case STOP_STATE:
		SmartDashboard::PutString("ELEVATOR", "STOP");
		ref_elevator = 0.0;

	}
}

void Elevator::StartElevatorThread() {

	Elevator *el = this;
	ElevatorThread = std::thread(&Elevator::ElevatorWrapper, el, &ref_elevator);
	ElevatorThread.detach();

}

void Elevator::ElevatorWrapper(Elevator *el, double *ref_el) {

	elevatorTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(ELEVATOR_SLEEP_TIME));

				if (elevatorTimer->HasPeriodPassed(ELEVATOR_WAIT_TIME)) {

					elevatorTimer->Reset();
					if(*ref_el == 0.0) {
						el->StopElevator();
					}
					else {
					el->Move(*ref_el);
					}

				}
		}
	}

}

void Elevator::EndElevatorThread() {

	ElevatorThread.~thread();

}
