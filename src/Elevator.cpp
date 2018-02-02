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
const int MID_STATE = 1;
const int UP_STATE = 2;
const int STOP_STATE = 3;

//to change
const double free_speed_e = 1967.0; //rad/s
const double m_e = 2.1;
const double l_e = 0.2;
const double Kt_e = 0.00595;
const double G_e = ((60.0 / 1.0) * (48.0 / 38.0));

const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e);
const double MAX_VELOCITY_E = 28.0;
const double MAX_ACCELERATION_E = 38.0;
const double TIME_STEP_E = 0.01;
const double Kv_in_e = 1 / MAX_THEORETICAL_VELOCITY_E;

const double WHEEL_DIAMETER = 0.0;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

double ref_elevator;

int last_elevator_state = 0;

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

MotionProfiler *elevator_profiler = new MotionProfiler(MAX_VELOCITY_E,
		MAX_ACCELERATION_E, TIME_STEP_E);

Timer *elevatorTimer = new Timer();

Elevator::Elevator() {

	talonElevator1 = new TalonSRX(8);
	talonElevator2 = new TalonSRX(9);

	talonElevator2->Set(ControlMode::Follower, 8);

}

void Elevator::Move(double ref_elevator[2][1]) {

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);

}

bool Elevator::EncodersRunning() {

	double current_pos = GetPosition();
	if (talonElevator1->GetOutputCurrent() > 3.0
			&& talonElevator1->GetSelectedSensorVelocity(0) == std::abs(0.2)
			&& std::abs(ref_elevator[0][0] - current_pos) > 0.2) { //figure out pointers for this
		return false;
	}
	return true;
}

double Elevator::GetPosition() {

	double current_pos = (talonElevator1->GetSelectedSensorPosition(0.0)
			/ 4096.0) * 3.14 * WHEEL_DIAMETER; //?
	return current_pos;

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE:
		SmartDashboard::PutString("ELEVATOR", "DOWN");
		if (last_elevator_state != DOWN_STATE) { //first time in state
			elevator_profiler->SetFinalGoal(DOWN_ANGLE);
			elevator_profiler->SetInitPos(GetPosition());
		}
		break;

	case MID_STATE:
		SmartDashboard::PutString("ELEVATOR", "MID");
		if (last_elevator_state != MID_STATE) { //first time in state
			elevator_profiler->SetFinalGoal(MID_ANGLE);
			elevator_profiler->SetInitPos(GetPosition());
		}
		break;

	case UP_STATE:
		SmartDashboard::PutString("ELEVATOR", "UP");
		if (last_elevator_state != UP_STATE) { //first time in state
			elevator_profiler->SetFinalGoal(UP_ANGLE);
			elevator_profiler->SetInitPos(GetPosition());
		}
		break;

	case STOP_STATE: //not used
		SmartDashboard::PutString("ELEVATOR", "STOP");
		break;
	}

	last_elevator_state = elevator_state;
}

void Elevator::StartElevatorThread() {

	Elevator *el = this;
	ElevatorThread = std::thread(&Elevator::ElevatorWrapper, el, elevator_profiler                                                                                                                                                              );
	ElevatorThread.detach();

}

void Elevator::ElevatorWrapper(Elevator *el,
		MotionProfiler *elevator_profiler) {

	elevatorTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(ELEVATOR_SLEEP_TIME));

			if (elevatorTimer->HasPeriodPassed(ELEVATOR_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_elevator =
						elevator_profiler->GetNextRef();

				double indeces[2][1] = { { profile_elevator.at(0).at(0) }, {
						profile_elevator.at(1).at(0) } }; //Rotate() takes an array, not a vector

				if (!el->EncodersRunning()) {
					el->StopElevator();
				} else {
					el->Move(indeces);
				}

				elevatorTimer->Reset();

			}
		}
	}

}

void Elevator::EndElevatorThread() {

	ElevatorThread.~thread();

}

void Elevator::ZeroEncs() {

	talonElevator1->SetSelectedSensorPosition(0, 0, 0);
	talonElevator2->SetSelectedSensorPosition(0, 0, 0);

}
