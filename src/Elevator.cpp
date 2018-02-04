/*
 * Elevator.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Elevator.h>
//#include "ctre/Phoenix.h"
#include <WPILib.h>

#define PI 3.14159265

const int DOWN_STATE_E = 0;
const int MID_STATE_E = 1;
const int UP_STATE_E = 2;
const int STOP_STATE_E = 3;

//TO CHANGE
const double free_speed_e = 1967.0; //rad/s
const double m_e = 2.1;
const double l_e = 0.2;
const double Kt_e = 0.00595;
const double G_e = ((60.0 / 1.0) * (48.0 / 38.0));

const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e);
const double MAX_VELOCITY_E = 28.0;
const double MAX_ACCELERATION_E = 38.0;
const double TIME_STEP_E = 0.01;
const double Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

const double TICKS_PER_ROT_E = 4096.0;
const double PULLEY_RADIUS = 0.0381; //radius of the pulley in meters
const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -12.0;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 1; //cannot equal first state or profile will not set the first time

const double DOWN_ANGLE_E = 0.0;
const double MID_ANGLE_E = 0.0;
const double UP_ANGLE_E = 0.0;

double u_e = 0; //this is the input in volts to the motor
double v_bat_e = 12.0; //this will be the voltage of the battery at every loop

std::vector<std::vector<double> > K_e = { { 0.0, 0.0 }, { 0.0, 0.0 } }; //controller matrix that is calculated in the Python simulation

std::vector<std::vector<double> > X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > ref_elevator;

std::vector<std::vector<double> > error_e = { { 0.0 }, { 0.0 } };

PowerDistributionPanel *pdp_e;

Timer *elevatorTimer = new Timer();

Elevator::Elevator(PowerDistributionPanel *pdp) {

	elevator_profiler = new MotionProfiler(MAX_VELOCITY_E, MAX_ACCELERATION_E,
			TIME_STEP_E);

	talonElevator1 = new TalonSRX(8);
	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonElevator2 = new TalonSRX(9);
	talonElevator2->Set(ControlMode::Follower, 8);

	talonElevator1->ConfigPeakCurrentLimit(30, 0);
	talonElevator2->ConfigPeakCurrentLimit(30, 0);

	pdp_e = pdp;

}

void Elevator::Move(std::vector<std::vector<double> > ref_elevator) {

	double current_pos_e = GetElevatorPosition();
	double current_vel_e = GetElevatorVelocity();
	double goal_pos_e = ref_elevator[0][0];
	double goal_vel_e = ref_elevator[1][0];

	error_e[0][0] = goal_pos_e - current_pos_e;
	error_e[1][0] = goal_vel_e - current_vel_e;

	u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]) + Kv_e; // for this system the second row of the K matrix is a copy and does not matter.

	//scaling the input value not to exceed the set parameters
	if (u_e > MAX_VOLTAGE_E) {
		u_e = MAX_VOLTAGE_E;
	} else if (u_e < MIN_VOLTAGE_E) {
		u_e = MIN_VOLTAGE_E;
	}

	v_bat_e = pdp_e->GetVoltage();

	//get the input into the -1 to +1 range for the talon

	u_e = u_e / (v_bat_e);

	//talonElevator2 is slaved to this talon and does not need to be set
	talonElevator1->Set(ControlMode::PercentOutput, u_e);

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);

}

bool Elevator::ElevatorEncodersRunning() {

	double current_pos_e = GetElevatorPosition();
	double current_ref_e = elevator_profiler->GetNextRef().at(0).at(0);

	if (talonElevator1->GetOutputCurrent() > 3.0
			&& talonElevator1->GetSelectedSensorVelocity(0) == std::abs(0.2)
			&& std::abs(current_ref_e - current_pos_e) > 0.2) {
		return false;
	}
	return true;
}

double Elevator::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	double elevator_pos = (talonElevator1->GetSelectedSensorPosition(0.0)
			/ TICKS_PER_ROT_E) * (2.0 * PI * PULLEY_RADIUS);
	return elevator_pos;

}

double Elevator::GetElevatorVelocity(){

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel = (talonElevator1->GetSelectedSensorVelocity(0.0) / (TICKS_PER_ROT_E)) * (2.0 * PULLEY_RADIUS * PI) * (10.0);
	return elevator_vel;

}

void Elevator::ManualElevator(Joystick *joyOpElev) {

	SmartDashboard::PutNumber("ELEV", talonElevator1->GetOutputCurrent());

	double output = joyOpElev->GetY() / 10.0;
	talonElevator1->Set(ControlMode::PercentOutput, output);

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "DOWN");
		if (last_elevator_state != DOWN_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoal(DOWN_ANGLE_E);
			elevator_profiler->SetInitPos(GetElevatorPosition());
		}
		break;

	case MID_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "MID");
		if (last_elevator_state != MID_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoal(MID_ANGLE_E);
			elevator_profiler->SetInitPos(GetElevatorPosition());
		}
		break;

	case UP_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "UP");
		if (last_elevator_state != UP_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoal(UP_ANGLE_E);
			elevator_profiler->SetInitPos(GetElevatorPosition());
		}
		break;

	case STOP_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "STOP");
		StopElevator();
		break;
	}

	last_elevator_state = elevator_state;
}

void Elevator::StartElevatorThread() {

	Elevator *elevator_ = this;

	ElevatorThread = std::thread(&Elevator::ElevatorWrapper, elevator_,
			elevator_profiler);
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

				std::vector<std::vector<double>> profile_elevator = elevator_profiler->GetNextRef();

				ref_elevator[0][0] = profile_elevator[0][0];
				ref_elevator[1][0] = profile_elevator[1][0];

				if (el->elevator_state != STOP_STATE_E) { //check if this works
					el->Move(ref_elevator);
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
