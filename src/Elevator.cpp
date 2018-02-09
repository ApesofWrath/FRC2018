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

const double free_speed_e = 18730.0; //rad/s
const double G_e = (20.0 / 1.0); //gear ratio

const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double PULLEY_DIAMETER = 0.0381; //radius of the pulley in meters

const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)  //4.0
const double MIN_VOLTAGE_E = -10.0;

//For motion profiler
const double MAX_VELOCITY_E = 1.6; //0.8
const double MAX_ACCELERATION_E = 10.0; //lower stutter 5.0
const double TIME_STEP_E = 0.01;

const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) * 60.0
		* PULLEY_DIAMETER * PI; //m/s
const double Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 1; //cannot equal first state or profile will not set the first time

const double DOWN_POS_E = 0.5;
const double MID_POS_E = 0.25; //TEST
const double UP_POS_E = 0.7;

double u_e = 0.0; //this is the input in volts to the motor
double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

std::vector<std::vector<double> > K_down_e =
		{ { 1.35, 2.35 }, { 1.35, 2.35 } }; //controller matrix that is calculated in the Python simulation
std::vector<std::vector<double> > K_up_e = { { 18.03, 8.37 }, { 18.03, 8.37 } }; //controller matrix that is calculated in the Python simulation

std::vector<std::vector<double> > X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_e = { { 0.0 }, { 0.0 } };

PowerDistributionPanel *pdp_e;

ElevatorMotionProfiler *elevator_profiler;

Timer *elevatorTimer = new Timer();

bool is_at_bottom_e = false;
bool is_at_top = false;
bool first_at_bottom_e = false;
bool last_at_bottom_e = false;

Elevator::Elevator(PowerDistributionPanel *pdp,
		ElevatorMotionProfiler *elevator_profiler_) {

	hallEffectTop = new DigitalInput(2);
	hallEffectBottom = new DigitalInput(1);

	elevator_profiler = elevator_profiler_;

	elevator_profiler->SetMaxAccElevator(MAX_ACCELERATION_E);
	elevator_profiler->SetMaxVelElevator(MAX_VELOCITY_E);

	talonElevator1 = new TalonSRX(33);
	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonElevator2 = new TalonSRX(0);
	//talonElevator2->Set(ControlMode::Follower, 33); //re-slaved

	talonElevator1->ConfigPeakCurrentLimit(40, 0);
	talonElevator2->ConfigPeakCurrentLimit(40, 0);

	pdp_e = pdp;

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);
	talonElevator2->Set(ControlMode::PercentOutput, 0.0);

}

void Elevator::Move(std::vector<std::vector<double> > ref_elevator) {

	double current_pos_e = GetElevatorPosition();
	double current_vel_e = GetElevatorVelocity();

	SmartDashboard::PutNumber("ELEVATOR POS", current_pos_e);
	SmartDashboard::PutNumber("ELEVATOR VEL", current_vel_e);

	double goal_pos_e = ref_elevator[0][0];
	double goal_vel_e = ref_elevator[1][0];

	SmartDashboard::PutNumber("ELEVATOR REF POS", goal_pos_e);
	SmartDashboard::PutNumber("ELEVATOR REF VEL", goal_vel_e);

	//std::cout << "GOAL: " << goal_pos_e << std::endl;

	error_e[0][0] = goal_pos_e - current_pos_e;
	error_e[1][0] = goal_vel_e - current_vel_e;

	SmartDashboard::PutNumber("ELEVATOR ERR POS", error_e[0][0]);
	SmartDashboard::PutNumber("ELEVATOR ERR VEL", error_e[1][0]);

	v_bat_e = pdp_e->GetVoltage();

	if (goal_pos_e < current_pos_e) {
		u_e = (K_down_e[0][0] * error_e[0][0])
				+ (K_down_e[0][1] * error_e[1][0]) //u_e is voltage
				+ (Kv_e * goal_vel_e * v_bat_e); // for this system the second row of the K matrix is a copy and does not matter. //
		std::cout << "DOWN" << std::endl;
	} else {
		u_e = (K_up_e[0][0] * error_e[0][0]) + (K_up_e[0][1] * error_e[1][0]) //u_e is voltage
				+ (Kv_e * goal_vel_e * v_bat_e); // for this system the second row of the K matrix is a copy and does not matter. //
	}
	//get the input into the -1 to +1 range for the talon
	//u_e /= v_bat_e;

	//SmartDashboard::PutNumber("ELEVATOR OUTPUT", u_e);

	SetVoltageElevator(u_e);

}

void Elevator::SetVoltageElevator(double elevator_voltage) {

	is_at_bottom_e = !hallEffectBottom->Get(); //reversed. will return true if sensed
	is_at_top = !hallEffectTop->Get();

	SmartDashboard::PutNumber("HALL EFF BOT", is_at_bottom_e);
	SmartDashboard::PutNumber("HALL EFF TOP", is_at_top);

	//upper soft limit
	if (GetElevatorPosition() >= (0.9) && elevator_voltage > 0.0) { //at max height and still trying to move up
		elevator_voltage = 0.0;
	}

	//zero first time seen, on the way down
	if (is_at_bottom_e && elevator_voltage < 0.0) {
		if (first_at_bottom_e) { //first time at bottom
			ZeroEncs();
			first_at_bottom_e = false;
		}
		if (elevator_voltage < 0.0) {
			elevator_voltage = 0.0;
		}
	} else {
		first_at_bottom_e = true;
	}

	//zero last time seen, on way up
	if (!is_at_bottom_e) {
		if (last_at_bottom_e) {
			ZeroEncs();
			last_at_bottom_e = false;
		}
		if (elevator_voltage < 0.0) {
			elevator_voltage = 0.0;
		}
	} else {
		last_at_bottom_e = true;
	}

	//if (elevator_voltage < 0.0) { //account for gravity
	//	elevator_voltage += 1.5;
	//}

	if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}

	elevator_voltage *= -1.0; //reverse at END

	//scale between -1 and +1
	elevator_voltage /= pdp_e->GetVoltage();

	SmartDashboard::PutNumber("EL OUTPUT", elevator_voltage);

	//2 is slaved to 1
	talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);
	talonElevator2->Set(ControlMode::PercentOutput, elevator_voltage);

}

bool Elevator::ElevatorEncodersRunning() {

	double current_pos_e = GetElevatorPosition();
	double current_ref_e = elevator_profiler->GetNextRefElevator().at(0).at(0);

//	if (talonElevator1->GetOutputCurrent() > 5.0
//			&& std::abs(talonElevator1->GetSelectedSensorVelocity(0)) <= 0.2
//			&& std::abs(current_ref_e - current_pos_e) > 0.2) {
//		return false;
//	}
	return true;
}

double Elevator::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	double elevator_pos = (talonElevator1->GetSelectedSensorPosition(0.0)
			/ TICKS_PER_ROT_E) * (PI * PULLEY_DIAMETER) * -1.0;
	return elevator_pos;

}

double Elevator::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel = (talonElevator1->GetSelectedSensorVelocity(0.0)
			/ (TICKS_PER_ROT_E)) * (PULLEY_DIAMETER * PI) * (10.0) * -1.0;
	return elevator_vel;

}

void Elevator::ManualElevator(Joystick *joyOpElev) {

	SmartDashboard::PutNumber("ELEV CUR", talonElevator1->GetOutputCurrent());
	SmartDashboard::PutNumber("ElEV ENC",
			talonElevator1->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

	SmartDashboard::PutNumber("TOP HALL EFF", !hallEffectTop->Get());
	SmartDashboard::PutNumber("BOT HALL EFF", !hallEffectBottom->Get());

	double output = (joyOpElev->GetY()) * pdp_e->GetVoltage() * 0.5; //multiply by voltage because setvoltageelevator takes voltage

	if (joyOpElev->GetRawButton(2)) {
		ZeroEncs();
	}

	SetVoltageElevator(output);

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "DOWN");
		if (last_elevator_state != DOWN_STATE_E) { //first time in state
			elevator_profiler->SetMaxAccElevator(MAX_ACCELERATION_E);
			elevator_profiler->SetMaxVelElevator(MAX_VELOCITY_E);
			elevator_profiler->SetFinalGoalElevator(DOWN_POS_E); //DOES set to 0
			elevator_profiler->SetInitPosElevator(GetElevatorPosition()); //0 because it's testing
		}
		last_elevator_state = DOWN_STATE_E;
		break;

	case MID_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "MID");
		if (last_elevator_state != MID_STATE_E) { //first time in state
			elevator_profiler->SetMaxAccElevator(5.0);
			elevator_profiler->SetMaxVelElevator(1.0);
			elevator_profiler->SetFinalGoalElevator(MID_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
			//SmartDashboard::PutString("RESET TO MID", "YES");
		}
		//SmartDashboard::PutString("RESET TO MID", "NO");
		last_elevator_state = MID_STATE_E;
		break;

	case UP_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "UP");
		if (last_elevator_state != UP_STATE_E) { //first time in state
			elevator_profiler->SetMaxAccElevator(MAX_ACCELERATION_E);
			elevator_profiler->SetMaxVelElevator(MAX_VELOCITY_E);
			elevator_profiler->SetFinalGoalElevator(UP_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = UP_STATE_E;
		break;

	case STOP_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "STOP");
		StopElevator();
		last_elevator_state = STOP_STATE_E;
		break;

	}

}

void Elevator::StartElevatorThread() {

	Elevator *elevator_ = this;

	ElevatorThread = std::thread(&Elevator::ElevatorWrapper, elevator_);
	ElevatorThread.detach();

}

void Elevator::ElevatorWrapper(Elevator *el) {

	elevatorTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(ELEVATOR_SLEEP_TIME));

			if (elevatorTimer->HasPeriodPassed(ELEVATOR_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_elevator =
						elevator_profiler->GetNextRefElevator();

				//std::cout << "pos: " << profile_elevator.at(0).at(0) << "    " // std::endl; //"  "
				//		<< "vel: " << profile_elevator.at(1).at(0) << "   "
				//		<< "acc: " << profile_elevator.at(2).at(0) << "   " << std::endl;
			//	std::cout << "ref: " << profile_elevator.at(3).at(0)
				//		<< std::endl; //ref is 0 //and current_pos is 0// still

				if (el->elevator_state != STOP_STATE_E) {
					el->Move(profile_elevator);
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
