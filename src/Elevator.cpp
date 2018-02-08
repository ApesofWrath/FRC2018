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
const double free_speed_e = 18730.0; //rad/s
const double G_e = (20.0 / 1.0);

const double TICKS_PER_ROT_E = 4096.0;
const double PULLEY_RADIUS = 0.0381; //radius of the pulley in meters
const double MAX_VOLTAGE_E = 4.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -4.0;

const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) * 60.0
		* PULLEY_RADIUS * 2.0 * PI;
const double MAX_VELOCITY_E = 1.0;
const double MAX_ACCELERATION_E = 2.0;
const double TIME_STEP_E = 0.01;
const double Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 1; //cannot equal first state or profile will not set the first time

const double DOWN_ANGLE_E = 0.7;
const double MID_ANGLE_E = 1.50; //TEST
const double UP_ANGLE_E = 3.0;

double u_e = 0.0; //this is the input in volts to the motor
double v_bat_e = 12.0; //this will be the voltage of the battery at every loop

std::vector<std::vector<double> > K_e =
		{ { 57.85, 1.98 }, { 57.85, 1.98 } }; //controller matrix that is calculated in the Python simulation

std::vector<std::vector<double> > X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_e = { { 0.0 }, { 0.0 } };

PowerDistributionPanel *pdp_e;

MotionProfiler *elevator_profiler;

Timer *elevatorTimer = new Timer();

int counter_e = 0;

double voltage_el = 0.0;

bool is_at_bottom_e = false;
bool is_at_top = false;

Elevator::Elevator(PowerDistributionPanel *pdp,
		MotionProfiler *elevator_profiler_) {

	hallEffectTop = new DigitalInput(2);
	hallEffectBottom = new DigitalInput(1);

	elevator_profiler = elevator_profiler_;

	elevator_profiler->SetMaxAcc(4.0);
	elevator_profiler->SetMaxVel(1.0);

	talonElevator1 = new TalonSRX(13);
	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonElevator2 = new TalonSRX(0);
	talonElevator2->Set(ControlMode::Follower, 13);

	talonElevator1->ConfigPeakCurrentLimit(2, 0);
	talonElevator2->ConfigPeakCurrentLimit(2, 0);

	pdp_e = pdp;

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

	u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]) //u_e is voltage
			+ (Kv_e * goal_vel_e * v_bat_e); // for this system the second row of the K matrix is a copy and does not matter. //

	//get the input into the -1 to +1 range for the talon

	//u_e = u_e / (v_bat_e);
	u_e /= v_bat_e;

	//scaling the input value not to exceed the set parameters
	if (u_e > 2.0) {
		u_e = 2.0;
	} else if (u_e < -2.0) {
		u_e = -2.0;
	}

	SmartDashboard::PutNumber("ELEVATOR OUTPUT", u_e);
	SetVoltageElevator(u_e);
	//talonElevator2 is slaved to this talon and does not need to be set
	//talonElevator1->Set(ControlMode::PercentOutput, u_e);

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);

}

void Elevator::SetVoltageElevator(double elevator_voltage) {

	//std::cout << "EL VOLT " << elevator_voltage << std::endl;

	is_at_bottom_e = !hallEffectBottom->Get();
	is_at_top = !hallEffectTop->Get();
	SmartDashboard::PutNumber("HALL EFF BOT", is_at_bottom_e);
	SmartDashboard::PutNumber("HALL EFF TOP", is_at_top);

	if (GetElevatorPosition() >= (0.9) && elevator_voltage > 0.0) { //at max height and still trying to move up
			elevator_voltage = 0.0;
	}

	if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}

	//down: zero first time seen
	//up: zero last time seen (first time unseen)
	//initialization: on way up

	//have soft limit too

//	if (is_at_bottom_e && elevator_voltage < 0.0) { //not done yet
//		if (first_at_bottom) { //first time at bottom
//			ZeroEnc();
//			first_at_bottom = false;
//		}
//		if (voltage_e < 0.0) {
//			voltage_e = 0.0;
//		}
//	} else {
//		first_at_bottom = true;
//	}

	//	if (is_at_top) {
	//		if (first_at_top) { //first time at bottom
	//			ZeroEnc();
	//			first_at_top = false;
	//		}
	//		if (voltage_e > 0.0) {
	//			voltage_e = 0.0;
	//		}
	//	} else {
	//		first_at_top = true;
	//	}

	elevator_voltage *= -1.0; //reverse at END
	elevator_voltage /= pdp_e->GetVoltage();

	SmartDashboard::PutNumber("VOLTAGE EL", elevator_voltage);

	talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);

}

bool Elevator::ElevatorEncodersRunning() {

	double current_pos_e = GetElevatorPosition();
	double current_ref_e = elevator_profiler->GetNextRef().at(0).at(0);

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
			/ TICKS_PER_ROT_E) * (2.0 * PI * PULLEY_RADIUS) * -1.0;
	return elevator_pos;

}

double Elevator::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel = (talonElevator1->GetSelectedSensorVelocity(0.0)
			/ (TICKS_PER_ROT_E)) * (2.0 * PULLEY_RADIUS * PI) * (10.0) * -1.0;
	return elevator_vel;

}

void Elevator::ManualElevator(Joystick *joyOpElev) {

	SmartDashboard::PutNumber("ELEV CUR", talonElevator1->GetOutputCurrent());
	SmartDashboard::PutNumber("ElEV ENC",
			talonElevator1->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

	SmartDashboard::PutNumber("TOP HALL EFF",
			!hallEffectTop->Get());
	SmartDashboard::PutNumber("BOT HALL EFF",
			!hallEffectBottom->Get());

	double output = (joyOpElev->GetY()) * pdp_e->GetVoltage() * 0.5; //multiply by voltage because setvoltageelevator takes voltage

	if(joyOpElev->GetRawButton(2)) {
		ZeroEncs();
	}

	SetVoltageElevator(output);

}

void Elevator::ElevatorStateMachine() {

	switch (elevator_state) {

	case DOWN_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "DOWN");
		if (last_elevator_state != DOWN_STATE_E) { //first time in state
			elevator_profiler->SetMaxAcc(4.0);
			elevator_profiler->SetMaxVel(1.0);
			elevator_profiler->SetFinalGoal(DOWN_ANGLE_E); //DOES set to 0
			elevator_profiler->SetInitPos(GetElevatorPosition()); //0 because it's testing
			SmartDashboard::PutString("RESET TO DOWN", "YES");
			counter_e++;
		}
		if (counter_e > 1) { //DOES enter the if statement, DOES SetFinalGoal
			SmartDashboard::PutString("RESET TO DOWN", "GOOD");
		} else {
			SmartDashboard::PutString("RESET TO DOWN", "NO");
		}
		last_elevator_state = DOWN_STATE_E;
		break;

	case MID_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "MID");
		if (last_elevator_state != MID_STATE_E) { //first time in state
			elevator_profiler->SetMaxAcc(4.0);
			elevator_profiler->SetMaxVel(1.0);
			elevator_profiler->SetFinalGoal(MID_ANGLE_E);
			elevator_profiler->SetInitPos(GetElevatorPosition());
			SmartDashboard::PutString("RESET TO MID", "YES");
		}
		//SmartDashboard::PutString("RESET TO MID", "NO");
		last_elevator_state = MID_STATE_E;
		break;

	case UP_STATE_E:
		SmartDashboard::PutString("ELEVATOR", "UP");
		if (last_elevator_state != UP_STATE_E) { //first time in state
			elevator_profiler->SetMaxAcc(4.0);
			elevator_profiler->SetMaxVel(1.0);
			elevator_profiler->SetFinalGoal(UP_ANGLE_E);
			elevator_profiler->SetInitPos(GetElevatorPosition());
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
						elevator_profiler->GetNextRef();

				//std::cout << "pos: " << profile_elevator.at(0).at(0) << "    " // std::endl; //"  "
				//		<< "vel: " << profile_elevator.at(1).at(0) << "   "
				//		<< "acc: " << profile_elevator.at(2).at(0) << "   " << std::endl;
				std::cout << "ref: " << profile_elevator.at(3).at(0)
						<< std::endl; //ref is 0 //and current_pos is 0// still

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
