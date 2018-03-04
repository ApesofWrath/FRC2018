/*
 * Elevator.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Elevator.h>
#include "ctre/Phoenix.h"
#include <WPILib.h>

#define PI 3.14159265

#define CORNELIUS 0

#if CORNELIUS
double ff_percent = 0.4;
#else
double ff_percent = 0.4;
#endif

const int INIT_STATE_E = 0;
const int DOWN_STATE_E = 1;
const int MID_STATE_E = 2;
const int UP_STATE_E = 3;
const int STOP_STATE_E = 4;
const int SWITCH_STATE_E = 5;

const double free_speed_e = 18730.0; //rad/s
const double G_e = (20.0 / 1.0); //gear ratio

const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double PULLEY_DIAMETER = 0.0381; //radius of the pulley in meters

const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)  //4.0
const double MIN_VOLTAGE_E = -10.0;

//For motion profiler
//const double MAX_VELOCITY_E = 1.6; //0.8 //1,3
//const double MAX_ACCELERATION_E = 10.0; //lower stutter 5.0 ///7.5
//const double TIME_STEP_E = 0.01;

const double friction_loss = 0.707; //checked with graph. ff matches ref vel

const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) / 60.0
		* PULLEY_DIAMETER * PI * friction_loss; //m/s
const double Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 0; //init state

double offset = 0.0;
double ff = 0.0; //feedforward
double u_e = 0.0; //this is the input in volts to the motor
double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

double position_offset_e = 0.0;

std::vector<std::vector<double> > K_e;
std::vector<std::vector<double> > K_down_e =
		{ { 17.22, 0.94 }, { 25.90, 1.57 } }; //controller matrix that is calculated in the Python simulation
std::vector<std::vector<double> > K_up_e = { { 25.90, 1.57 }, { 25.90, 1.57 } }; //controller matrix that is calculated in the Python simulation

std::vector<std::vector<double> > X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_e = { { 0.0 }, { 0.0 } };

ElevatorMotionProfiler *elevator_profiler;

Timer *elevatorTimer = new Timer();

bool is_at_bottom_e = false;
bool is_at_top = false;
bool first_at_bottom_e = false;
bool last_at_bottom_e = false;
bool encs_zeroed_e = false;
bool voltage_safety_e = false;

int init_counter = 0;
int encoder_counter_e = 0;

Elevator::Elevator(PowerDistributionPanel *pdp,
		ElevatorMotionProfiler *elevator_profiler_) {

	hallEffectTop = new DigitalInput(2);
	hallEffectBottom = new DigitalInput(1);

	elevator_profiler = elevator_profiler_;

//	elevator_profiler->SetMaxAccElevator(MAX_ACCELERATION_E);
//	elevator_profiler->SetMaxVelElevator(MAX_VELOCITY_E);

	talonElevator1 = new TalonSRX(33);
//	talonElevator1->ConfigVoltageCompSaturation(12.0, 0);
//	talonElevator1->EnableVoltageCompensation(true);
	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonElevator2 = new TalonSRX(0);
//	talonElevator2->ConfigVoltageCompSaturation(12.0, 0);
//	talonElevator2->EnableVoltageCompensation(true);
	talonElevator2->Set(ControlMode::Follower, 33); //re-slaved

	talonElevator1->EnableCurrentLimit(true);
	talonElevator2->EnableCurrentLimit(true);
	talonElevator1->ConfigContinuousCurrentLimit(40, 0);
	talonElevator2->ConfigContinuousCurrentLimit(40, 0);
	talonElevator1->ConfigPeakCurrentLimit(80, 0);
	talonElevator2->ConfigPeakCurrentLimit(80, 0);
	talonElevator1->ConfigPeakCurrentDuration(100, 0);
	talonElevator2->ConfigPeakCurrentDuration(100, 0);

	talonElevator1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	talonElevator1->ConfigVelocityMeasurementWindow(5, 0);

	pdp_e = pdp;

}

void Elevator::InitializeElevator() {

	if (!is_elevator_init) { //don't see hall effect
		//is_elevator_init = false;
		SetVoltageElevator(2.0); //double elevator_volt = (2.0 / pdp_e->GetVoltage()) * -1.0; //up  not called
//		talonElevator1->Set(ControlMode::PercentOutput,
//				elevator_volt);
//		talonElevator2->Set(ControlMode::PercentOutput,
//				elevator_volt);
	}

	//SmartDashboard::PutString("init elev", "yes");

	//double up_volt = (0.2 * -1.0) / pdp_e->GetVoltage(); //to not crash down
	//talonElevator1->Set(ControlMode::PercentOutput, up_volt);
	//talonElevator2->Set(ControlMode::PercentOutput, up_volt);

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);
	talonElevator2->Set(ControlMode::PercentOutput, 0.0);

}

void Elevator::Move(std::vector<std::vector<double> > ref_elevator) {

	double current_pos_e = GetElevatorPosition();
	double current_vel_e = GetElevatorVelocity();

	double goal_pos = ref_elevator[0][0];
	goal_vel_e = ref_elevator[1][0];

//	SmartDashboard::PutNumber("ELEVATOR REF POS", goal_pos_e);
//	SmartDashboard::PutNumber("ELEVATOR REF VEL", goal_vel_e);

	error_e[0][0] = goal_pos - current_pos_e;
	error_e[1][0] = goal_vel_e - current_vel_e;
//
//	SmartDashboard::PutNumber("ELEVATOR ERR POS", error_e[0][0]);
//	SmartDashboard::PutNumber("ELEVATOR ERR VEL", error_e[1][0]);

	v_bat_e = 12.0;

	if (elevator_profiler->GetFinalGoalElevator()
			< elevator_profiler->GetInitPosElevator()) { //can't be the next goal in case we get ahead of the profiler //elevator_profiler->final_goal
		K_e = K_down_e;
		ff = 0.0; //(Kv_e * goal_vel_e * v_bat_e);
		offset = 1.0; //dampen //1.0
		//SmartDashboard::PutString("EL GAINS", "DOWN");
	} else {
		offset = 0.0;
		K_e = K_up_e;

		ff = (Kv_e * goal_vel_e * v_bat_e) * ff_percent;

		//SmartDashboard::PutString("EL GAINS", "UP");
	}

//	K_e = K_up_e;

	u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]);

	//std::cout << "FF:  " << ff << "  FB: " << u_e << std::endl;

	u_e += ff + offset; //u_e is voltage
	//+ (Kv_e * goal_vel_e * v_bat_e); // for this system the second row of the K matrix is a copy and does not matter. //

//	if (goal_pos_e < current_pos_e) { //changed
//		u_e = -1.5;
//	} else {
//		u_e = 1.5;
//	}

	//talonElevator1->Set(ControlMode::PercentOutput, u_e);
	//(Kv_e * goal_vel_e * v_bat_e);

	SetVoltageElevator(u_e); //u_e

}

double Elevator::GetVoltageElevator() {

	return u_e;

}

void Elevator::SetVoltageElevator(double elevator_voltage) {

//	int enc = talonElevator1->GetSensorCollection().GetQuadraturePosition(); //encoders return ints?
//	SmartDashboard::PutNumber("ElEV ENC", enc);
//

	SmartDashboard::PutString("ELEVATOR SAFETY", "none");

//
//	SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());
//	SmartDashboard::PutNumber("ELEV VEL", GetElevatorVelocity());

//	std::cout << "EL VEL: " << GetElevatorVelocity() << std::endl;

	is_at_bottom_e = IsAtBottomElevator(); //reversed. will return true if sensed
	is_at_top = IsAtTopElevator();

//	SmartDashboard::PutNumber("HALL EFF BOT", is_at_bottom_e);
//	SmartDashboard::PutNumber("HALL EFF TOP", is_at_top);

//	//safety to make sure that the elevator doesn't go down when the arm is up
//	if (intake_e->GetAngularPosition() > 1.7 && elevator_voltage < 0.0){
//
//		elevator_voltage = 0.0;
//
//	}

	//upper soft limit
	if (GetElevatorPosition() >= (0.92) && elevator_voltage > 0.0) { //at max height and still trying to move up
		elevator_voltage = 0.0;

		SmartDashboard::PutString("ELEVATOR SAFETY", "upper soft");

		//std::cout << "S o f t l i m i t" << std::endl;
	}
//
//	//lower soft limit
	if (GetElevatorPosition() <= (-0.05) && elevator_voltage < 0.0) { //at max height and still trying to move up
		elevator_voltage = 0.0;

		SmartDashboard::PutString("ELEVATOR SAFETY", "lower soft");

		//std::cout << "S o f t l i m i t" << std::endl;
	}
//
//	//zero first time seen, on the way down
//	if (is_at_bottom_e) {
//		if (first_at_bottom_e) { //first time at bottom
//			ZeroEncs();
//			is_elevator_init = true;
//			first_at_bottom_e = false;
//		}
//		if (elevator_voltage < 0.0) {
//			elevator_voltage = 0.0;
//		}
//	} else {
//		first_at_bottom_e = true;
//	}
////
////	//zero last time seen, on way up
	if (!is_at_bottom_e) {
		//if (last_at_bottom_e) {
		if (ZeroEncs()) { //successfully zeroed one time
			is_elevator_init = true;
		}
		//last_at_bottom_e = false;
	}

	if (is_at_top && elevator_voltage > 0.1) {

		SmartDashboard::PutString("ELEVATOR SAFETY", "upper hall eff");

		elevator_voltage = 0.0;
	}
	//if (elevator_voltage < 0.0) { //account for gravity
	//	elevator_voltage += 1.5;
	//}

	//scale between -1 and +1

	if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}

	if (std::abs(GetElevatorVelocity()) <= 0.05
			&& std::abs(elevator_voltage) > 3.0) { //this has to be here at the end
		encoder_counter_e++;
		//	SmartDashboard::PutString("HERE", "yes");
	} else {
		encoder_counter_e = 0;
		//	SmartDashboard::PutString("HERE", "no");
	}

	if (encoder_counter_e > 3) { //bypass the initial high voltage to accelerate from 0
		voltage_safety_e = true;
	} else {
		voltage_safety_e = false;
	}

	if (voltage_safety_e) {
		elevator_voltage = 0.0;

		SmartDashboard::PutString("ELEVATOR SAFETY", "stall");

	}

	//SmartDashboard::PutNumber("EL VOLT", elevator_voltage);
	///std::cout << "el volt: " << elevator_voltage << std::endl;

	elevator_voltage /= 12.0;

	elevator_voltage *= -1.0; //reverse at END

	//std::cout << "EL OUTPUT " << elevator_voltage << std::endl;

///SmartDashboard::PutNumber("EL OUTPUT", elevator_voltage);

	//2 is not slaved to 1
	talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);
	//talonElevator2->Set(ControlMode::PercentOutput, elevator_voltage);

}

bool Elevator::ElevatorEncodersRunning() {

//	double current_pos_e = GetElevatorPosition();
//	double current_ref_e = elevator_profiler->GetNextRefElevator().at(0).at(0);

//	if (talonElevator1->GetOutputCurrent() > 5.0
//			&& std::abs(talonElevator1->GetSelectedSensorVelocity(0)) <= 0.2
//			&& std::abs(current_ref_e - current_pos_e) > 0.2) {
//		return false;
//	}

	return true;
}

double Elevator::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int elev_pos =
			talonElevator1->GetSensorCollection().GetQuadraturePosition();

	//std::cout << "elev pos:" << elev_pos << std::endl;
	//std::cout << "pos offset:" << position_offset_e << std::endl;

	double elevator_pos = ((elev_pos - position_offset_e) / TICKS_PER_ROT_E)
			* (PI * PULLEY_DIAMETER) * -1.0;

	return elevator_pos;

}

double Elevator::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel =
			(talonElevator1->GetSensorCollection().GetQuadratureVelocity()
					/ (TICKS_PER_ROT_E)) * (PULLEY_DIAMETER * PI) * (10.0)
					* -1.0;
	return elevator_vel;

}

bool Elevator::IsAtBottomElevator() {
	if (!hallEffectBottom->Get()) {
		return true;
	} else {
		return false;
	}
}

bool Elevator::IsAtTopElevator() {
	if (!hallEffectTop->Get()) {
		return true;
	} else {
		return false;
	}
}

void Elevator::ManualElevator(Joystick *joyOpElev) {

	//SmartDashboard::PutNumber("ELEV CUR", talonElevator1->GetOutputCurrent());

//	SmartDashboard::PutNumber("ElEV ENC",
//		talonElevator1->GetSensorCollection.GetQuadraturePosition()); //TODO: figure out

	//SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

	double output = (joyOpElev->GetY()) * 0.5 * 12.0; //multiply by voltage because setvoltageelevator takes voltage
//
	SetVoltageElevator(output);

}

void Elevator::ElevatorStateMachine() {

//	SmartDashboard::PutString("sm el", "yes");

//	SmartDashboard::PutNumber("ELEVATOR VEL", GetElevatorVelocity());

	switch (elevator_state) {

	case INIT_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "INIT");
		if (is_elevator_init) {
			elevator_state = DOWN_STATE_E;
		} else {
			InitializeElevator();
		}
		last_elevator_state = INIT_STATE_E;
		break;

	case DOWN_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "DOWN");

		if (last_elevator_state != DOWN_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(DOWN_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = DOWN_STATE_E;
		break;

	case MID_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "MID");

		if (last_elevator_state != MID_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(MID_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = MID_STATE_E;
		break;

	case UP_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "UP");

		if (last_elevator_state != UP_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(UP_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = UP_STATE_E;
		break;

	case STOP_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "STOP");

		StopElevator();
		last_elevator_state = STOP_STATE_E;
		break;

	case SWITCH_STATE_E:

		SmartDashboard::PutString("ELEVATOR.", "SWITCH");

		if (last_elevator_state != SWITCH_STATE_E) {
			elevator_profiler->SetFinalGoalElevator(SWITCH_POS_E);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = SWITCH_STATE_E;
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

		elevatorTimer->Reset();

		if (frc::RobotState::IsEnabled()) {

			std::vector<std::vector<double>> profile_elevator =
					elevator_profiler->GetNextRefElevator();

			if (el->elevator_state != STOP_STATE_E
					&& el->elevator_state != INIT_STATE_E) {
				el->Move(profile_elevator);
			}

		}

		double time_e = 0.01 - elevatorTimer->Get(); //change

		time_e *= 1000;
		if (time_e < 0) {
			time_e = 0;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds((int) time_e));

		//std::cout << "time el: " << elevatorTimer->Get() << std::endl;
	}

}

void Elevator::EndElevatorThread() {

	//elevatorTimer->Stop();
	ElevatorThread.~thread();

}

void Elevator::SetZeroOffsetElevator() {

	position_offset_e =
			talonElevator1->GetSensorCollection().GetQuadraturePosition();
}

bool Elevator::ZeroEncs() {

	if (zeroing_counter_e < 1) {
		//Great Robotic Actuation and Controls Execution ()
		SetZeroOffsetElevator();
		zeroing_counter_e++;
		return true;
	} else {
		return false;
	}

}
