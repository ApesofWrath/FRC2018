/*
* Elevator.cpp
*
*  Created on: Jan 12, 2018
*      Author: DriversStation
*/

#include "Elevator.h"
#include "ctre/Phoenix.h"
#include <WPILib.h>

#define PI 3.14159265

const int INIT_STATE_E = 0;
const int DOWN_STATE_E = 1;
const int MID_STATE_E = 2;
const int UP_STATE_E = 3;
const int STOP_STATE_E = 4;
const int HPS_STATE_E = 5;

const double free_speed_e = 18730.0; //rad/s
const double TICKS_PER_ROT_E = 4096.0; //possibly not
const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_E = -10.0;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 0; //init state

double offset = 0.0;
double ff = 0.0; //feedforward
double u_e = 0.0; //this is the output in volts to the motor
double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

double position_offset_e = 0.0;

double current_pos_e = 0.0;
double current_vel_e = 0.0;

double PULLEY_DIAMETER, G_e, ff_percent_e, friction_loss, MAX_THEORETICAL_VELOCITY_E, Kv_e; //gear ratio; //radius of the pulley in meters

std::vector<std::vector<double> > K_e, K_up_e, K_down_e, X_e, error_e;

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

int TOP_HALL, BOT_HALL;

std::string elev_type, elev_state, elev_safety;

Elevator::Elevator(PowerDistributionPanel *pdp, ElevatorMotionProfiler *elevator_profiler_, bool is_carr) { //carr

	is_carr_ = is_carr;

	if (is_carr) { //carr = second stage

		K_down_e =
		{ {  0, 0 }, { 0, 0 } }; //controller matrix that is calculated in the Python simulation 17.22, 0.94
		K_up_e = { { 0, 0 }, { 0, 0 } }; //controller matrix that is calculated in the Python simulation
		X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };
		error_e = { { 0.0 }, { 0.0 } };

		G_e = 0;

		ff_percent_e = 0;

		PULLEY_DIAMETER = 0;

		friction_loss = 0;

		down_pos = DOWN_POS_CARR;
		mid_pos = MID_POS_CARR;
		hps_pos = HPS_POS_CARR;
		up_pos = UP_POS_CARR;

		TOP_HALL = 0;
		BOT_HALL = 0;

		elev_type = "CARR";

		talonElevator1 = new TalonSRX(-20);

	} else { //middle stage = the stage right now

		K_down_e =
		{ {  27.89, 4.12 }, { 25.90, 1.57 } }; //controller matrix that is calculated in the Python simulation 17.22, 0.94
		K_up_e = { { 27.89, 4.12 }, { 22.11, 1.75 } }; //controller matrix that is calculated in the Python simulation
		X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };
		error_e = { { 0.0 }, { 0.0 } };

		G_e = (20.0 / 1.0);

		ff_percent_e = 0.4;

		PULLEY_DIAMETER = 0.0381; //radius of the pulley in meters

		friction_loss = 0.75;

		down_pos = DOWN_POS_MDS;
		mid_pos = MID_POS_MDS;
		hps_pos = HPS_POS_MDS;
		up_pos = UP_POS_MDS;

		TOP_HALL = 2;
		BOT_HALL = 1;

		elev_type = "MDS";

		talonElevator1 = new TalonSRX(-5);//33

		talonElevator2 = new TalonSRX(-2); //0
		talonElevator2->Set(ControlMode::Follower, -5); //re-slaved
		talonElevator2->EnableCurrentLimit(false);
		talonElevator2->ConfigContinuousCurrentLimit(40, 0);
		talonElevator2->ConfigPeakCurrentLimit(80, 0);
		talonElevator2->ConfigPeakCurrentDuration(100, 0);

	}

	MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) / 60.0
	* PULLEY_DIAMETER * PI * friction_loss; //m/s //1.87 //1.32

	Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	talonElevator1->EnableCurrentLimit(false);
	talonElevator1->ConfigContinuousCurrentLimit(40, 0);
	talonElevator1->ConfigPeakCurrentLimit(80, 0);
	talonElevator1->ConfigPeakCurrentDuration(100, 0);

	talonElevator1->ConfigVelocityMeasurementPeriod(
		VelocityMeasPeriod::Period_10Ms, 0);
		talonElevator1->ConfigVelocityMeasurementWindow(5, 0); //5 samples for every talon return

		talonElevator1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
		talonElevator1->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 0); //for getselectedsensor //getselectedsensor defaults to 10ms anyway. don't use getsensorcollection because that defaults to 160ms

		hallEffectTop = new DigitalInput(TOP_HALL);
		hallEffectBottom = new DigitalInput(BOT_HALL);

		elevator_profiler = elevator_profiler_;

		pdp_e = pdp;

	}

	void Elevator::InitializeElevator() {

		if (!is_elevator_init) { //don't see hall effect
		SetVoltage(0.0);
	}

}

void Elevator::StopElevator() {

	talonElevator1->Set(ControlMode::PercentOutput, 0.0);
	//	talonElevator2->Set(ControlMode::PercentOutput, 0.0); //is either slaved or dne

}

void Elevator::Move() {

	if (elevator_state != STOP_STATE_E_H && elevator_state != INIT_STATE_E_H) {

		std::vector<std::vector<double> > ref_elevator =
		elevator_profiler->GetNextRefElevator();

		current_pos_e = ref_elevator[0][0];//GetElevatorPosition(); //TAKE THIS BACK OUT
		current_vel_e = ref_elevator[1][0];//GetElevatorVelocity();

		///	SmartDashboard::PutNumber("Actual Vel", current_vel_e);
		//	SmartDashboard::PutNumber("Actual Pos", current_pos_e);

		double goal_pos = ref_elevator[0][0];
		goal_vel_e = ref_elevator[1][0];

		//	SmartDashboard::PutNumber("Goal Vel", goal_vel_e);
		//	SmartDashboard::PutNumber("Goal Pos", goal_pos);

		error_e[0][0] = goal_pos - current_pos_e;
		error_e[1][0] = goal_vel_e - current_vel_e;

		v_bat_e = 12.0;

		if (elevator_profiler->GetFinalGoalElevator()
		< elevator_profiler->GetInitPosElevator()) { //can't be the next goal in case we get ahead of the profiler
		K_e = K_down_e;
		ff = (Kv_e * goal_vel_e * v_bat_e) * 0.55;
		offset = 1.0; //dampen
	} else {
		offset = 1.0;
		K_e = K_up_e;

		ff = (Kv_e * goal_vel_e * v_bat_e) * ff_percent_e;

	}

	u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]);

	u_e += ff + offset;
	SetVoltage(u_e);

}

}

double Elevator::GetVoltageElevator() {

	return u_e;

}

void Elevator::SetVoltage(double elevator_voltage) {

	is_at_bottom_e = IsAtBottomElevator();
	is_at_top = IsAtTopElevator();

	double el_pos = GetElevatorPosition();

	if (std::abs(GetElevatorVelocity()) <= 0.05
	&& std::abs(elevator_voltage) > 3.0) { //this has to be here at the end
		encoder_counter_e++;
	} else {
		encoder_counter_e = 0;
	}
	if (encoder_counter_e > 3) { //bypass the initial high voltage to accelerate from 0
		voltage_safety_e = true;
	} else {
		voltage_safety_e = false;
	}

//TODO: may need to change order
	if (el_pos >= (0.92) && elevator_voltage > 0.0) { //upper soft limit
		elevator_voltage = 0.0;
		elev_safety = "upper soft";
	} else if (el_pos <= (-0.05) && elevator_voltage < 0.0) {  //lower soft limit
		elevator_voltage = 0.0;
		elev_safety = "lower soft";
	} else if (is_at_top && elevator_voltage < -0.2) { //elevator_voltage is actually reverse
		elev_safety = "top hall eff";
		elevator_voltage = 0.0;
	} else if (is_at_bottom_e && elevator_voltage > 0.2) { //elevator_voltage is actually reverse
		elev_safety = "bot hall eff";
		elevator_voltage = 0.0;
	} else if (keep_elevator_up) {
		elevator_voltage = 1.0;
		elev_safety = "arm safety";
	} else if (voltage_safety_e) {
		elevator_voltage = 0.0;
		elev_safety = "stall";
	}	else {
		elev_safety = "NONE";
	}

	if (is_carr_) {
		SmartDashboard::PutString("CARR SAFETY", elev_safety);
	} else {
		SmartDashboard::PutString("MDS SAFETY", elev_safety);
	}

	if (!is_elevator_init) { //changed this to just zero on start up (as it always be at the bottom at the start of the match)
		if (ZeroEncs()) { //successfully zeroed one time
			is_elevator_init = true;
		}
	}

	if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}

	elevator_voltage /= 12.0;

	elevator_voltage *= -1.0; //reverse at END

	//2 is slaved to 1
	talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);

}

double Elevator::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int elev_pos =
	talonElevator1->GetSelectedSensorPosition(0);

	double elevator_pos = ((elev_pos - position_offset_e) / TICKS_PER_ROT_E) //position offset to zero
	* (PI * PULLEY_DIAMETER) * -1.0;

	return elevator_pos;

}

double Elevator::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel =
	(talonElevator1->GetSelectedSensorVelocity(0)
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

bool Elevator::IsAtPos(double target_pos) {

	if (std::abs(current_pos_e - target_pos) < 0.2) { //TODO: revert this hack
		return true;
	}
	return false;

}

void Elevator::ManualElevator(Joystick *joyOpElev) {

	//SmartDashboard::PutNumber("ELEV CUR", talonElevator1->GetOutputCurrent());

	//	SmartDashboard::PutNumber("ElEV ENC",
	//		talonElevator1->GetSensorCollection.GetQuadraturePosition()); //TODO: figure out

	//SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

	double output = (joyOpElev->GetY()) * 0.5 * 12.0; //multiply by voltage because setvoltageelevator takes voltage
	//
	SetVoltage(output);

}

void Elevator::ElevatorStateMachine() {

	if (is_carr_) {
		SmartDashboard::PutString("CARR", elev_state);
	} else {
		SmartDashboard::PutString("MDS", elev_state);
	}

	switch (elevator_state) {

		case INIT_STATE_E:

		elev_state = "INIT";
		if (is_elevator_init) {
			elevator_state = DOWN_STATE_E;
		} else {
			InitializeElevator();
		}
		last_elevator_state = INIT_STATE_E;
		break;

		case DOWN_STATE_E:

		elev_state = "DOWN";

		if (last_elevator_state != DOWN_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(down_pos);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = DOWN_STATE_E;
		break;

		case MID_STATE_E:

		elev_state = "MID";

		if (last_elevator_state != MID_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(mid_pos);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = MID_STATE_E;
		break;

		case UP_STATE_E:

		elev_state = "UP";

		if (last_elevator_state != UP_STATE_E) { //first time in state
			elevator_profiler->SetFinalGoalElevator(up_pos);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = UP_STATE_E;
		break;

		case STOP_STATE_E:

		elev_state = "STOP";
		StopElevator();
		last_elevator_state = STOP_STATE_E;
		break;

		case HPS_STATE_E:

		elev_state = "HPS";

		if (last_elevator_state != HPS_STATE_E) {
			elevator_profiler->SetFinalGoalElevator(hps_pos);
			elevator_profiler->SetInitPosElevator(GetElevatorPosition());
		}
		last_elevator_state = HPS_STATE_E;
		break;

	}

}

void Elevator::SetZeroOffset() {

	position_offset_e =
	talonElevator1->GetSelectedSensorPosition(0);
}

bool Elevator::ZeroEncs() {

	if (zeroing_counter_e < 1) {
		//Great Robotic Actuation and Controls Execution ()
		SetZeroOffset();
		zeroing_counter_e++;
		return true;
	} else {
		return false;
	}

}
