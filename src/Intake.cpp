/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Intake.h>
#include <ctre/Phoenix.h> //double included
#include <WPILib.h>

#define PI 3.14159265

#define CORNELIUS 1

#if CORNELIUS
double ff_percent_i = 0.6;
double offset_angle = 1.65;
double SLOW_SPEED = 0.25;
//down angle 0.02
#else
double ff_percent_i = 0.6;
double offset_angle = 1.60; //raising this will make the arm positions be higher
double SLOW_SPEED = 0.4;
#endif

using namespace std::chrono;

//int time_counter = 0;
bool first_in_check = true;

const int INIT_STATE = 0;
const int UP_STATE = 1; //arm state machine
const int MID_STATE = 2;
const int DOWN_STATE = 3;
const int STOP_ARM_STATE = 4;
const int SWITCH_STATE = 5;
const int SWITCH_BACK_SHOT_STATE = 6;

const int STOP_WHEEL_STATE = 0; //wheel state machine
const int IN_STATE = 1;
const int OUT_STATE = 2;
const int SLOW_STATE = 3;
const int SLOW_SCALE_STATE = 4;

const double TICKS_PER_ROT_I = 4096.0;
const double MAX_VOLTAGE_I = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_I = -12.0;

const double free_speed_i = 18730.0; //rpm
const double G_i = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i))
		* ((2.0 * PI) / 60.0) * 1.05; //rad/s
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

const double MAX_INTAKE_CURRENT = 10000000.0;
const double OUTTAKE_INTAKE_CURRENT = 20.0;

const double OUTTAKE_CORR_VALUE = 1000.0;
const double INTAKE_CORR_VALUE = 1800.0;

const double ZERO_CURRENT = 4.0; //3.0;

const double PCL_WHEELS = 30.0; //peak current limit
const double CCL_WHEELS = 10.0; //continuous current limit
const double PCD_WHEELS = 200.0; //peak current duration

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

int last_intake_state = 0; //cannot equal the first state or profile will not set the first time
int last_intake_wheel_state = 3; //current limits first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 0.0; //will be set to pdp's voltage

std::vector<std::vector<double> > K_i;
std::vector<std::vector<double> > K_down_i = { { 15.00, 1.00 }, //controller matrix that is calculated in the Python simulation, pos and vel  10.32, 0.063
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_i = { { 15.00, 1.00 }, //controller matrix that is calculated in the Python simulation, pos and vel 16.75, 0.12
		{ 16.75, 0.12 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;
Elevator *elevator_i;

double starting_pos = 0.0;
bool start_counting = false;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

std::vector<double> volts = { };

const int sample_window_intake = 40;
const int sample_window_outtake = 50;
const int arr_len_int = 75;
alglib::ae_int_t arr_len = arr_len_int;
alglib::real_1d_array currents_intake;
alglib::real_1d_array currents_outtake_r;
alglib::real_1d_array currents_outtake_l;

//; //magic #

alglib::real_1d_array master_slow_scale(
		"[2.625,2.625,5,8.875,13.125,17.5,17.375,17.25,16.375,15.5,14.375,13.375,12.75,12,11,9.625,8.375,7.375,6.75,6,5.375,5,4.75,4.75,4.75,4.875,5,5,4.625,4.375,4,4,4,3.75,3.75,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]");
alglib::real_1d_array master_switch(
		"[2.625,2.625,4.25,5.5,7.125,8.125,8.25,8.375,8.375,8,7.5,6.75,6.5,6.375,6.25,6.25,6.375,6.25,5.875,5.25,4.625,4.25,3.75,3.5,3.25,3.125,3.25,3.375,3.125,2.75,2.75,2.75,2.5,2.5,2.5,2.5,2.5,2.5,2.125,2.5,2.5,2.125,2.125,2.125,2.125,2.125,2.125,2.125,2.125,2.125,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]");
alglib::real_1d_array master_back;
alglib::real_1d_array master_slow_back;
alglib::real_1d_array master_scale(
		"[2.625,2.625,9.25,32.875,44.75,43.875,41.875,39.25,35.875,33.875,31.5,29,25,21.5,18.375,16.125,14.375,13.125,12,11.125,9.875,9,8.25,8,8.125,8.5,8.5,8.125,7.5,6.25,5.875,5.625,5.875,6.25,6.25,6.25,5.875,5.875,5.875,5.875,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]");

double last_corr_val_r = 0.0; //can start at 0 because won't return true the first time anyway
double last_corr_val_l = 0.0; //can start at 0 because won't return true the first time anyway
double corr_val_now_r = 0.0;
double corr_val_now_l = 0.0;

alglib::real_1d_array corr_intake;
alglib::real_1d_array corr_outtake_r;
alglib::real_1d_array corr_outtake_l;

int time_counter = 0;
int TIME_LIMIT = 50; //1sec

int arr_counter = 0;
double filled_arr = 0.0;
double currents_sum_1 = 0;
double currents_avg_1 = 0;
double currents_sum_2 = 0;
double currents_avg_2 = 0;
bool arr_filled = false;

int currents_index = 0;

int init_counter_i = 0;
int current_counter_h = 0; //have
int current_counter_r = 0; //release
int current_counter_l = 0; //low
int current_counter = 0;
int counter_i = 0;
//int i = 0;
int encoder_counter = 0;

int have_cube_wait = 0;

int position_offset = 0;

IntakeMotionProfiler *intake_profiler;

int current_counter_first = 0;
bool cube_in = false;

Intake::Intake(PowerDistributionPanel *pdp,
		IntakeMotionProfiler *intake_profiler_, Elevator *el_) {

	intake_profiler = intake_profiler_;

	elevator_i = el_;

	hallEffectIntake = new DigitalInput(0);

	talonIntake1 = new TalonSRX(17);
	talonIntake2 = new TalonSRX(25);

	talonIntake1->EnableCurrentLimit(true);
	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake1->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake2->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake1->ConfigPeakCurrentDuration(PCD_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentDuration(PCD_WHEELS, 0);

	talonIntakeArm = new TalonSRX(55);

	talonIntakeArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	talonIntakeArm->EnableCurrentLimit(true);
	talonIntakeArm->ConfigPeakCurrentLimit(80, 0);
	talonIntakeArm->ConfigContinuousCurrentLimit(40, 0);
	talonIntakeArm->ConfigPeakCurrentDuration(300, 0);

	talonIntakeArm->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	talonIntakeArm->ConfigVelocityMeasurementWindow(5, 0);

	talonIntakeArm->SetControlFramePeriod(ControlFrame::Control_3_General, 5);

	talonIntakeArm->SetStatusFramePeriod(
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

	pdp_i = pdp;

	currents_intake.setlength(arr_len);
	currents_outtake_r.setlength(arr_len);
	currents_outtake_l.setlength(arr_len);

//	master_slow_scale.setlength(arr_len);
//	master_switch.setlength(arr_len);
//	master_back.setlength(arr_len);
//	master_slow_back.setlength(arr_len);
//	master_scale.setlength(arr_len);

	corr_intake.setlength(arr_len - 2);
	corr_outtake_r.setlength(arr_len - 2);
	corr_outtake_l.setlength(arr_len - 2);

	for (int i = 0; i < arr_len; i++) {
		currents_intake[i] = 0.0;
		currents_outtake_r[i] = 0.0;
		currents_outtake_l[i] = 0.0;
	}

//	master_scale = {10.3750000000000,
//		12.7500000000000,
//		23.3750000000000,
//		35.6250000000000,
//		46.5000000000000,
//		48.3750000000000,
//		44.0,
//		44.0,
//		40.5000000000000,
//		40.5000000000000,
//		34.7500000000000,
//		29.3750000000000,
//		24.5000000000000,
//		24.5000000000000,
//		17.7500000000000,
//		15.8750000000000,
//		14.1250000000000
//	};

//	double master_switch_arr[] = { 2.62500000000000, 2.62500000000000,
//			4.25000000000000, 5.50000000000000, 7.12500000000000,
//			8.12500000000000, 8.25000000000000, 8.37500000000000,
//			8.37500000000000, 8, 7.50000000000000, 6.75000000000000,
//			6.50000000000000, 6.37500000000000, 6.25000000000000,
//			6.25000000000000, 6.37500000000000, 6.25000000000000,
//			5.87500000000000, 5.25000000000000, 4.62500000000000,
//			4.25000000000000, 3.75000000000000, 3.50000000000000,
//			3.25000000000000, 3.12500000000000, 3.25000000000000,
//			3.37500000000000, 3.12500000000000, 2.75000000000000,
//			2.75000000000000, 2.75000000000000, 2.50000000000000,
//			2.50000000000000, 2.50000000000000, 2.50000000000000,
//			2.50000000000000, 2.50000000000000, 2.12500000000000,
//			2.50000000000000, 2.50000000000000, 2.12500000000000,
//			2.12500000000000, 2.12500000000000, 2.12500000000000,
//			2.12500000000000, 2.12500000000000, 2.12500000000000,
//			2.12500000000000, 2.12500000000000 };
////
//	master_switch[0] = 10.3750000000000;
//	master_switch[1] = 12.7500000000000;
//	master_switch[2] = 23.3750000000000;
//	master_switch[3] = 35.6250000000000;
//	master_switch[4] = 46.5000000000000;
//	master_switch[5] = 48.3750000000000;
//	master_switch[6] = 44;
//	master_switch[7] = 44;
//	master_switch[8] = 40.5000000000000;
//	master_switch[9] = 40.5000000000000;
//	master_switch[10] = 34.7500000000000;
//	master_switch[11] = 29.3750000000000;
//	master_switch[12] = 24.5000000000000;
//	master_switch[13] = 24.5000000000000;
//	master_switch[14] = 17.7500000000000;
//	master_switch[15] = 15.8750000000000;
//	master_switch[16] = 14.1250000000000;
//	master_switch[17] = 12.750000000000;
//	master_switch[18] = 11.750000000000;
//	master_switch[19] = 10.8750000000000;
//	master_switch[20] = 9.62500000000000;
//	master_switch[21] = 9.0000000000;
//	master_switch[22] = 8.50000000000;
//	master_switch[23] = 8.250000000000;
//	master_switch[24] = 8.0000000000;
//	master_switch[25] = 8.2500000000000;
//	master_switch[26] = 7.870000000000;
//	master_switch[27] = 7.870000000000;
//	master_switch[28] = 8.0000000000;
//	master_switch[29] = 8.250000000000;
////
//	master_slow[0] = 10.3750000000000;
//	master_slow[1] = 12.7500000000000;
//	master_slow[2] = 23.3750000000000;
//	master_slow[3] = 35.6250000000000;
//	master_slow[4] = 46.5000000000000;
//	master_slow[5] = 48.3750000000000;
//	master_slow[6] = 44;
//	master_slow[7] = 44;
//	master_slow[8] = 40.5000000000000;
//	master_slow[9] = 40.5000000000000;
//	master_slow[10] = 34.7500000000000;
//	master_slow[11] = 29.3750000000000;
//	master_slow[12] = 24.5000000000000;
//	master_slow[13] = 24.5000000000000;
//	master_slow[14] = 17.7500000000000;
//	master_slow[15] = 15.8750000000000;
//	master_slow[16] = 14.1250000000000;
//	master_slow[17] = 12.750000000000;
//	master_slow[18] = 11.750000000000;
//	master_slow[19] = 10.8750000000000;
//	master_slow[20] = 9.62500000000000;
//	master_slow[21] = 9.0000000000;
//	master_slow[22] = 8.50000000000;
//	master_slow[23] = 8.250000000000;
//	master_slow[24] = 8.0000000000;
//	master_slow[25] = 8.2500000000000;
//	master_slow[26] = 7.870000000000;
//	master_slow[27] = 7.870000000000;
//	master_slow[28] = 8.0000000000;
//	master_slow[29] = 8.250000000000;

	//master_switch.setcontent(50, master_switch_arr);
//
//	for (int i = 2; i < arr_len; i++) {
//		master_scale[i] = 0.0;
//		master_slow_scale[i] = 0.0;
//		master_switch[i] = 0.0;
//		master_slow_back[i] = 0.0;
//		master_back[i] = 0.0;
//	}

}

void Intake::EnableCurrentLimits() {

	talonIntake1->EnableCurrentLimit(true);
	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake1->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake2->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake1->ConfigPeakCurrentDuration(PCD_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentDuration(PCD_WHEELS, 0);

}

void Intake::DisableCurrentLimits() {

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

}

void Intake::SetStartingPos(double start) { //not used
	starting_pos = start;
}

void Intake::InitializeIntake() {

	if (!is_init_intake) {
		SetVoltageIntake(3.5); //offset is changed accordingly //TODO: increase zeroing voltage //2.5

	}

}

void Intake::In() {

	talonIntake1->Set(ControlMode::PercentOutput, -0.95); // +2.0/12.0 maybe -0.7
	talonIntake2->Set(ControlMode::PercentOutput, 0.95); // +2.0/12.0 maybe 0.7

}

void Intake::Out() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.85); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -0.85); // -1.0

}

void Intake::Slow() {

	talonIntake1->Set(ControlMode::PercentOutput, SLOW_SPEED); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -SLOW_SPEED); // -1.0

}

void Intake::SlowScale() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.6); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -0.6); // -1.0

}

void Intake::StopWheels() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.0 - 0.2); //1.8 v
	talonIntake2->Set(ControlMode::PercentOutput, 0.0 + 0.2); //0.15

}

void Intake::ManualArm(Joystick *joyOpArm) {

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	output *= 12.0;

	SetVoltageIntake(output);

}

void Intake::Rotate() { //a vector of a pos vector and a vel vector

	//top is position, bottom is velocity

	if (intake_arm_state != STOP_ARM_STATE_H
			&& intake_arm_state != INIT_STATE_H) {

		std::vector<std::vector<double> > ref_intake =
				intake_profiler->GetNextRefIntake();

		double current_pos = GetAngularPosition();
		double current_vel = GetAngularVelocity();

		///	SmartDashboard::PutNumber("IntakeActualVel", current_vel);
		//	SmartDashboard::PutNumber("IntakeActualPos", current_pos);

		double goal_pos = ref_intake[0][0];
		double goal_vel = ref_intake[1][0];

		//	SmartDashboard::PutNumber("IntakeGoalVel", goal_vel);
		//	SmartDashboard::PutNumber("IntakeGoalPos", goal_pos);

		error_i[0][0] = goal_pos - current_pos;
		error_i[1][0] = goal_vel - current_vel;

		v_bat_i = 12.0;

		if (intake_profiler->GetFinalGoalIntake()
				< intake_profiler->GetInitPosIntake()) {
			K_i = K_down_i;
		} else {
			K_i = K_up_i;
		}

//	if (IsAtBottomIntake() && std::abs(error_i[0][0]) > 0.4) { //shaking
//		intake_arm_state = DOWN_STATE;
//	}

		double offset = 1.3 * cos(current_pos); //counter gravity

		u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0])
				+ (Kv_i * goal_vel * v_bat_i) * ff_percent_i + offset; // for this system the second row of the K matrix is a copy and does not matter.

		SetVoltageIntake(u_i);
	}

}

void Intake::SetVoltageIntake(double voltage_i) {

	SmartDashboard::PutString("INTAKE SAFETY", "none");

	double ang_pos = GetAngularPosition();

	//top soft limits
	if (elevator_i->GetElevatorPosition() < elevator_safety_position) {
		if (ang_pos >= (1.6) && voltage_i > 0.0 && is_init_intake) { //at max height and still trying to move up //there is no upper soft limit when initializing
			voltage_i = 0.0;
			SmartDashboard::PutString("INTAKE SAFETY", "top soft limit");
		}
	} else {
		if (ang_pos >= (INTAKE_BACKWARDS_SOFT_LIMIT) && voltage_i > 0.0
				&& is_init_intake) { //at max height and still trying to move up //no upper soft limit when initializing
			voltage_i = 0.0;
			SmartDashboard::PutString("INTAKE SAFETY", "top soft limit");
		}
	}

	//zero height moves up sometimes. this will make sure the arm goes all the way down every time
	if (intake_arm_state == DOWN_STATE && GetAngularPosition() <= 0.2) {
		voltage_i = 0.0;
	}

	//safety to make sure that the elevator doesn't go down when the arm is up
	if (ang_pos > 1.7 && elevator_i->GetVoltageElevator() < 0.0) { //checking and changing u_e
		elevator_i->keep_elevator_up = true;
	} else {
		elevator_i->keep_elevator_up = false;
	}

	if (talonIntakeArm->GetOutputCurrent() > ZERO_CURRENT) { //probably don't need this current check
		counter_i++;
		if (counter_i > 1) {
			if (ZeroEnc()) { //successfully zeroed enc one time
				is_init_intake = true;
			}
		}
	} else {
		counter_i = 0;
	}
	if (voltage_i < 0.0 && ang_pos < -0.1) {
		voltage_i = 0.0;
		SmartDashboard::PutString("INTAKE SAFETY", "bottom soft limit");
	}

	//voltage limit
	if (voltage_i > MAX_VOLTAGE_I) {
		voltage_i = MAX_VOLTAGE_I;
	} else if (voltage_i < MIN_VOLTAGE_I) {
		voltage_i = MIN_VOLTAGE_I;
	}

	if (std::abs(GetAngularVelocity()) <= 0.05 && std::abs(voltage_i) > 3.0) { //outputting current, not moving, should be movingGetAngularVelocity()
		encoder_counter++;
	} else {
		encoder_counter = 0;
	}

	if (encoder_counter > 10) {
		voltage_safety = true;
	} else {
		voltage_safety = false;
	}

	if (voltage_safety) {
		SmartDashboard::PutString("INTAKE SAFETY", "stall");
		voltage_i = 0.0;
	}

	voltage_i /= 12.0; //scale from -1 to 1 for the talon

	voltage_i *= -1.0; //set AT END

	talonIntakeArm->Set(ControlMode::PercentOutput, voltage_i);

}

double Intake::GetAngularVelocity() {

	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel = (talonIntakeArm->GetSelectedSensorVelocity(0)
			/ (TICKS_PER_ROT_I)) * (2.0 * PI) * (10.0) * -1.0;
	//double ang_vel = 0.0;

	return ang_vel;

}

double Intake::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos =

	((talonIntakeArm->GetSelectedSensorPosition(0) - position_offset)//position offset
	/ (TICKS_PER_ROT_I)) * (2.0 * PI) * -1.0;
	//double ang_pos = 0.0;

	//double offset_angle = 1.5; //amount that the arm will stick up in radians// the top angle. greater offset = lower 0

	return ang_pos + offset_angle;//the angular position from the encoder plus the angle when we zero minus the offset for zeroing

}

bool Intake::IsAtBottomIntake() {
	if (!hallEffectIntake->Get()) {
		return true;
	} else {
		return false;
	}
}

bool Intake::IsAtAngle(double target_ang) {

	if (std::abs(GetAngularPosition() - target_ang) < 0.1) {
		return true;
	}
	return false;

}

void Intake::ManualWheels(Joystick *joyOpWheels) {

//	SmartDashboard::PutNumber("WHEELS CUR", talonIntake1->GetOutputCurrent());
//	double enc = (talonIntake1->GetSensorCollection().GetQuadraturePosition()
//			/ 4096.0) * 2.0 * PI; //change this
//	SmartDashboard::PutNumber("WHEELS ENC", enc);

	double out = joyOpWheels->GetRawAxis(3) * -1.0;

	talonIntake1->Set(ControlMode::PercentOutput, out);

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case INIT_STATE:
		SmartDashboard::PutString("IA", "INIT");
		if (is_init_intake) {
			intake_arm_state = UP_STATE; //PUT BACK IN
		} else {
			InitializeIntake();
		}
		last_intake_state = INIT_STATE;
		break;

	case UP_STATE:
		SmartDashboard::PutString("IA", "UP");
		//	SmartDashboard::PutString("actually in up state", "yep");
		if (last_intake_state != UP_STATE) { //first time in state
			intake_profiler->SetFinalGoalIntake(UP_ANGLE); //is 0.0 for testing
			intake_profiler->SetInitPosIntake(GetAngularPosition()); //is 0
		}
		last_intake_state = UP_STATE;
		break;

	case MID_STATE:
		SmartDashboard::PutString("IA", "MID");
		if (last_intake_state != MID_STATE) {
			intake_profiler->SetFinalGoalIntake(MID_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = MID_STATE;
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("IA", "DOWN");
		if (last_intake_state != DOWN_STATE) {
			intake_profiler->SetFinalGoalIntake(DOWN_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = DOWN_STATE;
		break;

	case STOP_ARM_STATE: //for emergencies
		SmartDashboard::PutString("IA", "STOP");
		StopArm();
		last_intake_state = STOP_ARM_STATE;
		break;

	case SWITCH_STATE: //these last two were in wrong order
		SmartDashboard::PutString("IA", "SWITCH");
		if (last_intake_state != SWITCH_STATE) {
			intake_profiler->SetFinalGoalIntake(SWITCH_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = SWITCH_STATE;
		break;

	case SWITCH_BACK_SHOT_STATE:
		SmartDashboard::PutString("IA", "DOWN");
		if (last_intake_state != SWITCH_BACK_SHOT_STATE) {
			intake_profiler->SetFinalGoalIntake(BACK_SHOT_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = SWITCH_BACK_SHOT_STATE;
		break;
	}


}

void Intake::IntakeWheelStateMachine() {

	switch (intake_wheel_state) {

	case STOP_WHEEL_STATE: //has offset, not actually stopped
		SmartDashboard::PutString("IW", "STOP");
		if (last_intake_wheel_state != STOP_WHEEL_STATE) {
			EnableCurrentLimits();
		}
		StopWheels();
		last_intake_wheel_state = STOP_WHEEL_STATE;
		break;

	case IN_STATE:
		SmartDashboard::PutString("IW", "IN");
		if (last_intake_wheel_state != IN_STATE) {
			EnableCurrentLimits();
		}
		In();
		last_intake_wheel_state = IN_STATE;
		break;

	case OUT_STATE:
		SmartDashboard::PutString("IW", "OUT");
		if (last_intake_wheel_state != OUT_STATE) {
			DisableCurrentLimits();
		}
		Out();
		last_intake_wheel_state = OUT_STATE;
		break;

	case SLOW_STATE:
		SmartDashboard::PutString("IW", "SLOW");
		if (last_intake_wheel_state != SLOW_STATE) {
			DisableCurrentLimits();
		}
		Slow();
		last_intake_wheel_state = SLOW_STATE;
		break;

	case SLOW_SCALE_STATE:
		SmartDashboard::PutString("IW", "SLOW");
		if (last_intake_wheel_state != SLOW_SCALE_STATE) {
			DisableCurrentLimits();
		}
		SlowScale();
		last_intake_wheel_state = SLOW_SCALE_STATE;
		break;

	}

}

bool Intake::HaveCube() {

	for (int i = 0; i < (sample_window_intake - 2); i++) { //to index 18
		currents_intake[i] = currents_intake[i + 1];
	}

	currents_intake[sample_window_intake - 1] =
			talonIntake1->GetOutputCurrent();

	alglib::corrr1d(currents_intake, arr_len, master_scale, arr_len,
			corr_intake);

	if (FindMaximum(corr_intake) > INTAKE_CORR_VALUE) { //just once, and will go through
		start_counting = true;
	}

	if (start_counting) {
		have_cube_wait++;
		if (have_cube_wait == 25) { ///////////////////////
			have_cube_wait = 0;
			start_counting = false;
			for (int i = 0; i < (sample_window_intake - 1); i++) { //to index 18
				currents_intake[i] = 0;
			}
			return true;
		} else {
			return false; //but keep counting up
		}
	} else {
		return false;
	}

}

bool Intake::ReleasedCube(int shot_type) { //forward scale or backwards scale. is irrelevant for switch
//75 for fast, 1 for slow
	time_counter++; //depends on calling this function when need to start the timer

	for (int i = 0; i < (sample_window_outtake - 2); i++) { //to index 18
		currents_outtake_r[i] = currents_outtake_r[i + 1];
		currents_outtake_l[i] = currents_outtake_l[i + 1];
	}

	///std::cout << "in the method" << std::endl;

	currents_outtake_r[sample_window_outtake - 1] =
			talonIntake1->GetOutputCurrent();
	currents_outtake_l[sample_window_outtake - 1] =
			talonIntake2->GetOutputCurrent();

	if (shot_type == SCALE) { //scale, backwards intake_arm_state == OUT_STATE && forward
//		alglib::corrr1d(currents_outtake_r, arr_len, master_switch, arr_len,
//				corr_outtake_r);
//		alglib::corrr1d(currents_outtake_l, arr_len, master_switch, arr_len,
//				corr_outtake_l);
		for (int i = 0; i < (arr_len - 2); i++) {
			corr_outtake_r[i] = 0.0;
			corr_outtake_l[i] = 0.0;
		}
		TIME_LIMIT = 42;
	} else if (shot_type == SWITCH) { //switch intake_arm_state == SLOW_STATE && forward
		//std::cout << "switch" << std::endl;
		for (int i = 0; i < (arr_len - 2); i++) {
			corr_outtake_r[i] = 0.0;
			corr_outtake_l[i] = 0.0;
		}
		TIME_LIMIT = 42;
//		alglib::corrr1d(currents_outtake_r, arr_len, master_scale, arr_len,
//				corr_outtake_r);
//		alglib::corrr1d(currents_outtake_l, arr_len, master_scale, arr_len,
//				corr_outtake_l);
	} else if (shot_type == SLOW_SCALE) {
//		alglib::corrr1d(currents_outtake_r, arr_len, master_slow_scale, arr_len,
//				corr_outtake_r);
//		alglib::corrr1d(currents_outtake_l, arr_len, master_slow_scale, arr_len,
//				corr_outtake_l);
		for (int i = 0; i < (arr_len - 2); i++) {
			corr_outtake_r[i] = 0.0;
			corr_outtake_l[i] = 0.0;
		}
		TIME_LIMIT = 50;
	} else if (shot_type == BACK) {
//		alglib::corrr1d(currents_outtake_r, arr_len, master_scale, arr_len, //back
//				corr_outtake_r);
//		alglib::corrr1d(currents_outtake_l, arr_len, master_scale, arr_len,
//				corr_outtake_l);
		for (int i = 0; i < (arr_len - 2); i++) {
			corr_outtake_r[i] = 0.0;
			corr_outtake_l[i] = 0.0;
		}

		TIME_LIMIT = 30;
	} else if (shot_type == SLOW_BACK) {
//		alglib::corrr1d(currents_outtake_r, arr_len, master_scale, arr_len, //slow_back
//				corr_outtake_r);
//		alglib::corrr1d(currents_outtake_l, arr_len, master_scale, arr_len,
//				corr_outtake_l);
		for (int i = 0; i < (arr_len - 2); i++) {
			corr_outtake_r[i] = 0.0;
			corr_outtake_l[i] = 0.0;
		}

		TIME_LIMIT = 50;
	}

	//std::cout << "corr: " << FindMaximum(corr_outtake_r) << ", "<< FindMaximum(corr_outtake_l) << std::endl;

	corr_val_now_r = corr_outtake_r[(int) (arr_len / 2)]; //FindMaximum(corr_outtake_r); //returns how correlated the actual currents are with the master currents. since the master dataset ends
	corr_val_now_l = corr_outtake_l[(int) (arr_len / 2)]; //FindMaximum(corr_outtake_l); //returns how correlated the actual currents are with the master currents. since the master dataset ends

	//std::cout << "corr val: " << (corr_val_now_r - last_corr_val_r) << std::endl;

	if (((last_corr_val_l > corr_val_now_l || last_corr_val_r > corr_val_now_r)) //if the arrays match close enough (corr_val_now_r > OUTTAKE_CORR_VALUE
	//|| corr_val_now_l > OUTTAKE_CORR_VALUE)
	///&&
			|| (time_counter > TIME_LIMIT)) {
		//	std::cout << "last corr val: " << last_corr_val_r << std::endl;
		for (int i = 0; i < (sample_window_outtake - 1); i++) {
			currents_outtake_r[i] = 0.0;
			currents_outtake_l[i] = 0.0;
		}
		time_counter = 0;
		last_corr_val_r = 0.0;
		last_corr_val_l = 0.0;
		return true;
	} else {
		last_corr_val_r = corr_val_now_r;
		last_corr_val_l = corr_val_now_l;
		return false;
	}

}

double Intake::FindMaximum(alglib::real_1d_array corr) {

	int max = -1;

	for (int i = 0; i < (arr_len - 3); i++) { //only works for corr
		if (std::abs(corr[i]) > max) {
			max = std::abs(corr[i]);
		}
	}

	return max;

}

void Intake::SetZeroOffset() {

	position_offset = (talonIntakeArm->GetSelectedSensorPosition(0));
}

bool Intake::ZeroEnc() { //called in Initialize() and in SetVoltage()

	if (zeroing_counter_i < 1) {
		SetZeroOffset();
		zeroing_counter_i++;
		return true;
	} else {
		//std::cout << "here" << std::endl;
		return false;
	}

}
