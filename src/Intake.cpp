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

#define CORNELIUS 0

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

int time_counter = 0;
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

const double OUTTAKE_CORR_VALUE = 2200.0;
const double INTAKE_CORR_VALUE = 1800.0;

const double ZERO_CURRENT = 4.0;//3.0;

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

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

std::vector<double> volts = { };

const int sample_window_intake = 40;
const int sample_window_outtake = 40;
alglib::ae_int_t arr_len = 75;
alglib::real_1d_array currents_intake;
alglib::real_1d_array currents_outtake_r;
alglib::real_1d_array currents_outtake_l;
alglib::real_1d_array master;
alglib::real_1d_array corr_intake;
alglib::real_1d_array corr_outtake_r;
alglib::real_1d_array corr_outtake_l;

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
	master.setlength(arr_len);
	corr_intake.setlength(arr_len - 2);
	corr_outtake_r.setlength(arr_len - 2);
	corr_outtake_l.setlength(arr_len - 2);

	for (int i = 0; i < arr_len; i++) {
		currents_intake[i] = 0.0;
		currents_outtake_r[i] = 0.0;
		currents_outtake_l[i] = 0.0;
	}

	master[0] = 10.3750000000000;
	master[1] = 12.7500000000000;
	master[2] = 23.3750000000000;
	master[3] = 35.6250000000000;
	master[4] = 46.5000000000000;
	master[5] = 48.3750000000000;
	master[6] = 44;
	master[7] = 44;
	master[8] = 40.5000000000000;
	master[9] = 40.5000000000000;
	master[10] = 34.7500000000000;
	master[11] = 29.3750000000000;
	master[12] = 24.5000000000000;
	master[13] = 24.5000000000000;
	master[14] = 17.7500000000000;
	master[15] = 15.8750000000000;
	master[16] = 14.1250000000000;
	master[17] = 12.750000000000;
	master[18] = 11.750000000000;
	master[19] = 10.8750000000000;
	master[20] = 9.62500000000000;
	master[21] = 9.0000000000;
	master[22] = 8.50000000000;
	master[23] = 8.250000000000;
	master[24] = 8.0000000000;
	master[25] = 8.2500000000000;
	master[26] = 7.870000000000;
	master[27] = 7.870000000000;
	master[28] = 8.0000000000;
	master[29] = 8.250000000000;

	for (int i = 30; i < arr_len; i++) {
		master[i] = 0.0;
	}

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

	talonIntake1->Set(ControlMode::PercentOutput, 1.0); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -1.0); // -1.0

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

		SmartDashboard::PutNumber("IntakeActualVel", current_vel);
		SmartDashboard::PutNumber("IntakeActualPos", current_pos);

		double goal_pos = ref_intake[0][0];
		double goal_vel = ref_intake[1][0];

		SmartDashboard::PutNumber("IntakeGoalVel", goal_vel);
		SmartDashboard::PutNumber("IntakeGoalPos", goal_pos);

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

	//last_intake_state = intake_arm_state; //move this into individual states if profile not switching

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

bool Intake::EncodersRunning() { //will stop the controller from run //or stalled //MOVE INTO SET VOLTAGE

//	double current_pos = GetAngularPosition(); //radians
//	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

	return true;
}

bool Intake::HaveCube() {

//	if (frc::RobotState::IsOperatorControl()) {
//		if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT
//				&& talonIntake2->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
//			current_counter++;
//		} else {
//			current_counter = 0;
//		}
//		if (current_counter >= 3) {
//			return true;
//		} else {
//			return false;
//		}
//	} else {
//		if (talonIntake1->GetOutputCurrent() >= 10.0
//				&& talonIntake2->GetOutputCurrent() >= 10.0) {
//			current_counter++;
//		} else {
//			current_counter = 0;
//		}
//		if (current_counter >= 3) {
//			return true;
//		} else {
//			return false;
//		}
//	}
	for (int i = 0; i < (sample_window_intake - 2); i++) { //to index 18
		currents_intake[i] = currents_intake[i + 1];
	}

	currents_intake[sample_window_intake - 1] = talonIntake1->GetOutputCurrent();

	alglib::corrr1d(currents_intake, arr_len, master, arr_len, corr_intake);

	//std::cout << "corr intake: " << FindMaximum(corr_intake) << std::endl;

	if (FindMaximum(corr_intake) > INTAKE_CORR_VALUE) {
		for (int i = 0; i < (sample_window_intake - 1); i++) { //to index 18
			currents_intake[i] = 0;
		}
		return true;
	} else {
		return false;
	}

}

bool Intake::ReleasedCube() { //TODO: change in havecube

	for (int i = 0; i < (sample_window_outtake - 2); i++) { //to index 18
		currents_outtake_r[i] = currents_outtake_r[i + 1];
		currents_outtake_l[i] = currents_outtake_l[i + 1];
	}

	currents_outtake_r[sample_window_outtake - 1] = talonIntake1->GetOutputCurrent();
	currents_outtake_l[sample_window_outtake - 1] = talonIntake2->GetOutputCurrent();

	alglib::corrr1d(currents_outtake_r, arr_len, master, arr_len, corr_outtake_r);
	alglib::corrr1d(currents_outtake_l, arr_len, master, arr_len, corr_outtake_l);


	//std::cout << "corr: " << FindMaximum(corr_outtake_r) << ", "<< FindMaximum(corr_outtake_l) << std::endl;

	if (FindMaximum(corr_outtake_r) > OUTTAKE_CORR_VALUE && FindMaximum(corr_outtake_l) > OUTTAKE_CORR_VALUE) {
		for (int i = 0; i < (sample_window_outtake - 1); i++) { //to index 18
			currents_outtake_r[i] = 0.0;
			currents_outtake_l[i] = 0.0;
		}
		return true;
	} else {
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

//std::vector<std::vector<double> > Intake::GetNextRef() {
//
//	return intake_profiler->GetNextRefIntake();
//
//}

void Intake::IntakeWrapper(Intake *in) {

	intakeTimer->Start();

	while (true) {

		intakeTimer->Reset();

		if (frc::RobotState::IsEnabled()) {

			if (in->intake_arm_state != STOP_ARM_STATE
					&& in->intake_arm_state != INIT_STATE) {
				//	in->Rotate(in->GetNextRef());
			}

		}

		double time = .010 - intakeTimer->Get();

		time *= 1000;
		if (time < 0) {
			time = 0;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds((int) time));

		//	std::cout << "time: " << intakeTimer->Get() << std::endl;
	}

}

void Intake::StartIntakeThread() {

	Intake *intake_ = this;

	IntakeThread = std::thread(&Intake::IntakeWrapper, intake_);
	IntakeThread.detach();

}

void Intake::EndIntakeThread() {

	//intakeTimer->Stop();
	IntakeThread.~thread();

}
