/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//ARM ENCODER HAS OFFSET: We set 0.35 to 0.0
//is_init_intake = true moved to ZeroEnc

//fix current limit

#include <Intake.h>
#include <ctre/Phoenix.h> //double included
#include <WPILib.h>

#define PI 3.14159265

using namespace std::chrono; //for thread

const int INIT_STATE = 0; //arm state machine
const int UP_STATE = 1;
const int MID_STATE = 2;
const int DOWN_STATE = 3;
const int STOP_ARM_STATE = 4;

const int STOP_WHEEL_STATE = 0; //wheel state machine
const int IN_STATE = 1;
const int OUT_STATE = 2;

const int HALL_EFF_INTAKE_ID = 0;
const int INTAKE_1_ID = 17;
const int INTAKE_2_ID = 25;

const double TICKS_PER_ROT_I = 4096.0;
const double MAX_VOLTAGE_I = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_I = -12.0;

const double free_speed_i = 18730.0; //rpm
const double G_i = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i)) * (2.0 * PI)
		/ 60.0; //rad/s
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

//For motion profiler
const double MAX_VELOCITY_I = 1.0; //rad/s
const double MAX_ACCELERATION_I = 2.5; //rad/s/s
const double TIME_STEP_I = 0.01; //sec

const double UP_LIMIT_I = 1.58;
const double DOWN_LIMIT_I = 0.0;

const double UP_VOLT_LIMIT_I = 0.0;
const double DOWN_VOLT_LIMIT_I = 0.0;

const double WHEEL_OFFSET = 0.2;
const double ARM_OFFSET = 1.3;

const double STALL_VEL_I = 0.05;
const double STALL_VOLT_I = 4.0;

const double MAX_INTAKE_CURRENT = 50.0;

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

const double DOWN_ANGLE = 0.1;
const double MID_ANGLE = 0.6;
const double UP_ANGLE = 1.3; //starting pos

double current_pos = 0.0;
double current_vel = 0.0;
double goal_pos = 0.0;
double goal_vel = 0.0;
double arm_offset = 0.0;

int last_intake_state = 0; //cannot equal the first state or profile will not set the first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 12.0;

std::vector<std::vector<double> > K_i;
std::vector<std::vector<double> > K_down_i = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_i = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 10.32, 0.063 } };

std::vector<std::vector<double> > X_i = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;

IntakeMotionProfiler *intake_profiler;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

int init_counter_i = 0;
int counter_i = 0;
int i = 0;
int encoder_counter = 0;

Intake::Intake(PowerDistributionPanel *pdp,
		IntakeMotionProfiler *intake_profiler_) {

	intake_profiler = intake_profiler_;

	intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I);
	intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);

	hallEffectIntake = new DigitalInput(HALL_EFF_INTAKE_ID);

	talonIntake1 = new TalonSRX(INTAKE_1_ID);
	talonIntake2 = new TalonSRX(INTAKE_2_ID);
	talonIntake2->Set(ControlMode::Follower, INTAKE_1_ID);

//	talonIntake1->EnableCurrentLimit(true);
//	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(5, 0); //20
	talonIntake2->ConfigPeakCurrentLimit(5, 0);
	talonIntake1->ConfigContinuousCurrentLimit(5, 0); //20
	talonIntake2->ConfigContinuousCurrentLimit(5, 0);

	talonIntakeArm = new TalonSRX(55);

	talonIntakeArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	//talonIntakeArm->EnableCurrentLimit(true);
	talonIntakeArm->ConfigPeakCurrentLimit(20, 0); //for now
	talonIntakeArm->ConfigContinuousCurrentLimit(20, 0); //for now

	pdp_i = pdp;

}

void Intake::InitializeIntake() {

	//ZeroEnc();

	//if (!IsAtBottomIntake()) { //don't see hall effect
	if (!is_init_intake) { //this has to be here for some reason
		SetVoltageIntake(-1.5); //double intake_volt = (2.0 / pdp_i->GetVoltage()) * 1.0; //down//SetVoltageIntake(-1.0);
		//talonIntakeArm->Set(ControlMode::PercentOutput, intake_volt);
	}

	//double up_volt = (0.2 * -1.0) / pdp_e->GetVoltage(); //to not crash down
	//talonElevator1->Set(ControlMode::PercentOutput, up_volt);
	//talonElevator2->Set(ControlMode::PercentOutput, up_volt);
}

void Intake::In() {

	//std::cout << "INTAKE IN" << std::endl;
	//SmartDashboard::PutNumber("BAT VOLT", pdp_i->GetVoltage());

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, -1.0); // +2.0/12.0 maybe -0.7
	talonIntake2->Set(ControlMode::PercentOutput, 1.0); // +2.0/12.0 maybe 0.7

}

void Intake::Out() {

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, 1.0); // +2.0/12.0 maybe
	talonIntake2->Set(ControlMode::PercentOutput, -1.0); // +2.0/12.0 maybe

}

void Intake::StopWheels() {

	talonIntake1->EnableCurrentLimit(true); //current limit might not actually work properly, but looks fine from looking at prints
	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(5, 0); //20
	talonIntake2->ConfigPeakCurrentLimit(5, 0);
	talonIntake1->ConfigContinuousCurrentLimit(5, 0); //20
	talonIntake2->ConfigContinuousCurrentLimit(5, 0);

	talonIntake1->Set(ControlMode::PercentOutput, 0.0 - WHEEL_OFFSET); //1.8 v
	talonIntake2->Set(ControlMode::PercentOutput, 0.0 + WHEEL_OFFSET); //0.15

}

void Intake::ManualArm(Joystick *joyOpArm) {

	//SmartDashboard::PutNumber("ARM CUR", talonIntakeArm->GetOutputCurrent());
	//double enc_arm = GetAngularPosition();
	//SmartDashboard::PutNumber("ARM ENC",
	//		talonIntakeArm->GetSensorCollection().GetQuadraturePosition());

	//SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	output *= MAX_VOLTAGE_I;

	//SmartDashboard::PutNumber("ARM OUTPUT", output);

	SetVoltageIntake(output);

}

void Intake::Rotate(std::vector<std::vector<double> > ref_intake) {

	//top is position, bottom is velocity

	current_pos = GetAngularPosition();
	current_vel = GetAngularVelocity();
	goal_pos = ref_intake[0][0];
	goal_vel = ref_intake[1][0];

	SmartDashboard::PutNumber("INTAKE POS", current_pos);
	SmartDashboard::PutNumber("INTAKE VEL", current_vel);

	SmartDashboard::PutNumber("INTAKE REF POS", goal_pos);
	SmartDashboard::PutNumber("INTAKE REF VEL", goal_vel);

	error_i[0][0] = goal_pos - current_pos;
	error_i[1][0] = goal_vel - current_vel;

	SmartDashboard::PutNumber("INTAKE ERR POS", error_i[0][0]);
	SmartDashboard::PutNumber("INTAKE ERR VEL", error_i[1][0]);

	if (intake_profiler->final_goal_i < current_pos) { //must use final ref, to account for getting slightly ahead of the profiler
		K_i = K_down_i;
	} else {
		K_i = K_up_i;
	}

//	if (IsAtBottomIntake() && std::abs(error_i[0][0]) > 0.4) { //shaking
//		intake_arm_state = DOWN_STATE;
//	}

	arm_offset = ARM_OFFSET * (double) cos(current_pos); //counter gravity when needed because of slack on the arm

	u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0]) //u_i is in voltage, so * by v_bat_i
			+ (Kv_i * goal_vel * v_bat_i) + arm_offset;

	SmartDashboard::PutNumber("INTAKE CONT VOLT", u_i);

	SetVoltageIntake(u_i);

}

void Intake::SetVoltageIntake(double voltage_i) {

	is_at_bottom = IsAtBottomIntake(); //hall effect returns 0 when at bottom. we reverse it here
	SmartDashboard::PutNumber("INTAKE HALL EFF", is_at_bottom); //actually means not at bottom //0 means up// 1 means dow

	SmartDashboard::PutString("INTAKE safety", "none");

	//soft limit top
	if (GetAngularPosition() >= (UP_LIMIT_I) && voltage_i > UP_VOLT_LIMIT_I) { //at max height and still trying to move up
		voltage_i = 0.0; //shouldn't crash
		SmartDashboard::PutString("INTAKE safety", "top soft limit");
	}

	//soft limit bottom
	if (is_at_bottom) {
		ZeroEnc(); //will not run after 2nd time (first time is in teleop init)
		if (GetAngularPosition() < (DOWN_LIMIT_I) && voltage_i < DOWN_VOLT_LIMIT_I) { //but hall effect goes off at 0.35
			voltage_i = 0.0;
			SmartDashboard::PutString("INTAKE safety", "bot hall eff");
		}
	}

//	} else {
//		counter_i = 0;
//	}

	//voltage limit
	if (voltage_i > MAX_VOLTAGE_I) {
		voltage_i = MAX_VOLTAGE_I;
		SmartDashboard::PutString("INTAKE safety", "clipped");
	} else if (voltage_i < MIN_VOLTAGE_I) {
		voltage_i = MIN_VOLTAGE_I;
		SmartDashboard::PutString("INTAKE safety", "clipped");
	}

	//stall
	if (std::abs(GetAngularVelocity()) <= STALL_VEL_I && std::abs(voltage_i) > STALL_VOLT_I) { //outputting current, not moving, should be moving
		encoder_counter++;
	} else {
		encoder_counter = 0;
	}
	if (encoder_counter > 10) {
		voltage_safety = true;
		voltage_i = 0.0;
		SmartDashboard::PutString("INTAKE safety", "stall");
	}

	SmartDashboard::PutNumber("INTAKE VOLTAGE", voltage_i);

	//scale
	voltage_i /= MAX_VOLTAGE_I; //scale from -1 to 1 for the talon // max voltage is positive

	//reverse
	voltage_i *= -1.0; //set AT END

	talonIntakeArm->Set(ControlMode::PercentOutput, voltage_i);

}

double Intake::GetAngularVelocity() {

	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel =
			(talonIntakeArm->GetSensorCollection().GetQuadratureVelocity()
					/ (TICKS_PER_ROT_I)) * (2.0 * PI) * (10.0) * -1.0;
	//double ang_vel = 0.0;

	return ang_vel;

}

double Intake::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos =
			(talonIntakeArm->GetSensorCollection().GetQuadraturePosition()
					/ (TICKS_PER_ROT_I)) * (2.0 * PI) * -1.0;
	//double ang_pos = 0.0;

	double offset_pos = .35; //amount that the arm will stick up in radians

	return ang_pos + offset_pos;

}

bool Intake::IsAtBottomIntake() {
	if (!hallEffectIntake->Get()) {
		return true;
	} else {
		return false;
	}
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

	SmartDashboard::PutNumber("ARM ENC",
			talonIntakeArm->GetSensorCollection().GetQuadraturePosition());

	//std::cout << "intake pos " << GetAngularPosition() << std::endl;

	SmartDashboard::PutNumber("WHEEL CUR 1", talonIntake1->GetOutputCurrent());
	SmartDashboard::PutNumber("WHEEL CUR 2", talonIntake2->GetOutputCurrent());

	switch (intake_arm_state) {

	case INIT_STATE:
		SmartDashboard::PutString("INTAKE ARM", "INIT");
		InitializeIntake();
		if (is_init_intake) {
			intake_arm_state = UP_STATE;
		}
		last_intake_state = INIT_STATE;
		break;

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		//SmartDashboard::PutString("actually in up state", "yep");
		if (last_intake_state != UP_STATE) { //first time in state
			intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);
			intake_profiler->SetFinalGoalIntake(UP_ANGLE); //is 0.0 for testing
			intake_profiler->SetInitPosIntake(GetAngularPosition()); //is 0
			//std::cout << "HERE" << std::endl;
			//SmartDashboard::PutString("IN RESET TO DOWN", "YES");
		}
		last_intake_state = UP_STATE;
		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");
		if (last_intake_state != MID_STATE) {
			intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
			intake_profiler->SetFinalGoalIntake(MID_ANGLE);
		}
		last_intake_state = MID_STATE;
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if (last_intake_state != DOWN_STATE) {
			intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);
			intake_profiler->SetFinalGoalIntake(DOWN_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = DOWN_STATE;
		break;

	case STOP_ARM_STATE:
		SmartDashboard::PutString("INTAKE ARM", "STOP");
		StopArm();
		last_intake_state = STOP_ARM_STATE;
		break;
	}

	//last_intake_state = intake_arm_state; //move this into individual states if profile not switching

}

void Intake::IntakeWheelStateMachine() {

	switch (intake_wheel_state) {

	case STOP_WHEEL_STATE:
		SmartDashboard::PutString("INTAKE WHEEL", "STOP");
		StopWheels();
		break;

	case IN_STATE:
		SmartDashboard::PutString("INTAKE WHEEL", "IN");
		In();
		break;

	case OUT_STATE:
		SmartDashboard::PutString("INTAKE WHEEL", "OUT");
		//std::cout << "out state" << std::endl;
		Out();
		break;

	}

}

bool Intake::EncodersRunning() { //will stop the controller from run //or stalled //MOVED INTO SET VOLTAGE

//	double current_pos = GetAngularPosition(); //radians
//	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

	return true;
}

bool Intake::HaveCube() {

	return false; //for now
	if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
		return true;
	} else {
		return false;
	}
	return false;

}

bool Intake::ReleasedCube() {
	//will look at current, distance, velocity
	return false;
}

void Intake::ZeroEnc() { //called in Initialize() and in SetVoltage()

	if (zeroing_counter_i < 2) {
		talonIntakeArm->GetSensorCollection().SetQuadraturePosition(0, 0);
		zeroing_counter_i++;
	} else {
		is_init_intake = true;
	}

}

void Intake::IntakeWrapper(Intake *in) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_intake =
						intake_profiler->GetNextRefIntake();

				if (in->intake_arm_state != STOP_ARM_STATE
						&& in->intake_arm_state != INIT_STATE) {
					in->Rotate(profile_intake);
				}

				intakeTimer->Reset();

			}
		}
	}

}

void Intake::StartIntakeThread() {

	Intake *intake_ = this;

	IntakeThread = std::thread(&Intake::IntakeWrapper, intake_);
	IntakeThread.detach();

}

void Intake::EndIntakeThread() {

	IntakeThread.~thread();

}
