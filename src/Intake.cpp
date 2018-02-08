/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//NEW K GAINS
#include <Intake.h>
#include <ctre/Phoenix.h> //double included
#include <WPILib.h>

#define PI 3.14159265

using namespace std::chrono;

const int UP_STATE = 0; //arm state machine
const int MID_STATE = 1;
const int DOWN_STATE = 2;
const int STOP_ARM_STATE = 3;

const int STOP_WHEEL_STATE = 0; //wheel state machine
const int IN_STATE = 1;
const int OUT_STATE = 2;

const double TICKS_PER_ROT_I = 4096.0; //possibly not
const double MAX_VOLTAGE_I = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_I = -12.0;

const double free_speed_i = 18730.0; //rpm
const double G_i = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i)) * 2.0 * PI
		* 10.0; //rad/s
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

//For motion profiler
const double MAX_VELOCITY_I = 2.3;
const double MAX_ACCELERATION_I = 5.0;
const double TIME_STEP_I = 0.01; //sec

const double MAX_INTAKE_CURRENT = 50.0;

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 1.2;
const double UP_ANGLE = 0.0;

int last_intake_state = 1; //cannot equal the first state or profile will not set the first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 0.0; //will be set to pdp's voltage

std::vector<std::vector<double> > K_i = { { 23.6, 0.11 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 23.6, 0.11 } };

std::vector<std::vector<double> > X_i = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;

MotionProfiler *intake_profiler;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
int counter_i = 0;

Intake::Intake(PowerDistributionPanel *pdp, MotionProfiler *intake_profiler_) {

	intake_profiler = intake_profiler_;

	intake_profiler->SetMaxAcc(MAX_ACCELERATION_I);
	intake_profiler->SetMaxVel(MAX_VELOCITY_I);

	hallEffectIntake = new DigitalInput(0); //will return 0 if arm is at bottom

	talonIntake1 = new TalonSRX(2);

	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(55);
	talonIntakeArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonIntakeArm->ConfigPeakCurrentLimit(20, 0); //for now
	talonIntake1->ConfigPeakCurrentLimit(40, 0);
	talonIntake2->ConfigPeakCurrentLimit(40, 0);

	pdp_i = pdp;

}

void Intake::In() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.7); // +2.0/12.0 maybe

}

void Intake::Out() {

	talonIntake1->Set(ControlMode::PercentOutput, -0.7); // +2.0/12.0 maybe

}

void Intake::StopWheels() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.0);

}

double Intake::GetAngularVelocity() {

	//Native vel units are in ticks per 100ms so divide by TICKS_PER_ROT to get rotations per 100ms then multiply 10 to get per second
	//multiply by 2pi to get into radians per second (2pi radians are in one revolution)
	double ang_vel = (talonIntakeArm->GetSelectedSensorVelocity(0)
			/ (TICKS_PER_ROT_I)) * (2.0 * PI) * (10.0) * -2.0;

	return ang_vel;

}

double Intake::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos = (talonIntakeArm->GetSelectedSensorPosition(0.0)
			/ (TICKS_PER_ROT_I)) * (2.0 * PI) * -2.0;

	return ang_pos;

}

void Intake::ManualWheels(Joystick *joyOpWheels) {

	SmartDashboard::PutNumber("WHEELS CUR", talonIntake1->GetOutputCurrent());
	double enc = (talonIntake1->GetSelectedSensorPosition(0) / 4096.0) * 2.0
			* PI; //change this
	SmartDashboard::PutNumber("WHEELS ENC", enc);

	double out = joyOpWheels->GetRawAxis(3) * -1.0;

	talonIntake1->Set(ControlMode::PercentOutput, out);

}

void Intake::ManualArm(Joystick *joyOpArm) {

	SmartDashboard::PutNumber("ARM CUR", talonIntakeArm->GetOutputCurrent());
	double enc_arm = ((talonIntakeArm->GetSelectedSensorPosition(0)) / 4096.0)
			* 2.0 * PI * -1.0;

	SmartDashboard::PutNumber("ARM ENC", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	//std::cout << "output " << output << std::endl;

	//std::cout << "COUNT " << counter_i << std::endl;

	//counter_i = 0;

	output *= pdp_i->GetVoltage();

	//last_is_at_bottom = is_at_bottom;

	SmartDashboard::PutNumber("ARM OUTPUT", output);

	SetVoltageIntake(output);

}

void Intake::SetVoltageIntake(double voltage_i) {

	is_at_bottom = !hallEffectIntake->Get(); //hall effect returns 0 when at bottom. we reverse it here
	SmartDashboard::PutNumber("HALL EFF", is_at_bottom); //actually means not at bottom //0 means up// 1 means down

	//soft limit
	if (GetAngularPosition() >= (PI / 2.0) && voltage_i > 0.0) { //at max height and still trying to move up
		voltage_i = 0.0;
	}

	//voltage limit
	if (voltage_i > MAX_VOLTAGE_I) {
		voltage_i = MAX_VOLTAGE_I;
	} else if (voltage_i < MIN_VOLTAGE_I) {
		voltage_i = MIN_VOLTAGE_I;
	}

	if (is_at_bottom) {
		if (counter_i == 0) { //first time at bottom
			ZeroEnc();
			counter_i++;
		}
		if (voltage_i < 0.0) {
			voltage_i = 0.0;
		}
	} else {
		counter_i = 0;
	}

	SmartDashboard::PutNumber("VOLTAGE", voltage_i);

	voltage_i /= pdp_i->GetVoltage(); //scale from -1 to 1 for the talon

	voltage_i *= -1.0; //set AT END

	talonIntakeArm->Set(ControlMode::PercentOutput, voltage_i);

}

void Intake::Rotate(std::vector<std::vector<double> > ref_intake) {

	//top is position, bottom is velocity

	double current_pos = GetAngularPosition();
	double current_vel = GetAngularVelocity();
	double goal_pos = ref_intake[0][0];
	double goal_vel = ref_intake[1][0];

	SmartDashboard::PutNumber("INTAKE POS", current_pos);
	SmartDashboard::PutNumber("INTAKE VEL", current_vel);

	SmartDashboard::PutNumber("INTAKE REF POS", goal_pos);
	SmartDashboard::PutNumber("INTAKE REF VEL", goal_vel);

	error_i[0][0] = goal_pos - current_pos;
	error_i[1][0] = goal_vel - current_vel;

	SmartDashboard::PutNumber("INTAKE ERR POS", error_i[0][0]);
	SmartDashboard::PutNumber("INTAKE ERR VEL", error_i[1][0]);

	v_bat_i = pdp_i->GetVoltage();

	u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0]) //u_i is in voltage, * by v_bat_i
			+ (Kv_i * goal_vel * v_bat_i); // for this system the second row of the K matrix is a copy and does not matter.

	SmartDashboard::PutNumber("INTAKE OUTPUT", u_i);

	SetVoltageIntake(u_i);

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		if (last_intake_state != UP_STATE) { //first time in state
			intake_profiler->SetMaxAcc(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVel(MAX_VELOCITY_I);
			intake_profiler->SetFinalGoal(UP_ANGLE);
			intake_profiler->SetInitPos(GetAngularPosition()); //is 0
			//std::cout << "HERE" << std::endl;
			//SmartDashboard::PutString("IN RESET TO DOWN", "YES");
		}
		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");
		if (last_intake_state != MID_STATE) {
			intake_profiler->SetMaxAcc(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVel(MAX_VELOCITY_I);
			intake_profiler->SetInitPos(GetAngularPosition());
			intake_profiler->SetFinalGoal(MID_ANGLE);
		}
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if (last_intake_state != DOWN_STATE) {
			intake_profiler->SetMaxAcc(MAX_ACCELERATION_I); //these must be reset in each state
			intake_profiler->SetMaxVel(MAX_VELOCITY_I);
			intake_profiler->SetFinalGoal(DOWN_ANGLE);
			intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case STOP_ARM_STATE:
		SmartDashboard::PutString("INTAKE ARM", "STOP");
		StopArm();
		break;
	}

	last_intake_state = intake_arm_state;

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
		Out();
		break;

	}

}

bool Intake::EncodersRunning() { //will stop the controller from run

	double current_pos = GetAngularPosition(); //radians
	double current_ref = intake_profiler->GetNextRef().at(0).at(0);

//	if (talonIntakeArm->GetOutputCurrent() > 3.0
//			&& std::abs(talonIntakeArm->GetSelectedSensorVelocity(0)) <= 0.2
//			&& std::abs(current_ref - current_pos) > 0.2) { //outputting current, not moving, should be moving
//		return false;
//	}
	return true;
}

bool Intake::HaveCube() {

	if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
		return true;
	} else {
		return false;
	}

}

void Intake::ZeroEnc() {

	talonIntakeArm->SetSelectedSensorPosition(0, 0, 0);
	is_arm_init = true;

}

void Intake::IntakeWrapper(Intake *in) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_intake =
						intake_profiler->GetNextRef();

//				std::cout << "pos: " << profile_intake.at(0).at(0) << "  "
//						<< "vel: " << profile_intake.at(1).at(0) << "   "
//						<< "acc: " << profile_intake.at(2).at(0) << std::endl;

				if (in->intake_arm_state != STOP_ARM_STATE) {
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
