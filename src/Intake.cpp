/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Intake.h>
#include <ctre/Phoenix.h>
//#include <MotionProfiler.h>

using namespace std::chrono;

const int UP_STATE = 0;
const int MID_STATE = 1;
const int DOWN_STATE = 2;
const int STOP_ARM_STATE = 3;

const int STOP_WHEEL_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;

const double TICKS_PER_ROT = 4096.0;
const double PI = 3.14159;
const double MAX_VOLTAGE = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE = -12.0;

//to change
double free_speed = 1967; //rad/s
double m = 2.1;
double l = 0.2;
double Kt = 0.00595;
double G = ((60.0 / 1.0) * (48.0 / 38.0));

double MAX_THEORETICAL_VELOCITY = (free_speed / G);
double MAX_VELOCITY = 28.0; //choose
double MAX_ACCELERATION = 38.0; //choose
double TIME_STEP = 0.01;
double Kv_in = 1 / MAX_THEORETICAL_VELOCITY;

double u = 0; //this is the input in volts to the motor
double v_bat = 12.0; //this will be the voltage of the battery at every loop

double K[2][2] = { { 0, 0 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 0, 0 } };

double X[2][1] = { { 0 }, //state matrix filled with the states of the system
		{ 0 } };

double error[2][1] = { { 0 }, { 0 } };

const int MAX_INTAKE_CURRENT = 0.0; //find

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

double ref_intake[2][1];

std::thread IntakeThread;

MotionProfiler *intake_profiler = new MotionProfiler(MAX_VELOCITY,
		MAX_ACCELERATION, TIME_STEP);

std::vector<double> down_ang = { { 0.0 } };
std::vector<double> mid_ang = { { 0.0 } };
std::vector<double> up_ang = { { 0.0 } };

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

std::vector<std::vector<double>> up_to_down_profile =
		intake_profiler->CreateProfile1D(UP_ANGLE, down_ang);
std::vector<std::vector<double>> mid_to_down_profile =
		intake_profiler->CreateProfile1D(MID_ANGLE, down_ang);
std::vector<std::vector<double>> up_to_mid_profile =
		intake_profiler->CreateProfile1D(UP_ANGLE, mid_ang);
std::vector<std::vector<double>> down_to_up_profile =
		intake_profiler->CreateProfile1D(DOWN_ANGLE, up_ang);
std::vector<std::vector<double>> down_to_mid_profile =
		intake_profiler->CreateProfile1D(DOWN_ANGLE, mid_ang);

const int UP_TO_DOWN = 0;
const int MID_TO_DOWN = 1;
const int UP_TO_MID = 2;
const int DOWN_TO_UP = 3;
const int DOWN_TO_MID = 4;

int profile_in = 0;

Timer *intakeTimer = new Timer();

Intake::Intake() {

	talonIntake1 = new TalonSRX(2);
	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(4); //set current limit for arm

	talonIntake1->ConfigPeakCurrentLimit(40, 0);
	talonIntake2->ConfigPeakCurrentLimit(40, 0);

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
	double ang_vel = (talonIntake1->GetSelectedSensorVelocity(0)
			/ (TICKS_PER_ROT)) * (2 * PI) * (10.0);

	return ang_vel;

}

double Intake::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//spi radians per rotations
	double ang_pos = (talonIntake1->GetSelectedSensorPosition(0.0)
			/ (TICKS_PER_ROT)) * (2.0 * PI);

	return ang_pos;

}

void Intake::Rotate(double ref_intake[2][1]) {

	double current_pos = GetAngularPosition();
	double current_vel = GetAngularVelocity();
	double goal_pos = ref_intake[0][0];
	double goal_vel = ref_intake[1][0];

	error[0][0] = goal_pos - current_pos;
	error[1][0] = goal_vel - current_vel;

	u = (K[0][0] * error[0][0]) + (K[0][1] * error[1][0]); // for this system the second row of the K matrix is a copy and does not matter.

	//scaling the input value not to exceed the set parameters
	if (u > MAX_VOLTAGE) {
		u = MAX_VOLTAGE;
	} else if (u < MIN_VOLTAGE) {
		u = MIN_VOLTAGE;
	}

	//get the input into the -1 to +1 range for the talon
	//TODO: change v_bat into a dynamic value that tracks the battery's current voltage
	u = u / (v_bat);

	//talonElevator2 is slaved to this talon and does not need to be set
	talonIntake1->Set(ControlMode::PercentOutput, u);

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");

		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");

		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");

		break;

	case STOP_ARM_STATE:
		SmartDashboard::PutString("INTAKE ARM", "STOP");

		break;

	}

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

bool Intake::EncodersRunning() { //TODO: Check these values

	double current_pos = (talonIntake1->GetSelectedSensorPosition(0.0) / 4096.0)
			* 2.0 * 3.14; //radians
	if (talonIntake1->GetOutputCurrent() > 3.0
			&& talonIntake1->GetSelectedSensorVelocity(0) == 0.0
			&& std::abs(ref_intake[0][0] - current_pos) > 0.2) {
		return false;
	}
	return true;
}

bool Intake::HaveCube() {

	if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
		return true;
	} else {
		return false;
	}

}

void Intake::IntakeWrapper(Intake *in, double *profile) {

	std::vector<std::vector<double>> intake_profile = { };

	int last_profile = 0;
	int profile_state = 0;

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				profile_state = *profile;

				switch (profile_state) {

				case UP_TO_DOWN:
					intake_profile = up_to_down_profile;
					if (last_profile != profile_state) { //first time in state
						in->SetIndex(0);
					}
					break;

				case MID_TO_DOWN:
					intake_profile = mid_to_down_profile;
					if (last_profile != profile_state) { //first time in state
						in->SetIndex(0);
					}
					break;

				case UP_TO_MID:
					intake_profile = up_to_mid_profile;
					if (last_profile != profile_state) { //first time in state
						in->SetIndex(0);
					}
					break;

				case DOWN_TO_UP:
					intake_profile = down_to_up_profile;
					if (last_profile != profile_state) { //first time in state
						in->SetIndex(0);
					}
					break;

				case DOWN_TO_MID:
					intake_profile = down_to_mid_profile;
					if (last_profile != profile_state) { //first time in state
						in->SetIndex(0);
					}
					break;

				}

				last_profile = profile_state;

				double indeces[2][1] = { { intake_profile.at(0).at(
						in->intake_index) }, { intake_profile.at(1).at(
						in->intake_index) } };

				in->Rotate(indeces);

				if (in->intake_index < intake_profile.at(0).size() - 1) {
					in->intake_index++;
				}

				//				if (*profile == -1) { //TODO: add state that stops, if profile = stop, then stopArm(), else Rotate()
				in->StopArm();
				//				} else {
				//					in->Rotate();
				//				}

				intakeTimer->Reset();

			}
		}
	}

}

void Intake::SetIndex(int index) {

	intake_index = index;

}

void Intake::StartIntakeThread() {

	Intake *in = this;

	IntakeThread = std::thread(&Intake::IntakeWrapper, in, &profile_in);
	IntakeThread.detach();

}

void Intake::EndIntakeThread() {

	IntakeThread.~thread();

}
