/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//Talon id's, max intake wheel current, encoderrunning numbers,
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

const double TICKS_PER_ROT_I = 4096.0;
const double MAX_VOLTAGE_I = 2.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE_I = -2.0;

const double free_speed_i = 18730.0; //rad/s
const double G_i = (831.0/1.0);

const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i)) * 2.0 * PI * 10;
const double MAX_VELOCITY_I = 1.0;
const double MAX_ACCELERATION_I = 1.0;
const double TIME_STEP_I = 0.01; //sec
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

const double MAX_INTAKE_CURRENT = 50.0;

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 1.0; //TEST
const double UP_ANGLE = 0.0;

int last_intake_state = 1; //cannot equal the first state or profile will not set the first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 12.0; //this will be the voltage of the battery at every loop

std::vector<std::vector<double> > K_i = { { 55.15, 0.428 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 55.15, 0.428 } };

std::vector<std::vector<double> > X_i = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;

MotionProfiler *intake_profiler;

Intake::Intake(PowerDistributionPanel *pdp, MotionProfiler *intake_profiler_) {

	intake_profiler = intake_profiler_;

	talonIntake1 = new TalonSRX(2);

	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(4);
	talonIntakeArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	talonIntakeArm->ConfigPeakCurrentLimit(5, 0); //for now
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
			/ (TICKS_PER_ROT_I)) * (2.0 * PI) * (10.0);

	return ang_vel;

}

double Intake::GetAngularPosition() {

	//Native position in ticks, divide by ticks per rot to get into revolutions multiply by 2pi to get into radians
	//2pi radians per rotations
	double ang_pos = (talonIntakeArm->GetSelectedSensorPosition(0.0)
			/ (TICKS_PER_ROT_I)) * (2.0 * PI);

	return ang_pos;

}

void Intake::ManualWheels(Joystick *joyOpWheels) {

	SmartDashboard::PutNumber("WHEELS CUR", talonIntake1->GetOutputCurrent());
	SmartDashboard::PutNumber("WHEELS ENC", talonIntake1->GetSelectedSensorPosition(0));

	double out = joyOpWheels->GetRawAxis(3) * -1.0;
	talonIntake1->Set(ControlMode::PercentOutput, out);

}

void Intake::ManualArm(Joystick *joyOpArm) {

	SmartDashboard::PutNumber("ARM CUR", talonIntakeArm->GetOutputCurrent());
	SmartDashboard::PutNumber("ARM ENC", talonIntakeArm->GetSelectedSensorPosition(0)); //left is negative, right is positive

	double output = joyOpArm->GetX() / 10.0;
	talonIntakeArm->Set(ControlMode::PercentOutput, output);


}

void Intake::Rotate(std::vector<std::vector<double> > ref_intake) {

	//top is position, bottom is velocity

	double current_pos = GetAngularPosition();
	double current_vel = GetAngularVelocity();
	double goal_pos = ref_intake[0][0];
	double goal_vel = ref_intake[1][0];

	error_i[0][0] = goal_pos - current_pos;
	error_i[1][0] = goal_vel - current_vel;

	u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0]) + Kv_i * goal_vel; // for this system the second row of the K matrix is a copy and does not matter.

	//scaling the input value not to exceed the set parameters
	if (u_i > MAX_VOLTAGE_I) {
		u_i = MAX_VOLTAGE_I;
	} else if (u_i < MIN_VOLTAGE_I) {
		u_i = MIN_VOLTAGE_I;
	}

	//get the input into the -1 to +1 range for the talon

	v_bat_i = pdp_i->GetVoltage();

	u_i = u_i / (v_bat_i);

	SmartDashboard::PutNumber("INTAKE OUTPUT", u_i);

	//talonElevator2 is slaved to this talon and does not need to be set
	talonIntakeArm->Set(ControlMode::PercentOutput, u_i);

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		if (last_intake_state != UP_STATE) { //first time in state
			intake_profiler->SetFinalGoal(UP_ANGLE);
			intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");
		if (last_intake_state != MID_STATE) {
			intake_profiler->SetFinalGoal(MID_ANGLE);
			intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if (last_intake_state != DOWN_STATE) {
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

bool Intake::EncodersRunning() { //TODO: put real values

	double current_pos = GetAngularPosition(); //radians
	double current_ref = intake_profiler->GetNextRef().at(0).at(0);

	if (talonIntakeArm->GetOutputCurrent() > 3.0
			&& talonIntakeArm->GetSelectedSensorVelocity(0) == std::abs(0.2)
			&& std::abs(current_ref - current_pos) > 0.2) { //outputting current, not moving, should be moving
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

void Intake::ZeroEnc() {

	talonIntakeArm->SetSelectedSensorPosition(0, 0, 0);

}

void Intake::IntakeWrapper(Intake *in) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_intake = intake_profiler->GetNextRef();

				std::cout << "pos: " << profile_intake.at(0).at(0) << "  " << "vel: " << profile_intake.at(1).at(0) << "   " << "acc: " << profile_intake.at(2).at(0) << std::endl;

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
