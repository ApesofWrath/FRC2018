/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Intake.h>
#include <ctre/Phoenix.h>

using namespace std::chrono;

const int UP_STATE = 0;
const int DOWN_STATE = 1;
const int STOP_ARM_STATE = 2;

const int STOP_WHEEL_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;

const int MAX_INTAKE_CURRENT = 0.0; //find

Timer *intakeTimer = new Timer();

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

int ref_intake;

const double DOWN_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

Intake::Intake() {

	talonIntake1 = new TalonSRX(2);
	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(4); //set current limit for arm

	talonIntake1->ConfigPeakCurrentLimit(10, 0);
	talonIntake2->ConfigPeakCurrentLimit(10, 0);

	ref_intake = DOWN_ANGLE;

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

void Intake::Rotate(double ref_intake_) {

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		ref_intake = UP_ANGLE;
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		ref_intake = DOWN_ANGLE;
		break;

	case STOP_ARM_STATE:
		SmartDashboard::PutString("INTAKE ARM", "STOP");
		ref_intake = 0.0;
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

bool Intake::HaveCube() {

	if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
		return true;
	} else {
		return false;
	}

}


void Intake::StartIntakeThread() {

	Intake *in = this;

//	IntakeThread = std::thread(&Intake::IntakeWrapper, in, &ref_);
//	IntakeThread.detach();

}

void Intake::IntakeWrapper(Intake *in, double *ref_in) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				intakeTimer->Reset();
				if(*ref_in == 0.0) {
					in->StopArm();
				}
				else {
				in->Rotate(*ref_in);
				}

			}
		}
	}

}

void Intake::EndIntakeThread() {

	//IntakeThread.~thread();

}
