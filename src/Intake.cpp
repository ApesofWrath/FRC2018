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
int intake_arm_state = UP_STATE;

const int STOP_WHEEL_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;
int intake_wheel_state = STOP_WHEEL_STATE;

const int MAX_INTAKE_CURRENT = 0.0; //check

Timer *intakeTimer = new Timer();

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

int ref_;

const double DOWN_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

Intake::Intake() {

	talonIntake1 = new TalonSRX(2);
	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(4); //set current limit for arm

	talonIntake1->ConfigPeakCurrentLimit(10, 0);
	talonIntake2->ConfigPeakCurrentLimit(10, 0);

	ref_ = DOWN_ANGLE;

}

void Intake::In() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.7); // +2.0/12.0 maybe

}

void Intake::Out() {

	talonIntake1->Set(ControlMode::PercentOutput, -0.7); // +2.0/12.0 maybe

}

void Intake::Stop() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::Rotate(double ref) {

}

void Intake::IntakeArmStateMachine() {

	switch (intake_arm_state) {

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		ref_ = UP_ANGLE;
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		ref_ = DOWN_ANGLE;
		break;

	}

}

void Intake::IntakeWheelStateMachine() {

	switch (intake_wheel_state) {

		case STOP_WHEEL_STATE:
			SmartDashboard::PutString("INTAKE WHEEL", "STOP");
			Stop();
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

	//Intake *in = this;

	//IntakeThread = std::thread(&Intake::IntakeWrapper, in, &ref_);
	//IntakeThread.detach();

}

void Intake::IntakeWrapper(Intake *in, double *ref) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				intakeTimer->Reset();
				in->Rotate(*ref);

			}
		}
	}

}

void Intake::EndIntakeThread() {

	IntakeThread.~thread();

}
