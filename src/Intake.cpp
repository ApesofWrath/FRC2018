/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Intake.h>

const int STOP_STATE = 0;
const int IN_STATE = 1;
const int OUT_STATE = 2;
const int DOWN_STATE = 3;
const int UP_STATE = 4;
int intake_state = 0;

Timer *intakeTimer = new Timer();

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

int ref_;

const double DOWN_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

Intake::Intake() {

	talonIntake1 = new TalonSRX(0);
	talonIntake2 = new TalonSRX(0);
	talonIntake2->Set(ControlMode::Follower, 0);

	talonIntakeArm = new TalonSRX(0);

	ref_ = DOWN_ANGLE;

}

void Intake::In() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.7);

}

void Intake::Out() {

	talonIntake1->Set(ControlMode::PercentOutput, -0.7);

}

void Intake::Stop() {

	talonIntake1->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::Rotate(double ref) {

}

void Intake::IntakeStateMachine() {

	switch (intake_state) {

	case STOP_STATE:
		Stop();
		break;

	case IN_STATE:
		In();
		break;

	case OUT_STATE:
		Out();
		break;

	case DOWN_STATE:
		ref_ = DOWN_ANGLE;
		break;

	case UP_STATE:
		ref_ = UP_ANGLE;
		break;

	}

}

void Intake::StartIntakeThread() {

	Intake *in = this;

	IntakeThread = std::thread(&Intake::IntakeWrapper, in, &ref_);
	IntakeThread.detach();

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
