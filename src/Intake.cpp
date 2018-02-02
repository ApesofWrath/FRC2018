/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Intake.h>
#include <ctre/Phoenix.h>

using namespace std::chrono;

const int UP_STATE = 0; //arm state machine
const int MID_STATE = 1;
const int DOWN_STATE = 2;
const int STOP_ARM_STATE = 3;

const int STOP_WHEEL_STATE = 0; //wheel state machine
const int IN_STATE = 1;
const int OUT_STATE = 2;

const double TICKS_PER_ROT = 4096.0;
const double PI = 3.14159;
const double MAX_VOLTAGE = 12.0; //CANNOT EXCEED abs(12)
const double MIN_VOLTAGE = -12.0;


//to change
const double free_speed = 1967.0; //rad/s
const double m = 2.1;
const double l = 0.2;
const double Kt = 0.00595;
const double G = ((60.0 / 1.0) * (48.0 / 38.0));

const double MAX_THEORETICAL_VELOCITY = (free_speed / G);
const double MAX_VELOCITY = 28.0;
const double MAX_ACCELERATION = 38.0;
const double TIME_STEP = 0.01;
const double Kv_in = 1 / MAX_THEORETICAL_VELOCITY;


const int MAX_INTAKE_CURRENT = 0.0;

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 0.0;
const double UP_ANGLE = 0.0;

int last_intake_state = 0;

double u = 0; //this is the input in volts to the motor
double v_bat = 12.0; //this will be the voltage of the battery at every loop

double K[2][2] = { { 0, 0 }, //controller matrix that is calculated in the Python simulation, pos and vel
		{ 0, 0 } };

double X[2][1] = { { 0 }, //state matrix filled with the states of the system
		{ 0 } };

double error[2][1] = { { 0 }, { 0 } };

std::thread IntakeThread;

MotionProfiler *intake_profiler = new MotionProfiler(MAX_VELOCITY,
		MAX_ACCELERATION, TIME_STEP);

std::vector<double> down_ang = { { DOWN_ANGLE } }; //is a vector because there is more than 1 waypoint for other profiles
std::vector<double> mid_ang = { { MID_ANGLE } };
std::vector<double> up_ang = { { UP_ANGLE } };

Timer *intakeTimer = new Timer();

Intake::Intake() {

	talonIntake1 = new TalonSRX(2);
	talonIntake2 = new TalonSRX(3);
	talonIntake2->Set(ControlMode::Follower, 2);

	talonIntakeArm = new TalonSRX(4);

	talonIntakeArm->ConfigPeakCurrentLimit(5, 0); //for now
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

	//top is position, bottom is velocity

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
		if(last_intake_state != UP_STATE) { //first time in state
		intake_profiler->SetFinalGoal(UP_ANGLE);
		intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");
		if(last_intake_state != MID_STATE) {
		intake_profiler->SetFinalGoal(MID_ANGLE);
		intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if(last_intake_state != DOWN_STATE) {
		intake_profiler->SetFinalGoal(DOWN_ANGLE);
		intake_profiler->SetInitPos(GetAngularPosition());
		}
		break;

	case STOP_ARM_STATE: //not used
		SmartDashboard::PutString("INTAKE ARM", "STOP");
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

	if (talonIntake1->GetOutputCurrent() > 3.0
			&& talonIntake1->GetSelectedSensorVelocity(0) == std::abs(0.2)
			&& std::abs(ref_intake[0][0] - current_pos) > 0.2) { //outputting current, not moving, should be moving //figure out pointers for this
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

void Intake::IntakeWrapper(Intake *in, MotionProfiler *intake_profiler) {

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {

				std::vector<std::vector<double>> profile_intake = intake_profiler->GetNextRef();

				double indeces[2][1] = { { profile_intake.at(0).at(0) }, { profile_intake.at(1).at(0) } }; //Rotate() takes an array, not a vector

				if (!in->EncodersRunning()) {
					in->StopArm();
				} else {
					in->Rotate(indeces);
				}

				intakeTimer->Reset();

			}
		}
	}

}

void Intake::StartIntakeThread() {

	Intake *in = this;


	IntakeThread = std::thread(&Intake::IntakeWrapper, in, intake_profiler);
	IntakeThread.detach();

}

void Intake::EndIntakeThread() {

	IntakeThread.~thread();

}
