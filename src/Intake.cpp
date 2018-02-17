/*
 * Intake.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//ARM ENCODER HAS OFFSET: We set 0.2 to 0.0
#include <Intake.h>
#include <ctre/Phoenix.h> //double included
#include <WPILib.h>

#define PI 3.14159265

using namespace std::chrono;

const int INIT_STATE = 0;
const int UP_STATE = 1; //arm state machine
const int MID_STATE = 2;
const int DOWN_STATE = 3;
const int STOP_ARM_STATE = 4;

const int STOP_WHEEL_STATE = 0; //wheel state machine
const int IN_STATE = 1;
const int OUT_STATE = 2;

const double TICKS_PER_ROT_I = 4096.0;
const double MAX_VOLTAGE_I = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_I = -12.0;

const double free_speed_i = 18730.0; //rpm
const double G_i = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i))
		* ((2.0 * PI) / 60.0) * 1.05; //rad/s
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

//For motion profiler
const double MAX_VELOCITY_I = 1.5; //1
const double MAX_ACCELERATION_I = 4.0; //2.5;
const double TIME_STEP_I = 0.01; //sec

const double MAX_INTAKE_CURRENT = 50.0;

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

const double DOWN_ANGLE = 0.0;
const double MID_ANGLE = 0.6;
const double UP_ANGLE = 1.3; //starting pos

int last_intake_state = 0; //cannot equal the first state or profile will not set the first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 0.0; //will be set to pdp's voltage

std::vector<std::vector<double> > K_i;
std::vector<std::vector<double> > K_down_i = { { 0.0, 0.0 }, //controller matrix that is calculated in the Python simulation, pos and vel  10.32, 0.063
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_i = { { 0.0, 0.0 }, //controller matrix that is calculated in the Python simulation, pos and vel 16.75, 0.12
		{ 16.75, 0.12 } };

std::vector<std::vector<double> > X_i = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;

IntakeMotionProfiler *intake_profiler;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

std::vector<double> volts = { };

int init_counter_i = 0;
int counter_i = 0;
//int i = 0;
int encoder_counter = 0;

int position_offset = 0;

Intake::Intake(PowerDistributionPanel *pdp,
		IntakeMotionProfiler *intake_profiler_) {

	intake_profiler = intake_profiler_;

	intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I);
	intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);

	hallEffectIntake = new DigitalInput(0);

	talonIntake1 = new TalonSRX(17);
	talonIntake2 = new TalonSRX(25);
	//talonIntake2->Set(ControlMode::Follower, 2);

//	talonIntake1->EnableCurrentLimit(true);
//	talonIntake2->EnableCurrentLimit(true);
//	talonIntake1->ConfigPeakCurrentLimit(5, 0); //20
//	talonIntake2->ConfigPeakCurrentLimit(5, 0);
//	talonIntake1->ConfigContinuousCurrentLimit(5, 0); //20
//	talonIntake2->ConfigContinuousCurrentLimit(5, 0);

	talonIntakeArm = new TalonSRX(55);

	talonIntakeArm->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	//talonIntakeArm->EnableCurrentLimit(true);
//	talonIntakeArm->ConfigPeakCurrentLimit(20, 0); //for now
//	talonIntakeArm->ConfigContinuousCurrentLimit(20, 0); //for now

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

void Intake::In() { //add quick out and back in

	//std::cout << "INTAKE IN" << std::endl;
	SmartDashboard::PutNumber("BAT VOLT", pdp_i->GetVoltage());

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, -0.95); // +2.0/12.0 maybe -0.7
	talonIntake2->Set(ControlMode::PercentOutput, 0.95); // +2.0/12.0 maybe 0.7

}

void Intake::Out() {

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, 0.8); // +2.0/12.0 maybe
	talonIntake2->Set(ControlMode::PercentOutput, -0.8); // +2.0/12.0 maybe

}

void Intake::StopWheels() {

	talonIntake1->EnableCurrentLimit(false);
	talonIntake2->EnableCurrentLimit(false);
//	talonIntake1->ConfigPeakCurrentLimit(5, 0); //20
//	talonIntake2->ConfigPeakCurrentLimit(5, 0);
//	talonIntake1->ConfigContinuousCurrentLimit(5, 0); //20
//	talonIntake2->ConfigContinuousCurrentLimit(5, 0);

	talonIntake1->Set(ControlMode::PercentOutput, 0.0 - 0.2); //1.8 v
	talonIntake2->Set(ControlMode::PercentOutput, 0.0 + 0.2); //0.15

}

void Intake::ManualArm(Joystick *joyOpArm) {

	SmartDashboard::PutNumber("ARM CUR", talonIntakeArm->GetOutputCurrent());
	//double enc_arm = GetAngularPosition();
	SmartDashboard::PutNumber("ARM ENC",
			talonIntakeArm->GetSensorCollection().GetQuadraturePosition());

	SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	output *= 12.0;

	SmartDashboard::PutNumber("ARM OUTPUT", output);

	SetVoltageIntake(output);

}

void Intake::Rotate(std::vector<std::vector<double> > ref_intake) { //a vector of a pos vector and a vel vector

	//top is position, bottom is velocity

	double current_pos = GetAngularPosition();
	double current_vel = GetAngularVelocity();
	double goal_pos = ref_intake[0][0];
	double goal_vel = ref_intake[1][0];

	SmartDashboard::PutNumber("INTAKE REF POS", goal_pos);
		SmartDashboard::PutNumber("INTAKE REF VEL", goal_vel);

	std::cout << "goal pos: " << goal_pos << "        goal vel: " << goal_vel
			<< std::endl;

	error_i[0][0] = goal_pos - current_pos;
	error_i[1][0] = goal_vel - current_vel;

	SmartDashboard::PutNumber("INTAKE ERR POS", error_i[0][0]);
	SmartDashboard::PutNumber("INTAKE ERR VEL", error_i[1][0]);

	v_bat_i = 12.0;

	if (intake_profiler->GetFinalGoalIntake()
			< intake_profiler->GetInitPosIntake()) { //changed
		K_i = K_down_i;
	} else {
		K_i = K_up_i;
	}

//	if (IsAtBottomIntake() && std::abs(error_i[0][0]) > 0.4) { //shaking
//		intake_arm_state = DOWN_STATE;
//	}

	double offset = 1.3 * cos(current_pos); //counter gravity

	u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0]) //u_i is in voltage, * by v_bat_i
			+ (Kv_i * goal_vel * v_bat_i) + offset; // for this system the second row of the K matrix is a copy and does not matter.

	//SmartDashboard::PutNumber("INTAKE FF", u_i);

	SetVoltageIntake(u_i);

}

void Intake::SetVoltageIntake(double voltage_i) {

	SmartDashboard::PutBoolean("INTAKE INIT", is_init_intake);

	is_at_bottom = IsAtBottomIntake(); //hall effect returns 0 when at bottom. we reverse it here
	SmartDashboard::PutNumber("HALL EFF INTAKE", is_at_bottom); //actually means not at bottom //0 means up// 1 means dow

	//soft limit
	if (GetAngularPosition() >= (PI / 2.0) && voltage_i > 0.0) { //at max height and still trying to move up
		voltage_i = 0.0; //shouldn't crash
	}

	if (is_at_bottom) {
		//if (counter_i == 0) { //first time at bottom
		SmartDashboard::PutNumber("SHOULD WORK", 4);
		if (ZeroEnc()) {
			is_init_intake = true;
		}
		SmartDashboard::PutNumber("SHOULD WORK", 3);
		////	counter_i++;
		//}
		if (voltage_i < 0.0 && GetAngularPosition() < -0.1) {
			voltage_i = 0.0;
		}
	} else {
		counter_i = 0;
	}

	//voltage limit
	if (voltage_i > MAX_VOLTAGE_I) {
		voltage_i = MAX_VOLTAGE_I;
	} else if (voltage_i < MIN_VOLTAGE_I) {
		voltage_i = MIN_VOLTAGE_I;
	}

	SmartDashboard::PutNumber("VOLTAGE INTAKE", voltage_i);

	if (std::abs(GetAngularVelocity()) <= 0.05 && std::abs(voltage_i) > 4.0) { //outputting current, not moving, should be moving
		encoder_counter++;
		//std::cout << "VEL: " << GetAngularVelocity() << "  VOLT: " << voltage_i << std::endl;
	} else {
		encoder_counter = 0;
	}

	if (encoder_counter > 10) { //10
		voltage_safety = true;
	} else {
		voltage_safety = false;
	}

	voltage_i /= 12.0; //scale from -1 to 1 for the talon

	voltage_i *= -1.0; //set AT END

	if (voltage_safety) {
		voltage_i = 0.0;
	}
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
			((talonIntakeArm->GetSensorCollection().GetQuadraturePosition()
					- position_offset) //position offset
			/ (TICKS_PER_ROT_I)) * (2.0 * PI) * -1.0;
	//double ang_pos = 0.0;

	double offset_angle = .25; //amount that the arm will stick up in radians

	return ang_pos + offset_angle; //the angular position from the encoder plus the angle when we zero minus the offset for zeroing

}

bool Intake::IsAtBottomIntake() {
	if (!hallEffectIntake->Get()) {
		return true;
	} else {
		return false;
	}
}

void Intake::ManualWheels(Joystick *joyOpWheels) {

	SmartDashboard::PutNumber("WHEELS CUR", talonIntake1->GetOutputCurrent());
	double enc = (talonIntake1->GetSensorCollection().GetQuadraturePosition()
			/ 4096.0) * 2.0 * PI; //change this
	SmartDashboard::PutNumber("WHEELS ENC", enc);

	double out = joyOpWheels->GetRawAxis(3) * -1.0;

	talonIntake1->Set(ControlMode::PercentOutput, out);

}

void Intake::StopArm() {

	talonIntakeArm->Set(ControlMode::PercentOutput, 0.0);

}

void Intake::IntakeArmStateMachine() {

	SmartDashboard::PutNumber("ARM ENC",
			talonIntakeArm->GetSensorCollection().GetQuadraturePosition());

	SmartDashboard::PutNumber("INTAKE POS.", GetAngularPosition());
	SmartDashboard::PutNumber("INTAKE VEL", GetAngularVelocity());

	//std::cout << "intake pos " << GetAngularPosition() << std::endl;

	SmartDashboard::PutNumber("WHEEL CUR 1", talonIntake1->GetOutputCurrent());
	SmartDashboard::PutNumber("WHEEL CUR 2", talonIntake2->GetOutputCurrent());

	switch (intake_arm_state) {

	case INIT_STATE:
		SmartDashboard::PutString("INTAKE ARM", "INIT");
		InitializeIntake();
		if (is_init_intake) { // && GetAngularPosition() == 0.35) {
			intake_arm_state = UP_STATE; //PUT BACK IN
		}
		last_intake_state = INIT_STATE;
		break;

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		SmartDashboard::PutString("actually in up state", "yep");
		if (last_intake_state != UP_STATE) { //first time in state
//			intake_profiler->SetMaxAccIntake(MAX_ACCELERATION_I); //these must be reset in each state
//			intake_profiler->SetMaxVelIntake(MAX_VELOCITY_I);
			intake_profiler->SetFinalGoalIntake(UP_ANGLE); //is 0.0 for testing
			intake_profiler->SetInitPosIntake(GetAngularPosition()); //is 0
			std::cout << " dojijodidj" << std::endl;
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

bool Intake::EncodersRunning() { //will stop the controller from run //or stalled //MOVE INTO SET VOLTAGE

//	double current_pos = GetAngularPosition(); //radians
//	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

	return true;
}

bool Intake::HaveCube() {

	return false;
	return false;
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

void Intake::SetZeroOffset() {

	position_offset =
			(talonIntakeArm->GetSensorCollection().GetQuadraturePosition());
}

bool Intake::ZeroEnc() { //called in Initialize() and in SetVoltage()

//	if (zeroing_counter_i < 2) {
//		if (talonIntakeArm->GetSensorCollection().SetQuadraturePosition(0,
//				1000) == 0) {
//			zeroing_counter_i++; //still initiializing even if the zero fails
//			return true;
//		} else {
//			std::cout << "here" << std::endl;
//			return false;
//		}
//	}

	if (zeroing_counter_i < 1) {
		SetZeroOffset();
		zeroing_counter_i++;
		return true;
	} else {
		//std::cout << "here" << std::endl;
		return false;
	}

}

void Intake::IntakeWrapper(Intake *in) {

//	int i = 0;
//
//	intake_profiler->SetMaxVelIntake(1.0);
//	intake_profiler->SetMaxAccIntake(4.5);
//
//	std::vector<std::vector<double>> profile_intake =
//			intake_profiler->CreateProfile1DIntake(0.2, { 1.3 });

	intakeTimer->Start();

	while (true) {
		while (frc::RobotState::IsEnabled()) {
			std::this_thread::sleep_for(
					std::chrono::milliseconds(INTAKE_SLEEP_TIME));

			//		SmartDashboard::PutNumber("timer", intakeTimer->Get());

			if (intakeTimer->HasPeriodPassed(INTAKE_WAIT_TIME)) {


				//std::vector<std::vector<double>> next_goal = {{profile_intake.at(0)}};

//				std::vector<std::vector<double>> profile_intake = intake_profiler->GetNextRefIntake();
//
//				if (in->intake_arm_state != STOP_ARM_STATE
//						&& in->intake_arm_state != INIT_STATE) {
//					in->Rotate(profile_intake);
//				}
//				if(i < profile_intake.size() - 1) {
//					i++;
//				}

//				std::vector<std::vector<double>> profile_intake =
//									intake_profiler->GetNextRefIntake();

				//std::cout << "pos: " << profile_intake.at(0).at(i) << "      vel: " << profile_intake.at(1).at(i) << std::endl;

//				if (in->intake_arm_state != STOP_ARM_STATE
//						&& in->intake_arm_state != INIT_STATE) {
//					in->Rotate( { { profile_intake.at(0).at(i) }, {
//							profile_intake.at(1).at(i) } });
					std::cout << "herde" << std::endl;

//
//				if (i < profile_intake.at(0).size() - 1) {
//						i++;
//						//std::cout << "OOOOOOOOOOOOOO" << std::endl;
//					}
//			//	}

				//	i++;

				std::vector<std::vector<double>> profile_intake =
						intake_profiler->GetNextRefIntake();

//				SmartDashboard::PutNumber("INTAKE REF POS", profile_intake.at(0));
//					SmartDashboard::PutNumber("INTAKE REF VEL", profile_intake.at(1));

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
