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

#define CORNELIUS 1

#if CORNELIUS
double ff_percent_i = 0.6;
double offset_angle = 1.65;
double SLOW_SPEED = 0.25;
#else
double ff_percent_i = 0.6;
double offset_angle = 1.75; //1.5; with the new flippy back arm
double SLOW_SPEED = 0.4;
#endif

using namespace std::chrono;

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
//TODO: add an agitate state

const double TICKS_PER_ROT_I = 4096.0;
const double MAX_VOLTAGE_I = 12.0; //CANNOT EXCEED abs(10)
const double MIN_VOLTAGE_I = -12.0;

const double free_speed_i = 18730.0; //rpm
const double G_i = (831.0 / 1.0); //gear ratio
const double MAX_THEORETICAL_VELOCITY_I = ((free_speed_i / G_i))
		* ((2.0 * PI) / 60.0) * 1.05; //rad/s
const double Kv_i = 1 / MAX_THEORETICAL_VELOCITY_I;

//For motion profiler
//const double MAX_VELOCITY_I = 2.0; //1
//const double MAX_ACCELERATION_I = 6.0; //2.5;
//const double TIME_STEP_I = 0.01; //sec

const double MAX_INTAKE_CURRENT = 12.0;
const double OUTTAKE_INTAKE_CURRENT = 20.0;

const double PCL_WHEELS = 30.0; //peak current limit
const double CCL_WHEELS = 10.0; //continuous current limit
const double PCD_WHEELS = 200.0; //peak current duration

const int INTAKE_SLEEP_TIME = 0;
const double INTAKE_WAIT_TIME = 0.01; //sec

int last_intake_state = 0; //cannot equal the first state or profile will not set the first time

double u_i = 0; //this is the input in volts to the motor
double v_bat_i = 0.0; //will be set to pdp's voltage

std::vector<std::vector<double> > K_i;
std::vector<std::vector<double> > K_down_i = { { 10.32, 0.063 }, //controller matrix that is calculated in the Python simulation, pos and vel  10.32, 0.063
		{ 10.32, 0.063 } };
std::vector<std::vector<double> > K_up_i = { { 16.75, 0.12 }, //controller matrix that is calculated in the Python simulation, pos and vel 16.75, 0.12
		{ 16.75, 0.12 } };

std::vector<std::vector<double> > X_i = { { 0.0 }, //state matrix filled with the states of the system //not used
		{ 0.0 } };

std::vector<std::vector<double> > error_i = { { 0.0 }, { 0.0 } };

Timer *intakeTimer = new Timer();

PowerDistributionPanel *pdp_i;
Elevator *elevator_i;
IntakeMotionProfiler *intake_profiler;

double starting_pos = 0.0;

bool is_at_bottom = false;
bool first_time_at_bottom = false;
bool voltage_safety = false;

std::vector<double> volts = { };

int init_counter_i = 0;
int current_counter_h = 0; //have
int current_counter_r = 0; //release
int current_counter_l = 0; //low
int current_counter = 0;
int counter_i = 0;
//int i = 0;
int encoder_counter = 0;

int position_offset = 0;

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

	pdp_i = pdp;

}

void Intake::SetStartingPos(double start) { //not used
	starting_pos = start;
}

void Intake::InitializeIntake() {

	if (!is_init_intake) {
		SetVoltageIntake(2.5); //offset is changed accordingly //TODO: increase zeroing voltage

	}

}

void Intake::In() { //add quick out and back in

	//std::cout << "INTAKE IN" << std::endl;
	//SmartDashboard::PutNumber("BAT VOLT", pdp_i->GetVoltage());

	talonIntake1->EnableCurrentLimit(true);
	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake1->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake2->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake1->ConfigPeakCurrentDuration(PCD_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentDuration(PCD_WHEELS, 0);

//	if (talonIntake1->GetOutputCurrent() >= 5.0
//			|| talonIntake2->GetOutputCurrent() >= 5.0) {
//		current_counter_first++;
//	} else {
//		current_counter_first = 0;
//		cube_in = false;
//	}
//	if (current_counter_first >= 3) {
//		cube_in = true;
//	}

	//if (cube_in) {
		talonIntake1->Set(ControlMode::PercentOutput, -0.95); // +2.0/12.0 maybe -0.7
		talonIntake2->Set(ControlMode::PercentOutput, 0.95); // +2.0/12.0 maybe 0.7
	//} else {
	//	talonIntake1->Set(ControlMode::PercentOutput, -0.60); // +2.0/12.0 maybe -0.7
	//	talonIntake2->Set(ControlMode::PercentOutput, 0.60); // +2.0/12.0 maybe 0.7
	//}

}

void Intake::Out() {

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, 1.0); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -1.0); // -1.0

}

void Intake::Slow() {

	talonIntake1->EnableCurrentLimit(false); //20
	talonIntake2->EnableCurrentLimit(false);

	talonIntake1->Set(ControlMode::PercentOutput, SLOW_SPEED); // 1.0
	talonIntake2->Set(ControlMode::PercentOutput, -SLOW_SPEED); // -1.0

}

void Intake::StopWheels() {

	talonIntake1->EnableCurrentLimit(true);
	talonIntake2->EnableCurrentLimit(true);
	talonIntake1->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentLimit(PCL_WHEELS, 0);
	talonIntake1->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake2->ConfigContinuousCurrentLimit(CCL_WHEELS, 0);
	talonIntake1->ConfigPeakCurrentDuration(PCD_WHEELS, 0);
	talonIntake2->ConfigPeakCurrentDuration(PCD_WHEELS, 0);

	talonIntake1->Set(ControlMode::PercentOutput, 0.0 - 0.2); //1.8 v
	talonIntake2->Set(ControlMode::PercentOutput, 0.0 + 0.2); //0.15

}

void Intake::ManualArm(Joystick *joyOpArm) {

//	SmartDashboard::PutNumber("ARM CUR", talonIntakeArm->GetOutputCurrent());

	//double enc_arm = GetAngularPosition();
	SmartDashboard::PutNumber("ARM ENC",
			talonIntakeArm->GetSensorCollection().GetQuadraturePosition());

	//SmartDashboard::PutNumber("ARM POS", GetAngularPosition()); //left is negative, right is positive

	double output = joyOpArm->GetY() * 0.5 * 1.0;

	output *= 12.0;

	//SmartDashboard::PutNumber("ARM OUTPUT", output);

	SetVoltageIntake(output);

}

void Intake::Rotate(std::vector<std::vector<double> > ref_intake) { //a vector of a pos vector and a vel vector

	//top is position, bottom is velocity

	double current_pos = GetAngularPosition();
	double current_vel = GetAngularVelocity();
	double goal_pos = ref_intake[0][0];
	double goal_vel = ref_intake[1][0];

	SmartDashboard::PutNumber("INTAKE POS", GetAngularPosition());

//	SmartDashboard::PutNumber("INTAKE REF POS", goal_pos);
//	SmartDashboard::PutNumber("INTAKE REF VEL", goal_vel);

//	std::cout << "goal pos: " << goal_pos << "        goal vel: " << goal_vel
//			<< std::endl;

	error_i[0][0] = goal_pos - current_pos;
	error_i[1][0] = goal_vel - current_vel;

//	SmartDashboard::PutNumber("INTAKE ERR POS", error_i[0][0]);
//	SmartDashboard::PutNumber("INTAKE ERR VEL", error_i[1][0]);

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

	u_i = (K_i[0][0] * error_i[0][0]) + (K_i[0][1] * error_i[1][0]) //+ offset ; //u_i is in voltage, * by v_bat_i
			+ (Kv_i * goal_vel * v_bat_i) * ff_percent_i + offset; // for this system the second row of the K matrix is a copy and does not matter.

	//SmartDashboard::PutNumber("INTAKE FF", u_i);

	SetVoltageIntake(u_i);

}

void Intake::SetVoltageIntake(double voltage_i) {

	SmartDashboard::PutString("INTAKE SAFETY", "none");

	//is_at_bottom = IsAtBottomIntake(); //hall effect returns 0 when at bottom. we reverse it here

	//soft limit
	if (elevator_i->GetElevatorPosition() < elevator_safety_position) {
		if (GetAngularPosition() >= (1.6) && voltage_i > 0.0
				&& is_init_intake) { //at max height and still trying to move up //no upper soft limit when initializing
			voltage_i = 0.0; //shouldn't crash
			SmartDashboard::PutString("INTAKE SAFETY", "top soft limit");
		}
	} else {
		if (GetAngularPosition() >= (INTAKE_BACKWARDS_SOFT_LIMIT)
				&& voltage_i > 0.0 && is_init_intake) { //at max height and still trying to move up //no upper soft limit when initializing
			voltage_i = 0.0; //shouldn't crash
			SmartDashboard::PutString("INTAKE SAFETY", "top soft limit");
		}
	}

	//safety to make sure that the elevator doesn't go down when the arm is up
	if (GetAngularPosition() > 1.7 && elevator_i->GetVoltageElevator() < 0.0) { //checking and changing u_e
		elevator_i->SetVoltageElevator(0.0);
	}

//	if (is_at_bottom) {
	if (talonIntakeArm->GetOutputCurrent() > 3.0) {
		counter_i++;
		if (counter_i > 1) {
			if (ZeroEnc()) { //successfully zeroed enc one time
				is_init_intake = true;
			}
		}
	} else {
		counter_i = 0;
	}
	if (voltage_i < 0.0 && GetAngularPosition() < -0.1) {
		voltage_i = 0.0;
		SmartDashboard::PutString("INTAKE SAFETY", "bottom soft limit");
	}

	//voltage limit
	if (voltage_i > MAX_VOLTAGE_I) {
		voltage_i = MAX_VOLTAGE_I;
	} else if (voltage_i < MIN_VOLTAGE_I) {
		voltage_i = MIN_VOLTAGE_I;
	}

	if (std::abs(GetAngularVelocity()) <= 0.05 && std::abs(voltage_i) > 3.0) { //outputting current, not moving, should be moving
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

	if (voltage_safety) {
		SmartDashboard::PutString("INTAKE SAFETY", "stall");
		voltage_i = 0.0;
	}

	//SmartDashboard::PutNumber("INTAKE VOLT", voltage_i);

	voltage_i /= 12.0; //scale from -1 to 1 for the talon

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

	((talonIntakeArm->GetSensorCollection().GetQuadraturePosition()
			- position_offset)	//position offset
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

//	SmartDashboard::PutNumber("WHEEL CUR 1", talonIntake1->GetOutputCurrent());
//	SmartDashboard::PutNumber("WHEEL CUR 2", talonIntake2->GetOutputCurrent());

	switch (intake_arm_state) {

	case INIT_STATE:
		SmartDashboard::PutString("INTAKE ARM", "INIT");
		if (is_init_intake) { // && GetAngularPosition() == 0.35) {
			intake_arm_state = UP_STATE; //PUT BACK IN
		}
		else {
			InitializeIntake();
		}
		last_intake_state = INIT_STATE;
		break;

	case UP_STATE:
		SmartDashboard::PutString("INTAKE ARM", "UP");
		//	SmartDashboard::PutString("actually in up state", "yep");
		if (last_intake_state != UP_STATE) { //first time in state
			intake_profiler->SetFinalGoalIntake(UP_ANGLE); //is 0.0 for testing
			intake_profiler->SetInitPosIntake(GetAngularPosition()); //is 0
		}
		last_intake_state = UP_STATE;
		break;

	case MID_STATE:
		SmartDashboard::PutString("INTAKE ARM", "MID");
		if (last_intake_state != MID_STATE) {
			intake_profiler->SetFinalGoalIntake(MID_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = MID_STATE;
		break;

	case DOWN_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if (last_intake_state != DOWN_STATE) {
			intake_profiler->SetFinalGoalIntake(DOWN_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = DOWN_STATE;
		break;

	case STOP_ARM_STATE: //for emergencies
		SmartDashboard::PutString("INTAKE ARM", "STOP");
		StopArm();
		last_intake_state = STOP_ARM_STATE;
		break;

	case SWITCH_BACK_SHOT_STATE:
		SmartDashboard::PutString("INTAKE ARM", "DOWN");
		if (last_intake_state != SWITCH_BACK_SHOT_STATE) {
			intake_profiler->SetFinalGoalIntake(BACK_SHOT_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = SWITCH_BACK_SHOT_STATE;
		break;

	case SWITCH_STATE:
		SmartDashboard::PutString("INTAKE ARM", "SWITCH");
		if (last_intake_state != SWITCH_STATE) {
			intake_profiler->SetFinalGoalIntake(SWITCH_ANGLE);
			intake_profiler->SetInitPosIntake(GetAngularPosition());
		}
		last_intake_state = SWITCH_STATE;
		break;
	}

	//last_intake_state = intake_arm_state; //move this into individual states if profile not switching

}

void Intake::IntakeWheelStateMachine() {

	///std::cout << "INTAKE STATE " << intake_wheel_state << std::endl;

	switch (intake_wheel_state) {

	case STOP_WHEEL_STATE: //has offset, not actually stopped
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

	case SLOW_STATE:
		SmartDashboard::PutString("INTAKE WHEEL", "SLOW");
		Slow();
		break;

	}

}

bool Intake::EncodersRunning() { //will stop the controller from run //or stalled //MOVE INTO SET VOLTAGE

//	double current_pos = GetAngularPosition(); //radians
//	double current_ref = intake_profiler->GetNextRefIntake().at(0).at(0);

	return true;
}

bool Intake::HaveCube() {

	if (talonIntake1->GetOutputCurrent() >= MAX_INTAKE_CURRENT
			&& talonIntake2->GetOutputCurrent() >= MAX_INTAKE_CURRENT) {
		current_counter++;
	} else {
		current_counter = 0;
	}
	if (current_counter >= 2) {
		return true;
	} else {
		return false;
	}

}

bool Intake::ReleasedCube() {

	if (intake_wheel_state == SLOW_STATE) { //out slow
		if (talonIntake1->GetOutputCurrent() <= 10.0
				&& talonIntake2->GetOutputCurrent() <= 10.0) {
			current_counter++;
		} else {
			current_counter = 0;
		}
		if (current_counter >= 15) {
			current_counter = 0;
			return true;
		} else {
			return false;
		}
	} else {
		if (talonIntake1->GetOutputCurrent() <= 17.0
				|| talonIntake2->GetOutputCurrent() <= 17.0) {
			current_counter++;
		} else {
			current_counter = 0;
		}
		if (current_counter >= 10) { //This usedto be 5 3/4/18
			current_counter = 0; //only zero once has reached 10
			return true;
		} else {
			return false;
		}
	}

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

	intakeTimer->Start();

	while (true) {

		intakeTimer->Reset();

		if (frc::RobotState::IsEnabled()) {

			std::vector<std::vector<double>> profile_intake =
					intake_profiler->GetNextRefIntake();

			if (in->intake_arm_state != STOP_ARM_STATE
					&& in->intake_arm_state != INIT_STATE) {
				in->Rotate(profile_intake);
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
