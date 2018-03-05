/*
 * DriveController.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */

#include <DriveControllerMother.h>
#include <WPILib.h>

#include "ctre/Phoenix.h"

#define CORNELIUS 0

#if CORNELIUS

#else

#endif

using namespace std::chrono;

////////////DIFFERENT WITH EVERY ROBOT////////////

const double WHEEL_DIAMETER = 4.0; //inches, for fps for auton
const double TICKS_PER_ROT = 1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders

const double MAX_Y_RPM_LOW = 550.0;
const double MAX_Y_RPM_HIGH = 1250.0;
const double MAX_Y_RPM_HD = 0; //HDrive

double MAX_FPS = 0; //used in auton pathfinder

const double ACTUAL_MAX_Y_RPM_LOW = 625.0; //668 what
const double ACTUAL_MAX_Y_RPM_HIGH = 1300.0;
const double ACTUAL_MAX_Y_RPM_HD = 0;

double DYN_MAX_Y_RPM = 625.0; //for field-centric
const double MAX_X_RPM = 400.0; // for HDrive

const double MAX_YAW_RATE_LOW = 12.0; //max angular velocity divided by the max rpm multiplied by set max rpm
const double MAX_YAW_RATE_HIGH = 28.0;
const double MAX_YAW_RATE_HD = 0.0;

const double MAX_KICK_FPS = ((MAX_X_RPM * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
const int Kv_KICK = (1 / MAX_KICK_FPS);

const double UP_SHIFT_VEL = 375.0; // (24/14) *9370 //RPM
const double DOWN_SHIFT_VEL = 200.0; //will be less than up shift vel (14/56) *9370 //RPM

/////////////////////////////////////////////////////

const double DRIVE_WAIT_TIME = 0.05; //seconds
const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

const double FF_SCALE = 1.0;

double l_last_current;

// Drive Gains

//		Teleop
const double K_P_RIGHT_VEL_LOW = 0.001;
const double K_P_LEFT_VEL_LOW = 0.001;
const double K_D_RIGHT_VEL_LOW = 0.000;
const double K_D_LEFT_VEL_LOW = 0.000;
const double K_P_YAW_VEL_LOW = 30.0;//85.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.001;
const double K_P_LEFT_VEL_HIGH = 0.001;
const double K_D_RIGHT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;
const double K_P_YAW_VEL_HIGH = 10.0;//120.0;
const double K_D_YAW_VEL_HIGH = 0.000;

const double K_P_YAW_HEADING_POS_HD = 0.0;
const double K_D_VISION_POS_HD = 0.0;
const double K_P_YAW_HEADING_POS_WC = 0.01;
const double K_D_VISION_POS_WC = 0.0;

const double K_P_KICK_VEL = 0.00365; //0.00311
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 1.0 / 400.0;

//		Auton
const double K_P_YAW_AU_HD = 0.0; //5.0;
const double K_D_YAW_AU_HD = 0.0; //0.085;
const double K_P_YAW_AU_WC = 0.0; //5.0;
const double K_D_YAW_AU_WC = 0.0; //0.085;

//const double k_p_right_vel_au = 0;
//const double k_p_left_vel_au = 0; k_p_kick_vel_au,
////			k_d_left_vel_au, k_d_right_vel_au, k_d_kick_vel_au

const double K_P_RIGHT_DIS = 0.1;//0.085; //0.1;
const double K_P_LEFT_DIS = 0.1;//0.085; // 0.1;
const double K_P_YAW_DIS = 0.5; //1.5;
const double K_P_KICKER_DIS = 0.280;

const double K_I_RIGHT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;
const double K_I_YAW_DIS = 0.0;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0;

// Drive Gains End

//Dynamic Values on down, should not be all-caps

double P_LEFT_VEL = 0;
double D_LEFT_VEL = 0;
double d_left_vel = 0;

double P_RIGHT_VEL = 0;
double D_RIGHT_VEL = 0;
double d_right_vel = 0;

double P_KICK_VEL = 0;
double D_KICK_VEL = 0;
double d_kick_vel = 0;

double P_RIGHT_DIS = 0;
double I_RIGHT_DIS = 0;
double D_RIGHT_DIS = 0;

double P_LEFT_DIS = 0;
double I_LEFT_DIS = 0;
double D_LEFT_DIS = 0;

double P_KICK_DIS = 0;
double I_KICK_DIS = 0;
double D_KICK_DIS = 0;

double P_YAW_DIS = 0;
double I_YAW_DIS = 0;

double d_vision = 0;

double d_right = 0;
double i_right = 0;

double d_yaw_dis = 0;

double d_left = 0;
double i_left = 0;

double d_kick = 0;
double i_kick = 0;

double i_yaw = 0;

double l_error_vel_au = 0;
double l_error_dis_au = 0;

double r_error_vel_au = 0;
double r_error_dis_au = 0;

double k_error_dis_au = 0;

double y_error_dis_au = 0; // yaw (theta) position error

double l_error_vel_t = 0;
double l_error_dis_t = 0;

double r_error_vel_t = 0;
double r_error_dis_t = 0;

double kick_error_vel = 0;

double l_last_error = 0;
double r_last_error = 0;
double yaw_last_error = 0;
double kick_last_error = 0;

double l_last_error_vel = 0;
double r_last_error_vel = 0;
double kick_last_error_vel = 0;

double max_y_rpm, actual_max_y_rpm, max_yaw_rate;

double k_p_right_vel, k_p_left_vel, k_p_yaw_vel, k_d_right_vel,
		k_d_left_vel, //gains vary depending on gear
		k_p_yaw_t, k_d_yaw_t, k_p_kick_vel, k_d_kick_vel, k_p_yaw_h_vel,
		k_p_yaw_au, k_d_yaw_au;
double k_p_right_vel_au = 0.0;
double k_p_left_vel_au = 0.0;
double k_p_kick_vel_au = 0.0;
double k_d_left_vel_au = 0.0;
double k_d_right_vel_au = 0.0;
double k_d_kick_vel_au = 0.0;

double k_p_yaw_heading_pos, k_d_vision_pos;

double k_f_left_vel, k_f_right_vel;

int row_index = 0;

bool is_last_index = false;
//int drive_index = 0;

double Kv; //scale from -1 to 1

Timer *timerTeleop = new Timer();
//Timer *timerAuton = new Timer();
//Timer *timerShift = new Timer();
//
//Timer *timerTest = new Timer();

double init_heading = 0;
double total_heading = 0;

bool set_profile = false;

bool tank = false;
bool is_low_gear = true;

int LF = 0, L2 = 0, L3 = 0, LR = 0, RF = 0, R2 = 0, R3 = 0, RR = 0, KICKER = 0;

std::vector<double> drive_ref = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //AutonDrive, row, individual points
std::vector<std::vector<double> > auton_profile(1500, std::vector<double>(6)); //rows stacked on rows, all points

DriveControllerMother::DriveControllerMother(int fl, int fr, int rl, int rr,
		int k, bool is_wc, bool start_low) { //not used and not complete

	max_y_rpm = MAX_Y_RPM_HD;
	actual_max_y_rpm = ACTUAL_MAX_Y_RPM_HD;
	max_yaw_rate = MAX_YAW_RATE_HD;
	k_p_yaw_au = K_P_YAW_AU_HD;
	k_d_yaw_au = K_D_YAW_AU_HD;
	k_p_yaw_heading_pos = K_P_YAW_HEADING_POS_HD;
	k_d_vision_pos = K_D_VISION_POS_HD;

	if (is_wc) {
		tank = true;
	}

	LF = fl;
	LR = rl;
	RF = fr;
	RR = rr;
	KICKER = k;

	canTalonLeft1 = new TalonSRX(LF);
	canTalonLeft1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	canTalonLeft2 = new TalonSRX(LR);
	canTalonLeft2->Set(ControlMode::Follower, LF);

	canTalonRight1 = new TalonSRX(RF);
	canTalonRight1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	canTalonRight2 = new TalonSRX(RR);
	canTalonRight2->Set(ControlMode::Follower, RF);

	canTalonKicker = new TalonSRX(KICKER);

	canTalonLeft1->ConfigPeakCurrentLimit(30, 0);
	canTalonLeft2->ConfigPeakCurrentLimit(30, 0);
	canTalonRight1->ConfigPeakCurrentLimit(30, 0);
	canTalonRight2->ConfigPeakCurrentLimit(30, 0);
	canTalonKicker->ConfigPeakCurrentLimit(30, 0);

	ahrs = new AHRS(SPI::Port::kMXP, 200);

	canTalonRight3 = new TalonSRX(-1);
	canTalonRight4 = new TalonSRX(-1);
	canTalonLeft3 = new TalonSRX(-1);
	canTalonLeft4 = new TalonSRX(-1);
	solenoid = new DoubleSolenoid(-1, -1, -1);

}

//We use this one
DriveControllerMother::DriveControllerMother(int l1, int l2, int l3, int l4,
		int r1, int r2, int r3, int r4, bool start_low) {

	k_p_yaw_au = K_P_YAW_AU_WC;
	k_d_yaw_au = K_D_YAW_AU_WC;

	if (start_low) { //CANNOT CALL OTHER FUNCTIONS IN THE CONSTRUCTOR

		max_y_rpm = MAX_Y_RPM_LOW;
		max_yaw_rate = MAX_YAW_RATE_LOW;

		actual_max_y_rpm = ACTUAL_MAX_Y_RPM_LOW;

		MAX_FPS = 19.5;//((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
		Kv = (1 / MAX_FPS);
		max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm;

		k_p_right_vel = K_P_RIGHT_VEL_LOW;
		k_p_left_vel = K_P_LEFT_VEL_LOW;
		k_p_yaw_t = K_P_YAW_VEL_LOW;
		k_d_yaw_t = K_D_YAW_VEL_LOW;
		k_d_right_vel = K_D_RIGHT_VEL_LOW;
		k_d_left_vel = K_D_LEFT_VEL_LOW;

		k_f_left_vel = 1.0 / actual_max_y_rpm;
		k_f_right_vel = 1.0 / actual_max_y_rpm;

		is_low_gear = true;

	} else {

		max_y_rpm = MAX_Y_RPM_HIGH;
		max_yaw_rate = MAX_YAW_RATE_HIGH;

		actual_max_y_rpm = ACTUAL_MAX_Y_RPM_HIGH;

		MAX_FPS = 19.5;//((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
		Kv = (1 / MAX_FPS);
		max_yaw_rate = (25 / actual_max_y_rpm) * max_y_rpm;

		k_p_right_vel = K_P_RIGHT_VEL_HIGH;
		k_p_left_vel = K_P_LEFT_VEL_HIGH;
		k_p_yaw_t = K_P_YAW_VEL_HIGH;
		k_d_yaw_t = K_P_YAW_VEL_HIGH;
		k_d_right_vel = K_D_RIGHT_VEL_HIGH;
		k_d_left_vel = K_D_LEFT_VEL_HIGH;

		k_f_left_vel = 1.0 / actual_max_y_rpm;
		k_f_right_vel = 1.0 / actual_max_y_rpm;

		is_low_gear = false;

	}

	tank = true;

	LF = l1;
	L2 = l2;
	L3 = l3;
	LR = l4;
	RF = r1;
	R2 = r2;
	R3 = r3;
	RR = r4;

	canTalonLeft1 = new TalonSRX(LF);
	canTalonLeft1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	canTalonLeft2 = new TalonSRX(L2);
	canTalonLeft2->Set(ControlMode::Follower, LF);

	canTalonLeft3 = new TalonSRX(L3);
	canTalonLeft3->Set(ControlMode::Follower, LF);

	canTalonLeft4 = new TalonSRX(LR);
	canTalonLeft4->Set(ControlMode::Follower, LF);

	canTalonRight1 = new TalonSRX(RF);
	canTalonRight1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

	canTalonRight2 = new TalonSRX(R2);
	canTalonRight2->Set(ControlMode::Follower, RF);

	canTalonRight3 = new TalonSRX(R3);
	canTalonRight3->Set(ControlMode::Follower, RF);

	canTalonRight4 = new TalonSRX(RR);
	canTalonRight4->Set(ControlMode::Follower, RF);

//	canTalonLeft1->EnableCurrentLimit(true);
//	canTalonLeft2->EnableCurrentLimit(true);
//	canTalonLeft3->EnableCurrentLimit(true);
//	canTalonLeft4->EnableCurrentLimit(true);
//	canTalonRight1->EnableCurrentLimit(true);
//	canTalonRight2->EnableCurrentLimit(true);
//	canTalonRight3->EnableCurrentLimit(true);
//	canTalonRight4->EnableCurrentLimit(true);

	canTalonLeft1->ConfigPeakCurrentLimit(100, 0);
	canTalonLeft2->ConfigPeakCurrentLimit(100, 0);
	canTalonLeft3->ConfigPeakCurrentLimit(100, 0);
	canTalonLeft4->ConfigPeakCurrentLimit(100, 0);
	canTalonRight1->ConfigPeakCurrentLimit(100, 0);
	canTalonRight2->ConfigPeakCurrentLimit(100, 0);
	canTalonRight3->ConfigPeakCurrentLimit(100, 0);
	canTalonRight4->ConfigPeakCurrentLimit(100, 0);

	canTalonLeft1->ConfigContinuousCurrentLimit(55, 0);
	canTalonLeft2->ConfigContinuousCurrentLimit(55, 0);
	canTalonLeft3->ConfigContinuousCurrentLimit(55, 0);
	canTalonLeft4->ConfigContinuousCurrentLimit(55, 0);
	canTalonRight1->ConfigContinuousCurrentLimit(55, 0);
	canTalonRight2->ConfigContinuousCurrentLimit(55, 0);
	canTalonRight3->ConfigContinuousCurrentLimit(55, 0);
	canTalonRight4->ConfigContinuousCurrentLimit(55, 0);

	canTalonLeft1->ConfigPeakCurrentDuration(400, 0);
	canTalonLeft2->ConfigPeakCurrentDuration(400, 0);
	canTalonLeft3->ConfigPeakCurrentDuration(400, 0);
	canTalonLeft4->ConfigPeakCurrentDuration(400, 0);
	canTalonRight1->ConfigPeakCurrentDuration(400, 0);
	canTalonRight2->ConfigPeakCurrentDuration(400, 0);
	canTalonRight3->ConfigPeakCurrentDuration(400, 0);
	canTalonRight4->ConfigPeakCurrentDuration(400, 0);

	canTalonLeft1->ConfigOpenloopRamp(0.15, 0); //TODO: adjust this as needed
	canTalonLeft2->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft3->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft4->ConfigOpenloopRamp(0.15, 0);
	canTalonRight1->ConfigOpenloopRamp(0.15, 0);
	canTalonRight2->ConfigOpenloopRamp(0.15, 0);
	canTalonRight3->ConfigOpenloopRamp(0.15, 0);
	canTalonRight4->ConfigOpenloopRamp(0.15, 0);

	canTalonLeft1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft1->ConfigVelocityMeasurementWindow(5, 0);
	canTalonLeft2->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft2->ConfigVelocityMeasurementWindow(5, 0);
	canTalonLeft3->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft3->ConfigVelocityMeasurementWindow(5, 0);
	canTalonLeft4->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft4->ConfigVelocityMeasurementWindow(5, 0);

	canTalonRight1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight1->ConfigVelocityMeasurementWindow(5, 0);
	canTalonRight2->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight2->ConfigVelocityMeasurementWindow(5, 0);
	canTalonRight3->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight3->ConfigVelocityMeasurementWindow(5, 0);
	canTalonRight4->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight4->ConfigVelocityMeasurementWindow(5, 0);

	//ahrs = new AHRS(SerialPort::Port::kUSB, AHRS::SerialDataType::kProcessedData, 200);
	ahrs = new AHRS(SerialPort::kUSB);
	//ahrs = new AHRS(SerialPort::Port::kMXP);

	solenoid = new DoubleSolenoid(1, 0, 1);

	canTalonKicker = new TalonSRX(-1);

}

void DriveControllerMother::ShiftUp() { //high gear, inside

//	SmartDashboard::PutString("GEAR", "HIGH");

	solenoid->Set(DoubleSolenoid::Value::kForward);
	SetGainsHigh();

}

void DriveControllerMother::ShiftDown() { //low gear, outside

//	SmartDashboard::PutString("GEAR", "LOW");

	solenoid->Set(DoubleSolenoid::Value::kReverse);
	//std::cout << "DOWN" << std::endl;

	SetGainsLow(); //separate for gains in case we want to initialize them by themselves in the constructor

}

void DriveControllerMother::SetGainsHigh() {

	max_y_rpm = MAX_Y_RPM_HIGH;
	max_yaw_rate = MAX_YAW_RATE_HIGH;

	actual_max_y_rpm = ACTUAL_MAX_Y_RPM_HIGH;

	MAX_FPS = 19.5;//((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
	Kv = (1 / MAX_FPS);
	max_yaw_rate = (25 / actual_max_y_rpm) * max_y_rpm;

	k_p_right_vel = K_P_RIGHT_VEL_HIGH;
	k_p_left_vel = K_P_LEFT_VEL_HIGH; //TODO: change for auton
	k_p_yaw_t = K_P_YAW_VEL_HIGH;
	k_d_yaw_t = K_P_YAW_VEL_HIGH;
	k_d_right_vel = K_D_RIGHT_VEL_HIGH;
	k_d_left_vel = K_D_LEFT_VEL_HIGH;

	k_f_left_vel = 1.0 / actual_max_y_rpm;
	k_f_right_vel = 1.0 / actual_max_y_rpm;

	is_low_gear = false;

}

void DriveControllerMother::SetGainsLow() {

	max_y_rpm = MAX_Y_RPM_LOW;
	max_yaw_rate = MAX_YAW_RATE_LOW;

	actual_max_y_rpm = ACTUAL_MAX_Y_RPM_LOW;

	MAX_FPS = 19.5;//((max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
	Kv = (1 / MAX_FPS);
	max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm; //(max_yaw_rate / actual_max_y_rpm) * set_max_y_rpm

	k_p_right_vel = K_P_RIGHT_VEL_LOW;
	k_p_left_vel = K_P_LEFT_VEL_LOW;
	k_p_yaw_t = K_P_YAW_VEL_LOW;
	k_d_yaw_t = K_D_YAW_VEL_LOW;
	k_d_right_vel = K_D_RIGHT_VEL_LOW;
	k_d_left_vel = K_D_LEFT_VEL_LOW;

	k_f_left_vel = 1.0 / actual_max_y_rpm;
	k_f_right_vel = 1.0 / actual_max_y_rpm;

	is_low_gear = true;

}

void DriveControllerMother::AutoShift() {

	double current_rpm_l = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	double current_rpm_r = -((double) canTalonRight1->GetSelectedSensorVelocity(
			0) / (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

//	SmartDashboard::PutNumber("left vel", current_rpm_l);
//	SmartDashboard::PutNumber("right vel", current_rpm_r);

	if (std::abs(current_rpm_l) > UP_SHIFT_VEL
			&& std::abs(current_rpm_r) > UP_SHIFT_VEL && is_low_gear) {
		ShiftUp();
	}

	//if in between, will stay in the gear it is in. in order to not shift back and forth at one point

	else if (std::abs(current_rpm_l) < DOWN_SHIFT_VEL
			&& std::abs(current_rpm_r) < DOWN_SHIFT_VEL && !is_low_gear) {
//		timerShift->Start();
//		if (timerShift->Get() > 3.0) {
//			ShiftDown();
//			timerShift->Reset();
//		}

	}

}

void DriveControllerMother::TeleopHDrive(Joystick *JoyThrottle,
		Joystick *JoyWheel, bool *is_fc) {

	double forward = -1.0 * (JoyThrottle->GetY());
	double strafe = (JoyThrottle->GetX());
	double current_yaw = (fmod((-1.0 * ahrs->GetRate() * (PI / 180.0)),
			(2.0 * PI))); // yaw position

	if ((bool) *is_fc) {

		if (current_yaw < -PI) {
			current_yaw += (2.0 * PI);
		} else if (current_yaw > PI) {
			current_yaw -= (2.0 * PI);
		}

		double psi = std::atan2(forward, strafe);

		double magnitude = sqrt((forward) + (strafe));

		forward = magnitude * (std::sin(psi - current_yaw));
		strafe = magnitude * (std::cos(psi - current_yaw));

		if ((psi - current_yaw) > (-1.0 * PI) && (psi - current_yaw) <= (0)) {
			if (forward > 0) {
				forward = -1.0 * forward;
			}
		}
		if (((psi - current_yaw) > (PI / 2) && (psi - current_yaw) <= (PI))
				|| ((psi - current_yaw) > (-1.0 * PI)
						&& (psi - current_yaw) <= (-1.0 * PI / 2))) {
			if (strafe > 0) {
				strafe = -1.0 * strafe;
			}
		}
	} else {

		forward = 1.0 * forward; //not needed

	}

	double target_l, target_r, target_kick, target_yaw_rate;

	double axis_ratio = 0.0; //ratio between x and y axes

	if (strafe != 0) { //if x is 0, it is undefined (division)
		axis_ratio = std::abs(forward / strafe); //dont use regular abs, that returns an int
		DYN_MAX_Y_RPM = MAX_X_RPM * (double) axis_ratio;
		DYN_MAX_Y_RPM = DYN_MAX_Y_RPM > max_y_rpm ? max_y_rpm : DYN_MAX_Y_RPM; //if DYN_max_Y is bigger than MAX_Y then set to MAX_Y, otherwise keep DYN_MAX_Y
	} else {
		DYN_MAX_Y_RPM = max_y_rpm; //deals with special case tht x = 0 (ratio cant divide by zero)
	}

	target_l = 1.0 * (forward < 0 ? -1 : 1) * (forward * forward)
			* DYN_MAX_Y_RPM; //if joy value is less than 0 set to -1, otherwise to 1

	target_r = target_l;

	target_kick = 1.0 * (strafe < 0 ? -1 : 1) * (strafe * strafe) * MAX_X_RPM; //if joy value is less than 0 set to -1, otherwise to 1

	double joy_wheel_val = JoyWheel->GetX();

	if (std::abs(joy_wheel_val) < .02) {
		joy_wheel_val = 0.0;
	}

	target_yaw_rate = -1.0 * (joy_wheel_val) * max_yaw_rate; //Left will be positive

	if (abs(target_kick) < 35) {
		target_kick = 0;
	}

	if (target_l > max_y_rpm) {
		target_l = max_y_rpm;
	} else if (target_l < -max_y_rpm) {
		target_l = -max_y_rpm;
	}

	if (target_r > max_y_rpm) {
		target_r = max_y_rpm;
	} else if (target_r < -max_y_rpm) {
		target_r = -max_y_rpm;
	}

	Controller(target_kick, target_r, target_l, target_yaw_rate, k_p_right_vel,
			k_p_left_vel, k_p_kick_vel, k_p_yaw_t, 0.0, k_d_left_vel,
			k_d_right_vel, k_d_kick_vel, 0.0, 0.0, 0.0);

}

void DriveControllerMother::TeleopWCDrive(Joystick *JoyThrottle, //finds targets for the Controller()
		Joystick *JoyWheel) {

	double target_l, target_r, target_yaw_rate;

	double reverse_y = 1.0;

	if (JoyThrottle->GetY() > 0.0) {
		reverse_y = -1.0;
	} else {
		reverse_y = 1.0;
	}
	double forward = (JoyThrottle->GetY()) * (JoyThrottle->GetY()); //SQUARED. will always be positive

	target_l = reverse_y * forward * max_y_rpm;

	target_r = target_l;

	double reverse_x = 1.0; //square

//	if (JoyWheel->GetX() < 0.0) {
//		reverse_x = -1.0;
//	} else {
//		reverse_x = 1.0;
//	}

	double joy_wheel_val = 1.0 * JoyWheel->GetX(); //not SQUARED //*reverse_x

//	if (!is_low_gear) { //squrare wheel in high gear
//		joy_wheel_val *= reverse_x * JoyWheel->GetX();
//	}

	if (std::abs(joy_wheel_val) < .03) {
		joy_wheel_val = 0.0;
	}

	target_yaw_rate = -1.0 * (joy_wheel_val) * max_yaw_rate; //Left will be positive

	if (target_l > max_y_rpm) {
		target_l = max_y_rpm;
	} else if (target_l < -max_y_rpm) {
		target_l = -max_y_rpm;
	}

	if (target_r > max_y_rpm) {
		target_r = max_y_rpm;
	} else if (target_r < -max_y_rpm) {
		target_r = -max_y_rpm;
	}

	//std::cout << "target yaw rate " << target_yaw_rate << std::endl;

	Controller(0.0, target_r, target_l, target_yaw_rate, k_p_right_vel,
			k_p_left_vel, 0.0, k_p_yaw_t, 0.0, k_d_left_vel, k_d_right_vel, 0.0,
			0.0, 0.0, 0.0);

}

void DriveControllerMother::RotationController(Joystick *JoyWheel) {

	double target_heading = init_heading
			+ (-1.0 * JoyWheel->GetX() * (90.0 * PI / 180.0)); //scaling, conversion to radians,left should be positive

	double current_heading = -1.0 * ahrs->GetYaw() * ( PI / 180.0); //degrees to radians, left should be positive

	double error_heading_h = target_heading - current_heading;

	double total_heading_h = k_p_yaw_heading_pos * error_heading_h;

	if (total_heading > max_yaw_rate) {
		total_heading = max_yaw_rate;
	} else if (total_heading < -max_yaw_rate) {
		total_heading = -max_yaw_rate;
	}

	Controller(0.0, 0.0, 0.0, total_heading_h, k_p_right_vel, k_p_left_vel,
			k_p_kick_vel, k_p_yaw_h_vel, 0.0, k_d_right_vel, k_d_left_vel,
			k_d_kick_vel, 0.0, 0.0, 0.0);

}

/**
 * Param: Feet forward, + = forward
 */
//position controlller
//auton targets, actually just pd
void DriveControllerMother::AutonDrive() { //yaw pos, left pos, right pos, yaw vel, left vel, right vel

	double refYaw = drive_ref.at(0); //reversed in Generate
	double refLeft = drive_ref.at(1);
	double refRight = drive_ref.at(2);
	double targetYawRate = drive_ref.at(3); //0 for now
	double tarVelLeft = drive_ref.at(4);
	double tarVelRight = drive_ref.at(5);

	if(refYaw > PI) {
		refYaw -= 2*PI;
	}

	SmartDashboard::PutNumber("refLeft", refLeft);
	SmartDashboard::PutNumber("refRight", refRight);
	SmartDashboard::PutNumber("refLeftVel", tarVelLeft);
	SmartDashboard::PutNumber("refRightVel", tarVelRight);
	SmartDashboard::PutNumber("refYaw", refYaw);

//	std::cout << "yep " << refYaw << "  " << refLeft << "  " << refRight << "  "
//			<< targetYawRate << "  " << tarVelLeft << "   " << tarVelRight
//			<< std::endl;

	//SmartDashboard::PutNumber("PROF VEL", tarVelLeft);

	//rpm //not needed besides check for jitter
	double r_current = -((double) canTalonRight1->GetSelectedSensorVelocity(0)
				/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
				/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	//SmartDashboard::PutNumber("Actual left", l_current);

	//feet
//	double r_dis = -(((double) canTalonRight1->GetSelectedSensorPosition(0) //rotations per 100 ms, distance (circumference) per 100m ms,
//	/ TICKS_PER_ROT) * (WHEEL_DIAMETER * PI) / 12);
//	double l_dis = (((double) canTalonLeft1->GetSelectedSensorPosition(0)
//			/ TICKS_PER_ROT) * (WHEEL_DIAMETER * PI) / 12);

	double r_dis = -((double) canTalonRight1->GetSelectedSensorPosition(0) / 1205.0);
	double l_dis = ((double) canTalonLeft1->GetSelectedSensorPosition(0) / 1205.0);

	SmartDashboard::PutNumber("actualLeftDis", l_dis);
	SmartDashboard::PutNumber("actualRightDis", r_dis);
	SmartDashboard::PutNumber("actualLeftVel", l_current);
	SmartDashboard::PutNumber("actualRightVel", r_current);


	//std::cout << "Right: " << r_dis << " Left: " << l_dis << std::endl;

	//double l_curr_fps = ((l_current * WHEEL_DIAMETER * PI) / 12.0) / 60.0;

	//std::cout << "pos: " << l_dis << " vel: " << l_curr_fps << std::endl;

	//SmartDashboard::PutNumber("POS", l_dis);

	double y_dis = -1.0 * ahrs->GetYaw() * (double) (PI / 180); //current theta (yaw) value

	SmartDashboard::PutNumber("Target Heading", refYaw);
	SmartDashboard::PutNumber("Actual Heading", y_dis);

	l_error_dis_au = refLeft - l_dis;
	r_error_dis_au = refRight - r_dis;

	y_error_dis_au = refYaw - y_dis;

	if (std::abs(tarVelLeft - tarVelRight) < .05 && (std::abs(r_current) < 10)
			&& (std::abs(l_current) < 10)) { //initial jitter

	}

	i_right += (r_error_dis_au);
	d_right = (r_error_dis_au - r_last_error);

	i_left += (l_error_dis_au);
	d_left = (l_error_dis_au - l_last_error);

	d_kick = (k_error_dis_au - kick_last_error);

	i_yaw += y_error_dis_au;

	P_RIGHT_DIS = K_P_RIGHT_DIS * r_error_dis_au;
	I_RIGHT_DIS = K_I_RIGHT_DIS * i_right;
	D_RIGHT_DIS = K_D_RIGHT_DIS * d_right;

	P_LEFT_DIS = K_P_LEFT_DIS * l_error_dis_au;
	I_LEFT_DIS = K_I_LEFT_DIS * i_left;
	D_LEFT_DIS = K_D_LEFT_DIS * d_left;

	P_YAW_DIS = K_P_YAW_DIS * y_error_dis_au; //position
	I_YAW_DIS = K_I_YAW_DIS * i_yaw;

	double total_right = P_RIGHT_DIS + I_RIGHT_DIS + D_RIGHT_DIS;
	double total_left = P_LEFT_DIS + I_LEFT_DIS + D_LEFT_DIS;

	double total_yaw = P_YAW_DIS + I_YAW_DIS;
	double target_rpm_yaw_change = total_yaw * MAX_FPS;
	double target_rpm_right = total_right * MAX_FPS; //max rpm* gear ratio
	double target_rpm_left = total_left * MAX_FPS;

	target_rpm_right = target_rpm_right + target_rpm_yaw_change + tarVelRight; //even when feedback is 0, will send actual target rpm to Controller
	target_rpm_left = target_rpm_left - target_rpm_yaw_change + tarVelLeft;

	//std::cout << "FB: " << target_rpm_right << std::endl;

	if (target_rpm_left > MAX_FPS) {
		target_rpm_left = MAX_FPS;
	} else if (target_rpm_left < -MAX_FPS) {
		target_rpm_left = -MAX_FPS;
	}
//target rpm right is in fps
	if (target_rpm_right > MAX_FPS) {
		target_rpm_right = MAX_FPS;
	} else if (target_rpm_right < -MAX_FPS) {
		target_rpm_right = -MAX_FPS;
	}

//	std::cout << "yep " << target_rpm_right << "  " << target_rpm_left << "  " << targetYawRate  << "  " << tarVelLeft <<  "   " << tarVelRight << std::endl;
//target rpm right, leftt
	Controller(0.0, 0.0, 0.0, targetYawRate,
			k_p_right_vel_au, k_p_left_vel_au, k_p_kick_vel_au, k_p_yaw_au, k_d_yaw_au,
			k_d_left_vel_au, k_d_right_vel_au, k_d_kick_vel_au, target_rpm_left, target_rpm_right,
			0.0);

	l_last_error = l_error_dis_au;
	r_last_error = r_error_dis_au;
	kick_last_error = k_error_dis_au;

}

//TODO: add check for encoders working

void DriveControllerMother::Controller(double ref_kick, double ref_right,
		double ref_left, double ref_yaw, double k_p_right, double k_p_left,
		double k_p_kick, double k_p_yaw, double k_d_yaw, double k_d_right,
		double k_d_left, double k_d_kick, double target_vel_left,
		double target_vel_right, double target_vel_kick) {

	double yaw_rate_current = -1.0 * (double) ahrs->GetRate()
			* (double) ((PI) / 180.0); //left should be positive

	double target_yaw_rate = ref_yaw;

	ref_left = ref_left - (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //left should be positive
	ref_right = ref_right + (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //0

	double yaw_error = target_yaw_rate - yaw_rate_current;

	//std::cout << "left: " << ref_left << std::endl;

//	SmartDashboard::PutNumber("yaw current", yaw_rate_current);
//	SmartDashboard::PutNumber("yaw target", target_yaw_rate);
//	SmartDashboard::PutNumber("yaw error", yaw_error);

	double l_dis = (((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_ROT) * (WHEEL_DIAMETER * PI) / 12);

	//std::cout << "Right: " << r_dis << " Left: " << l_dis << std::endl;

	//SmartDashboard::PutNumber("POS", l_dis);

	if (std::abs(yaw_error) < .3) {
		yaw_error = 0.0;
	}

	d_yaw_dis = yaw_error - yaw_last_error;

	double yaw_output = ((k_p_yaw * yaw_error) + (k_d_yaw * d_yaw_dis)); //pd for auton, p for teleop

	ref_right += yaw_output; //left should be positive
	ref_left -= yaw_output;

	if (std::abs(ref_kick) < 25) {
		ref_kick = 0;
	}

	if (ref_left > max_y_rpm) {
		ref_left = max_y_rpm;
	} else if (ref_left < -max_y_rpm) {
		ref_left = -max_y_rpm;
	}

	if (ref_right > max_y_rpm) {
		ref_right = max_y_rpm;
	} else if (ref_right < -max_y_rpm) {
		ref_right = -max_y_rpm;
	}

	double feed_forward_r = k_f_right_vel * ref_right; //teleop only, controlled
	double feed_forward_l = k_f_left_vel * ref_left;
	double feed_forward_k = K_F_KICK_VEL * ref_kick;

	//conversion to RPM from native unit
	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double r_current = -((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double kick_current = ((double) canTalonKicker->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION; //going right is positive

//	SmartDashboard::PutNumber("Velocity", l_current);

	double curr_fps = (l_current * TICKS_PER_ROT/1205.0) / 60.0;

	SmartDashboard::PutNumber("Left fps", curr_fps);

	//timerTest->Start();

	//std::cout << "VEL: " << l_current << " VEL: " << r_current << std::endl;

	if ((std::abs(l_current) <= 0.5 && canTalonLeft1->GetOutputCurrent() > 4.0) //encoders not working
			|| (std::abs(r_current) <= 0.5
					&& canTalonRight1->GetOutputCurrent() > 4.0)) {
	//	SmartDashboard::PutString("Drive Motor Encoders", "Not working");
		k_p_yaw = 0.0;
		k_d_yaw = 0.0;
		feed_forward_l = 0.0;
		feed_forward_r = 0.0;
		k_p_left = 0.0;
		k_p_right = 0.0;
		k_d_left = 0.0;
		k_d_right = 0.0;
	}

//	SmartDashboard::PutNumber("Left vel", l_current);
//	SmartDashboard::PutNumber("Right vel.", r_current);

	//std::cout << "left: " << l_current << "  right: " << r_current << std::endl;
	//std::cout << "yaw: " << yaw_error << std::endl;

	l_error_vel_t = ref_left - l_current;
	r_error_vel_t = ref_right - r_current;
	kick_error_vel = ref_kick - kick_current;

	d_left_vel = (l_error_vel_t - l_last_error_vel);
	d_right_vel = (r_error_vel_t - r_last_error_vel);
	d_kick_vel = (kick_error_vel - kick_last_error_vel);

//	if ((ref_left < -5.0) && (ref_right < -5.0)) { //not straight when going back, may be ok
//		std::cout << "HERE" << std::endl;
//		if (is_low_gear) {
//			k_p_left = 0.004;
//		} else {
//			k_p_left = 0.003;
//		}
//
//	}

	P_LEFT_VEL = k_p_left * l_error_vel_t; //non-zero
	P_RIGHT_VEL = k_p_right * r_error_vel_t;
	P_KICK_VEL = k_p_kick * kick_error_vel;

	D_LEFT_VEL = k_d_left * d_left_vel;
	D_RIGHT_VEL = k_d_right * d_right_vel;
	D_KICK_VEL = k_d_kick * d_kick_vel;

	if (frc::RobotState::IsAutonomous()) { //only want the feedforward based off the motion profile during autonomous. The root generated ones (in the if() statement)
		feed_forward_r = 0;	// will be close to 0  (low error between profile points) for the most part but will get quite aggressive when an error builds,
		feed_forward_l = 0;			//the PD controller should handle it itself
		feed_forward_k = 0;
	}

	//std::cout << "Kv: " << Kv << " fps: " << MAX_FPS << std::endl;
	double ff_dr = Kv * target_vel_right;

	double total_right = D_RIGHT_VEL + P_RIGHT_VEL + feed_forward_r
			+ (Kv * target_vel_right); //Kv only in auton, straight from motion profile
	double total_left = D_LEFT_VEL + P_LEFT_VEL + feed_forward_l
			+ (Kv * target_vel_left);
	double total_kick = D_KICK_VEL + P_KICK_VEL + feed_forward_k
			+ (Kv_KICK * target_vel_kick);

	SmartDashboard::PutNumber("FF", ff_dr);
	//std::cout << "FF: " << ff_dr << " total: " << total_right << std::endl;

	//std::cout << "total right: " << total_right << "  total left: " << total_left << std::endl;

	if (total_right > 1.0) {
		total_right = 1.0;
	} else if (total_right < -1.0) {
		total_right = -1.0;
	}
	if (total_left > 1.0) {
		total_left = 1.0;
	} else if (total_left < -1.0) {
		total_left = -1.0;
	}

	//std::cout << "OUT: " << total_left << std::endl;

	canTalonLeft1->Set(ControlMode::PercentOutput, total_left); //Right is reversed
	canTalonRight1->Set(ControlMode::PercentOutput, -total_right); //negative for Koba and for new drive train
	//canTalonRearRight->Set(ControlMode::PercentOutput,  total_right); //these are slaves
	//canTalonRearLeft->Set(ControlMode::PercentOutput, -total_left);
	canTalonKicker->Set(ControlMode::PercentOutput, -total_kick);

	//	std::cout << " Ref: " << ref_kick;
	//std::cout << " Left: " << l_error_vel_t << std::endl;
	//std::cout << " Right: " << r_error_vel_t << std::endl; //high
	//  std::cout << " Error: " << kick_error_vel << std::endl;
	//std::cout << "YAW RATE: " << yaw_rate_current << std::endl;
	//std::cout << "YAW ERROR: " << yaw_error << std::endl;
	//	std::cout << "R: " << r_current;<< " L: " << l_current << std::endl;

	yaw_last_error = yaw_error;
	l_last_error_vel = l_error_vel_t;
	r_last_error_vel = r_error_vel_t;
	kick_last_error_vel = kick_error_vel;
	l_last_current = l_current;

	//double l_acc = (l_current - l_last_current) / timerTest->Get();

	//SmartDashboard::PutNumber("ACC", l_acc);

	//timerTest->Reset();

}

void DriveControllerMother::ZeroAll(bool stop_motors) {

	ZeroI();
	ZeroEncs();
	ZeroYaw();

	if (stop_motors) {
		StopAll();
	}

}

//will stop all driven motors in the drive controller
void DriveControllerMother::StopAll() {

	canTalonLeft1->Set(ControlMode::PercentOutput, 0.0); //0.0 because of the double, 0 should still work but better safe than sorry
	canTalonRight1->Set(ControlMode::PercentOutput, 0.0);
	//canTalonRearRight->Set(ControlMode::PercentOutput, 0.0); //slaved
	//canTalonRearLeft->Set(ControlMode::PercentOutput, 0.0);
	canTalonKicker->Set(ControlMode::PercentOutput, 0.0);

}

//sets the position of all the drive encoders to 0
void DriveControllerMother::ZeroEncs() { //acc to 8

	canTalonRight1->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft1->SetSelectedSensorPosition(0, 0, 0);
	canTalonRight2->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft2->SetSelectedSensorPosition(0, 0, 0);
	canTalonRight3->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft3->SetSelectedSensorPosition(0, 0, 0);
	canTalonRight4->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft4->SetSelectedSensorPosition(0, 0, 0);
	canTalonKicker->SetSelectedSensorPosition(0, 0, 0);

}

void DriveControllerMother::ZeroYaw() {

	ahrs->ZeroYaw();

}

//Zeros the accumulating I
void DriveControllerMother::ZeroI() {

	i_right = 0;
	i_left = 0;
	i_yaw = 0;

	y_error_dis_au = 0;
	l_error_dis_au = 0;
	r_error_dis_au = 0;

}

//profile input from the autonomous routine selector, most likely in robot.cpp
void DriveControllerMother::SetRefs(std::vector<std::vector<double>> profile) {

	auton_profile = profile;
	set_profile = true; //cannot re-enable to restart profile
	row_index = 0;

}

void DriveControllerMother::SetMaxRpm(double rpm) {

	max_y_rpm = rpm;

}

double DriveControllerMother::GetMaxRpm() {

	return max_y_rpm;

}

double DriveControllerMother::GetLeftPosition() {

	double l_dis = ((double) canTalonLeft1->GetSelectedSensorPosition(0) / 1205.0);

	return l_dis;

}

double DriveControllerMother::GetRightPosition() {

	double r_dis = -((double) canTalonLeft1->GetSelectedSensorPosition(0) / 1205.0);

	return r_dis;

}

bool DriveControllerMother::IsLastIndex() {

	return is_last_index;

}

int DriveControllerMother::GetDriveIndex() {

	return row_index;

}

//TODO: add vision support
void DriveControllerMother::DriveWrapper(Joystick *JoyThrottle,
		Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc,
		DriveControllerMother *driveController) {

	timerTeleop->Start();

	while (true) {

		timerTeleop->Reset();

		if (frc::RobotState::IsEnabled() && !frc::RobotState::IsAutonomous()) { //may have been problem that this was !auton frc::RobotState::IsOperatorControl()

			if (tank && !(bool) *is_heading && !(bool) *is_vision) {
				driveController->TeleopWCDrive(JoyThrottle, JoyWheel);
			}

			else if (!tank && !(bool) *is_heading && !(bool) *is_vision) {
				driveController->TeleopHDrive(JoyThrottle, JoyWheel, is_fc);
			}

			else if ((bool) *is_heading && !(bool) *is_vision) { //regardless of drivetrain type
				driveController->RotationController(JoyWheel);
			}

		}

		else if (frc::RobotState::IsEnabled() && frc::RobotState::IsAutonomous()
				&& set_profile) {

			//std::cout << "Auton drive" << std::endl;

			//put in profile //was finishing the for loop before we got a profile
			for (int i = 0; i < auton_profile[0].size(); i++) { //looks through each row and then fills drive_ref with the column here, refills each interval with next set of refs
				drive_ref.at(i) = auton_profile.at(row_index).at(i); //from SetRef()
				//std::cout << "actual: " << i << std::endl;
			}

			driveController->AutonDrive(); //send each row to auton drive before getting the next row

			row_index++;

		}

		double time_a = DRIVE_WAIT_TIME - timerTeleop->Get(); //how much time left to sleep till 10 ms have passed. timerTeleop->Get() returns seconds

		time_a *= 1000.0; //convert to ms

		if (time_a < 0.0) { //can't wait for negative time
			time_a = 0.0;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds((int) time_a));

		//SmartDashboard::PutNumber("time", timerTeleop->Get());

	}
}

void DriveControllerMother::StartDriveThreads(Joystick *JoyThrottle, //must pass in parameters to wrapper to use them in functions
		Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc) {

	DriveControllerMother *dc = this;

	DriveThread = std::thread(&DriveControllerMother::DriveWrapper, JoyThrottle,
			JoyWheel, is_heading, is_vision, is_fc, dc);
	DriveThread.detach();
}

void DriveControllerMother::EndDriveThreads() {

	//timerTeleop->Stop();
	DriveThread.~thread();

}
