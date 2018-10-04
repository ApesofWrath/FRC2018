#include "DriveControllerMother.h"
#include <WPILib.h>

#include "ctre/Phoenix.h"

#define CORNELIUS 1

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

double FF_SCALE = 0.7;

const double TICKS_PER_FOOT = 1315.0;

double l_last_current;

// Drive Gains

//		Teleop
const double K_P_RIGHT_VEL_LOW = 0.001;
const double K_P_LEFT_VEL_LOW = 0.001;
const double K_D_RIGHT_VEL_LOW = 0.000;
const double K_D_LEFT_VEL_LOW = 0.000;
const double K_P_YAW_VEL_LOW = 30.0; //85.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.001;
const double K_P_LEFT_VEL_HIGH = 0.001;
const double K_D_RIGHT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;
const double K_P_YAW_VEL_HIGH = 10.0; //120.0;
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

const double K_P_RIGHT_DIS = 0.15; //0.085; //0.1;
const double K_P_LEFT_DIS = 0.15; //0.085; // 0.1;//2.4
const double K_P_KICKER_DIS = 0.280;

const double K_I_RIGHT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0; //f

double K_P_YAW_DIS = 0.5; //0.2;3.3;//3.5; //1.5; //3.0 //was spending too much time making the turn and when it actually got to the shoot the gap part of the profile, the profile was already ahead of it
double K_I_YAW_DIS = 0.0; //3;//1;//0.0
double K_D_YAW_DIS = 0.0; //4.03.2;//4.0; //pd controller on yaw //20, p sum of p and d

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

int next_zero_index = 0;

int zero_counter = 0;
int zero_wait_counter = 0;

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

double D_YAW_DIS = 0.0;

double k_p_yaw_heading_pos, k_d_vision_pos;

double k_f_left_vel, k_f_right_vel;

bool is_last_index = false;

double Kv;

Timer *timerTeleop = new Timer();

double feed_forward_r, feed_forward_l, feed_forward_k;

bool is_zero;

double init_heading = 0;
double total_heading = 0;

bool tank = false;
bool is_low_gear = true;

int LF = 0, L2 = 0, L3 = 0, LR = 0, RF = 0, R2 = 0, R3 = 0, RR = 0, KICKER = 0;

std::vector<std::vector<double> > auton_profile(1500, std::vector<double>(6)); //rows stacked on rows, all points // can't be in .h for some reason
std::vector<std::vector<double> > auton_rows(2, std::vector<double>(6));

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

//	canTalonKicker = new TalonSRX(KICKER);
//TODO: try enabling in teleop
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
		int r1, int r2, int r3, int r4, bool start_low, double time_step) {

	time_step_drive = time_step;

	k_p_yaw_au = K_P_YAW_AU_WC; //these get sent from AutonDrive to Controller, not used in AutonDrive
	k_d_yaw_au = K_D_YAW_AU_WC;

	if (start_low) { //CANNOT CALL OTHER FUNCTIONS IN THE CONSTRUCTOR

		max_y_rpm = MAX_Y_RPM_LOW;
		max_yaw_rate = MAX_YAW_RATE_LOW;

		actual_max_y_rpm = ACTUAL_MAX_Y_RPM_LOW;

		MAX_FPS = 19.5; //((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
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

		MAX_FPS = 19.5; //((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
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

	canTalonLeft1->EnableCurrentLimit(true);
	canTalonLeft2->EnableCurrentLimit(true);
	canTalonLeft3->EnableCurrentLimit(true);
	canTalonLeft4->EnableCurrentLimit(true);
	canTalonRight1->EnableCurrentLimit(true);
	canTalonRight2->EnableCurrentLimit(true);
	canTalonRight3->EnableCurrentLimit(true);
	canTalonRight4->EnableCurrentLimit(true);

	canTalonLeft1->ConfigPeakCurrentLimit(40, 0); //40
	canTalonLeft2->ConfigPeakCurrentLimit(40, 0);
	canTalonLeft3->ConfigPeakCurrentLimit(40, 0);
	canTalonLeft4->ConfigPeakCurrentLimit(40, 0);
	canTalonRight1->ConfigPeakCurrentLimit(40, 0);
	canTalonRight2->ConfigPeakCurrentLimit(40, 0);
	canTalonRight3->ConfigPeakCurrentLimit(40, 0);
	canTalonRight4->ConfigPeakCurrentLimit(40, 0);

	canTalonLeft1->ConfigContinuousCurrentLimit(30, 0);
	canTalonLeft2->ConfigContinuousCurrentLimit(30, 0); //330
	canTalonLeft3->ConfigContinuousCurrentLimit(30, 0);
	canTalonLeft4->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight1->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight2->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight3->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight4->ConfigContinuousCurrentLimit(30, 0);

	canTalonLeft1->ConfigPeakCurrentDuration(10, 0);
	canTalonLeft2->ConfigPeakCurrentDuration(10, 0); //10
	canTalonLeft3->ConfigPeakCurrentDuration(10, 0);
	canTalonLeft4->ConfigPeakCurrentDuration(10, 0);
	canTalonRight1->ConfigPeakCurrentDuration(10, 0);
	canTalonRight2->ConfigPeakCurrentDuration(10, 0);
	canTalonRight3->ConfigPeakCurrentDuration(10, 0);
	canTalonRight4->ConfigPeakCurrentDuration(10, 0);

	canTalonLeft1->ConfigOpenloopRamp(0.15, 0); //TODO: adjust this as needed
	canTalonLeft2->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft3->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft4->ConfigOpenloopRamp(0.15, 0);
	canTalonRight1->ConfigOpenloopRamp(0.15, 0);

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

	canTalonLeft1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	canTalonRight1->SetStatusFramePeriod(
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0); //for getselectedsensor //getselectedsensor defaults to 10ms anyway. don't use getsensorcollection because that defaults to 160ms

	ahrs = new AHRS(SerialPort::kUSB);
#ifndef CORNELIUS
	solenoid = new DoubleSolenoid(3, 1, 0); //101
#else
	solenoid = new DoubleSolenoid(0, 0, 1);
#endif
	canTalonKicker = new TalonSRX(-1);

}

void DriveControllerMother::ShiftUp() { //high gear, inside

//	SmartDashboard::PutString("GEAR", "HIGH");
	std::cout << "shift up" << std::endl;

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

	MAX_FPS = 19.5; //((actual_max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
	Kv = (1 / MAX_FPS);
	max_yaw_rate = (25 / actual_max_y_rpm) * max_y_rpm;

	k_p_right_vel = K_P_RIGHT_VEL_HIGH; //these are all for teleop; we don't shift gears in auton
	k_p_left_vel = K_P_LEFT_VEL_HIGH;
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

	MAX_FPS = 19.5; //((max_y_rpm * WHEEL_DIAMETER * PI) / 12.0) / 60.0;
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

void DriveControllerMother::SetAutonGains(bool same_side_scale) {

	if (same_side_scale) {
		K_P_YAW_DIS = 1.68;
		K_I_YAW_DIS = 0.001;
		FF_SCALE = 0.7;
		//zero wait counter = 50
	} else { //if need another one

	}

}

void DriveControllerMother::AutoShift(bool auto_shift) { //not used

	double current_rpm_l = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	double current_rpm_r = -((double) canTalonRight1->GetSelectedSensorVelocity(
			0) / (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	if (std::abs(current_rpm_l) > UP_SHIFT_VEL
			&& std::abs(current_rpm_r) > UP_SHIFT_VEL && is_low_gear
			&& auto_shift) {
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

	double throttle = JoyThrottle->GetY();

	double reverse_y = 1.0;

	if (throttle > 0.0) {
		reverse_y = -1.0;
	} else {
		reverse_y = 1.0;
	}

	double forward = (throttle) * (throttle); //squared and will always be positive, so we need reverse_y

	target_l = reverse_y * forward * max_y_rpm; //scale to velocity

	target_r = target_l;

	double reverse_x = 1.0;

	double wheel = JoyWheel->GetX(); //not take time to get wheel/throttle values multiple times

	if (wheel < 0.0) {
		reverse_x = -1.0;
	} else {
		reverse_x = 1.0;
	}

	double joy_wheel_val = reverse_x * wheel * wheel;

//	if (!is_low_gear) { //squrare wheel in high gear
//		joy_wheel_val *= reverse_x * JoyWheel->GetX();
//	}

	if (std::abs(joy_wheel_val) < .02) {
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
	double targetYawRate = 0.0; //drive_ref.at(3); //0 for now
	double tarVelLeft = drive_ref.at(4);
	double tarVelRight = drive_ref.at(5);

	if (refYaw > PI) { //get negative half and positive half on circle
		refYaw -= (2 * PI);
	}

// SmartDashboard::PutNumber(":", refYaw);

	 SmartDashboard::PutNumber("refLeft", refLeft);
	// SmartDashboard::PutNumber("refRight", refRight);
	 SmartDashboard::PutNumber("refLeftVel", tarVelLeft);
	// SmartDashboard::PutNumber("refRightVel", tarVelRight);
	// SmartDashboard::PutNumber("refYaw", refYaw);

	//fps //not needed besides check for jitter
	double r_current = -((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_FOOT) * MINUTE_CONVERSION / 60;
	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_FOOT) * MINUTE_CONVERSION / 60;

	//SmartDashboard::PutNumber("Actual left", l_current);

	double r_dis = -((double) canTalonRight1->GetSelectedSensorPosition(0) //empirically determined ticks per foot
	/ TICKS_PER_FOOT);
	double l_dis = ((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

 SmartDashboard::PutNumber("actualLeftDis", l_dis);
// SmartDashboard::PutNumber("actualRightDis", r_dis);
 SmartDashboard::PutNumber("actualLeftVel", l_current);
// SmartDashboard::PutNumber("actualRightVel", r_current);

	double y_dis = -1.0 * ahrs->GetYaw() * (double) (PI / 180); //current theta (yaw) value

//	SmartDashboard::PutNumber("Target Heading", refYaw);
//	SmartDashboard::PutNumber("Actual Heading", y_dis);

	l_error_dis_au = refLeft - l_dis;
	r_error_dis_au = refRight - r_dis;

	y_error_dis_au = refYaw - y_dis; //positive - 0

	//SmartDashboard::PutNumber("Heading error", y_error_dis_au);

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
	D_YAW_DIS = K_D_YAW_DIS * (y_error_dis_au - yaw_last_error);

	// SmartDashboard::PutNumber("P", P_YAW_DIS);
	// SmartDashboard::PutNumber("I", I_YAW_DIS);
	// SmartDashboard::PutNumber("D", D_YAW_DIS);

	double total_right = P_RIGHT_DIS + I_RIGHT_DIS + D_RIGHT_DIS;
	double total_left = P_LEFT_DIS + I_LEFT_DIS + D_LEFT_DIS;

	double total_yaw = P_YAW_DIS + I_YAW_DIS + D_YAW_DIS;

	//SmartDashboard::PutNumber("TOTAL", total_yaw);

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

	if (set_refs) { //only set to true if set_profile is set to true in the DriveContrller, by actually filling the profile, for everything except doNothing auton
	Controller(0.0, 0.0, 0.0, targetYawRate, k_p_right_vel_au, k_p_left_vel_au,
			0.0, k_p_yaw_au, k_d_yaw_au, k_d_left_vel_au, k_d_right_vel_au, 0.0, //sends all 0.0 gains
			target_rpm_left, target_rpm_right, 0.0);
	}

	l_last_error = l_error_dis_au;
	r_last_error = r_error_dis_au;
	kick_last_error = k_error_dis_au;
	yaw_last_error = y_error_dis_au;

}

void DriveControllerMother::Controller(double ref_kick,
		double ref_right, //first parameter refs are for teleop
		double ref_left, double ref_yaw, double k_p_right, double k_p_left,
		double k_p_kick, double k_p_yaw, double k_d_yaw, double k_d_right,
		double k_d_left, double k_d_kick, double target_vel_left,
		double target_vel_right, double target_vel_kick) { //last parameter targets are for auton

	double yaw_rate_current = -1.0 * (double) ahrs->GetRate()
			* (double) ((PI) / 180.0); //left should be positive

	 SmartDashboard::PutNumber("YAW POS", ahrs->GetYaw());
	// SmartDashboard::PutNumber("LEFT ENC VEL", GetLeftVel());
	// SmartDashboard::PutNumber("RIGHT ENC VEL", GetRightVel());

	double target_yaw_rate = ref_yaw;

	ref_left = ref_left - (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //left should be positive
	ref_right = ref_right + (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //ff

	double yaw_error = target_yaw_rate - yaw_rate_current;
//
//	if(yaw_rate_current == 0.0) {
//		k_p_yaw = 0.0;
//		k_d_yaw = 0.0;
//	}

	if (std::abs(yaw_error) < .3) { //TODO: maybe get rid of this
		yaw_error = 0.0;
	}

	d_yaw_dis = yaw_error - yaw_last_error;

	double yaw_output = ((k_p_yaw * yaw_error) + (k_d_yaw * d_yaw_dis)); //pd for auton, p for teleop //fb

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

	feed_forward_r = k_f_right_vel * ref_right; //teleop only, controlled
	feed_forward_l = k_f_left_vel * ref_left;
	feed_forward_k = K_F_KICK_VEL * ref_kick;

	//conversion to RPM from native unit
	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double r_current = -((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
//	double kick_current = ((double) canTalonKicker->GetSelectedSensorVelocity(0) //will timeout, taking too much time
//			 (double) TICKS_PER_ROT) * MINUTE_CONVERSION; //going right is positive

//	SmartDashboard::PutNumber("actual left vel", l_current);

//	if ((std::abs(l_current) <= 0.5 && canTalonLeft1->GetOutputCurrent() > 4.0) //encoders not working
//			|| (std::abs(r_current) <= 0.5
//					&& canTalonRight1->GetOutputCurrent() > 4.0)) {
//		SmartDashboard::PutString("Drive Motor Encoders", "Not working");
//		k_p_yaw = 0.0;
//		k_d_yaw = 0.0;
//		feed_forward_l = 0.0;
//		feed_forward_r = 0.0;
//		k_p_left = 0.0;
//		k_p_right = 0.0;
//		k_d_left = 0.0;
//		k_d_right = 0.0;
//	} else {
//		SmartDashboard::PutString("Drive Motor Encoders", "Not working");
//	}

SmartDashboard::PutNumber("l_current", l_current);
SmartDashboard::PutNumber("r_current", r_current);

SmartDashboard::PutNumber("ref_left", ref_left);
SmartDashboard::PutNumber("ref_right", ref_right);

SmartDashboard::PutNumber("l_error_vel_t", l_error_vel_t);
SmartDashboard::PutNumber("r_error_vel_t", r_error_vel_t);

	l_error_vel_t = ref_left - l_current;
	r_error_vel_t = ref_right - r_current;
	//kick_error_vel = ref_kick - kick_current;

	d_left_vel = (l_error_vel_t - l_last_error_vel);
	d_right_vel = (r_error_vel_t - r_last_error_vel);
	d_kick_vel = (kick_error_vel - kick_last_error_vel);

	P_LEFT_VEL = k_p_left * l_error_vel_t;
	P_RIGHT_VEL = k_p_right * r_error_vel_t;
	P_KICK_VEL = k_p_kick * kick_error_vel;

	D_LEFT_VEL = k_d_left * d_left_vel;
	D_RIGHT_VEL = k_d_right * d_right_vel;
	D_KICK_VEL = k_d_kick * d_kick_vel;

	SmartDashboard::PutNumber("D Right Vel", D_RIGHT_VEL);
	SmartDashboard::PutNumber("P Right Vel", P_RIGHT_VEL);

	if (frc::RobotState::IsAutonomous()) { //only want the feedforward based off the motion profile during autonomous. The root generated ones (in the if() statement) //should already be 0 during auton because we send 0 as refs
		feed_forward_r = 0;	// will be close to 0  (low error between profile points) for the most part but will get quite aggressive when an error builds,
		feed_forward_l = 0;			//the PD controller should handle it itself
		feed_forward_k = 0;
	}

	double total_right = D_RIGHT_VEL + P_RIGHT_VEL + feed_forward_r
			+ (Kv * target_vel_right * FF_SCALE); //Kv only in auton, straight from motion profile
	double total_left = D_LEFT_VEL + P_LEFT_VEL + feed_forward_l
			+ (Kv * target_vel_left * FF_SCALE);
//	double total_kick = D_KICK_VEL + P_KICK_VEL + feed_forward_k
//			+ (Kv_KICK * target_vel_kick);

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

	SmartDashboard::PutNumber("% OUT LEFT", total_left);
	SmartDashboard::PutNumber("% OUT RIGHT", -total_right);

	canTalonLeft1->Set(ControlMode::PercentOutput, total_left);
	canTalonRight1->Set(ControlMode::PercentOutput, -total_right); //negative for Koba and for new drive train
	//canTalonRearRight->Set(ControlMode::PercentOutput,  total_right); //these are slaves
	//canTalonRearLeft->Set(ControlMode::PercentOutput, -total_left);
	///canTalonKicker->Set(ControlMode::PercentOutput, -total_kick); //since there is no kicker, was causing timeout and spike on thread

	yaw_last_error = yaw_error;
	l_last_error_vel = l_error_vel_t;
	r_last_error_vel = r_error_vel_t;
	kick_last_error_vel = kick_error_vel;
	l_last_current = l_current;

	SmartDashboard::PutNumber("Yaw Error", yaw_error);

}

void DriveControllerMother::ZeroAll(bool stop_motors) {

	if (stop_motors) {
		StopAll();
	}

	ZeroI();
	ZeroEncs();
	ZeroYaw();

	// while (std::abs(-1.0 * ahrs->GetYaw() * (double) (PI / 180)) > 0.1) {
	// 	ZeroYaw();
	// }

	zeroing_counter++;

}

//will stop all driven motors in the drive controller
void DriveControllerMother::StopAll() {

	canTalonLeft1->Set(ControlMode::PercentOutput, 0.0); //0.0 because of the double, 0 should still work but better safe than sorry
	canTalonRight1->Set(ControlMode::PercentOutput, 0.0);
	//canTalonRearRight->Set(ControlMode::PercentOutput, 0.0); //slaved
	//canTalonRearLeft->Set(ControlMode::PercentOutput, 0.0);
	//canTalonKicker->Set(ControlMode::PercentOutput, 0.0);

}

//sets the position of all the drive encoders to 0
void DriveControllerMother::ZeroEncs() { //acc to 8

	canTalonRight1->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft1->SetSelectedSensorPosition(0, 0, 0);
//	canTalonRight2->SetSelectedSensorPosition(0, 0, 0);
//	canTalonLeft2->SetSelectedSensorPosition(0, 0, 0);
//	canTalonRight3->SetSelectedSensorPosition(0, 0, 0);

//	canTalonLeft3->SetSelectedSensorPosition(0, 0, 0);
//	canTalonRight4->SetSelectedSensorPosition(0, 0, 0);
//	canTalonLeft4->SetSelectedSensorPosition(0, 0, 0);
	//canTalonKicker->SetSelectedSensorPosition(0, 0, 0);

}

void DriveControllerMother::ZeroYaw() {

	ahrs->ZeroYaw();

}

double DriveControllerMother::GetLeftVel() {

	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return l_current;
}

double DriveControllerMother::GetRightVel() {

	double r_current = ((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return r_current;
}

//Zeros the accumulating I
void DriveControllerMother::ZeroI() {

	i_right = 0; //20,
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
	set_refs = true;
	row_index = 0;
	zeroing_counter = 0;

}

void DriveControllerMother::SetRows(
		std::vector<std::vector<double>> two_rows_profile) {

	auton_rows = two_rows_profile;
	set_profile = true; //cannot re-enable to restart profile
	set_refs = true;
	//row_index = 0;

}

void DriveControllerMother::SetMaxRpm(double rpm) {

	max_y_rpm = rpm;

}

double DriveControllerMother::GetMaxRpm() {

	return max_y_rpm;

}

void DriveControllerMother::StopProfile(bool stop_profile) {

	continue_profile = !stop_profile;

}

double DriveControllerMother::GetLeftPosition() {

	double l_dis = ((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return l_dis;

}

double DriveControllerMother::GetRightPosition() {

	double r_dis = -((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return r_dis;

}

bool DriveControllerMother::IsLastIndex() {

	return is_last_index;

}

int DriveControllerMother::GetDriveIndex() {

	return row_index;

}

void DriveControllerMother::SetZeroingIndex(std::vector<int> zero_index) {

	zeroing_index = zero_index;

}

std::vector<std::vector<double> > DriveControllerMother::GetAutonProfile() {

	return auton_profile;

}

//Increments through target points of the motion profile
void DriveControllerMother::RunAutonDrive() {

	double left_enc = canTalonLeft1->GetSelectedSensorPosition(0);
	double yaw_pos = ahrs->GetYaw();

	// SmartDashboard::PutNumber("enc.", left_enc);
	// SmartDashboard::PutNumber("yaw zeroed", yaw_pos);

	for (int i = 0; i < auton_profile[0].size(); i++) { //looks through each row and then fills drive_ref with the column here, refills each interval with next set of refs
		drive_ref.at(i) = auton_profile.at(row_index).at(i); //from SetRef()
	}

	if (zeroing_index.size() > 0) {
		next_zero_index = zeroing_index.at(zero_counter);
	}

	if (row_index == next_zero_index) {
		StopAll(); //maybe add counts after stop and before zero
		if (zero_wait_counter < 10) {
			ZeroAll(true); //ZEROING NO LONGER INCLUDES ZEROING THE YAW. ENCODERS STILL NEED TO BE ZEROED
			zero_wait_counter++;
//			if(std::abs(left_enc) < 0.1) {
//				zero_wait_counter = 50; //hack to just break out of the counter 'loop'
//			}
		} else {
			if (zero_counter < (zeroing_index.size() - 1)) {
				zero_counter++;
			}
			zero_wait_counter = 0; //for the next time we need to zero
			row_index++; //break out of this if
		}
	} else {
		//AutonDrive(); //send each row to auton drive before getting the next row
		if (continue_profile && row_index < auton_profile.size()) {
			AutonDrive();
			row_index++;
		} else {
			StopAll();
		}
	}

//	SmartDashboard::PutNumber("ROW INDEX", row_index);
}

void DriveControllerMother::RunTeleopDrive(Joystick *JoyThrottle,
		Joystick *JoyWheel, bool is_heading) {

	if (is_heading) {
		RotationController(JoyWheel);
	} else {
		TeleopWCDrive(JoyThrottle, JoyWheel);
	}

}
