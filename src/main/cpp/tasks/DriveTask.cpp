#include "DriveTask.h"

////////////ROBOT-SPECIFIC////////////

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
const double K_P_YAW_VEL_LOW = 30.0;
const double K_D_YAW_VEL_LOW = 0.0;

const double K_P_RIGHT_VEL_HIGH = 0.001;
const double K_P_LEFT_VEL_HIGH = 0.001;
const double K_D_RIGHT_VEL_HIGH = 0.00;
const double K_D_LEFT_VEL_HIGH = 0.0;
const double K_P_YAW_VEL_HIGH = 10.0;
const double K_D_YAW_VEL_HIGH = 0.000;

const double K_P_YAW_HEADING_POS_HD = 0.0;
const double K_D_VISION_POS_HD = 0.0;
const double K_P_YAW_HEADING_POS_WC = 0.01;
const double K_D_VISION_POS_WC = 0.0;

const double K_P_KICK_VEL = 0.00365;
const double K_D_KICK_VEL = 0.0;
const double K_F_KICK_VEL = 1.0 / 400.0;

//		Auton
const double K_P_YAW_AU_HD = 0.0;
const double K_D_YAW_AU_HD = 0.0;
const double K_P_YAW_AU_WC = 0.0;
const double K_D_YAW_AU_WC = 0.0;

const double K_P_RIGHT_DIS = 0.15;
const double K_P_LEFT_DIS = 0.15;
const double K_P_KICKER_DIS = 0.280;

const double K_I_RIGHT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_I_KICKER_DIS = 0.0;

const double K_D_RIGHT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;
const double K_D_KICKER_DIS = 0.0;

double K_P_YAW_DIS = 0.5;
double K_I_YAW_DIS = 0.0;
double K_D_YAW_DIS = 0.0;

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

double k_p_right_vel, k_p_left_vel, k_p_yaw_vel, k_d_right_vel, k_d_left_vel, //gains vary depending on gear
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

void DriveTask::DriveTask(int l1, int l2, int l3, int l4,
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

	ahrs = new AHRS(SerialPort::kUSB);

#ifndef CORNELIUS
	solenoid = new DoubleSolenoid(3, 1, 0);
#else
	solenoid = new DoubleSolenoid(0, 0, 1);
#endif
	canTalonKicker = new TalonSRX(-1);

}

void DriveTask::TaskStart() {
  SetCurrentLimits();
  ZeroAll(true);
  ShiftUp();
}

void DriveTask::TaskRun() {

}

void DriveTask::TaskStop() {

}

void DriveTask::ZeroAll(bool stop_motors) {
  StopAll();
  ZeroEncs();
  ZeroI();
  ZeroYaw();
}

void DriveTask::StopAll() {
  SetOutputs(0.0, 0.0);
}

void DriveTask::ZeroEncs() {
  canTalonRight1->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft1->SetSelectedSensorPosition(0, 0, 0);
}

void DriveTask::ZeroYaw() {
  ahrs->ZeroYaw();
}

void DriveTask::ZeroI() {
  i_right = 0;
	i_left = 0;
	i_yaw = 0;

	y_error_dis_au = 0;
	l_error_dis_au = 0;
	r_error_dis_au = 0;
}

void DriveTask::SetCurrentLimits() {

  canTalonLeft1->EnableCurrentLimit(true);
	canTalonLeft2->EnableCurrentLimit(true);
	canTalonLeft3->EnableCurrentLimit(true);
	canTalonLeft4->EnableCurrentLimit(true);
	canTalonRight1->EnableCurrentLimit(true);
	canTalonRight2->EnableCurrentLimit(true);
	canTalonRight3->EnableCurrentLimit(true);
	canTalonRight4->EnableCurrentLimit(true);

	canTalonLeft1->ConfigPeakCurrentLimit(40, 0);
	canTalonLeft2->ConfigPeakCurrentLimit(40, 0);
	canTalonLeft3->ConfigPeakCurrentLimit(40, 0);
	canTalonLeft4->ConfigPeakCurrentLimit(40, 0);
	canTalonRight1->ConfigPeakCurrentLimit(40, 0);
	canTalonRight2->ConfigPeakCurrentLimit(40, 0);
	canTalonRight3->ConfigPeakCurrentLimit(40, 0);
	canTalonRight4->ConfigPeakCurrentLimit(40, 0);

	canTalonLeft1->ConfigContinuousCurrentLimit(30, 0);
	canTalonLeft2->ConfigContinuousCurrentLimit(30, 0);
	canTalonLeft3->ConfigContinuousCurrentLimit(30, 0);
	canTalonLeft4->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight1->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight2->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight3->ConfigContinuousCurrentLimit(30, 0);
	canTalonRight4->ConfigContinuousCurrentLimit(30, 0);

	canTalonLeft1->ConfigPeakCurrentDuration(10, 0);
	canTalonLeft2->ConfigPeakCurrentDuration(10, 0);
	canTalonLeft3->ConfigPeakCurrentDuration(10, 0);
	canTalonLeft4->ConfigPeakCurrentDuration(10, 0);
	canTalonRight1->ConfigPeakCurrentDuration(10, 0);
	canTalonRight2->ConfigPeakCurrentDuration(10, 0);
	canTalonRight3->ConfigPeakCurrentDuration(10, 0);
	canTalonRight4->ConfigPeakCurrentDuration(10, 0);

	canTalonLeft1->ConfigOpenloopRamp(0.15, 0);
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
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

}

void DriveTask::SetOutputs(double left, double right) {
  canTalonLeft1->Set(ControlMode::PercentOutput, left);
	canTalonRight1->Set(ControlMode::PercentOutput, -right);
}
