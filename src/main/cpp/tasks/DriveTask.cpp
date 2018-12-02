#include "DriveTask.h"
#include "../Constants.h"

int LF = 0, L2 = 0, L3 = 0, LR = 0, RF = 0, R2 = 0, R3 = 0, RR = 0, KICKER = 0;

double target_l, target_r, target_yaw_rate;
double throttle, wheel;
double max_y_rpm, actual_max_y_rpm, max_yaw_rate;
double k_p_right_vel, k_p_left_vel, k_p_yaw_vel, k_d_right_vel, k_d_left_vel, //gains vary depending on gear
		k_p_yaw_t, k_d_yaw_t, k_p_kick_vel, k_d_kick_vel, k_p_yaw_h_vel,
		k_p_yaw_au, k_d_yaw_au;
double k_f_left_vel, k_f_right_vel;
double Kv;

//controller()

double ref_right = 0;
double ref_kick = 0;
double ref_left = 0;
double ref_yaw = 0;
double k_p_right = 0;
double k_p_left = 0;
double k_p_kick = 0;
double k_p_yaw = 0;
double k_d_yaw = 0;
double k_d_right = 0;
double k_d_left = 0;
double k_d_kick = 0;
double target_vel_left = 0;
double target_vel_right = 0;
double target_vel_kick = 0;

double feed_forward_r, feed_forward_l, feed_forward_k;

double P_LEFT_VEL = 0;
double D_LEFT_VEL = 0;
double d_left_vel = 0;

double P_RIGHT_VEL = 0;
double D_RIGHT_VEL = 0;
double d_right_vel = 0;

double P_KICK_VEL = 0;
double D_KICK_VEL = 0;
double d_kick_vel = 0;

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

double D_YAW_DIS = 0.0;

//autondrive()

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

double d_right = 0;
double i_right = 0;

double d_yaw_dis = 0;

double d_left = 0;
double i_left = 0;

double d_kick = 0;
double i_kick = 0;

double i_yaw = 0;

double r_error_dis_au = 0;
double l_error_dis_au = 0;
double k_error_dis_au = 0;
double y_error_dis_au = 0; // yaw (theta) position error

double k_p_right_vel_au = 0.0;
double k_p_left_vel_au = 0.0;
double k_p_kick_vel_au = 0.0;
double k_d_left_vel_au = 0.0;
double k_d_right_vel_au = 0.0;
double k_d_kick_vel_au = 0.0;

//zeroing

int next_zero_index = 0;
int zero_counter = 0;
int zero_wait_counter = 0;

//other

double k_p_yaw_heading_pos, k_d_vision_pos;

bool is_last_index = false;

bool is_zero;

double init_heading = 0;
double total_heading = 0;

bool tank = false;
bool is_low_gear = true;

std::vector<std::vector<double> > auton_profile(1500, std::vector<double>(6)); //rows stacked on rows, all points // can't be in .h for some reason
std::vector<std::vector<double> > auton_rows(2, std::vector<double>(6));

DriveTask::DriveTask(int l1, int l2, int l3, int l4,
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

	driveState = new DriveState();

}

void DriveTask::TaskStart() {
  SetCurrentLimits();
  ZeroAll(true);
  ShiftUp();
}

void DriveTask::TaskRun() {

}

void DriveTask::TaskRunAuto() {

}

void DriveTask::TaskRunTeleop(Joystick *JoyThrottle,
		Joystick *JoyWheel) { //from TeleopWCDrive
	UpdateInputs(JoyThrottle, JoyWheel);
	SquareInputs();
	UpdateTargets();
	LimitTargets();
	Controller();
}

void DriveTask::TaskStop() {
	ZeroAll(true);
}

void DriveTask::Controller() {
	double yaw_rate_current = -1.0 * ahrs->GetRate()
			* ((PI) / 180.0); //left should be positive
	driveState->UpdateYaw(yaw_rate_current);

}

void DriveTask::UpdateInputs(Joystick *JoyThrottle,
		Joystick *JoyWheel) {
	throttle = JoyThrottle->GetY();
	wheel = JoyWheel->GetX();
}

void DriveTask::SquareInputs() {
	if (throttle > 0.0) {
		throttle = throttle * throttle * -1.0;
	} else {
		throttle = throttle * throttle * 1.0;
	}

	if (wheel > 0.0) {
		wheel = wheel * wheel * 1.0;
	} else {
		wheel = wheel * wheel * -1.0;
	}
}

void DriveTask::UpdateTargets() { //took out deadzone
	ref_right = throttle * max_y_rpm;
	ref_left = ref_right;
	ref_yaw = wheel * max_yaw_rate;
}

void DriveTask::LimitTargets() {
	if (ref_right > max_y_rpm) {
		ref_right = max_y_rpm;
	} else if (ref_right < -max_y_rpm) {
		ref_right = -max_y_rpm;
	}

	if (ref_left > max_y_rpm) {
		ref_left = max_y_rpm;
	} else if (ref_left < -max_y_rpm) {
		ref_left = -max_y_rpm;
	}
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
