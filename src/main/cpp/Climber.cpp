#include "Climber.h"

//Will need to be controlled. On way up, will be like a single-stage climber

#define PI 3.14159265

const double DOWN_SPEED = 0.15;
const double UP_SPEED = 0.325;

const int UNITS_PER_ROT = 4096;

const double MAX_OUTPUT = 1.0;
const double MIN_OUTPUT = -1.0;

const int INIT_STATE_C = 0;
const int DOWN_STATE_C = 1;
const int UP_STATE_C = 2;

int last_climber_state = 0;

double offset = 0.0;
double ff = 0.0; //feedforward
double u_c = 0.0; //this is the input in volts to the motor
double v_bat_c = 0.0; //this will be the voltage of the battery at every loop

double position_offset_c = 0.0;

//ClimberMotionProfiler *climber_profiler; //TODO: Make a ClimberMotionProfiler class so that this works
ClimberMotionProfiler *climber_profiler; //not permanent. the above is correct

std::vector<std::vector<double>> X_c, error_c;

double MAX_THEORETICAL_VELOCITY_C, Kv_c; //gear ratio; //radius of the pulley in meters

Timer *climberTimer = new Timer();

std::vector<std::vector<double>> K_down_c, K_up_c, K_c;
double down_pos, mid_pos, hps_pos, up_pos, G_e, ff_percent_e, PULLEY_DIAMETER, friction_loss;


Climber::Climber() {

	talonClimber1 = new TalonSRX(0);
	talonClimber1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);


	talonClimber2 = new TalonSRX(1);
	talonClimber2->Set(ControlMode::Follower, 0);

	talonClimber1->EnableCurrentLimit(false);
	talonClimber2->EnableCurrentLimit(false);
	talonClimber1->ConfigContinuousCurrentLimit(40, 0);
	talonClimber2->ConfigContinuousCurrentLimit(40, 0);
	talonClimber1->ConfigPeakCurrentLimit(80, 0);
	talonClimber2->ConfigPeakCurrentLimit(80, 0);
	talonClimber1->ConfigPeakCurrentDuration(100, 0);
	talonClimber2->ConfigPeakCurrentDuration(100, 0);

}

void Climber::Move() {

	if (climber_state != INIT_STATE_C_H) {

		std::vector<std::vector<double> > ref_climber =
				climber_profiler->GetNextRefClimber();

		double current_pos_c = GetClimberPosition();
		double current_vel_c = GetClimberVelocity();

	///	SmartDashboard::PutNumber("Actual Vel", current_vel_c);
	//	SmartDashboard::PutNumber("Actual Pos", current_pos_c);

		double goal_pos = ref_climber[0][0];
		goal_vel_c = ref_climber[1][0];

	//	SmartDashboard::PutNumber("Goal Vel", goal_vel_c);
	//	SmartDashboard::PutNumber("Goal Pos", goal_pos);

		error_c[0][0] = goal_pos - current_pos_c;
		error_c[1][0] = goal_vel_c - current_vel_c;

		v_bat_c = 12.0;

		if (climber_profiler->GetFinalGoalClimber()
				< climber_profiler->GetInitPosClimber()) { //can't be the next goal in case we get ahead of the profiler
			K_c = K_down_c;
			ff = (Kv_c * goal_vel_c * v_bat_c) * 0.55;
			offset = 1.0; //dampen
		} else {
			offset = 1.0;
			K_c = K_up_c;

			ff = (Kv_c * goal_vel_c * v_bat_c) * ff_percent_c;

		}

		u_c = (K_c[0][0] * error_c[0][0]) + (K_c[0][1] * error_c[1][0]);

		u_c += ff + offset;
		SetVoltageClimber(u_c);

	}

}

void Climber::SetVoltageClimber(double climber_voltage) {

	is_at_bottom_c = IsAtBottomClimber();
	is_at_top = IsAtTopClimber();

	double cl_pos = GetClimberPosition();

	if (std::abs(GetClimberVelocity()) <= 0.05
	&& std::abs(climber_voltage) > 3.0) { //this has to be here at the end
		encoder_counter_c++;
	} else {
		encoder_counter_c = 0;
	}
	if (encoder_counter_c > 3) { //bypass the initial high voltage to accelerate from 0
		voltage_safety_c = true;
	} else {
		voltage_safety_c = false;
	}

	//TODO: may need to change order
	if (cl_pos >= (0.92) && climber_voltage > 0.0) { //upper soft limit
		climber_voltage = 0.0;
		climb_safety = "upper soft";
	} else if (cl_pos <= (-0.05) && climber_voltage < 0.0) {  //lower soft limit
		climber_voltage = 0.0;
		climb_safety = "lower soft";
	} else if (is_at_top && climber_voltage < -0.2) { //climber_voltage is actually reverse
		climb_safety = "top hall eff";
		climber_voltage = 0.0;
	} else if (is_at_bottom_c && climber_voltage > 0.2) { //climber_voltage is actually reverse
		climb_safety = "bot hall eff";
		climber_voltage = 0.0;
	} else if (keep_climber_up) {
		climber_voltage = 1.0;
		climb_safety = "arm safety";
	} else if (voltage_safety_c) {
		climber_voltage = 0.0;
		climb_safety = "stall";
	}	else {
		climb_safety = "NONE";
	}

	if (is_carr_) {
		SmartDashboard::PutString("CARR SAFETY", climb_safety);
	} else {
		SmartDashboard::PutString("MDS SAFETY", climb_safety);
	}

	if (!is_climber_init) { //changed this to just zero on start up (as it always be at the bottom at the start of the match)
		if (ZeroEncs()) { //successfully zeroed one time
			is_climber_init = true;
		}
	}

	if (climber_voltage > MAX_VOLTAGE_C) {
		climber_voltage = MAX_VOLTAGE_C;
	} else if (climber_voltage < MIN_VOLTAGE_C) {
		climber_voltage = MIN_VOLTAGE_C;
	}

	climber_voltage /= 12.0;

	climber_voltage *= -1.0; //reverse at END

	//2 is slaved to 1
	talonClimber1->Set(ControlMode::PercentOutput, climber_voltage);

}

double Climber::GetClimberPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int climb_pos =
			talonClimber1->GetSelectedSensorPosition(0);

	double climber_pos = ((climb_pos - position_offset_c) / UNITS_PER_ROT) //position offset to zero
	* (PI * PULLEY_DIAMETER) * -1.0;

	return climber_pos;

}


void Climber::ClimberStateMachine() {

//	SmartDashboard::PutNumber("C1", talonClimber1->GetOutputCurrent());
//	SmartDashboard::PutNumber("C2", talonClimber2->GetOutputCurrent());

	switch (climber_state) {

	case INIT_STATE_C:

		SmartDashboard::PutString("CLIMBER.", "INIT");

		climber_state = DOWN_STATE_C;
		last_climber_state = INIT_STATE_C;
		break;

	case DOWN_STATE_C:

		SmartDashboard::PutString("CLIMBER.", "DOWN");

		if (last_climber_state != DOWN_STATE_C) { //first time in state
			climber_profiler->SetFinalGoalClimber(down_pos);
			climber_profiler->SetInitPosClimber(GetClimberPosition());
		}
		last_climber_state = DOWN_STATE_C;
		break;

	case UP_STATE_C:

		SmartDashboard::PutString("CLIMBER.", "UP");

		if (last_climber_state != UP_STATE_C) { //first time in state
			climber_profiler->SetFinalGoalClimber(up_pos);
			climber_profiler->SetInitPosClimber(GetClimberPosition());
		}
		last_climber_state = UP_STATE_C;
		break;

	}

}

void Climber::StartClimberThread() {

	Climber *climber_ = this;

	ClimberThread = std::thread(&Climber::ClimberWrapper, climber_);
	ClimberThread.detach();

}

void Climber::ClimberWrapper(Climber *cl) {

	climberTimer->Start();

	while (true) {

		climberTimer->Reset();

		if (frc::RobotState::IsEnabled()) {

			if (cl->climber_state != INIT_STATE_C) {
				//	cl->Move(cl->ClimberGetNextRef());
			}

		}

		double time_c = 0.01 - climberTimer->Get(); //change

		time_c *= 1000;
		if (time_c < 0) {
			time_c = 0;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds((int) time_c));

	}

}

void Climber::EndClimberThread() {

	//climberTimer->Stop();
	ClimberThread.~thread();

}
