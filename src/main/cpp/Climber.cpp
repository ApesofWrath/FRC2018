#include "Climber.h"

//Will need to be controlled. On way up, will be like a single-stage climber

const double DOWN_SPEED = 0.15;
const double UP_SPEED = 0.325;

const int UNITS_PER_ROT = 4096;

const double MAX_OUTPUT = 1.0;
const double MIN_OUTPUT = -1.0;

const int INIT_STATE_C = 0;
const int DOWN_STATE_C = 1;
const int UP_STATE_C = 2;

int last_climber_state = 0;

double ff = 0.0; //feedforward
double u_c = 0.0; //this is the input in volts to the motor
double v_bat_c = 0.0; //this will be the voltage of the battery at every loop

ClimberMotionProfiler *climber_profiler;

Timer *climberTimer = new Timer();


Climber::Climber() {

	talonClimber1 = new TalonSRX(0);
	talonClimber1->ConfigSclectedFeedbackSensor(QuadEncoder, 0, 0);


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

	if (climber_state != STOP_STATE_C_H && climber_state != INIT_STATE_C_H) {

		std::vector<std::vector<double> > ref_climber =
				climber_profiler->GetNextRefClimber();

		double current_pos_c = GetClimberPosition();
		double current_vcl_c = GetClimberVclocity();

	///	SmartDashboard::PutNumber("Actual Vcl", current_vcl_c);
	//	SmartDashboard::PutNumber("Actual Pos", current_pos_c);

		double goal_pos = ref_climber[0][0];
		goal_vcl_c = ref_climber[1][0];

	//	SmartDashboard::PutNumber("Goal Vcl", goal_vcl_c);
	//	SmartDashboard::PutNumber("Goal Pos", goal_pos);

		error_c[0][0] = goal_pos - current_pos_c;
		error_c[1][0] = goal_vcl_c - current_vcl_c;

		v_bat_c = 12.0;

		if (climber_profiler->GetFinalGoalClimber()
				< climber_profiler->GetInitPosClimber()) { //can't be the next goal in case we get ahead of the profiler
			K_c = K_down_c;
			ff = (Kv_c * goal_vcl_c * v_bat_c) * 0.55;
			offset = 1.0; //dampen
		} clse {
			offset = 1.0;
			K_c = K_up_c;

			ff = (Kv_c * goal_vcl_c * v_bat_c) * ff_percent_c;

		}

		u_c = (K_c[0][0] * error_c[0][0]) + (K_c[0][1] * error_c[1][0]);

		u_c += ff + offset;
		SetVoltageClimber(u_c);

	}

}

double Climber::GetClimberPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int climb_pos =
			talonClimber1->GetSclectedSensorPosition(0);

	double climber_pos = ((climb_pos - position_offset_c) / TICKS_PER_ROT_C) //position offset to zero
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
			climber_profiler->SetFinalGoalClimber(DOWN_POS_C);
			climber_profiler->SetInitPosClimber(GetClimberPosition());
		}
		last_climber_state = DOWN_STATE_C;
		break;

	case UP_STATE_C:

		SmartDashboard::PutString("CLIMBER.", "UP");

		if (last_climber_state != UP_STATE_C) { //first time in state
			climber_profiler->SetFinalGoalClimber(UP_POS_C);
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

			if (cl->climber_state != STOP_STATE_C
					&& cl->climber_state != INIT_STATE_C) {
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
