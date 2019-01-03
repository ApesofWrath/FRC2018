#include "ElevatorTask.h"
#include "../constants.h"

#define PI 3.14159265

const int INIT_STATE_E = 0;
const int DOWN_STATE_E = 1;
const int MID_STATE_E = 2;
const int UP_STATE_E = 3;
const int STOP_STATE_E = 4;
const int HPS_STATE_E = 5;

// how many times the robots loops through code with high current draw before stopping elevator; used in StallSafety()
const int STALLS_TILL_STOP = 3;

const int ELEVATOR_SLEEP_TIME = 0;
const double ELEVATOR_WAIT_TIME = 0.01; //sec

int last_elevator_state = 0; //init state

double offset = 0.0;
double ff = 0.0; //feedforward
double u_e = 0.0; //this is the output in volts to the motor
double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

double position_offset_e = 0.0;

double current_pos_e = 0.0;
double current_vel_e = 0.0;

double MAX_THEORETICAL_VELOCITY_E, Kv_e; //gear ratio; //radius of the pulley in meters

std::vector<std::vector<double>> X_e, error_e;

ElevatorMotionProfiler *elevator_profiler;

Timer *elevatorTimer = new Timer();

bool is_at_bottom_e = false;
bool is_at_top = false;
bool first_at_bottom_e = false;
bool last_at_bottom_e = false;
bool encs_zeroed_e = false;

int init_counter = 0;
int encoder_counter_e = 0;

double elevator_voltage = 0.0;
double el_pos = 0;

ElevatorTask::Elevator(ElevatorMotionProfiler *elevator_profiler_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_,
	double down_pos_, double mid_pos_, double hps_pos_, double up_pos_, double G_e_, double ff_percent_e_, double PULLEY_DIAMETER_,
	double friction_loss_, int TOP_HALL_, int BOT_HALL_, std::string elev_type_, int TALON_ID_1_, int TALON_ID_2_) { //carr

		X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
		{ 0.0 } };
		error_e = { { 0.0 }, { 0.0 } };

		K_down_e = K_down_e_;
		K_up_e = K_up_e_;

		G_e = G_e_;

		ff_percent_e = ff_percent_e_;

		PULLEY_DIAMETER = PULLEY_DIAMETER_;

		friction_loss = friction_loss_;

		down_pos = down_pos_;
		mid_pos = mid_pos_;
		hps_pos = hps_pos_;
		up_pos = up_pos_;

		TOP_HALL = TOP_HALL_;
		BOT_HALL = BOT_HALL_;

		elev_type = elev_type_;

		TALON_ID_1 = TALON_ID_1_;
		SetupTalon1();

		TALON_ID_2 = TALON_ID_2_;
		SetupTalon2();

		MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) / 60.0
		* PULLEY_DIAMETER * PI * friction_loss; //m/s //1.87 //1.32

		Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

		hallEffectTop = new DigitalInput(TOP_HALL);
		hallEffectBottom = new DigitalInput(BOT_HALL);

		elevator_profiler = elevator_profiler_;

}

void ElevatorTask::SetupTalon2() {
	if (TALON_ID_2 >= 0) {
		talonElevator2 = new TalonSRX(TALON_ID_2); //0
		talonElevator2->Set(ControlMode::Follower, TALON_ID_1); //re-slaved
		talonElevator2->EnableCurrentLimit(false);
		talonElevator2->ConfigContinuousCurrentLimit(40, 0);
		talonElevator2->ConfigPeakCurrentLimit(80, 0);
		talonElevator2->ConfigPeakCurrentDuration(100, 0);
	}
}

void ElevatorTask::SetupTalon1() {
	talonElevator1 = new TalonSRX(TALON_ID_1);
	talonElevator1->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
	talonElevator1->EnableCurrentLimit(false);
	talonElevator1->ConfigContinuousCurrentLimit(40, 0);
	talonElevator1->ConfigPeakCurrentLimit(80, 0);
	talonElevator1->ConfigPeakCurrentDuration(100, 0);

	talonElevator1->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, 0);
	talonElevator1->ConfigVelocityMeasurementWindow(5, 0); //5 samples for every talon return
	talonElevator1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	talonElevator1->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 0); //for getselectedsensor //getselectedsensor defaults to 10ms anyway. don't use getsensorcollection because that defaults to 160ms
}

void ElevatorTask::InitializeElevator() {
	if (!is_elevator_init) { //don't see hall effect
	SetVoltage(0.0);
}

void ElevatorTask::TaskRun() {

}

void ElevatorTask::TaskStart() {

}

void ElevatorTask::TaskStop() {
	talonElevator1->Set(ControlMode::PercentOutput, 0.0);
}




// TODO: rename hall effect sensor inpputs (is_at_top_e), implement Task interface functions (controller, stop, etc), Move revamp, copy in and revise constructor
void ElevatorTask::SetVoltage(double voltage) {
     //set the global variable to be the parameter of the function to preserve the logic of the function call [ex: double output = 5; SetVoltage(); is less obvious than SetVoltage(5)]
     elevator_voltage = voltage;

     is_at_bottom_e = IsAtBottomElevator();
	is_at_top = IsAtTopElevator();
     el_pos = GetElevatorPosition();

	// Safeties
	elev_safety = "NONE"; // Default value
	StallSafety();
     UpperSoftLimit();
     LowerSoftLimit();
	TopHallEffectSafety();
	BottomHallEffectSafety();
	ArmSafety();

     SmartDashboard::PutString(elev_type + "SAFETY", elev_safety);

	// Output Voltage
     ZeroElevator();
     CapVoltage();
	ScaleOutput();
	InvertOutput();
	OutputToTalon();

     SmartDashboard::PutNumber("ELEV VOLT", elevator_voltage);
}


void ElevatorTask::ManualElevator(Joystick *joyOpElev) {
	PrintElevatorInfo();
	ManualElevatorOutput();
}

void ElevatorTask::ManualElevatorOutput() {
	double output = (joyOpElev->GetY()) * 0.5 * 12.0; //multiply by voltage because setvoltageelevator takes voltage
	SetVoltage(output);
}

void ElevatorTask::PrintElevatorInfo() {
	SmartDashboard::PutString(elev_type, elev_state[elevator_state]);

	SmartDashboard::PutNumber("ELEV CUR 1", talonElevator1->GetOutputCurrent());
	SmartDashboard::PutNumber("ELEV CUR 2", talonElevator2->GetOutputCurrent());

	SmartDashboard::PutNumber("ElEV ENC", talonElevator1->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber("ELEV HEIGHT", GetElevatorPosition());

	SmartDashboard::PutBoolean("TOP HALL", IsAtTopElevator());
	SmartDashboard::PutBoolean("BOT HALL", IsAtBottomElevator());
}

// TODO: once clearance obtained, make goal_pos global and move beginning to UpdateMoveCoordinates()
	// move errors to UpdateMoveError()
void ElevatorTask::Move() {

	if (NotStopOrInitState()) {

		std::vector<std::vector<double> > ref_elevator = elevator_profiler->GetNextRefElevator();

		current_pos_e = GetElevatorPosition();//GetElevatorPosition(); //TAKE THIS BACK OUT
		current_vel_e = GetElevatorVelocity();//GetElevatorVelocity();

		///	SmartDashboard::PutNumber("Actual Vel", current_vel_e);
		//	SmartDashboard::PutNumber("Actual Pos", current_pos_e);

		double goal_pos = ref_elevator[0][0];
		goal_vel_e = ref_elevator[1][0];

		//	SmartDashboard::PutNumber("Goal Vel", goal_vel_e);
		//	SmartDashboard::PutNumber("Goal Pos", goal_pos);

		error_e[0][0] = goal_pos - current_pos_e;
		error_e[1][0] = goal_vel_e - current_vel_e;

		v_bat_e = 12.0;

		// Need to go down
		if (elevator_profiler->GetFinalGoalElevator()
		< elevator_profiler->GetInitPosElevator()) { //can't be the next goal in case we get ahead of the profiler
			UpdateToMoveDirection(1.0, 0.55, K_down_e);
		} else {
			UpdateToMoveDirection(1.0, ff_percent_e, K_up_e);
		}

		UpdateVoltage();
		SetVoltage(u_e);
	}
}

void ElevatorTask::UpdateVoltage() {
	u_e = (K_e[0][0] * error_e[0][0]) + (K_e[0][1] * error_e[1][0]);
	u_e += ff + offset;
}

void ElevatorTask::UpdateMoveCoordinates() {

}

void ElevatorTask::UpdateMoveError() {

}

void ElevatorTask::UpdateToMoveDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_) {
	K_e = K_e_;
	offset = offset_; //dampen
	ff = (Kv_e * goal_vel_e * v_bat_e) * percent;
}

bool ElevatorTask::NotStopOrInitState() {
	return elevator_state != STOP_STATE_E_H && elevator_state != INIT_STATE_E_H;
}

void ElevatorTask::ElevatorStateMachine() {
	PrintElevatorInfo();

	switch (elevator_state) {

		case INIT_STATE_E:
		InitState();
		break;

		case DOWN_STATE_E:
		CheckElevatorGoal(DOWN_STATE_E, down_pos);
		break;

		case MID_STATE_E:
		CheckElevatorGoal(MID_STATE_E, mid_pos);
		break;

		case UP_STATE_E:
		CheckElevatorGoal(UP_STATE_E, up_pos);
		break;

		case STOP_STATE_E:
		TaskStop();
		last_elevator_state = STOP_STATE_E;
		break;

		case HPS_STATE_E:
		CheckElevatorGoal(HPS_STATE_E, hps_pos);
		break;

	}

}

void ElevatorTask::InitState() {
	if (is_elevator_init) {
		elevator_state = DOWN_STATE_E;
	} else {
		InitializeElevator();
	}
	last_elevator_state = INIT_STATE_E;
}

void ElevatorTask::CheckElevatorGoal(int current_state, double goal_pos) {
	if (last_elevator_state != current_state) {
		elevator_profiler->SetFinalGoalElevator(goal_pos);
		elevator_profiler->SetInitPosElevator(GetElevatorPosition());
	}
	last_elevator_state = current_state;
}


void ElevatorTask::ScaleOutput() {
     elevator_voltage /= 12.0;
}

void ElevatorTask::InvertOutput() {
     elevator_voltage *= -1.0; //reverse at END
}

void ElevatorTask::OutputToTalon() {
     talonElevator1->Set(ControlMode::PercentOutput, elevator_voltage);
}

void ElevatorTask::CapVoltage() {
     if (elevator_voltage > MAX_VOLTAGE_E) {
		elevator_voltage = MAX_VOLTAGE_E;
	} else if (elevator_voltage < MIN_VOLTAGE_E) {
		elevator_voltage = MIN_VOLTAGE_E;
	}
}

void ElevatorTask::ZeroElevator() {
     if (!is_elevator_init) { //changed this to just zero on start up (as it always be at the bottom at the start of the match)
		if (ZeroEncs()) { //successfully zeroed one time
			is_elevator_init = true;
		}
	}
}

void ElevatorTask::StallSafety() {
	if (std::abs(GetElevatorVelocity()) <= 0.05
	&& std::abs(elevator_voltage) > 3.0) { //this has to be here at the end
		encoder_counter_e++;
	} else {
		encoder_counter_e = 0;
	}
	if (encoder_counter_e > STALLS_TILL_STOP) { //bypass the initial high voltage to accelerate from 0
		elevator_voltage = 0.0;
		elev_safety = "stall";
	}
}

void ElevatorTask::ArmSafety() {
	if (keep_elevator_up) {
		elevator_voltage = 1.0;
		elev_safety = "arm safety";
	}
}

void ElevatorTask::BottomHallEffectSafety() {
	if (is_at_bottom_e && elevator_voltage > 0.2) { //elevator_voltage is actually reverse
		elev_safety = "bot hall eff";
		elevator_voltage = 0.0;
	}
}

void ElevatorTask::TopHallEffectSafety() {
	if (is_at_top && elevator_voltage < -0.2) { //elevator_voltage is actually reverse
		elev_safety = "top hall eff";
		elevator_voltage = 0.0;
	}
}

void ElevatorTask::LowerSoftLimit() {
     if (el_pos <= (-0.05) && elevator_voltage < 0.0) {  //lower soft limit
		elevator_voltage = 0.0;
		elev_safety = "lower soft";
	}
}

void  ElevatorTask::UpperSoftLimit() {
     //TODO: may need to change order
	if (el_pos >= (0.92) && elevator_voltage > 0.0) { //upper soft limit //TODO: separate carr, mds top height
		elevator_voltage = 0.0;
		elev_safety = "upper soft";
	}
}




bool ElevatorTask::IsAtBottomElevator() {
	if (!hallEffectBottom->Get()) {
		return true;
	} else {
		return false;
	}
}

bool ElevatorTask::IsAtTopElevator() {
	if (!hallEffectTop->Get()) {
		return true;
	} else {
		return false;
	}
}

double ElevatorTask::GetElevatorPosition() {

	//divide by the native ticks per rotation then multiply by the circumference of the pulley
	//radians

	int elev_pos =
	talonElevator1->GetSelectedSensorPosition(0);

	double elevator_pos = ((elev_pos - position_offset_e) / TICKS_PER_ROT_E) //position offset to zero
	* (PI * PULLEY_DIAMETER) * -1.0;

	return elevator_pos;

}

double ElevatorTask::GetElevatorVelocity() {

	//native units are ticks per 100 ms so we multiply the whole thing by 10 to get it into per second. Then divide by ticks per rotation to get into
	//RPS then muliply by circumference for m/s
	double elevator_vel =
	(talonElevator1->GetSelectedSensorVelocity(0)
	/ (TICKS_PER_ROT_E)) * (PULLEY_DIAMETER * PI) * (10.0)
	* -1.0;
	return elevator_vel;

}

// something that goes in the ElevatorState?
double ElevatorTask::GetVoltageElevator() { //not voltage sent to the motor. the voltage the controller sends to SetVoltage()
	return u_e;
}

void ElevatorTask::SetZeroOffset() {

	position_offset_e =
	talonElevator1->GetSelectedSensorPosition(0);
}

bool ElevatorTask::ZeroEncs() {

	if (zeroing_counter_e < 1) {
		//Great Robotic Actuation and Controls Execution ()
		SetZeroOffset();
		zeroing_counter_e++;
		return true;
	} else {
		return false;
	}

}
