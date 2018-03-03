/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>

#define CORNELIUS 0

#if CORNELIUS

#else

#endif

using namespace std::chrono;

const int INIT_STATE_A = 0;
const int WAIT_FOR_BUTTON_STATE_A = 1;
const int GET_CUBE_GROUND_STATE_A = 2;
const int GET_CUBE_STATION_STATE_A = 3;
const int POST_INTAKE_STATE_A = 4;
const int PLACE_SCALE_STATE_A = 5;
const int PLACE_SWITCH_STATE_A = 6;
int state_a = INIT_STATE_A;

int last_state_a = 0;

const int NUM_POINTS = 1500; //every 10ms
const int NUM_INDEX = 10; //change this

double refs[NUM_POINTS][NUM_INDEX];

std::thread AutonStateMachineThread;

Timer *autonTimer = new Timer();

DriveController *drive_controller;
Elevator *elevator_;
Intake *intake_;

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;

	StartAutonStateMachineThread(
			//starts all of the state machines
			&wait_for_button, &intake_spin_in, &intake_spin_out,
			&intake_spin_stop, &get_cube_ground, &get_cube_station,
			&post_intake, &raise_to_switch, &raise_to_scale, &intake_arm_up,
			&intake_arm_mid, &intake_arm_down, &elevator_up, &elevator_mid,
			&elevator_down); //will get through init state into wfb state

}

void Autonomous::AutonStateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_stop, bool get_cube_ground,
		bool get_cube_station, bool post_intake, bool raise_to_switch,
		bool raise_to_scale, bool intake_arm_up, bool intake_arm_mid,
		bool intake_arm_down, bool elevator_up, bool elevator_mid,
		bool elevator_down) {

	switch (state_a) {

	case INIT_STATE_A:

		SmartDashboard::PutString("STATE", "INIT");
		elevator_->elevator_state = elevator_->INIT_STATE_E_H;
		intake_->intake_arm_state = intake_->INIT_STATE_H;
		intake_->intake_wheel_state = intake_->STOP_WHEEL_STATE_H;
		state_a = WAIT_FOR_BUTTON_STATE_A;
		last_state_a = INIT_STATE_A;
		break;

	case WAIT_FOR_BUTTON_STATE_A: //will look at position

		SmartDashboard::PutString("STATE", "WAIT FOR BUTTON");

		if (get_cube_ground) { //can go to all states below wfb state
			state_a = GET_CUBE_GROUND_STATE_A;
		} else if (get_cube_station) {
			state_a = GET_CUBE_STATION_STATE_A;
		} else if (post_intake) {
			state_a = POST_INTAKE_STATE_A;
		} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
			state_a = PLACE_SCALE_STATE_A;
		} else if (raise_to_switch) {
			state_a = PLACE_SWITCH_STATE_A;
		}
		last_state_a = WAIT_FOR_BUTTON_STATE_A;
		break;

	case GET_CUBE_GROUND_STATE_A:

		SmartDashboard::PutString("STATE", "GET CUBE GROUND");

		//std::cout << "state intake " << state_intake_arm << std::endl;
		elevator_->elevator_state = elevator_->DOWN_STATE_E_H;
		intake_->intake_wheel_state = intake_->IN_STATE_H;
		intake_->intake_arm_state = 3;
		if (intake_->HaveCube()) {
			state_a = POST_INTAKE_STATE_A;
		}
		last_state_a = GET_CUBE_GROUND_STATE_A;
		break;

	case GET_CUBE_STATION_STATE_A: //human player station

		SmartDashboard::PutString("STATE", "GET CUBE STATION");
		elevator_->elevator_state = elevator_->SWITCH_STATE_E_H;
		intake_->intake_wheel_state = intake_->IN_STATE_H;
		intake_->intake_arm_state = intake_->DOWN_STATE_H;
		if (intake_->HaveCube()) {
			state_a = POST_INTAKE_STATE_A;
		}
		last_state_a = GET_CUBE_STATION_STATE_A;
		break;

	case POST_INTAKE_STATE_A: //have cube, waiting to place cube
		elevator_->elevator_state = elevator_->DOWN_STATE_E_H;
		intake_->intake_arm_state = intake_->UP_STATE_H;
		intake_->intake_wheel_state = intake_->STOP_WHEEL_STATE_H;
		if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
			state_a = PLACE_SCALE_STATE_A;
		} else if (raise_to_switch) {
			state_a = PLACE_SWITCH_STATE_A;
		} else if (last_state_a == PLACE_SCALE_STATE_A
				|| last_state_a == PLACE_SWITCH_STATE_A) { //came from placing
			state_a = WAIT_FOR_BUTTON_STATE_A;
		}
		last_state_a = POST_INTAKE_STATE_A;
		//can always go back to wait for button state
		break;

	case PLACE_SCALE_STATE_A:

		SmartDashboard::PutString("STATE", "SCALE");
		intake_->intake_arm_state = intake_->UP_STATE_H;
		elevator_->elevator_state = elevator_->UP_STATE_E_H;
		//if (elevator_->GetElevatorPosition() >= 0.55 && state_intake_wheel) { //start shooting at 0.6
		intake_->intake_wheel_state = intake_->OUT_STATE_H;
		if (intake_->ReleasedCube()) {
			state_a = POST_INTAKE_STATE_A;
		}
		//}
		last_state_a = PLACE_SCALE_STATE_A;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SWITCH_STATE_A:

		SmartDashboard::PutString("STATE", "SWITCH");
		elevator_->elevator_state = elevator_->MID_STATE_E_H;
		intake_->intake_arm_state = intake_->MID_STATE_H;
		if (std::abs(intake_->GetAngularPosition() - intake_->MID_ANGLE)
				<= 0.2) { //start shooting when high enough
			intake_->intake_wheel_state = intake_->SLOW_STATE_H;
			if (intake_->ReleasedCube()) {
				state_a = POST_INTAKE_STATE_A;
			}
		}
		last_state_a = PLACE_SWITCH_STATE_A;
		//stay in this state when spitting cube, then return to WFB
		break;
	}

}

void Autonomous::InitializeAuton() {

	elevator_->zeroing_counter_e = 0;
	intake_->zeroing_counter_i = 0;

	intake_->is_init_intake = false;
	elevator_->is_elevator_init = false;

	state_a = INIT_STATE_A;

}

void Autonomous::StartAutonStateMachineThread(bool *wait_for_button,
		bool *intake_spin_in, bool *intake_spin_out, bool *intake_spin_stop,
		bool *get_cube_ground, bool *get_cube_station, bool *post_intake,
		bool *raise_to_switch, bool *raise_to_scale, bool *intake_arm_up,
		bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up,
		bool *elevator_mid, bool *elevator_down) {

	Autonomous *aut = this;
	AutonStateMachineThread = std::thread(&Autonomous::AutonStateMachineWrapper,
			aut, wait_for_button, intake_spin_in, intake_spin_out,
			intake_spin_stop, get_cube_ground, get_cube_station, post_intake,
			raise_to_switch, raise_to_scale, intake_arm_up, intake_arm_mid,
			intake_arm_down, elevator_up, elevator_mid, elevator_down);
	AutonStateMachineThread.detach();

}

void Autonomous::FillProfile(
		std::vector<std::vector<double> > pathfinder_refs) {

	drive_controller->SetRefs(pathfinder_refs);

}

double Autonomous::GetLeftPos() {

	return drive_controller->GetLeftPosition();

}

double Autonomous::GetRightPos() {

	return drive_controller->GetRightPosition();

}


void Autonomous::AutonStateMachineWrapper(
		//may move to drive controller auton thread
		Autonomous *auton_state_machine, bool *wait_for_button,
		bool *intake_spin_in, bool *intake_spin_out, bool *intake_spin_stop,
		bool *get_cube_ground, bool *get_cube_station, bool *post_intake,
		bool *raise_to_switch, bool *raise_to_scale, bool *intake_arm_up,
		bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up,
		bool *elevator_mid, bool *elevator_down) {

	autonTimer->Start();

	while (true) {

		autonTimer->Reset();

		if (frc::RobotState::IsEnabled() && frc::RobotState::IsAutonomous()) { //this thread will still run in auton

			intake_->IntakeArmStateMachine();
			intake_->IntakeWheelStateMachine();
			elevator_->ElevatorStateMachine();

			auton_state_machine->AutonStateMachine((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_stop, (bool) *get_cube_ground,
					(bool) *get_cube_station, (bool) *post_intake,
					(bool) *raise_to_switch, (bool) *raise_to_scale,
					(bool) *intake_arm_up, (bool) *intake_arm_mid,
					(bool) *intake_arm_down, (bool) *elevator_up,
					(bool) *elevator_mid, (bool) *elevator_down);

		}

		double time = 0.05 - autonTimer->Get();

		time *= 1000;
		if (time < 0) {
			time = 0;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds((int) time));

	}

}

void Autonomous::EndAutonStateMachineThread() {

	AutonStateMachineThread.~thread();

}

