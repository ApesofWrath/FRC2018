/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <TeleopStateMachine.h>
#include <WPILib.h>

using namespace std::chrono;

//Teleop
const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_GROUND_STATE = 2;
const int GET_CUBE_STATION_STATE = 3;
const int POST_INTAKE_STATE = 4;
const int PLACE_SCALE_STATE = 5;
const int PLACE_SWITCH_STATE = 6;
const int PLACE_SCALE_BACKWARDS_STATE = 7;
int state = INIT_STATE;

int last_state = 0;
int last_state_a = 0;

bool state_intake_wheel = false; //set to true to override the states set in the state machine
bool state_intake_arm = false;
bool state_elevator = false;

//Auton
const int INIT_STATE_A = 0;
const int WAIT_FOR_BUTTON_STATE_A = 1;
const int GET_CUBE_GROUND_STATE_A = 2;
const int GET_CUBE_STATION_STATE_A = 3;
const int POST_INTAKE_STATE_A = 4;
const int PLACE_SCALE_STATE_A = 5;
const int PLACE_SWITCH_STATE_A = 6;
const int PLACE_SCALE_BACKWARDS_STATE_A = 7;
int state_a = INIT_STATE_A;

bool is_intake_low_enough;

Elevator *elevator;
Intake *intake;
DriveController *driveController;

Timer *teleopTimer = new Timer();

std::thread StateMachineThread;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_,
		DriveController *drive_controller) {

	elevator = elevator_;
	intake = intake_;
	driveController = drive_controller;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_slow, bool intake_spin_stop,
		bool get_cube_ground, bool get_cube_station, bool post_intake,
		bool raise_to_switch, bool raise_to_scale, bool intake_arm_up,
		bool intake_arm_mid, bool intake_arm_down, bool elevator_up,
		bool elevator_mid, bool elevator_down, bool raise_to_scale_backwards) {

	if (wait_for_button) { //can always return to wait for button state
		state = WAIT_FOR_BUTTON_STATE;
	}

	//intake wheels
	if (intake_spin_out) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->OUT_STATE_H;
	} else if (intake_spin_in) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->IN_STATE_H;
	} else if (intake_spin_stop) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
	} else {
		state_intake_wheel = true;
	}

	//intake arm
	if (intake_arm_up) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->UP_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
	} else if (intake_arm_mid) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->MID_STATE_H;
	} else if (intake_arm_down) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->DOWN_STATE_H;
	} else {
		state_intake_arm = true;
	}
//TODO: add manual for intake all the way back

	//elevator
	if (elevator_up) {
		state_elevator = false;
		elevator->elevator_state = elevator->UP_STATE_E_H;
	} else if (elevator_mid) {
		state_elevator = false;
		elevator->elevator_state = elevator->MID_STATE_E_H;
	} else if (elevator_down) {
		state_elevator = false;
		elevator->elevator_state = elevator->DOWN_STATE_E_H;
	} else {
		state_elevator = true;
	}

	switch (state) {

	case INIT_STATE:

		SmartDashboard::PutString("STATE", "INIT");
		elevator->elevator_state = elevator->DOWN_STATE_E_H;
		intake->intake_arm_state = intake->UP_STATE_H;
		//elevator->elevator_state = elevator->INIT_STATE_E_H;
		//intake->intake_arm_state = intake->INIT_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		state = WAIT_FOR_BUTTON_STATE;
		last_state = INIT_STATE;
		break;

	case WAIT_FOR_BUTTON_STATE:

		SmartDashboard::PutString("STATE", "WAIT FOR BUTTON");

		if (get_cube_ground) { //can go to all states below wfb state
			state = GET_CUBE_GROUND_STATE;
		} else if (get_cube_station) {
			state = GET_CUBE_STATION_STATE;
		} else if (post_intake) {
			state = POST_INTAKE_STATE;
		} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		}
		last_state = WAIT_FOR_BUTTON_STATE;
		break;

	case GET_CUBE_GROUND_STATE:

		SmartDashboard::PutString("STATE", "GET CUBE GROUND");

		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = 3;
		}
		if (intake->HaveCube() || post_intake) { //there is no post intake button
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_GROUND_STATE;
		break;

	case GET_CUBE_STATION_STATE: //human player station

		SmartDashboard::PutString("STATE", "GET CUBE STATION");

		if (state_elevator) {
			elevator->elevator_state = elevator->SWITCH_STATE_E_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->DOWN_STATE_H;
		}
		if (intake->HaveCube() || post_intake) {
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_STATION_STATE;
		break;

	case POST_INTAKE_STATE: //have cube, waiting to place cube

		SmartDashboard::PutString("STATE", "POST INTAKE");

		is_intake_low_enough = (intake->GetAngularPosition()
				< (intake->UP_ANGLE + 0.05)); //use same check for the entirety of the state

		if (state_elevator && is_intake_low_enough) { //last_state == PLACE_SCALE_BACKWARDS_STATE - don't need this in the check since it will only be true the first time
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
			//intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) { //TODO: add raise to switch backwards
			state = PLACE_SWITCH_STATE;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		} else if (last_state == PLACE_SCALE_STATE //will keep checking if arm is low enough to start lowering the elevator
		|| last_state == PLACE_SWITCH_STATE || is_intake_low_enough) { //little bit of ahack but the check wont run if it only goes through this state once

			state = WAIT_FOR_BUTTON_STATE;
		}
		last_state = POST_INTAKE_STATE;
		//can always go back to wait for button state
		break;

	case PLACE_SCALE_STATE:

		SmartDashboard::PutString("STATE", "SCALE FORWARDS");

		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_elevator) {
			elevator->elevator_state = elevator->UP_STATE_E_H;
		}
		if (elevator->GetElevatorPosition() >= 0.55 && state_intake_wheel) { //start shooting at 0.6
			if (!intake_spin_slow) {
				intake->intake_wheel_state = intake->OUT_STATE_H;
			} else {
				intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H;
			}
			if (intake->ReleasedCube()) {
				state = POST_INTAKE_STATE;
			}
		}
		last_state = PLACE_SCALE_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SWITCH_STATE:

		SmartDashboard::PutString("STATE", "SWITCH");

		if (state_elevator) {
			elevator->elevator_state = elevator->MID_STATE_E_H;
		}
		if (state_intake_arm) { //elevator->GetElevatorPosition() >= 0.1 &&
			intake->intake_arm_state = intake->MID_STATE_H;
		}
		if (std::abs(intake->GetAngularPosition() - intake->MID_ANGLE) <= 0.2
				&& state_intake_wheel) { //start shooting when close enough
			intake->intake_wheel_state = intake->SLOW_STATE_H;
			if (intake->ReleasedCube()) {
				state = POST_INTAKE_STATE;
			}
		}
		last_state = PLACE_SWITCH_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SCALE_BACKWARDS_STATE:

		SmartDashboard::PutString("STATE", "SCALE BACKWARDS");

		if (state_intake_arm && elevator->GetElevatorPosition() >= .85) { //move to the flippy angle when safe
			intake->intake_arm_state = intake->SWITCH_BACK_SHOT_STATE_H;
		} else if (state_intake_arm && elevator->GetElevatorPosition() < .85) { //move to normal up angle if not safe to go all the way to flippy angle
			intake->intake_arm_state = intake->UP_STATE_H;
		}

		if (state_elevator) {
			elevator->elevator_state = elevator->UP_STATE_E_H;
		}
		if (elevator->GetElevatorPosition() >= 0.85
				&& intake->GetAngularPosition() > 1.98 && state_intake_wheel) { //shoot if the height of the elevator and the angle of the arm is good enough
			intake->intake_wheel_state = intake->OUT_STATE_H;
			if (intake->ReleasedCube()) {
				state = POST_INTAKE_STATE;
			}
		}

		last_state = PLACE_SCALE_BACKWARDS_STATE;

		break;
	}

}

//TODO: MAKE SURE THESE CHANGES WORK FOR SWITCH AND SINGLE SCALE TOO
void TeleopStateMachine::AutonStateMachine(bool wait_for_button,
		bool intake_spin_in, bool intake_spin_out, bool intake_spin_slow,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool raise_to_scale,
		bool intake_arm_up, bool intake_arm_mid, bool intake_arm_down,
		bool elevator_up, bool elevator_mid, bool elevator_down,
		bool raise_to_scale_backwards) {

	switch (state_a) {

	case INIT_STATE_A:
		//this always has to run only once
		SmartDashboard::PutString("STATE", "INIT");
		elevator->elevator_state = elevator->INIT_STATE_E_H;
		intake->intake_arm_state = intake->INIT_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		//	if (elevator->elevator_state == elevator->DOWN_STATE_E_H && intake->intake_arm_state == intake->UP_STATE_H) { //this check is very important: nothing should happen until everything has initialized
		state_a = WAIT_FOR_BUTTON_STATE_A;
		//	}
		last_state_a = INIT_STATE_A;
		break;

	case WAIT_FOR_BUTTON_STATE_A: //will look at position

		SmartDashboard::PutString("STATE", "WAIT FOR BUTTON."); //was fighting between this state and place switch

		if (elevator->is_elevator_init && intake->is_init_intake) {
			if (last_state_a == POST_INTAKE_STATE_A) {
				state_a == GET_CUBE_GROUND_STATE_A;
			} else if (get_cube_ground) { //can go to all states below wfb state
				state_a = GET_CUBE_GROUND_STATE_A;
			} else if (get_cube_station) {
				state_a = GET_CUBE_STATION_STATE_A;
			} else if (post_intake) {
				state_a = POST_INTAKE_STATE_A;
			} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
				state_a = PLACE_SCALE_STATE_A;
			} else if (raise_to_switch) {
				std::cout << "ALTERNATE NO NO" << std::endl;
				state_a = PLACE_SWITCH_STATE_A;
			} else if (raise_to_scale_backwards) {
				state_a = PLACE_SCALE_BACKWARDS_STATE_A;
			}
		}
		last_state_a = WAIT_FOR_BUTTON_STATE_A;
		break;

	case GET_CUBE_GROUND_STATE_A:

		if (intake->GetAngularPosition() <= 0.3) {
			driveController->StopProfile(false);
		}

		SmartDashboard::PutString("STATE", "GET CUBE GROUND");
		elevator->elevator_state = elevator->DOWN_STATE_E_H;
		intake->intake_wheel_state = intake->IN_STATE_H;
		intake->intake_arm_state = 3;
		if (intake->HaveCube()) {
			state_a = POST_INTAKE_STATE_A;
		}
		last_state_a = GET_CUBE_GROUND_STATE_A;
		break;

	case GET_CUBE_STATION_STATE_A: //human player station

		SmartDashboard::PutString("STATE", "GET CUBE STATION");
		elevator->elevator_state = elevator->SWITCH_STATE_E_H;
		intake->intake_wheel_state = intake->IN_STATE_H;
		intake->intake_arm_state = intake->DOWN_STATE_H;
		if (intake->HaveCube()) {
			state_a = POST_INTAKE_STATE_A;
		}
		last_state_a = GET_CUBE_STATION_STATE_A;
		break;

	case POST_INTAKE_STATE_A: //have cube, waiting to place cube

		driveController->StopProfile(true);

		SmartDashboard::PutString("STATE", "POST INTAKE");
//make bool for same check
		if (intake->GetAngularPosition() < (intake->UP_ANGLE + 0.05)) {
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
			//if (last_state_a == PLACE_SCALE_BACKWARDS_STATE_A) { THIS IS THE SAME BUG WE TALKED ABOUT,ONLY IS TRUE ONCE
			//}
		}

//		if (last_state_a != PLACE_SWITCH_STATE_A) { //TODO: breaks 2-cube auto
//			intake->intake_arm_state = intake->UP_STATE_H;
//		} else {
//			state_a = GET_CUBE_GROUND_STATE_A;
//		}

		intake->intake_arm_state = intake->UP_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;

		if (raise_to_switch) {
			state_a = PLACE_SWITCH_STATE_A;
		} else if (raise_to_scale) { //came from placing
			state_a = PLACE_SCALE_STATE_A;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		} else if (last_state == PLACE_SCALE_STATE_A //will keep checking if arm is low enough to start lowering the elevator
		|| last_state == PLACE_SWITCH_STATE_A
				|| (last_state == POST_INTAKE_STATE_A
						&& (intake->GetAngularPosition()
								< (intake->UP_ANGLE + 0.05)))) { //not going back to wfb //not making this check
			state_a = WAIT_FOR_BUTTON_STATE_A;
		}
		last_state_a = POST_INTAKE_STATE_A;
		//can always go back to wait for button state
		break;

	case PLACE_SCALE_STATE_A:

		SmartDashboard::PutString("STATE", "SCALE");
		intake->intake_arm_state = intake->UP_STATE_H;
		elevator->elevator_state = elevator->UP_STATE_E_H;
		if (elevator->GetElevatorPosition() >= 0.55) { //start shooting at 0.6
			intake->intake_wheel_state = intake->OUT_STATE_H;
			if (intake->ReleasedCube()) {
				state_a = POST_INTAKE_STATE_A;
			}
		}
		last_state_a = PLACE_SCALE_STATE_A;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SWITCH_STATE_A:

		SmartDashboard::PutString("STATE", "SWITCH.");
		elevator->elevator_state = elevator->MID_STATE_E_H;
		intake->intake_arm_state = intake->MID_STATE_H;
		if (std::abs(intake->GetAngularPosition() - intake->MID_ANGLE) <= 0.2) { //start shooting when high enough
			intake->intake_wheel_state = intake->SLOW_STATE_H;
			if (intake->ReleasedCube()) {
				state_a = POST_INTAKE_STATE_A;
			}
		}
		last_state_a = PLACE_SWITCH_STATE_A;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SCALE_BACKWARDS_STATE_A:

		SmartDashboard::PutString("STATE", "SCALE BACKWARDS");

		driveController->StopProfile(true);

		if (elevator->GetElevatorPosition() >= .85) { //move to the flippy angle when safe
			intake->intake_arm_state = intake->SWITCH_BACK_SHOT_STATE_H;
		} else if (elevator->GetElevatorPosition() < .85) { //move to normal up angle if not safe to go all the way to flippy angle
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		elevator->elevator_state = elevator->UP_STATE_E_H;
		if (elevator->GetElevatorPosition() >= 0.85
				&& intake->GetAngularPosition() > 1.98) { //shoot if the height of the elevator and the angle of the arm is good enough
			intake->intake_wheel_state = intake->OUT_STATE_H;
			if (intake->ReleasedCube()) {
				state_a = POST_INTAKE_STATE_A;
			}
		}

		last_state_a = PLACE_SCALE_BACKWARDS_STATE_A;

		break;
	}

}

void TeleopStateMachine::Initialize() {

	elevator->zeroing_counter_e = 0;
	intake->zeroing_counter_i = 0;

	intake->is_init_intake = false;
	elevator->is_elevator_init = false;

	state = INIT_STATE; //won't initialize arm and elevator
	state_a = INIT_STATE_A;

}

void TeleopStateMachine::StartStateMachineThread(bool *wait_for_button,
		bool *intake_spin_in, bool *intake_spin_out, bool *intake_spin_slow,
		bool *intake_spin_stop, bool *get_cube_ground, bool *get_cube_station,
		bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
		bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down,
		bool *elevator_up, bool *elevator_mid, bool *elevator_down,
		bool *raise_to_scale_backwards) {

	TeleopStateMachine *tsm = this;
	StateMachineThread = std::thread(&TeleopStateMachine::StateMachineWrapper,
			tsm, wait_for_button, intake_spin_in, intake_spin_out, intake_spin_slow,
			intake_spin_stop, get_cube_ground, get_cube_station, post_intake,
			raise_to_switch, raise_to_scale, intake_arm_up, intake_arm_mid,
			intake_arm_down, elevator_up, elevator_mid, elevator_down,
			raise_to_scale_backwards);
	StateMachineThread.detach();

}

void TeleopStateMachine::StateMachineWrapper(
		TeleopStateMachine *teleop_state_machine, bool *wait_for_button,
		bool *intake_spin_in, bool *intake_spin_out, bool *intake_spin_slow,
		bool *intake_spin_stop, bool *get_cube_ground, bool *get_cube_station,
		bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
		bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down,
		bool *elevator_up, bool *elevator_mid, bool *elevator_down,
		bool *raise_to_scale_backwards) {

	teleopTimer->Start();

	while (true) {

		teleopTimer->Reset();

		if (frc::RobotState::IsEnabled()
				&& frc::RobotState::IsOperatorControl()) {

			intake->IntakeArmStateMachine();
			intake->IntakeWheelStateMachine();
			elevator->ElevatorStateMachine();

			teleop_state_machine->StateMachine((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch,
					(bool) *raise_to_scale, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *elevator_up, (bool) *elevator_mid,
					(bool) *elevator_down, (bool) *raise_to_scale_backwards);

		}

		else if (frc::RobotState::IsEnabled()
				&& frc::RobotState::IsAutonomous()) {

			intake->IntakeArmStateMachine();
			intake->IntakeWheelStateMachine();
			elevator->ElevatorStateMachine();

			teleop_state_machine->AutonStateMachine((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch,
					(bool) *raise_to_scale, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *elevator_up, (bool) *elevator_mid,
					(bool) *elevator_down, (bool) *raise_to_scale_backwards);

		}

		double wait_time = 0.05 - teleopTimer->Get();

		wait_time *= 1000;
		if (wait_time < 0) {
			wait_time = 0;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds((int) wait_time));

	}

}

void TeleopStateMachine::EndStateMachineThread() {

	StateMachineThread.~thread(); //does not actually kill the thread

}
