/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//put in thread
//arm back down in get cube ground
#include <TeleopStateMachine.h>

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_GROUND_STATE = 2;
const int GET_CUBE_STATION_STATE = 3;
const int POST_INTAKE_STATE = 4;
const int PLACE_SCALE_STATE = 5;
const int PLACE_SWITCH_STATE = 6;
int state = INIT_STATE;

int last_state = 0;

bool state_intake_wheel = false; //set to true to override the states set in the state machine
bool state_intake_arm = false;
bool state_elevator = false;

Elevator *elevator;
Intake *intake;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_) {

	elevator = elevator_;
	intake = intake_;

}

void TeleopStateMachine::StateMachine(bool wait_for_button,
		bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_stop, bool get_cube_ground,
		bool get_cube_station, bool post_intake, bool raise_to_switch,
		bool raise_to_scale, bool intake_arm_up, bool intake_arm_mid,
		bool intake_arm_down, bool elevator_up, bool elevator_mid,
		bool elevator_down) {

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
	} else if (intake_arm_mid) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->MID_STATE_H;
	} else if (intake_arm_down) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->DOWN_STATE_H;
	} else {
		state_intake_arm = true;
	}

	state_intake_arm = true;

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
		elevator->elevator_state = elevator->INIT_STATE_E_H;
		intake->intake_arm_state = intake->INIT_STATE_H;
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
//			if (state_intake_wheel) {
//				intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H; //in order to not have to change intake wheel state immediately
//			}
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) {
//			if (state_intake_wheel) {
//				intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H; //in order to not have to change intake wheel state immediately
//			}
			state = PLACE_SWITCH_STATE;
		}
		last_state = WAIT_FOR_BUTTON_STATE;
		break;

	case GET_CUBE_GROUND_STATE:
		SmartDashboard::PutString("STATE", "GET CUBE GROUND");
		std::cout << "state intake " << state_intake_arm << std::endl;
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
			//SmartDashboard::PutBoolean("state elevator", state_elevator);
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
			//SmartDashboard::PutBoolean("state wheel", state_intake_wheel);
		}
		if (state_intake_arm) {
			intake->intake_arm_state = 3;
			std::cout << "intake arm state: " << intake->intake_arm_state
					<< std::endl;
		}
		if (intake->HaveCube() || post_intake) {
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_GROUND_STATE;
		break;

	case GET_CUBE_STATION_STATE: //human player station
		SmartDashboard::PutString("STATE", "GET CUBE STATION");
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (intake->HaveCube() || post_intake) {
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_STATION_STATE;
		break;

	case POST_INTAKE_STATE: //have cube, waiting to place cube
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
		}
		if (state_intake_arm) {
			//std::cout << "uP /state " << std::endl;
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		} else if (last_state == PLACE_SCALE_STATE
				|| last_state == PLACE_SWITCH_STATE) {
			state = WAIT_FOR_BUTTON_STATE;
		}
		last_state = POST_INTAKE_STATE;
		//can always go back to wait for button state
		break;

	case PLACE_SCALE_STATE:
		SmartDashboard::PutString("STATE", "SCALE");
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_elevator) {
			elevator->elevator_state = elevator->UP_STATE_E_H;
		}
		if (elevator->GetElevatorPosition() >= 0.55 && state_intake_wheel) { //start shooting at 0.6
			intake->intake_wheel_state = intake->OUT_STATE_H;
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
	}

}

void TeleopStateMachine::Initialize() {

	state = INIT_STATE;

}
