/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <TeleopStateMachine.h>

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_GROUND_STATE = 2;
const int GET_CUBE_STATION_STATE = 3;
const int POST_INTAKE_STATE = 4;
const int PLACE_SCALE_STATE = 5;
const int PLACE_SWITCH_STATE = 6;
int state = INIT_STATE;

bool state_intake_wheel = false;
bool state_intake_arm = false;
bool state_elevator = false;

Elevator *elevator;
Intake *intake;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_) {

	elevator = elevator_;
	intake = intake_;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in, //look this over
		bool intake_spin_out, bool intake_spin_stop, bool get_cube_ground,
		bool get_cube_station, bool post_intake, bool raise_to_switch, bool raise_to_scale,
		bool intake_arm_up, bool intake_arm_down, bool elevator_up,
		bool elevator_down) {

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

	if(!intake->EncodersRunning()) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->STOP_ARM_STATE_H;
	} else if (intake_arm_up) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->UP_STATE_H;
	} else if (intake_arm_down) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->DOWN_STATE_H;
	} else {
		state_intake_arm = true;
	}

	if(!elevator->EncodersRunning()) {
		state_elevator = false;
		elevator->elevator_state = elevator->STOP_STATE_H;
	} else if (elevator_up) {
		state_elevator = false;
		elevator->elevator_state = elevator->UP_STATE_H;
	} else if (elevator_down) {
		state_elevator = false;
		elevator->elevator_state = elevator->DOWN_STATE_H;
	} else {
		state_elevator = true;
	}

	if (wait_for_button) {
		state = WAIT_FOR_BUTTON_STATE;
	}

	switch (state) {

	case INIT_STATE:
		SmartDashboard::PutString("STATE", "INIT");
		elevator->elevator_state = elevator->DOWN_STATE_H;
		intake->intake_arm_state = intake->UP_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		state = WAIT_FOR_BUTTON_STATE;
		break;

	case WAIT_FOR_BUTTON_STATE:
		SmartDashboard::PutString("STATE", "WAIT FOR BUTTON");

		if (get_cube_ground) { //these buttons?
			state = GET_CUBE_GROUND_STATE;
		} else if (get_cube_station) {
			state = GET_CUBE_STATION_STATE;
		} else if (raise_to_scale) {
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		}

		break;

	case GET_CUBE_GROUND_STATE:
		SmartDashboard::PutString("STATE", "GET CUBE GROUND");
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_H;
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
		break;

	case GET_CUBE_STATION_STATE:
		SmartDashboard::PutString("STATE", "GET CUBE STATION");
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_H;
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
		break;

	case POST_INTAKE_STATE:
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		if(raise_to_scale) {
			state = PLACE_SCALE_STATE;
		}
		else if(raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		}
		break;

	case PLACE_SCALE_STATE:
		SmartDashboard::PutString("STATE", "SCALE");
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_elevator) {
			elevator->elevator_state = elevator->UP_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
		}
		break;

	case PLACE_SWITCH_STATE:
		SmartDashboard::PutString("STATE", "SWITCH");
		if (state_elevator) {
			elevator->elevator_state = elevator->DOWN_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->IN_STATE_H;
		}
		break;

	}

}

void TeleopStateMachine::Initialize() {

	state = WAIT_FOR_BUTTON_STATE;

}
