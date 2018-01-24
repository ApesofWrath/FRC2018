/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <TeleopStateMachine.h>

const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_STATE = 2;
const int SCALE_STATE = 3;
const int SWITCH_STATE = 4;
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

void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in, bool intake_spin_out, bool intake_spin_stop, bool get_cube, bool raise_to_switch, bool raise_to_scale, bool intake_arm_up, bool intake_arm_down, bool elevator_up, bool elevator_down) {

	switch(state) {

		if(intake_spin_out) {
			state_intake_wheel = false;
			intake->intake_wheel_state_h = intake->OUT_STATE_H;
		}
		else if(intake_spin_in) {
			state_intake_wheel = false;
			intake->intake_wheel_state_h = intake->IN_STATE_H;
		}
		else if(intake_spin_stop) {
			state_intake_wheel = false;
			intake->intake_wheel_state_h = intake->STOP_WHEEL_STATE_H;
		}
		else {
			state_intake_wheel = true;
		}

		if(intake_arm_up) {
			state_intake_arm = false;
			intake->intake_arm_state_h = intake->UP_STATE_H;
		}
		else if(intake_arm_down) {
			state_intake_arm = false;
			intake->intake_arm_state_h = intake->DOWN_STATE_H;
		}
		else {
			state_intake_arm = true;
		}

		if(elevator_up) {
			state_elevator = false;
			elevator->elevator_state_h = elevator->UP_STATE_H;
		}
		else if(elevator_down) {
			state_elevator = false;
			elevator->elevator_state_h = elevator->DOWN_STATE_H;
		}
		else {
			state_elevator = true;
		}

		if(wait_for_button) {
			state = WAIT_FOR_BUTTON_STATE;
		}

		case INIT_STATE:

			elevator->elevator_state_h = elevator->DOWN_STATE_H;
			intake->intake_arm_state_h = intake->UP_STATE_H;
			intake->intake_wheel_state_h = intake->STOP_WHEEL_STATE_H;
			state = WAIT_FOR_BUTTON_STATE;
			break;

		case WAIT_FOR_BUTTON_STATE:
			elevator->elevator_state_h = elevator->DOWN_STATE_H;
			intake->intake_arm_state_h = intake->UP_STATE_H;
			if(state_intake_wheel) {
			intake->intake_wheel_state_h = intake->STOP_WHEEL_STATE_H;
			}
			if(get_cube) {
				state = GET_CUBE_STATE;
			}
			else if(raise_to_scale) {
				state = SCALE_STATE;
			}
			else if(raise_to_switch) {
				state = SWITCH_STATE;
			}
			break;

		case GET_CUBE_STATE:
			elevator->elevator_state_h = elevator->DOWN_STATE_H;
			if(state_intake_wheel) {
			intake->intake_wheel_state_h = intake->IN_STATE_H;
			}
			intake->intake_arm_state_h = intake->DOWN_STATE_H;
			if(intake->HaveCube()) {
				state = WAIT_FOR_BUTTON_STATE;
			}
			break;
		case SCALE_STATE:
			if(state_intake_arm) {
			intake->intake_arm_state_h = intake->UP_STATE_H;
			}
			elevator->elevator_state_h = elevator->UP_STATE_H;
			break;
		case SWITCH_STATE:
			elevator->elevator_state_h = elevator->DOWN_STATE_H;
			if(state_intake_arm) {
			intake->intake_arm_state_h = intake->UP_STATE_H;
			}
			break;

	}
}

void TeleopStateMachine::Initialize() {

	state = WAIT_FOR_BUTTON_STATE;

}
