/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <TeleopStateMachine.h>

const int WAIT_FOR_BUTTON_STATE = 0;
const int INTAKE_SPIN_IN_STATE = 1;
const int INTAKE_ARM_UP_STATE = 2;
const int ELEVATOR_UP_STATE = 3;
const int INTAKE_SPIN_OUT_STATE = 4;
int state = 0;

Elevator *elevator;
Intake *intake;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_) {

	elevator = elevator_;
	intake = intake_;

}

void TeleopStateMachine::StateMachine() {

	switch(state) {
		case WAIT_FOR_BUTTON_STATE:
			elevator->elevator_state_h = elevator->DOWN_STATE_H;
			intake->intake_state_h = intake->DOWN_STATE_H;
			intake->intake_state_h = intake->STOP_WHEEL_STATE_H;
			break;
		case INTAKE_SPIN_IN_STATE:
			intake->intake_state_h = intake->IN_STATE_H;
			break;
		case INTAKE_ARM_UP_STATE:
			intake->intake_state_h = intake->UP_STATE_H;
			break;
		case ELEVATOR_UP_STATE:
			elevator->elevator_state_h = elevator->UP_STATE_H;
			break;
		case INTAKE_SPIN_OUT_STATE:
			intake->intake_state_h = intake->OUT_STATE_H;
			break;

	}
}

void TeleopStateMachine::Initialize() {

	state = WAIT_FOR_BUTTON_STATE;

}

/*
 * START: Elevator and IntakeArm are up
 *
 * Elevator down, IntakeArm down
 * Drive to power cube
 * Intake in
 * Elevator up halfway
 * Intake in (again)
 * Elevator up allway
 * IntakeArm up
 * Drive to place power cube
 *
 *
 *
 *
 */
