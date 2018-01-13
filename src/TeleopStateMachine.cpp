/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <TeleopStateMachine.h>

const int WAIT_FOR_BUTTON_STATE = 0;
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
