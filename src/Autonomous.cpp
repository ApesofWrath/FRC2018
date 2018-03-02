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

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;

}

void Autonomous::AutonStateMachine() {

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

		//if (get_cube_ground) { //can go to all states below wfb state
		state_a = GET_CUBE_GROUND_STATE_A;
		//} else if (get_cube_station) {
		state_a = GET_CUBE_STATION_STATE_A;
		//} else if (post_intake) {
		state_a = POST_INTAKE_STATE_A;
		//} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
		state_a = PLACE_SCALE_STATE_A;
		//} else if (raise_to_switch) {
		state_a = PLACE_SWITCH_STATE_A;
		//}
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
		//	if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
		state_a = PLACE_SCALE_STATE_A;
		//	} else if (raise_to_switch) {
		state_a = PLACE_SWITCH_STATE_A;
		//	} else if (last_state == PLACE_SCALE_STATE
		//	|| last_state == PLACE_SWITCH_STATE) {
		state_a = WAIT_FOR_BUTTON_STATE_A;
		//	}
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
		//	if (std::abs(intake->GetAngularPosition() - intake->MID_ANGLE) <= 0.2
		//			&& state_intake_wheel) { //start shooting when close enough
		intake_->intake_wheel_state = intake_->SLOW_STATE_H;
		if (intake_->ReleasedCube()) {
			state_a = POST_INTAKE_STATE_A;
			//	}
		}
		last_state_a = PLACE_SWITCH_STATE_A;
		//stay in this state when spitting cube, then return to WFB
		break;
	}

}
