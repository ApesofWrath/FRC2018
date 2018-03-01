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

const int NUM_POINTS = 1500; //every 10ms
const int NUM_INDEX = 10; //change this

double refs[NUM_POINTS][NUM_INDEX];

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;

}


void Autonomous::AutonStateMachine() {

//	switch (state) {
//
//	case INIT_STATE:
//
//		SmartDashboard::PutString("STATE", "INIT");
//
//		elevator->elevator_state = elevator->INIT_STATE_E_H;
//		intake->intake_arm_state = intake->INIT_STATE_H;
//		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
//		state = WAIT_FOR_BUTTON_STATE;
//		last_state = INIT_STATE;
//		break;
//
//	case WAIT_FOR_BUTTON_STATE: //will look at position
//
//		SmartDashboard::PutString("STATE", "WAIT FOR BUTTON");
//
//		if (get_cube_ground) { //can go to all states below wfb state
//			state = GET_CUBE_GROUND_STATE;
//		} else if (get_cube_station) {
//			state = GET_CUBE_STATION_STATE;
//		} else if (post_intake) {
//			state = POST_INTAKE_STATE;
//		} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
//			state = PLACE_SCALE_STATE;
//		} else if (raise_to_switch) {
//			state = PLACE_SWITCH_STATE;
//		}
//		last_state = WAIT_FOR_BUTTON_STATE;
//		break;
//
//	case GET_CUBE_GROUND_STATE:
//
//		SmartDashboard::PutString("STATE", "GET CUBE GROUND");
//
//		//std::cout << "state intake " << state_intake_arm << std::endl;
//		if (state_elevator) {
//			elevator->elevator_state = elevator->DOWN_STATE_E_H;
//			//SmartDashboard::PutBoolean("state elevator", state_elevator);
//		}
//		if (state_intake_wheel) {
//			intake->intake_wheel_state = intake->IN_STATE_H;
//			//SmartDashboard::PutBoolean("state wheel", state_intake_wheel);
//		}
//		if (state_intake_arm) {
//			intake->intake_arm_state = 3;
////			std::cout << "intake arm state: " << intake->intake_arm_state
////					<< std::endl;
//		}
//		if (intake->HaveCube() || post_intake) {
//			state = POST_INTAKE_STATE;
//		}
//		last_state = GET_CUBE_GROUND_STATE;
//		break;
//
//	case GET_CUBE_STATION_STATE: //human player station
//
//		SmartDashboard::PutString("STATE", "GET CUBE STATION");
//
//		if (state_elevator) {
//			elevator->elevator_state = elevator->SWITCH_STATE_E_H;
//		}
//		if (state_intake_wheel) {
//			intake->intake_wheel_state = intake->IN_STATE_H;
//		}
//		if (state_intake_arm) {
//			intake->intake_arm_state = intake->DOWN_STATE_H;
//		}
//		if (intake->HaveCube() || post_intake) {
//			state = POST_INTAKE_STATE;
//		}
//		last_state = GET_CUBE_STATION_STATE;
//		break;
//
//	case POST_INTAKE_STATE: //have cube, waiting to place cube
//		if (state_elevator) {
//			elevator->elevator_state = elevator->DOWN_STATE_E_H;
//		}
//		if (state_intake_arm) {
//			//std::cout << "uP /state " << std::endl;
//			intake->intake_arm_state = intake->UP_STATE_H;
//		}
//		if (state_intake_wheel) {
//			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
//		}
//		if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
//			state = PLACE_SCALE_STATE;
//		} else if (raise_to_switch) {
//			state = PLACE_SWITCH_STATE;
//		} else if (last_state == PLACE_SCALE_STATE
//				|| last_state == PLACE_SWITCH_STATE) {
//			state = WAIT_FOR_BUTTON_STATE;
//		}
//		last_state = POST_INTAKE_STATE;
//		//can always go back to wait for button state
//		break;
//
//	case PLACE_SCALE_STATE:
//
//		SmartDashboard::PutString("STATE", "SCALE");
//
//		if (state_intake_arm) {
//			intake->intake_arm_state = intake->UP_STATE_H;
//		}
//		if (state_elevator) {
//			elevator->elevator_state = elevator->UP_STATE_E_H;
//		}
//		if (elevator->GetElevatorPosition() >= 0.55 && state_intake_wheel) { //start shooting at 0.6
//			intake->intake_wheel_state = intake->OUT_STATE_H;
//			if (intake->ReleasedCube()) {
//				state = POST_INTAKE_STATE;
//			}
//		}
//		last_state = PLACE_SCALE_STATE;
//		//stay in this state when spitting cube, then return to WFB
//		break;
//
//	case PLACE_SWITCH_STATE:
//
//		SmartDashboard::PutString("STATE", "SWITCH");
//
//		if (state_elevator) {
//			elevator->elevator_state = elevator->MID_STATE_E_H;
//		}
//		if (state_intake_arm) { //elevator->GetElevatorPosition() >= 0.1 &&
//			intake->intake_arm_state = intake->MID_STATE_H;
//		}
//		if (std::abs(intake->GetAngularPosition() - intake->MID_ANGLE) <= 0.2
//				&& state_intake_wheel) { //start shooting when close enough
//			intake->intake_wheel_state = intake->SLOW_STATE_H;
//			if (intake->ReleasedCube()) {
//				state = POST_INTAKE_STATE;
//			}
//		}
//		last_state = PLACE_SWITCH_STATE;
//		//stay in this state when spitting cube, then return to WFB
//		break;
//	}

}

//void Autonomous::FillProfile(std::string profileName) { //fill array and run auton, extra column of 0s in csv are not carried over into array //not needed
//
//	for (int r = 0; r < NUM_POINTS; r++) { //sets the entire array to 0 so that all the points that arent filled are zeros, easy to check for
//		for (int c = 0; c < NUM_INDEX; c++) {
//			refs[r][c] = 0;
//		}
//	}
//
//	int r = 0;
//	std::fstream file(profileName, std::ios::in);
//	while (r < NUM_POINTS) {
//		std::string data;
//		std::getline(file, data);
//		std::stringstream iss(data);
//		if (!file.good()) {
//			std::cout << "FAIL" << std::endl;
//		}
//		int c = 0;
//		while (c < NUM_INDEX) {
//			std::string val;
//			std::getline(iss, val, ',');
//			std::stringstream convertor(val);
//			convertor >> refs[r][c];
//			c++;
//			//if (file.eof()) {
//			//	drive_controller->SetProfileLength(r); //sets array length to length of csv file
//			//}
//		}
//		r++;
//	}
//
//	//drive_controller->SetRef(refs);
//	drive_controller->StartAutonThreads();
//
//}


//void Autonomous::FillProfile(std::string profileName) { //fill array and run auton, extra column of 0s in csv are not carried over into array
//
//	for (int r = 0; r < NUM_POINTS; r++) { //sets the entire array to 0 so that all the points that arent filled are zeros, easy to check for
//		for (int c = 0; c < NUM_INDEX; c++) {
//			refs[r][c] = 0;
//		}
//	}
//
//	int r = 0;
//	std::fstream file(profileName, std::ios::in);
//	while (r < NUM_POINTS) {
//		std::string data;
//		std::getline(file, data);
//		std::stringstream iss(data);
//		if (!file.good()) {
//			std::cout << "FAIL" << std::endl;
//		}
//		int c = 0;
//		while (c < NUM_INDEX) {
//			std::string val;
//			std::getline(iss, val, ',');
//			std::stringstream convertor(val);
//			convertor >> refs[r][c];
//			c++;
//			//if (file.eof()) {
//			//	drive_controller->SetProfileLength(r); //sets array length to length of csv file
//			//}
//		}
//		r++;
//	}
//
//	//drive_controller->SetRef(refs);
//	drive_controller->StartAutonThreads();
//
//}
