/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//CHANGE: hold button until ready to shoot, elevator and intake will be in position - test this
//TODO: add safety in case we end auton unsafely
#include <TeleopStateMachine.h>
#include <WPILib.h>

using namespace std::chrono;

//Teleop
const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_GROUND_STATE = 2;
const int GET_CUBE_STATION_STATE = 3;
const int POST_INTAKE_SWITCH_STATE = 4; //once we have gotten a cube, AND after we have shot a cube
const int POST_INTAKE_SCALE_STATE = 5; //scale AND backwards scale
const int PLACE_SCALE_STATE = 6;
const int PLACE_SWITCH_STATE = 7;
const int PLACE_SCALE_BACKWARDS_STATE = 8;
int state = INIT_STATE;

bool state_intake_wheel = false; //set to true to override the states set in the state machine
bool state_intake_arm = false;
bool state_elevator = false;

bool is_intake_low_enough;

int last_state = 0;

bool arm_up = true;

Elevator *elevator;
Intake *intake;
DriveController *driveController;

TeleopStateMachine::TeleopStateMachine(Elevator *elevator_, Intake *intake_,
		DriveController *drive_controller) {

	elevator = elevator_;
	intake = intake_;
	driveController = drive_controller;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_slow, bool intake_spin_med,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool raise_to_scale,
		bool intake_arm_up, bool intake_arm_mid, bool intake_arm_down,
		bool elevator_up, bool elevator_mid, bool elevator_down,
		bool raise_to_scale_backwards) {

	if (wait_for_button) { //can always return to wait for button state
		state = WAIT_FOR_BUTTON_STATE;
	}
	//same with intake_spin_med
	//intake wheels -- intake_spin_slow CANNOT make state_intake_wheel false or else when shooting forward/backward scale will not outtake cube
	if (intake_spin_out) { //driver's slow button can control intake spin speed always
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->OUT_STATE_H;
	} else if (intake_spin_in) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->IN_STATE_H;
	} else if (intake_spin_stop) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
	} else if (intake_spin_med) { //'pop' shot
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H; //stronger than slow
	} else if (intake_spin_slow) {
		state_intake_wheel = false;
		intake->intake_wheel_state = intake->SLOW_STATE_H;
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

		SmartDashboard::PutString("STATE", "INIT"); //stay up
//		if (intake->GetAngularPosition() > 1.7 && elevator->elevator_state == elevator_->UP_STATE_H) { //depends on arm being already initialized from auton //this height is from the safety for the elevator in intake's setvoltage()
//			elevator->elevator_state = elevator->UP_STATE_E_H; //there is the safety for the elevator to not go down if the arm is too high (in setvoltagelevator), but that safety will basically just kill the elevator.
//			//since the elevator will be slowly going down, the arm will not be able to go down for the elevator to actually go down. //will need to manually down if this happens
//		} else {
		elevator->elevator_state = elevator->DOWN_STATE_E_H;
		//	}
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
			state = POST_INTAKE_SWITCH_STATE; //TODO: note that will go to switch
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
			state = POST_INTAKE_SWITCH_STATE; //TODO: should not matter which post intake state, but look at this again
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
			state = POST_INTAKE_SWITCH_STATE; //TODO: should not matter which post intake state, but look at this again
		}
		last_state = GET_CUBE_STATION_STATE;
		break;

	case POST_INTAKE_SWITCH_STATE: //does not have the intermediate height for the arm. do not use this for after scale //scale-scale ends in this state

		SmartDashboard::PutString("STATE", "POST INTAKE SWITCH");

		is_intake_low_enough = (intake->GetAngularPosition()
				< (intake->UP_ANGLE + 0.05)); //use same check for the entirety of the state

		if (state_elevator && is_intake_low_enough) { //last_state == PLACE_SCALE_BACKWARDS_STATE - don't need this in the check since it will only be true the first time //higher elevator heights will trigger the safety and zero the elevator, bringing it down too early
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
			//intake->intake_arm_state = intake->UP_STATE_H; //TODO: why is this commented out
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H; //have to change up angle because we don't GO to safe angle
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		if (raise_to_scale) { //go to place from this state, return to this state after placing and then wfb
			state = PLACE_SCALE_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		} else if (last_state == PLACE_SCALE_STATE //will keep checking if arm is low enough to start lowering the elevator
		|| last_state == PLACE_SWITCH_STATE || is_intake_low_enough) { //little bit of ahack but the check wont run if it only goes through this state once

			state = WAIT_FOR_BUTTON_STATE;
		}
		last_state = POST_INTAKE_SWITCH_STATE;
		//can always go back to wait for button state
		break;

	case POST_INTAKE_SCALE_STATE: //HAS been changed

		//arm down to mid_ang, elev up
		//arm up to up_ang, elev going down

		SmartDashboard::PutString("STATE", "POST INTAKE SCALE");

		is_intake_low_enough = (intake->GetAngularPosition()
				< (intake->SWITCH_ANGLE + 0.05)); //use same check for the entirety of the state

		if (state_elevator && is_intake_low_enough) { //start moving elevator down once intake has reached mid angle
			elevator->elevator_state = elevator->DOWN_STATE_E_H;
			if (elevator->GetElevatorPosition() < 0.7) {
				intake->intake_arm_state = intake->UP_STATE_H;
				state = WAIT_FOR_BUTTON_STATE;
			}
		} else if (state_intake_arm) {
			intake->intake_arm_state = intake->SWITCH_STATE_H;
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		last_state = POST_INTAKE_SCALE_STATE;
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
		if (elevator->GetElevatorPosition() >= 0.55 //&& state_intake_wheel
		&& !raise_to_scale) { //hold button until ready to shoot, elevator and intake will be in position
			if (intake_spin_slow) {
				intake->intake_wheel_state = intake->SLOW_STATE_H;
				if (intake->ReleasedCube(intake->SLOW_SCALE)) {
					state = POST_INTAKE_SCALE_STATE;
				}
			} else if (intake_spin_out) {
				intake->intake_wheel_state = intake->OUT_STATE_H;
				if (intake->ReleasedCube(intake->SCALE)) {
					//state = POST_INTAKE_SWITCH_STATE;
					state = POST_INTAKE_SCALE_STATE;
				}
			} else {
				intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H;
				if (intake->ReleasedCube(intake->SLOW_SCALE)) {
					state = POST_INTAKE_SCALE_STATE;
				}
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
		if (std::abs(intake->GetAngularPosition() - intake->MID_ANGLE) <= 0.2 //switch will not shoot if you press a shooting button
		&& state_intake_wheel && !raise_to_switch) { //hold button until ready to shoot, elevator and intake will be in position
			intake->intake_wheel_state = intake->SLOW_STATE_H;
			if (intake->ReleasedCube(intake->SWITCH)) {
				state = POST_INTAKE_SWITCH_STATE;
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
				&& intake->GetAngularPosition() > 1.98 //&& state_intake_wheel
				&& !raise_to_scale_backwards) { //shoot if the height of the elevator and the angle of the arm is good enough //hold button until ready to shoot, elevator and intake will be in position
			if (!intake_spin_slow) {
				intake->intake_wheel_state = intake->OUT_STATE_H;
				if (intake->ReleasedCube(intake->BACK)) {
					state = POST_INTAKE_SCALE_STATE;
				}
			} else {
				intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H;
				if (intake->ReleasedCube(intake->BACK)) {
					state = POST_INTAKE_SCALE_STATE;
				}
			}

		}

		last_state = PLACE_SCALE_BACKWARDS_STATE;

		break;
	}

}
