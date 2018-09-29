/*
 * TeleopStateMachine.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

//TODO: add safety in case we end auton unsafely
#include "TeleopStateMachine.h"
#include <WPILib.h>

using namespace std::chrono;

//Teleop
const int INIT_STATE = 0;
const int WAIT_FOR_BUTTON_STATE = 1;
const int GET_CUBE_GROUND_STATE = 2;
const int GET_CUBE_STATION_STATE = 3;
const int POST_INTAKE_SWITCH_STATE = 4; //once we have gotten a cube, AND after we have shot a cube
const int POST_INTAKE_SCALE_STATE = 5; //scale AND backwards scale
const int PLACE_SCALE_SLOW_STATE = 6;
const int PLACE_SCALE_MED_STATE = 7;
const int PLACE_SCALE_FAST_STATE = 8;
const int PLACE_SWITCH_STATE = 9;
const int PLACE_SWITCH_POP_STATE = 10;
const int PLACE_SCALE_BACKWARDS_STATE = 11;
int state = INIT_STATE;
//TODO: post_outtake?

bool state_intake_wheel = false; //set to true to override the states set in the state machine
bool state_intake_arm = false;
bool state_mds = false;
bool state_carr = false;

bool is_intake_low_enough, is_carr_low_enough, is_mds_low_enough;

int last_state = 0;

bool arm_up = true;

MiddleStage *mds;
Carriage *carr;
Intake *intake;
DriveController *driveController;

TeleopStateMachine::TeleopStateMachine(MiddleStage *mds_, Carriage *carr_, Intake *intake_,
		DriveController *drive_controller) {

	mds = mds_;  //current elevator will be middle stage
	carr = carr_;
	intake = intake_;
	driveController = drive_controller;

}

void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_slow, bool intake_spin_med,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool pop_switch, bool raise_to_scale_slow,
		bool raise_to_scale_med, bool raise_to_scale_fast, bool intake_arm_up,
		bool intake_arm_mid, bool intake_arm_down, bool mds_up, bool mds_mid, bool mds_down,
	  bool carr_down, bool carr_mid, bool carr_up, bool raise_to_scale_backwards) {

		//	std::cout << "mds: " << mds_up << std::endl;

	if (wait_for_button) { //can always return to wait for button state
		state = WAIT_FOR_BUTTON_STATE;
	}

	is_carr_low_enough = carr->GetElevatorPosition() < carr->SAFE_CARR_HEIGHT; //carr low enough for mds to start moving down

	//intake wheels
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
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H; //cannot do manual arm up and pop shot
	} else if (intake_arm_mid) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->MID_STATE_H;
	} else if (intake_arm_down) {
		state_intake_arm = false;
		intake->intake_arm_state = intake->DOWN_STATE_H;
	} else {
		state_intake_arm = true;
	}
	//may need to add safeties even for manual control
	//middle stage
	if (mds_up) {
		state_mds = false;
		//	std::cout << "mds up in MANUAL" << std::endl;
		mds->elevator_state = mds->UP_STATE_E_H;
	} else if (mds_mid) {
		state_mds = false;
	//		std::cout << "mds mid in MANUAL" << std::endl;
		mds->elevator_state = mds->MID_STATE_E_H;
	} else if (mds_down) {
		state_mds = false;
	//		std::cout << "mds down in MANUAL" << std::endl;
		mds->elevator_state = mds->DOWN_STATE_E_H;
	} else {
		state_mds = true;
	}

	//carriage
	if (carr_up) {
		state_carr = false;

		carr->elevator_state = carr->UP_STATE_E_H;
	} else if (carr_mid) {
		state_carr = false;
	//	std::cout << "carr mid in MANUAL" << std::endl;
		carr->elevator_state = carr->MID_STATE_E_H;
	} else if (carr_down) {
		state_carr = false;
	//	std::cout << "carr mid in MANUAL" << std::endl;
		carr->elevator_state = carr->DOWN_STATE_E_H;
	} else {
		state_carr = true;
	}


	switch (state) {

	case INIT_STATE:

		SmartDashboard::PutString("STATE", "INIT");
		mds->elevator_state = mds->DOWN_STATE_E_H;
		carr->elevator_state = carr->DOWN_STATE_E_H;
		intake->intake_arm_state = intake->UP_STATE_H;
		//mds->elevator_state = mds->INIT_STATE_E_H; //initialize in ausm
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
			state = POST_INTAKE_SWITCH_STATE;
		} else if (raise_to_scale_slow) {
			state = PLACE_SCALE_SLOW_STATE;
		} else if (raise_to_scale_med) {
			state = PLACE_SCALE_MED_STATE;
		} else if (raise_to_scale_fast) {
			state = PLACE_SCALE_FAST_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		} else if (pop_switch) {
			state = PLACE_SWITCH_POP_STATE;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		}
		last_state = WAIT_FOR_BUTTON_STATE;
		break;

	case GET_CUBE_GROUND_STATE:

		SmartDashboard::PutString("STATE", "GET CUBE GROUND");

		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_carr) {
			carr->elevator_state = carr->DOWN_STATE_E_H;
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

		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_carr) {
			carr->elevator_state = carr->HPS_STATE_E_H;
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

	case POST_INTAKE_SWITCH_STATE: //do not use this for after scale //scale-scale ends in this state

		SmartDashboard::PutString("STATE", "POST INTAKE SWITCH");

		is_intake_low_enough = (intake->GetAngularPosition()
				< (intake->UP_ANGLE + 0.05)); //use same check for the entirety of the state

		if (state_carr) { //higher elevator heights will trigger the safety and zero the elevator, bringing it down too early
			carr->elevator_state = carr->DOWN_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H; //have to change up angle because we don't GO to safe angle
		}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}

		if (raise_to_scale_slow) {
			state = PLACE_SCALE_SLOW_STATE;
		} else if (raise_to_scale_med) {
			state = PLACE_SCALE_MED_STATE; //go to place from this state, return to this state after placing and then wfb
		} else if (raise_to_scale_fast) {
			state = PLACE_SCALE_FAST_STATE;
		} else if (raise_to_switch) {
			state = PLACE_SWITCH_STATE;
		} else if (raise_to_scale_backwards) {
			state = PLACE_SCALE_BACKWARDS_STATE;
		} else if (last_state == PLACE_SCALE_SLOW_STATE || last_state == PLACE_SCALE_MED_STATE || last_state == PLACE_SCALE_FAST_STATE//will keep checking if arm is low enough to start lowering the elevator
		|| last_state == PLACE_SWITCH_STATE || is_intake_low_enough) { //little bit of a hack but the check wont run if it only goes through this state once
			state = WAIT_FOR_BUTTON_STATE;
		}
		last_state = POST_INTAKE_SWITCH_STATE;
		//can always go back to wait for button state
		break;

	case POST_INTAKE_SCALE_STATE:

		SmartDashboard::PutString("STATE", "POST INTAKE SCALE");

		is_intake_low_enough = (intake->GetAngularPosition()
				< (intake->SWITCH_ANGLE + 0.05)); //use same check for the entirety of the state

		if (state_carr) {
			carr->elevator_state = carr->DOWN_STATE_E_H;
			if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
			if (mds->GetElevatorPosition() < 0.7) {
				intake->intake_arm_state = intake->UP_STATE_H;
				state = WAIT_FOR_BUTTON_STATE;
			}
		}
		else if (state_intake_arm) {
			intake->intake_arm_state = intake->SWITCH_STATE_H;
		}
	}
		if (state_intake_wheel) {
			intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		}
		last_state = POST_INTAKE_SCALE_STATE;
		//can always go back to wait for button state
		break;

	case PLACE_SCALE_SLOW_STATE:

		SmartDashboard::PutString("STATE", "SCALE SLOW FORWARDS");

		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_carr) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (carr->GetElevatorPosition() >= 0.84 && mds->GetElevatorPosition() < 0.2 //TODO: actually find
		&& !raise_to_scale_slow) { //hold button until ready to shoot, elevator and intake will be in positio
			intake->intake_wheel_state = intake->SLOW_STATE_H;
			if (intake->ReleasedCube(intake->SLOW_SCALE)) {
				state = POST_INTAKE_SCALE_STATE;
			}
		}
		last_state = PLACE_SCALE_SLOW_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SCALE_MED_STATE:

		SmartDashboard::PutString("STATE", "SCALE MED FORWARDS");

		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_carr) {
		//		std::cout << "carr up in STATE" << std::endl;
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (carr->GetElevatorPosition() >= 0.84 && mds->GetElevatorPosition() < 0.2 //TODO: actually find//&& state_intake_wheel
		&& !raise_to_scale_med) { //hold button until ready to shoot, elevator and intake will be in position
			intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H;
			if (intake->ReleasedCube(intake->SLOW_SCALE)) {
				state = POST_INTAKE_SCALE_STATE;
			}
		}
		last_state = PLACE_SCALE_MED_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SCALE_FAST_STATE: //if mess up on strength button, can TRY keep holding the button, press and hold down wfb, let go of the shoot button, and them manual outtake

		SmartDashboard::PutString("STATE", "SCALE FAST FORWARDS");

		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_carr) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (carr->GetElevatorPosition() >= 0.84 && mds->GetElevatorPosition() < 0.2 //TODO: actually find//&& state_intake_wheel
		&& !raise_to_scale_med) { //hold button until ready to shoot, elevator and intake will be in position
			intake->intake_wheel_state = intake->OUT_STATE_H;
			if (intake->ReleasedCube(intake->SLOW_SCALE)) {
				state = POST_INTAKE_SCALE_STATE;
			}
		}
		last_state = PLACE_SCALE_FAST_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SWITCH_STATE: //moves carr first, then mds

		SmartDashboard::PutString("STATE", "SWITCH");

		if (state_carr) {
			mds->elevator_state = mds->MID_STATE_E_H;
		}
		// if (state_mds && carr->IsAtPos(carr->MID_POS_CARR)) {
		// 	mds->elevator_state = mds->DOWN_STATE_E_H;
		// }
		if (state_intake_arm) {
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

	case PLACE_SWITCH_POP_STATE:

		SmartDashboard::PutString("STATE", "POP SWITCH");

		if (state_carr) {
			carr->elevator_state = carr->DOWN_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_intake_arm) { //mds->GetElevatorPosition() >= 0.1 &&
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (std::abs(intake->GetAngularPosition() - intake->UP_ANGLE) <= 0.2 //switch will not shoot if you press a shooting button
		&& state_intake_wheel && mds->IsAtPos(0.1) && !pop_switch) { //hold button until ready to shoot, elevator and intake will be in position //state_intake_wheel means you let go of intake_spin_mid
			intake->intake_wheel_state = intake->POP_SWITCH_STATE_H;
			if (intake->ReleasedCube(intake->SWITCH)) {
				state = POST_INTAKE_SWITCH_STATE;
			}
		}
		last_state = PLACE_SWITCH_STATE;
		//stay in this state when spitting cube, then return to WFB
		break;

	case PLACE_SCALE_BACKWARDS_STATE:

		SmartDashboard::PutString("STATE", "SCALE BACKWARDS");

		if (state_intake_arm && carr->GetElevatorPosition() >= .85) { //move to the flippy angle when safe
			intake->intake_arm_state = intake->SWITCH_BACK_SHOT_STATE_H;
		} else if (state_intake_arm && carr->GetElevatorPosition() < .85) { //move to normal up angle if not safe to go all the way to flippy angle
			intake->intake_arm_state = intake->UP_STATE_H;
		}

		if (state_mds) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_carr && mds->IsAtPos(mds->DOWN_POS_MDS)) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (carr->GetElevatorPosition() >= 0.85
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
