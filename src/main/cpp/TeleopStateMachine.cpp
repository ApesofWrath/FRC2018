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
const int POST_INTAKE_STATE = 4; //once we have gotten a cube, AND after we have shot a cube
const int POST_INTAKE_SCALE_STATE = 5; //scale AND backwards scale
const int SCALE_LOW_STATE = 6;
const int SCALE_MID_STATE = 7;
const int SCALE_HIGH_STATE = 8;
const int SWITCH_STATE = 9;
const int SWITCH_POP_STATE = 10;
const int SCALE_LOW_BACK_STATE = 11;
const int SCALE_MID_BACK_STATE = 12;
const int SCALE_HIGH_BACK_STATE = 13;
const int OUTTAKE_STATE = 14;
const int INIT_CLIMB_STATE = 15;
const int CLIMB_STATE = 16;
int state = INIT_STATE;

bool state_intake_wheel = false; //set to true to override the states set in the state machine
bool state_intake_arm = false;
bool state_mds = false;
bool state_carr = false;
bool state_intake_solenoid = false;

bool is_intake_low_enough, is_carr_low_enough, is_mds_low_enough;

int last_state = 0;

double slider_input = 0.0;

int shot_type = 0;

bool arm_up = true;

MiddleStage *mds;
Carriage *carr;
Intake *intake;
DriveController *driveController;
Climber *climber;

TeleopStateMachine::TeleopStateMachine(MiddleStage *mds_, Carriage *carr_, Intake *intake_, Climber *climber_,
	DriveController *drive_controller) {

		mds = mds_;  //current elevator will be middle stage
		carr = carr_;
		intake = intake_;
		driveController = drive_controller;
		climber = climber_;

	}

	void TeleopStateMachine::StateMachine(bool wait_for_button, bool intake_spin_in,
		bool intake_spin_out, bool intake_spin_slow, bool intake_spin_med,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool pop_switch, bool raise_to_scale_low,
		bool raise_to_scale_mid, bool raise_to_scale_high, bool intake_arm_up,
		bool intake_arm_mid, bool intake_arm_down, bool mds_up, bool mds_mid, bool mds_down, bool open_intake, bool close_intake,
		bool carr_down, bool carr_mid, bool carr_up, bool raise_to_scale_backwards, bool climb_button, Joystick *joySlider) {

			if (wait_for_button) { //can always return to wait for button state
				state = WAIT_FOR_BUTTON_STATE;
			}

			is_carr_low_enough = carr->GetElevatorPosition() < carr->SAFE_CARR_HEIGHT; //carr low enough for mds to start moving down
			is_mds_low_enough = mds->GetElevatorPosition() < mds->SAFE_MDS_HEIGHT; //mds low enough for carr to start

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
		mds->elevator_state = mds->UP_STATE_E_H;
	} else if (mds_mid) {
		state_mds = false;
		mds->elevator_state = mds->MID_STATE_E_H;
	} else if (mds_down) {
		state_mds = false;
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
		carr->elevator_state = carr->MID_STATE_E_H;
	} else if (carr_down) {
		state_carr = false;
		carr->elevator_state = carr->DOWN_STATE_E_H;
	} else {
		state_carr = true;
	}

	if (open_intake) {
		state_intake_solenoid = false;
		intake->intake_solenoid_state = intake->OPEN_STATE_H;
	} else if (close_intake) {
		state_intake_solenoid = false;
		intake->intake_solenoid_state = intake->CLOSE_STATE_H;
	} else {
		state_intake_solenoid = true;
	}

	switch (state) {

		case INIT_STATE:

		SmartDashboard::PutString("STATE", "INIT");
		mds->elevator_state = mds->DOWN_STATE_E_H;
		carr->elevator_state = carr->DOWN_STATE_E_H;
		intake->intake_arm_state = intake->UP_STATE_H;
		intake->intake_solenoid_state = intake->OPEN_STATE_H;
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
			state = POST_INTAKE_STATE;
		} else if (raise_to_scale_low) {
			state = SCALE_LOW_STATE;
		} else if (raise_to_scale_mid) {
			state = SCALE_MID_STATE;
		} else if (raise_to_scale_high) {
			state = SCALE_HIGH_STATE;
		} else if (raise_to_switch) {
			state = SWITCH_STATE;
		} else if (pop_switch) {
			state = SWITCH_POP_STATE;
		} else if (raise_to_scale_backwards) {
			state = SCALE_HIGH_BACK_STATE;
		} else if (climb_button) {
			state = INIT_CLIMB_STATE;
		}
		last_state = WAIT_FOR_BUTTON_STATE;
		break;

		case GET_CUBE_GROUND_STATE:

		SmartDashboard::PutString("STATE", "GET CUBE GROUND");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->OPEN_STATE_H;
		}
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
			intake->intake_arm_state = intake->DOWN_STATE_H; //3
		}
		if (intake->HaveCube()) {
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_GROUND_STATE;
		break;

		case GET_CUBE_STATION_STATE: //human player station

		SmartDashboard::PutString("STATE", "GET CUBE STATION");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->OPEN_STATE_H;
		}
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
		if (intake->HaveCube()) {
			state = POST_INTAKE_STATE;
		}
		last_state = GET_CUBE_STATION_STATE;
		break;

		case POST_INTAKE_STATE: //do not use this for after scale

		SmartDashboard::PutString("STATE", "POST INTAKE");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		}
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

		if (raise_to_scale_low) {
			state = SCALE_LOW_STATE;
		} else if (raise_to_scale_mid) {
			state = SCALE_MID_STATE; //go to place from this state, return to this state after placing and then wfb
		} else if (raise_to_scale_high) {
			state = SCALE_HIGH_STATE;
		} else if (raise_to_switch) {
			state = SWITCH_STATE;
		} else if (raise_to_scale_backwards) {
			state = SCALE_HIGH_BACK_STATE;
		} else if (pop_switch) {
			state = SWITCH_POP_STATE;
		}
		last_state = POST_INTAKE_STATE;
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

		case SCALE_LOW_STATE:

		SmartDashboard::PutString("STATE", "SCALE LOW FORWARDS");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_mds) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (is_mds_low_enough && state_carr) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (!raise_to_scale_low && carr->IsAtPos(carr->UP_POS_CARR) && intake->IsAtAngle(intake->UP_ANGLE)) {
			shot_type = intake->SLOW_SCALE;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_LOW_STATE;
		break;

		case SCALE_MID_STATE:

		SmartDashboard::PutString("STATE", "SCALE MID FORWARDS");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_mds) {
			mds->elevator_state = mds->MID_STATE_E_H;
		}
		if (mds->IsAtPos(mds->MID_POS_MDS) && state_carr) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (!raise_to_scale_mid && carr->IsAtPos(carr->UP_POS_CARR) && intake->IsAtAngle(intake->UP_ANGLE)) {
			shot_type = intake->SLOW_SCALE;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_MID_STATE;
		break;

		case SCALE_HIGH_STATE:

		SmartDashboard::PutString("STATE", "SCALE HIGH FORWARDS");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (state_mds) {
			mds->elevator_state = mds->UP_STATE_E_H;
		}
		if (mds->IsAtPos(mds->UP_POS_MDS) && state_carr) {
			carr->elevator_state = carr->UP_STATE_E_H;
		}
		if (!raise_to_scale_high && carr->IsAtPos(carr->UP_POS_CARR) && intake->IsAtAngle(intake->UP_ANGLE)) {
			shot_type = intake->SLOW_SCALE;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_HIGH_STATE;
		break;

		case SWITCH_STATE:

		SmartDashboard::PutString("STATE", "SWITCH");

		if (state_intake_solenoid) {
			intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		}
		if (state_mds) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_carr && is_mds_low_enough) {
			carr->elevator_state = carr->MID_STATE_E_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->MID_STATE_H;
		}
		if (!raise_to_switch && carr->IsAtPos(carr->MID_POS_CARR) && intake->IsAtAngle(intake->MID_ANGLE)) {
			shot_type = intake->SWITCH;
			state = OUTTAKE_STATE;
		}
		last_state = SWITCH_STATE;
		break;

		case SWITCH_POP_STATE:

		SmartDashboard::PutString("STATE", "POP SWITCH");

		if (state_carr) {
			carr->elevator_state = carr->DOWN_STATE_E_H;
		}
		if (state_mds && is_carr_low_enough) {
			mds->elevator_state = mds->DOWN_STATE_E_H;
		}
		if (state_intake_arm) {
			intake->intake_arm_state = intake->UP_STATE_H;
		}
		if (std::abs(intake->GetAngularPosition() - intake->UP_ANGLE) <= 0.2 //switch will not shoot if you press a shooting button
		&& state_intake_wheel && is_mds_low_enough && !pop_switch) { //hold button until ready to shoot, elevator and intake will be in position //state_intake_wheel means you let go of intake_spin_mid
			shot_type = intake->SWITCH;
			state = OUTTAKE_STATE;
		}
		last_state = SWITCH_STATE;
		break;

		case SCALE_LOW_BACK_STATE: //arm parallel to floor

		SmartDashboard::PutString("STATE", "SCALE LOW BACK");

		if (state_intake_arm && carr->GetElevatorPosition() >= .88) { //move to the flippy angle when safe
			intake->intake_arm_state = intake->LOW_BACK_SHOT_STATE_H;
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
			shot_type = intake->BACK;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_LOW_BACK_STATE;
		break;

		case SCALE_MID_BACK_STATE: //arm parallel to floor

		SmartDashboard::PutString("STATE", "SCALE MID BACK");

		if (state_intake_arm && carr->GetElevatorPosition() >= .88) { //move to the flippy angle when safe
			intake->intake_arm_state = intake->LOW_BACK_SHOT_STATE_H;
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
			shot_type = intake->BACK;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_MID_BACK_STATE;
		break;


		case SCALE_HIGH_BACK_STATE: //arm at regular back angle, angled upwards

		SmartDashboard::PutString("STATE", "SCALE HIGH BACK");

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
			shot_type = intake->BACK;
			state = OUTTAKE_STATE;
		}
		last_state = SCALE_HIGH_BACK_STATE;
		break;

		case OUTTAKE_STATE:

		if (last_state != OUTTAKE_STATE) {
			slider_input = joySlider->GetY(); //or whatever
		}

		if (slider_input > 0.5) { //place
			if (state_intake_solenoid) {
				intake->intake_solenoid_state = intake->OPEN_STATE_H;
				intake->intake_wheel_state = intake->SLOW_STATE_H;
			}
			if (intake->ReleasedCube(shot_type)) {
				state = POST_INTAKE_STATE;
			}
		} else if ((slider_input <= 0.5) && (slider_input > -0.3)) { //shoot slow
			if (state_intake_solenoid) {
				intake->intake_solenoid_state = intake->CLOSE_STATE_H;
				intake->intake_wheel_state = intake->SLOW_SCALE_STATE_H;
			}
			if (intake->ReleasedCube(shot_type)) {
				state = POST_INTAKE_STATE;
			}
		} else if (slider_input <= -0.3) { //shoot fast
			if (state_intake_solenoid) {
				intake->intake_solenoid_state = intake->CLOSE_STATE_H;
				intake->intake_wheel_state = intake->OUT_STATE_H;
			}
			if (intake->ReleasedCube(shot_type)) {
				state = POST_INTAKE_STATE;
			}
		}
		last_state = OUTTAKE_STATE;
		break;

		case INIT_CLIMB_STATE:
		mds->elevator_state = mds->DOWN_STATE_E_H;
		carr->elevator_state = carr->DOWN_STATE_E_H;
		intake->intake_arm_state = intake->UP_STATE_H;
		intake->intake_solenoid_state = intake->CLOSE_STATE_H;
		intake->intake_wheel_state = intake->STOP_WHEEL_STATE_H;
		state = CLIMB_STATE;
		break;

		case CLIMB_STATE:
		climber->elevator_state = climber->UP_STATE_E_H;
		if (false) { //TODO: what check
			climber->elevator_state = climber->STOP_STATE_E_H;
		}
		break;

	}

}
