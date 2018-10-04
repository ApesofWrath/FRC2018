/*
* AutonStateMachine.cpp
*
*  Created on: Mar 14, 2018
*      Author: DriversStation
*/

#include "AutonStateMachine.h"

using namespace std::chrono;

int last_state_a = 0;

//Auton
const int INIT_STATE_A = 0;
const int WAIT_FOR_BUTTON_STATE_A = 1;
const int GET_CUBE_GROUND_STATE_A = 2;
const int GET_CUBE_STATION_STATE_A = 3;
const int POST_INTAKE_SWITCH_STATE_A = 4;
const int POST_INTAKE_SCALE_STATE_A = 5;
const int PLACE_SCALE_STATE_A = 6;
const int PLACE_SWITCH_STATE_A = 7;
const int PLACE_SCALE_BACKWARDS_STATE_A = 8;

bool came_from_switch = false;
bool is_intake_low_enough_a = false;

int store_last_state = 0;

MiddleStage *mds_a;
Carriage *carr_a;
Intake *intake_a;
DriveController *driveController_a;

Timer *autonTimer = new Timer();

std::thread AutonStateMachineThread;

AutonStateMachine::AutonStateMachine(MiddleStage *mds_, Carriage *carr_, Intake *intake_,
	DriveController *drive_controller) {

		mds_a = mds_;
		carr_a = carr_;
		intake_a = intake_;
		driveController_a = drive_controller;

		has_started_shoot = false;
		shoot_counter = 0;
		shoot_cube = false; //TODO: add shoot_cube to other shoot states

	}

	void AutonStateMachine::StateMachineAuton(bool wait_for_button,
		bool intake_spin_in, bool intake_spin_out, bool intake_spin_slow,
		bool intake_spin_stop, bool get_cube_ground, bool get_cube_station,
		bool post_intake, bool raise_to_switch, bool raise_to_scale,
		bool intake_arm_up, bool intake_arm_mid, bool intake_arm_down,
		bool mds_up, bool mds_mid, bool mds_down, bool open_intake, bool close_intake, bool carr_up,
		bool carr_mid, bool carr_down,
		bool raise_to_scale_backwards) {

			switch (state_a) {

				case INIT_STATE_A:
				//this always has to run only once
				SmartDashboard::PutString("STATE", "INIT");
				mds_a->elevator_state = mds_a->INIT_STATE_E_H;
				carr_a->elevator_state = carr_a->INIT_STATE_E_H;
				intake_a->intake_arm_state = intake_a->INIT_STATE_H;
				intake_a->intake_solenoid_state = intake_a->CLOSE_STATE_H;
				intake_a->intake_wheel_state = intake_a->STOP_WHEEL_STATE_H;
				if (mds_a->is_elevator_init && carr_a->is_elevator_init && intake_a->is_init_intake) {
					state_a = WAIT_FOR_BUTTON_STATE_A;
				}
				last_state_a = INIT_STATE_A;
				break;

				case WAIT_FOR_BUTTON_STATE_A: //will start arm up and elev down

				SmartDashboard::PutString("STATE", "WAIT FOR BUTTON.");
				if (get_cube_ground) { //can go to all states below wfb state
					state_a = GET_CUBE_GROUND_STATE_A;
				} else if (get_cube_station) {
					state_a = GET_CUBE_STATION_STATE_A;
				} else if (post_intake) {
					state_a = POST_INTAKE_SWITCH_STATE_A;
				} else if (raise_to_scale) { //should not need to go from wfb state to a raise state, but in case
					state_a = PLACE_SCALE_STATE_A;
				} else if (raise_to_switch) {
					state_a = PLACE_SWITCH_STATE_A;
				} else if (raise_to_scale_backwards) {
					state_a = PLACE_SCALE_BACKWARDS_STATE_A;
				}
				last_state_a = WAIT_FOR_BUTTON_STATE_A;
				break;

				case GET_CUBE_GROUND_STATE_A:

				SmartDashboard::PutString("STATE", "GET CUBE GROUND");
				mds_a->elevator_state = mds_a->DOWN_STATE_E_H;
				intake_a->intake_wheel_state = intake_a->IN_STATE_H;
				intake_a->intake_solenoid_state = intake_a->OPEN_STATE_H;
				intake_a->intake_arm_state = intake_a->DOWN_STATE_H;
				if (intake_a->HaveCube()) {
					state_a = POST_INTAKE_SWITCH_STATE_A;
				}
				last_state_a = GET_CUBE_GROUND_STATE_A;
				break;

				case GET_CUBE_STATION_STATE_A: //human player station

				SmartDashboard::PutString("STATE", "GET CUBE STATION");
				mds_a->elevator_state = mds_a->HPS_STATE_E_H;
				intake_a->intake_wheel_state = intake_a->IN_STATE_H;
				intake_a->intake_arm_state = intake_a->DOWN_STATE_H;
				if (intake_a->HaveCube()) {
					state_a = POST_INTAKE_SWITCH_STATE_A;
				}
				last_state_a = GET_CUBE_STATION_STATE_A;
				break;

				case POST_INTAKE_SWITCH_STATE_A: //for switch and forward scale

				SmartDashboard::PutString("STATE", "POST INTAKE SWITCH");

				mds_a->elevator_state = mds_a->DOWN_STATE_E_H;
				intake_a->intake_solenoid_state = intake_a->CLOSE_STATE_H;
				intake_a->intake_arm_state = intake_a->UP_STATE_H;
				intake_a->intake_wheel_state = intake_a->STOP_WHEEL_STATE_H;

				if(last_state_a == PLACE_SWITCH_STATE_A || last_state_a == PLACE_SCALE_STATE_A) { //after place scale backwards, does not go to this state
					state_a = WAIT_FOR_BUTTON_STATE_A;
				} else if (raise_to_switch && last_state_a != PLACE_SWITCH_STATE_A) {
					state_a = PLACE_SWITCH_STATE_A;
				} else if (raise_to_scale && last_state_a != PLACE_SCALE_STATE_A) { //came from placing
					state_a = PLACE_SCALE_STATE_A;
				} else if (raise_to_scale_backwards && last_state_a != PLACE_SCALE_BACKWARDS_STATE_A) { //came from placing
					state_a = PLACE_SCALE_BACKWARDS_STATE_A;
				}
				last_state_a = POST_INTAKE_SWITCH_STATE_A;
				//can always go back to wait for button state
				break;

				case POST_INTAKE_SCALE_STATE_A: //backwards scale, is exclusively for post-shooting, not post intake

				SmartDashboard::PutString("STATE", "POST INTAKE SCALE");

				is_intake_low_enough_a = (intake_a->GetAngularPosition()
				< (intake_a->SWITCH_ANGLE + 0.05)); //use same check for the entirety of the state

				if (is_intake_low_enough_a) { //start moving elevator down once intake has reached mid angle
					mds_a->elevator_state = mds_a->DOWN_STATE_E_H;
					if (mds_a->GetElevatorPosition() < 0.7) {
						intake_a->intake_arm_state = intake_a->UP_STATE_H;
						state_a = WAIT_FOR_BUTTON_STATE_A;
					}
				} else {
					intake_a->intake_arm_state = intake_a->SWITCH_STATE_H;
				}

				intake_a->intake_wheel_state = intake_a->STOP_WHEEL_STATE_H;
				last_state_a = POST_INTAKE_SCALE_STATE_A;
				//can always go back to wait for button state
				break;

				case PLACE_SCALE_STATE_A: //for the shooting states, drivecontroller->stopprofile is called in the runstatemachine functions to start getting superstructure ready to shoot before drive has gotten to shooting spot

				SmartDashboard::PutString("STATE", "SCALE");
				intake_a->intake_arm_state = intake_a->UP_STATE_H;
				mds_a->elevator_state = mds_a->UP_STATE_E_H;
				if (mds_a->GetElevatorPosition() >= 0.55) { //start shooting at 0.6
					intake_a->intake_wheel_state = intake_a->OUT_STATE_H;
					if (intake_a->ReleasedCube(intake_a->SCALE)) {
						state_a = POST_INTAKE_SWITCH_STATE_A;
						shoot_counter++;
					}
				}
				last_state_a = PLACE_SCALE_STATE_A;
				//stay in this state when spitting cube, then return to WFB
				break;

				case PLACE_SWITCH_STATE_A:

				SmartDashboard::PutString("STATE", "SWITCH.");
				mds_a->elevator_state = mds_a->MID_STATE_E_H;
				intake_a->intake_arm_state = intake_a->MID_STATE_H;
				if (std::abs(intake_a->GetAngularPosition() - intake_a->MID_ANGLE)
				<= 0.2) { //start shooting when high enough
						intake_a->intake_solenoid_state = intake_a->OPEN_STATE_H;
						intake_a->intake_wheel_state = intake_a->SLOW_STATE_H;
					if (intake_a->ReleasedCube(intake_a->SWITCH)) { //param does not matter: that's lie
					shoot_counter++;
					state_a = POST_INTAKE_SWITCH_STATE_A;
				}
			}
			last_state_a = PLACE_SWITCH_STATE_A;
			//stay in this state when spitting cube, then return to WFB
			break;

			case PLACE_SCALE_BACKWARDS_STATE_A:

			SmartDashboard::PutString("STATE", "SCALE BACKWARDS");

			double el_pos = mds_a->GetElevatorPosition();
			double arm_pos = intake_a->GetAngularPosition();

			if (el_pos >= .85) { //move to the flippy angle when safe
				intake_a->intake_arm_state = intake_a->SWITCH_BACK_SHOT_STATE_H;
			} else if (el_pos < .85) { //move to normal up angle if not safe to go all the way to flippy angle
				intake_a->intake_arm_state = intake_a->UP_STATE_H;
			}
			mds_a->elevator_state = mds_a->UP_STATE_E_H;
			if (el_pos >= 0.82
				&& arm_pos > 1.9 && shoot_cube) { //shoot if the height of the elevator and the angle of the arm is good enough //WAS 1.98
					intake_a->intake_wheel_state = intake_a->OUT_STATE_H;
					//std::cout << "intake out " << std::endl;
					if (intake_a->ReleasedCube(intake_a->BACK)) {
						state_a = POST_INTAKE_SCALE_STATE_A;
						shoot_counter++;
					}
				}

				last_state_a = PLACE_SCALE_BACKWARDS_STATE_A;

				break;
			}

		}

		void AutonStateMachine::Initialize() {

			mds_a->zeroing_counter_e = 0;
			carr_a->zeroing_counter_e = 0;
			intake_a->zeroing_counter_i = 0;

			intake_a->is_init_intake = false;
			mds_a->is_elevator_init = false;
			carr_a->is_elevator_init = false;

			state_a = INIT_STATE_A;

		}
