/*
 * TaskManager.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: DriversStation
 */

#include "TaskManager.h"

Timer *threadTimer = new Timer();

std::thread Thread;

TeleopStateMachine *teleop_state_machine;
AutonStateMachine *auton_state_machine;
DriveController *drive_controller;
Elevator *elevator_t;
Intake *intake_t;

TaskManager::TaskManager(TeleopStateMachine *tsm, AutonStateMachine *ausm,
		DriveController *dc, Elevator *el, Intake *in) {

	teleop_state_machine = tsm;
	auton_state_machine = ausm;
	drive_controller = dc;
	elevator_t = el;
	intake_t = in;

}

void TaskManager::StartThread(bool *wait_for_button, bool *intake_spin_in,
		bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_stop,
		bool *get_cube_ground, bool *get_cube_station, bool *post_intake,
		bool *raise_to_switch, bool *raise_to_scale, bool *intake_arm_up,
		bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up,
		bool *elevator_mid, bool *elevator_down, bool *raise_to_scale_backwards,
		Joystick *JoyThrottle, Joystick *JoyWheel) {

	TaskManager *tm = this;
	Thread = std::thread(&TaskManager::ThreadWrapper, tm, JoyThrottle, JoyWheel,
			wait_for_button, intake_spin_in, intake_spin_out, intake_spin_slow,
			intake_spin_stop, get_cube_ground, get_cube_station, post_intake,
			raise_to_switch, raise_to_scale, intake_arm_up, intake_arm_mid,
			intake_arm_down, elevator_up, elevator_mid, elevator_down,
			raise_to_scale_backwards);
	Thread.detach();

}

void TaskManager::ThreadWrapper(TaskManager *task_manager,
		Joystick *JoyThrottle, Joystick *JoyWheel, bool *wait_for_button,
		bool *intake_spin_in, bool *intake_spin_out, bool *intake_spin_slow,
		bool *intake_spin_stop, bool *get_cube_ground, bool *get_cube_station,
		bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
		bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down,
		bool *elevator_up, bool *elevator_mid, bool *elevator_down,
		bool *raise_to_scale_backwards) {

	threadTimer->Start();

	while (true) {

		threadTimer->Reset();

		if (frc::RobotState::IsEnabled() && frc::RobotState::IsAutonomous()) {

			intake_t->Rotate();
			elevator_t->Move();

			drive_controller->RunAutonDrive();

			intake_t->IntakeArmStateMachine();
			intake_t->IntakeWheelStateMachine();
			elevator_t->ElevatorStateMachine();

			auton_state_machine->StateMachineAuton((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch,
					(bool) *raise_to_scale, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *elevator_up, (bool) *elevator_mid,
					(bool) *elevator_down, (bool) *raise_to_scale_backwards);

		} else if (frc::RobotState::IsEnabled()
				&& frc::RobotState::IsOperatorControl()) {

			intake_t->Rotate();
			elevator_t->Move();

			drive_controller->TeleopWCDrive(JoyThrottle, JoyWheel); //0.01

			intake_t->IntakeArmStateMachine();
			intake_t->IntakeWheelStateMachine();
			elevator_t->ElevatorStateMachine();

			teleop_state_machine->StateMachine((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch,
					(bool) *raise_to_scale, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *elevator_up, (bool) *elevator_mid,
					(bool) *elevator_down, (bool) *raise_to_scale_backwards);

		}

		double wait_time = 0.02 - threadTimer->Get(); //time step also needs to be changed in the motion profiler parameter

		wait_time *= 1000000;
		if (wait_time < 0) {
			wait_time = 0;
		}

		std::this_thread::sleep_for(std::chrono::microseconds((int) wait_time));

		SmartDashboard::PutNumber("TIME", threadTimer->Get());

	}

}

void TaskManager::EndThread() {

	Thread.~thread(); //does not actually kill the thread

}

