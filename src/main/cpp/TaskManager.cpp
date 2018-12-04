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
MiddleStage *mds_t;
Carriage *carr_t;
Intake *intake_t;
Climber *climb_t;

TaskManager::TaskManager(TeleopStateMachine *tsm, AutonStateMachine *ausm,
		DriveController *dc, MiddleStage *mds, Carriage *carr, Intake *in, Climber *climb, double thread_time_dt) {

	teleop_state_machine = tsm;
	auton_state_machine = ausm;
	drive_controller = dc;
	mds_t = mds;
	carr_t = carr;
	intake_t = in;
	climb_t = climb;

	thread_time_step = thread_time_dt;

}

void TaskManager::StartThread(bool *wait_for_button, bool *intake_spin_in,
		bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_med, bool *intake_spin_stop,
		bool *get_cube_ground, bool *get_cube_station, bool *post_intake,
		bool *raise_to_switch, bool *pop_switch, bool *raise_to_scale_low, bool *raise_to_scale_mid, bool *raise_to_scale_high, bool *intake_arm_up,
		bool *intake_arm_mid, bool *intake_arm_down, bool *mds_up, bool *mds_mid, bool *mds_down, bool *open_intake, bool *close_intake, bool *carr_up,
		bool *carr_mid, bool *carr_down, bool *raise_to_scale_backwards, bool *climb_button,
		Joystick *JoyThrottle, Joystick *JoyWheel, Joystick *JoySlider, bool *is_heading) {

	TaskManager *tm = this;
	Thread = std::thread(&TaskManager::ThreadWrapper, tm, JoyThrottle, JoyWheel, JoySlider,
			wait_for_button, intake_spin_in, intake_spin_out, intake_spin_slow, intake_spin_med,
			intake_spin_stop, get_cube_ground, get_cube_station, post_intake,
			raise_to_switch, pop_switch, raise_to_scale_low, raise_to_scale_mid, raise_to_scale_high, intake_arm_up, intake_arm_mid,
			intake_arm_down, mds_up, mds_mid, mds_down, open_intake, close_intake, carr_up,
			carr_mid, carr_down,
			raise_to_scale_backwards, climb_button, is_heading);
	Thread.detach();

}

void TaskManager::ThreadWrapper(TaskManager *task_manager, Joystick *JoyThrottle,
				Joystick *JoyWheel, Joystick *JoySlider, bool *wait_for_button, bool *intake_spin_in,
				bool *intake_spin_out, bool *intake_spin_slow, bool *intake_spin_med, bool *intake_spin_stop, bool *get_cube_ground,
				bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *pop_switch, bool *raise_to_scale_low, bool *raise_to_scale_mid, bool *raise_to_scale_high,
				bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *mds_up, bool *mds_mid, bool *mds_down, bool *open_intake, bool *close_intake, bool *carr_up,
				bool *carr_mid, bool *carr_down, bool *raise_to_scale_backwards, bool *climb_button, bool *is_heading) {

	threadTimer->Start();

	while (true) {

		threadTimer->Reset();

		if (frc::RobotState::IsEnabled() && frc::RobotState::IsAutonomous() && drive_controller->set_profile) { //this is always running, even during auton init

			intake_t->Rotate();
			mds_t->Move();
			carr_t->Move();

			drive_controller->RunAutonDrive();

			intake_t->IntakeArmStateMachine();
			intake_t->IntakeWheelStateMachine();
			mds_t->ElevatorStateMachine();
			carr_t->ElevatorStateMachine();
			intake_t->IntakeSolenoidStateMachine();

			auton_state_machine->StateMachineAuton((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch,
					(bool) *raise_to_scale_mid, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *mds_up, (bool) *mds_mid, (bool) *mds_down, (bool) *open_intake, (bool) *close_intake, (bool) *carr_up,
					(bool) *carr_mid, (bool) *carr_down, (bool) *raise_to_scale_backwards);

		} else if (frc::RobotState::IsEnabled()
				&& frc::RobotState::IsOperatorControl()) {

			intake_t->Rotate();
			mds_t->Move();
			carr_t->Move();
			//TODO: how to put climber in without being expensive with time

			drive_controller->RunTeleopDrive(JoyThrottle, JoyWheel, (bool)*is_heading);

			intake_t->IntakeArmStateMachine();
			intake_t->IntakeWheelStateMachine();
			mds_t->ElevatorStateMachine();
			carr_t->ElevatorStateMachine();
			intake_t->IntakeSolenoidStateMachine();
			climb_t->ElevatorStateMachine();

			teleop_state_machine->StateMachine((bool) *wait_for_button,
					(bool) *intake_spin_in, (bool) *intake_spin_out,
					(bool) *intake_spin_slow, (bool) *intake_spin_med, (bool) *intake_spin_stop,
					(bool) *get_cube_ground, (bool) *get_cube_station,
					(bool) *post_intake, (bool) *raise_to_switch, (bool) *pop_switch,
					(bool) *raise_to_scale_low, (bool) *raise_to_scale_mid, (bool) *raise_to_scale_high, (bool) *intake_arm_up,
					(bool) *intake_arm_mid, (bool) *intake_arm_down,
					(bool) *mds_up, (bool) *mds_mid, (bool) *mds_down, (bool) *open_intake, (bool) *close_intake, (bool) *carr_up,
					(bool) *carr_mid, (bool) *carr_down, (bool) *raise_to_scale_backwards, (bool) *climb_button, JoySlider);

		}

		double wait_time = task_manager->thread_time_step - threadTimer->Get(); //time step also needs to be changed in the motion profiler parameter

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
