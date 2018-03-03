/*
 * Autonomous.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include <Elevator.h>
#include <Intake.h>
#include <fstream>
#include <vector>
#include <list>
#include <DriveController.h>
#include <pathfinder.h>
#include <ElevatorMotionProfiler.h>
#include <thread>
#include <chrono>
#include <Timer.h>

class Autonomous {
public:

	bool wait_for_button, intake_spin_in, intake_spin_out, intake_spin_stop,
			get_cube_ground, get_cube_station, post_intake, raise_to_switch,
			raise_to_scale, intake_arm_up, intake_arm_mid, intake_arm_down,
			elevator_up, elevator_mid, elevator_down; //state machine

	Autonomous(DriveController *dc, Elevator *el, Intake *in);
	void AutonStateMachine(bool wait_for_button, bool intake_spin_in,
			bool intake_spin_out, bool intake_spin_stop, bool get_cube_ground,
			bool get_cube_station, bool post_intake, bool raise_to_switch, bool raise_to_scale,
			bool intake_arm_up, bool intake_arm_mid, bool intake_arm_down, bool elevator_up, bool elevator_mid,
			bool elevator_down);

	void FillProfile(std::vector<std::vector<double> > pathfinder_refs);

	void InitializeAuton();
	void StartAutonStateMachineThread(bool *wait_for_button, bool *intake_spin_in,
			bool *intake_spin_out, bool *intake_spin_stop, bool *get_cube_ground,
			bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
			bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up, bool *elevator_mid,
			bool *elevator_down);
	static void AutonStateMachineWrapper(Autonomous *aut, bool *wait_for_button, bool *intake_spin_in,
				bool *intake_spin_out, bool *intake_spin_stop, bool *get_cube_ground,
				bool *get_cube_station, bool *post_intake, bool *raise_to_switch, bool *raise_to_scale,
				bool *intake_arm_up, bool *intake_arm_mid, bool *intake_arm_down, bool *elevator_up, bool *elevator_mid,
				bool *elevator_down);
	void EndAutonStateMachineThread();

	double GetLeftPos();
	double GetRightPos();

};

#endif /* SRC_AUTONOMOUS_H_ */

