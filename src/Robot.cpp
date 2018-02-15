/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <DriveController.h>
#include <Intake.h>
#include <Elevator.h>
#include <Autonomous.h>
#include <Joystick.h>
#include <TeleopStateMachine.h>

#define THREADS 0
#define STATEMACHINE 1

class Robot: public frc::IterativeRobot {
public:

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;
	const int JOY_OP = 2;

	const int LOW_GEAR_BUTTON = 10;
	const int HIGH_GEAR_BUTTON = 11;

	const int WAIT_FOR_BUTTON = 1;
	const int GET_CUBE_GROUND = 2;
	const int GET_CUBE_STATION = 3;
	const int POST_INTAKE = 4;
	const int RAISE_TO_SWITCH = 5;
	const int RAISE_TO_SCALE = 6;

	const int INTAKE_SPIN_IN = 6;
	const int INTAKE_SPIN_OUT = 7;

	const int INTAKE_SPIN_STOP = 8;
	const int INTAKE_ARM_UP = 3;
	const int INTAKE_ARM_MID = 4;
	const int INTAKE_ARM_DOWN = 5;
	const int ELEVATOR_UP = 9;
	const int ELEVATOR_MID = 10;

	const int ELEVATOR_DOWN = 11;

	bool acceptable_current_r, acceptable_current_l; //testperiodic

	bool is_heading, is_vision, is_fc;

	DriveController *drive_controller;
	PowerDistributionPanel *pdp_;
	Elevator *elevator_;
	Intake *intake_;
	Autonomous *autonomous_;
	TeleopStateMachine *teleop_state_machine;
	ElevatorMotionProfiler *elevator_profiler_;
	IntakeMotionProfiler *intake_profiler_;

	Joystick *joyThrottle, *joyWheel, *joyOp;

//	DigitalInput *wait_for_button, *intake_spin_in,
//	*intake_spin_out, *intake_spin_stop, *get_cube_ground,
//	*get_cube_station, *post_intake, *raise_to_switch, *raise_to_scale,
//	*intake_arm_up, *intake_arm_mid, *intake_arm_down, *elevator_up,
//	*elevator_mid, *elevator_down;

	void RobotInit() {

		elevator_profiler_ = new ElevatorMotionProfiler(0.0, 0.0, 0.0001); //will be set in intake and elevator classes for now
		intake_profiler_ = new IntakeMotionProfiler(0.0, 0.0, 0.0001);

		pdp_ = new PowerDistributionPanel(3);
		drive_controller = new DriveController();
		intake_ = new Intake(pdp_, intake_profiler_);
		elevator_ = new Elevator(pdp_, elevator_profiler_);
		autonomous_ = new Autonomous(drive_controller, elevator_, intake_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_);

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);
		joyOp = new Joystick(JOY_OP);

	}

	void AutonomousInit() override {

		//start auton threads

//		drive_controller->ZeroI(true);
//		drive_controller->ZeroEncs();
//		drive_controller->ZeroYaw();
//
//		elevator_->ZeroEncs();
//		intake_->ZeroEnc();
//
//		autonomous_->FillProfile("");

	}

	void AutonomousPeriodic() {

//		elevator_->ElevatorStateMachine();
//		intake_->IntakeArmStateMachine();
//		intake_->IntakeWheelStateMachine();
//
//		autonomous_->RunAuton();

	}

	void TeleopInit() {

		//disable auton threads

//		drive_controller->ZeroI(true);
//		drive_controller->ZeroEncs();
//		drive_controller->ZeroYaw();

		elevator_->zeroing_counter_e = 0;
		intake_->zeroing_counter_i = 0;

		intake_->is_init_intake = false;
		elevator_->is_elevator_init = false;

		elevator_->ZeroEncs();
		intake_->ZeroEnc();

		teleop_state_machine->Initialize();

//#ifndef THREADS
//		drive_controller->StartTeleopThreads(joyThrottle, joyWheel, &is_heading,
//				&is_vision, &is_fc); //pass by reference through the wrapper
		intake_->StartIntakeThread();
		elevator_->StartElevatorThread();
//#endif

	}

	void TeleopPeriodic() {

#if !STATEMACHINE
		intake_->ManualArm(joyOp);
		//	intake_->ManualWheels(joyOp);
		elevator_->ManualElevator(joyThrottle);

#else

		bool low_gear = joyThrottle->GetRawButton(LOW_GEAR_BUTTON);
		bool high_gear = joyThrottle->GetRawButton(HIGH_GEAR_BUTTON);


		bool wait_for_button = joyThrottle->GetRawButton(WAIT_FOR_BUTTON); //testing
		bool get_cube_ground = joyThrottle->GetRawButton(GET_CUBE_GROUND);
		bool get_cube_station = joyThrottle->GetRawButton(GET_CUBE_STATION);
		bool post_intake = joyThrottle->GetRawButton(POST_INTAKE);
		bool raise_to_switch = joyThrottle->GetRawButton(RAISE_TO_SWITCH);
		bool raise_to_scale = joyThrottle->GetRawButton(RAISE_TO_SCALE);

		bool intake_spin_in = joyOp->GetRawButton(INTAKE_SPIN_IN);
		bool intake_spin_out = joyOp->GetRawButton(INTAKE_SPIN_OUT);
		bool intake_spin_stop = joyOp->GetRawButton(INTAKE_SPIN_STOP);
		bool intake_arm_up = joyOp->GetRawButton(INTAKE_ARM_UP);
		bool intake_arm_mid = joyOp->GetRawButton(INTAKE_ARM_MID);
		bool intake_arm_down = joyOp->GetRawButton(INTAKE_ARM_DOWN);
		bool elevator_up = joyOp->GetRawButton(ELEVATOR_UP);
		bool elevator_mid = joyOp->GetRawButton(ELEVATOR_MID);
		bool elevator_down = joyOp->GetRawButton(ELEVATOR_DOWN);

		teleop_state_machine->StateMachine(wait_for_button, intake_spin_in,
				intake_spin_out, intake_spin_stop, get_cube_ground,
				get_cube_station, post_intake, raise_to_switch, raise_to_scale,
				intake_arm_up, intake_arm_mid, intake_arm_down, elevator_up,
				elevator_mid, elevator_down);

		elevator_->ElevatorStateMachine();
		intake_->IntakeArmStateMachine();
		intake_->IntakeWheelStateMachine();

		is_heading = false;
		is_vision = false;
		is_fc = false;

		if (low_gear) {
	//		drive_controller->ShiftDown();
		} else if (high_gear) {
		//	drive_controller->ShiftUp();
		}
#endif
	}

	void DisabledInit() override {

		//drive_controller->EndTeleopThreads();
		intake_->EndIntakeThread(); //may not actually disable threads
		elevator_->EndElevatorThread();

		teleop_state_machine->Initialize();

		intake_->is_init_intake = false;
		elevator_->is_elevator_init = false;

		elevator_->zeroing_counter_e = 0;
		intake_->zeroing_counter_i = 0;

		elevator_->ZeroEncs(); //counter zeroing?
		intake_->ZeroEnc();

	}

	void TestPeriodic() {

		/* Standard dev
		 double sumL = 0;
		 double sumR = 0;
		 double meanR = 0;
		 double meanL = 0;

		 double standard_dev_l = 0;
		 double standard_dev_r = 0;

		 drive_controller->canTalonLeft1->Set(ControlMode::PercentOutput, 1.0);
		 drive_controller->canTalonRight1->Set(ControlMode::PercentOutput, 1.0);

		 SmartDashboard::PutNumber("Left 1",
		 drive_controller->canTalonLeft1->GetOutputCurrent());
		 SmartDashboard::PutNumber("Left 2",
		 drive_controller->canTalonLeft2->GetOutputCurrent());
		 SmartDashboard::PutNumber("Left 3",
		 drive_controller->canTalonLeft3->GetOutputCurrent());
		 SmartDashboard::PutNumber("Left 4",
		 drive_controller->canTalonLeft4->GetOutputCurrent());
		 SmartDashboard::PutNumber("Right 1",
		 drive_controller->canTalonRight1->GetOutputCurrent());
		 SmartDashboard::PutNumber("Right 2",
		 drive_controller->canTalonRight2->GetOutputCurrent());
		 SmartDashboard::PutNumber("Right 3",
		 drive_controller->canTalonRight3->GetOutputCurrent());
		 SmartDashboard::PutNumber("Right 4",
		 drive_controller->canTalonRight4->GetOutputCurrent());

		 //standard dev calculation

		 sumL += drive_controller->canTalonLeft1->GetOutputCurrent();
		 sumL += drive_controller->canTalonLeft2->GetOutputCurrent();
		 sumL += drive_controller->canTalonLeft3->GetOutputCurrent();
		 sumL += drive_controller->canTalonLeft4->GetOutputCurrent();
		 sumR += drive_controller->canTalonRight1->GetOutputCurrent();
		 sumR += drive_controller->canTalonRight2->GetOutputCurrent();
		 sumR += drive_controller->canTalonRight3->GetOutputCurrent();
		 sumR += drive_controller->canTalonRight4->GetOutputCurrent();

		 meanL = sumL / 8.0;
		 meanR = sumR / 8.0;

		 standard_dev_l += pow(
		 drive_controller->canTalonLeft1->GetOutputCurrent() - meanL, 2);
		 standard_dev_l += pow(
		 drive_controller->canTalonLeft2->GetOutputCurrent() - meanL, 2);
		 standard_dev_l += pow(
		 drive_controller->canTalonLeft3->GetOutputCurrent() - meanL, 2);
		 standard_dev_l += pow(
		 drive_controller->canTalonLeft4->GetOutputCurrent() - meanL, 2);
		 standard_dev_r += pow(
		 drive_controller->canTalonRight1->GetOutputCurrent() - meanR,
		 2);
		 standard_dev_r += pow(
		 drive_controller->canTalonRight2->GetOutputCurrent() - meanR,
		 2);
		 standard_dev_r += pow(
		 drive_controller->canTalonRight3->GetOutputCurrent() - meanR,
		 2);
		 standard_dev_r += pow(
		 drive_controller->canTalonRight4->GetOutputCurrent() - meanR,
		 2);

		 standard_dev_l = sqrt(standard_dev_l / 10.0);
		 standard_dev_r = sqrt(standard_dev_r / 10.0);

		 SmartDashboard::PutNumber("Standard Dev Left", standard_dev_l);
		 SmartDashboard::PutNumber("Standard Dev Right", standard_dev_r);

		 if (standard_dev_l > 1.0) {
		 acceptable_current_l = false;
		 } else {
		 acceptable_current_l = true;
		 }

		 if (standard_dev_r > 1.0) {
		 acceptable_current_r = false;
		 } else {
		 acceptable_current_r = true;
		 }

		 SmartDashboard::PutBoolean("Close Currents ", acceptable_current_l);
		 SmartDashboard::PutBoolean("Close Currents ", acceptable_current_r);
		 */

	}

private:

};

START_ROBOT_CLASS(Robot)
