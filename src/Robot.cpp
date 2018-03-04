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
#include <Joystick.h>

#include <Autonomous.h>
#include <DriveController.h>
#include <Intake.h>
#include <Elevator.h>
#include <AutonSequences/DriveForward.h>
#include <AutonSequences/Switch.h>
#include <TeleopStateMachine.h>

#define THREADS 1
#define STATEMACHINE 1
#define CORNELIUS 1 //in every class

class Robot: public frc::IterativeRobot {
public:

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;
	const int JOY_OP = 2;

	const int LOW_GEAR_BUTTON = 6;
	const int HIGH_GEAR_BUTTON = 9;

	const int WAIT_FOR_BUTTON = 1;
	const int GET_CUBE_GROUND = 2;
	const int GET_CUBE_STATION = 3;
	const int POST_INTAKE = 4;
	const int RAISE_TO_SWITCH = 5;
	const int RAISE_TO_SCALE = 6;
	const int RAISE_TO_SCALE_BACKWARDS = 3;

	const int INTAKE_SPIN_IN = 9; //THROTTLE
	const int INTAKE_SPIN_OUT = 10;
	const int INTAKE_SPIN_STOP = 11;

	const int INTAKE_ARM_UP = 7;
	const int INTAKE_ARM_MID = 8;
	const int INTAKE_ARM_DOWN = 9;

	const int ELEVATOR_UP = 10;
	const int ELEVATOR_MID = 11;
	const int ELEVATOR_DOWN = 12;

	bool wait_for_button, intake_spin_in, intake_spin_out, intake_spin_stop,
			get_cube_ground, get_cube_station, post_intake, raise_to_switch,
			raise_to_scale, intake_arm_up, intake_arm_mid, intake_arm_down,
			elevator_up, elevator_mid, elevator_down, raise_to_scale_backwards; //BOTH state machines

	bool is_heading, is_vision, is_fc; //drive

	bool left_scale, left_switch;

	DriveController *drive_controller;
	PowerDistributionPanel *pdp_;
	Elevator *elevator_;
	Intake *intake_;
	TeleopStateMachine *teleop_state_machine;
	ElevatorMotionProfiler *elevator_profiler_;
	IntakeMotionProfiler *intake_profiler_;
	Compressor *compressor_;
	Joystick *joyThrottle, *joyWheel, *joyOp;

	DriveForward *drive_forward;
	Switch *switch_;

	frc::SendableChooser<std::string> autonChooser;

	const std::string driveForward = "Drive Forward";
	const std::string cubeSwitch = "Switch";
	const std::string cubeScale = "Scale";

	std::string autoSelected;

//	DigitalInput *wait_for_button, *intake_spin_in,
//	*intake_spin_out, *intake_spin_stop, *get_cube_ground,
//	*get_cube_station, *post_intake, *raise_to_switch, *raise_to_scale,
//	*intake_arm_up, *intake_arm_mid, *intake_arm_down, *elevator_up,
//	*elevator_mid, *elevator_down;

	void RobotInit() {

		elevator_profiler_ = new ElevatorMotionProfiler(1.6, 10.0, 0.01);
		intake_profiler_ = new IntakeMotionProfiler(2.0, 8.0, 0.01);

		compressor_ = new Compressor(3);
		pdp_ = new PowerDistributionPanel(3);

		drive_controller = new DriveController(); //inherits from mother class
		elevator_ = new Elevator(pdp_, elevator_profiler_);
		intake_ = new Intake(pdp_, intake_profiler_, elevator_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_);

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);
		joyOp = new Joystick(JOY_OP);

		autonChooser.AddDefault(driveForward, driveForward);
		autonChooser.AddObject(cubeSwitch, cubeSwitch);
		autonChooser.AddObject(cubeScale, cubeScale);

		frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

#if THREADS
		//starting threads in robot init so that they only are created once
		drive_controller->StartDriveThreads(joyThrottle, joyWheel, &is_heading, //both auton and teleop drive
				&is_vision, &is_fc); //auton drive will not start until profile for auton is sent through

		intake_->StartIntakeThread(); //controllers
		elevator_->StartElevatorThread();

		teleop_state_machine->StartStateMachineThread(&wait_for_button, //both auton and teleop state machines
				&intake_spin_in, &intake_spin_out, &intake_spin_stop,
				&get_cube_ground, &get_cube_station, &post_intake,
				&raise_to_switch, &raise_to_scale, &intake_arm_up,
				&intake_arm_mid, &intake_arm_down, &elevator_up, &elevator_mid,
				&elevator_down, &raise_to_scale_backwards);

#else
#endif
	}

	void AutonomousInit() override {

		autoSelected = autonChooser.GetSelected();

		teleop_state_machine->Initialize();
		compressor_->Stop();

		drive_controller->ZeroAll(true);
		drive_controller->ShiftUp(); //for now

		if (autoSelected == driveForward) {
			drive_forward = new DriveForward(drive_controller, elevator_,
					intake_);
			drive_forward->GenerateForward();
		} else if (autoSelected == cubeSwitch) {
			switch_ = new Switch(drive_controller, elevator_, intake_);
			switch_->GenerateSwitch(true);

		} else if (autoSelected == cubeScale) {

		}

//		if (autoSelected == driveForward) {
//			//drive_forward->Generate();
//		} else if (autoSelected == cube_switch && left_switch) {
//
//		} else if (autoSelected == cube_switch && !left_switch) {
//
//		} else if (autoSelected == cube_scale && left_scale) {
//
//		} else if (autoSelected == cube_scale && !left_scale) {
//
//		}

	}

	void AutonomousPeriodic() {

		//drive thread, auton state machine thread

		if (autoSelected == driveForward) {

		} else if (autoSelected == cubeSwitch) {
			switch_->RunStateMachine(&raise_to_switch);

		} else if (autoSelected == cubeScale) {

		}
	}

	void TeleopInit() {

		compressor_->SetClosedLoopControl(true);
		teleop_state_machine->Initialize();
		drive_controller->ZeroAll(true);
		drive_controller->ShiftUp();

	}

	void TeleopPeriodic() {

#if !STATEMACHINE
		intake_->ManualArm(joyOp);
		//	intake_->ManualWheels(joyOp);
		elevator_->ManualElevator(joyThrottle);

#else

		bool low_gear = joyWheel->GetRawButton(LOW_GEAR_BUTTON);
		bool high_gear = joyWheel->GetRawButton(HIGH_GEAR_BUTTON);

		wait_for_button = joyOp->GetRawButton(WAIT_FOR_BUTTON);
		get_cube_ground = joyOp->GetRawButton(GET_CUBE_GROUND);
		get_cube_station = joyOp->GetRawButton(GET_CUBE_STATION);

		post_intake = joyOp->GetRawButton(POST_INTAKE);
		raise_to_switch = joyOp->GetRawButton(RAISE_TO_SWITCH);
		raise_to_scale = joyOp->GetRawButton(RAISE_TO_SCALE);

		raise_to_scale_backwards = joyThrottle->GetRawButton(RAISE_TO_SCALE_BACKWARDS);

		intake_spin_in = joyThrottle->GetRawButton(INTAKE_SPIN_IN);
		intake_spin_out = joyThrottle->GetRawButton(INTAKE_SPIN_OUT);
		intake_spin_stop = joyThrottle->GetRawButton(INTAKE_SPIN_STOP);

		intake_arm_up = joyOp->GetRawButton(INTAKE_ARM_UP);
		intake_arm_mid = joyOp->GetRawButton(INTAKE_ARM_MID);
		intake_arm_down = joyOp->GetRawButton(INTAKE_ARM_DOWN);
		elevator_up = joyOp->GetRawButton(ELEVATOR_UP);
		elevator_mid = joyOp->GetRawButton(ELEVATOR_MID);
		elevator_down = joyOp->GetRawButton(ELEVATOR_DOWN);

//		SmartDashboard::PutNumber("Left 1",
//				drive_controller->canTalonLeft1->GetOutputCurrent());
//		SmartDashboard::PutNumber("Left 2",
//				drive_controller->canTalonLeft2->GetOutputCurrent());
//		SmartDashboard::PutNumber("Left 3",
//				drive_controller->canTalonLeft3->GetOutputCurrent());
//		SmartDashboard::PutNumber("Left 4",
//				drive_controller->canTalonLeft4->GetOutputCurrent());
//		SmartDashboard::PutNumber("Right 1",
//				drive_controller->canTalonRight1->GetOutputCurrent());
//		SmartDashboard::PutNumber("Right 2",
//				drive_controller->canTalonRight2->GetOutputCurrent());
//		SmartDashboard::PutNumber("Right 3",
//				drive_controller->canTalonRight3->GetOutputCurrent());
//		SmartDashboard::PutNumber("Right 4",
//				drive_controller->canTalonRight4->GetOutputCurrent());

		is_heading = false;
		is_vision = false;
		is_fc = false;

		//	drive_controller->AutoShift();
//
//		if (low_gear) {
//			drive_controller->ShiftDown();
//		} else if (high_gear) {
//			drive_controller->ShiftUp();
//		}
#endif
	}

	void DisabledInit() override {

		teleop_state_machine->EndStateMachineThread();
		drive_controller->EndDriveThreads();
		intake_->EndIntakeThread(); //may not actually disable threads
		elevator_->EndElevatorThread();

		teleop_state_machine->Initialize(); //17%

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
