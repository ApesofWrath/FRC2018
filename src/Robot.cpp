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
#include <AutonSequences/Scale.h>
#include <TeleopStateMachine.h>

#define THREADS 1
#define STATEMACHINE 1
#define CORNELIUS 1 //in every class
#define BUTTONBOX 1

class Robot: public frc::IterativeRobot {
public:

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;
	const int JOY_OP = 2;

	const int LOW_GEAR_BUTTON = 6;
	const int HIGH_GEAR_BUTTON = 9;

#if BUTTONBOX

	const int WAIT_FOR_BUTTON = 13;

	const int GET_CUBE_GROUND = 14;
	const int GET_CUBE_STATION = 4;
	const int POST_INTAKE = 5;
	const int RAISE_TO_SWITCH = 6;
	const int RAISE_TO_SCALE = 1;
	const int RAISE_TO_SCALE_BACKWARDS = 12;

	const int INTAKE_SPIN_IN = 99; //not enough buttons for these three
	const int INTAKE_SPIN_OUT = 99;
	const int INTAKE_SPIN_STOP = 99;
	//no slow

	const int INTAKE_ARM_UP = 8;
	const int INTAKE_ARM_MID = 2;
	const int INTAKE_ARM_DOWN = 15;
	const int INTAKE_ARM_BACKWARDS = 4; //no manual for this one

	const int ELEVATOR_UP = 3;
	const int ELEVATOR_MID = 16;
	const int ELEVATOR_DOWN = 9;
	//no human player station height

#else

	const int WAIT_FOR_BUTTON = 1;

	const int GET_CUBE_GROUND = 2;
	const int GET_CUBE_STATION = 3;
	const int POST_INTAKE = 4;
	const int RAISE_TO_SWITCH = 5;
	const int RAISE_TO_SCALE = 6;
	const int RAISE_TO_SCALE_BACKWARDS = 3; //THROTTLE

	const int INTAKE_SPIN_IN = 9;//THROTTLE
	const int INTAKE_SPIN_OUT = 10;//THROTTLE
	const int INTAKE_SPIN_STOP = 11;//THROTTLE
	//no slow

	const int INTAKE_ARM_UP = 7;
	const int INTAKE_ARM_MID = 8;
	const int INTAKE_ARM_DOWN = 9;
	const int INTAKE_ARM_BACKWARDS = 4;//THROTTLE

	const int ELEVATOR_UP = 10;
	const int ELEVATOR_MID = 11;
	const int ELEVATOR_DOWN = 12;
	//no human player station height

#endif

	bool wait_for_button, intake_spin_in, intake_spin_out, intake_spin_stop,
			get_cube_ground, get_cube_station, post_intake, raise_to_switch,
			raise_to_scale, intake_arm_up, intake_arm_mid, intake_arm_down,
			elevator_up, elevator_mid, elevator_down, raise_to_scale_backwards; //BOTH state machines

	bool is_heading, is_vision, is_fc; //drive
	bool is_auto_shift;

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
	Scale *scale_;

	frc::SendableChooser<std::string> autonChooser;
	frc::SendableChooser<std::string> positionChooser;

	const std::string driveForward = "Drive Forward";
	const std::string cubeSwitch = "Switch";
	const std::string cubeScale = "Scale";
	const std::string scaleSwit = "Scale & Switch";

	const std::string center = "Center";
	const std::string left = "Left";
	const std::string right = "Right";

	bool leftSwitch, leftScale;

	bool switchState, scaleState;

	std::string autoSelected;
	std::string positionSelected;

	void RobotInit() {

		elevator_profiler_ = new ElevatorMotionProfiler(1.6, 10.0, 0.01); //max vel, max accel, timestep
		intake_profiler_ = new IntakeMotionProfiler(2.0, 8.0, 0.01);

		compressor_ = new Compressor(3);
		pdp_ = new PowerDistributionPanel(3);

		drive_controller = new DriveController(); //inherits from mother class
		elevator_ = new Elevator(pdp_, elevator_profiler_);
		intake_ = new Intake(pdp_, intake_profiler_, elevator_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_,
				drive_controller); //actually has both state machines

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);
		joyOp = new Joystick(JOY_OP);

		autonChooser.AddDefault(driveForward, driveForward);
		autonChooser.AddObject(cubeSwitch, cubeSwitch);
		autonChooser.AddObject(cubeScale, cubeScale);
		autonChooser.AddObject(scaleSwit, scaleSwit);

		positionChooser.AddDefault(center, center);
		positionChooser.AddObject(left, left);
		positionChooser.AddObject(right, right);

		frc::SmartDashboard::PutData("Auto Modes", &autonChooser);
		frc::SmartDashboard::PutData("Position", &positionChooser);

#if THREADS
		//starting threads in robot init so that they only are created once
		drive_controller->StartDriveThreads(joyThrottle, joyWheel, &is_heading, //both auton and teleop drive
				&is_vision, &is_fc); //auton drive will not start until profile for auton is sent through

		intake_->StartIntakeThread(); //controllers
		elevator_->StartElevatorThread();

		teleop_state_machine->StartStateMachineThread(
				&wait_for_button, //both auton and teleop state machines
				&intake_spin_in, &intake_spin_out, &intake_spin_stop,
				&get_cube_ground, &get_cube_station, &post_intake,
				&raise_to_switch, &raise_to_scale, &intake_arm_up,
				&intake_arm_mid, &intake_arm_down, &elevator_up, &elevator_mid,
				&elevator_down, &raise_to_scale_backwards);

#else
#endif
	}

	void AutonomousInit() override {

		teleop_state_machine->Initialize();
		compressor_->Stop(); //not working

		drive_controller->ZeroAll(true);
		drive_controller->ShiftUp();

		std::string gameData = "";

		for (int i = 0; i < 1000; i++) { //FMS data may not come immediately
			gameData =
					frc::DriverStation::GetInstance().GetGameSpecificMessage();
			if (gameData.length() > 0) {
				break; //leave for loop early
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		} //will stop checking after 15 counts, but does not necessarily mean data has been received

		autoSelected = autonChooser.GetSelected();
		positionSelected = positionChooser.GetSelected();

		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				leftSwitch = true;
			} else {
				leftSwitch = false;
			}
			if (gameData[1] == 'L') {
				leftScale = true;
			} else {
				leftScale = false;
			}
		} else { //still have not received FMS data
			autoSelected = driveForward; //regardless of auton chooser
		}

		//auton, position, switch, scale

		//Switch
		if (autoSelected == cubeSwitch) { //has no edge cases
			switch_ = new Switch(drive_controller, elevator_, intake_);
			if (positionSelected == left) {

			} else if (positionSelected == right) {

			} else if (positionSelected == center) {
				switch_->GenerateSwitch(leftSwitch);
				switchState = true;
			}

			//Scale only
		} else if (autoSelected == cubeScale) { //can only scale if scale is on our side //TODO: Add logic for what to do if scale is not on our side
			scale_ = new Scale(drive_controller, elevator_, intake_);

			if (positionSelected == left && leftScale) {
				scale_->GenerateScale(true, false, false);
				scaleState = true;

			} else if (positionSelected == right && !leftScale) {
				scale_->GenerateScale(false, false, false);
				scaleState = true;

			} else if ((positionSelected == center)
					|| (positionSelected == right && leftScale)
					|| (positionSelected == left && !leftScale)) {
				drive_forward = new DriveForward(drive_controller, elevator_, //can't just set autoSelected to driveForward because code only goes through if-ifelse once
						intake_);
				drive_forward->GenerateForward();
			}
		}

		//Scale and switch
		else if (autoSelected == scaleSwit) { //if only scale is on our side, will do scale only //TODO: add the switch only edge case

			scale_ = new Scale(drive_controller, elevator_, intake_);

			if (positionSelected == left) {
				if (leftScale && leftSwitch) { //scale and switch
					scale_->GenerateScale(true, true, true);
				} else if (leftScale && !leftSwitch) { //only scale
					scale_->GenerateScale(true, false, false);
				}
				scaleState = true;

			} else if (positionSelected == right) {
				if (!leftScale && !leftSwitch) {
					scale_->GenerateScale(false, true, false); //add side of switch desired, right for now
				} else if (!leftScale && leftSwitch) {
					scale_->GenerateScale(false, false, false);
				}
				scaleState = true;

			} else {
				drive_forward = new DriveForward(drive_controller, elevator_,
						intake_);
				drive_forward->GenerateForward();
			}
		}

		else if (autoSelected == driveForward) {
			drive_forward = new DriveForward(drive_controller, elevator_,
					intake_);
			drive_forward->GenerateForward();

		} else {

		}

	}

	void AutonomousPeriodic() {

		//drive thread, auton state machine thread

		if (scaleState) {
			scale_->RunStateMachine(&raise_to_scale_backwards, &raise_to_switch,
					&get_cube_ground); //works for both scale only and scale+switch
		} else if (switchState) {
			switch_->RunStateMachine(&raise_to_switch);
		} //else is drive forward

	}

	void TeleopInit() {

		compressor_->SetClosedLoopControl(true);
		//teleop_state_machine->Initialize(); //only initialize in auton state machine
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

		raise_to_scale_backwards = joyThrottle->GetRawButton(
				RAISE_TO_SCALE_BACKWARDS);

		intake_spin_in = joyThrottle->GetRawButton(INTAKE_SPIN_IN);
		intake_spin_out = joyThrottle->GetRawButton(INTAKE_SPIN_OUT);
		intake_spin_stop = joyThrottle->GetRawButton(INTAKE_SPIN_STOP);

		intake_arm_up = joyOp->GetRawButton(INTAKE_ARM_UP);
		intake_arm_mid = joyOp->GetRawButton(INTAKE_ARM_MID);
		intake_arm_down = joyOp->GetRawButton(INTAKE_ARM_DOWN);
		elevator_up = joyOp->GetRawButton(ELEVATOR_UP);
		elevator_mid = joyOp->GetRawButton(ELEVATOR_MID);
		elevator_down = joyOp->GetRawButton(ELEVATOR_DOWN);

		is_heading = false;
		is_vision = false;
		is_fc = false;

		if (low_gear) {
			is_auto_shift = false;
			drive_controller->ShiftDown();
		} else if (high_gear) {
			drive_controller->ShiftUp();
			is_auto_shift = false;
		} else {
			is_auto_shift = true;
		}

		drive_controller->AutoShift(is_auto_shift);

#endif
	}

	void DisabledInit() override { //between auton and teleop

		teleop_state_machine->EndStateMachineThread();
		drive_controller->EndDriveThreads();
		intake_->EndIntakeThread(); //may not actually disable threads
		elevator_->EndElevatorThread();

		//teleop_state_machine->Initialize(); //17%

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
