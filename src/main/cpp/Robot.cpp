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

#include "MiddleStage.h"
#include "Carriage.h"

#define STATEMACHINE 1
#define CORNELIUS 1 //in every class
#define BUTTONBOX 1
#define TESTING 1

class Robot: public frc::IterativeRobot {
public:

	double TIME_STEP = 0.02; //not consolidated

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;
	const int JOY_OP = 2;
	const int JOY_SLIDER = 3;

	const int LOW_GEAR_BUTTON = 5;
	const int HIGH_GEAR_BUTTON = 6;
	const int HEADING_BUTTON = 9;

	const int WAIT_FOR_BUTTON = 13;

	const int GET_CUBE_GROUND = 14;
	const int GET_CUBE_STATION = 99;
	//const int POST_INTAKE = 5; //taken for middle shot
	const int RAISE_TO_SWITCH = 6;
	const int POP_SWITCH = 2;
	const int RAISE_TO_SCALE_MID = 1;
	const int RAISE_TO_SCALE_LOW = 5;
	const int RAISE_TO_SCALE_HIGH = 4;
	const int RAISE_TO_SCALE_BACKWARDS = 12;

	const int INTAKE_SPIN_IN = 10; //not enough buttons for these three
	const int INTAKE_SPIN_OUT = 1; //throttle
	const int INTAKE_SPIN_STOP = 99;
	const int INTAKE_SPIN_SLOW = 8;
	const int INTAKE_SPIN_MED = 99;

	const int INTAKE_ARM_UP = 8;
	const int INTAKE_ARM_MID = 99; //TAKEN OUT
	const int INTAKE_ARM_DOWN = 15;
	const int INTAKE_ARM_BACKWARDS = 99; //no manual for this one

	const int MDS_UP = 3;
	const int MDS_MID = 16;
	const int MDS_DOWN = 9;

	const int OPEN_INTAKE = -1;
	const int CLOSE_INTAKE = -1;

	const int CARR_UP = 0;
	const int CARR_MID = 33;
	const int CARR_DOWN = 56;

	bool wait_for_button, intake_spin_in, intake_spin_out, intake_spin_slow,
			intake_spin_med, intake_spin_stop, get_cube_ground,
			get_cube_station, post_intake, raise_to_switch, pop_switch,
			raise_to_scale_low, raise_to_scale_mid, raise_to_scale_high,
			intake_arm_up, intake_arm_mid, intake_arm_down, mds_up,
				mds_mid, mds_down, open_intake, close_intake, raise_to_scale_backwards, carr_down,
				carr_mid, carr_up, climb_button; //for BOTH state machines

	bool is_heading, is_vision, is_fc; //drive
	bool is_auto_shift;

	bool left_scale, left_switch;

	PowerDistributionPanel *pdp_;
  MiddleStage *mds_;
	Carriage *carr_;
	ElevatorMotionProfiler *elevator_profiler_;

	frc::SendableChooser<std::string> autonChooser;

	//first letter indicates starting position: Left, Right, Center, Side (Left or Right)
	const std::string sideDriveForward = "S DriveForward";
	const std::string centerDriveForward = "C DriveForward";
//	const std::string leftCubeSwitch = "L Switch";
//	const std::string rightCubeSwitch = "R Switch";
	const std::string centerCubeSwitch = "C Switch";
	const std::string leftCubeScale = "L Scale";
	const std::string rightCubeScale = "R Scale";
	const std::string leftScaleSwitch = "L Scale-Switch";
	const std::string rightScaleSwitch = "R Scale-Switch";
	const std::string doNothing = "Do Nothing";
//	const std::string centerCubeSwitchSwitch = "C 2-Switch";
	const std::string leftCubeScaleScale = "L Scale-Scale";
	const std::string rightCubeScaleScale = "R Scale-Scale";

	bool leftSwitch, leftScale;

	bool switchCenterOneState, sameScaleTwoState, oppScaleTwoState,
			sameScaleSwitchState, oppScaleSwitchState, //can do opp scale switch! make happen!
			sameScaleOneState, oppScaleOneState; //switchSideState, switchSwitchState, switchCenterTwoState,

	Joystick *joyOp;

	std::string autoSelected;

	std::string positionSelected;

	int state_test = 0;
	int last_state_test = 1;

	Timer *timerTest = new Timer();

	void RobotInit() {

		// SmartDashboard::PutNumber("targetHeading", 0);
		// SmartDashboard::PutNumber("Actual Heading", 0);
		// SmartDashboard::PutNumber("refYaw", 0);
		//
		// SmartDashboard::PutNumber("refLeft", 0);
		// SmartDashboard::PutNumber("refRight", 0);
		// SmartDashboard::PutNumber("actualLeftDis", 0);
		// SmartDashboard::PutNumber("actualRightDis", 0);
		//
		// SmartDashboard::PutNumber("refLeftVel", 0);
		// SmartDashboard::PutNumber("actualLeftVel", 0);
		//
		// SmartDashboard::PutNumber("Left 1", 0);
		// SmartDashboard::PutNumber("Left 2", 0);
		// SmartDashboard::PutNumber("Left 3", 0);
		// SmartDashboard::PutNumber("Left 4", 0);
		//
		// SmartDashboard::PutNumber("Right 1", 0);
		// SmartDashboard::PutNumber("Right 2", 0);
		// SmartDashboard::PutNumber("Right 3", 0);
		// SmartDashboard::PutNumber("Right 4", 0);
		//
		// SmartDashboard::PutNumber("Encoder Left", 0);
		// SmartDashboard::PutNumber("Encoder Right", 0);
		elevator_profiler_ = new ElevatorMotionProfiler(1.15, 5.0, TIME_STEP); //max vel, max accel, timestep

		pdp_ = new PowerDistributionPanel(3);

		carr_ = new Carriage(elevator_profiler_);
		mds_ = new MiddleStage(elevator_profiler_);

		joyOp = new Joystick(JOY_OP);
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {

		mds_up = joyOp->GetRawButton(MDS_UP);
		mds_mid = joyOp->GetRawButton(MDS_MID);
		mds_down = joyOp->GetRawButton(MDS_DOWN);
 		carr_up = joyOp->GetRawButton(CARR_UP);
		carr_mid = joyOp->GetRawButton(CARR_MID);
		carr_down = joyOp->GetRawButton(CARR_DOWN);

		if (mds_up) {

		} else if (mds_mid) {

		} else if (mds_down) {

		}

		if (carr_up) {

		} else if (carr_down) {

		} else if (carr_mid) {

		}


	}

	void DisabledInit() override { //between auton and teleop

	}

	void TestPeriodic() {
		//

//		drive_controller->canTalonLeft1->Set(ControlMode::PercentOutput, 1.0);
//		drive_controller->canTalonRight1->Set(ControlMode::PercentOutput, 1.0);
//
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
//
//		SmartDashboard::PutNumber("Left 1 Vel", drive_controller->GetLeftVel());
//		SmartDashboard::PutNumber("Right 1 Vel",
//				drive_controller->GetRightVel());


	} //arm up, elev down

private:

}
;

START_ROBOT_CLASS(Robot)
