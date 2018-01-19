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
#include <MotionProfiling.h>
#include <Joystick.h>
#include <TeleopStateMachine.h>

class Robot: public frc::IterativeRobot { //MARTDASHOBOARD
public:

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;
	const int JOY_OP = 2;

	const int LOW_GEAR_BUTTON = 3;
	const int HIGH_GEAR_BUTTON = 4;

	bool is_heading, is_vision, is_fc;

	DriveController *drive_controller;
	Elevator *elevator_;
	Intake *intake_;
	Autonomous *autonomous_;
	TeleopStateMachine *teleop_state_machine;

	double sumL = 0;
	double sumR = 0;
	double meanR = 0;
	double meanL = 0;

	double standard_dev_l = 0;
	double standard_dev_r = 0;
	bool acceptable_current_l = true;
	bool acceptable_current_r = true;

	Joystick *joyThrottle, *joyWheel, *joyOp;

	void RobotInit() {

		drive_controller = new DriveController();
		elevator_ = new Elevator();
		intake_ = new Intake();
		autonomous_ = new Autonomous(drive_controller, elevator_, intake_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_);

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);
		joyOp = new Joystick(JOY_OP);


	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

		drive_controller->ZeroI(true);
		drive_controller->ZeroEncs();
		drive_controller->ZeroYaw();

		drive_controller->StartTeleopThreads(joyThrottle, joyWheel, &is_heading,
				&is_vision, &is_fc);

	}

	void TeleopPeriodic() {

/*		SmartDashboard::PutNumber("Left 1", drive_controller->canTalonLeft1->GetOutputCurrent());
		SmartDashboard::PutNumber("Left 2", drive_controller->canTalonLeft2->GetOutputCurrent());
		SmartDashboard::PutNumber("Left 3", drive_controller->canTalonLeft3->GetOutputCurrent());
		SmartDashboard::PutNumber("Left 4", drive_controller->canTalonLeft4->GetOutputCurrent());
		SmartDashboard::PutNumber("Right 1", drive_controller->canTalonRight1->GetOutputCurrent());
		SmartDashboard::PutNumber("Right 2", drive_controller->canTalonRight2->GetOutputCurrent());
		SmartDashboard::PutNumber("Right 3", drive_controller->canTalonRight3->GetOutputCurrent());
		SmartDashboard::PutNumber("Right 4", drive_controller->canTalonRight4->GetOutputCurrent());

		//standard dev calculation

		sumL = 0;
		sumR = 0;
		meanL = 0;
		meanR = 0;
		standard_dev_l = 0;
		standard_dev_r = 0;

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

		standard_dev_l += pow(drive_controller->canTalonLeft1->GetOutputCurrent() - meanL, 2);
		standard_dev_l += pow(drive_controller->canTalonLeft2->GetOutputCurrent() - meanL, 2);
		standard_dev_l += pow(drive_controller->canTalonLeft3->GetOutputCurrent() - meanL, 2);
		standard_dev_l += pow(drive_controller->canTalonLeft4->GetOutputCurrent() - meanL, 2);
		standard_dev_r += pow(drive_controller->canTalonRight1->GetOutputCurrent() - meanR, 2);
		standard_dev_r += pow(drive_controller->canTalonRight2->GetOutputCurrent() - meanR, 2);
		standard_dev_r += pow(drive_controller->canTalonRight3->GetOutputCurrent() - meanR, 2);
		standard_dev_r += pow(drive_controller->canTalonRight4->GetOutputCurrent() - meanR, 2);

		standard_dev_l = sqrt(standard_dev_l / 10.0);
		standard_dev_r = sqrt(standard_dev_r / 10.0);

		//end standard dev calculation

		SmartDashboard::PutNumber("Standard Dev Left", standard_dev_l);
		SmartDashboard::PutNumber("Standard Dev Right", standard_dev_r);

		if(standard_dev_l > 1.0) {
			acceptable_current_l = false;
		}
		else {
			acceptable_current_l = true;
		}

		if(standard_dev_r > 1.0) {
			acceptable_current_r = false;
		}
		else {
			acceptable_current_r = true;
		} */

	///	SmartDashboard::PutBoolean("Close Currents ", acceptable_current);

		is_heading = false;
		is_vision = false;
		is_fc = false;

		bool low_gear = joyOp->GetRawButton(LOW_GEAR_BUTTON);
		bool high_gear = joyOp->GetRawButton(HIGH_GEAR_BUTTON);

		if(low_gear) {
			drive_controller->ShiftDown();
		}
		else if (high_gear) {
			drive_controller->ShiftUp();
		}

	}

	void DisabledInit() override {

		drive_controller->EndTeleopThreads();

	}

	void TestPeriodic() {


	}

private:

};

START_ROBOT_CLASS(Robot)
