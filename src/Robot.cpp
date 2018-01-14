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

class Robot: public frc::IterativeRobot {
public:

	const int JOY_THROTTLE = 0;
	const int JOY_WHEEL = 1;

	bool is_heading, is_vision, is_fc;

	DriveController *drive_controller;
	Elevator *elevator_;
	Intake *intake_;
	Autonomous *autonomous_;
	TeleopStateMachine *teleop_state_machine;

	Joystick *joyThrottle, *joyWheel;

	void RobotInit() {

		drive_controller = new DriveController();
		elevator_ = new Elevator();
		intake_ = new Intake();
		autonomous_ = new Autonomous(drive_controller, elevator_, intake_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_);

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {

		drive_controller->ZeroI(true);
		drive_controller->ZeroEncs();
		drive_controller->ahrs->ZeroYaw();

		drive_controller->StartTeleopThreads(joyThrottle, joyWheel, &is_heading,
				&is_vision, &is_fc);

	}

	void TeleopPeriodic() {

		is_heading = false;
		is_vision = false;
		is_fc = false;


	}

	void DisabledInit() override {

		drive_controller->EndTeleopThreads();

	}

	void TestPeriodic() {


	}

private:

};

START_ROBOT_CLASS(Robot)
