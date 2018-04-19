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
#include <AutonSequences/SwitchCenter.h>
#include <AutonSequences/ScaleSide.h>
#include <TeleopStateMachine.h>
#include <AutonStateMachine.h>
#include <AutonSequences/SwitchSide.h>
#include <TaskManager.h>

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

	const int LOW_GEAR_BUTTON = 5;
	const int HIGH_GEAR_BUTTON = 6;
	const int HEADING_BUTTON = 9;

#if BUTTONBOX

	const int WAIT_FOR_BUTTON = 13;

	const int GET_CUBE_GROUND = 14;
	const int GET_CUBE_STATION = 99;
	//const int POST_INTAKE = 5; //taken for middle shot
	const int RAISE_TO_SWITCH = 6;
	const int RAISE_TO_SCALE_MED = 1;
	const int RAISE_TO_SCALE_SLOW = 5;
	const int RAISE_TO_SCALE_FAST = 4;
	const int RAISE_TO_SCALE_BACKWARDS = 12;

	const int INTAKE_SPIN_IN = 10; //not enough buttons for these three
	const int INTAKE_SPIN_OUT = 1; //throttle
	const int INTAKE_SPIN_STOP = 99;
	const int INTAKE_SPIN_SLOW = 8;
	const int INTAKE_SPIN_MED = 2; //pop

	const int INTAKE_ARM_UP = 8;
	const int INTAKE_ARM_MID = 99; //TAKEN OUT
	const int INTAKE_ARM_DOWN = 15;
	const int INTAKE_ARM_BACKWARDS = 99; //no manual for this one

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

	bool wait_for_button, intake_spin_in, intake_spin_out, intake_spin_slow,
			intake_spin_med, intake_spin_stop, get_cube_ground,
			get_cube_station, post_intake, raise_to_switch, raise_to_scale_slow,
			raise_to_scale_med, raise_to_scale_fast, intake_arm_up,
			intake_arm_mid, intake_arm_down, elevator_up, elevator_mid,
			elevator_down, raise_to_scale_backwards; //for BOTH state machines

	bool is_heading, is_vision, is_fc; //drive
	bool is_auto_shift;

	bool left_scale, left_switch;

	DriveController *drive_controller;
	PowerDistributionPanel *pdp_;
	Elevator *elevator_;
	Intake *intake_;
	TeleopStateMachine *teleop_state_machine;
	AutonStateMachine *auton_state_machine;
	ElevatorMotionProfiler *elevator_profiler_;
	IntakeMotionProfiler *intake_profiler_;
	Compressor *compressor_;
	Joystick *joyThrottle, *joyWheel, *joyOp;
	TaskManager *task_manager;

	DriveForward *drive_forward;
	SwitchCenter *switch_center;
	ScaleSide *scale_side;
	SwitchSide *switch_side;

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

	std::string autoSelected;

	std::string positionSelected;

	int state_test = 0;
	int last_state_test = 1;

	Timer *timerTest = new Timer();

	void RobotInit() {

		SmartDashboard::PutNumber("targetHeading", 0);

		SmartDashboard::PutNumber("refLeft", 0);
		SmartDashboard::PutNumber("refRight", 0);
		SmartDashboard::PutNumber("refLeftVel", 0);
		SmartDashboard::PutNumber("actualLeftDis", 0);
		SmartDashboard::PutNumber("actualRightDis", 0);
		SmartDashboard::PutNumber("actualLeftVel", 0);
		SmartDashboard::PutNumber("Actual Heading", 0);
		SmartDashboard::PutNumber("refYaw", 0);

		elevator_profiler_ = new ElevatorMotionProfiler(1.15, 5.0, TIME_STEP); //max vel, max accel, timestep
		intake_profiler_ = new IntakeMotionProfiler(2.0, 10.0, TIME_STEP);

		compressor_ = new Compressor(3); //commenting these out breaks the code
		pdp_ = new PowerDistributionPanel(3);

		drive_controller = new DriveController(TIME_STEP); //inherits from mother class //pass in time step here for auton subclasses
		elevator_ = new Elevator(pdp_, elevator_profiler_);
		intake_ = new Intake(pdp_, intake_profiler_, elevator_);
		teleop_state_machine = new TeleopStateMachine(elevator_, intake_,
				drive_controller); //actually has both state machines
		auton_state_machine = new AutonStateMachine(elevator_, intake_,
				drive_controller);
		task_manager = new TaskManager(teleop_state_machine,
				auton_state_machine, drive_controller, elevator_, intake_,
				TIME_STEP);

		//if(drive_controller->ahrs->IsCalibrating())
		//drive_controller->ahrs->IsConnected()

		joyThrottle = new Joystick(JOY_THROTTLE);
		joyWheel = new Joystick(JOY_WHEEL);
		joyOp = new Joystick(JOY_OP);

		autonChooser.AddDefault(sideDriveForward, sideDriveForward); //drives backward
		autonChooser.AddObject(centerDriveForward, centerDriveForward);
		autonChooser.AddObject(doNothing, doNothing);

		autonChooser.AddObject(centerCubeSwitch, centerCubeSwitch);

//		autonChooser.AddObject(leftCubeSwitch, leftCubeSwitch); //these do NOT exist yet
//		autonChooser.AddObject(rightCubeSwitch, rightCubeSwitch);
//		autonChooser.AddObject(centerCubeSwitchSwitch, centerCubeSwitchSwitch);

		autonChooser.AddObject(leftCubeScale, leftCubeScale);
		autonChooser.AddObject(rightCubeScale, rightCubeScale);
		autonChooser.AddObject(leftCubeScaleScale, leftCubeScaleScale);
		autonChooser.AddObject(rightCubeScaleScale, rightCubeScaleScale);
		autonChooser.AddObject(leftScaleSwitch, leftScaleSwitch);
		autonChooser.AddObject(rightScaleSwitch, rightScaleSwitch);

		frc::SmartDashboard::PutData("Auto Modes", &autonChooser);

#if TESTING
		//starting threads in robot init so that they only are created once
		task_manager->StartThread(
				&wait_for_button, //both auton and teleop state machines
				&intake_spin_in, &intake_spin_out, &intake_spin_slow,
				&intake_spin_med, &intake_spin_stop, &get_cube_ground,
				&get_cube_station, &post_intake, &raise_to_switch,
				&raise_to_scale_slow, &raise_to_scale_med, &raise_to_scale_fast,
				&intake_arm_up, &intake_arm_mid, &intake_arm_down, &elevator_up,
				&elevator_mid, &elevator_down, &raise_to_scale_backwards,
				joyThrottle, joyWheel, &is_heading);

#else
		intake_->StartIntakeThread(); //controllers
		elevator_->StartElevatorThread();

		drive_controller->StartDriveThreads(joyThrottle, joyWheel, &is_heading,//both auton and teleop drive
				&is_vision, &is_fc);//auton drive will not start until profile for auton is sent through

		auton_state_machine->StartAutonStateMachineThread(
				&wait_for_button,//both auton and teleop state machines
				&intake_spin_in, &intake_spin_out, &intake_spin_slow, &intake_spin_stop,
				&get_cube_ground, &get_cube_station, &post_intake,
				&raise_to_switch, &raise_to_scale, &intake_arm_up,
				&intake_arm_mid, &intake_arm_down, &elevator_up, &elevator_mid,
				&elevator_down, &raise_to_scale_backwards);

		teleop_state_machine->StartStateMachineThread(
				&wait_for_button,//both auton and teleop state machines
				&intake_spin_in, &intake_spin_out, &intake_spin_slow, &intake_spin_stop,
				&get_cube_ground, &get_cube_station, &post_intake,
				&raise_to_switch, &raise_to_scale, &intake_arm_up,
				&intake_arm_mid, &intake_arm_down, &elevator_up, &elevator_mid,
				&elevator_down, &raise_to_scale_backwards);
#endif
	}

	void AutonomousInit() override {

		auton_state_machine->Initialize();
		compressor_->Stop(); //not working

		drive_controller->ZeroAll(true);
		drive_controller->ShiftUp();

		///////////////////////////////////////////////////////////////////////////

		std::string gameData = "";

		for (int i = 0; i < 2000; i++) { //FMS data may not come immediately
			gameData =
					frc::DriverStation::GetInstance().GetGameSpecificMessage();
			if (gameData.length() > 0) { //sizeof(gameData) / sizeof(gameData[0]
				//std::cout << "BREAK" << std::endl;
				break;//leave for loop early
			}

			//std::this_thread::sleep_for(std::chrono::milliseconds(1));
		} //will stop checking after 15 counts, but does not necessarily mean data has been received

		autoSelected = autonChooser.GetSelected();

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
			autoSelected = sideDriveForward; //regardless of auton chooser //centerDriveForward
			//std::cout << "DEFAULT" << std::endl;
		}

		/////////////////////////////////////////////////////////////////////////

		if (autoSelected == centerCubeSwitch) {
			switch_center = new SwitchCenter(drive_controller, elevator_,
					intake_, auton_state_machine);
			switch_center->GenerateSwitch(leftSwitch, false);
			switchCenterOneState = true;

		} else if (autoSelected == leftCubeScale) {
			if (leftScale) {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateSameScale(true, false, false);
				sameScaleOneState = true;
			} else {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(true, false, false); //first param is starting pos
				oppScaleOneState = true;
			}

		} else if (autoSelected == rightCubeScale) {
			if (!leftScale) {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateSameScale(false, false, false);
				sameScaleOneState = true;
			} else {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(false, false, false);
				oppScaleOneState = true;
			}

		} else if (autoSelected == leftCubeScaleScale) {
			if (leftScale) {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateSameScale(true, false, true);
				sameScaleTwoState = true;
			} else {
//				drive_forward = new DriveForward(drive_controller, elevator_, // IF WRONG SIDE, will stay safe and just do one on opp
//						intake_, auton_state_machine);
//				drive_forward->GenerateForward(false);

				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(true, false, true);
				oppScaleTwoState = true;
			}

		} else if (autoSelected == rightCubeScaleScale) {
			if (!leftScale) {
				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateSameScale(false, false, true);
				sameScaleTwoState = true;
			} else {
//				drive_forward = new DriveForward(drive_controller, elevator_,
//						intake_, auton_state_machine);
//				drive_forward->GenerateForward(false);

				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(false, false, true);
				oppScaleTwoState = true;
			}

		} else if (autoSelected == leftScaleSwitch) {
			scale_side = new ScaleSide(drive_controller, elevator_, intake_,
					auton_state_machine);
			if (leftScale && leftSwitch) { //scale and switch
				scale_side->GenerateSameScale(true, true, false);
				sameScaleSwitchState = true;
			} else if (leftScale && !leftSwitch) { //only scale
				scale_side->GenerateSameScale(true, false, false);
				sameScaleOneState = true;
			} else { //!leftScale
//				drive_forward = new DriveForward(drive_controller, elevator_,
//						intake_, auton_state_machine);
//				drive_forward->GenerateForward(false);

				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(true, false, false);
				oppScaleOneState = true;
			}

		} else if (autoSelected == rightScaleSwitch) {
			scale_side = new ScaleSide(drive_controller, elevator_, intake_,
					auton_state_machine);
			if (!leftScale && !leftSwitch) { //scale and switch
				scale_side->GenerateSameScale(false, true, false);
				sameScaleSwitchState = true; //scale state machine works for both scale and scale+switch
			} else if (!leftScale && leftSwitch) { //only scale
				scale_side->GenerateSameScale(false, false, false);
				sameScaleOneState = true;
			} else { //leftScale
//				drive_forward = new DriveForward(drive_controller, elevator_,
//						intake_, auton_state_machine);
//				drive_forward->GenerateForward(false);

				scale_side = new ScaleSide(drive_controller, elevator_, intake_,
						auton_state_machine);
				scale_side->GenerateOppScale(false, false, false);
				oppScaleOneState = true;
			}

		} else if (autoSelected == centerDriveForward) { //depends on starting robot backwards when on side, and forwards when in middle
			drive_forward = new DriveForward(drive_controller, elevator_,
					intake_, auton_state_machine);
			drive_forward->GenerateForward(true);

		} else if (autoSelected == sideDriveForward) { //depends on starting robot backwards when on side, and forwards when in middle
			drive_forward = new DriveForward(drive_controller, elevator_,
					intake_, auton_state_machine);
			drive_forward->GenerateForward(false);

		} else if (autoSelected == doNothing) {

		} else {
			drive_forward = new DriveForward(drive_controller, elevator_,
					intake_, auton_state_machine);
			drive_forward->GenerateForward(false); // risky, assuming start backward
		}

	}

	void AutonomousPeriodic() {

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

		if (sameScaleOneState) { //same side
			scale_side->RunStateMachineSameScale(&raise_to_scale_backwards,
					&get_cube_ground);

		} else if (oppScaleOneState) {
			scale_side->RunStateMachineOppScale(&raise_to_scale_backwards,
					&get_cube_ground);

		} else if (sameScaleSwitchState) {
			scale_side->RunStateMachineSameScaleSwitch(
					&raise_to_scale_backwards, &raise_to_switch,
					&get_cube_ground);

		} else if (oppScaleSwitchState) {
			scale_side->RunStateMachineOppScaleSwitch(&raise_to_scale_backwards,
					&raise_to_switch, &get_cube_ground);

		} else if (sameScaleTwoState) {
			scale_side->RunStateMachineSameScaleScale(&raise_to_scale_backwards,
					&get_cube_ground);

		} else if (oppScaleTwoState) {
			scale_side->RunStateMachineOppScaleScale(&raise_to_scale_backwards,
					&get_cube_ground);

		} else if (switchCenterOneState) {
			switch_center->RunStateMachine(&raise_to_switch);

		} else {

		}

	}

	void TeleopInit() {

		compressor_->SetClosedLoopControl(true);
		//teleop_state_machine->Initialize(); //only initialize in auton state machine
		drive_controller->ZeroAll(true);
		drive_controller->ShiftDown();

	}

	void TeleopPeriodic() {

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

#if !STATEMACHINE
		intake_->ManualArm(joyOp);
		//	intake_->ManualWheels(joyOp);
		elevator_->ManualElevator(joyThrottle);

#else

		bool low_gear = joyWheel->GetRawButton(LOW_GEAR_BUTTON);
		bool high_gear = joyWheel->GetRawButton(HIGH_GEAR_BUTTON);

		wait_for_button = joyOp->GetRawButton(WAIT_FOR_BUTTON);
		get_cube_ground = joyOp->GetRawButton(GET_CUBE_GROUND);
		get_cube_station = false; //joyOp->GetRawButton(GET_CUBE_STATION); //not in existencee
		//post_intake = joyOp->GetRawButton(POST_INTAKE); //taken for medium shot
		raise_to_switch = joyOp->GetRawButton(RAISE_TO_SWITCH);
		raise_to_scale_med = joyOp->GetRawButton(RAISE_TO_SCALE_MED);
		raise_to_scale_slow = joyOp->GetRawButton(RAISE_TO_SCALE_SLOW);
		raise_to_scale_fast = joyOp->GetRawButton(RAISE_TO_SCALE_FAST);
		raise_to_scale_backwards = joyOp->GetRawButton(
				RAISE_TO_SCALE_BACKWARDS);
		intake_spin_med = joyOp->GetRawButton(INTAKE_SPIN_MED); //operator switch pop shot
		//intake_spin_stop = joyThrottle->GetRawButton(INTAKE_SPIN_STOP);
		intake_arm_up = joyOp->GetRawButton(INTAKE_ARM_UP);
		intake_arm_mid = joyOp->GetRawButton(INTAKE_ARM_MID);
		intake_arm_down = joyOp->GetRawButton(INTAKE_ARM_DOWN);
		elevator_up = joyOp->GetRawButton(ELEVATOR_UP);
		elevator_mid = joyOp->GetRawButton(ELEVATOR_MID);
		elevator_down = joyOp->GetRawButton(ELEVATOR_DOWN);

		intake_spin_in = joyThrottle->GetRawButton(INTAKE_SPIN_IN); //these all are manual and can always happen
		intake_spin_out = joyThrottle->GetRawButton(INTAKE_SPIN_OUT);
		intake_spin_slow = joyThrottle->GetRawButton(INTAKE_SPIN_SLOW); //this one specifically for slowing down backwards shot //TODO: maybe add a different state for the two back shot speeds
		intake_spin_stop = false;

		is_heading = joyThrottle->GetRawButton(HEADING_BUTTON);
		is_vision = false;
		is_fc = false;

		if (low_gear) {
			is_auto_shift = false;
			drive_controller->ShiftDown();
		} else if (high_gear) { //bad because if want to shift down when in up shift velocity range, once you let go of the low_gear, it will go back to high gear
			drive_controller->ShiftUp();
			is_auto_shift = false;
		} else {
			is_auto_shift = true;
		}

#endif
	}

	void DisabledInit() override { //between auton and teleop

	}

	void TestPeriodic() {

		SmartDashboard::PutNumber("enc l",
				drive_controller->canTalonLeft1->GetSelectedSensorPosition(0));

		SmartDashboard::PutNumber("enc r",
				drive_controller->canTalonRight1->GetSelectedSensorPosition(0));

		SmartDashboard::PutNumber("EL POS", elevator_->GetElevatorPosition());
		SmartDashboard::PutNumber("ARM POS", intake_->GetAngularPosition());

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
//
//		if (last_state_test != 0) {
//			timerTest->Start();
//		}
//
//		if (timerTest->HasPeriodPassed(3)
//				&& std::abs(drive_controller->GetLeftVel()) > 0.0
//				&& std::abs(drive_controller->GetRightVel()) > 0.0) { //first one not needed //550, 1250 //JUST LOOK AT CURRENTS
//
//			state_test = 1;
//			timerTest->Reset();
//
//		}
//
//		last_state_test = 0;

		switch (state_test) { //threads are always running

		case 0:
			intake_->IntakeArmStateMachine(); //init state
			intake_->IntakeWheelStateMachine();
			elevator_->ElevatorStateMachine();

			if (intake_->is_init_intake && elevator_->is_elevator_init) { //once initialized, state machines are ou
				state_test = 0;
			}
			last_state_test = 0;

			break;

		case 1:
			intake_->IntakeArmStateMachine(); //up
			intake_->IntakeWheelStateMachine();
			if (std::abs(intake_->GetAngularPosition() - intake_->UP_ANGLE)
					< 0.1) {
				state_test = 2;
			}
			break;

		case 2:
			intake_->intake_arm_state = intake_->DOWN_STATE_H;
			intake_->IntakeArmStateMachine(); //down states
			intake_->IntakeWheelStateMachine();
			if (std::abs(intake_->GetAngularPosition() - intake_->DOWN_ANGLE)
					< 0.1) {
				state_test = 3;
			}
			break;

		case 3:
			elevator_->elevator_state = elevator_->MID_STATE_E_H;
			elevator_->ElevatorStateMachine();
			if (std::abs(
					elevator_->GetElevatorPosition() - elevator_->MID_POS_E)
					< 0.1) {

			}
		}

	} //arm up, elev down

private:

}
;

START_ROBOT_CLASS(Robot)
