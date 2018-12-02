#include "Task.h"
#include "../states/DriveState.h"

#include <iostream>
#include <WPILib.h>
#include <Joystick.h>
#include "AHRS.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <Timer.h>
#include "ctre/Phoenix.h"
#include <pathfinder.h>

class DriveTask : public Task {
public:

	TalonSRX *canTalonLeft1, *canTalonLeft2, *canTalonLeft3, *canTalonLeft4, *canTalonRight1, *canTalonRight2,
			*canTalonRight3, *canTalonRight4, *canTalonKicker; //for 4 talons: 1 is front right, 2 is back right, 3 is front left, 4 is back left

	DoubleSolenoid *solenoid;
	AHRS *ahrs;

	DriveState *driveState;

	bool continue_profile = true;
	bool set_profile = false;
	bool set_refs = false;

	std::vector<double> drive_ref = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //AutonDrive, row, individual points
	std::vector<std::vector<double> > GetAutonProfile();

	int row_index = 0;
	int zeroing_counter = 0;
	std::vector<int> zeroing_index;

	double time_step_drive;

	DriveTask(int l1, int l2, int l3, int l4,
			int r1, int r2, int r3, int r4, bool start_low, double time_step);

	void ZeroAll(bool stop_motors);
	void StopAll();
	void ZeroEncs();
	void ZeroYaw();
	void ZeroI();
	void SetCurrentLimits();

	void TaskStart() override;
	void TaskRun() override;
	void TaskRunAuto();
	void TaskRunTeleop(Joystick *JoyThrottle,
			Joystick *JoyWheel);
	void TaskStop() override;

	void Controller();

  void UpdateInputs(Joystick *JoyThrottle,
			Joystick *JoyWheel);
	void SquareInputs();
  void NormalizeInputs();
  void UpdateState();
  void UpdateTargets();
  void UpdateError();
  void LimitTargets();
  void SetOutputs(double left, double right);

	void ShiftUp();
	void ShiftDown();
	void AutoShift(bool auto_shift);

	void SetGainsHigh();
	void SetGainsLow();
	void SetAutonGains(bool same_side_scale);


};
