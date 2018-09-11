#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

#include <ElevatorMotionProfiler.h>
#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <chrono>
#include <Timer.h>
#include <thread>

//TODO: make variable names different from innerelevator's, or make private?

class OuterElevator {
public:

  OuterElevator(ElevatorMotionProfiler *elevator_profiler_);

  TalonSRX *talonElevator1, *talonElevator2;

  DigitalInput *hallEffectTop;
  DigitalInput *hallEffectBottom;

  std::thread ElevatorThread;

  void InitializeElevator();

	void ElevatorStateMachine();
	void Move();
	void StopElevator();

	double GetVoltageElevator();
	void SetVoltageElevator(double elevator_voltage);

  void SetZeroOffsetElevator();

	double GetElevatorPosition();
	double GetElevatorVelocity();

	bool IsAtBottomElevator();
	bool IsAtTopElevator();
	bool ElevatorEncodersRunning();

	bool ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_);
	void EndElevatorThread();

private:

  const double free_speed_e = 18730.0; //rad/s
  const double G_e = (20.0 / 1.0); //gear ratio

  const double TICKS_PER_ROT_E = 4096.0; //possibly not
  const double PULLEY_DIAMETER = 0.0381; //diameter of the pulley in meters

  const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)  //4.0
  const double MIN_VOLTAGE_E = -10.0;

  const double friction_loss = 0.75; //checked with graph. ff matches ref vel

  const double MAX_THEORETICAL_VELOCITY_E = (free_speed_e / G_e) / 60.0
  		* PULLEY_DIAMETER * PI * friction_loss; //m/s //1.87 //1.32
  const double Kv_e = 1 / MAX_THEORETICAL_VELOCITY_E;

  const int ELEVATOR_SLEEP_TIME = 0;
  const double ELEVATOR_WAIT_TIME = 0.01; //sec

  int last_elevator_state = 0; //init state

  double offset = 0.0;
  double ff = 0.0; //feedforward
  double u_e = 0.0; //this is the input in volts to the motor
  double v_bat_e = 0.0; //this will be the voltage of the battery at every loop

  double position_offset_e = 0.0;

  std::vector<std::vector<double> > K_e;
  std::vector<std::vector<double> > K_down_e =
  		{ {  27.89, 4.12 }, { 25.90, 1.57 } }; //controller matrix that is calculated in the Python simulation 17.22, 0.94
  std::vector<std::vector<double> > K_up_e = { { 27.89, 4.12 }, { 22.11, 1.75 } }; //controller matrix that is calculated in the Python simulation

  std::vector<std::vector<double> > X_e = { { 0.0 }, //state matrix filled with the state of the states of the system //not used
  		{ 0.0 } };

  std::vector<std::vector<double> > error_e = { { 0.0 }, { 0.0 } };

  ElevatorMotionProfiler *elevator_profiler;

  Timer *elevatorTimer = new Timer();

};

#endif
