#include "Task.h"
#include "../states/ElevatorState.h"

class ElevatorTask : public Task {

public:
     ElevatorState *elevatorState;

	Elevator(ElevatorMotionProfiler *elevator_profiler_, std::vector<std::vector<double> > K_down_e_, std::vector<std::vector<double> > K_up_e_,
		double down_pos_, double mid_pos_, double hps_pos_, double up_pos_, double G_e_, double ff_percent_e_, double PULLEY_DIAMETER_,
		double friction_loss_, int TOP_HALL_, int BOT_HALL_, std::string elev_type_, int TALON_ID_1_, int TALON_ID_2_);

	TalonSRX *talonElevator1, *talonElevator2;

	DigitalInput *hallEffectTop, *hallEffectBottom;

	std::thread ElevatorThread;

	std::string GetElevatorState();

	bool is_elevator_init = false;
	bool keep_elevator_up = false; //used in intake for safety

	double goal_vel_e = 0.0;
	double goal_pos_e = 0.0;

	int zeroing_counter_e = 0;

	const int INIT_STATE_E_H = 0;
	const int DOWN_STATE_E_H = 1;
	const int MID_STATE_E_H = 2;
	const int UP_STATE_E_H = 3;
	const int STOP_STATE_E_H = 4;
	const int HPS_STATE_E_H = 5; //human player station
	int elevator_state = INIT_STATE_E_H;

	const double DOWN_POS_MDS = 0.005;
	const double MID_POS_MDS = 0.668;
	const double HPS_POS_MDS = 0.5;
	const double UP_POS_MDS = 0.89;

	const double DOWN_POS_CARR = 0.01;
	const double MID_POS_CARR = 0.01;
	const double HPS_POS_CARR = 0.01;
	const double UP_POS_CARR = 0.01;

	const double SAFE_CARR_HEIGHT = 0.2; //TODO: actually determine this //carr height at which ms can start moving down
	const double SAFE_MDS_HEIGHT = 0.2;

	void InitializeElevator(); //should be able to initialize carr/ms at same time

	void ElevatorStateMachine(); //same for both
	void Move();

     void TaskStart override();
     void TaskRun oveerride();
	void TaskStop override();

	double GetVoltageElevator();

	void ManualElevator(Joystick *joyOpElev);

	double GetElevatorPosition();
	double GetElevatorVelocity();
	double GetGearRatio();

	bool IsAtBottomElevator();
	bool IsAtTopElevator();
	bool ElevatorEncodersRunning();
	bool IsAtPos(double target_pos);

	bool ZeroEncs();

	void StartElevatorThread();
	static void ElevatorWrapper(Elevator *elevator_); //TODO: may need to have two separate static functions for mds and carr
	void EndElevatorThread();

private:
	const double free_speed_e = 18730.0; //rad/s
	const double TICKS_PER_ROT_E = 4096.0; //possibly not
	const double MAX_VOLTAGE_E = 12.0; //CANNOT EXCEED abs(12)
	const double MIN_VOLTAGE_E = -10.0;

	std::vector<std::vector<double>> K_down_e, K_up_e, K_e; //parameter variables specific to carr/mds MUST be here. otherwise the second object creation will override the first one's variables
	double down_pos, mid_pos, hps_pos, up_pos, G_e, ff_percent_e, PULLEY_DIAMETER, friction_loss;
	int TOP_HALL, BOT_HALL, TALON_ID_1, TALON_ID_2;
	std::string elev_type, elev_safety;

	std::map <int, std::string> elev_state = {
		{INIT_STATE_E, "INIT"},
		{DOWN_STATE_E, "DOWN"},
		{MID_STATE_E, "MID"},
		{UP_STATE_E, "UP"},
		{STOP_STATE_E, "STOP"},
		{HPS_STATE_E, "HPS"}
	};

	// Constructor helpers
	void SetupTalon1();
	void SetupTalon2();

	// Setvoltage helpers
  	void SetVoltage(double voltage);
	void SetZeroOffset();
     void UpperSoftLimit();
	void LowerSoftLimit();
	void TopHallEffectSafety();
	void BottomHallEffectSafety();
	void ArmSafety();
	void StallSafety();
     void ZeroElevator();
     void CapVoltage();
     void ScaleOutput();
     void InvertOutput();
     void OutputToTalon();

	// State machine helpers
	void ManualElevatorOutput();
	void PrintElevatorInfo();
	void InitState();
	void CheckElevatorGoal(int current_state, double goal_pos);

	// Move helpers
	void UpdateToMoveDirection(double offset_, double percent, std::vector<std::vector<double>> K_e_);
	void UpdateVoltage();
	void UpdateMoveError();
	void UpdateMoveCoordinates();
};
