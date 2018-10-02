#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <thread>
#include <chrono>
#include <Timer.h>

#ifndef CLIMBER_H_
#define CLIMBER_H_

class Climber {
public:

	TalonSRX *talonClimber1;
	TalonSRX *talonClimber2;

	const int INIT_STATE_C_H = 0;
	const int DOWN_STATE_C_H = 1;
	const int UP_STATE_C_H = 2;
	int climber_state = INIT_STATE_C_H;

	Climber();

void ClimberStateMachine();
void Move();

double GetClimberPosition();

void StartClimberThread();
static void ClimberWrapper(Climber *climber_);
void EndClimberThread();

};

#endif /* CLIMBER_H_ */

//compt bot is comp bt for everything
