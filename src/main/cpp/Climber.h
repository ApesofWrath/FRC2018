#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <thread>
#include <chrono>
#include <Timer.h>

#ifndef CLIMBER_H_
#define CLIMBER_H_

class Climber {
public:

	TalonSRX *canTalonClimber;

	Climber();

};

#endif /* BARREL_H_ */

//compt bot is comp bt for everything
