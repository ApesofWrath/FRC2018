/*
 * Intake.h
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */


#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include <Timer.h>
#include <thread>

class Intake {
public:

	TalonSRX *talonIntake1, *talonIntake2, *talonIntakeArm;

	std::thread IntakeThread;

	const int STOP_WHEEL_STATE_H = 0;
	const int IN_STATE_H = 1;
	const int OUT_STATE_H = 2;
	const int DOWN_STATE_H = 3;
	const int UP_STATE_H = 4;
	int intake_state_h = 0;

	Intake();

	void In();
	void Out();
	void Stop();
	void Rotate(double ref);

	void IntakeStateMachine();

	void StartIntakeThread();
	void EndIntakeThread();
	static void IntakeWrapper(Intake *in, double *ref);



};

#endif /* SRC_INTAKE_H_ */
