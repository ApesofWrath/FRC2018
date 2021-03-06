/*
 * MotionProfiler.h
 *
 *  Created on: Jan 29, 2018
 *      Author: DriversStation
 */

#include <list>
#include <vector>
#include <iostream>
#include <cmath>
#include <WPILib.h>

#ifndef SRC_INTAKEMOTIONPROFILER_H_
#define SRC_INTAKEMOTIONPROFILER_H_

class IntakeMotionProfiler {

private:

	double ref = 0.0;
	double final_goal = 0.0;
	double init_pos = 0.0;
	double acc = 0.0;
	double vel = 0.0;
	double pos = init_pos;
	double last_vel = 0.0;
	double last_pos = init_pos;

	double ramp_time = 0.0; //for both
	double ramp_dis = 0.0;
	double max_acceleration = 0.0;
	double max_velocity = 0.0;

	double iterations = 0.0;
	double time_dt = 0.0001; //this is the interval that the profiler will run the simulation at,
	//needs to be faster for accurate integration (area calculation) since this is a reiman sum, it is in seconds
	double interval = 0.0;

	double robot_width = 0;
	double wheel_width = 0;

public:

	double GetInitPosIntake();
	double GetFinalGoalIntake();

	//maximum velocity acceleration of the arm or system and the time step for the controller is needed
	IntakeMotionProfiler(double max_vel, double max_acc, double time_step);

	//will generate a 1 dimensional profile for appendages
	std::vector<std::vector<double> > CreateProfile1DIntake(double init_pos,
			std::vector<double> waypoints);

	//get only next point, generate while moving
	//init pos set in a separate function
	std::vector<std::vector<double> > GetNextRefIntake();

	double FindIntakeAngle(std::vector<double> p1, std::vector<double> p2);

	//for generating 2 dimensional profiles for robot drives (specifically west coast.
	//Abstract method that will have to be written in the inherited class
	//virtual std::vector<std::vector<double>> CreateWCProfile(double init_pos, std::vector<std::vector<double> > waypoints) = 0; //TODO: create this function

	//for CreateProfile1D
	void ZeroProfileIndexIntake();

	//for GetNextRef
	void SetInitPosIntake(double position_init);
	void SetFinalGoalIntake(double goal);
	void SetMaxVelIntake(double max_vel);
	void SetMaxAccIntake(double max_acc);

};

#endif /* SRC_MOTIONPROFILER_H_ */
