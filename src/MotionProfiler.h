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

#ifndef SRC_MOTIONPROFILER_H_
#define SRC_MOTIONPROFILER_H_

//namespace std {

class MotionProfiler {
public:

	//maximum velocity acceleration of the arm or system and the time step for the controller is needed
	MotionProfiler(double max_vel, double max_acc, double time_step);

	//will generate a 1 dimensional profile for appendages
	std::vector<std::vector<double> > CreateProfile1D(double init_pos, std::vector<double> waypoints);

	//get only next point, generate while moving
	//init pos set in a separate function
	std::vector<std::vector<double> > GetNextRef();

	double FindAngle(std::vector<double> p1, std::vector<double> p2);

	//for generating 2 dimensional profiles for robot drives (specifically west coast.
	//Abstract method that will have to be written in the inherited class
	//virtual std::vector<std::vector<double>> CreateWCProfile(double init_pos, std::vector<std::vector<double> > waypoints) = 0; //TODO: create this function

	//CreateProfile1D
	void ZeroProfileIndex();

	//GetNextRef
	void SetInitPos(double pos);
	void SetFinalGoal(double goal);

	//option to change max vel and max acc
	void SetMaxVel(double max_vel);
	void SetMaxAcc(double max_acc);

};

//} /* namespace std */

#endif /* SRC_MOTIONPROFILER_H_ */
