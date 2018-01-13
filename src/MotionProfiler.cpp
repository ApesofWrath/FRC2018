/*
 * MotionProfiler.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: maxkwon
 */

#include "MotionProfiler.h"

namespace std {

double ramp_time = 0.0;
double ramp_dis = 0.0;

double max_acceleration = 0.0;
double max_velocity = 0.0;

double iterations = 0.0;
double time_dt = 0.00001; //this is the interval that the profiler will run the simulation at,
						  //needs to be faster for accurate integration (area calculation) since this is a reiman sum, it is in seconds
double interval = 0.0;

double robot_width = 0;
double wheel_width = 0;


MotionProfiler::MotionProfiler(double max_vel, double max_acc,
		double time_step) {

	max_velocity = max_vel;
	max_acceleration = max_acc;

	interval = time_step / time_dt; //frequency of when points should be recorded

}



//works off basic triangle geometry calculating times through area calculations under velocity time curves (acceleration is known and constant)
std::vector<std::vector<double> > MotionProfiler::CreateProfile1D(double init_pos,
		std::vector<double> waypoints) {

	double ref = 0;

	double acc = 0.0;
	double vel = 0.0;
	double pos = init_pos;

	double last_vel = 0.0;
	double last_pos = init_pos;

	double time = 0.0;

	int counter = 0;

	//cant initialize any vectors outside of the function or their previous values will carry over into the next profiles made. Don't pull a ChezyChamps2k17
	std::vector<std::vector<double> > matrix; //new matrix every time because .push_back adds rows, moved from the top of the class
	std::vector<double> positions; //first points will be 0
	std::vector<double> velocity;

	int length_waypoint = waypoints.size();

	for (int i = 0; i < length_waypoint; i++) { //will make a profile between each waypoint

		ref = waypoints.at(i);


		if (ref >= init_pos) {
			while (pos < ref) {

				ramp_time = vel / max_acceleration;
				ramp_dis = 0.5 * (vel * ramp_time);

				if ((ref - ramp_dis) <= pos) { //should
					acc = -1.0 * max_acceleration;
				} else if (vel < max_velocity) {
					acc = max_acceleration;
				} else {
					acc = 0.0;
				}

				pos = last_pos + (vel * time_dt);
				last_pos = pos;

				vel = last_vel + (acc * time_dt);
				last_vel = vel;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int)interval == 0) { //only add the points to the motion profile every so often to keep the profile at the desired tick rate
					positions.push_back(pos);   //interval is the frequency of when points should be recorded.
					velocity.push_back(vel);
					counter = 0;
				}
			}
		} else if (ref < init_pos) {
			while (pos > ref) {

				ramp_time = vel / max_acceleration;
				ramp_dis = 0.5 * (vel * ramp_time);

				if ((ramp_dis - ref) >= pos) {
					acc = 1.0 * max_acceleration;
				} else if (vel > (-1.0 * max_velocity)) {
					acc = -1.0 * max_acceleration;
				} else {
					acc = 0.0;
				}

				pos = last_pos + (vel * time_dt);
				last_pos = pos;

				vel = last_vel + (acc * time_dt);
				last_vel = vel;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int)interval == 0) {
					positions.push_back(pos);
					velocity.push_back(vel);
					counter = 0;
				}
			}
		}

		init_pos = ref; //have to redefine the initial position after each waypoint as the waypoint (which is equal to the reference [ref])

	}

	matrix.push_back(positions); //first vector,  row 0
	matrix.push_back(velocity); //second vector, row 1

	return matrix;

}

//returns the angle between two waypoints in radians
double MotionProfiler::FindAngle(std::vector<double> p1, std::vector<double> p2){

	double y1 = p1.at(1);
	double x1 = p1.at(0);
	double y2 = p2.at(1);
	double x2 = p2.at(0);

	double angle = atan2(y1 - y2, x1 - x2); //angle is in radians

	return angle;

}




} /* namespace std */
