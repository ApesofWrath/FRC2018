/*
 * MotionProfiler.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: DriversStation
 */

#include <ElevatorMotionProfiler.h>

ElevatorMotionProfiler::ElevatorMotionProfiler(double max_vel, double max_acc,
		double time_step) {

	max_velocity = max_vel;
	max_acceleration = max_acc;

	interval = time_step / time_dt; //frequency of when points should be recorded

}

void ElevatorMotionProfiler::SetFinalGoalElevator(double goal) {

	final_goal_e = goal;

}

void ElevatorMotionProfiler::SetInitPosElevator(double position_init) { //at every new whole profile

	init_pos = position_init;
	pos = init_pos; //this is necessary
	last_vel = 0.0;
	last_pos = init_pos;
	acc = 0.0;
	vel = 0.0;

}

void ElevatorMotionProfiler::SetMaxVelElevator(double max_vel) {

	max_velocity = max_vel;

}

void ElevatorMotionProfiler::SetMaxAccElevator(double max_acc) {

	max_acceleration = max_acc;

}

//pre: set init pos and final goal for the first point in the whole profile
std::vector<std::vector<double>> ElevatorMotionProfiler::GetNextRefElevator() { //used by both elevator and intake

	time_dt = 0.0001; //seconds //lower res without 0.000001 and counter, but still ok

	//cant initialize any vectors outside of the function or their previous values will carry over into the next profiles made. Don't pull a ChezyChamps2k17
	std::vector<std::vector<double> > matrix; //new matrix every time because .push_back adds rows, moved from the top of the class
	std::vector<double> positions; //first points will be 0
	std::vector<double> velocities;
	std::vector<double> accelerations;
	std::vector<double> references; //DOES go down to 0

	ref = final_goal_e; //swtiches constantly for elevator and intake objects

	int counter = 0;

	while (counter < 100) {
		if (ref >= init_pos) { //profile to go up
			if (pos < ref) { //still need to go up, profile not over

				ramp_time = vel / max_acceleration; //y / slope
				ramp_dis = 0.5 * (vel * ramp_time); //area

				if ((ref - ramp_dis) <= pos) { //start ramp down
					acc = -1.0 * max_acceleration;
				} else if (vel < max_velocity) { //ramp up
					acc = max_acceleration;
				} else { //stay at speed
					acc = 0.0;
				}

				pos = last_pos + (vel * time_dt); //update states
				last_pos = pos;

				vel = last_vel + (acc * time_dt);
				last_vel = vel;

			}
		} else if (ref < init_pos) {
			if (pos > ref) {

			//	std::cout << "POS > REF" << std::endl;

				ramp_time = vel / max_acceleration;
				ramp_dis = 0.5 * (vel * ramp_time);

				if ((ramp_dis + ref) >= pos) { //changed to +
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

			}
		}

		counter++;
	}

	positions.push_back(pos);
	velocities.push_back(vel);
	accelerations.push_back(acc);
	references.push_back(ref);

	//can't directly push the pos and vel doubles into matrix because matrix is an array of arrays

	matrix.push_back(positions); //first vector,  row 0
	matrix.push_back(velocities); //second vector, row 1
	matrix.push_back(accelerations);
	matrix.push_back(references);

	return matrix;

}

//works off basic triangle geometry calculating times through area calculations under velocity time curves (acceleration is known and constant)
std::vector<std::vector<double> > ElevatorMotionProfiler::CreateProfile1DElevator(
		double init_pos, //1D movement
		std::vector<double> waypoints) {

	double ref = 0;

	double acceleration = 0.0;
	double velocity = 0.0;
	double position = init_pos;
	double last_velocity = 0.0;
	double last_position = init_pos;

	double time = 0.0;

	time_dt = 0.00001;

	int counter = 0;

	//cant initialize any vectors outside of the function or their previous values will carry over into the next profiles made. Don't pull a ChezyChamps2k17
	std::vector<std::vector<double> > matrix; //new matrix every time because .push_back adds rows, moved from the top of the class
	std::vector<double> positions; //first points will be 0
	std::vector<double> velocities;

	int length_waypoint = waypoints.size();

	for (int i = 0; i < length_waypoint; i++) { //will make a profile between each waypoint

		ref = waypoints.at(i);

		if (ref >= init_pos) {
			while (position < ref) {

				ramp_time = velocity / max_acceleration;
				ramp_dis = 0.5 * (velocity * ramp_time);

				if ((ref - ramp_dis) <= position) { //should
					acceleration = -1.0 * max_acceleration;
				} else if (velocity < max_velocity) {
					acceleration = max_acceleration;
				} else {
					acceleration = 0.0;
				}

				position = last_position + (velocity * time_dt);
				last_position = position;

				velocity = last_velocity + (acceleration * time_dt);
				last_velocity = velocity;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int) interval == 0) { //only add the points to the motion profile every so often to keep the profile at the desired tick rate
					positions.push_back(position); //interval is the frequency of when points should be recorded.
					velocities.push_back(velocity);
					counter = 0;
				}
			}
		} else if (ref < init_pos) {
			while (position > ref) {

				ramp_time = velocity / max_acceleration;
				ramp_dis = 0.5 * (velocity * ramp_time);

				if ((ramp_dis + ref) >= position) {
					acceleration = 1.0 * max_acceleration;
				} else if (velocity > (-1.0 * max_velocity)) {
					acceleration = -1.0 * max_acceleration;
				} else {
					acceleration = 0.0;
				}

				position = last_position + (velocity * time_dt);
				last_position = position;

				velocity = last_velocity + (acceleration * time_dt);
				last_velocity = velocity;

				counter++;
				time += time_dt; //time not used, can be used to graph (not supported)

				if (counter % (int) interval == 0) {
					positions.push_back(position);
					velocities.push_back(velocity);
					counter = 0;
				}
			}
		}

		init_pos = ref; //have to redefine the initial position after each waypoint as the waypoint (which is equal to the reference [ref])

	}

	matrix.push_back(positions); //first vector,  row 0
	matrix.push_back(velocities); //second vector, row 1

	return matrix;

}

//returns the angle between two waypoints in radians
double ElevatorMotionProfiler::FindAngleElevator(std::vector<double> p1,
		std::vector<double> p2) { //point 1, point 2

	double y1 = p1.at(1);
	double x1 = p1.at(0);
	double y2 = p2.at(1);
	double x2 = p2.at(0);

	double angle = atan2(y1 - y2, x1 - x2); //angle is in radians

	return angle;

}
