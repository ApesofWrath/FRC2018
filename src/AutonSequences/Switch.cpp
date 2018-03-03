/*
 * Switch.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/Switch.h>

std::vector<std::vector<double> > full_refs_sw (1500, std::vector<double>(6)); //initalizes each index value to 0

void Switch::GenerateSwitch(bool left) { //left center right

	//Auton thread started in auton constructor

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	//feet
	Waypoint p1 = { 0.0, 0.0, 0.0 }; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
	Waypoint p2 = {  5.0, 10.0, PI/2.0 };
	//Waypoint p3 = { 5.0, 5.0, 0.0 }; //cannot just move in Y axis because of spline math
	//Waypoint p3 = { 10.0, 0.0, 0.0 }; //cannot just move in Y axis because of spline math

	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
		PATHFINDER_SAMPLES_FAST, 0.05, 19.0, 10.0, 100000.0, &candidate); //max vel, acc, jerk

	int length = candidate.length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	int l;
	for (l = 0; l < 1500; l++) { ////yaw pos, left pos, right pos, yaw vel, left vel, right vel
		Segment sl = leftTrajectory[l];
		Segment sr = rightTrajectory[l];

		full_refs_sw.at(l).at(0) = ((double)sl.heading);
		full_refs_sw.at(l).at(1) = ((double)sl.position);
		full_refs_sw.at(l).at(2) = ((double)sr.position);
		full_refs_sw.at(l).at(3) = (0.0);
		full_refs_sw.at(l).at(4) = ((double)sl.velocity);
		full_refs_sw.at(l).at(5) = ((double)sr.velocity);

		if(l >= length) {
			full_refs_sw.at(l).at(0) = full_refs_sw.at(l-1).at(0);
			full_refs_sw.at(l).at(1) = full_refs_sw.at(l-1).at(1);
			full_refs_sw.at(l).at(2) = full_refs_sw.at(l-1).at(2);
			full_refs_sw.at(l).at(3) = full_refs_sw.at(l-1).at(3);
			full_refs_sw.at(l).at(4) = full_refs_sw.at(l-1).at(4);
			full_refs_sw.at(l).at(5) = full_refs_sw.at(l-1).at(5);
		}
	}

	FillProfile(full_refs_sw);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void Switch::RunStateMachine() {

	//no other state machine booleans needed, all other ones will stay false

	if(GetLeftPos() == 10.0) {
		raise_to_switch = true;
	}
	else {
		raise_to_switch = false;
	}

}

