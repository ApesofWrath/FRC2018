/*
 * DriveForward.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/DriveForward.h>

std::vector<std::vector<double> > full_refs_df (1500, std::vector<double>(6)); //initalizes each index value to 0

void DriveForward::GenerateForward() {

	//Auton state machine thread started in Auton constructor

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	//feet
	Waypoint p1 = { 0.0, 0.0, 0.0 };
	Waypoint p2 = { 10.0, 0.2, 0.0 };

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.05, 19.0, 10.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step

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

		full_refs_df.at(l).at(0) = ((double)sl.heading);
		full_refs_df.at(l).at(1) = ((double)sl.position);
		full_refs_df.at(l).at(2) = ((double)sr.position);
		full_refs_df.at(l).at(3) = (0.0);
		full_refs_df.at(l).at(4) = ((double)sl.velocity);
		full_refs_df.at(l).at(5) = ((double)sr.velocity);

		if(l >= length) { //remaining points of the 1500 are set to the last point given by pathfinder
			full_refs_df.at(l).at(0) = full_refs_df.at(l-1).at(0);
			full_refs_df.at(l).at(1) = full_refs_df.at(l-1).at(1);
			full_refs_df.at(l).at(2) = full_refs_df.at(l-1).at(2);
			full_refs_df.at(l).at(3) = full_refs_df.at(l-1).at(3);
			full_refs_df.at(l).at(4) = full_refs_df.at(l-1).at(4);
			full_refs_df.at(l).at(5) = full_refs_df.at(l-1).at(5);
		}
	}

	FillProfile(full_refs_df);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}
