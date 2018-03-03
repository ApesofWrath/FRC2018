/*
 * DriveForward.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/DriveForward.h>

std::vector<std::vector<double> > full_refs_df (1500, std::vector<double>(6)); //initalizes each index value to 0

void DriveForward::Generate() {

	int POINT_LENGTH = 2;
//center left switch scale
	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	//feet
	Waypoint p1 = { 0.0, 0.0, 0.0 }; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
	Waypoint p2 = { 5.0, 10.0, PI/2.0};//1.0, 7.5, .78};
	//forward : 10.0, 0.2, 0.0
	//Waypoint p3 = { 4.5, 8.0, 0.0 };
   //Waypoint p4 = { 9.0, 11.0, 0.0}; //cannot just move in Y axis because of spline math
	//Waypoint p3 = { 10.0, 0.0, 0.0 }; //cannot just move in Y axis because of spline math

	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;
	//points[3] = p4;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //19
	PATHFINDER_SAMPLES_FAST, 0.05, 4.0, 10.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step

	int length = candidate.length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	//								pathfinder points				1500						6									6
	//std::cout << "PATHFINDER MADE IT " << length  << "  " << full_refs.size() << "  " << full_refs[0].size() <<  "   "  << std::endl;

	int l;
	for (l = 0; l < 1500; l++) { ////yaw pos, left pos, right pos, yaw vel, left vel, right vel //TODO: LENGTH will only take first 1500 points from pathfinder
		Segment sl = leftTrajectory[l];
		Segment sr = rightTrajectory[l];

		full_refs_df.at(l).at(0) = ((double)sl.heading); //ZERO yaw vel gains
		full_refs_df.at(l).at(1) = ((double)sl.position);
		full_refs_df.at(l).at(2) = ((double)sr.position);
		full_refs_df.at(l).at(3) = (0.0);
		full_refs_df.at(l).at(4) = ((double)sl.velocity);
		full_refs_df.at(l).at(5) = ((double)sr.velocity);

		if(l >= length) {
			full_refs_df.at(l).at(0) = full_refs_df.at(l-1).at(0);
			full_refs_df.at(l).at(1) = full_refs_df.at(l-1).at(1);
			full_refs_df.at(l).at(2) = full_refs_df.at(l-1).at(2);
			full_refs_df.at(l).at(3) = full_refs_df.at(l-1).at(3);
			full_refs_df.at(l).at(4) = full_refs_df.at(l-1).at(4);
			full_refs_df.at(l).at(5) = full_refs_df.at(l-1).at(5);
		}
	}


//	for (int r = 0; r < full_refs.size(); r++) {
//		for (int c = 0; c < full_refs[0].size(); c++) {
//			std::cout << full_refs.at(r).at(c) << " ";
//		}
//		std::cout << "" << std::endl;
//	}

	drive_controller->SetRefs(full_refs_df);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}
