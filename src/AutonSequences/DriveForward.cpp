/*
 * DriveForward.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/DriveForward.h>

std::vector<std::vector<double> > full_refs = { {} };

void DriveForward::Generate() {

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	//feet
	Waypoint p1 = { 0, 0, 0 }; //starting position may not be allowed to be 0,0,0
	Waypoint p2 = { 0.5, 4, 0 }; //cannot just move in Y axis because of spline math

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.001, 15.0, 10.0, 60.0, &candidate); //max vel, acc, jerk

	int length = candidate.length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 0.6;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	std::cout << "PATHFINDER MADE IT" << length << std::endl;

//	for (int r = 0; r < length; r++) {
//		for (int c = 0; c < 6; c++) {
//			full_refs.at(r).push_back(0.0);
//			std::cout << "filling" << std::endl;
//		}
//	}
//
//	std::cout << "Filled" << std::endl;
//	std::cout << "rows " << sizeof(full_refs) << " cols "
//			<< full_refs.at(0).size() << std::endl;

	for(int i = 0; i < 1500; i++) {
		full_refs.push_back({});
	}

	int l;
	for (l = 0; l < 1500; l++) { ////yaw pos, left pos, right pos, yaw vel, left vel, right vel //TODO: LENGTH
		Segment sl = leftTrajectory[l];
		Segment sr = rightTrajectory[l];

		full_refs.at(l).push_back((double)sl.heading); //ZERO yaw vel gains
		full_refs.at(l).push_back((double)sl.position);
		full_refs.at(l).push_back((double)sr.position);
		full_refs.at(l).push_back(0.0);
		full_refs.at(l).push_back((double)sl.velocity);
		full_refs.at(l).push_back((double)sr.velocity);

//		full_refs.at(l).at(0) = sl.heading; //ZERO yaw vel gains
//		full_refs.at(l).at(1) = sl.position;
//		full_refs.at(l).at(2) = sr.position;
//		full_refs.at(l).at(3) = 0.0;
//		full_refs.at(l).at(4) = sl.velocity;
//		full_refs.at(l).at(5) = sr.velocity;

	//	full_refs.push_back({});

		std::cout << "l: " << l << std::endl;
	}

	std::cout << "filled full refs" << "  " << sizeof(full_refs) << " " << full_refs.at(0).size() << std::endl;

	SmartDashboard::PutNumber("pathfinder points", sizeof(full_refs));
	SmartDashboard::PutNumber("pathfinder point length",
			full_refs.at(0).size());

	for (int r = 0; r < sizeof(full_refs); r++) {
		for (int c = 0; c < full_refs.at(0).size(); c++) { //not printing the 6th
			std::cout << full_refs.at(r).at(c) << " ";
		}
		std::cout << "" << std::endl;
	}

	drive_controller->SetRefs(full_refs);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

//void DriveForward::SetDriveRefs() {
//
//	drive_controller->SetRefs(full_refs);
//
//}

