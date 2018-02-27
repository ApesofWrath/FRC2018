/*
 * DriveForward.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/DriveForward.h>

std::vector<std::vector<double> > full_refs;

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

	int l;
	for (l = 0; l < length; l++) { ////yaw pos, left pos, right pos, yaw vel, left vel, right vel
		Segment s = leftTrajectory[l];
		full_refs.at(l).at(0) = s.heading; //ZERO yaw vel gains
		full_refs.at(l).at(1) = s.position;
		full_refs.at(l).at(3) = 0.0;
		full_refs.at(l).at(4) = s.velocity;

		// SmartDashboard::PutNumber("time step", s.dt);
		//	       printf("Time Step: %f\n", s.dt);
		//	       printf("Coords: (%f, %f)\n", s.x, s.y);
		//	       printf("Position (Distance): %f\n", s.position);
		//	       printf("Velocity: %f\n", s.velocity);
		//	       printf("Acceleration: %f\n", s.acceleration);
		//	       printf("Jerk (Acceleration per Second): %f\n", s.jerk);
		//	       printf("Heading (radians): %f\n", s.heading);
	}

	int r;
	for (r = 0; r < length; r++) { ////yaw pos, left pos, right pos, yaw vel, left vel, right vel
		Segment s = rightTrajectory[l];
		full_refs.at(r).at(2) = s.position; //ZERO yaw vel gains
		full_refs.at(r).at(5) = s.velocity;
//		full_refs.at(l).at(3) = 0.0;
//		full_refs.at(l).at(4) = s.velocity;

		// SmartDashboard::PutNumber("time step", s.dt);
		//	       printf("Time Step: %f\n", s.dt);
		//	       printf("Coords: (%f, %f)\n", s.x, s.y);
		//	       printf("Position (Distance): %f\n", s.position);
		//	       printf("Velocity: %f\n", s.velocity);
		//	       printf("Acceleration: %f\n", s.acceleration);
		//	       printf("Jerk (Acceleration per Second): %f\n", s.jerk);
		//	       printf("Heading (radians): %f\n", s.heading);
	}

	SmartDashboard::PutString("here", "yep");
	SmartDashboard::PutNumber("pos", full_refs.at(r).at(2));

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

