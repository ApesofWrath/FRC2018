/*
 * DriveForward.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: DriversStation
 */

#include <DriveForward.h>

std::vector<std::vector<double> > full_refs;

DriveForward::DriveForward(DriveController *drive_cont, Elevator, *el, Intake *in) {


}

void DriveForward::Generate() {

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1 = { 0, 1, 0 }; // Waypoint @ x=-4, y=-1, exit angle=45 degrees
	Waypoint p2 = { -1, 2, 0 };  // Waypoint @ x=-1, y= 2, exit angle= 0 radians
	//Waypoint p3 = { 2, 4, 0 };   // Waypoint @ x= 2, y= 4, exit angle= 0 radians
	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.001, 15.0, 10.0, 60.0, &candidate);

	int length = candidate.length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment)); ///

	//SmartDashboard::PutNumber("d.", 1);

	pathfinder_generate(&candidate, trajectory);

	//SmartDashboard::PutNumber("fk.", 1);

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

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

	/*	SmartDashboard::PutString("PLS", "y");

	 int POINT_LENGTH = 3;

	 Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

	 Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
	 Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
	 Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
	 points[0] = p1;
	 points[1] = p2;
	 points[2] = p3;

	 TrajectoryCandidate candidate;
	 pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

	 int length = candidate.length;
	 Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));

	 SmartDashboard::PutString("HELL", "y");

	 pathfinder_generate(&candidate, trajectory);

	 SmartDashboard::PutString("WISE MAN", "manny");

	 Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	 Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	 double wheelbase_width = 0.6;

	 pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

	 // Do something with the trajectories...

	 SmartDashboard::PutString("her", "y");

	 int i;
	 for (i = 0; i < length; i++) {
	 Segment s = trajectory[i];

	 SmartDashboard::PutNumber("time step", s.dt);
	 //	       printf("Time Step: %f\n", s.dt);
	 //	       printf("Coords: (%f, %f)\n", s.x, s.y);
	 //	       printf("Position (Distance): %f\n", s.position);
	 //	       printf("Velocity: %f\n", s.velocity);
	 //	       printf("Acceleration: %f\n", s.acceleration);
	 //	       printf("Jerk (Acceleration per Second): %f\n", s.jerk);
	 //	       printf("Heading (radians): %f\n", s.heading);
	 }

	 free(trajectory);
	 free(leftTrajectory);
	 free(rightTrajectory); */
//
//	    // Do something with the trajectories...
//
	/*
	 //	modules = RobotMap::tigerSwerve->GetModules();

	 Waypoint points[POINT_LENGTH];
	 Waypoint p1 = {0, 0, d2r(0)};
	 Waypoint p2 = {0, 1, d2r(0)}; //x, y, angle

	 //Waypoint p3 = {10, -4, d2r(0)};

	 points[0] = p1;
	 points[1] = p2;

	 //points[2] = p3;

	 int trajLength = pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VEL, MAX_ACCEL, MAX_JERK, &candidate);

	 //std::cout << "trajLength: " << trajLength << "\n";

	 length = candidate.length;
	 trajectory = (Segment*)malloc(length * sizeof(Segment));

	 SmartDashboard::PutString("made", "yeah");
	 pathfinder_generate(&candidate, trajectory);

	 //Modify for swerve
	 lTraj = (Segment*)malloc(length * sizeof(Segment));
	 rTraj = (Segment*)malloc(length * sizeof(Segment));
	 //blTraj = (Segment*)malloc(length * sizeof(Segment));
	 //brTraj = (Segment*)malloc(length * sizeof(Segment));

	 //SWERVE_MODE mode = SWERVE_DEFAULT;

	 //pathfinder_modify_swerve(trajectory, length, flTraj, frTraj, blTraj, brTraj, WHEELBASE_WIDTH, WHEELBASE_LENGTH, mode);

	 pathfinder_modify_tank(trajectory, length, lTraj, rTraj, WHEELBASE_WIDTH);

	 FILE* fp = fopen("/home/lvuser/myfile.csv", "w");
	 pathfinder_serialize_csv(fp, trajectory, length);
	 fclose(fp);
	 */

}

void DriveForward::SetDriveRefs() {

	drive_controller->SetRefs(full_refs);
	drive_controller->StartAutonThread();

}

