/*
 * Scale.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/Scale.h>

int scale_traj_len = 0;
int added_switch_len = 0;
std::vector<std::vector<double> > full_refs_sc(1500, std::vector<double>(6)); //initalizes each index value to 0

void Scale::GenerateScale(bool left_scale, bool switch_, bool left_switch) { //left center right //left is positive for x and for angle //bool switch too

	//Auton thread started in auton constructor

	SmartDashboard::PutString("made it", "no");

	int POINT_LENGTH = 3;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (left_scale) { //start left, left scale
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-10.5, -2.0, d2r(0.0)}; //3.0, 10.0, d2r(90)}; //-
		p3 = {-21.3, 5.7, d2r(0.0)}; //cannot just move in Y axis because of spline math
		//6.0 forward, 1.0 left, 20 left
	}
	else {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-10.5, 2.0, d2r(0.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
		p3 = {-21.3, -5.7, d2r(0.0)}; //cannot just move in Y axis because of spline math
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.05, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk

	length = candidate.length;
	scale_traj_len = length;
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

		full_refs_sc.at(l).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards
		full_refs_sc.at(l).at(1) = -1.0 * ((double) sl.position);
		full_refs_sc.at(l).at(2) = -1.0 * ((double) sr.position); //couldn't just reverse these
		full_refs_sc.at(l).at(3) = (0.0);
		full_refs_sc.at(l).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(l).at(5) = -1.0 *((double) sr.velocity);

//		full_refs_sc.at(l).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards
//		full_refs_sc.at(l).at(1) = -((double) sl.position);
//		full_refs_sc.at(l).at(2) = -((double) sr.position);
//		full_refs_sc.at(l).at(3) = (0.0);
//		full_refs_sc.at(l).at(4) = -((double) sl.velocity);
//		full_refs_sc.at(l).at(5) = -((double) sr.velocity);

		if (l >= length) { //still have more in the 1500 allotted points
			if (switch_) {
				drive_controller->ZeroAll(true);
				GenerateAddedSwitch(left_switch);
			} else {
				full_refs_sc.at(l).at(0) = full_refs_sc.at(l - 1).at(0);
				full_refs_sc.at(l).at(1) = full_refs_sc.at(l - 1).at(1);
				full_refs_sc.at(l).at(2) = full_refs_sc.at(l - 1).at(2);
				full_refs_sc.at(l).at(3) = full_refs_sc.at(l - 1).at(3);
				full_refs_sc.at(l).at(4) = full_refs_sc.at(l - 1).at(4);
				full_refs_sc.at(l).at(5) = full_refs_sc.at(l - 1).at(5);
			}
		}
	}

	SmartDashboard::PutString("made it", "yep");
	std::cout << "MADE IT PAST CREATING PROFILES" << std::endl;

	FillProfile(full_refs_sc);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);
}

void Scale::GenerateAddedSwitch(bool left) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

//Auton thread started in auton constructor

//	std::vector<std::vector<double> > added_sw((1500 - scale_traj_len),
//			std::vector<double>(6)); //depends on profile lasting exactly 1500 points

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

//feet
	if (left) {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {4.4, 0.87, d2r(20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
	}
	else { //change these
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {4.4, -0.87, d2r(20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
	//	p3 = {9.5, -3.5, d2r(0)}; //cannot just move in Y axis because of spline math
	}

	points[0] = p1;
	points[1] = p2;
//points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.05, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk

	length = candidate.length;
	added_switch_len = length; //is 0
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (scale_traj_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (scale_traj_len)];
		Segment sr = rightTrajectory[i - (scale_traj_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading); //profile tries to turn robot around and go straight, in order to go backwards
		full_refs_sc.at(i).at(1) = ((double) sl.position);
		full_refs_sc.at(i).at(2) = ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = ((double) sr.velocity);
	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

//USED FOR BOTH SCALE AND SCALE/SWITCH
void Scale::RunStateMachine(bool *place_scale, bool *place_switch, bool *get_cube_ground) {

//no other state machine booleans needed, all other ones will stay false
	SmartDashboard::PutNumber("scale traj length", scale_traj_len);
	SmartDashboard::PutNumber("switch traj length", added_switch_len);

//start being true at end of drive profile, stop being true once start shooting
	if (GetIndex() >= scale_traj_len) { //at the end of the drive, while we have not released a cube //GetIndex() >= length && //should be has started shooting //IsCubeRelease is needed
		if (!StartedShoot()) { //still need to change this to be reusable
			*place_scale = true; //must run once initialized!
		}
		else {
			*place_scale = false;
		}
		if(IsCubeReleased()) {
			*get_cube_ground = true;
		}

		if(GetIndex() >= (scale_traj_len + added_switch_len) && added_switch_len > 0) { //if at end of profile, and added profile exists
			*place_switch = true;
		}
	}

}

