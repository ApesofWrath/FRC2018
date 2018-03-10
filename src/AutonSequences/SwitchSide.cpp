/*
 * SwitchSide.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/SwitchSide.h>

int back_switch_len = 0;
int forward_switch_len = 0;

std::vector<std::vector<double> > full_refs_sw_s(1500, std::vector<double>(6)); //initalizes each index value to 0

Timer *timerPause = new Timer();

void SwitchSide::GenerateSwitchSide(bool left) { //left center right //left is positive for x and for angle //TODO: make center switch, side switch subclasses

	//Auton thread started in auton constructor

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (left) {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-20.0, 0.0, d2r(45.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
	}
	else {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-20.0, 0.0, d2r(-45.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.05, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk

	length = candidate.length;
	back_switch_len = length;
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

		full_refs_sw_s.at(l).at(0) = ((double) sl.heading) - PI;
		full_refs_sw_s.at(l).at(1) = -1.0 * ((double) sl.position);
		full_refs_sw_s.at(l).at(2) = -1.0 * ((double) sr.position);
		full_refs_sw_s.at(l).at(3) = (0.0);
		full_refs_sw_s.at(l).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sw_s.at(l).at(5) = -1.0 * ((double) sr.velocity);

		if (l >= length) { //still have more in the 1500 allotted points
			ForwardSwitch(left); //will finish off 1500 points
			break;
		}

		//SmartDashboard::PutNumber("length", length);

		//always zero for single switch side
		drive_controller->SetZeroingIndex(back_switch_len); //DONT DRIVE WHILE SHOOTING

	//	timerPause->Start();
	//	if(timerPause->HasPeriodPassed(3)) {
			drive_controller->SetRefs(full_refs_sw_s);
	//	}
			//timerPause->Stop();

		free(trajectory);
		free(leftTrajectory);
		free(rightTrajectory);

	}

}

void SwitchSide::ForwardSwitch(bool left) { //must zero profile

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	if(left) {
		p1 = { 0.0, 0.0, 0.0 };
		p2 = { 5.0, 0.1, d2r(10.0) }; //10
	} else {
		p1 = { 0.0, 0.0, 0.0 };
		p2 = { 5.0, -0.1, d2r(-10.0) }; //10
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.05, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step

	int length = candidate.length;
	forward_switch_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (back_switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (back_switch_len)]; //start at beginning of new profile
		Segment sr = rightTrajectory[i - (back_switch_len)];

		full_refs_sw_s.at(i).at(0) = ((double) sl.heading); //positive
		full_refs_sw_s.at(i).at(1) = ((double) sl.position);
		full_refs_sw_s.at(i).at(2) = ((double) sr.position);
		full_refs_sw_s.at(i).at(3) = (0.0);
		full_refs_sw_s.at(i).at(4) = ((double) sl.velocity);
		full_refs_sw_s.at(i).at(5) = ((double) sr.velocity);

		if (i >= (back_switch_len + forward_switch_len)) { //still have more in the 1500 allotted points
			full_refs_sw_s.at(i).at(0) = full_refs_sw_s.at(i - 1).at(0); //i - 1 will always be the last sensible value since it cascades
			full_refs_sw_s.at(i).at(1) = full_refs_sw_s.at(i - 1).at(1);
			full_refs_sw_s.at(i).at(2) = full_refs_sw_s.at(i - 1).at(2);
			full_refs_sw_s.at(i).at(3) = full_refs_sw_s.at(i - 1).at(3);
			full_refs_sw_s.at(i).at(4) = full_refs_sw_s.at(i - 1).at(4);
			full_refs_sw_s.at(i).at(5) = full_refs_sw_s.at(i - 1).at(5);

		}

	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void SwitchSide::RunStateMachineSide(bool *place_switch) {

	//no other state machine booleans needed, all other ones will stay false

	//start being true at end of drive profile, stop being true once start shooting
	if (GetIndex() >= (back_switch_len + forward_switch_len) && !StartedShoot()) { //at the end of the drive, while we have not released a cube //GetIndex() >= length && //should be has started shooting
		*place_switch = true; //must run once initialized!
	} else {
		*place_switch = false;
	}

}

