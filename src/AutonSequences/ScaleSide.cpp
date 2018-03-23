/*
 * Scale.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

#include <AutonSequences/ScaleSide.h>

int scale_traj_len = 0;
int added_switch_len = 0;
int added_scale_len = 0;

std::vector<std::vector<double> > full_refs_sc(1500, std::vector<double>(6)); //initalizes each index value to 0, depends on only needing 1500 points: one every 10 ms, should only be using 300 since actually using a 50 ms time step, but we may change the time step

//Timer *timerPauseScale = new Timer(); //may need later

void ScaleSide::GenerateScale(bool left_scale, bool switch_, bool left_switch, bool added_scale, bool left_added_scale) { //true, false, true, true, true //direction on the switch needs to be accurate, but switch_ can be false

	//FC FORWARD TO PLACE ON SCALE BACKWARDS, robot starts backwards

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (left_scale) {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-19.8, 4.0, d2r(-20.0)}; //yaw is still from the robot's perspective
	//	p3 = {-19.8, 6.2, d2r(0.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-19.8, -4.0, d2r(20.0)}; //TODO: change this
	//	p3 = {-19.8, -6.2, d2r(0.0)};
	}

	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 17.0, 8.0, 100000.0, &candidate); //had to be slowed down

	length = candidate.length;
	scale_traj_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	int i;
	for (i = 0; i < 1500; i++) {

		Segment sl = leftTrajectory[i];
		Segment sr = rightTrajectory[i];

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position); //pathfinder does not give negative references
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= length) { //still have more in the 1500 allotted points, finished putting in the points to get to the place backwards position
			if (switch_ || added_scale) {
				GenerateAddedSwitch(left_switch, added_scale, left_added_scale); //this function will finish off 1500 points //added scale goes through switch first //true, true, true
				break;
			} else { //fill the rest with the last point to just stay there
				full_refs_sc.at(i).at(0) = full_refs_sc.at(i - 1).at(0); //l - 1 will always be the last sensible value since it cascades through the vector
				full_refs_sc.at(i).at(1) = full_refs_sc.at(i - 1).at(1);
				full_refs_sc.at(i).at(2) = full_refs_sc.at(i - 1).at(2);
				full_refs_sc.at(i).at(3) = full_refs_sc.at(i - 1).at(3);
				full_refs_sc.at(i).at(4) = full_refs_sc.at(i - 1).at(4);
				full_refs_sc.at(i).at(5) = full_refs_sc.at(i - 1).at(5);
			}
		}
	}

	//std::cout << "scale profile" << std::endl;

	drive_controller->SetRefs(full_refs_sc);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);
}

void ScaleSide::GenerateAddedSwitch(bool left_switch, bool added_scale, bool left_added_scale) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

	//PLACE SWITCH POSITION

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

//feet
	if (left_switch) {
		p1 = {0.0, 0.0, 0.0}; //Y, X, yaw
		p2 = {5.7, 1.25, d2r(20.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {5.7, -1.25, d2r(20.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //TODO: update time step

	length = candidate.length;
	added_switch_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (scale_traj_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (scale_traj_len)]; //starting from the first point in the new trajectory
		Segment sr = rightTrajectory[i - (scale_traj_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading); //regular forward, no need to reverse
		full_refs_sc.at(i).at(1) = ((double) sl.position);
		full_refs_sc.at(i).at(2) = ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = ((double) sr.velocity);

		if (i >= (scale_traj_len + added_switch_len)) { //still have more points left after placing on scale backwards and placing switch
			if (added_scale) {
				GenerateAddedScale(left_added_scale);
				break; //generateAddedScale will finish off the 1500 points itself
			} else {
				full_refs_sc.at(i).at(0) = full_refs_sc.at(i - 1).at(0); //i - 1 will always be the last sensible value since it cascades
				full_refs_sc.at(i).at(1) = full_refs_sc.at(i - 1).at(1);
				full_refs_sc.at(i).at(2) = full_refs_sc.at(i - 1).at(2);
				full_refs_sc.at(i).at(3) = full_refs_sc.at(i - 1).at(3);
				full_refs_sc.at(i).at(4) = full_refs_sc.at(i - 1).at(4);
				full_refs_sc.at(i).at(5) = full_refs_sc.at(i - 1).at(5);
			}

		}

	}

	drive_controller->SetZeroingIndex(scale_traj_len);

	free(trajectory); //need to free malloc'd elements
	free(leftTrajectory);
	free(rightTrajectory);

}

void ScaleSide::GenerateAddedScale(bool left) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

	//GO BACK TO PLACE SCALE BACKWARDS POSITION

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

//feet
	if (left) {
		p1 = {0.0, 0.0, 0.0}; //Y, X, yaw //just reversed all of these points
		p2 = {-5.7, -1.25, d2r(-20.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-5.7, 1.25, d2r(-20.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate);

	length = candidate.length;
	added_scale_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory,
			rightTrajectory, wheelbase_width);

	for (int i = (scale_traj_len + added_switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (scale_traj_len + added_switch_len)]; //starting from the first point in the new trajectory
		Segment sr = rightTrajectory[i - (scale_traj_len + added_switch_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //need to reverse
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position);
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= (scale_traj_len + added_switch_len + added_scale_len)) { //still have more points left after placing on scale backwards twice
			full_refs_sc.at(i).at(0) = full_refs_sc.at(i - 1).at(0); //i - 1 will always be the last sensible value since it cascades
			full_refs_sc.at(i).at(1) = full_refs_sc.at(i - 1).at(1);
			full_refs_sc.at(i).at(2) = full_refs_sc.at(i - 1).at(2);
			full_refs_sc.at(i).at(3) = full_refs_sc.at(i - 1).at(3);
			full_refs_sc.at(i).at(4) = full_refs_sc.at(i - 1).at(4);
			full_refs_sc.at(i).at(5) = full_refs_sc.at(i - 1).at(5);

		}

	}

	free(trajectory); //need to free malloc'd elements
	free(leftTrajectory);
	free(rightTrajectory);

	drive_controller->SetZeroingIndex(scale_traj_len + added_switch_len);

}

//USED FOR BOTH SCALE AND SCALE/SWITCH
void ScaleSide::RunStateMachineScaleSwitch(bool *place_scale_backwards, bool *place_switch,
		bool *get_cube_ground) {

//no other state machine booleans needed, all other ones will stay false

	if (drive_controller->GetDriveIndex() >= (scale_traj_len - (scale_traj_len/2.0))) { //robot is at position to place scale backwards
		if(drive_controller->GetDriveIndex() >= scale_traj_len) {
			drive_controller->StopProfile(true);
		}
		if (!StartedShoot()) { //still need to change this to be reusable
			*place_scale_backwards = true; //needs to go back to being false
		} else {
			*place_scale_backwards = false;
		}
		if (intake_->ReleasedCube()) {
			*get_cube_ground = true;
		}

		if (drive_controller->GetDriveIndex()
				>= (scale_traj_len + added_switch_len)
				&& added_switch_len > 0) { //if at end of profile, and added profile exists
			*place_switch = true;
			//place_scale_backwards = true;
		}
	}

}

void ScaleSide::RunStateMachineScaleScale(bool *place_scale_backwards,
		bool *get_cube_ground) {

//no other state machine booleans needed, all other ones will stay false

	if (drive_controller->GetDriveIndex() >= scale_traj_len) { //robot is at position to place scale backwards
		if (!StartedShoot()) { //still need to change this to be reusable
			*place_scale_backwards = true; //needs to go back to being false
		} else {
			*place_scale_backwards = false;
		}
		if (intake_->ReleasedCube()) {
			*get_cube_ground = true;
		}

		if (drive_controller->GetDriveIndex()
				>= (scale_traj_len + added_switch_len + added_scale_len)
				&& added_switch_len > 0) { //if at end of profile, and added profile exists
			*place_scale_backwards = true;
		}
	}

}
