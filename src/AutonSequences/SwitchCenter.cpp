/*
 * Switch.cpp
 *
 *  Created on: Feb 28, 2018
 *      Author: DriversStation
 */

//Center Switch
#include <AutonSequences/SwitchCenter.h>

int switch_len = 0; //first place switch, we start auton with a cube in
int added_move_to_switch_len = 0;
int added_get_switch_len = 0;
int added_back_up_len = 0;
int added_score_switch_len = 0;

Timer *timerPauseSwitch = new Timer();

std::vector<std::vector<double> > full_refs_sw(1500, std::vector<double>(6)); //initalizes each index value to 0

void SwitchCenter::GenerateSwitch(bool left, bool added_switch) { //left center right //left is positive for x and for angle

	//Auton thread started in auton constructor

	int POINT_LENGTH = 3;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (left) {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {6.0, 4.0, d2r(20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
		p3 = {9.5, 5.0, d2r(0)}; //cannot just move in Y axis because of spline math
	}
	else {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {6.0, -3.0, d2r(-20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
		p3 = {9.5, -4.0, d2r(0)}; //cannot just move in Y axis because of spline math //CENTER STARTS CLOSER TO THE RIGHT //3.5
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, TODO:jerk time_step_auton

	length = candidate.length;
	switch_len = length;
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

		full_refs_sw.at(l).at(0) = ((double) sl.heading);
		full_refs_sw.at(l).at(1) = ((double) sl.position);
		full_refs_sw.at(l).at(2) = ((double) sr.position);
		full_refs_sw.at(l).at(3) = (0.0);
		full_refs_sw.at(l).at(4) = ((double) sl.velocity);
		full_refs_sw.at(l).at(5) = ((double) sr.velocity);

		if (l >= switch_len) {
			if (added_switch) {
				MoveToAddedSwitch(left);
				zeroing_indeces.push_back(switch_len);
				break; //TODO: make sure there are breaks
			} else {
				full_refs_sw.at(l).at(0) = full_refs_sw.at(l - 1).at(0);
				full_refs_sw.at(l).at(1) = full_refs_sw.at(l - 1).at(1);
				full_refs_sw.at(l).at(2) = full_refs_sw.at(l - 1).at(2);
				full_refs_sw.at(l).at(3) = full_refs_sw.at(l - 1).at(3);
				full_refs_sw.at(l).at(4) = full_refs_sw.at(l - 1).at(4);
				full_refs_sw.at(l).at(5) = full_refs_sw.at(l - 1).at(5);
			}
		}
	}

	drive_controller->SetZeroingIndex(zeroing_indeces);
	drive_controller->SetRefs(full_refs_sw);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void SwitchCenter::MoveToAddedSwitch(bool left) { //must zero profile, need to not carry over old orientation and pause before continuing //must also zero encoders

	int POINT_LENGTH = 3;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (left) {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-6.0, -4.0, d2r(-20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
		p3 = {-7.5, -5.0, d2r(-0)}; //cannot just move in Y axis because of spline math
	} else {
		p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
		p2 = {-6.0, 3.0, d2r(20.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0
		p3 = {-7.5, 4.0, d2r(0)}; //cannot just move in Y axis because of spline math //CENTER STARTS CLOSER TO THE RIGHT //3.5
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step //TODO:make time step global

	length = candidate.length;
	added_move_to_switch_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (switch_len)]; //start at beginning of new profile
		Segment sr = rightTrajectory[i - (switch_len)];

		full_refs_sw.at(i).at(0) = ((double) sl.heading) - PI;
		full_refs_sw.at(i).at(1) = -1.0 * ((double) sl.position);
		full_refs_sw.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sw.at(i).at(3) = (0.0);
		full_refs_sw.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sw.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= (switch_len + added_move_to_switch_len)) { //still have more in the 1500 allotted points
			GetAddedSwitch(left);
			zeroing_indeces.push_back(switch_len + added_move_to_switch_len);
			break;
		}

	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void SwitchCenter::GetAddedSwitch(bool left) {

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
	p2 = {2.0, 0.2, d2r(0.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step //TODO:make time step global

	length = candidate.length;
	added_get_switch_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (switch_len + added_move_to_switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (switch_len + added_move_to_switch_len)]; //start at beginning of new profile
		Segment sr = rightTrajectory[i - (switch_len + added_move_to_switch_len)];

		full_refs_sw.at(i).at(0) = ((double) sl.heading);
		full_refs_sw.at(i).at(1) = ((double) sl.position);
		full_refs_sw.at(i).at(2) = ((double) sr.position);
		full_refs_sw.at(i).at(3) = (0.0);
		full_refs_sw.at(i).at(4) = ((double) sl.velocity);
		full_refs_sw.at(i).at(5) = ((double) sr.velocity);

		if (i >= (switch_len + added_move_to_switch_len + added_get_switch_len)) { //still have more in the 1500 allotted points
			BackUp(left);
			zeroing_indeces.push_back(switch_len + added_get_switch_len + added_get_switch_len);
			break;
		}

	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void SwitchCenter::BackUp(bool left) {

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	p1 = {0.0, 0.0, 0.0}; //starting position may not be allowed to be 0,0,0 // Y, X, YAW
	p2 = {-2.0, 0.2, d2r(0.0)}; //3.0, 10.0, d2r(90)}; //-3.25 //9.0

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step //TODO:make time step global

	length = candidate.length;
	added_back_up_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (switch_len + added_move_to_switch_len + added_get_switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (switch_len + added_move_to_switch_len + added_get_switch_len)]; //start at beginning of new profile
		Segment sr = rightTrajectory[i - (switch_len + added_move_to_switch_len + added_get_switch_len)];

		full_refs_sw.at(i).at(0) = ((double) sl.heading) - PI;
		full_refs_sw.at(i).at(1) = -1.0 * ((double) sl.position);
		full_refs_sw.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sw.at(i).at(3) = (0.0);
		full_refs_sw.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sw.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= (switch_len + added_move_to_switch_len + added_get_switch_len + added_back_up_len)) { //still have more in the 1500 allotted points
			PlaceAddedSwitch(left);
			zeroing_indeces.push_back(switch_len + added_get_switch_len + added_get_switch_len + added_back_up_len);
			break;
		}

	}

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);


}

void SwitchCenter::PlaceAddedSwitch(bool left) { //TODO: backwards refs

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	if (left) { //FIX
		p1 = {0.0, 0.0, 0.0};
		p2 = {2.0, -1.0, d2r(50.0)};
	} else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {1.0, 2.0, d2r(0.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 8.0, 4.0, 100000.0, &candidate); //max vel, acc, jerk //profile speed must equal drive thread time step //TODO:make time step global

	length = candidate.length;
	added_score_switch_len = length;

	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (switch_len + added_move_to_switch_len + added_get_switch_len + added_back_up_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (switch_len + added_move_to_switch_len + added_get_switch_len + added_back_up_len)]; //start at beginning of new profile
		Segment sr = rightTrajectory[i - (switch_len + added_move_to_switch_len + added_get_switch_len + added_back_up_len)];

		full_refs_sw.at(i).at(0) = ((double) sl.heading); //positive
		full_refs_sw.at(i).at(1) = ((double) sl.position);
		full_refs_sw.at(i).at(2) = ((double) sr.position);
		full_refs_sw.at(i).at(3) = (0.0);
		full_refs_sw.at(i).at(4) = ((double) sl.velocity);
		full_refs_sw.at(i).at(5) = ((double) sr.velocity);

		if (i >= (switch_len + added_move_to_switch_len + added_get_switch_len + added_back_up_len + added_score_switch_len)) { //still have more in the 1500 allotted points
			full_refs_sw.at(i).at(0) = full_refs_sw.at(i - 1).at(0); //i - 1 will always be the last sensible value sinwe it waswades
			full_refs_sw.at(i).at(1) = full_refs_sw.at(i - 1).at(1);
			full_refs_sw.at(i).at(2) = full_refs_sw.at(i - 1).at(2);
			full_refs_sw.at(i).at(3) = full_refs_sw.at(i - 1).at(3);
			full_refs_sw.at(i).at(4) = full_refs_sw.at(i - 1).at(4);
			full_refs_sw.at(i).at(5) = full_refs_sw.at(i - 1).at(5);
		}

	}

	SmartDashboard::PutNumber("all together",
			switch_len + added_get_switch_len + added_score_switch_len);
	SmartDashboard::PutNumber("just the first", switch_len);
	SmartDashboard::PutNumber("just the middle", added_get_switch_len);
	SmartDashboard::PutNumber("just the last", added_score_switch_len);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void SwitchCenter::RunStateMachine(bool *place_switch) {

	//start being true at end of drive profile, stop being true once start shooting
	if (drive_controller->GetDriveIndex() >= switch_len
			&& auton_state_machine->shoot_counter == 0) {
		*place_switch = true;
	} else {
		*place_switch = false;
	}

}

void SwitchCenter::RunStateMachineTwo(bool *place_switch,
		bool *get_cube_ground) { //TODO: have the drive wait until postintakestate configuration is there //add in changes in scale-switch

	int drive_index = drive_controller->GetDriveIndex();

	if (auton_state_machine->state_a
			== auton_state_machine->PLACE_SWITCH_STATE_A_H
			|| (auton_state_machine->shoot_counter == 1
					&& elevator_->GetElevatorPosition() > 0.3)
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index >= (switch_len + added_get_switch_len)
					&& auton_state_machine->shoot_counter == 1)) { //will stop until releases cube
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if ((drive_controller->GetDriveIndex() >= switch_len
			&& auton_state_machine->shoot_counter == 0)
			|| (drive_controller->GetDriveIndex()
					>= (switch_len + added_get_switch_len
							+ added_score_switch_len)
					&& auton_state_machine->shoot_counter == 1)) {
		*place_switch = true;
	} else {
		*place_switch = false;
	}

	if (auton_state_machine->shoot_counter == 1
			&& drive_controller->GetDriveIndex()
					>= ((switch_len + added_get_switch_len) * 0.9)) {
		*get_cube_ground = true;
	} else {
		*get_cube_ground = false;
	}

	//may need if arm is low enough and in get cube ground, stop the drive

}
