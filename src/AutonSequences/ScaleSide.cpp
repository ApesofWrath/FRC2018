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
int added_crossed_scale_len = 0;
int crossed_scale_len = 0;
int first_traj_len = 0;

std::vector<std::vector<double> > full_refs_sc(1500, std::vector<double>(6)); //initalizes each index value to 0, depends on only needing 1500 points: one every 10 ms, should only be using 300 since actually using a 50 ms time step, but we may change the time step

void ScaleSide::GenerateScale(bool left_start, bool switch_, bool left_switch,
		bool added_scale, bool left_added_scale) { //true, true, true, false, false //direction on the switch needs to be accurate, but switch_ can be false //**switch_ and added_scale refer to if we want the second cube we get to be for scale or switch

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	if (left_start) {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-22.5, 6.5, d2r(-35.0)}; //yaw is still from the robot's perspective
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-22.5, -6.5, d2r(35.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 17.0, 6.0, 100000.0, &candidate); //had to be slowed down

	length = candidate.length;
	scale_traj_len = length;
	first_traj_len = scale_traj_len;
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
				zeroing_indeces.push_back(scale_traj_len);
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

	drive_controller->SetZeroingIndex(zeroing_indeces);
	drive_controller->SetRefs(full_refs_sc);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);
}

void ScaleSide::GenerateCrossedScale(bool left_start, bool added_switch,
		bool left_switch, bool added_scale, bool left_added_scale) {

	int POINT_LENGTH = 5;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3, p4, p5;//, p6, p7;

	//feet
	if (left_start) { //will do the right scale
		p1 = { 0.0, 0.0, 0.0 };
		p2 = {-11.5, -2.0, d2r(-15.0)}; //have to pull back the y on this one too
		p3 = {-15.5, 15.5, d2r(-90.0)}; //16
		p4 = {-15.5, 16.2, d2r(-90.0)}; //18.2
		p5 = {-18.5, 18.5, d2r(-35.0)}; //19.5, -45
//		p6 = {-17.5, 21.0, d2r(-15.0)}; //17.5 //25
//		//p7 = {-16.0, 21.0, d2r(0.0)}; //{-18.0, 19.0, d2r(-10.0
//
//		p1 = {0.0, 0.0, 0.0};
//		p2 = {-12.0, -1.5, d2r(-25.0)}; //15
//		p3 = {-13.0, 15.5, d2r(-90.0)};
//		p4 = {-15.0, 17.5, d2r(-90.0)};
//		p5 = {-16.5, 20.5, d2r(-45.0)};
//		p6 = {-19.0, 19.5, d2r(0.0)}; //20 deg 2.2 back, -0.1
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-22.5, -6.5, d2r(35.0)};
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;
	points[3] = p4;
	points[4] = p5;
	//points[5] = p6;
	//points[6] = p7;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 14.0, 5.0, 10000000.0, &candidate); //had to be slowed down //17.0, 6.0

	length = candidate.length;
	crossed_scale_len = length;
	first_traj_len = crossed_scale_len;
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
//			if (added_switch || added_scale) {
//				zeroing_indeces.push_back(crossed_scale_len); //note that will need for opposite scale
//				GenerateAddedSwitch(left_switch, added_scale, left_added_scale); //this function will finish off 1500 points //added scale goes through switch first //true, true, true
//				break;
//			} else { //fill the rest with the last point to just stay there
//				full_refs_sc.at(i).at(0) = full_refs_sc.at(i - 1).at(0); //l - 1 will always be the last sensible value since it cascades through the vector
//				full_refs_sc.at(i).at(1) = full_refs_sc.at(i - 1).at(1);
//				full_refs_sc.at(i).at(2) = full_refs_sc.at(i - 1).at(2);
//				full_refs_sc.at(i).at(3) = full_refs_sc.at(i - 1).at(3);
//				full_refs_sc.at(i).at(4) = full_refs_sc.at(i - 1).at(4);
//				full_refs_sc.at(i).at(5) = full_refs_sc.at(i - 1).at(5);
//			}
			zeroing_indeces.push_back(crossed_scale_len);
			GenerateShootCrossedScale(left_start, added_switch, left_switch, added_scale, left_added_scale);
			break;
		}
	}

	//SmartDashboard::PutNumber("1st ZERO", zeroing_indeces.at(0));

	drive_controller->SetZeroingIndex(zeroing_indeces);
	drive_controller->SetRefs(full_refs_sc);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void ScaleSide::GenerateShootCrossedScale(bool left_start, bool added_switch, bool left_switch, bool added_scale, bool left_added_scale) {

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

//feet
	if (left_start) {
		p1 = { 0.0, 0.0, 0.0 }; //Y, X, yaw
		p2 = { -3.8, -1.0, d2r(50.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {5.65, -2.35, d2r(-10.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 17.0, 8.0, 100000.0, &candidate);

	length = candidate.length;
	added_crossed_scale_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (first_traj_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (first_traj_len)]; //starting from the first point in the new trajectory
		Segment sr = rightTrajectory[i - (first_traj_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //regular forward, no need to reverse
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position);
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= (first_traj_len + added_crossed_scale_len)) { //still have more points left after placing on scale backwards and placing switch
			if (added_scale) { //BROKEN
				zeroing_indeces.push_back(first_traj_len + added_crossed_scale_len);
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

	free(trajectory); //need to free malloc'd elements
	free(leftTrajectory);
	free(rightTrajectory);

}

void ScaleSide::GenerateAddedSwitch(bool left_switch, bool added_scale,
		bool left_added_scale) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

	//PLACE SWITCH POSITION

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

//feet
	if (left_switch) {
		p1 = {0.0, 0.0, 0.0}; //Y, X, yaw
		p2 = {5.65, 2.35, d2r(10.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {5.65, -2.35, d2r(-10.0)};
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 17.0, 8.0, 100000.0, &candidate); //TODO: update time step

	length = candidate.length;
	added_switch_len = length;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (first_traj_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (first_traj_len)]; //starting from the first point in the new trajectory
		Segment sr = rightTrajectory[i - (first_traj_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading); //regular forward, no need to reverse
		full_refs_sc.at(i).at(1) = ((double) sl.position);
		full_refs_sc.at(i).at(2) = ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = ((double) sr.velocity);

		if (i >= (first_traj_len + added_switch_len)) { //still have more points left after placing on scale backwards and placing switch
			if (added_scale) {
				zeroing_indeces.push_back(first_traj_len + added_switch_len);
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
		p2 = {-4.65, -2.35, d2r(-15.0)};
	}
	else {
		p1 = {0.0, 0.0, 0.0};
		p2 = {-4.25, 1.25, d2r(60.0)};
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

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	for (int i = (first_traj_len + added_switch_len); i < 1500; i++) { //starting from the next point, right after the pathfinder trajectory ends

		Segment sl = leftTrajectory[i - (first_traj_len + added_switch_len)]; //starting from the first point in the new trajectory
		Segment sr = rightTrajectory[i - (first_traj_len + added_switch_len)];

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //need to reverse
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position);
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= (first_traj_len + added_switch_len + added_scale_len)) { //still have more points left after placing on scale backwards twice
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

}

//TODO: change state machines for first_traj_len if needed

//1-scale, 1-switch
void ScaleSide::RunStateMachineScaleSwitch(bool *place_scale_backwards, //state machine works but does not drive forward after it gets a cube - that's right
		bool *place_switch, bool *get_cube_ground) { //switch and post-intake

//no other state machine booleans needed, all other ones will stay false

	int drive_index = drive_controller->GetDriveIndex();

	SmartDashboard::PutNumber("total indeces", //went through whole profile without shooting the second cube
			added_switch_len + scale_traj_len);
	SmartDashboard::PutNumber("1", scale_traj_len);
	SmartDashboard::PutNumber("2", added_switch_len);
	//SmartDashboard::PutNumber("3", added_scale_len);
	SmartDashboard::PutNumber("index", drive_index); //maybe can't call getindex more than onc

	std::cout << "scale switch state machine" << std::endl;

	//added check for state to stop profile
	if (((drive_index >= scale_traj_len
			&& auton_state_machine->shoot_counter == 0)
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator going down
			&& auton_state_machine->shoot_counter == 1))
			|| auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H
			|| (intake_->GetAngularPosition() <= 0.3 //TODO: why is this here
					&& auton_state_machine->state_a
							== auton_state_machine->GET_CUBE_GROUND_STATE_A_H)
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index >= (scale_traj_len + added_switch_len)
					&& auton_state_machine->shoot_counter == 1)) { //for shooting first cube, for waiting for elev/arm to come back down to get ready to get the second cube... a possibly redundant case, for getting the second cube
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= ((scale_traj_len + added_switch_len) / 1.5) //start placing once close enough to switch
	&& auton_state_machine->shoot_counter == 1) {
		*place_switch = true;
	} else {
		*place_switch = false;
	}

	if (drive_index >= (scale_traj_len / 3)) { //start moving superstructure on the way to the scale
		if (auton_state_machine->shoot_counter == 0) {
			*place_scale_backwards = true;
			if (std::abs(drive_controller->GetLeftVel()) < 0.5) { //only shoot when drive is slow enough, to not shoot around the world
				auton_state_machine->shoot_cube = true; //different from shoot_counter
				*get_cube_ground = true; //canNOT stay true for after it's needed
			} else {
				auton_state_machine->shoot_cube = false;
			}
		} else {
			*place_scale_backwards = false;
		}
		if (auton_state_machine->shoot_counter == 2) {
			*get_cube_ground = false;
		}
	}

}

//SAME SIDE SCALE
void ScaleSide::RunStateMachineScaleOnly(bool *place_scale_backwards,
		bool *get_cube_ground) { //arm/elev must go back down for the start of teleop, to not get caught on the scale

	int drive_index = drive_controller->GetDriveIndex();

	if (drive_index >= (scale_traj_len / 3)) { //start moving superstructure on the way
		if (drive_index >= scale_traj_len) { //drive profile refs should stay at the last index, at the scale position, anyway, but just for clarity
			drive_controller->StopProfile(true);
		} //no else
		if (auton_state_machine->shoot_counter == 0) {
			*place_scale_backwards = true; //needs to go back to being false so that after going through post_intake and staying in post_intake configuration, it stays in wfb_state
			if (std::abs(drive_controller->GetLeftVel()) < 0.5) {
				auton_state_machine->shoot_cube = true;
			} else {
				auton_state_machine->shoot_cube = false; //will start that slow, and need to reset to false during middle ofprofile
			}
		} else {
			*place_scale_backwards = false;
		}
//		if (intake_->ReleasedCube()) {
//			*get_cube_ground = true; //may change this
//		}
	}

}

//2-scale
//init, wfb, place_scale_backwards, post_intake_scale, wfb (just passing through), get_cube_ground_state, post_intake_switch, place_scale_backwards, post_intake_scale, wfb
//      psb(1)		gcg	, !psb																					psb(2)				!psb , !gcg
void ScaleSide::RunStateMachineScaleScale(bool *place_scale_backwards, //state machine works, but drive is wrong
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();

	SmartDashboard::PutNumber("total indeces.", //went through whole profile without shooting the second cube
			added_switch_len + scale_traj_len + added_scale_len);
	SmartDashboard::PutNumber("1.", scale_traj_len);
	SmartDashboard::PutNumber("2.", added_switch_len);
	SmartDashboard::PutNumber("3.", added_scale_len);
	SmartDashboard::PutNumber("index..", drive_index);

//no other state machine booleans needed, all other ones will stay false

	if ((drive_index >= scale_traj_len
			&& auton_state_machine->shoot_counter == 0)
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator going down
			&& auton_state_machine->shoot_counter == 1) //when shoot counter is 0, will be going up to shoot first cube and will not stop drive. once shot first cube and everything is coming down, will stop drive. once everything is coming back up, will stop drive
			|| auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index
					>= (scale_traj_len + added_switch_len + added_scale_len)
					&& auton_state_machine->shoot_counter == 1)) { //second case should not be needed, but just there //scale cube, was driving
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= (scale_traj_len / 3)) { //start moving superstructure

		if (auton_state_machine->shoot_counter == 0 || ((drive_index //if have not shot before, if at end of the total profile and there is that addded profile
		>= (scale_traj_len + added_switch_len + added_scale_len) //will need to divide by 2
		) && auton_state_machine->shoot_counter == 1)) {

			*place_scale_backwards = true; //needs to go back to being false

			if (std::abs(drive_controller->GetLeftVel()) < 0.5) {
				auton_state_machine->shoot_cube = true;
			} else {
				auton_state_machine->shoot_cube = false; //will start that slow, and need to reset to false during middle ofprofile
			}

		} else {
			*place_scale_backwards = false; //have shot the first one, but drive has not gotten to the position to shoot the second one
		}

		if (auton_state_machine->shoot_counter == 1) { //if we have shot twice, then don't get more cubes
			*get_cube_ground = true;
		} else {
			*get_cube_ground = false;
		}
	}

}

//OPPOSITE SIDE SCALE
void ScaleSide::RunStateMachineScaleSideOnly(bool *place_scale_backwards,
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();
//crossed scale len
	if (drive_index >= (300)) { //start moving superstructure on the way
		if (drive_index >= (crossed_scale_len + added_crossed_scale_len)) { //drive profile refs should stay at the last index, at the scale position, anyway, but just for clarity
			drive_controller->StopProfile(true);
		} //no else
		if (auton_state_machine->shoot_counter == 0) {
			*place_scale_backwards = true; //needs to go back to being false so that after going through post_intake and staying in post_intake configuration, it stays in wfb_state
			if (std::abs(drive_controller->GetLeftVel()) < 0.5) {
				auton_state_machine->shoot_cube = true;
			} else {
				auton_state_machine->shoot_cube = false; //will start that slow, and need to reset to false during middle ofprofile
			}
		} else {
			*place_scale_backwards = false;
		}
//		if (intake_->ReleasedCube()) {
//			*get_cube_ground = true; //may change this
//		}
	}

}

void ScaleSide::RunStateMachineCrossedScaleScale(bool *place_scale_backwards,
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();

//no other state machine booleans needed, all other ones will stay false

	if ((drive_index >= crossed_scale_len
			&& auton_state_machine->shoot_counter == 0)
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator going down
			&& auton_state_machine->shoot_counter == 1) //when shoot counter is 0, will be going up to shoot first cube and will not stop drive. once shot first cube and everything is coming down, will stop drive. once everything is coming back up, will stop drive
			|| auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index
					>= (crossed_scale_len + added_switch_len + added_scale_len)
					&& auton_state_machine->shoot_counter == 1)) { //second case should not be needed, but just there //scale cube, was driving
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= (crossed_scale_len * 0.8)) { //start moving superstructure

		if (auton_state_machine->shoot_counter == 0 || ((drive_index //if have not shot before, if at end of the total profile and there is that addded profile
		>= (crossed_scale_len + added_switch_len + added_scale_len) //will need to divide by 2
		) && auton_state_machine->shoot_counter == 1)) {

			*place_scale_backwards = true; //needs to go back to being false

			if (std::abs(drive_controller->GetLeftVel()) < 0.5) {
				auton_state_machine->shoot_cube = true;
			} else {
				auton_state_machine->shoot_cube = false; //will start that slow, and need to reset to false during middle ofprofile
			}

		} else {
			*place_scale_backwards = false; //have shot the first one, but drive has not gotten to the position to shoot the second one
		}

		if (auton_state_machine->shoot_counter == 1) { //if we have shot twice, then don't get more cubes
			*get_cube_ground = true;
		} else {
			*get_cube_ground = false;
		}
	}

}

