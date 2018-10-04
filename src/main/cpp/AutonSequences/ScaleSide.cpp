/*
 * Scale.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: DriversStation
 */

#include "ScaleSide.h"

int same_scale_len = 0;
int added_switch_len = 0;
int added_scale_len = 0;
int added_opp_scale_len = 0;
int opp_scale_len = 0;
int first_traj_len = 0;

bool left_scale;

std::vector<std::vector<double> > full_refs_sc(1500, std::vector<double>(6)); //initalizes each index value to 0, depends on only needing 1500 points: one every 10 ms, should only be using 300 since actually using a 50 ms time step, but we may change the time step

void ScaleSide::GenerateSameScale(bool is_left, bool added_switch,
		bool added_scale) { //direction on the switch needs to be accurate, but switch_ can be false //added_switch and added_scale refer to if we want the second cube we get to be for scale or switch

	int POINT_LENGTH = 3;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3;

	//feet
	if (is_left) { // left
		left_scale = true;
		p1 = {0.0, 0.0, 0.0};
		p2 = {-16.0, 0.0, d2r(0.0)}; //yaw is still from the robot's perspective
		p3 = {-21.5, 1.1, d2r(-10.0)};
	}
	else { // right
		left_scale = false;
		p1 = {0.0, 0.0, 0.0};
		p2 = {-16.0, 0.0, d2r(0.0)}; //yaw is still from the robot's perspective
		p3 = {-21.5, -1.1, d2r(10.0)};
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 11.0, 6.0, 100000.0, &candidate); //11

	length = candidate.length;
	same_scale_len = length;
	first_traj_len = same_scale_len;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 2.1;

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	int i;
	for (i = 0; i < 1500; i++) {

		Segment sl = rightTrajectory[i]; //yes, going backwards requires switching the left and right profiles to have appropriate magnitudes that correspond to the yaw targets!
		Segment sr = leftTrajectory[i];

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards?
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position); //pathfinder does not give negative references; it always assumes forward
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= length) { //still have more in the 1500 allotted points, finished putting in the points to get to the place backwards position
			if (added_switch || added_scale) {
				zeroing_indeces.push_back(same_scale_len);
				GenerateAddedSwitch(true, added_scale); //either for going to the switch and scoring there, or for going to the switch just to grab the 2nd cube
				break; //this function will finish off 1500 points
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

void ScaleSide::GenerateOppScale(bool left_start, bool added_switch,
		bool added_scale) {

	int POINT_LENGTH = 6;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2, p3, p4, p5, p6, p7;

	//feet
	if (left_start) { //right scale
		left_scale = false;
		p1 = {0.0, 0.0, 0.0}; //TODO: tweak these to perfection, remember to update the other side as well
		p2 = {-16.0, -1.2, d2r(-30.0)};
		p3 = {-17.8, 4.0, d2r(-90.0)};
		p4 = {-17.8, 13.0, d2r(-90.0)}; //shorter this x, tighter the turn
		p5 = {-20.8, 16.0, d2r(0.0)};
		p6 = {-22.5, 16.5, d2r(25.0)};
	}
	else { //left scale
		left_scale = true;
		p1 = {0.0, 0.0, 0.0};
		p2 = {-16.0, 1.2, d2r(30.0)}; //little out
		p3 = {-17.8, -4.0, d2r(90.0)}; //turn
		p4 = {-17.8, -13.0, d2r(90.0)}; //shoot gap
		p5 = {-20.8, -16.0, d2r(0.0)}; //end shooting gap
		p6 = {-22.5, -16.0, d2r(-25.0)}; //position to place
	}

	points[0] = p1;
	points[1] = p2;
	points[2] = p3;
	points[3] = p4;
	points[4] = p5;
	points[5] = p6;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_FAST, 0.02, 9.0, 7.0, 10000000.0, &candidate); //had to be slowed down //17.0, 6.0

	length = candidate.length;
	opp_scale_len = length;
	first_traj_len = opp_scale_len;
	Segment *trajectory = (Segment*) malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	double wheelbase_width = 1.8; //was messing with this to see if it would help, works as it is

	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);

	int i;
	for (i = 0; i < 1500; i++) {

		Segment sl = rightTrajectory[i]; //THE LEFT AND RIGHT TRAJECTORIES MUST BE SWITCHED WHEN GOING BACKWARDS. OTHERWISE, ANGLE WILL NOT CORRESPOND TO EACH SIDE'S MAGNITUDES. PATHFINDER ALWAYS ASSUMES FORWRAD TRAJECTORIES; IN ADDITION TO REVERSING (*-1.0 and -PI for yaw), WE MUST SWITCH THE LEFT AND RIGHT TRAJECTORIES
		Segment sr = leftTrajectory[i]; //magnitudes AND directions must reverse, to get mirror image

		full_refs_sc.at(i).at(0) = ((double) sl.heading) - PI; //profile tries to turn robot around and go straight, in order to go backwards
		full_refs_sc.at(i).at(1) = -1.0 * ((double) sl.position); //pathfinder does not give negative references
		full_refs_sc.at(i).at(2) = -1.0 * ((double) sr.position);
		full_refs_sc.at(i).at(3) = (0.0);
		full_refs_sc.at(i).at(4) = -1.0 * ((double) sl.velocity);
		full_refs_sc.at(i).at(5) = -1.0 * ((double) sr.velocity);

		if (i >= length) { //still have more in the 1500 allotted points, finished putting in the points to get to the place backwards position
			if (added_switch || added_scale) {
				zeroing_indeces.push_back(opp_scale_len); //note that will need for opposite scale
				GenerateAddedSwitch(false, added_scale); //this function will finish off 1500 points //added scale goes through switch first
				break;
			} else { //fill the rest with the last point to just stay there
				full_refs_sc.at(i).at(0) = full_refs_sc.at(i - 1).at(0); //l - 1 will always be the last sensible value since it cascades through the vector
				full_refs_sc.at(i).at(1) = full_refs_sc.at(i - 1).at(1);
				full_refs_sc.at(i).at(2) = full_refs_sc.at(i - 1).at(2);
				full_refs_sc.at(i).at(3) = full_refs_sc.at(i - 1).at(3);
				full_refs_sc.at(i).at(4) = full_refs_sc.at(i - 1).at(4);
				full_refs_sc.at(i).at(5) = full_refs_sc.at(i - 1).at(5);
			}
			break;
		}
	}

	drive_controller->SetZeroingIndex(zeroing_indeces);
	drive_controller->SetRefs(full_refs_sc);

	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);

}

void ScaleSide::GenerateAddedSwitch(bool same_side, bool added_scale //if doing scale-switch, just going to the switch to get another cube is the last trajectory needed
		) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	//WAYPOINTS ARE NO LONGER RELATIVE TO THE LAST POINT, THEY ARE CONTNUOUS. Last point of the previous profile MUST equal first point of the next profile.
	if (!left_scale) {
		if (same_side) {
			p1 = {-21.5, -1.1, d2r(10.0)}; //NOT tested; estimated according to same_side_left_scale
			p2 = {-16.3, -2.0, d2r(0.0)};
			SmartDashboard::PutString("waypoints1", "same side right");
		} else {
			p1 = {-22.5, 16.5, d2r(25.0)}; //these are just points, pathfinder will give whatever trajectory just according to these points
			p2 = {-18.3, 16.2, d2r(-7.0)}; //if moving in x and have an angle that doesn't correspond to the needed theta, trajectory will be an s to turn the robot to the correct theta after moving to the correct x
			SmartDashboard::PutString("waypoints1", "opp side right");
		}
	}
	else { //left_scale
		if (same_side) {
			p1 = {-21.5, 1.1, d2r(-10.0)}; //1.1
			p2 = {-16.3, 2.6, d2r(0.0)}; //2.4
			SmartDashboard::PutString("waypoints1", "same side left");
		} else {
			p1 = {-22.5, -16.5, d2r(-25.0)};
			p2 = {-19.5, -16.0, d2r(0.0)};
			SmartDashboard::PutString("waypoints1", "opp side left");
		}
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 15.0, 7.0, 100000.0, &candidate);

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

		Segment sl = leftTrajectory[i - (first_traj_len)]; //starting from the first point in the new trajectory //not switched
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
				GenerateAddedScale(same_side);
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

	SmartDashboard::PutNumber("finished traj", 0);

	free(trajectory); //need to free malloc'd elements
	free(leftTrajectory);
	free(rightTrajectory);

}

void ScaleSide::GenerateAddedScale(bool same_side) { //new trajectory so that old spline interpolation does not carry over and new waypoints do not change old trajectory

	int POINT_LENGTH = 2;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1, p2;

	//feet
	//WAYPOINTS ARE NO LONGER RELATIVE TO THE LAST POINT, THEY ARE CONTNUOUS. Last waypoint of previous trajectory MUST equal the first waypoint of the next trajectory
	if (!left_scale) {
		if (same_side) {
			p1 = {-17.6, -2.0, d2r(0.0)}; //not tested
			p2 = {-21.9, -0.5, d2r(10.0)};
			SmartDashboard::PutString("waypoints2", "same side right");
		} else {
			p1 = {-18.3, 16.2, d2r(-7.0)}; //working on these
			p2 = {-23.0, 16.0, d2r(20.0)};
			SmartDashboard::PutString("waypoints2", "opp side right");
		}

	}
	else { //left_scale
		if (same_side) { //{-16.3, 2.4, d2r(0.0)};
			p1 = {-16.3, 2.8, d2r(0.0)}; //should be right, we tested these
			p2 = {-20.5, 1.4, d2r(0.0)}; //-21.5, 1.4, -10
			SmartDashboard::PutString("waypoints2", "same side left");
		} else {
			p1 = {-19.5, -17.0, d2r(0.0)}; //not tested
			p2 = {-23.0, -16.0, d2r(-20.0)};
			SmartDashboard::PutString("waypoints2", "opp side left");
		}
	}

	points[0] = p1;
	points[1] = p2;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, //always using cubic, to not go around the points so much
			PATHFINDER_SAMPLES_FAST, 0.02, 15.0, 7.0, 100000.0, &candidate);

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

		Segment sl = rightTrajectory[i - (first_traj_len + added_switch_len)]; //switched, yes!
		Segment sr = leftTrajectory[i - (first_traj_len + added_switch_len)];

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

void ScaleSide::RunStateMachineSameScaleSwitch(bool *place_scale_backwards, //does not drive forward after it gets the second cube
		bool *place_switch, bool *get_cube_ground) {

	//no other state machine booleans needed, all other ones will stay false

	int drive_index = drive_controller->GetDriveIndex();

//	SmartDashboard::PutNumber("total indeces", //went through whole profile without shooting the second cube
//			added_switch_len + scale_traj_len);
//	SmartDashboard::PutNumber("1", scale_traj_len);
//	SmartDashboard::PutNumber("2", added_switch_len);
//	//SmartDashboard::PutNumber("3", added_scale_len);
//	SmartDashboard::PutNumber("index", drive_index);

	//added check for state to stop profile
	if (((drive_index >= same_scale_len
			&& auton_state_machine->shoot_counter == 0) //in position to place scale
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator still going down after placing scale
			&& auton_state_machine->shoot_counter == 1))
			|| (auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H)
		//	|| (intake_->GetAngularPosition() <= 0.3 //wait until have cube
			//		&& auton_state_machine->state_a
				//			== auton_state_machine->GET_CUBE_GROUND_STATE_A_H)
			|| auton_state_machine->shoot_counter == 2 //after shooting both
			|| (drive_index >= (same_scale_len + added_switch_len) //in position for place switch
					&& auton_state_machine->shoot_counter == 1)) { //for shooting first cube, for waiting for elev/arm to come back down to get ready to get the second cube... a possibly redundant case, for getting the second cube
		drive_controller->StopProfile(true);
		SmartDashboard::PutNumber("stopped", 0);
	} else {
		drive_controller->StopProfile(false);
		SmartDashboard::PutNumber("started", 0);
	}

	if (drive_index >= ((same_scale_len + added_switch_len) / 1.5) //start placing once close enough to switch
	&& auton_state_machine->shoot_counter == 1) {
		*place_switch = true;
	} else {
		*place_switch = false;
	}

	if (drive_index >= (same_scale_len / 3)) { //start moving superstructure on the way to the scale
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

void ScaleSide::RunStateMachineOppScaleSwitch(bool *place_scale_backwards, bool *place_switch, bool *get_cube_ground) {

	//no other state machine booleans needed, all other ones will stay false

	int drive_index = drive_controller->GetDriveIndex();

//	SmartDashboard::PutNumber("total indeces", //went through whole profile without shooting the second cube
//			added_switch_len + scale_traj_len);
//	SmartDashboard::PutNumber("1", scale_traj_len);
//	SmartDashboard::PutNumber("2", added_switch_len);
//	//SmartDashboard::PutNumber("3", added_scale_len);
//	SmartDashboard::PutNumber("index", drive_index);

	//added check for state to stop profile
	if (((drive_index >= opp_scale_len
			&& auton_state_machine->shoot_counter == 0)
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator going down
			&& auton_state_machine->shoot_counter == 1))
			|| auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H
			|| (intake_->GetAngularPosition() <= 0.3 //TODO: why is this here
					&& auton_state_machine->state_a
							== auton_state_machine->GET_CUBE_GROUND_STATE_A_H)
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index >= (opp_scale_len + added_switch_len)
					&& auton_state_machine->shoot_counter == 1)) { //for shooting first cube, for waiting for elev/arm to come back down to get ready to get the second cube... a possibly redundant case, for getting the second cube
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= ((opp_scale_len + added_switch_len) / 1.5) //start placing once close enough to switch
	&& auton_state_machine->shoot_counter == 1) {
		*place_switch = true;
	} else {
		*place_switch = false;
	}

	if (drive_index >= (opp_scale_len / 3)) { //start moving superstructure on the way to the scale
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

void ScaleSide::RunStateMachineSameScale(bool *place_scale_backwards,
		bool *get_cube_ground) { //arm/elev must go back down for the start of teleop, to not get caught on the scale

	int drive_index = drive_controller->GetDriveIndex();

	if (drive_index >= same_scale_len) { //drive profile refs should stay at the last index, at the scale position, anyway, but just for clarity
		drive_controller->StopProfile(true);
	} //no else

	if (drive_index >= (same_scale_len)) { //start moving superstructure on the way
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
	}

}

void ScaleSide::RunStateMachineSameScaleScale(bool *place_scale_backwards, //state machine works, but drive is wrong
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();

	// SmartDashboard::PutNumber("total indeces.", //went through whole profile without shooting the second cube
	// 		added_switch_len + same_scale_len + added_scale_len);
	// SmartDashboard::PutNumber("1.", same_scale_len);
	// SmartDashboard::PutNumber("2.", added_switch_len);
	// SmartDashboard::PutNumber("3.", added_scale_len);
	// SmartDashboard::PutNumber("index..", drive_index);

//no other state machine booleans needed, all other ones will stay false

	if ((drive_index >= same_scale_len
			&& auton_state_machine->shoot_counter == 0)
			||
			(elevator_->GetElevatorPosition() > 0.3 && elevator_->elevator_state == elevator_->DOWN_STATE_E_H//elevator going down
			&& auton_state_machine->shoot_counter == 1) //when shoot counter is 0, will be going up to shoot first cube and will not stop drive. once shot first cube and everything is coming down, will stop drive. once everything is coming back up, will stop drive
			|| (auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H)
			|| (auton_state_machine->shoot_counter == 2)
			|| (drive_index
		>= (same_scale_len + added_switch_len)
		&& auton_state_machine->state_a == auton_state_machine->GET_CUBE_GROUND_STATE_A_H)) {
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= (same_scale_len / 1.5)) { //start moving superstructure

		if (auton_state_machine->shoot_counter == 0 || ((drive_index //if have not shot before, if at end of the total profile and there is that addded profile
		>= (same_scale_len + added_switch_len) //will need to divide by 2  ///FOR SAME SIDE + added_scale_len
		) && auton_state_machine->shoot_counter == 1)) {

			*place_scale_backwards = true; //needs to go back to being false

			if (((std::abs(drive_controller->GetLeftVel()) < 0.5)
					&& auton_state_machine->shoot_counter == 0)
					|| (auton_state_machine->shoot_counter == 1
							&& (std::abs(drive_controller->GetLeftVel()) < 0.5)
							&& drive_index
									>= (same_scale_len + added_switch_len
											+ added_scale_len))) {
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

void ScaleSide::RunStateMachineOppScale(bool *place_scale_backwards,
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();

	if (drive_index >= (opp_scale_len / 2)) { //start moving superstructure on the way //slowing down the profile will increase the number of points
		if (drive_index >= (opp_scale_len)) { //drive profile refs should stay at the last index, at the scale position, anyway, but just for clarity
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
	}

}

void ScaleSide::RunStateMachineOppScaleScale(bool *place_scale_backwards,
		bool *get_cube_ground) {

	int drive_index = drive_controller->GetDriveIndex();

//no other state machine booleans needed, all other ones will stay false

	if ((drive_index >= opp_scale_len
			&& auton_state_machine->shoot_counter == 0)
			|| (elevator_->GetElevatorPosition() > 0.3 //elevator going down
			&& auton_state_machine->shoot_counter == 1) //when shoot counter is 0, will be going up to shoot first cube and will not stop drive. once shot first cube and everything is coming down, will stop drive. once everything is coming back up, will stop drive
			|| auton_state_machine->state_a
					== auton_state_machine->POST_INTAKE_SCALE_STATE_A_H
			|| auton_state_machine->shoot_counter == 2
			|| (drive_index
					>= (opp_scale_len + added_switch_len + added_scale_len)
					&& auton_state_machine->shoot_counter == 1)) { //second case should not be needed, but just there //scale cube, was driving
		drive_controller->StopProfile(true);
	} else {
		drive_controller->StopProfile(false);
	}

	if (drive_index >= (opp_scale_len / 1.1)) { //start moving superstructure

		if (auton_state_machine->shoot_counter == 0 || ((drive_index //if have not shot before, if at end of the total profile and there is that addded profile
		>= (opp_scale_len + added_switch_len + added_scale_len) //will need to divide by 2
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
