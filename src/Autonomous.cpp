/*
 * Autonomous.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: DriversStation
 */

#include <Autonomous.h>


#define CORNELIUS 0

#if CORNELIUS

#else

#endif

const int NUM_POINTS = 1500; //every 10ms
const int NUM_INDEX = 10; //change this

double refs[NUM_POINTS][NUM_INDEX];

DriveController *drive_controller; //can't be in .h
Elevator *elevator_;
Intake *intake_;

Autonomous::Autonomous(DriveController *dc, Elevator *el, Intake *in) {

	drive_controller = dc;
	elevator_ = el;
	intake_ = in;

}


//void Autonomous::RunAuton() { //TODO: make this
//
//}

//void Autonomous::FillProfile(std::string profileName) { //fill array and run auton, extra column of 0s in csv are not carried over into array //not needed
//
//	for (int r = 0; r < NUM_POINTS; r++) { //sets the entire array to 0 so that all the points that arent filled are zeros, easy to check for
//		for (int c = 0; c < NUM_INDEX; c++) {
//			refs[r][c] = 0;
//		}
//	}
//
//	int r = 0;
//	std::fstream file(profileName, std::ios::in);
//	while (r < NUM_POINTS) {
//		std::string data;
//		std::getline(file, data);
//		std::stringstream iss(data);
//		if (!file.good()) {
//			std::cout << "FAIL" << std::endl;
//		}
//		int c = 0;
//		while (c < NUM_INDEX) {
//			std::string val;
//			std::getline(iss, val, ',');
//			std::stringstream convertor(val);
//			convertor >> refs[r][c];
//			c++;
//			//if (file.eof()) {
//			//	drive_controller->SetProfileLength(r); //sets array length to length of csv file
//			//}
//		}
//		r++;
//	}
//
//	//drive_controller->SetRef(refs);
//	drive_controller->StartAutonThreads();
//
//}


//void Autonomous::FillProfile(std::string profileName) { //fill array and run auton, extra column of 0s in csv are not carried over into array
//
//	for (int r = 0; r < NUM_POINTS; r++) { //sets the entire array to 0 so that all the points that arent filled are zeros, easy to check for
//		for (int c = 0; c < NUM_INDEX; c++) {
//			refs[r][c] = 0;
//		}
//	}
//
//	int r = 0;
//	std::fstream file(profileName, std::ios::in);
//	while (r < NUM_POINTS) {
//		std::string data;
//		std::getline(file, data);
//		std::stringstream iss(data);
//		if (!file.good()) {
//			std::cout << "FAIL" << std::endl;
//		}
//		int c = 0;
//		while (c < NUM_INDEX) {
//			std::string val;
//			std::getline(iss, val, ',');
//			std::stringstream convertor(val);
//			convertor >> refs[r][c];
//			c++;
//			//if (file.eof()) {
//			//	drive_controller->SetProfileLength(r); //sets array length to length of csv file
//			//}
//		}
//		r++;
//	}
//
//	//drive_controller->SetRef(refs);
//	drive_controller->StartAutonThreads();
//
//}
