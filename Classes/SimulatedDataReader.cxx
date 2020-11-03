// main.cpp

#include <SimulatedDataReader.h>
#include <string>
#include <iostream>
#include <OpenSim.h>
#include <XMLFunctions.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
SimulatedDataReader::SimulatedDataReader() {
	populateLabelVector();
}

// DESTRUCTOR
SimulatedDataReader::~SimulatedDataReader() {
}

// creates a quaternion with randomized elements and returns it as a unit quaternion
SimTK::Quaternion SimulatedDataReader::generateQuaternion() {
	// create a quaternion (not a unit quaternion) with random elements
	SimTK::Quaternion quat(rand() % 100, rand() % 100, rand() % 100, rand() % 100);
	// return a normalized version of the random quaternion
	return quat.normalize();
}

// populate labels_ from config file
void SimulatedDataReader::populateLabelVector() {
	// read the user-given values from the config file
	std::string IMULabelString = ConfigReader("MainConfiguration.xml", "simulated_bodies");
	// stringstream is a simple way to separate the whitespace-separated numbers from the whole string
	std::stringstream ss(IMULabelString);
	// loop through all elements (loop as long as there is anything left in the stringstream)
	do {
		// represents a piece of text such as "pelvis_imu"
		std::string IMULabel;
		// write from stringstream to IMULabel
		ss >> IMULabel;
		// push IMULabel in a vector
		labels_.push_back(IMULabel);
	} while (!ss.eof());
	labelsSize_ = labels_.size();
}

// updates quatTable_ with new quaternion values
void SimulatedDataReader::updateQuaternionTable() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
	for (unsigned int m = 0; m < labelsSize_; ++m) {
		// generate a new unit quaternion on a spot in the matrix
		quatMatrix.set(0, m, generateQuaternion());
	}

	// finally create/overwrite the time series table using the randomized quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}


// this is actually pointless, because there is no actual connection to IMUs when simulating IMU data
void SimulatedDataReader::closeConnection() {
	std::cout << "Simulated connection closed!" << std::endl;
}

