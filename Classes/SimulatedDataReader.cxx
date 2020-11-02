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
	SimTK::Quaternion quat(rand() % 100, rand() % 100, rand() % 100, rand() % 100);
	return quat.normalize();
}

// populate labels_ from config file
void SimulatedDataReader::populateLabelVector() {
	std::string IMULabelString = ConfigReader("MainConfiguration.xml", "simulated_bodies");
	// stringstream is a simple way to separate the whitespace-separated numbers from the whole string
	std::stringstream ss(IMULabelString);
	// loop through all elements
	do {
		std::string IMULabel;
		// write from stringstream to IMULabel
		ss >> IMULabel;
		// push IMULabel in a vector
		labels_.push_back(IMULabel);
	} while (!ss.eof());
}

// updates quatTable_ with new quaternion values
void SimulatedDataReader::updateQuaternionTable() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labels_.size());
	for (unsigned int m = 0; m < labels_.size(); ++m) {
		quatMatrix.set(0, m, generateQuaternion());
	}

	// create a time vector for time series table
	std::vector<double> timeVector = { 0 };

	// finally create/overwrite the time series table
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>(timeVector, quatMatrix, labels_);
}

// returns the time series table of quaternion orientations
OpenSim::TimeSeriesTable_<SimTK::Quaternion> SimulatedDataReader::getTimeSeriesTable() {
	return quatTable_;
}

// this is actually pointless, because there is no actual connection to IMUs when simulating IMU data
void SimulatedDataReader::closeConnection() {
	std::cout << "Simulated connection closed!" << std::endl;
}

