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
}

// DESTRUCTOR
SimulatedDataReader::~SimulatedDataReader() {
}

// creates a quaternion with randomized elements and returns it as a unit quaternion
SimTK::Quaternion SimulatedDataReader::generateQuaternion() {
	SimTK::Quaternion quat(rand() % 100, rand() % 100, rand() % 100, rand() % 100);
	return quat.normalize();
}

// updates quatTable_ with new quaternion values
void SimulatedDataReader::updateQuaternionTable() {
	// labels lists IMUs to simulate
	std::vector<std::string> labels = { "pelvis_imu", "torso_imu", "femur_r_imu", "femur_l_imu" };
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labels.size());
	for (unsigned int m = 0; m < labels.size(); ++m) {
		quatMatrix.set(0, m, generateQuaternion());
	}

	// create a time vector for time series table
	std::vector<double> timeVector = { 0 };

	// finally create/overwrite the time series table
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>(timeVector, quatMatrix, labels);
}

// returns the time series table of quaternion orientations
OpenSim::TimeSeriesTable_<SimTK::Quaternion> SimulatedDataReader::getTimeSeriesTable() {
	return quatTable_;
}

// this is actually pointless, because there is no actual connection to IMUs when simulating IMU data
void SimulatedDataReader::closeConnection() {
	std::cout << "Simulated connection closed!" << std::endl;
}

