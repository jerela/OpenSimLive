#pragma once
#include <OpenSim.h>
#include <string>
#include <vector>
#include <XMLFunctionsXsens.h>
#include <XsensDataReader.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// This function fills a TimeSeriesTable with quaternion values for a single time frame.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> fillQuaternionTable(std::vector<MtwCallback*>& mtwCallbacks, std::vector<XsQuaternion>& quaternionVector)
{
	// get the number of active sensors
	unsigned int numberOfSensors = mtwCallbacks.size();

	// declare a vector for the sensor names in the OpenSim model
	std::vector<std::string> sensorNameVector;

	// define a vector with a single time value (such as zero) for constructing OpenSim::TimeSeriesTable
	std::vector<double> timeVector{ 0 };

	// initialize a matrix where each element is an entire quaternion, requires for constructing OpenSim::TimeSeriesTable
	SimTK::Matrix_<SimTK::Quaternion> quaternionMatrix(1, numberOfSensors);

	std::string currentSensorId; std::string sensorNameInModel;

	// "intermediary" variable for populating quaternionMatrix inside the for-loop
	SimTK::Quaternion_<SimTK::Real> quat;

	// populate sensorNameVector and quaternionMatrix
	for (size_t i = 0; i < mtwCallbacks.size(); ++i) {
		// get the ID of the current IMU
		currentSensorId = mtwCallbacks[i]->device().deviceId().toString().toStdString();

		// match the ID of the sensor to the name of the sensor on the model
		sensorNameInModel = sensorIdToLabel(currentSensorId, OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "mappings_file"));

		// populate the vector of sensor names
		sensorNameVector.push_back(sensorNameInModel);

		// get the quaternions from XsQuaternion, put them into SimTK::Quaternion and put that quaternion into quaternionMatrix
		quat[0] = quaternionVector[i].w();
		quat[1] = quaternionVector[i].x();
		quat[2] = quaternionVector[i].y();
		quat[3] = quaternionVector[i].z();
		quaternionMatrix.set(0, i, quat);
	}

	// construct a TimeSeriesTable from the data we calculated in this function and return it
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> outputTable(timeVector, quaternionMatrix, sensorNameVector);
	return outputTable;
}
