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
	// randomize seed based on current time
	srand(std::time(NULL));
}

// DESTRUCTOR
SimulatedDataReader::~SimulatedDataReader() {
	// if the setting is true, save quaternions to file
	if (saveQuaternionsToFile_) {
		saveQuaternionsToFile(OPENSIMLIVE_ROOT, "OpenSimLive-results");
	}
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

	// resize quaternions_ to equal the number of sensors aka the number of quaternions that will be saved in quaternions_
	quaternions_.resize(labelsSize_);
	// pre-reserve a fair amount of space in quaternionData_ to speed things up later
	quaternionData_.reserve(100000);
}

// updates quatTable_ with new quaternion values
void SimulatedDataReader::updateQuaternionTable() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
	for (unsigned int m = 0; m < labelsSize_; ++m) {
		// generate a new unit quaternion on a spot in the matrix
		SimTK::Quaternion quat = generateQuaternion();
		quatMatrix.set(0, m, quat);
		if (saveQuaternionsToFile_) {
			quaternions_[m] = quat;
		}
	}

	// if we are going to save quaternions to file later, push current quaternions to relevant vectors and update timevector
	if (saveQuaternionsToFile_) {
		// push quaternions into vector
		quaternionData_.push_back(quaternions_);
		// get current time
		clockNow_ = std::chrono::high_resolution_clock::now();
		// update elapsed time (clockDuration_)
		clockDuration_ = (clockNow_ - clockStart_);
		// push time into vector
		timeVector_.push_back(clockDuration_.count());
	}

	// finally create/overwrite the time series table using the randomized quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}


// this is actually pointless, because there is no actual connection to IMUs when simulating IMU data
void SimulatedDataReader::closeConnection() {
	std::cout << "Simulated connection closed!" << std::endl;
}


// This function saves the time points and the corresponding quaternions to file for later examination.
void SimulatedDataReader::saveQuaternionsToFile(const std::string& rootDir, const std::string& resultsDir) {
	
	if (timeVector_.size() > 100000 || quaternionData_.size() > 100000) {
		std::cout << "In a normal situation we would save quaternions to file now, but because there are " << timeVector_.size() << " data points, for the sake of hard drive space we won't do it." << std::endl;
		return;
	}
	else {
		std::cout << "Saving quaternion time series to file..." << std::endl;
	}

	// create the complete path of the file, including the file itself, as a string
	std::string filePath(rootDir + "/" + resultsDir + "/" + "QuaternionTimeSeriesSimulated.txt");
	// create an output file stream that is used to write into the file
	std::ofstream outputFile;
	// open and set file to discard any contents that existed in the file previously (truncate mode)
	outputFile.open(filePath, std::ios_base::out | std::ios_base::trunc);
	// check that the file was successfully opened and write into it
	if (outputFile.is_open())
	{
		outputFile << "Time series of measured orientation data in quaternions:\n";
		outputFile << "Time (s)";

		for (unsigned int j = 0; j < labelsSize_; ++j) {
			outputFile << "\t" << "Quaternion" + std::to_string(j + 1);
		}

		for (unsigned int i = 0; i < quaternionData_.size(); ++i) { // iteration through rows
			// after the first 2 rows of text, start with a new line and put time values in the first column
			outputFile << "\n" << std::setprecision(9) << timeVector_[i];
			for (unsigned int j = 0; j < labelsSize_; ++j) {
				// then input quaternion values, separating them from time and other quaternion values with a tab
				outputFile << "\t" << quaternionData_[i][j];
			}
		}
		outputFile.close();
		std::cout << "Quaternion time series written to file " << filePath << std::endl;
	}
	else {
		std::cout << "Failed to open file " << filePath << std::endl;
	}
}





// updates quatTable_ with new quaternion values that are all identity quaternions; used for performance testing purposes
void SimulatedDataReader::generateIdentityQuaternions() {
	// create a quaternion matrix for time series table
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, labelsSize_);
	// loop through all elements of labels_ (the vector containing labels for IMUs on the model)
	for (unsigned int m = 0; m < labelsSize_; ++m) {
		// generate a new identity quaternion on a spot in the matrix
		SimTK::Quaternion quat(1, 0, 0, 0);
		quatMatrix.set(0, m, quat);
		if (saveQuaternionsToFile_) {
			//std::cout << quat << std::endl;
			quaternions_[m] = quat;
		}
	}

	// if we are going to save quaternions to file later, push current quaternions to relevant vectors and update timevector
	if (saveQuaternionsToFile_) {
		// push quaternions into vector
		quaternionData_.push_back(quaternions_);
		// get current time
		clockNow_ = std::chrono::high_resolution_clock::now();
		// update elapsed time (clockDuration_)
		clockDuration_ = (clockNow_ - clockStart_);
		// push time into vector
		timeVector_.push_back(clockDuration_.count());
	}

	// finally create/overwrite the time series table using the randomized quaternion data
	quatTable_ = OpenSim::TimeSeriesTable_<SimTK::Quaternion>({ 0 }, quatMatrix, labels_);
}