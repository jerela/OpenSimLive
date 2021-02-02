// IMUHandler.cxx

#include <IMUHandler.h>
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>
#include <stdlib.h>
#include <cmath>
#include <ctgmath>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
IMUHandler::IMUHandler() {
}

// DESTRUCTOR
IMUHandler::~IMUHandler() {
}

// set IMUType_ as xsens, delsys or simulated to guide what methods will be called later
void IMUHandler::setManufacturer(IMUType manufacturer) {
	IMUType_ = manufacturer;
}

// establish connection to the IMUs
void IMUHandler::initialize() {
	bool saveQuaternions = (ConfigReader("MainConfiguration.xml", "save_quaternions_to_file") == "true");
	if (IMUType_ == xsens) {
		xsensObject_.reset(new OpenSimLive::XsensDataReader);
		xsensObject_->SetDesiredUpdateRate(stoi(ConfigReader("MainConfiguration.xml", "desired_update_rate")));
		xsensObject_->setSaveQuaternions(saveQuaternions);
		// loop until startup is successful
		unsigned int dataReaderResult = 0;
		while (dataReaderResult == 0) {
			dataReaderResult = xsensObject_->InitiateStartupPhase();
		}
		// if InitiateStartupPhase returned 1, we close the program
		if (dataReaderResult == 1) {
			xsensObject_->CloseConnection();
			// exit with 0 indicating a successful exit
			exit(0);
		}
		// initialize quatVector_
		quatVector_ = xsensObject_->getQuaternionData();
	}
	else if (IMUType_ == delsys) {
		delsysObject_.reset(new OpenSimLive::DelsysDataReader);
		delsysObject_->setSaveQuaternions(saveQuaternions);
		// loop until startup is successful
		while (!delsysObject_->initiateConnection()) {}
	}
	else if (IMUType_ == simulated) {
		simulatedObject_.reset(new OpenSimLive::SimulatedDataReader);
		simulatedObject_->setSaveQuaternions(saveQuaternions);
	}
}

// update the value of quaternionTimeSeriesTable_
void IMUHandler::updateQuaternionTable() {
	if (IMUType_ == xsens) {
		//quaternionTimeSeriesTable_ = (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->GetQuaternionData(quatVector_)));
		quaternionTimeSeriesTable_ = (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->getQuaternionData()));
	}
	else if (IMUType_ == delsys) {
		//delsysObject_->updateQuaternionData();
		delsysObject_->updateQuaternionDataNoOffset();
		quaternionTimeSeriesTable_ = delsysObject_->getTimeSeriesTable();
	}
	else if (IMUType_ == simulated) {
		simulatedObject_->updateQuaternionTable();
		quaternionTimeSeriesTable_ = simulatedObject_->getTimeSeriesTable();
	}
	if (enable_IMU_feedback_) {
		// calculate and print a norm that describes the amount of drift when the subject is still
		estimateDrift();
	}
}

// This is an option that combines updateQuaternionTable() and getQuaternionTable(), resulting in better performance because quaternionTimeSeriesTable_ variable is not needlessly initialized in this class.
OpenSim::TimeSeriesTableQuaternion IMUHandler::updateAndGetQuaternionTable() {
	if (IMUType_ == xsens) {
		return (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->GetQuaternionData(quatVector_)));
	}
	else if (IMUType_ == delsys) {
		//delsysObject_->updateQuaternionData();
		delsysObject_->updateQuaternionDataNoOffset();
		return delsysObject_->getTimeSeriesTable();
	}
	else if (IMUType_ == simulated) {
		simulatedObject_->updateQuaternionTable();
		return simulatedObject_->getTimeSeriesTable();
	}
}


// update EMG values
void IMUHandler::updateEMG() {
	if (IMUType_ == xsens)
	{
		std::cout << "Xsens has no EMG capabilities, updateEMG() will do nothing!" << std::endl;
	}
	else if (IMUType_ == delsys) {
		delsysObject_->updateEMG();
	}
	else if (IMUType_ == simulated)
	{
		std::cout << "Simulation has no EMG capabilities, updateEMG() will do nothing!" << std::endl;
	}
}

// close the connection, save to file etc
void IMUHandler::closeConnection() {
	if (IMUType_ == xsens) {
		xsensObject_->CloseConnection();
	}
	else if (IMUType_ == delsys) {
		delsysObject_->closeConnection();
	}
	else if (IMUType_ == simulated) {
		simulatedObject_->closeConnection();
	}
}

// return current time from IMU classes; if none is found, return -1
double IMUHandler::getTime() {
	double time = -1;
	if (IMUType_ == xsens) {
		time = xsensObject_->getTime();
	}
	else if (IMUType_ == delsys) {
		time = delsysObject_->getOrientationTime();
	}
	else if (IMUType_ == simulated) {
		time = simulatedObject_->getTime();
	}
	return time;
}

// converts a quaternion to roll, pitch and yaw angles in degrees
std::array<double, 3> IMUHandler::quaternionToRPY(SimTK::Quaternion_<double> q) {
	// conversion from quaternion to roll, pitch and yaw
	double roll = SimTK::convertRadiansToDegrees(atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (std::pow(q[1], 2) + std::pow(q[2], 2))));
	double pitch = SimTK::convertRadiansToDegrees(asin(2 * (q[0] * q[2] - q[3] * q[1])));
	double yaw = SimTK::convertRadiansToDegrees(atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (std::pow(q[2], 2) + std::pow(q[3], 2))));
	// construct an array of the angles
	std::array<double, 3> RPY({ roll, pitch, yaw });
	return RPY;
}

// calculates an estimate of drift as the norm of vector (roll-prevRoll, pitch-prevPitch, yaw-prevYaw) and prints it
void IMUHandler::estimateDrift() {
	// iterate driftInterval_ to get us closer to printing the norm
	++driftInterval_;
	// if we're above a threshold, calculate the norm and print it
	if (driftInterval_ > 100) {
		// reset the "timer"
		driftInterval_ = 0;
		// get the IMU labels
		std::vector<std::string> labels = quaternionTimeSeriesTable_.getColumnLabels();
		// if RPYVector_ hasn't been initialized, populate it with zeros for initial values
		if (RPYVector_.size() == 0) {
			for (size_t label = 0; label < labels.size(); ++label) {
				RPYVector_.push_back({ 0, 0, 0 });
			}
		}

		// iterate through all sensors to calculate their roll, pitch and yaw angles and calculate the norm based on the difference from previous RPY angles
		for (size_t label = 0; label < labels.size(); ++label) {
			// get a matrix containing the quaternion for one of the IMUs
			SimTK::MatrixView_<SimTK::Quaternion> matrix = quaternionTimeSeriesTable_.getMatrixBlock(0, label, 1, 1);
			// convert the quaternion to RPY angles
			std::array<double, 3> RPY = quaternionToRPY(matrix(0, 0));
			// get individual angles from the array
			double roll = RPY[0];
			double pitch = RPY[1];
			double yaw = RPY[2];
			// get the angles from the previous time this part of the code was executed
			std::array<double, 3> prevRPY = RPYVector_[label];
			double prevRoll = prevRPY[0];
			double prevPitch = prevRPY[1];
			double prevYaw = prevRPY[2];
			// calculate the norm of vector (roll-prevRoll, pitch-prevPitch, yaw-prevYaw) to give an estimate of drift for the IMU
			double norm = std::sqrt(std::pow(roll - prevRoll, 2) + std::pow(pitch - prevPitch, 2) + std::pow(yaw - prevYaw, 2));
			// update the values of RPYVector_ for the IMU
			RPYVector_[label] = RPY;
			// print to console
			if (label == 0) {
				std::cout << "Drifts: [" << labels[label] << "(" << norm << ")";
			}
			else {
				std::cout << ", " << labels[label] << "(" << norm << ")";
			}

			if (label+1 == labels.size()) {
				std::cout << "]" << std::endl;
			}
			
		}

		for (size_t label = 0; label < labels.size(); ++label) {
			std::cout << "RPY for sensor " << (label + 1) << ": [" << RPYVector_[label][0] << ", " << RPYVector_[label][1] << ", " << RPYVector_[label][2] << "]" << std::endl;
		}


	}
}

