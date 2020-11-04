// IMUHandler.cxx

#include <IMUHandler.h>
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>
#include <stdlib.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
IMUHandler::IMUHandler() {
}

// DESTRUCTOR
IMUHandler::~IMUHandler() {
}

// set IMUType_ as either xsens or delsys to guide what methods will be called later
void IMUHandler::setManufacturer(IMUType manufacturer) {
	IMUType_ = manufacturer;
}

// establish connection to the IMUs
void IMUHandler::initialize() {
	if (IMUType_ == xsens) {
		xsensObject_.reset(new OpenSimLive::XsensDataReader);
		unsigned int dataReaderResult = 0;
		while (dataReaderResult == 0) {
			dataReaderResult = xsensObject_->InitiateStartupPhase();
		}
		// if InitiateStartupPhase returned 1, we close the program
		if (dataReaderResult == 1) {
			xsensObject_->CloseConnection();
			exit;
		}
		// initialize quatVector_
		quatVector_ = xsensObject_->getQuaternionData();

	}
	else if (IMUType_ == delsys) {
		delsysObject_.reset(new OpenSimLive::DelsysDataReader);
		while (!delsysObject_->initiateConnection()) {}
	}
	else if (IMUType_ == simulated) {
		simulatedObject_.reset(new OpenSimLive::SimulatedDataReader);
	}
}

// update the value of quaternionTimeSeriesTable_
void IMUHandler::updateQuaternionTable() {
	if (IMUType_ == xsens) {
		quaternionTimeSeriesTable_ = (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->GetQuaternionData(quatVector_)));
	}
	else if (IMUType_ == delsys) {
		delsysObject_->updateQuaternionData();
		quaternionTimeSeriesTable_ = delsysObject_->getTimeSeriesTable();
	}
	else if (IMUType_ == simulated) {
		simulatedObject_->updateQuaternionTable();
		quaternionTimeSeriesTable_ = simulatedObject_->getTimeSeriesTable();
	}
}

// This is an option that combines updateQuaternionTable() and getQuaternionTable(), resulting in better performance because quaternionTimeSeriesTable_ variable is not needlessly initialized in this class.
OpenSim::TimeSeriesTableQuaternion IMUHandler::updateAndGetQuaternionTable() {
	if (IMUType_ == xsens) {
		return (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->GetQuaternionData(quatVector_)));
	}
	else if (IMUType_ == delsys) {
		delsysObject_->updateQuaternionData();
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
	if (IMUType_ == xsens)
	{
		xsensObject_->CloseConnection();
	}
	else if (IMUType_ == delsys) {
		delsysObject_->closeConnection();
	}
	else if (IMUType_ == simulated) {
		simulatedObject_->closeConnection();
	}
}


