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
	}
	else if (IMUType_ == delsys) {
		delsysObject_.reset(new OpenSimLive::DelsysDataReader);
		while (!delsysObject_->initiateConnection()) {}
	}
}

// update the value of quaternionTimeSeriesTable_
void IMUHandler::updateQuaternionTable() {
	if (IMUType_ == xsens) {
		quaternionTimeSeriesTable_ = (fillQuaternionTable(xsensObject_->GetMtwCallbacks(), xsensObject_->getQuaternionData()));
	}
	else if (IMUType_ == delsys) {
		delsysObject_->updateQuaternionData();
		quaternionTimeSeriesTable_ = delsysObject_->getTimeSeriesTable();
	}
}

// return quaternionTimeSeriesTable_ when needed e.g. for IK
OpenSim::TimeSeriesTable_<SimTK::Quaternion> IMUHandler::getQuaternionTable() {
	return quaternionTimeSeriesTable_;
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
}


