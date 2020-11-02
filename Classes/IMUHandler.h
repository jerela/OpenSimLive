// IMUHandler.h
#pragma once
#include <DelsysDataReader.h>
#include <XsensDataReader.h>
#include <SimulatedDataReader.h>
#include <memory>
#include <OpenSim.h>


enum IMUType { xsens = 0, delsys = 1, simulated = 2 };

namespace OpenSimLive {

	class IMUHandler {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		IMUHandler();
		~IMUHandler();

		// PUBLIC METHODS
		void setManufacturer(IMUType newIMUType);
		void initialize();
		void updateQuaternionTable();
		void updateEMG();
		void closeConnection();
		// return quaternionTimeSeriesTable_ when needed e.g. for IK
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getQuaternionTable() { return quaternionTimeSeriesTable_; };

	protected:
			
	private:
		// PRIVATE METHODS

		// PRIVATE VARIABLES
		IMUType IMUType_; // holds the enum that is either delsys or xsens
		std::unique_ptr<OpenSimLive::XsensDataReader> xsensObject_; // pointer to XsensDataReader object
		std::unique_ptr<OpenSimLive::DelsysDataReader> delsysObject_; // pointer to DelsysDataReader object
		std::unique_ptr<OpenSimLive::SimulatedDataReader> simulatedObject_; // pointer to SimulatedDataReader object
		OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable_; // quaternion time series table that is fed to IK, among other things

	}; // end of class
}
