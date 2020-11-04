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
		// set manufacturer; currently either delsys, xsens or simulated
		void setManufacturer(IMUType newIMUType);
		// create a new unique pointer to the object type determined by manufacturer and establish connection to IMUs
		void initialize();
		// update the value of quaternionTimeSeriesTable_
		void updateQuaternionTable();
		// updates EMG value (not a member variable of this class)
		void updateEMG();
		// closes the connection to IMUs
		void closeConnection();
		// return quaternionTimeSeriesTable_ when needed e.g. for IK
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getQuaternionTable() { return quaternionTimeSeriesTable_; };
		// this is an option that combines updateQuaternionTable() and getQuaternionTable(), resulting in better performance because quaternionTimeSeriesTable_ variable is not needlessly initialized here
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> updateAndGetQuaternionTable();

	protected:
			
	private:
		// PRIVATE METHODS

		// PRIVATE VARIABLES
		// holds the enumerable that is delsys, xsens or simulated
		IMUType IMUType_;
		// smart pointer to XsensDataReader object
		std::unique_ptr<OpenSimLive::XsensDataReader> xsensObject_;
		// smart pointer to DelsysDataReader object
		std::unique_ptr<OpenSimLive::DelsysDataReader> delsysObject_;
		// smart pointer to SimulatedDataReader object
		std::unique_ptr<OpenSimLive::SimulatedDataReader> simulatedObject_;
		// quaternion time series table that is fed to IK, among other things
		OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable_;
		// a quaternion vector for xsensObject_; needed to call GetQuaternionData(std::vector<XsQuaternion>) instead of getQuaternionData(), which is much slower
		std::vector<XsQuaternion> quatVector_;

	}; // end of class
}
