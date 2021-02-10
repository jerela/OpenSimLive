// This class manages different IMU manufacturers and their methods, providing general methods that can be called in tests and other programs and then invoking manufacturer-specific methods through them.
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
		// return time
		double getTime();
		// update and return time
		//double updateTime();
		// enable or disable IMU feedback
		void setEnableIMUFeedback(bool setting) { enable_IMU_feedback_ = setting; }
		// generate identity quaternions
		void generateIdentityQuaternions();

	protected:
			
	private:
		// PRIVATE METHODS
		// converts a quaternion to roll, pitch and yaw angles (in degrees)
		std::array<double, 3> quaternionToRPY(SimTK::Quaternion_<double> quat);
		// calculates an estimate of drift as the norm of vector (roll-prevRoll, pitch-prevPitch, yaw-prevYaw) and prints it
		void estimateDrift();

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

		// contains the latest RPY arrays for all sensors
		std::vector<std::array<double, 3>> RPYVector_;
		// iterating index that is used to determine how often drift is displayed
		unsigned int driftInterval_ = 0;
		// whether to use drift estimation and printing RPY angles for each sensor
		bool enable_IMU_feedback_ = false;

	}; // end of class
}
