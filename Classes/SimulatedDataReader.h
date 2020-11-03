// This class is for creating data to simulate IMUs. The data is not sensible for motion analysis, but can be useful to check if the program works at all.
#pragma once

#include <OpenSim.h>

namespace OpenSimLive {

	class SimulatedDataReader {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		SimulatedDataReader();
		~SimulatedDataReader();

		// PUBLIC METHODS
		// updates the value of quatTable_
		void updateQuaternionTable();
		// simply prints out a message that a connection is being closed to indicate that we got this far successfully; there is no actual connection to close in this class
		void closeConnection();
		// returns the time series table of quaternion orientations
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return quatTable_; }
		
	protected:
			
	private:
		// PRIVATE METHODS
		// returns a randomized unit quaternion
		SimTK::Quaternion generateQuaternion();
		// reads a config file and populates a string vector with labels such as "pelvis_imu" and "femur_l_imu", which name the bodies that are given simulated orientations
		void populateLabelVector();

		// PRIVATE VARIABLES
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable_;
		// labels_ lists IMUs to simulate
		std::vector<std::string> labels_;
		// size of labels; fairly unimportant, but it helps to have this to prevent calling vector::size() twice whenever updateQuaternionTable() is called
		unsigned int labelsSize_ = 0;

	}; // end of class
}