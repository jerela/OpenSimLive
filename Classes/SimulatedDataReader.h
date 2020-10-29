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
		void updateQuaternionTable();
		void closeConnection();
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable();
		
	protected:
			
	private:
		// PRIVATE METHODS
		SimTK::Quaternion generateQuaternion();

		// PRIVATE VARIABLES
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable_;

	}; // end of class
}