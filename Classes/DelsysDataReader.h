#pragma once

#include <OpenSim.h>
#include <Client.h>


namespace OpenSimLive {

	class DelsysDataReader {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		DelsysDataReader();
		~DelsysDataReader();

		// PUBLIC METHODS
		bool initiateConnection();
		bool closeConnection();
		void updateQuaternionData();
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return *quatTable_; }
		
	protected:
			
	private:
		// PRIVATE METHODS
		unsigned int correctSensorIndex(const std::vector<unsigned int>& sensorLabels, std::vector<unsigned int>& sensorIndices);
		float convertBytesToFloat(char b1, char b2, char b3, char b4, int rev);
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices);

		// PRIVATE VARIABLES
		union byteFloater;
		Client* commandPort_;
		Client* AUXPort_;
		OpenSim::TimeSeriesTable_<SimTK::Quaternion>* quatTable_;

	}; // end of class
}