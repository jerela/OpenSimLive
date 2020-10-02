#pragma once

#include <Simbody.h>

// ACTUAL XSENSDATAREADER CLASS BEGINS HERE
namespace OpenSimLive {

	class DelsysDataReader {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		DelsysDataReader();
		~DelsysDataReader();

		// PUBLIC METHODS
		bool initiateConnection();
		int main();
		
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

	}; // end of class
}