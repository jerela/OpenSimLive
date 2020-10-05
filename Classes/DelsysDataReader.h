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
		bool initiateConnection(); // sends the START command to Delsys SDK / Trigno Control Utility
		bool closeConnection(); // sends the STOP command ...
		void updateQuaternionData(); // reads byte stream from the IMUs and updates the time series table of quaternions (private variable quatTable_)
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return *quatTable_; } // return the value of quatTable_
		
	protected:
			
	private:
		// PRIVATE METHODS
		unsigned int correctSensorIndex(const std::vector<unsigned int>& sensorLabels, std::vector<unsigned int>& sensorIndices); // calculate the offset between detected sensor indices and actual sensor index labels
		float convertBytesToFloat(char b1, char b2, char b3, char b4, int rev); // uses a union data type to convert between floats and bytes
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices); // reads an XML file and returns a vector of string labels that connect the index of each IMU to a body on the model (e.g. IMU label/index '1' -> 'pelvis_imu')

		// PRIVATE VARIABLES
		union byteFloater; // data type that can contain several different variable types in one memory location; used in convertBytesToFloat
		Client* commandPort_; // pointer to Delsys SDK command port, which receives commands
		Client* AUXPort_; // pointer to Delsys SDK AUX port, which sends orientation data
		OpenSim::TimeSeriesTable_<SimTK::Quaternion>* quatTable_; // pointer to the time series table of quaternions for each IMU w.r.t. time
		std::vector<std::string> labels_; // vector that contains labels of IMUs on the model, for example "pelvis_imu" and "femur_r_imu"

	}; // end of class
}