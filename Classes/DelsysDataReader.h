#pragma once

#include <OpenSim.h>
#include <Client.h>
#include <memory>

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
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices, unsigned int offset); // reads an XML file and returns a vector of string labels that connect the index of each IMU to a body on the model (e.g. IMU label/index '1' -> 'pelvis_imu')
		std::vector<std::string> getLabelsFromFile(); // reads all 16 labels from DelsysMappings into a vector

		// PRIVATE VARIABLES
		union byteFloater; // data type that can contain several different variable types in one memory location; used in convertBytesToFloat
		//Client* commandPort_; // pointer to Delsys SDK command port, which receives commands
		std::unique_ptr<Client> commandPort_;
		//Client* AUXPort_; // pointer to Delsys SDK AUX port, which sends orientation data
		std::unique_ptr<Client> AUXPort_;
		//OpenSim::TimeSeriesTable_<SimTK::Quaternion>* quatTable_; // pointer to the time series table of quaternions for each IMU w.r.t. time
		std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>> quatTable_;
		std::vector<std::string> labels_; // vector that contains labels of IMUs on the model, for example "pelvis_imu" and "femur_r_imu"
		// vector that contains the labels (numbers 1-16) of the Delsys sensors that are being used, as defined in <active_sensors> in the mappings file
		std::vector<unsigned int> activeSensors_;
		// number of active sensors
		unsigned int nActiveSensors_ = 0;

	}; // end of class
}