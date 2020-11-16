// This class reads realtime orientation and EMG data from Delsys sensors.
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
		// sends the START command to Delsys SDK / Trigno Control Utility
		bool initiateConnection();
		// sends the STOP command ...
		bool closeConnection(); 
		// reads byte stream from the IMUs and updates the time series table of quaternions (private variable quatTable_)
		//void updateQuaternionData();
		// reads byte stream from the IMUs and updates the time series table of quaternions (private variable quatTable_); a more elegant solution when we can assume there is no offset between detected sensor indices and actual sensors being used
		void updateQuaternionDataNoOffset();
		// returns the value of quatTable_
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return *quatTable_; }
		// reads byte stream from the IMUs and updates the EMG signal into a vector
		void updateEMG();
		// returns the latest EMG values read from stream as an array of 16 floats, where i'th float element represents the EMG value read from i'th sensor
		std::array<float, 16> getLatestEMGValues() { return EMGDataPoints_; }
		// returns the number of sensors that are being used for the measurement
		unsigned int getNActiveSensors() { return nActiveSensors_; }
		// append time to vector of time points
		void appendTime(double time) { timeVector_.push_back(time); }
		// enable or disable saving quaternion time series to file
		void setSaveQuaternions(bool setting) { saveQuaternionsToFile_ = setting; }
		
	protected:
			
	private:
		// PRIVATE METHODS
		// calculate the offset between detected sensor indices and actual sensor index labels
		//unsigned int correctSensorIndex(std::vector<unsigned int>& sensorIndices);
		// use a union data type to convert between floats and bytes
		float convertBytesToFloat(char b1, char b2, char b3, char b4, int rev);
		// reads an XML file and returns a vector of string labels that connect the index of each IMU to a body on the model (e.g. IMU label/index '1' -> 'pelvis_imu')
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices);
		// reads all 16 labels from DelsysMappings into a vector
		std::vector<std::string> getLabelsFromFile();

		// PRIVATE VARIABLES
		// data type that can contain several different variable types in one memory location; used in convertBytesToFloat
		union byteFloatConverter;
		// pointer to Delsys SDK command port, which receives commands
		std::unique_ptr<Client> commandPort_;
		// pointer to Delsys SDK AUX port, which sends orientation data
		std::unique_ptr<Client> AUXPort_;
		// pointer to Delsys SDK EMG port, which sends EMG data
		std::unique_ptr<Client> EMGPort_;
		// pointer to the time series table of quaternions for each IMU w.r.t. time
		std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>> quatTable_;
		// vector that contains labels of IMUs on the model, for example "pelvis_imu" and "femur_r_imu"
		std::vector<std::string> labels_;
		// vector that contains the labels (numbers 1-16) of the Delsys sensors that are being used, as defined in <active_sensors> in the mappings file
		std::vector<unsigned int> activeSensors_;
		// number of active sensors
		unsigned int nActiveSensors_ = 0;
		// quaternions are saved here for later saving to file
		std::vector<std::array<SimTK::Quaternion, 16>> quaternionData_;
		// a quaternion array from a single data reading is stored here and overwritten each time
		std::array<SimTK::Quaternion, 16> quaternionArray_;
		// boolean to toggle if quaternions are saved to file or not
		bool saveQuaternionsToFile_ = false;
		// save quaternion time series as .txt
		void saveQuaternionsToFile(const std::string& rootDir, const std::string& resultsDir);

		// PRIVATE METHODS FOR EMG
		// reads byte stream from IMUs and updates the float array of EMG data points (EMGDataPoints_), argument is the index of the sensor we use to plot EMG data
		void updateEMGData();
		// save EMG time series as .txt
		void saveEMGToFile(const std::string& rootDir, const std::string& resultsDir);
		// give initial values to startTime_ and currentTime
		void prepareTime();
		// update the value of currentTime_
		void updateTime();

		// PRIVATE VARIABLES FOR EMG AND PLOTTING
		// time point where EMG measurement begins
		std::chrono::steady_clock::time_point startTime_;
		// time point that is used together with startTime_ to calculate elapsed time since the beginning of EMG measurement
		std::chrono::steady_clock::time_point currentTime_;
		// EMG points are saved here
		std::vector<std::array<float, 16>> EMGData_;
		// tracks elapsed time since the beginning of EMG measurement
		std::vector<double> timeVector_;
		// tracks EMG for different sensors from the latest update
		std::array<float, 16> EMGDataPoints_ = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	}; // end of class
}