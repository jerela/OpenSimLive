#pragma once

#include <OpenSim.h>
#include <Client.h>
#include <memory>
#include <PythonPlotter.h>

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
		void updateQuaternionData(); 
		// returns the value of quatTable_
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return *quatTable_; }
		// reads byte stream from the IMUs and updates the EMG signal into a vector
		void updateEMG();
		
	protected:
			
	private:
		// PRIVATE METHODS
		unsigned int correctSensorIndex(std::vector<unsigned int>& sensorIndices); // calculate the offset between detected sensor indices and actual sensor index labels
		float convertBytesToFloat(char b1, char b2, char b3, char b4, int rev); // uses a union data type to convert between floats and bytes
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices); // reads an XML file and returns a vector of string labels that connect the index of each IMU to a body on the model (e.g. IMU label/index '1' -> 'pelvis_imu')
		std::vector<std::string> getLabelsFromFile(); // reads all 16 labels from DelsysMappings into a vector

		// PRIVATE VARIABLES
		union byteFloater; // data type that can contain several different variable types in one memory location; used in convertBytesToFloat
		// pointer to Delsys SDK command port, which receives commands
		std::unique_ptr<Client> commandPort_;
		// pointer to Delsys SDK AUX port, which sends orientation data
		std::unique_ptr<Client> AUXPort_;
		// pointer to Delsys SDK EMG port, which sends EMG data
		std::unique_ptr<Client> EMGPort_;
		// pointer to the time series table of quaternions for each IMU w.r.t. time
		std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>> quatTable_;
		std::vector<std::string> labels_; // vector that contains labels of IMUs on the model, for example "pelvis_imu" and "femur_r_imu"
		// vector that contains the labels (numbers 1-16) of the Delsys sensors that are being used, as defined in <active_sensors> in the mappings file
		std::vector<unsigned int> activeSensors_;
		// number of active sensors
		unsigned int nActiveSensors_ = 0;

		// PRIVATE METHODS FOR EMG AND PLOTTING
		// reads byte stream from IMUs and updates the float array of EMG data points (EMGDataPoints_), argument is the index of the sensor we use to plot EMG data
		void updateEMGData();
		// make PythonPlotter run preparatory Python commands
		void prepareEMGGraph();
		// pass updated parameters to PythonPlotter
		void updateEMGGraph();
		// save EMG time series as .txt
		void saveEMGToFile(const std::string& rootDir, const std::string& resultsDir);

		// PRIVATE VARIABLES FOR EMG AND PLOTTING
		// unique pointer for PythonPlotter object
		std::unique_ptr<PythonPlotter> pythonPlotter_;
		// time point where EMG measurement begins
		std::chrono::steady_clock::time_point startTime_;
		// time point that is used together with startTime_ to calculate elapsed time since the beginning of EMG measurement
		std::chrono::steady_clock::time_point currentTime_;
		// EMG points are saved here
		std::vector<std::array<float, 16>> EMGData_;
		// tracks elapsed time since the beginning of EMG measurement
		std::vector<float> timeVector_;
		// tracks EMG for different sensors from the latest update
		std::array<float, 16> EMGDataPoints_ = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		// if plotterPrepared_ is false, we run prepare functions instead of update functions
		bool plotterPrepared_ = false;

	}; // end of class
}