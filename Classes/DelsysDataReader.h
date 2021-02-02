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
		// reads byte stream from the IMUs and updates the time series table of quaternions; differs from udpateQuaternionDataNoOffset() because in the fast version, we do not wait for all IMUs to broadcast into the stream successfully
		void updateQuaternionDataFast();
		// returns the value of quatTable_
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getTimeSeriesTable() { return *quatTable_; }
		// reads byte stream from the IMUs and updates the EMG signal into a vector
		void updateEMG();
		// returns the latest EMG values read from stream as an array of 16 floats, where i'th float element represents the EMG value read from i'th sensor
		std::array<float, 16> getLatestEMGValues() { return EMGDataPoints_; }
		// returns the number of sensors that are being used for the measurement
		unsigned int getNActiveSensors() { return nActiveSensors_; }
		// enable or disable saving quaternion time series to file
		void setSaveQuaternions(bool setting) { saveQuaternionsToFile_ = setting; }
		// return current time as double
		//double getTime() { return std::chrono::duration<double>(currentTime_ - startTime_).count(); }
		// get latest updated time for orientation
		double getOrientationTime() { return std::chrono::duration<double>(currentOrientationTime_ - startTime_).count(); }
		// get latest updated time for EMG
		double getEMGTime() { return std::chrono::duration<double>(currentEMGTime_ - startTime_).count(); }
		
	protected:
			
	private:
		// enumerationc class that is used with updateTime() to determine if time should be stored into orientationTimeVector_ or EMGTimeVector_ 
		/*enum class VectorType {
			ORIENTATION,
			EMG
		};*/

		// PRIVATE METHODS
		// calculate the offset between detected sensor indices and actual sensor index labels
		//unsigned int correctSensorIndex(std::vector<unsigned int>& sensorIndices);
		// use a union data type to convert between floats and bytes
		float convertBytesToFloat(char b1, char b2, char b3, char b4, int rev);
		// reads an XML file and returns a vector of string labels that connect the index of each IMU to a body on the model (e.g. IMU label/index '1' -> 'pelvis_imu')
		std::vector<std::string> getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices);
		// reads all 16 labels from DelsysMappings into a vector
		std::vector<std::string> getLabelsFromFile();
		// reads byte stream from IMUs and updates the float array of EMG data points (EMGDataPoints_), argument is the index of the sensor we use to plot EMG data
		void updateEMGData();
		// save EMG time series as .txt
		void saveEMGToFile(const std::string& rootDir, const std::string& resultsDir);
		// give initial values to startTime_ and currentTime
		void prepareTime();
		// update the value of currentTime_
		//void updateTime(VectorType type);
		// update the value of currentOrientationTime_
		void updateOrientationTime();
		// update the value of currentEMGTime
		void updateEMGTime();


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
		// vector that contains labels of IMUs on the model, but only those of active IMUs
		std::vector<std::string> activeLabels_;
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
		// time point where the measurement begins
		std::chrono::steady_clock::time_point startTime_;
		// time point that is used together with startTime_ to calculate elapsed time since the beginning of the measurement
		//std::chrono::steady_clock::time_point currentTime_;
		// time points that are used together with startTime_ to calculate elapsed time since the beginning of the measurement
		std::chrono::steady_clock::time_point currentOrientationTime_;
		std::chrono::steady_clock::time_point currentEMGTime_;
		// number of decimals for numbers in output files
		std::streamsize outputPrecision_ = 15;

		// EMG points are saved here
		std::vector<std::array<float, 16>> EMGData_;
		// stores elapsed time since the beginning of the measurement
		std::vector<double> orientationTimeVector_;
		// stores EMG for different sensors from the latest update
		std::array<float, 16> EMGDataPoints_ = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		// stores elapsed time since the beginning of tehe measurement
		std::vector<double> EMGTimeVector_;

	}; // end of class
}