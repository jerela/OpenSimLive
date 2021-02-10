// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <ThreadPoolContainer.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>
#include <thread>
#include <future>
#include <functional>
#include <IMUHandler.h>
#include <cmath>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// Holds various variables that are passed to several functions so that function arguments can be reduced by passing only this struct or a reference to it.
struct VariableManager {
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool mainDataLoop = true; // IMU data is being measured and analyzed while this is true
	bool continuousMode = false; // IK is calculated continuously while this is true
	bool sendMode = false; // data is sent to client with each IK step while this is true
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	int continuousModeMsDelay = std::stoi(ConfigReader("MainConfiguration.xml", "continuous_mode_ms_delay")); // delay between consequent IK calculations in consequent mode, in milliseconds
	bool print_roll_pitch_yaw = ("true" == ConfigReader("MainConfiguration.xml", "print_roll_pitch_yaw")); // boolean that tells whether to print roll, pitch and yaw of IMUs while calculating IK
	bool resetClockOnContinuousMode = ("true" == ConfigReader("MainConfiguration.xml", "reset_clock_on_continuous_mode")); // if true, clock will be reset to zero when entering continuous mode; if false, the clock will be set to zero at calibration
	bool enableMirrorTherapy = false;
	unsigned int maxThreads = stoi(ConfigReader("MainConfiguration.xml", "threads")); // get the maximum number of concurrent threads for multithreading
	std::string manufacturer = ConfigReader("MainConfiguration.xml", "IMU_manufacturer");
	std::string stationReferenceBody = ConfigReader("MainConfiguration.xml", "station_reference_body"); // get the name of the reference body used in mirror therapy

	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::steady_clock::time_point clockNow = std::chrono::high_resolution_clock::now(); // this value will be updated in the loop
	std::chrono::steady_clock::time_point clockPrev = clockStart; // this value will be updated in the loop to present the time point of the previous IK calculation
	std::chrono::duration<double> clockDuration;
	std::chrono::duration<double> prevDuration;

	unsigned int orderIndex = 0;
	std::queue<OpenSim::TimeSeriesTableQuaternion> orientationBuffer;
	std::queue<double> timeBuffer;
	bool bufferInUse = false;
	unsigned int maxBufferSize = 8;
	bool trialDone = false;
	std::atomic<bool> runProducerThread = true;
	std::atomic<bool> runIKThread = true;
	double trialDuration = 0;

}; // struct dataHolder ends



void ConnectToDataStream() {
	OpenSimLive::IMUHandler genericDataReader;

	std::string manufacturerStr = ConfigReader("MainConfiguration.xml", "IMU_manufacturer");
	IMUType manufacturer = simulated; // default to simulated in case the following if-statements fail
	if (manufacturerStr == "delsys")
		manufacturer = delsys;
	else if (manufacturerStr == "xsens")
		manufacturer = xsens;
	else if (manufacturerStr == "simulated")
		manufacturer = simulated;

	genericDataReader.setManufacturer(manufacturer);

	genericDataReader.initialize();

	VariableManager vm;

		
	//std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	vm.saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	vm.enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(vm.saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// CALIBRATION STEP
	genericDataReader.updateQuaternionTable();
	// fill a timeseriestable with quaternion orientations of IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable = genericDataReader.getQuaternionTable();
	// calibrate the model and return its file name
	vm.calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(vm.calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(0); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (vm.enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
		IKTool.setPointTrackerEnabled(true);
		IKTool.setSavePointTrackerResults(true);
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}


	std::cout << "Entering measurement loop." << std::endl;
	//int iteration = 0;
	//auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::duration<double> clockDuration;
	vm.clockStart = std::chrono::high_resolution_clock::now();

	std::cout << vm.clockDuration.count() << std::endl;


	unsigned int iteration = 0;
	std::vector<double> IKTimes;
	do {
		OpenSim::TimeSeriesTableQuaternion quatTable = genericDataReader.updateAndGetQuaternionTable();
		clockDuration = (std::chrono::high_resolution_clock::now() - vm.clockStart);
		double time = clockDuration.count();
		IKTool.updateOrdered(false, quatTable, ++iteration, time);
		clockDuration = (std::chrono::high_resolution_clock::now() - vm.clockStart);
		double time2 = clockDuration.count();
		IKTimes.push_back(time2 - time);

	} while (iteration < 10000);



	clockDuration = std::chrono::high_resolution_clock::now() - vm.clockStart;
	double finalTime = clockDuration.count();

	std::cout << "Performed " << iteration << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)iteration / finalTime) << " iterations per second." << std::endl;

	double timeSum = 0;
	for (unsigned int k = 0; k < IKTimes.size(); ++k) {
		timeSum += IKTimes[k];
	}
	
	double meanDelay = timeSum / (double)iteration;

	double stdDiff = 0;
	for (unsigned int k = 0; k < IKTimes.size(); ++k) {
		stdDiff += (IKTimes[k] - meanDelay) * (IKTimes[k] - meanDelay);
	}
	double std = std::sqrt(stdDiff / (double)iteration);

	std::cout << "Mean delay: " << meanDelay*1000 << " ms, std: " << std*1000 << " ms" << std::endl;


	if (vm.saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (vm.orderIndex < 100000) {
			IKTool.reportToFile();
		}
		else
		{
			std::cout << "More than 100 000 iterations calculated, as a safety precaution program is not saving results to file!" << std::endl;
		}
	}

	// close the connection to IMUs
	genericDataReader.closeConnection();

	return;
}


int main(int argc, char *argv[])
{

	std::cout << "Connecting..." << std::endl;
	// connect to XSens IMUs, perform IK etc
	ConnectToDataStream();

	std::cout << "Program finished." << std::endl;
	return 1;
}
