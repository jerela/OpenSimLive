// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <Server.h>
#include <DelsysDataReader.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <ThreadPoolContainer.h>
#include <mutex>
#ifdef PYTHON_ENABLED
#include <PythonPlotter.h>
#endif

std::mutex mainMutex;

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// Holds various variables that are passed to several functions so that function arguments can be reduced by passing only this struct or a reference to it.
struct VariableManager {
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool mainDataLoop = true; // IMU data is being measured and analyzed while this is true
	bool continuousMode = false; // IK is calculated continuously while this is true
	bool sendMode = false; // data is sent to client with each IK step while this is true
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	int continuousModeMsDelay = std::stoi(ConfigReader("MainConfiguration.xml", "continuous_mode_ms_delay")); // delay between consequent IK calculations in consequent mode, in milliseconds
	bool resetClockOnContinuousMode = ("true" == ConfigReader("MainConfiguration.xml", "reset_clock_on_continuous_mode")); // if true, clock will be reset to zero when entering continuous mode; if false, the clock will be set to zero at calibration
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false
	bool enableEMGPlotting = ("true" == ConfigReader("MainConfiguration.xml", "enable_EMG_plotting")); // if true, PythonPlotter will be used to visualize live EMG data, but with extremely low frequency
	unsigned int maxThreads = stoi(ConfigReader("MainConfiguration.xml", "threads")); // get the maximum number of concurrent threads for multithreading
	std::string stationReferenceBody = ConfigReader("MainConfiguration.xml", "station_reference_body"); // get the name of the reference body used in mirror therapy

	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::steady_clock::time_point clockNow = std::chrono::high_resolution_clock::now(); // this value will be updated in the loop
	std::chrono::steady_clock::time_point clockPrev = clockStart; // this value will be updated in the loop to present the time point of the previous IK calculation
	std::chrono::duration<double> clockDuration;
	std::chrono::duration<double> prevDuration;
}; // struct dataHolder ends





void EMGThread(OpenSimLive::DelsysDataReader& delsysDataReader) {
	//std::unique_lock<std::mutex> delsysMutex(mainMutex);
	delsysDataReader.updateEMG();
	//delsysMutex.unlock();
}



int main(int argc, char *argv[])
{

	// create Delsys connection object
	OpenSimLive::DelsysDataReader delsysDataReader;
	while (!delsysDataReader.initiateConnection()) {}


	// Create a struct to hold a number of variables, and to pass them to functions
	VariableManager vm;


	bool getDataKeyHit = false; // tells if the key that initiates a single IK calculation is hit
	bool referenceBaseRotationKeyHit = false; // tells if the key that initiates the fetching of the current rotation of the base IMU is hit
	bool calibrateModelKeyHit = false; // tells if the key that initiates model calibration is hit
	bool startContinuousModeKeyHit = false; // tells if the key that starts continuous mode is hit
	bool stopContinuousModeKeyHit = false; // tells if the key that pauses continuous mode is hit
	bool startSendModeKeyHit = false; // tells if the key that starts send mode is hit
	bool stopSendModeKeyHit = false; // tells if the key that pauses send mode is hit

	// initialize the object that handles multithreading
	OpenSimLive::ThreadPoolContainer threadPoolContainer(vm.maxThreads);


#ifdef PYTHON_ENABLED
	// initialize and prepare PythonPlotter
	PythonPlotter pythonPlotter;
	pythonPlotter.setMaxSize(50);
	pythonPlotter.setSubPlots(delsysDataReader.getNActiveSensors());
	pythonPlotter.prepareGraph();
#endif

	// get the current times and start counting
	vm.clockStart = std::chrono::high_resolution_clock::now();
	vm.clockNow = vm.clockStart;
	vm.clockPrev = vm.clockStart;

	std::cout << "Entering EMG reading loop. Press X to quit." << std::endl;

	// Send a function to be multithreaded
	//threadPoolContainer.offerFuture(EMGThreadLoop, std::ref(delsysDataReader), std::ref(vm));

	do
	{

		// update time
		vm.clockNow = std::chrono::high_resolution_clock::now();
		vm.clockDuration = vm.clockNow - vm.clockStart;
		double elapsedTime = vm.clockDuration.count();

		// update EMG and set time for DelsysDataReader
		delsysDataReader.updateEMG();
		//threadPoolContainer.offerFuture(EMGThread, std::ref(delsysDataReader));

		// send EMG data points to PythonPlotter
#ifdef PYTHON_ENABLED
		if (vm.enableEMGPlotting) {
			pythonPlotter.setTime(elapsedTime);
			pythonPlotter.setYData(delsysDataReader.getLatestEMGValues());
			pythonPlotter.updateGraph();
		}
#endif

		/*
		// use high resolution clock to count time since the measurement started
		vm.clockNow = std::chrono::high_resolution_clock::now();
		// calculate the duration since the beginning of counting
		vm.clockDuration = vm.clockNow - vm.clockStart;
		// calculate the duration since the previous time IK was continuously calculated
		vm.prevDuration = vm.clockNow - vm.clockPrev;
		// if more than the set time delay has passed since the last time IK was calculated
		if (vm.prevDuration.count()*1000 > vm.continuousModeMsDelay) {
			// set current time as the time IK was previously calculated for the following iterations of the while-loop
			vm.clockPrev = vm.clockNow;
			delsysDataReader.updateEMG();
		}*/



		char hitKey = ' ';
		if (_kbhit())
		{
			hitKey = toupper((char)_getch());
			vm.mainDataLoop = (hitKey != 'X'); // stay in main data loop as long as we don't hit X
		}

	} while (vm.mainDataLoop);

	std::cout << "Exiting main data loop!" << std::endl;

	// close the connection to IMUs
	delsysDataReader.closeConnection();


		
	std::cout << "Program finished." << std::endl;
	return 1;
	
}


