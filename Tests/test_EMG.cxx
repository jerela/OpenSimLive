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



// IK for multithreading, each thread runs this function separately
void concurrentIK(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, const VariableManager& vm, Server& myLink) {
	IKTool.setTime(vm.clockDuration.count());
	// calculate the IK and update the visualization
	IKTool.update(true);
	if (vm.enableMirrorTherapy)
	{
		// get the data we want to send to Java program
		std::array<double, 6> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
		// get a double array from the double std::array
		//double* mirrorTherapyPacket = &trackerResults[0];
		// create mirrorTherapyPacket as a pointer to the underlying array of trackerResults
		double* mirrorTherapyPacket = trackerResults.data();
		// send the data
		if (vm.sendMode)
			myLink.SendDoubles(mirrorTherapyPacket, 6);
	}
}

// Runs IK and related shenanigans; this function is not yet multithreaded, but it contains the statement to begin multithreading
void RunIKProcedure(OpenSimLive::DelsysDataReader& delsysDataReader, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, Server& myLink, OpenSimLive::ThreadPoolContainer& threadPoolContainer, const VariableManager& vm) {
	// fill a time series table with quaternion orientations of the IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(delsysDataReader.getTimeSeriesTable());

	// get the orientation of station_reference_body and pass it to PointTracker through IKTool; MOVE THIS TO CONCURRENTIK TO SPEED UP PROGRAM SLIGHTLY
	if (IKTool.getUseReferenceRotation())
	{
		bool foundBodyIMUOrientation = false;
		for (unsigned int i = 0; i < quatTable.getNumColumns(); ++i) // iterate through the quaternions of all IMUs
		{
			if (vm.stationReferenceBody + "_imu" == quatTable.getColumnLabel(i)) // if we find the data of the IMU on station_reference_body
			{
				// get the quaternion orientation of that IMU
				SimTK::Quaternion_<SimTK::Real> quat = quatTable.getMatrixBlock(0, i, 1, 1)[0][0];
				// pass it on to IKTool that inherits PointTracker, and use it at the end of PointTracker rotation calculations
				IKTool.setReferenceBodyRotation(quat);
				foundBodyIMUOrientation = true;
				break;
			}
		}
		if (!foundBodyIMUOrientation)
		{
			std::cout << "Base body IMU orientation not found! Disabling feature." << std::endl;
			IKTool.setUseReferenceRotation(false);
		}
	}

	// give the necessary inputs to IKTool
	IKTool.setQuaternion(quatTable);

	// Send a function to be multithreaded
	threadPoolContainer.offerFuture(concurrentIK, std::ref(IKTool), std::ref(vm), std::ref(myLink));

}




void EMGThread(OpenSimLive::DelsysDataReader& delsysDataReader) {
	std::unique_lock<std::mutex> delsysMutex(mainMutex);
	delsysDataReader.updateEMG();
	delsysMutex.unlock();
}

void orientationThread(OpenSimLive::DelsysDataReader& delsysDataReader) {
	std::unique_lock<std::mutex> delsysMutex(mainMutex);
	delsysDataReader.updateQuaternionData();
	delsysMutex.unlock();
}


void ConnectToDataStream() {

	// create Delsys connection object
	OpenSimLive::DelsysDataReader delsysDataReader;
	while (!delsysDataReader.initiateConnection()) {}

	
	// Create a struct to hold a number of variables, and to pass them to functions
	VariableManager vm;

	// enable or disable Python-based EMG plotting
	delsysDataReader.setPlotterEnabled(vm.enableEMGPlotting);

	bool getDataKeyHit = false; // tells if the key that initiates a single IK calculation is hit
	bool referenceBaseRotationKeyHit = false; // tells if the key that initiates the fetching of the current rotation of the base IMU is hit
	bool calibrateModelKeyHit = false; // tells if the key that initiates model calibration is hit
	bool startContinuousModeKeyHit = false; // tells if the key that starts continuous mode is hit
	bool stopContinuousModeKeyHit = false; // tells if the key that pauses continuous mode is hit
	bool startSendModeKeyHit = false; // tells if the key that starts send mode is hit
	bool stopSendModeKeyHit = false; // tells if the key that pauses send mode is hit

	// initialize the object that handles multithreading
	OpenSimLive::ThreadPoolContainer threadPoolContainer(vm.maxThreads);

	// get the current times
	vm.clockStart = std::chrono::high_resolution_clock::now();
	vm.clockNow = vm.clockStart;
	vm.clockPrev = vm.clockStart;
		
		

	std::cout << "Entering data streaming and IK loop. Press C to calibrate model, Z to calculate IK once, N to enter continuous mode, M to exit continuous mode, V to enter send mode, B to exit send mode, L to save base reference orientation and X to quit." << std::endl;

	// Send a function to be multithreaded
	//threadPoolContainer.offerFuture(EMGThreadLoop, std::ref(delsysDataReader), std::ref(vm));

	do
	{
		// show EMG data
		//delsysDataReader.updateEMG();
		threadPoolContainer.offerFuture(EMGThread, std::ref(delsysDataReader));
		bool newDataAvailable = true;


		// if new data is available and continuous mode has been switched on
		if (newDataAvailable && vm.continuousMode) {
			// use high resolution clock to count time since the IMU measurement began
			vm.clockNow = std::chrono::high_resolution_clock::now();
			// calculate the duration since the beginning of counting
			vm.clockDuration = vm.clockNow - vm.clockStart;
			// calculate the duration since the previous time IK was continuously calculated
			vm.prevDuration = vm.clockNow - vm.clockPrev;
			// if more than the set time delay has passed since the last time IK was calculated
			if (vm.prevDuration.count()*1000 > vm.continuousModeMsDelay) {
				// set current time as the time IK was previously calculated for the following iterations of the while-loop
				vm.clockPrev = vm.clockNow;
				//RunIKProcedure(delsysDataReader, IKTool, myLink, threadPoolContainer, vm);
			}
		}

		if (!vm.continuousMode && startContinuousModeKeyHit && !vm.calibratedModelFile.empty()) {
			std::cout << "Entering continuous mode." << std::endl;
			vm.continuousMode = true;
			startContinuousModeKeyHit = false;
			if (vm.resetClockOnContinuousMode && !(vm.clockDuration.count() > 0) ) // ensure that the config setting is set to true and that this is the first time continuous mode is entered
				vm.clockStart = std::chrono::high_resolution_clock::now();
		}

		if (vm.continuousMode && stopContinuousModeKeyHit) {
			std::cout << "Exiting continuous mode." << std::endl;
			vm.continuousMode = false;
			stopContinuousModeKeyHit = false;
		}

		char hitKey = ' ';
		if (_kbhit())
		{
			hitKey = toupper((char)_getch());
			vm.mainDataLoop = (hitKey != 'X'); // stay in main data loop as long as we don't hit X
			getDataKeyHit = (hitKey == 'Z');
			calibrateModelKeyHit = (hitKey == 'C');
			startContinuousModeKeyHit = (hitKey == 'N');
			stopContinuousModeKeyHit = (hitKey == 'M');
			startSendModeKeyHit = (hitKey == 'V');
			stopSendModeKeyHit = (hitKey == 'B');
			referenceBaseRotationKeyHit = (hitKey == 'L');
		}
			
	} while (vm.mainDataLoop);

	std::cout << "Exiting main data loop!" << std::endl;

	// close the connection to IMUs
	delsysDataReader.closeConnection();

	return;
}


int main(int argc, char *argv[])
{
    if (argc < 2) {
		// report version
		std::cout << argv[0] << " version " << OpenSimLive_VERSION_MAJOR << "." << OpenSimLive_VERSION_MINOR << std::endl;
		std::cout << "Usage: " << argv[0] << " number" << std::endl;
		


		std::cout << "Connecting to MTw Awinda data stream..." << std::endl;
		// connect to XSens IMUs, perform IK etc
		ConnectToDataStream();

		
		std::cout << "Program finished." << std::endl;
		return 1;
	}
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
