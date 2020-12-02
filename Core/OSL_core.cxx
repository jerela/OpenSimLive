// OSL_common.cxx : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <ThreadPoolContainer.h>
#include <IMUHandler.h>

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
}; // struct dataHolder ends



void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, VariableManager& vm, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable) {
	// calculate current duration
	vm.clockDuration = (std::chrono::high_resolution_clock::now() - vm.clockStart);
	// update current duration as time in IKTool
	IKTool.setTime(vm.clockDuration.count());

	IKTool.update(true, quatTable);
}


std::mutex mainMutex;
std::queue<OpenSim::TimeSeriesTableQuaternion> bufferContainer;
bool bufferInUse = false;
unsigned int maxBufferSize = 8;
unsigned int nIK = 0;
bool trialDone = false;
bool runProducerThread = true;
bool runIKThread = true;

// This function reads quaternion values in a loop and saves them in a queue buffer.
void producerThread(OpenSimLive::IMUHandler& genericDataReader) {
	do {
		// update new quaternion table but don't get it yet
		genericDataReader.updateQuaternionTable();

		// if consumer is not accessing the buffer and the buffer size is not maximal
		if (!bufferInUse && bufferContainer.size() < maxBufferSize)
		{
			// set bufferInUse to true to prevent consumer from accessing the buffer
			bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// push the time series table to the shared buffer
			bufferContainer.push(genericDataReader.getQuaternionTable());
			bufferInUse = false;
		}

	} while (runProducerThread);

	std::cout << "Producer done!" << std::endl;
}

// This function reads quaternion tables from a queue buffer and performs IK on them.
void consumerThread(VariableManager& vm, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSimLive::ThreadPoolContainer& threadPoolContainer) {
		do {
			// if producer is not accessing the buffer and the buffer contains values
			if (!bufferInUse && bufferContainer.size() > 0) {
				bufferInUse = true;
				std::lock_guard<std::mutex> lock(mainMutex);
				// get and pop the front of the queue
				OpenSim::TimeSeriesTableQuaternion quatTable(bufferContainer.front());
				bufferContainer.pop();
				bufferInUse = false;

				// start a new thread where the IK is calculated and increment the number of IK operations done
				threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(vm), quatTable);
				++nIK;
			}
		} while (runIKThread);
	std::cout << "Consumer done!" << std::endl;
}


int main(int argc, char* argv[])
{
	// Create a struct to hold a number of variables, and to pass them to functions
	VariableManager vm;

	// create Xsens connection object and connect the program to IMUs
	OpenSimLive::IMUHandler genericDataReader;
	//int desiredUpdateRate = stoi(ConfigReader("MainConfiguration.xml", "desired_update_rate"));
	//xsensDataReader.SetDesiredUpdateRate(desiredUpdateRate);

	IMUType manufacturer = simulated; // default to simulated if the if-statement below fails
	if (vm.manufacturer == "delsys")
		manufacturer = delsys;
	else if (vm.manufacturer == "xsens")
		manufacturer = xsens;
	else if (vm.manufacturer == "simulated")
		manufacturer = simulated;
	// set the value in the IMUHandler object
	genericDataReader.setManufacturer(manufacturer);
	// establish connection to IMUs
	genericDataReader.initialize();

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

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	// whether IKTool writes IK orientations into a .mot file when program finishes
	IKTool.setReportErrors(vm.saveIKResults);
	// whether PointTracker (through IKTool) writes calculated mirrored positions and rotations into a .sto file when program finishes
	IKTool.setSavePointTrackerResults(vm.enableMirrorTherapy);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();


	auto givemetime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cout << ctime(&givemetime) << std::endl;
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	//auto days = (ms.count() / 86400000) % 365;
	auto hours = (ms.count() / 3600000) % 24;
	auto minutes = (ms.count() / 60000) % 60;
	auto seconds = (ms.count() / 1000) % 60;
	auto millisecs = ms.count() % 1000;
	auto GMTOffset = 2;
	std::cout << ms.count() << std::endl;
	std::cout << hours+GMTOffset << ":" << minutes << ":" << seconds << "." << millisecs << std::endl;

	std::cout << "Entering data streaming and IK loop. Press C to calibrate model, Z to calculate IK once, N to enter continuous mode, M to exit continuous mode, V to enter send mode, B to exit send mode, L to save base reference orientation and X to quit." << std::endl;

	std::thread producer(producerThread, std::ref(genericDataReader));

	do
	{
		// get IMU orientation data in quaternions
		//genericDataReader.updateQuaternionTable();


		// if user hits the calibration key and new data is available
		if (calibrateModelKeyHit) {
			// set clock to start from calibration
			vm.clockStart = std::chrono::high_resolution_clock::now();
			// fill a timeseriestable with quaternion orientations of IMUs
			OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable = genericDataReader.getQuaternionTable();
			// calibrate the model and return its file name
			vm.calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
			// reset the keyhit so that we won't re-enter this if-statement before hitting the key again
			calibrateModelKeyHit = false;
			// give IKTool the necessary inputs and run it
			IKTool.setModelFile(vm.calibratedModelFile); // the model to perform IK on
			IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
			IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
			IKTool.setTime(0); // set the time of the first state as 0 at calibration
			IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
			// set private variables to be accessed in IK calculations
			IKTool.setPointTrackerEnabled(false);
			IKTool.run(true); // true for visualization
			std::cout << "Model has been calibrated." << std::endl;
		}

		// if user hits the single IK calculation key, new data is available and the model has been calibrated
		if (getDataKeyHit && !vm.calibratedModelFile.empty())
		{
			// use high resolution clock to count time since the IMU measurement began
			vm.clockNow = std::chrono::high_resolution_clock::now();
			vm.clockDuration = vm.clockNow - vm.clockStart; // time since calibration
			runIKThread = true;
			std::thread consumer(consumerThread, std::ref(vm), std::ref(IKTool), std::ref(threadPoolContainer));
			runIKThread = false;
			consumer.join();
			getDataKeyHit = false;
		}

		if (!vm.continuousMode && startContinuousModeKeyHit && !vm.calibratedModelFile.empty()) {
			std::cout << "Entering continuous mode." << std::endl;
			vm.continuousMode = true;
			startContinuousModeKeyHit = false;
			if (vm.resetClockOnContinuousMode && !(vm.clockDuration.count() > 0)) // ensure that the config setting is set to true and that this is the first time continuous mode is entered
				vm.clockStart = std::chrono::high_resolution_clock::now();
			runIKThread = true;
			std::thread consumer(consumerThread, std::ref(vm), std::ref(IKTool), std::ref(threadPoolContainer));
			consumer.detach();
		}

		if (vm.continuousMode && stopContinuousModeKeyHit) {
			std::cout << "Exiting continuous mode." << std::endl;
			runIKThread = false;
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
		}

	} while (vm.mainDataLoop);

	runProducerThread = false;
	runIKThread = false;

	std::cout << "Exiting main data loop!" << std::endl;

	// when exiting, save acquired data to file
	if (IKTool.get_report_errors())
	{
		std::cout << "Reporting IK to file..." << std::endl;
		IKTool.reportToFile();
	}

	// close the connection to IMUs
	genericDataReader.closeConnection();

	std::cout << "Program finished." << std::endl;
	return 1;
}
