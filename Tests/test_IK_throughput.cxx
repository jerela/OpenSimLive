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

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

//std::vector<double> enterTimes;
//std::vector<double> finishTimes;

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



void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, VariableManager& vm, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, double time, unsigned int orderIndex) {
	IKTool.updateOrdered(false, quatTable, orderIndex, time);
}



std::mutex mainMutex;

// This function reads quaternion values in a loop and saves them in a queue buffer.
void producerThread(OpenSimLive::IMUHandler& genericDataReader, VariableManager& vm) {
	
	
	// update new quaternion table but don't get it yet; do this ONLY ONCE to do all the IK with the same orientations
	genericDataReader.updateQuaternionTable();
	
	do {

		// get time stamp
		std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - vm.clockStart;
		double time = duration.count();

		

		// if consumer is not accessing the buffer and the buffer size is not maximal
		if (!vm.bufferInUse && vm.orientationBuffer.size() < vm.maxBufferSize)
		{
			// set bufferInUse to true to prevent consumer from accessing the buffer
			vm.bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// push time stamp to the shared buffer
			vm.timeBuffer.push(time);
			// push the time series table to the shared buffer
			vm.orientationBuffer.push(genericDataReader.getQuaternionTable());
			vm.bufferInUse = false;
		}
		if (time > vm.trialDuration) {
			vm.runIKThread = false;
			vm.runProducerThread = false;
		}

	} while (vm.runProducerThread);

	std::cout << "Producer done!" << std::endl;
}

// This function reads quaternion tables from a queue buffer and performs IK on them.
void consumerThread(VariableManager& vm, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSimLive::ThreadPoolContainer& threadPoolContainer) {

	do {
		// if producer is not accessing the buffer and the buffer contains values
		if (!vm.bufferInUse && vm.orientationBuffer.size() > 0) {
			vm.bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// get and pop time from the buffer
			double time = vm.timeBuffer.front();
			vm.timeBuffer.pop();
			// get and pop the front of the queue
			OpenSim::TimeSeriesTableQuaternion quatTable(vm.orientationBuffer.front());
			vm.orientationBuffer.pop();
			vm.bufferInUse = false;
			++vm.orderIndex;
			// start a new thread where the IK is calculated and increment the number of IK operations done
			threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(vm), quatTable, time, vm.orderIndex);
		}

	} while (vm.runIKThread);
	std::cout << "Consumer done!" << std::endl;
}








void ConnectToDataStream(double inputSeconds, int inputThreads) {
	//enterTimes.reserve(100000);
	//finishTimes.reserve(100000);
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

	vm.trialDuration = inputSeconds;
		
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

	unsigned int maxThreads = inputThreads;
	//std::vector<std::future<void>> futureVector;

	//ThreadPool threadPool(maxThreads);
	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);

	std::cout << "Entering measurement loop." << std::endl;
	//int iteration = 0;
	vm.orderIndex = 0;
	//auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	//std::chrono::duration<double> clockDuration;
	vm.clockStart = std::chrono::high_resolution_clock::now();

	std::cout << vm.clockDuration.count() << std::endl;
	// loop until enough time has been elapsed
	//auto table = genericDataReader.updateAndGetQuaternionTable();


	std::thread producer(producerThread, std::ref(genericDataReader), std::ref(vm));
	std::thread consumer(consumerThread, std::ref(vm), std::ref(IKTool), std::ref(threadPoolContainer));

	producer.join();
	consumer.join();

	std::chrono::duration<double> clockDuration = std::chrono::high_resolution_clock::now() - vm.clockStart;
	double finalTime = clockDuration.count();

	std::cout << "Performed " << vm.orderIndex << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)vm.orderIndex / finalTime) << " iterations per second." << std::endl;

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

	/*std::vector<double> IKTimes;
	double mean = 0;
	for (unsigned int j = 0; j < finishTimes.size(); ++j) {
		IKTimes.push_back(finishTimes[j]-enterTimes[j]);
		std::cout << finishTimes[j] << " - " << enterTimes[j] << " = " << IKTimes[j] << std::endl;
		mean += IKTimes[j];
	}
	std::cout << "Mean: " << mean/(double)finishTimes.size() << std::endl;*/

	// close the connection to IMUs
	genericDataReader.closeConnection();

	return;
}


int main(int argc, char *argv[])
{
	std::string inputSecsStr;
	std::cout << "Please input test duration in seconds: ";
	std::cin >> inputSecsStr;
	double inputSecs = stod(inputSecsStr);

	std::string inputThreadsStr;
	std::cout << "Please input the number of threads to be used: ";
	std::cin >> inputThreadsStr;
	int inputThreads = stoi(inputThreadsStr);

	std::cout << "Connecting..." << std::endl;
	// connect to XSens IMUs, perform IK etc
	ConnectToDataStream(inputSecs, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
