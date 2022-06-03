// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <ThreadPoolContainer.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <thread>
#include <future>
#include <functional>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;


// Holds various variables that are passed to several functions so that function arguments can be reduced by passing only this struct or a reference to it.
struct VariableManager {
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = false;
	
	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop

	unsigned int orderIndex = 0;
	std::queue<OpenSim::TimeSeriesTableQuaternion> orientationBuffer;
	std::queue<double> timeBuffer;
	bool bufferInUse = false;
	unsigned int maxBufferSize = 8;
	std::atomic<bool> runProducerThread = true;
	std::atomic<bool> runIKThread = true;

}; // struct dataHolder ends









void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, VariableManager& vm, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, double time, unsigned int orderIndex) {
	IKTool.updateOrdered(false, quatTable, orderIndex, time);
}



std::mutex mainMutex;

// This function reads quaternion values in a loop and saves them in a queue buffer.
void producerThread(std::vector< OpenSim::TimeSeriesTable_<SimTK::Quaternion>>& quaternionVector, const std::vector<double>& timeVector, VariableManager& vm) {

	unsigned int data_index = 0;
	
	unsigned int data_n = timeVector.size();


		
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
			vm.timeBuffer.push(timeVector[data_index]);
			// push the time series table to the shared buffer
			vm.orientationBuffer.push(quaternionVector[data_index]);
			vm.bufferInUse = false;
			++data_index;
		}
		// If we reach the end of the data, finish program.
		if (data_index == data_n) {
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
			// start a new thread where the IK is calculated and increment the number of IK operations done
			threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(vm), quatTable, time, ++vm.orderIndex);
		}

	} while (vm.runIKThread);
	std::cout << "Consumer done!" << std::endl;
}








void manageIK(int inputThreads, double calibTime, double startTime, double endTime, std::string calibratedModelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> clippedTable) {


	VariableManager vm;

	vm.saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	vm.enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(vm.saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(clippedTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(calibTime); // set the time of the first state at calibration
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

	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);


	const std::vector<double> timeVector = clippedTable.getIndependentColumn();
	std::vector< OpenSim::TimeSeriesTable_<SimTK::Quaternion>> quatVector;
	for (unsigned int i = 0; i < timeVector.size(); ++i) {
		auto currentMatrix = clippedTable.updRowAtIndex(i);
		double currentTime = timeVector[i];
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> currentQuatTable(std::vector<double>({ currentTime }), currentMatrix.getAsMatrix(), clippedTable.getColumnLabels());
		quatVector.push_back(currentQuatTable);
	}


	std::cout << "Entering measurement loop." << std::endl;

	// reset clock
	vm.clockStart = std::chrono::high_resolution_clock::now();

	std::thread producer(producerThread, std::ref(quatVector), std::ref(timeVector), std::ref(vm));
	std::thread consumer(consumerThread, std::ref(vm), std::ref(IKTool), std::ref(threadPoolContainer));

	producer.join();
	consumer.join();

	// wait until all IK threads are finished, otherwise reporting IK to file may throw an exception
	threadPoolContainer.waitForFinish();

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


	return;
}


int main(int argc, char *argv[])
{



	std::string quatFileName;
	std::string inputThreadsStr;
	std::string calibTimeStr;
	std::string startTimeStr;
	std::string endTimeStr;
	std::string bodyStr;

	if (argc == 7) {
		quatFileName = argv[1];
		inputThreadsStr = argv[2];
		calibTimeStr = argv[3];
		startTimeStr = argv[4];
		endTimeStr = argv[5];
		bodyStr = argv[6];
	}
	else if (argc == 1) {

		std::cout << "Please input the name of the text file to be analyzed in /OpenSimLive-results/ folder: ";
		std::cin >> quatFileName;

		std::cout << "Please input the number of threads to be used: ";
		std::cin >> inputThreadsStr;


		std::cout << "Please input the time of IMU calibration: ";
		std::cin >> calibTimeStr;


		std::cout << "Please input IK start time: ";
		std::cin >> startTimeStr;


		std::cout << "Please input IK end time: ";
		std::cin >> endTimeStr;

	}
	else {
		std::cout << "Give command line in the following format: test_ik_throughput_from_file.exe quaternionTimeSeriesFileName.txt number_of_IK_threads calib_time IK_start_time IK_end_time" << std::endl;
		std::cout << "Alternatively, do not give any command line arguments and the program will let you input them manually." << std::endl;
	}


	int inputThreads = stoi(inputThreadsStr);
	double calibTime = stod(calibTimeStr);
	double startTime = stod(startTimeStr);
	double endTime = stod(endTimeStr);

	// parse the list of bodies given as command line argument into a vector of strings
	std::vector<std::string> bodiesToInclude;
	parse(bodiesToInclude, bodyStr, ",");


	// function that constructs a time series table of quaternions from quaternion time series text file
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable = quaternionTableFromTextFile(quatFileName);
	// function that calibrates the model and updates calibTime to match the closest similar time in the time series table
	std::string calibratedModelFile = calibrateModel(quatTable, calibTime);
	// function that clips the time series table for IK both time-wise and column-wise
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> clippedTable = clipDependentData(clipTable(quatTable, startTime, endTime), bodiesToInclude);

	/*for (unsigned int i = 0; i < bodiesToInclude.size(); ++i) {
		std::cout << bodiesToInclude[i] << std::endl;
	}*/

	// perform IK etc
	manageIK(inputThreads, calibTime, startTime, endTime, calibratedModelFile, clippedTable);

	std::cout << "Program finished." << std::endl;
	return 1;
}
