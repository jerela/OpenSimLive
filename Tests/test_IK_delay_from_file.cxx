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
#include <cmath>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;



void manageIK(double calibTime, double startTime, double endTime, std::string calibratedModelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> clippedTable) {

	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(clippedTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(calibTime); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
		IKTool.setPointTrackerEnabled(true);
		IKTool.setSavePointTrackerResults(true);
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}


	std::cout << "Entering measurement loop." << std::endl;
	std::chrono::duration<double> clockDuration;
	std::chrono::steady_clock::time_point clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	

	const std::vector<double> timeVector = clippedTable.getIndependentColumn();
	unsigned int data_n = timeVector.size();

	unsigned int iteration = 0;
	std::vector<double> IKTimes;
	do {
		// get quaternions
		double currentTime = timeVector[iteration];
		auto currentMatrix = clippedTable.updRowAtIndex(iteration);
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(std::vector<double>({ currentTime }), currentMatrix.getAsMatrix(), clippedTable.getColumnLabels());

		clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
		double time = clockDuration.count();
		IKTool.updateOrdered(false, quatTable, ++iteration, currentTime);
		clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
		double time2 = clockDuration.count();
		IKTimes.push_back(time2 - time);

	} while (iteration < data_n);



	clockDuration = std::chrono::high_resolution_clock::now() - clockStart;
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


	if (saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (iteration < 100000) {
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

	if (argc == 5) {
		quatFileName = argv[1];
		calibTimeStr = argv[2];
		startTimeStr = argv[3];
		endTimeStr = argv[4];
	}
	else if (argc == 1) {

		std::cout << "Please input the name of the text file to be analyzed in /OpenSimLive-results/ folder: ";
		std::cin >> quatFileName;


		std::cout << "Please input the time of IMU calibration: ";
		std::cin >> calibTimeStr;


		std::cout << "Please input IK start time: ";
		std::cin >> startTimeStr;


		std::cout << "Please input IK end time: ";
		std::cin >> endTimeStr;

	}
	else {
		std::cout << "Give command line in the following format: test_ik_delay_from_file.exe quaternionTimeSeriesFileName.txt calib_time IK_start_time IK_end_time" << std::endl;
		std::cout << "Alternatively, do not give any command line arguments and the program will let you input them manually." << std::endl;
	}

	double calibTime = stod(calibTimeStr);
	double startTime = stod(startTimeStr);
	double endTime = stod(endTimeStr);

	bool visualize = false;

	// function that constructs a time series table of quaternions from quaternion time series text file
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable = quaternionTableFromTextFile(quatFileName);
	// function that calibrates the model and updates calibTime to match the closest similar time in the time series table
	std::string calibratedModelFile = calibrateModel(quatTable, calibTime);
	// function that clips the time series table for IK
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> clippedTable = clipTable(quatTable, startTime, endTime);

	// perform IK etc
	manageIK(calibTime, startTime, endTime, calibratedModelFile, clippedTable);

	std::cout << "Program finished." << std::endl;
	return 1;
}
