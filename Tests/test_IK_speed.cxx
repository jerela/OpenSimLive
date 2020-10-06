// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <XsensDataReader.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

void ConnectToDataStream(int inputSeconds) {

	// create Xsens connection object and connect the program to IMUs
	OpenSimLive::XsensDataReader xsensDataReader;
	int desiredUpdateRate = stoi(ConfigReader("MainConfiguration.xml", "desired_update_rate"));
	xsensDataReader.SetDesiredUpdateRate(desiredUpdateRate);
	xsensDataReader.InitiateStartupPhase();

	std::vector<XsQuaternion> quaternionData(xsensDataReader.GetMtwCallbacks().size()); // for data in quaternion form
		
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// CALIBRATION STEP

	// get IMU orientation data in quaternions
	quaternionData = xsensDataReader.GetQuaternionData(quaternionData);
	// fill a timeseriestable with quaternion orientations of IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
	// calibrate the model and return its file name
	calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(0); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}

	std::cout << "Entering measurement loop." << std::endl;
	int iteration = 0;
	auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::duration<double> clockDuration;


	do {
		// get IMU orientation data in quaternions
		quaternionData = xsensDataReader.GetQuaternionData(quaternionData);

		// fill a time series table with quaternion orientations of the IMUs
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));

		// give the necessary inputs to IKTool
		IKTool.setQuaternion(quatTable);
		clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
		IKTool.setTime(clockDuration.count());

		// calculate the IK and update the visualization
		IKTool.update(false);

		if (enableMirrorTherapy)
		{
			// get the data we want to send to Java program
			std::array<double, 6> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
			// get a double array from the double vector
			//double* mirrorTherapyPacket = &trackerResults[0];
			double* mirrorTherapyPacket = trackerResults.data();
		}

		++iteration;
		
	} while (clockDuration.count() < inputSeconds);

	double finalTime = clockDuration.count();

	std::cout << "Performed " << iteration << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)iteration / finalTime) << " iterations per second." << std::endl;

	if (saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (iteration < 1000) {
			IKTool.reportToFile();
		}
		else
		{
			std::cout << "More than 1000 iterations calculated, as a safety precaution program is not saving results to file!" << std::endl;
		}
	}

	// close the connection to IMUs
	xsensDataReader.CloseConnection();

	return;
}


int main(int argc, char *argv[])
{
	std::string inputSecsStr;
	std::cout << "How many seconds will we run the program: ";
	std::cin >> inputSecsStr;
	int inputSecs = stoi(inputSecsStr);

	std::cout << "Connecting to MTw Awinda data stream..." << std::endl;
	// connect to XSens IMUs, perform IK etc
	ConnectToDataStream(inputSecs);

	std::cout << "Program finished." << std::endl;
	return 1;
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
