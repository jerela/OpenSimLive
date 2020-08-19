// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <XsensDataReader.h>
#include <ThreadPoolContainer.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <thread>
#include <future>
#include <mutex>
#include <functional>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;


// This function fills a TimeSeriesTable with quaternion values for a single time frame.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> fillQuaternionTable(std::vector<MtwCallback*>& mtwCallbacks, std::vector<XsQuaternion>& quaternionVector)
{
	// get the number of active sensors
	int numberOfSensors = mtwCallbacks.size();

	// declare a vector for the sensor names in the OpenSim model
	std::vector<std::string> sensorNameVector;

	// define a vector with a single time value (such as zero) for constructing OpenSim::TimeSeriesTable
	std::vector<double> timeVector{ 0 };

	// initialize a matrix where each element is an entire quaternion, requires for constructing OpenSim::TimeSeriesTable
	SimTK::Matrix_<SimTK::Quaternion> quaternionMatrix(1, numberOfSensors);

	std::string currentSensorId; std::string sensorNameInModel;

	// "intermediary" variable for populating quaternionMatrix inside the for-loop
	SimTK::Quaternion_<SimTK::Real> quat;

	// populate sensorNameVector and quaternionMatrix
	for (size_t i = 0; i < mtwCallbacks.size(); ++i) {
		// get the ID of the current IMU
		currentSensorId = mtwCallbacks[i]->device().deviceId().toString().toStdString();

		// match the ID of the sensor to the name of the sensor on the model
		//sensorNameInModel = sensorIdToLabel(currentSensorId, "C:/Users/wksadmin/source/repos/OpenSimLive/Config/SensorMappings.xml");
		sensorNameInModel = sensorIdToLabel(currentSensorId, OPENSIMLIVE_ROOT+"/Config/"+ConfigReader("MainConfiguration.xml", "mappings_file"));

		// populate the vector of sensor names
		sensorNameVector.push_back(sensorNameInModel);

		// get the quaternions from XsQuaternion, put them into SimTK::Quaternion and put that quaternion into quaternionMatrix
		quat[0] = quaternionVector[i].w();
		quat[1] = quaternionVector[i].x();
		quat[2] = quaternionVector[i].y();
		quat[3] = quaternionVector[i].z();
		quaternionMatrix.set(0, i, quat);
	}

	// construct a TimeSeriesTable from the data we calculated in this function and return it
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> outputTable(timeVector, quaternionMatrix, sensorNameVector);
	return outputTable;
}

// This function calibrates an OpenSim model from setup file, similarly to how MATLAB scripting commands for OpenSense work.
std::string calibrateModelFromSetupFile(const std::string& IMUPlacerSetupFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quaternionTimeSeriesTable)
{
	// construct IMUPlacer
	OpenSimLive::IMUPlacerLive IMUPlacer(IMUPlacerSetupFile);

	// give the TimeSeriesTable of quaternions to IMUPlacer and run IMUPlacer to calibrate the model
	IMUPlacer.setQuaternion(quaternionTimeSeriesTable);
	IMUPlacer.run(false); // false as argument = do not visualize

	// add a station to a desired body in the calibrated .osim file if the parent body is named
	std::string stationParentBody = ConfigReader("MainConfiguration.xml", "station_parent_body");
	if (stationParentBody != "none") {
		// read the location of the station as a string
		std::string stationLocationString = ConfigReader("MainConfiguration.xml", "station_location");
		// write the location into doubles station_x,y,z using stringstream
		std::stringstream ss(stationLocationString);
		double station_x; ss >> station_x;
		double station_y; ss >> station_y;
		double station_z; ss >> station_z;
		// add the station under the parent body in the calibrated model file
		IMUPlacer.addStationToBody(stationParentBody, { station_x, station_y, station_z }, IMUPlacer.get_output_model_file());
	}

	return IMUPlacer.get_output_model_file();
}


void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, std::chrono::steady_clock::time_point& clockStart, std::chrono::duration<double>& clockDuration) {
	// calculate current duration
	clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
	// update current duration as time in IKTool
	IKTool.setTime(clockDuration.count());

	IKTool.update(false);
	std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
	double* mirrorTherapyPacket = &trackerResults[0];
}



void ConnectToDataStream(double inputSeconds, int inputThreads) {

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
	IKTool.setSaveIKResults(saveIKResults);
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

	unsigned int maxThreads = inputThreads;
	//std::vector<std::future<void>> futureVector;

	//ThreadPool threadPool(maxThreads);
	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);

	std::cout << "Entering measurement loop." << std::endl;
	int iteration = 0;
	auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::duration<double> clockDuration;

	// loop until enough time has been elapsed
	do {
		// update quaternions for IKTool
		IKTool.setQuaternion(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), xsensDataReader.GetQuaternionData(quaternionData)));
		
		// begin multithreading a function that consists of IK calculations + PointTracker
		threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(clockStart), std::ref(clockDuration));

		// increment iterations number
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
	std::cout << "Please input test duration in seconds: ";
	std::cin >> inputSecsStr;
	double inputSecs = stod(inputSecsStr);

	std::string inputThreadsStr;
	std::cout << "Please input the number of threads to be used: ";
	std::cin >> inputThreadsStr;
	int inputThreads = stoi(inputThreadsStr);

	std::cout << "Connecting to MTw Awinda data stream..." << std::endl;
	// connect to XSens IMUs, perform IK etc
	ConnectToDataStream(inputSecs, inputThreads);

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
