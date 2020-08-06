// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include <OpenSimLiveConfig.h>
#include <OpenSim.h>

#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <Server.h>
#include <XsensDataReader.h>

#include "conio.h" // for non-ANSI _kbhit() and _getch()

#include <XMLFunctions.h>


const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;


void printRollPitchYaw(std::vector<MtwCallback*> mtwCallbacks, std::vector<XsEuler> eulerData) {
	for (size_t i = 0; i < mtwCallbacks.size(); ++i)
	{
		std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
			<< ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
			<< ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
			<< ", Yaw: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
			<< std::endl;
	}
}

// This function fills a TimeSeriesTable with quaternion values for a single time frame.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> fillQuaternionTable(std::vector<MtwCallback*> mtwCallbacks, std::vector<XsQuaternion> quaternionVector)
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
std::string calibrateModelFromSetupFile(std::string IMUPlacerSetupFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quaternionTimeSeriesTable)
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

// This function calculates the values for all joint angles of the model based on live IMU data.
std::vector<double> OpenSimInverseKinematicsFromIMUs(std::string modelFileName, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, double duration, bool initialCall) {

	OpenSimLive::IMUInverseKinematicsToolLive IKTool(modelFileName, quatTable);
	IKTool.setTime(duration);
	IKTool.run(true);
	


	return IKTool.getQ();
}




void ConnectToDataStream() {

	// create Xsens connection object and connect the program to IMUs
	OpenSimLive::XsensDataReader xsensDataReader;
	xsensDataReader.InitiateStartupPhase();
		



	std::vector<XsEuler> eulerData(xsensDataReader.GetMtwCallbacks().size()); // Room to store euler data for each mtw
	std::vector<XsQuaternion> quaternionData(xsensDataReader.GetMtwCallbacks().size()); // for data in quaternion form
		
	std::string calibratedModelFile;

	bool mainDataLoop = true; // IMU data is being measured while this is true
	bool continuousMode = false; // IK is calculated continuously while this is true
	bool getDataKeyHit = false; // tells if the key that initiates a single IK calculation is hit
	bool calibrateModelKeyHit = false; // tells if the key that initiates model calibration is hit
	bool startContinuousModeKeyHit = false; // tells if the key that starts continuous mode is hit
	bool stopContinuousModeKeyHit = false; // tells if the key that ends continuous mode is hit
	int continuousModeMsDelay = std::stoi(ConfigReader("MainConfiguration.xml", "continuous_mode_ms_delay")); // delay between consequent IK calculations in consequent mode, in milliseconds
	bool print_roll_pitch_yaw = ("true" == ConfigReader("MainConfiguration.xml", "print_roll_pitch_yaw")); // boolean that tells whether to print roll, pitch and yaw of IMUs while calculating IK
	bool resetClockOnContinuousMode = ("true" == ConfigReader("MainConfiguration.xml", "reset_clock_on_continuous_mode")); // if true, clock will be reset to zero when entering continuous mode; if false, the clock will be set to zero at calibration
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false
	std::vector<std::vector<double>> jointAngles; // vector that will hold the joint angles

	auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	auto clockNow = std::chrono::high_resolution_clock::now(); // this value will be updated in the loop
	auto clockPrev = clockStart; // this value will be updated in the loop to present the time point of the previous IK calculation
	std::chrono::duration<double> clockDuration; std::chrono::duration<double> prevDuration;

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
		
	// SOCKET COMMUNICATION
	bool bResult = false;
	int port = std::stoi(ConfigReader("MainConfiguration.xml", "socket_port"));
	int dataport = -1;
	Server myLink(port, dataport, &bResult);
	if (!bResult)
	{
		printf("Failed to create Server object!\n");
	}
	if (enableMirrorTherapy) {	
		std::cout << "Waiting for client program to connect..." << std::endl;
		myLink.Connect();
		std::cout << "Client program connected." << std::endl;
	}
		
		

	std::cout << "Entering data streaming and IK loop. Press C to calibrate model, V to calculate IK, N to enter continuous mode, M to exit continuous mode and X to quit." << std::endl;

	do
	{
		// get IMU orientation data in quaternions
		quaternionData = xsensDataReader.GetQuaternionData();
		if (print_roll_pitch_yaw)
			eulerData = xsensDataReader.GetEulerData();
		// update the boolean value to see if new data is available since orientation data was last retrieved
		bool newDataAvailable = xsensDataReader.GetNewDataAvailable();

		// if user hits the calibration key and new data is available
		if (newDataAvailable && calibrateModelKeyHit) {
			// set clock to start from calibration
			clockStart = std::chrono::high_resolution_clock::now();
			// fill a timeseriestable with quaternion orientations of IMUs
			OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
			// calibrate the model and return its file name
			calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
			// reset the keyhit so that we won't re-enter this if-statement before hitting the key again
			calibrateModelKeyHit = false;
			// give IKTool the necessary inputs and run it
			IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
			IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
			IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
			IKTool.setTime(0); // set the time of the first state as 0 at calibration
			IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
			IKTool.run(true); // true for visualization
			std::cout << "Model has been calibrated." << std::endl;

			// set private variables to be accessed in IK calculations
			if (enableMirrorTherapy == true) {
				IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
				IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
			}
			else {
				IKTool.setPointTrackerEnabled(false);
			}
		}

		// if user hits the single IK calculation key, new data is available and the model has been calibrated
		if (newDataAvailable && getDataKeyHit && !calibratedModelFile.empty())
		{
			// use high resolution clock to count time since the IMU measurement began
			clockNow = std::chrono::high_resolution_clock::now();
			clockDuration = clockNow - clockStart; // time since calibration
			// fill a time series table with quaternion orientations of the IMUs
			OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
			// give the necessary inputs to IKTool
			IKTool.setQuaternion(quatTable); // update the IMU orientations
			IKTool.setTime(clockDuration.count()); // time since calibration given as the time of the state to perform IK on
			// calculate the IK and update the visualization
			IKTool.update(true);
			// push joint angles to vector
			//jointAngles.push_back(IKTool.getQ());
				
			// print the roll, pitch and yaw angles for all IMUs
			if (print_roll_pitch_yaw)
				printRollPitchYaw(xsensDataReader.GetMtwCallbacks(), eulerData);
			//std::cout << "Positions: " << "[" << trackerResults[0] << ", " << trackerResults[1] << ", " << trackerResults[2] << "]" << std::endl;
			//std::cout << "Rotations: " << "[" << trackerResults[3] << ", " << trackerResults[4] << ", " << trackerResults[5] << "]" << std::endl;
			if (enableMirrorTherapy)
			{
				// get the data we want to send to Java program
				std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
				// get a double array from the double vector
				double* mirrorTherapyPacket = &trackerResults[0];
				// send the data
				myLink.SendDoubles(mirrorTherapyPacket, 6);
			}
			getDataKeyHit = false;
		}

		// if new data is available and continuous mode has been switched on
		if (newDataAvailable && continuousMode) {
			// use high resolution clock to count time since the IMU measurement began
			clockNow = std::chrono::high_resolution_clock::now();
			// calculate the duration since the beginning of counting
			clockDuration = clockNow - clockStart;
			// calculate the duration since the previous time IK was continuously calculated
			prevDuration = clockNow - clockPrev;
			// if more than the set time delay has passed since the last time IK was calculated
			if (prevDuration.count()*1000 > continuousModeMsDelay) {
				// set current time as the time IK was previously calculated for the following iterations of the while-loop
				clockPrev = clockNow;

				// fill a time series table with quaternion orientations of the IMUs
				OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
				// give the necessary inputs to IKTool
				IKTool.setQuaternion(quatTable);
				IKTool.setTime(clockDuration.count());
				// calculate the IK and update the visualization
				IKTool.update(true);
				// push joint angles to vector
				jointAngles.push_back(IKTool.getQ());
				if (print_roll_pitch_yaw)
					printRollPitchYaw(xsensDataReader.GetMtwCallbacks(), eulerData);
					
				if (enableMirrorTherapy)
				{
					// get the data we want to send to Java program
					std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
					// get a double array from the double vector
					double* mirrorTherapyPacket = &trackerResults[0];
					// send the data
					myLink.SendDoubles(mirrorTherapyPacket, 6);
				}
				//std::cout << "Positions: " << "[" << trackerResults[0] << ", " << trackerResults[1] << ", " << trackerResults[2] << "]" << std::endl;
				//std::cout << "Rotations: " << "[" << trackerResults[3] << ", " << trackerResults[4] << ", " << trackerResults[5] << "]" << std::endl;
			}
		}

		if (!continuousMode && startContinuousModeKeyHit && !calibratedModelFile.empty()) {
			std::cout << "Entering continuous mode." << std::endl;
			continuousMode = true;
			startContinuousModeKeyHit = false;
			if (resetClockOnContinuousMode && !(clockDuration.count() > 0) ) // ensure that the config setting is set to true and that this is the first time continuous mode is entered
				clockStart = std::chrono::high_resolution_clock::now();
		}

		if (continuousMode && stopContinuousModeKeyHit) {
			std::cout << "Exiting continuous mode." << std::endl;
			continuousMode = false;
			stopContinuousModeKeyHit = false;
		}

		char hitKey = ' ';
		if (_kbhit())
		{
			hitKey = toupper((char)_getch());
			mainDataLoop = (hitKey != 'X'); // stay in main data loop as long as we don't hit X
			getDataKeyHit = (hitKey == 'V');
			calibrateModelKeyHit = (hitKey == 'C');
			startContinuousModeKeyHit = (hitKey == 'N');
			stopContinuousModeKeyHit = (hitKey == 'M');
		}
			
	} while (mainDataLoop);

	std::cout << "Exiting main data loop!" << std::endl;

	// when exiting, close socket communication
	if (enableMirrorTherapy) {
		std::cout << "Socket server closing down, sending final packet..." << std::endl;
		// send a packet that exceeds the limits of the rotation values, indicating this cannot be a legitimate packet and shutting client down on the Java side
		double socketShutdownArray[6] = { 500, 500, 500, 500, 500, 500 };
		myLink.SendDoubles(socketShutdownArray, 6);
		// close socket conection
		myLink.Close();
	}

	// when exiting, save acquired data to file
	std::cout << "Reporting IK to file..." << std::endl;
	IKTool.reportToFile();

	// close the connection to IMUs
	xsensDataReader.CloseConnection();

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
