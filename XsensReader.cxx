// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <Server.h>
#include <XsensDataReader.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <ThreadPoolContainer.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

void printRollPitchYaw(std::vector<MtwCallback*> mtwCallbacks, const std::vector<XsEuler>& eulerData) {
	for (size_t i = 0; i < mtwCallbacks.size(); ++i)
	{
		std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
			<< ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
			<< ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
			<< ", Yaw: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
			<< std::endl;
	}
}

// IK for multithreading
void concurrentIK(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, std::chrono::duration<double>& clockDuration, bool enableMirrorTherapy, bool sendMode, Server& myLink) {
	IKTool.setTime(clockDuration.count());
	// calculate the IK and update the visualization
	IKTool.update(true);
	if (enableMirrorTherapy)
	{
		// get the data we want to send to Java program
		std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
		// get a double array from the double vector
		double* mirrorTherapyPacket = &trackerResults[0];
		// send the data
		if (sendMode)
			myLink.SendDoubles(mirrorTherapyPacket, 6);
	}
}

// Runs IK and related shenanigans
void RunIKProcedure(OpenSimLive::XsensDataReader& xsensDataReader, std::vector<XsQuaternion>& quaternionData, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, std::chrono::duration<double>& clockDuration, const bool print_roll_pitch_yaw, const bool enableMirrorTherapy, bool sendMode, const std::vector<XsEuler>& eulerData, Server& myLink, OpenSimLive::ThreadPoolContainer& threadPoolContainer) {
	// fill a time series table with quaternion orientations of the IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
	// give the necessary inputs to IKTool
	IKTool.setQuaternion(quatTable);

	threadPoolContainer.offerFuture(concurrentIK, std::ref(IKTool), std::ref(clockDuration), enableMirrorTherapy, sendMode, std::ref(myLink));

	// push joint angles to vector
	//jointAngles.push_back(IKTool.getQ());
	if (print_roll_pitch_yaw)
		printRollPitchYaw(xsensDataReader.GetMtwCallbacks(), eulerData);

	//std::cout << "Positions: " << "[" << trackerResults[0] << ", " << trackerResults[1] << ", " << trackerResults[2] << "]" << std::endl;
	//std::cout << "Rotations: " << "[" << trackerResults[3] << ", " << trackerResults[4] << ", " << trackerResults[5] << "]" << std::endl;
}







void ConnectToDataStream() {

	// create Xsens connection object and connect the program to IMUs
	OpenSimLive::XsensDataReader xsensDataReader;
	int desiredUpdateRate = stoi(ConfigReader("MainConfiguration.xml", "desired_update_rate"));
	xsensDataReader.SetDesiredUpdateRate(desiredUpdateRate);
	xsensDataReader.InitiateStartupPhase();

	std::vector<XsEuler> eulerData(xsensDataReader.GetMtwCallbacks().size()); // Room to store euler data for each mtw
	std::vector<XsQuaternion> quaternionData(xsensDataReader.GetMtwCallbacks().size()); // for data in quaternion form
		
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool mainDataLoop = true; // IMU data is being measured and analyzed while this is true
	bool continuousMode = false; // IK is calculated continuously while this is true
	bool sendMode = false; // data is sent to client with each IK step while this is true
	bool getDataKeyHit = false; // tells if the key that initiates a single IK calculation is hit
	bool referenceBaseRotationKeyHit = false; // tells if the key that initiates the fetching of the current rotation of the base IMU is hit
	bool calibrateModelKeyHit = false; // tells if the key that initiates model calibration is hit
	bool startContinuousModeKeyHit = false; // tells if the key that starts continuous mode is hit
	bool stopContinuousModeKeyHit = false; // tells if the key that pauses continuous mode is hit
	bool startSendModeKeyHit = false; // tells if the key that starts send mode is hit
	bool stopSendModeKeyHit = false; // tells if the key that pauses send mode is hit
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	int continuousModeMsDelay = std::stoi(ConfigReader("MainConfiguration.xml", "continuous_mode_ms_delay")); // delay between consequent IK calculations in consequent mode, in milliseconds
	bool print_roll_pitch_yaw = ("true" == ConfigReader("MainConfiguration.xml", "print_roll_pitch_yaw")); // boolean that tells whether to print roll, pitch and yaw of IMUs while calculating IK
	bool resetClockOnContinuousMode = ("true" == ConfigReader("MainConfiguration.xml", "reset_clock_on_continuous_mode")); // if true, clock will be reset to zero when entering continuous mode; if false, the clock will be set to zero at calibration
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false
	unsigned int maxThreads = stoi(ConfigReader("MainConfiguration.xml", "threads")); // get the maximum number of concurrent threads for multithreading
	std::string stationReferenceBody = ConfigReader("MainConfiguration.xml", "station_reference_body"); // get the name of the reference body used in mirror therapy

	// initialize the object that handles multithreading
	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);

	//std::vector<std::vector<double>> jointAngles; // vector that will hold the joint angles

	auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	auto clockNow = std::chrono::high_resolution_clock::now(); // this value will be updated in the loop
	auto clockPrev = clockStart; // this value will be updated in the loop to present the time point of the previous IK calculation
	std::chrono::duration<double> clockDuration; std::chrono::duration<double> prevDuration;

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
		
	// SOCKET COMMUNICATION
	bool bResult = false;
	int port = std::stoi(ConfigReader("MainConfiguration.xml", "socket_port"));
	int dataport = -1; // datagram port, not in use
	Server myLink(port, dataport, &bResult);
	if (!bResult)
	{
		printf("Failed to create Server object!\n");
	}
	if (enableMirrorTherapy) {	
		std::cout << "Waiting for client program to connect..." << std::endl;
		myLink.Connect(); // create socket connection
		std::cout << "Client program connected." << std::endl;
	}
		
		

	std::cout << "Entering data streaming and IK loop. Press C to calibrate model, Z to calculate IK once, N to enter continuous mode, M to exit continuous mode, V to enter send mode, B to exit send mode, , to save base reference orientation and X to quit." << std::endl;

	do
	{
		// get IMU orientation data in quaternions
		quaternionData = xsensDataReader.GetQuaternionData(quaternionData);
		if (print_roll_pitch_yaw)
			eulerData = xsensDataReader.GetEulerData(eulerData);
		// update the boolean value to see if new data is available since orientation data was last retrieved
		bool newDataAvailable = xsensDataReader.GetNewDataAvailable();

		// if user hits the key to save the current orientation of the station reference body IMU when it is placed against the mounting surface of the robot arm
		if (referenceBaseRotationKeyHit)
		{
			if (stationReferenceBody == "none")
			{
				std::cout << "Reference base rotation cannot be calculated because station reference body has not been defined in XML configuration!" << std::endl;
				break;
			}
			OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
			for (unsigned int i = 0; i < quaternionTimeSeriesTable.getNumColumns(); ++i)
			{
				if (stationReferenceBody + "_imu" == quaternionTimeSeriesTable.getColumnLabel(i))
				{
					SimTK::Quaternion_<SimTK::Real> quat = quaternionTimeSeriesTable.getMatrixBlock(0, i, 1, 1)[0][0];
					std::cout << "Captured reference body IMU orientation: " << quat << std::endl;
					// NEXT: pass it on to PointTracker, or IMUIKTool that inherits PointTracker, and use it at the end of PointTracker rotation calculations
					IKTool.setReferenceBaseRotation(quat);
					/*
					1. base IMU asetetaan robotin tasoa vasten siten päin, miten se olisi lantiolla, jos koehenkilö painaisi selkänsä
					tasoa vasten
					2. painetaan näppäimistöltä referensointinappulaa, jolloin XsensDataReaderilta otetaan talteen senhetkinen
					orientaatio kvaternioina tai rotaatiomatriiseina
					3. myöhemmin tätä orientaatiota verrataan kunkin ajanhetkiseen orientaatioon, josta lasketaan matriisi, jolla
					PointTrackerin rotaatiota kerrotaan jotta saadaan rotaatio robotilla
					*/
				}
			}
			referenceBaseRotationKeyHit = false;
		}

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
			// set private variables to be accessed in IK calculations
			if (enableMirrorTherapy == true) {
				IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
				IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
			}
			else {
				IKTool.setPointTrackerEnabled(false);
			}
			IKTool.run(true); // true for visualization
			std::cout << "Model has been calibrated." << std::endl;

			
		}

		// if user hits the single IK calculation key, new data is available and the model has been calibrated
		if (newDataAvailable && getDataKeyHit && !calibratedModelFile.empty())
		{
			OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
			for (unsigned int i = 0; i < quaternionTimeSeriesTable.getNumColumns(); ++i)
			{
				if (stationReferenceBody + "_imu" == quaternionTimeSeriesTable.getColumnLabel(i))
				{
					SimTK::Quaternion_<SimTK::Real> quat = quaternionTimeSeriesTable.getMatrixBlock(0, i, 1, 1)[0][0];
					// NEXT: pass it on to PointTracker, or IMUIKTool that inherits PointTracker, and use it at the end of PointTracker rotation calculations
					IKTool.setReferenceBodyRotation(quat);
				}
			}
			// use high resolution clock to count time since the IMU measurement began
			clockNow = std::chrono::high_resolution_clock::now();
			clockDuration = clockNow - clockStart; // time since calibration
			RunIKProcedure(xsensDataReader, quaternionData, IKTool, clockDuration, print_roll_pitch_yaw, enableMirrorTherapy, sendMode, eulerData, myLink, threadPoolContainer);
			getDataKeyHit = false;
		}

		// if new data is available and continuous mode has been switched on
		if (newDataAvailable && continuousMode) {
			OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(xsensDataReader.GetMtwCallbacks(), quaternionData));
			for (unsigned int i = 0; i < quaternionTimeSeriesTable.getNumColumns(); ++i)
			{
				if (stationReferenceBody + "_imu" == quaternionTimeSeriesTable.getColumnLabel(i))
				{
					SimTK::Quaternion_<SimTK::Real> quat = quaternionTimeSeriesTable.getMatrixBlock(0, i, 1, 1)[0][0];
					// NEXT: pass it on to PointTracker, or IMUIKTool that inherits PointTracker, and use it at the end of PointTracker rotation calculations
					IKTool.setReferenceBodyRotation(quat);
				}
			}
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

				RunIKProcedure(xsensDataReader, quaternionData, IKTool, clockDuration, print_roll_pitch_yaw, enableMirrorTherapy, sendMode, eulerData, myLink, threadPoolContainer);
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

		if (!sendMode && startSendModeKeyHit) {
			std::cout << "Starting send mode." << std::endl;
			sendMode = true;
			startSendModeKeyHit = false;
		}

		if (sendMode && stopSendModeKeyHit) {
			std::cout << "Exiting send mode." << std::endl;
			sendMode = false;
			stopSendModeKeyHit = false;
		}

		char hitKey = ' ';
		if (_kbhit())
		{
			hitKey = toupper((char)_getch());
			mainDataLoop = (hitKey != 'X'); // stay in main data loop as long as we don't hit X
			getDataKeyHit = (hitKey == 'Z');
			calibrateModelKeyHit = (hitKey == 'C');
			startContinuousModeKeyHit = (hitKey == 'N');
			stopContinuousModeKeyHit = (hitKey == 'M');
			startSendModeKeyHit = (hitKey == 'V');
			stopSendModeKeyHit = (hitKey == 'B');
			referenceBaseRotationKeyHit = (hitKey == ',');
		}
			
	} while (mainDataLoop);

	std::cout << "Exiting main data loop!" << std::endl;

	// when exiting, close socket communication
	if (enableMirrorTherapy) {
		std::cout << "Socket server closing down, sending final packet to disable client..." << std::endl;
		// send a packet that exceeds the limits of the rotation values, indicating this cannot be a legitimate packet and shutting client down on the Java side
		double socketShutdownArray[6] = { 500, 500, 500, 500, 500, 500 };
		myLink.SendDoubles(socketShutdownArray, 6);
		// close socket conection
		myLink.Close();
	}

	// when exiting, save acquired data to file
	if (IKTool.get_report_errors())
	{
		std::cout << "Reporting IK to file..." << std::endl;
		IKTool.reportToFile();
	}

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
