#include <XsensDataReader.h>


// \brief Stream insertion operator overload for XsPortInfo
std::ostream& operator << (std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
		;
	return out;
}

// \brief Stream insertion operator overload for XsDevice
std::ostream& operator << (std::ostream& out, XsDevice const& d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}


using namespace OpenSimLive;

XsensDataReader::XsensDataReader() {}

XsensDataReader::~XsensDataReader() {}



int XsensDataReader::findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
{
	if (supportedUpdateRates.empty())
	{
		return 0;
	}

	if (supportedUpdateRates.size() == 1)
	{
		return supportedUpdateRates[0];
	}

	int uRateDist = -1;
	int closestUpdateRate = -1;
	for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
	{
		const int currDist = std::abs(*itUpRate - desiredUpdateRate);

		if ((uRateDist == -1) || (currDist < uRateDist))
		{
			uRateDist = currDist;
			closestUpdateRate = *itUpRate;
		}
	}
	return closestUpdateRate;
}

void XsensDataReader::InitiateStartupPhase() {

	//const int desiredUpdateRate = 75; // use 75 Hz update rate for MTWs
	//const int desiredRadioChannel = 19; // use radio channel 19 for wireless master

	// Callback for wireless master
	//WirelessMasterCallback wirelessMasterCallback;
	// Callbacks for MTw devices
	//std::vector<MtwCallback*> mtwCallbacks;

	/*
	STARTUP PHASE
	*/

	// 1. INITIALIZE XDA: create XsControl
	//XsControl* control = XsControl::construct(); assert(control != 0);
	control_ = XsControl::construct(); assert(control_ != 0);
	if (control_ == 0)
	{
		std::cout << "Failed to construct XsControl instance." << std::endl;
	}

	try
	{
		// 2. DISCOVER DEVICES: detect available devices using XsScanner::scanPorts function
		std::cout << "Scanning ports..." << std::endl;
		XsPortInfoArray detectedDevices = XsScanner::scanPorts();

		std::cout << "Finding wireless master..." << std::endl;
		XsPortInfoArray::const_iterator wirelessMasterPort = detectedDevices.begin();
		while (wirelessMasterPort != detectedDevices.end() && !wirelessMasterPort->deviceId().isWirelessMaster())
		{
			++wirelessMasterPort;
		}
		if (wirelessMasterPort == detectedDevices.end())
		{
			throw std::runtime_error("No wireless masters found!");
		}
		std::cout << "Wireless master found @ " << *wirelessMasterPort << std::endl;

		// 3. OPEN DEVICES OF INTEREST: open devices with XsControl::openPort()
		std::cout << "Opening port..." << std::endl;
		if (!control_->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		//XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		wirelessMasterDevice_ = control_->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice_ == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice_ << std::endl;

		// 4. CONFIGURE DEVICES
		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice_->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to go to config mode: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}

		// 5. PREPARE FOR DATA HANDLING (EVENT BASED)
		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice_->addCallbackHandler(&wirelessMasterCallback_);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice_->supportedUpdateRates();

		std::cout << "Supported update rates:";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate_);

		// 6. CONFIGURE AWINDA MASTER: set the desired update rate using XsDevice::setUpdateRate
		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice_->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled... " << std::endl;
		if (wirelessMasterDevice_->isRadioEnabled())
		{
			if (!wirelessMasterDevice_->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice_;
				throw std::runtime_error(error.str());
			}
		}

		// 7. ENABLE THE RADIO: set the radio channel and enable the radio
		std::cout << "Setting radio channel to " << desiredRadioChannel_ << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice_->enableRadio(desiredRadioChannel_))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTw to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback_.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback_.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTws: " << nextCount << ". Press 'Y' to start measurement." << std::endl;
					connectedMTWCount = nextCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				waitForConnections = (toupper((char)_getch()) != 'Y');
			}
		} while (waitForConnections);


		// 8. SWITCH DEVICES TO MEASUREMENT MODE: set devices to measurement mode with XsDevice::gotoMeasurement
		std::cout << "Starting measurement..." << std::endl;
		if (!wirelessMasterDevice_->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to go to measurement mode: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTws..." << std::endl;
		XsDeviceIdArray allDeviceIds = control_->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		//XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control_->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices_.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTw XsDevice instance.");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks_.resize(mtwDevices_.size());
		for (int i = 0; i < (int)mtwDevices_.size(); ++i)
		{
			mtwCallbacks_[i] = new MtwCallback(i, mtwDevices_[i]);
			mtwDevices_[i]->addCallbackHandler(mtwCallbacks_[i]);
		}

	}
	catch (std::exception const& ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
}
		

std::vector<XsQuaternion> XsensDataReader::GetQuaternionData() {
	try
	{
		newDataAvailable_ = false;
		std::vector<XsQuaternion> quaternionData(mtwCallbacks_.size());
		for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
		{
			if (mtwCallbacks_[i]->dataAvailable())
			{
				newDataAvailable_ = true;
				XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();
				quaternionData[i] = packet->orientationQuaternion();
				mtwCallbacks_[i]->deleteOldestPacket();
			}
		}
		return quaternionData;
	}
	catch (std::exception const& ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
}


/*void XsensDataReader::InitiateProcessingPhase() {
	
	//PROCESSING PHASE
	

	try
	{
		// 1. HANDLE INCOMING DATA

		std::vector<XsEuler> eulerData(mtwCallbacks_.size()); // Room to store euler data for each mtw
		std::vector<XsQuaternion> quaternionData(mtwCallbacks_.size()); // for data in quaternion form

		std::string calibratedModelFile;

		bool mainDataLoop = true; // IMU data is being measured while this is true
		bool continuousMode = false; // IK is calculated continuously while this is true
		bool getDataKeyHit = false; // tells if the key that initiates a single IK calculation is hit
		bool calibrateModelKeyHit = false; // tells if the key that initiates model calibration is hit
		bool startContinuousModeKeyHit = false; // tells if the key that starts continuous mode is hit
		bool stopContinuousModeKeyHit = false; // tells if the key that ends continuous mode is hit
		int continuousModeMsDelay = std::stoi(mainConfigReader("continuous_mode_ms_delay")); // delay between consequent IK calculations in consequent mode, in milliseconds
		bool print_roll_pitch_yaw = ("true" == mainConfigReader("print_roll_pitch_yaw")); // boolean that tells whether to print roll, pitch and yaw of IMUs while calculating IK
		bool resetClockOnContinuousMode = ("true" == mainConfigReader("reset_clock_on_continuous_mode")); // if true, clock will be reset to zero when entering continuous mode; if false, the clock will be set to zero at calibration
		bool enableMirrorTherapy = (mainConfigReader("station_parent_body") != "none"); // if "none", then set to false
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
		int port = std::stoi(mainConfigReader("socket_port"));
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

			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
			{
				if (mtwCallbacks_[i]->dataAvailable())
				{
					newDataAvailable = true;
					XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();
					// only get Euler orientation data if we want to print it to console
					if (print_roll_pitch_yaw)
						eulerData[i] = packet->orientationEuler();
					quaternionData[i] = packet->orientationQuaternion();
					mtwCallbacks_[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable && calibrateModelKeyHit) {
				// set clock to start from calibration
				clockStart = std::chrono::high_resolution_clock::now();
				// fill a timeseriestable with quaternion orientations of IMUs
				OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(mtwCallbacks_, quaternionData));
				// calibrate the model and return its file name
				calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + mainConfigReader("imu_placer_setup_file"), quaternionTimeSeriesTable);
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
					IKTool.setPointTrackerBodyName(mainConfigReader("station_parent_body"));
					IKTool.setPointTrackerReferenceBodyName(mainConfigReader("station_reference_body"));
				}
				else {
					IKTool.setPointTrackerEnabled(false);
				}
			}

			if (newDataAvailable && getDataKeyHit && !calibratedModelFile.empty())
			{
				// use high resolution clock to count time since the IMU measurement began
				clockNow = std::chrono::high_resolution_clock::now();
				clockDuration = clockNow - clockStart; // time since calibration
				// fill a time series table with quaternion orientations of the IMUs
				OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(mtwCallbacks_, quaternionData));
				// give the necessary inputs to IKTool
				IKTool.setQuaternion(quatTable); // update the IMU orientations
				IKTool.setTime(clockDuration.count()); // time since calibration given as the time of the state to perform IK on
				// calculate the IK and update the visualization
				IKTool.update(true);
				// push joint angles to vector
				//jointAngles.push_back(IKTool.getQ());

				// print the roll, pitch and yaw angles for all IMUs
				if (print_roll_pitch_yaw)
					printRollPitchYaw(mtwCallbacks_, eulerData);
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

			if (newDataAvailable && continuousMode) {
				// use high resolution clock to count time since the IMU measurement began
				clockNow = std::chrono::high_resolution_clock::now();
				// calculate the duration since the beginning of counting
				clockDuration = clockNow - clockStart;
				// calculate the duration since the previous time IK was continuously calculated
				prevDuration = clockNow - clockPrev;
				// if more than the set time delay has passed since the last time IK was calculated
				if (prevDuration.count() * 1000 > continuousModeMsDelay) {
					// set current time as the time IK was previously calculated for the following iterations of the while-loop
					clockPrev = clockNow;

					// fill a time series table with quaternion orientations of the IMUs
					OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(mtwCallbacks, quaternionData));
					// give the necessary inputs to IKTool
					IKTool.setQuaternion(quatTable);
					IKTool.setTime(clockDuration.count());
					// calculate the IK and update the visualization
					IKTool.update(true);
					// push joint angles to vector
					jointAngles.push_back(IKTool.getQ());
					if (print_roll_pitch_yaw)
						printRollPitchYaw(mtwCallbacks_, eulerData);

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
				if (resetClockOnContinuousMode && !(clockDuration.count() > 0)) // ensure that the config setting is set to true and that this is the first time continuous mode is entered
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

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice_->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice_->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice_;
			throw std::runtime_error(error.str());
		}
	}
	catch (std::exception const& ex)
	{
		std::cout << ex.what() << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		std::cout << "****ABORT****" << std::endl;
	}

	// finally done with try...catch

}*/

	

void XsensDataReader::CloseConnection(){

	std::cout << "Setting config mode..." << std::endl;
	if (!wirelessMasterDevice_->gotoConfig())
	{
		std::ostringstream error;
		error << "Failed to goto config mode: " << *wirelessMasterDevice_;
		throw std::runtime_error(error.str());
	}

	std::cout << "Disabling radio... " << std::endl;
	if (!wirelessMasterDevice_->disableRadio())
	{
		std::ostringstream error;
		error << "Failed to disable radio: " << *wirelessMasterDevice_;
		throw std::runtime_error(error.str());
	}


	/*
	EXIT PHASE
	*/

	// 1. CLOSE OPEN DEVICES

	std::cout << "Closing XsControl..." << std::endl;
	control_->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks_.begin(); i != mtwCallbacks_.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

	//return;
}

