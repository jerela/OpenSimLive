#include <XsensDataReader.h>
#include <fstream>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// \brief Stream insertion operator overload for XsPortInfo
std::ostream& operator << (std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString();
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

XsensDataReader::~XsensDataReader() {
	if (saveQuaternionsToFile_) {
		saveQuaternionsToFile(OPENSIMLIVE_ROOT, "OpenSimLive-results");
	}
}



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

unsigned int XsensDataReader::InitiateStartupPhase() {

	//const int desiredUpdateRate = 75; // use 75 Hz update rate for MTWs
	//const int desiredRadioChannel = 19; // use radio channel 19 for wireless master

	// Callback for wireless master
	//WirelessMasterCallback wirelessMasterCallback;
	// Callbacks for MTw devices
	//std::vector<MtwCallback*> mtwCallbacks;

	// returnMode: 1 for quitting the program, 2 for continuing
	unsigned int returnMode = 0;

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
			//throw std::runtime_error("No wireless masters found!");
			std::cout << "Failed to find wireless masters, trying again..." << std::endl;
			return 0;
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

		//bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback_.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback_.getWirelessMTWs().size();
				if (nextCount != connectedMTWCount)
				{
					std::cout << "Number of connected MTws: " << nextCount << ". Press 'Y' to start measurement or 'N' to disconnect connected MTws." << std::endl;
					connectedMTWCount = nextCount;
					// save the number of used sensors in a private variable
					nSensors_ = connectedMTWCount;
				}
				else
				{
					break;
				}
			}
			if (_kbhit())
			{
				char hitKey = toupper((char)_getch());
				if (hitKey == 'Y') {
					returnMode = 2;
				}
				else if (hitKey == 'N') {
					returnMode = 1;
				}
				//waitForConnections = (toupper((char)_getch()) != 'Y');
			}
		} while (!returnMode);


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
	return returnMode;
}
		

std::vector<XsQuaternion> XsensDataReader::GetQuaternionData(std::vector<XsQuaternion>& quaternionData) {
	newDataAvailable_ = false;
	// timeGot is used to ensure that we push only one time value to a vector per data retrieval if we are saving quaternions to file
	bool timeGot = false;
	//std::vector<XsQuaternion> quaternionData(mtwCallbacks_.size());
	for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
	{
		if (mtwCallbacks_[i]->dataAvailable())
		{
			newDataAvailable_ = true;
			XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();
			quaternionData[i] = packet->orientationQuaternion();
			// if saving quaternions to file, get the time of each data retrieval into a vector
			if (saveQuaternionsToFile_ && !timeGot) {
				if (initialTime_ == 0) {
					initialTime_ = packet->timeOfArrival().secTime();
				}
				timeVector_.push_back(packet->timeOfArrival().secTime()-initialTime_);
				timeGot = true;
			}
			mtwCallbacks_[i]->deleteOldestPacket();
		}
	}

	// if we got time and are saving quaternions to file, push the quaternions into a vector
	if (saveQuaternionsToFile_ && timeGot) {
		quaternionData_.push_back(quaternionData);
	}

	return quaternionData;
}


std::vector<XsQuaternion> XsensDataReader::getQuaternionData() {
	//newDataAvailable_ = false;
	size_t mtwCallbackSize = mtwCallbacks_.size();
	std::vector<XsQuaternion> quaternionData(mtwCallbackSize);
	for (size_t i = 0; i < mtwCallbackSize; ++i)
	{
		// wait until new data is available
		do {} while (!mtwCallbacks_[i]->dataAvailable());
		
		//newDataAvailable_ = true;
		XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();
		quaternionData[i] = packet->orientationQuaternion();
		mtwCallbacks_[i]->deleteOldestPacket();
		
	}
	return quaternionData;
}




std::vector<XsEuler> XsensDataReader::GetEulerData(std::vector<XsEuler>& eulerData) {
	newDataAvailable_ = false;
	//std::vector<XsEuler> eulerData(mtwCallbacks_.size());
	for (size_t i = 0; i < mtwCallbacks_.size(); ++i)
	{
		if (mtwCallbacks_[i]->dataAvailable())
		{
			newDataAvailable_ = true;
			XsDataPacket const* packet = mtwCallbacks_[i]->getOldestPacket();
			eulerData[i] = packet->orientationEuler();
			mtwCallbacks_[i]->deleteOldestPacket();
		}
	}
	return eulerData;
}




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



// A function to convert XsQuaternion to string before writing it to file; it can be written directly to file, but the formatting in this function will make its formatting identical to SimTK::Quaternions in file
std::string XsensDataReader::convertXsQuaternionToString(XsQuaternion quaternion) {
	std::string str = "~[" + std::to_string(quaternion.w()) + "," + std::to_string(quaternion.x()) + "," + std::to_string(quaternion.y()) + "," + std::to_string(quaternion.z()) + "]";
	return str;
}



// This function saves the time points and the corresponding quaternions to file for later examination.
void XsensDataReader::saveQuaternionsToFile(const std::string& rootDir, const std::string& resultsDir) {

	if (timeVector_.size() > 100000 || quaternionData_.size() > 100000) {
		std::cout << "In a normal situation we would save quaternions to file now, but because there are " << timeVector_.size() << " data points, for the sake of hard drive space we won't do it." << std::endl;
		return;
	}
	else {
		std::cout << "Saving quaternion time series to file..." << std::endl;
	}

	// create the complete path of the file, including the file itself, as a string
	std::string filePath(rootDir + "/" + resultsDir + "/" + "QuaternionTimeSeriesXsens.txt");
	// create an output file stream that is used to write into the file
	std::ofstream outputFile;
	// open and set file to discard any contents that existed in the file previously (truncate mode)
	outputFile.open(filePath, std::ios_base::out | std::ios_base::trunc);
	// check that the file was successfully opened and write into it
	if (outputFile.is_open())
	{
		outputFile << "Time series of measured orientation data in quaternions:\n";
		outputFile << "Time (s)";

		for (unsigned int j = 0; j < nSensors_; ++j) {
			outputFile << "\t Quaternion" + std::to_string(j + 1);
		}

		for (unsigned int i = 0; i < quaternionData_.size(); ++i) { // iteration through rows
			// after the first 2 rows of text, start with a new line and put time values in the first column
			outputFile << "\n" << timeVector_[i];
			for (unsigned int j = 0; j < nSensors_; ++j) {
				// then input quaternion values, separating them from time and other quaternion values with a tab
				outputFile << "\t" << convertXsQuaternionToString(quaternionData_[i][j]);
			}
		}
		outputFile.close();
		std::cout << "Quaternion time series written to file " << filePath << std::endl;
	}
	else {
		std::cout << "Failed to open file " << filePath << std::endl;
	}
}


