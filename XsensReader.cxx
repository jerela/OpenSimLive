// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include <OpenSimLiveConfig.h>
#include <OpenSim.h>

#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <set>
#include <list>
#include <utility>
#include <vector>

#include <xsensdeviceapi.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <xsens/xsmutex.h>

#include <XMLFunctions.h>


const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

/*! \brief Stream insertion operator overload for XsPortInfo */
std::ostream& operator << (std::ostream& out, XsPortInfo const& p)
{
	out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
		<< std::setw(7) << p.baudrate() << " Bd"
		<< ", " << "ID: " << p.deviceId().toString().toStdString()
		;
	return out;
}

/*! \brief Stream insertion operator overload for XsDevice */
std::ostream& operator << (std::ostream& out, XsDevice const& d)
{
	out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
	return out;
}

/*! \brief Given a list of update rates and a desired update rate, returns the closest update rate to the desired one */
int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate)
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

//----------------------------------------------------------------------
// Callback handler for wireless master
//----------------------------------------------------------------------
class WirelessMasterCallback : public XsCallback
{
public:
	typedef std::set<XsDevice*> XsDeviceSet;

	XsDeviceSet getWirelessMTWs() const
	{
		XsMutexLocker lock(m_mutex);
		return m_connectedMTWs;
	}

protected:
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{
		XsMutexLocker lock(m_mutex);
		switch (newState)
		{
		case XCS_Disconnected:		/*!< Device has disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Disconnected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Rejected:			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
			std::cout << "\nEVENT: MTW Rejected -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_PluggedIn:			/*!< Device is connected through a cable. */
			std::cout << "\nEVENT: MTW PluggedIn -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Wireless:			/*!< Device is connected wirelessly. */
			std::cout << "\nEVENT: MTW Connected -> " << *dev << std::endl;
			m_connectedMTWs.insert(dev);
			break;
		case XCS_File:				/*!< Device is reading from a file. */
			std::cout << "\nEVENT: MTW File -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		case XCS_Unknown:			/*!< Device is in an unknown state. */
			std::cout << "\nEVENT: MTW Unknown -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		default:
			std::cout << "\nEVENT: MTW Error -> " << *dev << std::endl;
			m_connectedMTWs.erase(dev);
			break;
		}
	}
private:
	mutable XsMutex m_mutex;
	XsDeviceSet m_connectedMTWs;
};

//----------------------------------------------------------------------
// Callback handler for MTw
// Handles onDataAvailable callbacks for MTW devices
//----------------------------------------------------------------------
class MtwCallback : public XsCallback
{
public:
	MtwCallback(int mtwIndex, XsDevice* device)
		:m_mtwIndex(mtwIndex)
		, m_device(device)
	{}

	bool dataAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return !m_packetBuffer.empty();
	}

	XsDataPacket const* getOldestPacket() const
	{
		XsMutexLocker lock(m_mutex);
		XsDataPacket const* packet = &m_packetBuffer.front();
		return packet;
	}

	void deleteOldestPacket()
	{
		XsMutexLocker lock(m_mutex);
		m_packetBuffer.pop_front();
	}

	int getMtwIndex() const
	{
		return m_mtwIndex;
	}

	XsDevice const& device() const
	{
		assert(m_device != 0);
		return *m_device;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		// NOTE: Processing of packets should not be done in this thread.

		m_packetBuffer.push_back(*packet);
		if (m_packetBuffer.size() > 300)
		{
			std::cout << std::endl;
			deleteOldestPacket();
		}
	}

private:
	mutable XsMutex m_mutex;
	std::list<XsDataPacket> m_packetBuffer;
	int m_mtwIndex;
	XsDevice* m_device;
};

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
		sensorNameInModel = sensorIdToLabel(currentSensorId, OPENSIMLIVE_ROOT+"/Config/"+mainConfigReader("mappings_file"));
		
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

	const int desiredUpdateRate = 75; // use 75 Hz update rate for MTWs
	const int desiredRadioChannel = 19; // use radio channel 19 for wireless master

	// Callback for wireless master
	WirelessMasterCallback wirelessMasterCallback;
	// Callbacks for MTw devices
	std::vector<MtwCallback*> mtwCallbacks;

	/*
	STARTUP PHASE
	*/

	// 1. INITIALIZE XDA: create XsControl
	XsControl* control = XsControl::construct(); assert(control != 0);
	if (control == 0)
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
		if (!control->openPort(wirelessMasterPort->portName().toStdString(), wirelessMasterPort->baudrate()))
		{
			std::ostringstream error;
			error << "Failed to open port " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
		XsDevicePtr wirelessMasterDevice = control->device(wirelessMasterPort->deviceId());
		if (wirelessMasterDevice == 0)
		{
			std::ostringstream error;
			error << "Failed to construct XsDevice instance: " << *wirelessMasterPort;
			throw std::runtime_error(error.str());
		}

		std::cout << "XsDevice instance created @ " << *wirelessMasterDevice << std::endl;

		// 4. CONFIGURE DEVICES
		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to go to config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		// 5. PREPARE FOR DATA HANDLING (EVENT BASED)
		std::cout << "Attaching callback handler..." << std::endl;
		wirelessMasterDevice->addCallbackHandler(&wirelessMasterCallback);

		std::cout << "Getting the list of the supported update rates..." << std::endl;
		const XsIntArray supportedUpdateRates = wirelessMasterDevice->supportedUpdateRates();

		std::cout << "Supported update rates:";
		for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end(); ++itUpRate)
		{
			std::cout << *itUpRate << " ";
		}
		std::cout << std::endl;

		const int newUpdateRate = findClosestUpdateRate(supportedUpdateRates, desiredUpdateRate);

		// 6. CONFIGURE AWINDA MASTER: set the desired update rate using XsDevice::setUpdateRate
		std::cout << "Setting update rate to " << newUpdateRate << " Hz..." << std::endl;
		if (!wirelessMasterDevice->setUpdateRate(newUpdateRate))
		{
			std::ostringstream error;
			error << "Failed to set update rate: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio channel if previously enabled... " << std::endl;
		if (wirelessMasterDevice->isRadioEnabled())
		{
			if (!wirelessMasterDevice->disableRadio())
			{
				std::ostringstream error;
				error << "Failed to disable radio channel: " << *wirelessMasterDevice;
				throw std::runtime_error(error.str());
			}
		}

		// 7. ENABLE THE RADIO: set the radio channel and enable the radio
		std::cout << "Setting radio channel to " << desiredRadioChannel << " and enabling radio..." << std::endl;
		if (!wirelessMasterDevice->enableRadio(desiredRadioChannel))
		{
			std::ostringstream error;
			error << "Failed to set radio channel: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Waiting for MTw to wirelessly connect...\n" << std::endl;

		bool waitForConnections = true;
		size_t connectedMTWCount = wirelessMasterCallback.getWirelessMTWs().size();
		do
		{
			XsTime::msleep(100);

			while (true)
			{
				size_t nextCount = wirelessMasterCallback.getWirelessMTWs().size();
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
		if (!wirelessMasterDevice->gotoMeasurement())
		{
			std::ostringstream error;
			error << "Failed to go to measurement mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Getting XsDevice instances for all MTws..." << std::endl;
		XsDeviceIdArray allDeviceIds = control->deviceIds();
		XsDeviceIdArray mtwDeviceIds;
		for (XsDeviceIdArray::const_iterator i = allDeviceIds.begin(); i != allDeviceIds.end(); ++i)
		{
			if (i->isMtw())
			{
				mtwDeviceIds.push_back(*i);
			}
		}
		XsDevicePtrArray mtwDevices;
		for (XsDeviceIdArray::const_iterator i = mtwDeviceIds.begin(); i != mtwDeviceIds.end(); ++i)
		{
			XsDevicePtr mtwDevice = control->device(*i);
			if (mtwDevice != 0)
			{
				mtwDevices.push_back(mtwDevice);
			}
			else
			{
				throw std::runtime_error("Failed to create an MTw XsDevice instance.");
			}
		}

		std::cout << "Attaching callback handlers to MTWs..." << std::endl;
		mtwCallbacks.resize(mtwDevices.size());
		for (int i = 0; i < (int)mtwDevices.size(); ++i)
		{
			mtwCallbacks[i] = new MtwCallback(i, mtwDevices[i]);
			mtwDevices[i]->addCallbackHandler(mtwCallbacks[i]);
		}



		
		



		/*
		PROCESSING PHASE
		*/

		// 1. HANDLE INCOMING DATA

		std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Room to store euler data for each mtw
		std::vector<XsQuaternion> quaternionData(mtwCallbacks.size()); // for data in quaternion form
		
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
		std::vector<std::vector<double>> jointAngles; // vector that will hold the joint angles

		auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
		auto clockNow = std::chrono::high_resolution_clock::now(); // this value will be updated in the loop
		auto clockPrev = clockStart; // this value will be updated in the loop to present the time point of the previous IK calculation
		std::chrono::duration<double> clockDuration; std::chrono::duration<double> prevDuration;

		OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK

		// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
		SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
		
		std::cout << "Entering data streaming and IK loop. Press C to calibrate model, V to calculate IK, N to enter continuous mode, M to exit continuous mode and X to quit." << std::endl;

		do
		{
			
			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;
					XsDataPacket const* packet = mtwCallbacks[i]->getOldestPacket();
					// only get Euler orientation data if we want to print it to console
					if (print_roll_pitch_yaw)
						eulerData[i] = packet->orientationEuler();
					quaternionData[i] = packet->orientationQuaternion();
					mtwCallbacks[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable && calibrateModelKeyHit) {
				// set clock to start from calibration
				clockStart = std::chrono::high_resolution_clock::now();
				// fill a timeseriestable with quaternion orientations of IMUs
				OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable(fillQuaternionTable(mtwCallbacks, quaternionData));
				// calibrate the model and return its file name
				calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT+"/Config/"+mainConfigReader("imu_placer_setup_file"), quaternionTimeSeriesTable);
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
			}

			if (newDataAvailable && getDataKeyHit && !calibratedModelFile.empty())
			{
				// use high resolution clock to count time since the IMU measurement began
				clockNow = std::chrono::high_resolution_clock::now();
				clockDuration = clockNow - clockStart; // time since calibration
				// fill a time series table with quaternion orientations of the IMUs
				OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(fillQuaternionTable(mtwCallbacks, quaternionData));
				// give the necessary inputs to IKTool
				IKTool.setQuaternion(quatTable); // update the IMU orientations
				IKTool.setTime(clockDuration.count()); // time since calibration given as the time of the state to perform IK on
				// calculate the IK and update the visualization
				IKTool.update(true);
				// push joint angles to vector
				//jointAngles.push_back(IKTool.getQ());
				
				// print the roll, pitch and yaw angles for all IMUs
				if (print_roll_pitch_yaw)
					printRollPitchYaw(mtwCallbacks, eulerData);

				std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
				std::cout << "Positions: " << "[" << trackerResults[0] << ", " << trackerResults[1] << ", " << trackerResults[2] << "]" << std::endl;
				std::cout << "Rotations: " << "[" << trackerResults[3] << ", " << trackerResults[4] << ", " << trackerResults[5] << "]" << std::endl;

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
				if (prevDuration.count()*1000 > continuousModeMsDelay) {
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
						printRollPitchYaw(mtwCallbacks, eulerData);
					std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
					std::cout << "Positions: " << "[" << trackerResults[0] << ", " << trackerResults[1] << ", " << trackerResults[2] << "]" << std::endl;
					std::cout << "Rotations: " << "[" << trackerResults[3] << ", " << trackerResults[4] << ", " << trackerResults[5] << "]" << std::endl;
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

		// when exiting, save acquired data to file
		IKTool.reportToFile();

		std::cout << "Setting config mode..." << std::endl;
		if (!wirelessMasterDevice->gotoConfig())
		{
			std::ostringstream error;
			error << "Failed to goto config mode: " << *wirelessMasterDevice;
			throw std::runtime_error(error.str());
		}

		std::cout << "Disabling radio... " << std::endl;
		if (!wirelessMasterDevice->disableRadio())
		{
			std::ostringstream error;
			error << "Failed to disable radio: " << *wirelessMasterDevice;
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

	/*
	EXIT PHASE
	*/

	// 1. CLOSE OPEN DEVICES

	std::cout << "Closing XsControl..." << std::endl;
	control->close();

	std::cout << "Deleting mtw callbacks..." << std::endl;
	for (std::vector<MtwCallback*>::iterator i = mtwCallbacks.begin(); i != mtwCallbacks.end(); ++i)
	{
		delete (*i);
	}

	std::cout << "Successful exit." << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

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
