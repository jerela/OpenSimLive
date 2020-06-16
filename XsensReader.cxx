// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include <OpenSimLiveConfig.h>
#include <OpenSim.h>

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



void InverseKinematicsFromIMUs(std::vector<XsMatrix> matrixData, std::vector<MtwCallback*> mtwCallbacks){

	/* ALTERNATIVE WAY TO HANDLE THIS?
	Use OpenSim:DataAdapter::OutputTables.createTablesFromMatrices to create an OutputTables object from matrix data given by XSens
	Use OpenSim::IMUDataReader.getOrientationsTable to construct a TimeSeriesTable_ with the orientations
	Construct an OrientationsReference object using the TimeSeriesTable_
	Use the constructed OrientationsReference object to solve IK using InverseKinematicsSolver
	
	Possibly use OpenSim::XsensDataReader instead of IMUDataReader?
	*/

	int numberOfSensors = mtwCallbacks.size();

	// create a vector for IMU names
	std::vector<std::string> sensorNameVector;
	// create a vector for time (in a single frame, so just a vector with a single value)
	std::vector<double> timeVector{ 0 };
	// create a matrix for IMU orientations
	SimTK::Matrix_<SimTK::Rotation_<double>> orientationDataMatrix(1, numberOfSensors); // marker positions go here

	// get the connection between IMU serials and bodies on the model from an XML file - necessary or not?
	OpenSim::XMLDocument SensorMappingsFile("C:/Users/wksadmin/source/repos/OpenSimLive/SensorMappings.xml");

	// declare variables to be used in the loop
	double m11, m12, m13, m21, m22, m23, m31, m32, m33;
	SimTK::Rotation_<double> tempMatrix;

	std::string currentSensorId; std::string sensorNameInModel;

	// insert orientation data from matrixData to orientationDataMatrix; note that orientationDataMatrix elements are 3x3 rotation matrices
	for (int k = 0; k < numberOfSensors; k++) {
		// give values to variables representing individual elements of a 3x3 rotation matrix
		m11 = matrixData.at(k).value(1, 1);
		m12 = matrixData.at(k).value(1, 2);
		m13 = matrixData.at(k).value(1, 3);
		m21 = matrixData.at(k).value(2, 1);
		m22 = matrixData.at(k).value(2, 2);
		m23 = matrixData.at(k).value(2, 3);
		m31 = matrixData.at(k).value(3, 1);
		m32 = matrixData.at(k).value(3, 2);
		m33 = matrixData.at(k).value(3, 3);
		// set these values to tempMatrix
		tempMatrix.set(1, 1, m11);
		tempMatrix.set(1, 1, m12);
		tempMatrix.set(1, 1, m13);
		tempMatrix.set(1, 1, m21);
		tempMatrix.set(1, 1, m22);
		tempMatrix.set(1, 1, m23);
		tempMatrix.set(1, 1, m31);
		tempMatrix.set(1, 1, m32);
		tempMatrix.set(1, 1, m33);

		// fill orientationDataMatrix with orientation matrices for each sensor
		orientationDataMatrix.set(0, k, tempMatrix);

		// populate sensorNameVector with names of the IMUs on the model
		// NOTE: if SensorMappings cannot be read to map IMU serials to IMU names in the model, use IMU serials directly as IMU names in the model?
		// could OpenSim::ExperimentalSensor be of use here?

		// initialize currentSensorId with the serial of the IMU
		currentSensorId = mtwCallbacks[k]->device().deviceId().toString().toStdString();
		
		// find the ExperimentalSensors element
		SimTK::Xml::Element elementXSensDataReaderSettings("XSensDataReaderSettings");
		SimTK::Xml::Element elementExperimentalSensors = SensorMappingsFile.findElementWithName(elementXSensDataReaderSettings, "ExperimentalSensors");
		// inform the user if nothing was found
		if (!elementExperimentalSensors.isValid()) {
			std::cout << "Failed to find the XML element." << std::endl;
			break;
		}
		// iterate through the immediate child elements until match to currentSensorId is found
		SimTK::Array_<SimTK::Xml::Element> elementArray = elementExperimentalSensors.getAllElements();
		for (unsigned int elit = 0; elit < elementArray.size(); elit++) {
			if (elementArray.at(elit).hasAttribute("name")) {
				if (elementArray.at(elit).getRequiredAttributeValue("name") == currentSensorId) {
					// get the value of element name_in_model into sensorNameInModel
					sensorNameInModel = elementArray.at(elit).getRequiredElement("name_in_model").getValue();
				}
			}
		}
		// push the name of the IMU in the model into sensorNameVector
		sensorNameVector.push_back(sensorNameInModel);
	}

	// form a timeseriestable of the IMU orientations
	OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> timeSeriesTable(timeVector, orientationDataMatrix, sensorNameVector);


	// load model
	OpenSim::Model model("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full_calibrated.osim");

	// create state
	SimTK::State s = model.initSystem();

	// create MarkersReference object
	OpenSim::MarkersReference markersReference;

	// create OrientationsReference object
	OpenSim::OrientationsReference orientationsReference(timeSeriesTable);

	// create coordinate reference
	SimTK::Array_<OpenSim::CoordinateReference> coordinateReference;

	// use inversekinematicssolver with markersreference to solve
	OpenSim::InverseKinematicsSolver iks(model, markersReference, orientationsReference, coordinateReference, SimTK::Infinity);

	iks.setAccuracy(1e-4);
	bool isAssembled = false;
	while (!isAssembled) {
		try {

			iks.assemble(s);
			isAssembled = true;
		}
		catch (...) {
			std::cerr << "Model not assembled" << std::endl;
			isAssembled = false;
		}
	}
	// calculate coordinates for state s
	iks.track(s);

	// get coordinates from state s
	SimTK::Vector stateQ(s.getQ());
	// get number of coordinates (joint angles) in the model
	int numCoordinates = model.getNumCoordinates();
	std::vector<double> q(numCoordinates);
	for (int j = 0; j < numCoordinates; j++) {
		q[j] = stateQ[j];
		std::cout << "Q" << j << ": " << q[j] << std::endl;
	}


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

		std::cout << "\nMain loop. Press any key to quit\n" << std::endl;
		std::cout << "Waiting for data available..." << std::endl;

		/*
		PROCESSING PHASE
		*/

		// 1. HANDLE INCOMING DATA

		std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Room to store euler data for each mtw
		std::vector<XsMatrix> matrixData(mtwCallbacks.size()); // for data in orientation matrix form that OpenSim requires
		unsigned int printCounter = 0;
		while (!_kbhit()) {
			XsTime::msleep(0);

			bool newDataAvailable = false;
			for (size_t i = 0; i < mtwCallbacks.size(); ++i)
			{
				if (mtwCallbacks[i]->dataAvailable())
				{
					newDataAvailable = true;
					XsDataPacket const* packet = mtwCallbacks[i]->getOldestPacket();
					eulerData[i] = packet->orientationEuler();
					// also get data as orientation matrices for OpenSim
					matrixData[i] = packet->orientationMatrix();
					mtwCallbacks[i]->deleteOldestPacket();
				}
			}

			if (newDataAvailable)
			{

				// OPENSIM BEGINS
				size_t numberOfSensors = mtwCallbacks.size();
				InverseKinematicsFromIMUs(matrixData, mtwCallbacks);
				// OPENSIM ENDS

				// Don't print too often for performance. Console output is very slow.
				if (printCounter % 25 == 0)
				{
					for (size_t i = 0; i < mtwCallbacks.size(); ++i)
					{
						std::cout << "[" << i << "]: ID: " << mtwCallbacks[i]->device().deviceId().toString().toStdString()
							<< ", Roll: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].roll()
							<< ", Pitch: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].pitch()
							<< ", Yaw: " << std::setw(7) << std::fixed << std::setprecision(2) << eulerData[i].yaw()
							<< "\n";
					}
				}
				++printCounter;
			}

		}
		(void)_getch();

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
	EXIT PHAE
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



	/*
	// create marker weight set
	OpenSim::Set<OpenSim::MarkerWeight> markerWeightSet;
	markerWeightSet.setSize(numberOfMarkers);

	// create a vector for marker names
	std::vector<std::string> markerNameVector;
	// create a vector for time (in a single frame, so just a vector with a single value)
	std::vector<double> timeVector{ 0 };
	// create a matrix for marker translations
	SimTK::Matrix_<SimTK::Vec3> markerDataMatrix(1,numberOfMarkers); // marker positions go here


	for (int n = 0; n < 3; n++) {

		markerDataMatrix.setToNaN();
		markerNameVector.clear();
		timeVector.clear();

		// get the current frame
		frame = client.GetFrame();
		// get the number of the current frame
		frameNumber = client.GetFrameNumber();
		std::cout << "Got frame number " << frameNumber.FrameNumber << std::endl;

		timeVector.push_back(frameNumber.FrameNumber);

		std::cout << "Inserting global marker translations into marker data matrix..." << std::endl;

		for (int k = 0; k < numberOfMarkers; k++) {
			// get the name of the marker at index k
			//std::string markerName = client.GetMarkerName(subjectName, k).MarkerName;
			markerNameVector.push_back(markerName); // push name of marker at index k to vector markerNameVector
			markerWeightSet.insert(k, OpenSim::MarkerWeight(markerName, 10)); // insert a MarkerWeight object with weight 10 to index k in markerWeightSet
			// get global translations of the marker
			//Output_GetMarkerGlobalTranslation markerGlobalTranslation = client.GetMarkerGlobalTranslation(subjectName, markerName);
			// insert global translations of the marker into a Vec3
			//SimTK::Vec3 markerTranslations(markerGlobalTranslation.Translation[0]*0.001, markerGlobalTranslation.Translation[1]*0.001, markerGlobalTranslation.Translation[2]*0.001);
			// set translations of marker at index k into markerDataMatrix
			markerDataMatrix.set(0, k, markerTranslations);
			std::cout << "[" << markerTranslations.get(0) << ", " << markerTranslations.get(1) << ", " << markerTranslations.get(2) << "]" << std::endl;
			std::cout << markerNameVector.at(k) << std::endl;
		}
		std::cout << "Done." << std::endl;

		// load model
		OpenSim::Model model("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full.osim");

		// create state
		SimTK::State s = model.initSystem();

		s.updTime() = frameNumber.FrameNumber;

		// form a timeseriestable of the marker coordinates
		OpenSim::TimeSeriesTable_<SimTK::Vec3> timeSeriesTable(timeVector, markerDataMatrix, markerNameVector);

		// create a markers reference using the timeseriestable
		OpenSim::MarkersReference markersReference(timeSeriesTable, markerWeightSet, OpenSim::Units::Millimeters);

		// write markers reference to file
		markersReference.print("C:/Users/wksadmin/source/repos/OpenSimLive/markersRef.osim");

		// create coordinate reference
		SimTK::Array_<OpenSim::CoordinateReference> coordinateReference;

		// use inversekinematicssolver with markersreference to solve
		OpenSim::InverseKinematicsSolver iks(model, markersReference, coordinateReference, SimTK::Infinity);

		iks.setAccuracy(1e-4);
		bool isAssembled = false;
		while (!isAssembled) {
			try {

				iks.assemble(s);
				isAssembled = true;
			}
			catch (...) {
				std::cerr << "Time " << s.getTime() << " Model not assembled" << std::endl;
				isAssembled = false;
			}
		}
		// calculate coordinates for state s
		iks.track(s);

		// get coordinates from state s
		SimTK::Vector stateQ(s.getQ());
		// get number of coordinates (joint angles) in the model
		int numCoordinates = model.getNumCoordinates();
		std::vector<double> q(numCoordinates);
		for (int j = 0; j < numCoordinates; j++) {
			q[j] = stateQ[j];
			std::cout << "Q" << j << ": " << q[j] << std::endl;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(msDelay));
	}
	*/

	return;
}


int main(int argc, char *argv[])
{
    if (argc < 2) {
		// report version
		std::cout << argv[0] << " version " << OpenSimLive_VERSION_MAJOR << "." << OpenSimLive_VERSION_MINOR << std::endl;
		std::cout << "Usage: " << argv[0] << " number" << std::endl;
		


		std::cout << "Connecting to MTw Awinda data stream..." << std::endl;
		// connect to XSens IMUs
		ConnectToDataStream();

		
		std::cout << "Finished." << std::endl;
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
