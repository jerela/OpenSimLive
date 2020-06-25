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

// this function takes a serial/ID of a sensor and the filename of an XML file mapping serials to IMU labels, and returns the label
std::string sensorIdToLabel(std::string id, std::string mappingsFileName) {

	// get the file connecting IMU serials to their names in the model
	SimTK::Xml::Document sensorMappingsXML(mappingsFileName);
	// get the root element of the XML file
	SimTK::Xml::Element rootElement = sensorMappingsXML.getRootElement();
	// get the child element of the root element
	SimTK::Xml::Element xsensDataReaderSettingsElement = rootElement.getRequiredElement("XsensDataReaderSettings");
	// get the child element of the child element of the root element
	SimTK::Xml::Element experimentalSensorsElement = xsensDataReaderSettingsElement.getRequiredElement("ExperimentalSensors");
	// get all child elements with tag <ExperimentalSensor> of the element <ExperimentalSensors>
	SimTK::Array_<SimTK::Xml::Element> experimentalSensorElements = experimentalSensorsElement.getAllElements("ExperimentalSensor");
	// get the number of sensors defined in the XML file
	int numberOfXMLSensors = experimentalSensorElements.size();
	SimTK::Xml::Element nameInModelElement;

	// boolean to check if we have found a matching sensor
	bool sensorMatchFound;

	// initially as "NotFound"
	std::string sensorNameInModel("NotFound");

	// initially no matching IDs/serials are found
	sensorMatchFound = 0;

	// iterate through all sensors defined in the XML file
	for (int m = 0; m < numberOfXMLSensors; m++) {

		// get the serial number of m'th sensors
		std::string currentXMLSerial = experimentalSensorElements.at(m).getRequiredAttributeValue("name");
		// remove the first char (underscore) from the string
		currentXMLSerial.erase(0, 1);

		// get the serial number of m'
		nameInModelElement = experimentalSensorElements.at(m).getRequiredElement("name_in_model");
		//std::cout << "Checking if " << currentXMLSerial << " matches " << currentSensorId << std::endl;

		// if serial of currently iterated IMU in XML file matches the serial of the IMU we are receiving data from
		if (id == currentXMLSerial) {
			sensorMatchFound = 1;
			//std::cout << "Match found!" << std::endl;

			// get the name of the experimental sensor in the model
			sensorNameInModel = nameInModelElement.getValue();
			break;
		}

	}
	return sensorNameInModel;
}

// this function converts degrees to radians
double deg2rad(double deg) {
	return (deg * acos(0.0) / 90);
}

// this function creates a rotation matrix out of Euler angles
SimTK::Matrix_<double> eulerToRotationMatrix(SimTK::Vec3 euler) {
	// define the elements of the rotation matrix
	double c1 = cos(euler[0]);
	double c2 = cos(euler[1]);
	double c3 = cos(euler[2]);
	double s1 = sin(euler[0]);
	double s2 = sin(euler[1]);
	double s3 = sin(euler[2]);
	// create a 3x3 matrix
	SimTK::Matrix_<double> rotationMatrix(3, 3);
	// set the values of the matrix
	rotationMatrix.set(0, 0, c2 * c3);
	rotationMatrix.set(0, 1, -c2 * s3);
	rotationMatrix.set(0, 2, s2);
	rotationMatrix.set(1, 0, c1 * s3 + c3 * s1 * s2);
	rotationMatrix.set(1, 1, c1 * c3 - s1 * s2 * s3);
	rotationMatrix.set(1, 2, c2 * s1);
	rotationMatrix.set(2, 0, s1 * s3 - c1 * c3 * s2);
	rotationMatrix.set(2, 1, c3 * s1 + c1 * s2 * s3);
	rotationMatrix.set(2, 2, c1 * c2);

	return rotationMatrix;
}

SimTK::Vec3 rotationMatrixToEuler(SimTK::Matrix_<double> matrix) {
	double m11 = matrix.get(0, 0);
	double m12 = matrix.get(0, 1);
	double m13 = matrix.get(0, 2);
	double m21 = matrix.get(1, 0);
	double m22 = matrix.get(1, 1);
	double m23 = matrix.get(1, 2);
	double m31 = matrix.get(2, 0);
	double m32 = matrix.get(2, 1);
	double m33 = matrix.get(2, 2);

	// get Euler angles as radians
	double pitch = asin(m13);
	double yaw = acos(m11 / cos(pitch));
	double roll = acos(m33 / cos(pitch));
	SimTK::Vec3 euler(roll, pitch, yaw);
	return euler;
}

// performs matrix multiplication on 3x3 matrices and returns the product
SimTK::Matrix_<double> matrixMultiplication(SimTK::Matrix_<double> firstMatrix, SimTK::Matrix_<double> secondMatrix) {
	// construct the product matrix
	SimTK::Matrix_<double> productMatrix(3, 3);
	double value;

	value = firstMatrix.get(0, 0) * secondMatrix.get(0, 0) + firstMatrix.get(0, 1) * secondMatrix.get(1, 0) + firstMatrix.get(0, 2) * secondMatrix.get(2, 0);
	productMatrix.set(0, 0, value);
	value = firstMatrix.get(0, 0) * secondMatrix.get(0, 1) + firstMatrix.get(0, 1) * secondMatrix.get(1, 1) + firstMatrix.get(0, 2) * secondMatrix.get(2, 1);
	productMatrix.set(0, 1, value);
	value = firstMatrix.get(0, 0) * secondMatrix.get(0, 2) + firstMatrix.get(0, 1) * secondMatrix.get(1, 2) + firstMatrix.get(0, 2) * secondMatrix.get(2, 2);
	productMatrix.set(0, 2, value);

	value = firstMatrix.get(1, 0) * secondMatrix.get(0, 0) + firstMatrix.get(1, 1) * secondMatrix.get(1, 0) + firstMatrix.get(1, 2) * secondMatrix.get(2, 0);
	productMatrix.set(1, 0, value);
	value = firstMatrix.get(1, 0) * secondMatrix.get(0, 1) + firstMatrix.get(1, 1) * secondMatrix.get(1, 1) + firstMatrix.get(1, 2) * secondMatrix.get(2, 1);
	productMatrix.set(1, 1, value);
	value = firstMatrix.get(1, 0) * secondMatrix.get(0, 2) + firstMatrix.get(1, 1) * secondMatrix.get(1, 2) + firstMatrix.get(1, 2) * secondMatrix.get(2, 2);
	productMatrix.set(1, 2, value);

	value = firstMatrix.get(2, 0) * secondMatrix.get(0, 0) + firstMatrix.get(2, 1) * secondMatrix.get(1, 0) + firstMatrix.get(2, 2) * secondMatrix.get(2, 0);
	productMatrix.set(2, 0, value);
	value = firstMatrix.get(2, 0) * secondMatrix.get(0, 1) + firstMatrix.get(2, 1) * secondMatrix.get(1, 1) + firstMatrix.get(2, 2) * secondMatrix.get(2, 1);
	productMatrix.set(2, 1, value);
	value = firstMatrix.get(2, 0) * secondMatrix.get(0, 2) + firstMatrix.get(2, 1) * secondMatrix.get(1, 2) + firstMatrix.get(2, 2) * secondMatrix.get(2, 2);
	productMatrix.set(2, 2, value);

	return productMatrix;
}

/*
// this function takes as input two euler angle sets, converts them to rotation matrices, rotates the other set and returns it as Euler angles
SimTK::Vec3 transformEuler(SimTK::Vec3 sensorToOpenSim, SimTK::Vec3 sensorEuler) {
	
	std::cout << "Euler for sensor to OpenSim: [" << sensorToOpenSim[0] << ", " << sensorToOpenSim[1] << ", " << sensorToOpenSim[2] << "]" << std::endl;
	std::cout << "Euler for IMU sensor: [" << sensorEuler[0] << ", " << sensorEuler[1] << ", " << sensorEuler[2] << "]" << std::endl;

	// convert Euler angles to rotation matrices
	SimTK::Matrix_<double> sensorToOpenSimMatrix = eulerToRotationMatrix(sensorToOpenSim);
	SimTK::Matrix_<double> sensorMatrix = eulerToRotationMatrix(sensorEuler);
	std::cout << "Euler angles converted to rotation matrices." << std::endl;

	std::cout << "Rotation matrix for IMU:" << std::endl;
	std::cout << "[" << sensorMatrix.get(0, 0) << ", " << sensorMatrix.get(0, 1) << ", " << sensorMatrix.get(0, 2) << std::endl;
	std::cout << " " << sensorMatrix.get(1, 0) << ", " << sensorMatrix.get(1, 1) << ", " << sensorMatrix.get(1, 2) << std::endl;
	std::cout << " " << sensorMatrix.get(0, 0) << ", " << sensorMatrix.get(0, 1) << ", " << sensorMatrix.get(0, 2) << "]" << std::endl;

	// calculate the matrix product
	SimTK::Matrix_<double> productMatrix = matrixMultiplication(sensorToOpenSimMatrix, sensorMatrix);
	std::cout << "Matrix product calculated." << std::endl;

	// convert it back into Euler angles
	SimTK::Vec3 finalEuler = rotationMatrixToEuler(productMatrix);
	std::cout << "Rotation matrix converted into Euler angles:" << std::endl;
	std::cout << "[" << finalEuler[0] << ", " << finalEuler[1] << ", " << finalEuler[2] << "]" << std::endl;
	return finalEuler;
}
*/

SimTK::Vec3 transformOrientation(SimTK::Vec3 sensorToOpenSim, XsMatrix sensorMatrix) {

	std::cout << "Euler for sensor to OpenSim: [" << sensorToOpenSim[0] << ", " << sensorToOpenSim[1] << ", " << sensorToOpenSim[2] << "]" << std::endl;

	// convert Euler angles to rotation matrices
	SimTK::Matrix_<double> sensorToOpenSimMatrix = eulerToRotationMatrix(sensorToOpenSim);
	std::cout << "Euler angles converted to rotation matrices." << std::endl;

	std::cout << "Rotation matrix for IMU:" << std::endl;
	std::cout << "[" << sensorMatrix.value(0, 0) << ", " << sensorMatrix.value(0, 1) << ", " << sensorMatrix.value(0, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(1, 0) << ", " << sensorMatrix.value(1, 1) << ", " << sensorMatrix.value(1, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(0, 0) << ", " << sensorMatrix.value(0, 1) << ", " << sensorMatrix.value(0, 2) << "]" << std::endl;

	std::cout << "Rotation matrix for sensor to OpenSim:" << std::endl;
	std::cout << "[" << sensorToOpenSimMatrix.get(0, 0) << ", " << sensorToOpenSimMatrix.get(0, 1) << ", " << sensorToOpenSimMatrix.get(0, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(1, 0) << ", " << sensorToOpenSimMatrix.get(1, 1) << ", " << sensorToOpenSimMatrix.get(1, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(0, 0) << ", " << sensorToOpenSimMatrix.get(0, 1) << ", " << sensorToOpenSimMatrix.get(0, 2) << "]" << std::endl;

	// convert sensor rotation matrix from Xs to SimTK
	SimTK::Matrix_<double> sensorMatrixSimTK(3,3);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sensorMatrixSimTK.set(i, j, sensorMatrix.value(i, j));
		}
	}
	
	
	// calculate the matrix product
	SimTK::Matrix_<double> productMatrix = matrixMultiplication(sensorToOpenSimMatrix, sensorMatrixSimTK);
	std::cout << "Matrix product calculated." << std::endl;

	// convert it back into Euler angles
	SimTK::Vec3 finalEuler = rotationMatrixToEuler(productMatrix);
	std::cout << "Rotation matrix converted into Euler angles:" << std::endl;
	std::cout << "[" << finalEuler[0] << ", " << finalEuler[1] << ", " << finalEuler[2] << "]" << std::endl;
	return finalEuler;
}


// this function creates a calibrated .osim model from live data (as opposed to data exported after recording from MT manager)
std::string calibrateOpenSimModel(std::vector<MtwCallback*> mtwCallbacks, std::vector<XsMatrix> initialMatrix){

	size_t callbacksSize = mtwCallbacks.size();

	// model used as a base for the calibrated model
	OpenSim::Model baseModel("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full.osim");
	std::string baseIMU("pelvis_imu");
	std::string baseHeadingAxis("z");
	SimTK::Vec3 sensorToOpenSimRotations(-1.570796, 0, 0);
	std::string outModelFile("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full_calib.osim");
	
	// let's calculate that in OpenSim coordinate system

	// the next step is to add PhysicalOffsetFrames representing the IMUs under each body in the model

	SimTK::Xml::Document baseModelFile("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full.osim");
	// get the root element of the XML file
	SimTK::Xml::Element rootElement = baseModelFile.getRootElement();
	// get the child element of the root element
	SimTK::Xml::Element modelElement = rootElement.getRequiredElement("Model");
	// get the child element of the child element of the root element
	SimTK::Xml::Element bodySetElement = modelElement.getRequiredElement("BodySet");
	// get the child element of the BodySet element
	SimTK::Xml::Element objectsElement = bodySetElement.getRequiredElement("objects");
	// get all child elements with tag <Body> of the element <objects>
	SimTK::Array_<SimTK::Xml::Element> bodyElements = objectsElement.getAllElements("Body");
	// get the number of sensors defined in the XML file
	int numberOfBodies = bodyElements.size();

	std::string sensorLabel; std::string currentSensorId; std::string bodyName; std::string currentBody;

	// iterate through all sensors that are active
	for (size_t k = 0; k < callbacksSize; ++k) {
		// get ID/serial of currently iterated sensor
		currentSensorId = mtwCallbacks[k]->device().deviceId().toString().toStdString();
		// get the corresponding label
		sensorLabel = sensorIdToLabel(currentSensorId, "C:/Users/wksadmin/source/repos/OpenSimLive/SensorMappings.xml");
		bodyName = sensorLabel;
		// remove 4 trailing characters ("_imu") from bodyName
		for (int m = 0; m < 4; ++m) {
			bodyName.pop_back();
		}
		// iterate through bodies in the model until a match to bodyName is found
		for (int m = 0; m < numberOfBodies; ++m) {
			currentBody = bodyElements.at(m).getRequiredAttributeValue("name");
			// if we find a match, add PhysicalOffsetFrame for the IMU
			if (bodyName == currentBody) {
				// create components
				//SimTK::Xml::Element componentsElement = bodyElements[m].getRequiredElement("components");
				SimTK::Xml::Element componentsElement("components");
				// create the new XML element to be inserted
				SimTK::Xml::Element physicalOffsetFrameElement("PhysicalOffsetFrame");
				physicalOffsetFrameElement.setAttributeValue("name", sensorLabel);
				// create child elements of the new XML element
				SimTK::Xml::Element frameGeometryElement("FrameGeometry");
				frameGeometryElement.setAttributeValue("name", "frame_geometry");
				SimTK::Xml::Element socketFrameFGElement("socket_frame", "..");
				SimTK::Xml::Element scaleFactorsElement("scale_factors", "0.2 0.2 0.2");
				frameGeometryElement.appendNode(socketFrameFGElement);
				frameGeometryElement.appendNode(scaleFactorsElement);
				SimTK::Xml::Element attachedGeometryElement("attached_geometry");
				SimTK::Xml::Element brickElement("Brick");
				brickElement.setAttributeValue("name", sensorLabel+"_geom1");
				SimTK::Xml::Element socketFrameBElement("socket_frame", "..");
				SimTK::Xml::Element appearanceElement("Appearance");
				SimTK::Xml::Element colorElement("color", "1 0.5 0");
				appearanceElement.appendNode(colorElement);
				SimTK::Xml::Element halfLengthsElement("half_lengths", "0.02 0.01 0.005");
				brickElement.appendNode(socketFrameBElement);
				brickElement.appendNode(appearanceElement);
				brickElement.appendNode(halfLengthsElement);
				attachedGeometryElement.appendNode(brickElement);
				SimTK::Xml::Element socketParentElement("socket_parent", "..");
				SimTK::Xml::Element translationElement("translation", "-0.0707 0 0");
				// orientation must be calculated from initial IMU orientations and given as a string with 3 elements separated by whitespaces
				SimTK::Vec3 finalIMUOrientation = transformOrientation(sensorToOpenSimRotations, initialMatrix[k]);
				std::string orientationVector = std::to_string(finalIMUOrientation[0]) + " " + std::to_string(finalIMUOrientation[1]) + " " + std::to_string(finalIMUOrientation[2]);
				SimTK::Xml::Element orientationElement("orientation", orientationVector);
				// insert child elements into the XML element to be inserted
				physicalOffsetFrameElement.appendNode(frameGeometryElement);
				physicalOffsetFrameElement.appendNode(attachedGeometryElement);
				physicalOffsetFrameElement.appendNode(socketParentElement);
				physicalOffsetFrameElement.appendNode(translationElement);
				physicalOffsetFrameElement.appendNode(orientationElement);
				// insert the new XML element under the parent node
				componentsElement.appendNode(physicalOffsetFrameElement);
				bodyElements[m].appendNode(componentsElement);
			}
		}
	}

	// write the modified model file into an XML document
	baseModelFile.writeToFile(outModelFile);

	return outModelFile;
}

void InverseKinematicsFromIMUs(std::vector<XsMatrix> matrixData, std::vector<MtwCallback*> mtwCallbacks, SimTK::Real timeInteger, std::string modelFileName){

	int numberOfSensors = mtwCallbacks.size();

	std::cout << "Numbers of sensors: " << numberOfSensors << ", size of matrixData: " << matrixData.size() << std::endl;

	// create a vector for IMU names
	std::vector<std::string> sensorNameVector;
	// create a vector for time (in a single frame, so just a vector with a single value)
	std::vector<double> timeVector{ 0 };
	// create a matrix for IMU orientations
	SimTK::Matrix_<SimTK::Rotation_<double>> orientationDataMatrix(1, numberOfSensors); // marker positions go here

	// declare variables to be used in the loop
	double m11, m12, m13, m21, m22, m23, m31, m32, m33;
	SimTK::Rotation_<double> tempMatrix;
	XsMatrix xsMatrix;
	std::string currentSensorId; std::string sensorNameInModel;


	// insert orientation data from matrixData to orientationDataMatrix; note that orientationDataMatrix elements are 3x3 rotation matrices
	for (int k = 0; k < numberOfSensors; k++) {

		xsMatrix = matrixData[k];
		if (xsMatrix.empty()) {
			std::cout << "Matrix at k=" << k << " is empty. Returning." << std::endl;
			return;
		}

		// give values to variables representing individual elements of a 3x3 rotation matrix
		m11 = xsMatrix.value(0, 0);
		m12 = xsMatrix.value(0, 1);
		m13 = xsMatrix.value(0, 2);
		m21 = xsMatrix.value(1, 0);
		m22 = xsMatrix.value(1, 1);
		m23 = xsMatrix.value(1, 2);
		m31 = xsMatrix.value(2, 0);
		m32 = xsMatrix.value(2, 1);
		m33 = xsMatrix.value(2, 2);
		std::cout << "[" << m11 << "; " << m12 << "; " << m13 << " " << std::endl;
		std::cout << " " << m21 << "; " << m22 << "; " << m23 << " " << std::endl;
		std::cout << " " << m31 << "; " << m32 << "; " << m33 << "]" << std::endl;

		// set these values to tempMatrix
		tempMatrix.set(0, 0, m11);
		tempMatrix.set(0, 1, m12);
		tempMatrix.set(0, 2, m13);
		tempMatrix.set(1, 0, m21);
		tempMatrix.set(1, 1, m22);
		tempMatrix.set(1, 2, m23);
		tempMatrix.set(2, 0, m31);
		tempMatrix.set(2, 1, m32);
		tempMatrix.set(2, 2, m33);

		// fill orientationDataMatrix with orientation matrices for each sensor
		orientationDataMatrix.set(0, k, tempMatrix);

		// populate sensorNameVector with names of the IMUs on the model

		// initialize currentSensorId with the serial of the IMU
		currentSensorId = mtwCallbacks[k]->device().deviceId().toString().toStdString();
		
		sensorNameInModel = sensorIdToLabel(currentSensorId, "C:/Users/wksadmin/source/repos/OpenSimLive/SensorMappings.xml");

		if (sensorNameInModel != "NotFound"){
			// push the name of the IMU in the model into sensorNameVector
			sensorNameVector.push_back(sensorNameInModel);
		}
		else{
			std::cout << "No match found for sensor " << currentSensorId << std::endl;
		}
	}

	std::cout << "For-loop finished, preparing OpenSim API methods for IK." << std::endl;

	// form a timeseriestable of the IMU orientations
	OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> timeSeriesTable(timeVector, orientationDataMatrix, sensorNameVector);

	// load model
	OpenSim::Model model(modelFileName);

	//std::cout << "TimeSeriesTable and Model objects created, preparing to initialize state." << std::endl;

	// enable visualizer
	//model.setUseVisualizer(true);

	// create state
	SimTK::State s = model.initSystem();

	// update state time
	//s.updTime() = timeInteger/1000;

	//std::cout << "State initialized, creating Reference objects." << std::endl;

	// create MarkersReference object
	OpenSim::MarkersReference markersReference;

	// create OrientationsReference object
	OpenSim::OrientationsReference orientationsReference(timeSeriesTable);

	// create coordinate reference
	SimTK::Array_<OpenSim::CoordinateReference> coordinateReference;

	//std::cout << "Reference objects created, constructing InverseKinematicsSolver" << std::endl;

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
		}
	}

	//std::cout << "Preparing to track state s" << std::endl;
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


	//OpenSim::ModelVisualizer::show(s);

	std::cout << "IK finished successfully." << std::endl;
	return;
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

		SimTK::Real timeInteger = 1;
		std::vector<XsEuler> eulerData(mtwCallbacks.size()); // Room to store euler data for each mtw
		std::vector<XsMatrix> matrixData(mtwCallbacks.size()); // for data in orientation matrix form that OpenSim requires
		//unsigned int printCounter = 0;
		
		std::string calibratedModelFile;


		bool mainDataLoop = true;
		bool getDataKeyHit = false;
		bool calibrateModelKeyHit = false;
		
		std::cout << "Entering data streaming and IK loop. Press C to calibrate model, V to calculate IK and X to quit." << std::endl;

		//while (!_kbhit()) {
		do
		{
			//XsTime::msleep(0);

		

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

			if (newDataAvailable && calibrateModelKeyHit) {
				calibratedModelFile = calibrateOpenSimModel(mtwCallbacks,matrixData);
				calibrateModelKeyHit = false;
				std::cout << "Model has been calibrated." << std::endl;
			}

			if (newDataAvailable && getDataKeyHit)
			{

				//size_t numberOfSensors = mtwCallbacks.size();
				InverseKinematicsFromIMUs(matrixData, mtwCallbacks, timeInteger, calibratedModelFile);
				timeInteger++;
				
				/*
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
				++printCounter;*/
				getDataKeyHit = false;
			}

			char hitKey = ' ';
			if (_kbhit())
			{
				hitKey = toupper((char)_getch());
				mainDataLoop = (hitKey != 'X');
				getDataKeyHit = (hitKey == 'V');
				calibrateModelKeyHit = (hitKey == 'C');
			}
			
		} while (mainDataLoop);
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
