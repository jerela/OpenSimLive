#pragma once
#include <OpenSim.h>
#include <string>
#include <vector>
#include <XMLFunctions.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// This function reads variables such as model file to be used from an XML file.
std::string ConfigReader(const std::string& fileName, const std::string& elementName) {
	// get the XML file
	SimTK::Xml::Document mainConfigXML(OPENSIMLIVE_ROOT + "/Config/" + fileName);
	// get the root element of the XML file
	SimTK::Xml::Element rootElement = mainConfigXML.getRootElement();
	// get the child element of the root element
	SimTK::Xml::Element soughtElement = rootElement.getRequiredElement(elementName);
	return soughtElement.getValue();
}

// This function reads variables as a vector of strings from an XML file.
std::vector<std::string> ConfigReaderVector(const std::string& fileName, const std::string& elementName) {
	// get the XML file
	SimTK::Xml::Document mainConfigXML(OPENSIMLIVE_ROOT + "/Config/" + fileName);
	// get the root element of the XML file
	SimTK::Xml::Element rootElement = mainConfigXML.getRootElement();
	// get the child element of the root element
	SimTK::Xml::Element soughtElement = rootElement.getRequiredElement(elementName);
	// string containing all the substrings we want to separate into a vector
	std::string wholeString = soughtElement.getValue();
	// empty vector that will be populated and returned
	std::vector<std::string> outputVector;
	// stringstream is a simple way to separate the whitespace-separated elements from the whole string
	std::stringstream ss(wholeString);
	// loop through all elements (loop as long as there is anything left in the stringstream)
	do {
		// represents a piece of text separated by whitespaces from other text elements
		std::string textElem;
		// write from stringstream to IMULabel
		ss >> textElem;
		// push IMULabel in a vector
		outputVector.push_back(textElem);
	} while (!ss.eof());
	return outputVector;
}

// This function returns the sensor to OpenSim rotations as defined in the IMU placer setup file.
SimTK::Vec3 get_sensor_to_opensim_rotations() {
	std::string IMUPlacerSetupFileName = ConfigReader("MainConfiguration.xml", "imu_placer_setup_file");
	SimTK::Xml::Document IMUPlacerXML(OPENSIMLIVE_ROOT + "/Config/" + IMUPlacerSetupFileName);
	SimTK::Xml::Element rootElement = IMUPlacerXML.getRootElement();
	SimTK::Xml::Element placerElement = rootElement.getRequiredElement("IMUPlacer");
	SimTK::Xml::Element soughtElement = placerElement.getRequiredElement("sensor_to_opensim_rotations");
	SimTK::Vec3 rotations;
	soughtElement.getValueAs(rotations);
	return rotations;
}

// this function takes a serial/ID of a sensor and the filename of an XML file mapping serials to IMU labels, and returns the label
std::string sensorIdToLabel(const std::string& id, const std::string& mappingsFileName) {

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

// This function calibrates an OpenSim model from setup file, similarly to how MATLAB scripting commands for OpenSense work.
std::string calibrateModelFromSetupFile(const std::string& IMUPlacerSetupFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quaternionTimeSeriesTable)
{
	// construct IMUPlacer
	OpenSimLive::IMUPlacerLive IMUPlacer(IMUPlacerSetupFile);

	// set the value of subjectHeight_ for model scaling
	double subjectHeight = std::stod(ConfigReader("MainConfiguration.xml", "subject_height"));
	double modelHeight = std::stod(ConfigReader("MainConfiguration.xml", "model_height"));
	if (subjectHeight > 0) {
		IMUPlacer.setSubjectHeight(subjectHeight);
		IMUPlacer.setModelHeight(modelHeight);
		std::cout << "Set subject height to " << subjectHeight << std::endl;
	}

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

template<typename myType>
void saveTimeSeriesToTxtFile(std::vector<myType> timeVector, std::vector<myType> dataVector, const std::string& rootDir, const std::string& resultsDir, const std::string& fileName, const std::string& description, const std::string& labels) {
	std::string filePath(rootDir + "/" + resultsDir + "/" + fileName);
	std::ofstream outputFile;
	// open and set file to discard any contents that existed in the file previously (truncate mode)
	outputFile.open(filePath, std::ios_base::out | std::ios_base::trunc);
	if (outputFile.is_open())
	{
		outputFile << description;
		outputFile << labels;

		for (unsigned int i = 0; i < dataVector.size(); ++i) { // iteration through rows
			outputFile << "\n" << timeVector[i] << "\t" << dataVector[i];
		}
		outputFile.close();
		std::cout << "Data written to file " << filePath << std::endl;
	}
	else {
		std::cout << "Failed to open file " << filePath << std::endl;
	}
}
template void saveTimeSeriesToTxtFile<int>(std::vector<int> timeVector, std::vector<int> dataVector, const std::string& rootDir, const std::string& resultsDir, const std::string& fileName, const std::string& description, const std::string& labels);
template void saveTimeSeriesToTxtFile<double>(std::vector<double> timeVector, std::vector<double> dataVector, const std::string& rootDir, const std::string& resultsDir, const std::string& fileName, const std::string& description, const std::string& labels);
template void saveTimeSeriesToTxtFile<float>(std::vector<float> timeVector, std::vector<float> dataVector, const std::string& rootDir, const std::string& resultsDir, const std::string& fileName, const std::string& description, const std::string& labels);
