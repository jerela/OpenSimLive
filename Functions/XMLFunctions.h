// Miscellaneous functions that are used by several programs are declared here.
#pragma once
#include <OpenSim.h>
#include <string>
#include <vector>
#include <IMUPlacerLive.h>

// This function reads variables such as model file to be used from an XML file.
std::string ConfigReader(const std::string& fileName, const std::string& elementName);
// This function reads variables as a vector of strings from an XML file.
std::vector<std::string> ConfigReaderVector(const std::string& fileName, const std::string& elementName);
// This function returns the sensor to OpenSim rotations as defined in the IMU placer setup file.
SimTK::Vec3 get_sensor_to_opensim_rotations();
// This function takes a serial/ID of a sensor and the filename of an XML file mapping serials to IMU labels, and returns the label
std::string sensorIdToLabel(const std::string& id, const std::string& mappingsFileName);
// This function calibrates an OpenSim model from setup file, similarly to how MATLAB scripting commands for OpenSense work.
std::string calibrateModelFromSetupFile(const std::string& IMUPlacerSetupFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quaternionTimeSeriesTable);
// Generic function to save time series as .txt files.
template<typename myType>
void saveTimeSeriesToTxtFile(std::vector<myType> timeVector, std::vector<myType> dataVector, const std::string& rootDir, const std::string& resultsDir, const std::string& fileName, const std::string& description, const std::string& labels);
