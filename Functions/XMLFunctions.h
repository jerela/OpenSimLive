// Miscellaneous functions that are used by several programs are declared here.
#pragma once
#include <OpenSim.h>
#include <string>
#include <vector>
#include <XsensDataReader.h>
#include <IMUPlacerLive.h>

// This function reads variables such as model file to be used from an XML file.
std::string ConfigReader(const std::string& fileName, const std::string& elementName);
// This function returns the sensor to OpenSim rotations as defined in the IMU placer setup file.
SimTK::Vec3 get_sensor_to_opensim_rotations();
// This function takes a serial/ID of a sensor and the filename of an XML file mapping serials to IMU labels, and returns the label
std::string sensorIdToLabel(const std::string& id, const std::string& mappingsFileName);
// This function fills a TimeSeriesTable with quaternion values for a single time frame.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> fillQuaternionTable(std::vector<MtwCallback*>& mtwCallbacks, std::vector<XsQuaternion>& quaternionVector);
// This function calibrates an OpenSim model from setup file, similarly to how MATLAB scripting commands for OpenSense work.
std::string calibrateModelFromSetupFile(const std::string& IMUPlacerSetupFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quaternionTimeSeriesTable);
