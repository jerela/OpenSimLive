// FUNCTIONS THAT USE XML HEAVILY ARE DECLARED HERE
#include <OpenSim.h>
#include <string>
#include <vector>

// This function reads some "main" variables such as model file to be used from an XML file.
std::string mainConfigReader(std::string elementName);
// This function returns the sensor to OpenSim rotations as defined in the IMU placer setup file.
SimTK::Vec3 get_sensor_to_opensim_rotations();
// This function takes a serial/ID of a sensor and the filename of an XML file mapping serials to IMU labels, and returns the label
std::string sensorIdToLabel(std::string id, std::string mappingsFileName);
