// Miscellaneous functions that are used by several programs are declared here.
#pragma once
#include <OpenSim.h>
#include <string>
#include <vector>
#include <IMUPlacerLive.h>
#include <XMLFunctions.h>
#include <XsensDataReader.h>

// This function fills a TimeSeriesTable with quaternion values for a single time frame.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> fillQuaternionTable(std::vector<MtwCallback*>& mtwCallbacks, std::vector<XsQuaternion>& quaternionVector);