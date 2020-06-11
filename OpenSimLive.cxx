// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <OpenSimLiveConfig.h>
#include "DataStreamClient.h"
#include <chrono>
#include <thread>
using namespace ViconDataStreamSDK::CPP;

void ConnectToDataStream() {
	
	// construct Datastream SDK
	ViconDataStreamSDK::CPP::Client client;

	// construct a version output object
	Output_GetVersion versionOutput = client.GetVersion();
	// print current revision number
	std::cout << "Revision of SDK: " << versionOutput.Revision << std::endl;

	// check if we're already connected
	Output_IsConnected connBool = client.IsConnected();
	if (connBool.Connected == 1) {
		std::cout << "Already connected." << std::endl;
	}
	// while we're not connected, attempt to connect every second
	while (connBool.Connected == 0) {
		std::cout << "Attempting to connect to Vicon Nexus." << std::endl;
		Output_Connect connectionOutput = client.Connect("localhost");
		std::cout << "Result: " << connectionOutput.Result << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
		connBool = client.IsConnected();
	}

	// enable marker data on the stream
	Output_EnableMarkerData markerOutput = client.EnableMarkerData();
	
	// set the number of frames the client should buffer to 1 frame
	client.SetBufferSize(1);

	return;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
		// report version
		std::cout << argv[0] << " version " << OpenSimLive_VERSION_MAJOR << "." << OpenSimLive_VERSION_MINOR << std::endl;
		std::cout << "Usage: " << argv[0] << " number" << std::endl;
		
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
