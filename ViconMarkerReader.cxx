// OpenSimLive.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <OpenSimLiveConfig.h>
#include "DataStreamClient.h"
#include <chrono>
#include <thread>
#include <OpenSim.h>
#include <vector>
using namespace ViconDataStreamSDK::CPP;

void ConnectToDataStream(int msDelay) {
	
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

	Output_GetFrame frame;
	Output_GetFrameNumber frameNumber;
	// get the number of the current frame
	frameNumber = client.GetFrameNumber();
	// get the current frame
	frame = client.GetFrame();
	std::cout << "Got frame number " << frameNumber.FrameNumber << std::endl;

	// set the number of frames the client should buffer to 1 frame
	client.SetBufferSize(1);

	// set the stream mode into receiving frames to the client only when we call GetFrame()
	client.SetStreamMode(StreamMode::ClientPull);

	// set the mapping of the 3D axes to match a right-handed coordinate system
	client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);


	std::cout << "Initialization of connection done" << std::endl;

	// INITIALIZING DONE, GETTING DATA

	// get number of subjects
	std::cout << "Found " << client.GetSubjectCount().SubjectCount << " subjects." << std::endl;

	// get name of subject
	std::string subjectName = client.GetSubjectName(0).SubjectName;
	std::cout << "Found subject " << subjectName << std::endl;


	// print number of found markers
	Output_GetMarkerCount markerCount = client.GetMarkerCount(subjectName);
	int numberOfMarkers = markerCount.MarkerCount;
	std::cout << "Found " << std::to_string(numberOfMarkers) << " markers." << std::endl;



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
			std::string markerName = client.GetMarkerName(subjectName, k).MarkerName;
			markerNameVector.push_back(markerName); // push name of marker at index k to vector markerNameVector
			markerWeightSet.insert(k, OpenSim::MarkerWeight(markerName, 10)); // insert a MarkerWeight object with weight 10 to index k in markerWeightSet
			// get global translations of the marker
			Output_GetMarkerGlobalTranslation markerGlobalTranslation = client.GetMarkerGlobalTranslation(subjectName, markerName);
			// insert global translations of the marker into a Vec3
			SimTK::Vec3 markerTranslations(markerGlobalTranslation.Translation[0]*0.001, markerGlobalTranslation.Translation[1]*0.001, markerGlobalTranslation.Translation[2]*0.001);
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

	std::cout << "Disconnecting..." << std::endl;
	client.Disconnect();

	return;
}


int main(int argc, char *argv[])
{
    if (argc < 2) {
		// report version
		std::cout << argv[0] << " version " << OpenSimLive_VERSION_MAJOR << "." << OpenSimLive_VERSION_MINOR << std::endl;
		std::cout << "Usage: " << argv[0] << " number" << std::endl;
		
		// frequency to acquire data with
		int msDelay = 1000;


		std::cout << "Connecting to and initializing Vicon Nexus data stream..." << std::endl;
		// connect to Vicon Nexus and set initial settings
		ConnectToDataStream(msDelay);

		
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
