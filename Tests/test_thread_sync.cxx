#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <ThreadPoolContainer.h>
#include "conio.h" // for non-ANSI _kbhit() and _getch()
#include <XMLFunctions.h>
#include <XMLFunctionsXsens.h>
#include <thread>
#include <future>
#include <functional>
#include <IMUHandler.h>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

void updateConcurrentIKTool(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, std::chrono::steady_clock::time_point& clockStart, std::chrono::duration<double>& clockDuration, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable) {
	
	// calculate current duration
	clockDuration = (std::chrono::high_resolution_clock::now() - clockStart);
	// update current duration as time in IKTool
	IKTool.setTime(clockDuration.count());

	IKTool.update(false, quatTable);
	
	// simulate data sending to client, but only fetch the data without sending it
	std::vector<double> trackerResults = IKTool.getPointTrackerPositionsAndOrientations();
	size_t packetSize = trackerResults.size();
	//double* mirrorTherapyPacket = &trackerResults[0];
	double* mirrorTherapyPacket = trackerResults.data();
	
}




std::mutex mainMutex;
std::queue<OpenSim::TimeSeriesTableQuaternion> bufferContainer;
bool bufferInUse = false;
unsigned int maxBufferSize = 8;
unsigned int nIK = 0;
bool trialDone = false;

// This function reads quaternion values in a loop and saves them in a queue buffer.
void producerThread(std::chrono::duration<double>& clockDuration, double inputSeconds, OpenSimLive::IMUHandler& genericDataReader) {
	do {
		// update new quaternion table but don't get it yet
		genericDataReader.updateQuaternionTable();
		
		// if consumer is not accessing the buffer and the buffer size is not maximal
		if (!bufferInUse && bufferContainer.size() < maxBufferSize)
		{
			// set bufferInUse to true to prevent consumer from accessing the buffer
			bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// push the time series table to the shared buffer
			bufferContainer.push(genericDataReader.getQuaternionTable());
			bufferInUse = false;
		}

	} while (clockDuration.count() < inputSeconds);

	std::cout << "Producer done!" << std::endl;
}

// This function reads quaternion tables from a queue buffer and performs IK on them.
void consumerThread(std::chrono::duration<double>& clockDuration, std::chrono::steady_clock::time_point& clockStart, double inputSeconds, OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSimLive::ThreadPoolContainer& threadPoolContainer) {
	do {
		// if producer is not accessing hte buffer and the buffer contains values
		if (!bufferInUse && bufferContainer.size() > 0) {
			bufferInUse = true;
			std::lock_guard<std::mutex> lock(mainMutex);
			// get and pop the front of the queue
			OpenSim::TimeSeriesTableQuaternion quatTable(bufferContainer.front());
			bufferContainer.pop();
			bufferInUse = false;

			// start a new thread where the IK is calculated and increment the number of IK operations done
			threadPoolContainer.offerFuture(updateConcurrentIKTool, std::ref(IKTool), std::ref(clockStart), std::ref(clockDuration), quatTable);
			++nIK;
		}
	} while (clockDuration.count() < inputSeconds);

	std::cout << "Consumer done!" << std::endl;
}





void ConnectToDataStream(double inputSeconds, int inputThreads) {

	OpenSimLive::IMUHandler genericDataReader;

	std::string manufacturerStr = ConfigReader("MainConfiguration.xml", "IMU_manufacturer");
	IMUType manufacturer = simulated; // default to simulated in case the following if-statements fail
	if (manufacturerStr == "delsys")
		manufacturer = delsys;
	else if (manufacturerStr == "xsens")
		manufacturer = xsens;
	else if (manufacturerStr == "simulated")
		manufacturer = simulated;

	genericDataReader.setManufacturer(manufacturer);

	genericDataReader.initialize();

		
	std::string calibratedModelFile; // the file name of the calibrated OpenSim model will be stored here
	bool saveIKResults = ("true" == ConfigReader("MainConfiguration.xml", "save_ik_results")); // Boolean that determines if we want to save IK results (joint angles and their errors) to file when the program finishes. This can be very memory-heavy especially with long measurements.
	bool enableMirrorTherapy = (ConfigReader("MainConfiguration.xml", "station_parent_body") != "none"); // if "none", then set to false

	

	OpenSimLive::IMUInverseKinematicsToolLive IKTool; // object that calculates IK
	IKTool.setReportErrors(saveIKResults);

	// get the sensor to opensim rotations for IMUInverseKinematicsToolLive
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	
	// CALIBRATION STEP
	genericDataReader.updateQuaternionTable();
	// fill a timeseriestable with quaternion orientations of IMUs
	OpenSim::TimeSeriesTable_<SimTK::Quaternion>  quaternionTimeSeriesTable = genericDataReader.getQuaternionTable();
	// calibrate the model and return its file name
	calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quaternionTimeSeriesTable);
	// give IKTool the necessary inputs and run it
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quaternionTimeSeriesTable); // the orientations of IMUs
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setTime(0); // set the time of the first state as 0 at calibration
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.run(false); // true for visualization
	std::cout << "Model has been calibrated." << std::endl;
	// set private variables to be accessed in IK calculations
	if (enableMirrorTherapy == true) {
		IKTool.setPointTrackerBodyName(ConfigReader("MainConfiguration.xml", "station_parent_body"));
		IKTool.setPointTrackerReferenceBodyName(ConfigReader("MainConfiguration.xml", "station_reference_body"));
		IKTool.setPointTrackerEnabled(true);
		IKTool.setSavePointTrackerResults(true);
	}
	else {
		IKTool.setPointTrackerEnabled(false);
	}

	unsigned int maxThreads = inputThreads;
	//std::vector<std::future<void>> futureVector;

	//ThreadPool threadPool(maxThreads);
	OpenSimLive::ThreadPoolContainer threadPoolContainer(maxThreads);

	std::cout << "Entering measurement loop." << std::endl;
	auto clockStart = std::chrono::high_resolution_clock::now(); // get the starting time of IMU measurement loop
	std::chrono::duration<double> clockDuration;

	std::cout << clockDuration.count() << std::endl;

	// begin producer-consumer loops
	std::thread producer(producerThread, std::ref(clockDuration), inputSeconds, std::ref(genericDataReader));
	std::thread consumer(consumerThread, std::ref(clockDuration), std::ref(clockStart), inputSeconds, std::ref(IKTool), std::ref(threadPoolContainer));

	// wait until the threads are finished
	producer.join();
	consumer.join();

	double finalTime = clockDuration.count();

	std::cout << "Performed " << nIK << " iterations in " << finalTime << " seconds." << std::endl;
	std::cout << "Frame rate: " << ((double)nIK / finalTime) << " iterations per second." << std::endl;

	if (saveIKResults) {
		std::cout << "Saving IK results to file..." << std::endl;
		if (nIK < 100000) {
			IKTool.reportToFile();
		}
		else
		{
			std::cout << "More than 100000 iterations calculated, as a safety precaution program is not saving results to file!" << std::endl;
		}
	}

	// close the connection to IMUs
	genericDataReader.closeConnection();

	return;
}


int main(int argc, char *argv[])
{
	std::string inputSecsStr;
	std::cout << "Please input test duration in seconds: ";
	std::cin >> inputSecsStr;
	double inputSecs = stod(inputSecsStr);

	std::string inputThreadsStr;
	std::cout << "Please input the number of threads to be used: ";
	std::cin >> inputThreadsStr;
	int inputThreads = stoi(inputThreadsStr);

	std::cout << "Connecting to MTw Awinda data stream..." << std::endl;
	// connect to XSens IMUs, perform IK etc
	ConnectToDataStream(inputSecs, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
