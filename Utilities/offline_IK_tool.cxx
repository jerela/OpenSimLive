#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
#include <ThreadPoolContainer.h>
#include <XMLFunctions.h>
#include <future>
#include <functional>
#include <fstream>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

// Function that parses delimited-separated tokens from a string
template <typename elementType>
void parse(std::vector<elementType>& vector, std::string& targetString, std::string delimiter) {
	// initial position to start making the substring
	size_t position = 0;
	// token will hold "Time (s)", "Quaternion1" etc
	std::string token;
	// use tabulator as the delimiter between labels
	//std::string delimiter("\t");
	// this will be set to false when we are finally at the final token on a line
	bool readLine = true;
	// loop until all tokens have been read
	while (readLine) {
		readLine = ((position = targetString.find(delimiter)) != std::string::npos);
		// update token
		token = targetString.substr(0, position);
		// push it to the label vector
		vector.push_back(token);
		// erase that token + the delimiter from the beginning of the text line string
		targetString.erase(0, position + delimiter.length());
	}
}

// Functions that parses quaternions from a string containing several quaternions
void parseQuaternion(std::vector<SimTK::Quaternion>& quatVector, std::string quatString) {
	std::vector<std::string> quatElementVector;
	parse(quatElementVector, quatString, ",");
	// remove ~[ from the beginning of the first element
	quatElementVector[0].erase(0, 2);
	// remove ] from the end of the last element
	quatElementVector[3] = quatElementVector[3].substr(0,quatElementVector[3].find("]"));
	// construct a quaternion
	SimTK::Quaternion_<SimTK::Real> quaternion(stod(quatElementVector[0]), stod(quatElementVector[1]), stod(quatElementVector[2]), stod(quatElementVector[3]));
	// push the quaternion in a vector
	//std::cout << quaternion << std::endl;
	quatVector.push_back(quaternion);
}

// Function that constructs a time series table of quaternions, which is used in IK, from a time series of quaternions in .txt file
OpenSim::TimeSeriesTable_<SimTK::Quaternion> quaternionTableFromTextFile(std::string quatFileName) {

	std::cout << "Starting to process the file..." << std::endl;
	std::ifstream quatFile(OPENSIMLIVE_ROOT + "/OpenSimLive-results/" + quatFileName);
	std::string textLine;

	std::vector<std::string> labels;
	std::vector<double> times;
	std::vector<std::vector<SimTK::Quaternion>> quaternionVector;

	unsigned int nLine = 0;
	if (quatFile.is_open()) {
		while (std::getline(quatFile, textLine)) {
			++nLine;
			// if we are reading labels
			if (nLine == 2) {
				parse(labels, textLine, "\t");
				// remove "Time" from labels as it's not required for TimeSeriesTable
				labels.erase(labels.begin());
				// trim leading and trailing whitespaces
				for (unsigned int i = 0; i < labels.size(); ++i) {
					size_t startPos = labels[i].find_first_not_of(" ");
					labels[i] = labels[i].substr(startPos);
					size_t endPos = labels[i].find_last_not_of(" ");
					labels[i] = labels[i].substr(0, endPos + 1);
					std::cout << labels[i] << std::endl;
				}
			}
			// if we are past the description and labels
			if (nLine > 2) {
				std::vector<SimTK::Quaternion> quaternions;
				size_t position = 0;
				std::string token;
				std::string delimiter("\t");
				bool readLine = true;
				unsigned int elementIndex = 0;
				//readLine = ((position = textLine.find(delimiter)) != std::string::npos);
				while (readLine) {
					readLine = ((position = textLine.find(delimiter)) != std::string::npos);
					//std::cout << "Position of delimiter: " << position << std::endl;
					token = textLine.substr(0, position);
					// if we are reading the time value, push it in a vector as a double
					if (elementIndex == 0) {
						times.push_back(stod(token));
					} // or if we are reading a quaternion
					else if (elementIndex > 0) {
						// parse quaternion and form a quaternion, then push it to a vector
						parseQuaternion(quaternions, textLine);
					}

					//std::cout << "Text line pre-erase: " << std::endl << textLine << std::endl << std::endl;
					textLine.erase(0, position + delimiter.length());
					//std::cout << "Text line post-erase: " << std::endl << textLine << std::endl << std::endl;
					++elementIndex;

				}
				// push a quaternion vector (all quaternions from a single time point) to a vector of quaternion vectors
				//std::cout << quaternions.size() << std::endl;
				quaternionVector.push_back(quaternions);
			}
		}
	}

	// finish reading file once labels, time vector and quaternions are obtained
	quatFile.close();
	std::cout << " done." << std::endl;

	// THIS IS NOT NECESSARY IF WE JUST RENAME THE LABELS IN QUATERNIONTIMESERIESDELSYS.TXT TO "pelvis_imu" etc instead of "Quaternion1"
	// match labels to IMU labels according to IMU placer settings
	/*std::vector<std::string> IMULabels(labels);
	for (unsigned int i = 0; i < IMULabels.size(); ++i) {
		IMULabels[i] = quaternionIndexToIMULabel(labels[i]);
	}*/


	std::cout << "Creating a quaternion matrix..." << std::endl;
	// turn quaternion vector into a quaternion matrix
	SimTK::Matrix_<SimTK::Quaternion> quatMatrix(quaternionVector.size(), quaternionVector[0].size());
	for (unsigned int i = 0; i < quaternionVector.size(); ++i) {
		for (unsigned int j = 0; j < quaternionVector[i].size(); ++j) {
			//std::cout << i << ", " << j << std::endl;
			//quatMatrix.set(i, j, quaternionVector[i][j]);
			quatMatrix[i][j] = quaternionVector[i][j];
		}
	}
	std::cout << " done." << std::endl;

	//std::cout << times.size() << ", " << quatMatrix.nrow() << ", " << quatMatrix[0].size() << ", " << labels.size() << std::endl;

	std::cout << "Creating a time series table..." << std::endl;
	try {
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(times, quatMatrix, labels);
		std::cout << quatTable << std::endl;
		std::cout << " done." << std::endl;
		return quatTable;
	}
	catch (std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 0;
	}
	catch (...) {
		std::cerr << "Unknown error!" << std::endl;
		return 0;
	}

}

// Function that is sent to be handled in a separate thread by ThreadPoolContainer
void concurrentIKThread(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, size_t i, bool visualize) {
	//IKTool.setTime(quatTable.getIndependentColumn()[i]);
	IKTool.updateOrdered(visualize, quatTable, i+1, quatTable.getIndependentColumn()[0], true);
}

// Calibrate model and return the name of the calibrated model file, and update calibTime to match the closest similar entry in the time series table.
std::string calibrateModel(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, double& calibTime)
{
	size_t calibIndex = quatTable.getNearestRowIndexForTime(calibTime);
	auto calibLabels = quatTable.getColumnLabels();
	auto calibMatrix = quatTable.updNearestRow(calibTime);
	//auto calibMatrix = quatTable.updDependentColumnAtIndex(calibIndex);
	//auto calibMatrix = quatTable.getDependentColumnAtIndex(calibIndex);
	double trueCalibTime = quatTable.getIndependentColumn()[calibIndex];
	std::cout << "Calibrating for time " << trueCalibTime << " found at row index " << calibIndex << "." << std::endl;
	std::cout << "Quaternion for calibration: " << std::endl << calibMatrix << std::endl;
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> calibQuatTable(std::vector<double>({ trueCalibTime }), calibMatrix.getAsMatrix(), calibLabels);
	// calibrate model
	std::cout << "Calibrating..." << std::endl;
	std::string calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), calibQuatTable);
	std::cout << " done." << std::endl;

	calibTime = trueCalibTime;
	return calibratedModelFile;
}

// Remove rows that are not within the bounds of [startTime, endTime] in the time series table.
OpenSim::TimeSeriesTable_<SimTK::Quaternion> clipTable(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, double startTime, double endTime)
{
	// prepare clipped quaternion time series table
	size_t startIndex = quatTable.getNearestRowIndexForTime(startTime);
	size_t endIndex = quatTable.getNearestRowIndexForTime(endTime);

	// get the original number of rows in the time series table before removing any of them
	size_t numRowsOriginal = quatTable.getNumRows();

	std::cout << "Preparing to clip the time series table from " << numRowsOriginal << " rows to " << endIndex - startIndex + 1 << " rows." << std::endl;

	// clip the end of the quat table by removing rows right after the index of endTime
	for (size_t i = endIndex + 1; i < numRowsOriginal; ++i) {
		quatTable.removeRowAtIndex(endIndex + 1);
	}

	// clip the beginning of the quat table by removing rows at the beginning of the table
	for (size_t i = 0; i < startIndex; ++i) {
		quatTable.removeRowAtIndex(0);
	}

	std::cout << "Clipping done. Time series table now has " << quatTable.getNumRows() << " rows." << std::endl;

	return quatTable;
}

void performIK(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, std::string& calibratedModelFile, unsigned int nThreads, double calibTime, bool visualize)
{
	// do IK
	std::cout << "Performing inverse kinematics..." << std::endl;
	OpenSimLive::IMUInverseKinematicsToolLive IKTool;
	IKTool.setReportErrors(true);
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quatTable); // the orientations of IMUs
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.setPointTrackerEnabled(false);
	IKTool.setTime(calibTime);
	IKTool.setOutputFileName("IK-offline");
	IKTool.setOutputDataName("IK-offline");
	IKTool.run(visualize);
	try{
		// create a thread pool container with user-given maximum number of concurrent threads
		OpenSimLive::ThreadPoolContainer tpc(nThreads);
		// iterate through all rows (all time / data points) in the quaternion table
		std::cout << "Preparing to iterate through " << quatTable.getNumRows() << " rows." << std::endl;
		for (size_t i = 0; i < quatTable.getNumRows(); ++i) {
			double currentTime = quatTable.getIndependentColumn()[i];
			auto currentMatrix = quatTable.updNearestRow(currentTime);
			OpenSim::TimeSeriesTable_<SimTK::Quaternion> currentQuatTable(std::vector<double>({ currentTime }), currentMatrix.getAsMatrix(), quatTable.getColumnLabels());

			tpc.offerFuture(concurrentIKThread, std::ref(IKTool), currentQuatTable, i, visualize);
			std::cout << "Progress: " << i << "/" << quatTable.getNumRows() << "\r";
			std::cout.flush();
		}
		tpc.waitForFinish();
	}
	catch (std::exception& e) {
		std::cerr << "Error in offline IK tool: " << e.what() << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown error in offline IK tool!" << std::endl;
	}
	std::cout << " done." << std::endl;
	
	IKTool.reportToFile();

}

int main(int argc, char *argv[])
{

	std::string quatFileName;
	std::string inputThreadsStr;
	std::string calibTimeStr;
	std::string startTimeStr;
	std::string endTimeStr;

	if (argc == 6) {
		quatFileName = argv[1];
		inputThreadsStr = argv[2];
		calibTimeStr = argv[3];
		startTimeStr = argv[4];
		endTimeStr = argv[5];
	}
	else if (argc == 1) {

		std::cout << "Please input the name of the text file to be analyzed in /OpenSimLive-results/ folder: ";
		std::cin >> quatFileName;

		std::cout << "Please input the number of threads to be used: ";
		std::cin >> inputThreadsStr;


		std::cout << "Please input the time of IMU calibration: ";
		std::cin >> calibTimeStr;


		std::cout << "Please input IK start time: ";
		std::cin >> startTimeStr;


		std::cout << "Please input IK end time: ";
		std::cin >> endTimeStr;
		
	}
	else {
		std::cout << "Give command line in the following format: offline_IK_tool.exe quaternionTimeSeriesFileName.txt number_of_IK_threads calib_time IK_start_time IK_end_time" << std::endl;
		std::cout << "Alternatively, do not give any command line arguments and the program will let you input them manually." << std::endl;
	}

	int inputThreads = stoi(inputThreadsStr);
	double calibTime = stod(calibTimeStr);
	double startTime = stod(startTimeStr);
	double endTime = stod(endTimeStr);

	bool visualize = false;

	// function that constructs a time series table of quaternions from quaternion time series text file
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable = quaternionTableFromTextFile(quatFileName);
	// function that calibrates the model and updates calibTime to match the closest similar time in the time series table
	std::string calibratedModelFile = calibrateModel(quatTable, calibTime);
	// function that clips the time series table for IK
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> clippedTable = clipTable(quatTable, startTime, endTime);
	// function that performs IK
	performIK(clippedTable, calibratedModelFile, inputThreads, calibTime, visualize);

	std::cout << "Program finished." << std::endl;
	return 1;
}
