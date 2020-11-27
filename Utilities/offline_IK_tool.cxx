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
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(times, quatMatrix, labels);
	std::cout << quatTable << std::endl;
	std::cout << " done." << std::endl;
	return quatTable;
}

// Function that is sent to be handled in a separate thread by ThreadPoolContainer
void concurrentIKThread(OpenSimLive::IMUInverseKinematicsToolLive& IKTool, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, size_t i) {
	IKTool.setTime(quatTable.getIndependentColumn()[i]);
	IKTool.update(true, true);
}

void calibrateAndPerformIK(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, unsigned int nThreads)
{
	// calibrate model
	std::cout << "Calibrating..." << std::endl;
	std::string calibratedModelFile = calibrateModelFromSetupFile(OPENSIMLIVE_ROOT + "/Config/" + ConfigReader("MainConfiguration.xml", "imu_placer_setup_file"), quatTable);
	std::cout << " done." << std::endl;

	// do IK
	std::cout << "Performing inverse kinematics..." << std::endl;
	OpenSimLive::IMUInverseKinematicsToolLive IKTool;
	IKTool.setModelFile(calibratedModelFile); // the model to perform IK on
	IKTool.setQuaternion(quatTable); // the orientations of IMUs
	SimTK::Vec3 sensorToOpenSimRotations = get_sensor_to_opensim_rotations();
	IKTool.setSensorToOpenSimRotations(sensorToOpenSimRotations);
	IKTool.setOpenSimLiveRootDirectory(OPENSIMLIVE_ROOT); // this is needed for saving the IK report to file
	IKTool.setPointTrackerEnabled(false);
	IKTool.setTime(quatTable.getIndependentColumn()[0]);
	IKTool.run(true);
	// create a thread pool container with user-given maximum number of concurrent threads
	OpenSimLive::ThreadPoolContainer tpc(nThreads);
	// iterate through all rows (all time / data points) in the quaternion table
	for (size_t i = 0; i < quatTable.getNumRows(); ++i) {
		tpc.offerFuture(concurrentIKThread, std::ref(IKTool), std::ref(quatTable), i);
	}
	std::cout << " done." << std::endl;
	


}

int main(int argc, char *argv[])
{
	std::string quatFileName;
	std::cout << "Please input the name of the text file to be analyzed in /OpenSimLive-results/ folder: ";
	std::cin >> quatFileName;

	std::string inputThreadsStr;
	std::cout << "Please input the number of threads to be used: ";
	std::cin >> inputThreadsStr;
	int inputThreads = stoi(inputThreadsStr);

	// function that constructs a time series table of quaternions from quaternion time series text file
	OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable = quaternionTableFromTextFile(quatFileName);
	// function that calibrates the model and performs IK
	calibrateAndPerformIK(quatTable, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
