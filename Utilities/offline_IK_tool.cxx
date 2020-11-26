#include <OpenSimLiveConfig.h>
#include <OpenSim.h>
#include <IMUPlacerLive.h>
#include <IMUInverseKinematicsToolLive.h>
//#include <ThreadPoolContainer.h>
#include <XMLFunctions.h>
//#include <future>
//#include <functional>
#include <fstream>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

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

void IKFromTextFile(std::string quatFileName, unsigned int nThreads) {
	
	std::cout << "Starting to process the file...";
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
					size_t startPos = labels[i].find_first_not_of(" \n\r\t\f\v");
					labels[i] = labels[i].substr(startPos);
					size_t endPos = labels[i].find_last_not_of(" \n\r\t\f\v");
					labels[i] = labels[i].substr(0, endPos);
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


	std::cout << "Creating a quaternion matrix...";
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

	std::cout << "Creating a time series table...";
	try {
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(times, quatMatrix, labels);
		std::cout << quatTable << std::endl;
		std::cout << " done." << std::endl;
	}
	catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}
	catch (...) {
		std::cerr << "Error!" << std::endl;
	}
	
	// now do IK



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

	
	IKFromTextFile(quatFileName, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
