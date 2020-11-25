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

void parseQuaternion(std::vector<SimTK::Quaternion>& quatVector, std::string& quatString) {
	std::vector<std::string> quatElementVector;
	parse(quatElementVector, quatString, ",");
	// remove ~[ from the beginning of the first element
	quatElementVector[0].erase(0, 2);
	// remove ] from the end of the last element
	quatElementVector[3] = quatElementVector[3].substr(0,quatElementVector[3].find("]"));
	// construct a quaternion
	SimTK::Quaternion_<SimTK::Real> quaternion(stod(quatElementVector[0]), stod(quatElementVector[1]), stod(quatElementVector[2]), stod(quatElementVector[3]));
	// push the quaternion in a vector
	quatVector.push_back(quaternion);
}

void IKFromTextFile(std::string quatFileName, unsigned int nThreads) {
	std::ifstream quatFile(OPENSIMLIVE_ROOT + "/OpenSimLive-results/" + quatFileName);
	std::string textLine;

	std::vector<std::string> labels;
	std::vector<double> times;
	std::vector<SimTK::Quaternion> quaternions;

	unsigned int nLine = 0;
	if (quatFile.is_open()) {
		while (std::getline(quatFile, textLine)) {
			++nLine;
			// if we are reading labels
			if (nLine == 2) {
				parse(labels, textLine, "\t");
			}
			// if we are past the description and labels
			if (nLine > 2) {
				size_t position = 0;
				std::string token;
				std::string delimiter("\t");
				bool readLine = true;
				unsigned int elementIndex = 0;
				readLine = ((position = textLine.find(delimiter)) != std::string::npos);
				while (readLine) {					
					token = textLine.substr(0, position);
					// if we are reading the time value
					if (elementIndex == 0) {
						times.push_back(stod(token));
					} // or if we are reading a quaternion
					else if (elementIndex > 0) {
						// parse quaternion and form a quaternion, then push it to a vector
						parseQuaternion(quaternions, textLine);
					}
					
					textLine.erase(0, position + delimiter.length());
					++elementIndex;
					readLine = ((position = textLine.find(delimiter)) != std::string::npos);
				}
			}
		}
	}

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

	std::cout << "Beginning to process the file..." << std::endl;
	IKFromTextFile(quatFileName, inputThreads);

	std::cout << "Program finished." << std::endl;
	return 1;
}
