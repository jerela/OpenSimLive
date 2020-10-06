// main.cpp

#include <DelsysDataReader.h>
#include <string>
#include <iostream>
#include <Client.h>
#include <conio.h> // for kbhit
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <OpenSim.h>
#include <XMLFunctions.h>
#include <memory> // for std::unique_ptr
#include <map>


using namespace OpenSimLive;

// CONSTRUCTOR
DelsysDataReader::DelsysDataReader() {
	// read labels from the mappings file
	labels_ = getLabelsFromFile();

	// get number of active sensors and their indices (1-16) from the mappings file
	nActiveSensors_ = std::stoi(ConfigReader("DelsysMappings.xml", "number_of_active_sensors"));
	std::string activeSensorNumbersString = ConfigReader("DelsysMappings.xml", "active_sensors");
	std::stringstream ss(activeSensorNumbersString);
	// perhaps this loop could be implemented by checking when stringstream has reached its end rather than separately reading the number of sensors, which is heavier in terms of performance?
	for (unsigned int i = 0; i < nActiveSensors_; ++i) {
		std::string tempSensorNumber;
		ss >> tempSensorNumber;
		activeSensors_.push_back(std::stoi(tempSensorNumber));
	}
}

// DESTRUCTOR
DelsysDataReader::~DelsysDataReader() {
/*	if (commandPort_)
		delete commandPort_;
	if (AUXPort_)
		delete AUXPort_;
	if (quatTable_)
		delete quatTable_;*/
}

// this union is used to convert bytes to float; all its data members share a memory location, meaning that the byte array we save into it can be accessed as a float
union DelsysDataReader::byteFloater {
	float f;
	unsigned char c[0];
};

// Takes four bytes and whether to reverse byte order or not as an input and returns a float.
float DelsysDataReader::convertBytesToFloat(char b1, char b2, char b3, char b4, int rev) {
	byteFloater bytesToFloat;
	if (rev == 0) {
		bytesToFloat.c[0] = b1;
		bytesToFloat.c[1] = b2;
		bytesToFloat.c[2] = b3;
		bytesToFloat.c[3] = b4;
	}
	if (rev == 1) {
		bytesToFloat.c[0] = b4;
		bytesToFloat.c[1] = b3;
		bytesToFloat.c[2] = b2;
		bytesToFloat.c[3] = b1;
	}
	return bytesToFloat.f;
}



// Read all 16 IMU labels from mappings file into a vector.
std::vector<std::string> DelsysDataReader::getLabelsFromFile() {
	// declare the vector of string labels
	std::vector<std::string> labels;
	// iterate through all integers in the sensorIndices vector
	for (unsigned int i = 1; i < 17; ++i) {
		labels.push_back(ConfigReader("DelsysMappings.xml", "sensor_" + std::to_string(i) + "_label"));
	}
	return labels;
}



// Takes the number indices of found sensors as input and returns the labels of corresponding IMUs on the skeletal model.
std::vector<std::string> DelsysDataReader::getSegmentLabelsForNumberLabels(std::vector<unsigned int> sensorIndices, unsigned int offset) {
	// declare the vector of string labels
	std::vector<std::string> labels;
	// iterate through all integers in the sensorIndices vector
	for (auto i : sensorIndices) {
		//std::cout << "i: " << i << std::endl;
		// define j as i + the required offset
		//int j = i + offset;
		//std::cout << "j: " << j << std::endl;
		// if we go over bounds, reduce by 16
		//while (j > nActiveSensors_)
		//	j = j - nActiveSensors_;
		//std::cout << "j: " << j << std::endl;
		//j = j - 1;
		labels.push_back(labels_[i-1]);
	}
	return labels;
}








// This function calculates an index offset to correct the indices of sensors. THIS RELIES ON THE ASSUMPTION THAT SENSOR INDICES ARE ASYMMETRIC; MUST BE FIXED LATER
unsigned int DelsysDataReader::correctSensorIndex(const std::vector<unsigned int>& sensorLabels, std::vector<unsigned int>& sensorIndices) {

	// make sure sensorIndices is in ascending order
	std::sort(sensorIndices.begin(), sensorIndices.end());

	unsigned int numberOfIndices = sensorIndices.size();
	unsigned int offset = 0;
	while (sensorIndices != sensorLabels) {
		++offset;
		// iterate through all elements in sensorIndices
		for (unsigned int i = 0; i < numberOfIndices; ++i) {
			// append +1 to sensorIndices
			sensorIndices[i] = sensorIndices[i] + 1;
			// turn any 17s into 1s
			if (sensorIndices[i] == 17)
				sensorIndices[i] = 1;
			// sort sensorindices
			//std::cout << "Element " << std::to_string(i) << " before sorting: " << sensorIndices[i] << std::endl;
			std::sort(sensorIndices.begin(), sensorIndices.end());
			//std::cout << "Element " << std::to_string(i) << " after sorting: " << sensorIndices[i] << std::endl;
		}
		// if offset is 17, we've gone through the whole "round" and something is wrong
		if (offset == 17)
			break;
	}
	//std::cout << "Final offset: " << offset << std::endl;
	return offset;
}


// This function connects to and commands Delsys SDK to start streaming data.
bool DelsysDataReader::initiateConnection() {
	// SOCKET COMMUNICATION
	bool bResult = false;
	const char* ipc = "10.139.24.243";
	int reverse = 0;
	int dataport = -1; // datagram port, not in use
	// see SDK user's guide page 6/7 about ports
	//Client commandPort(50040, dataport, ipc, reverse, &bResult);
	//Client EMGPort(50043, dataport, ipc, reverse, &bResult);
	//Client AUXPort(50044, dataport, ipc, reverse, &bResult);
	//commandPort_ = new Client(50040, dataport, ipc, reverse, &bResult);
	commandPort_ = std::make_unique<Client>(50040, dataport, ipc, reverse, &bResult);
	//AUXPort_ = new Client(50044, dataport, ipc, reverse, &bResult);
	AUXPort_ = std::make_unique<Client>(50044, dataport, ipc, reverse, &bResult);

	commandPort_->SendString("\r\n\r\n");
	commandPort_->SendString("BACKWARDS COMPATIBILITY ON\r\n");
	commandPort_->SendString("\r\n\r\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	commandPort_->SendString("UPSAMPLE ON\r\n");
	commandPort_->SendString("\r\n\r\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	commandPort_->SendString("START\r\n");
	commandPort_->SendString("\r\n\r\n");
	//std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	return 1;
}


// This function commands Delsys SDK to stop streaming data.
bool DelsysDataReader::closeConnection() {
	commandPort_->SendString("STOP\r\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	commandPort_->SendString("\r\n\r\n");
	//delete commandPort_;
	//delete AUXPort_;
	return 1;
}


// This function reads data from Delsys SDK and calculates a time series table of quaternions based on it.
void DelsysDataReader::updateQuaternionData()
{

	int reverse = 0;


	// numbers of elements between starting bytes of consecutive sensors
	unsigned int dataGap = 36;

	// initialize array for holding bytes that are read from Trigno SDK
	char receivedBytes[6400];

	bool success = false;
	do {
		// returns 1 if bytes were successfully read
		success = AUXPort_->RecvBytes(receivedBytes, 6400);

		if (success)
		{
			// dictionary/map of quaternions
			std::map<int, SimTK::Quaternion_<SimTK::Real>> quatMap;
			// vector that contains the indices that nonzero byte sequences begin for each quaternion
			std::vector<unsigned int> startIndices;
			// vector that contains the indices (with offset) of vectors that we recognize in the data
			std::vector<unsigned int> sensorIndices;
			// number of sensors recognized in the data
			unsigned int nDetectedSensors = 0;
			// number of consecutive nonzero byte values
			unsigned int streak = 0;
			// starting index of a streak
			int streakStartIndex = 0;
			// starting index of the first streak
			int firstStartIndex = -1;
			// iterate through whole data (6400 byte values)
			for (unsigned int k = 0; k < 6400; ++k) {

				// if the current byte value is nonzero, increment streak; otherwise set streak to zero
				if (receivedBytes[k] != 0) {
					if (streak == 0)
						streakStartIndex = k;
					++streak;
				}
				else {
					//if (streak > 0)
					//	std::cout << "Streak was " << std::to_string(streak) << " when it was reset." << std::endl;
					//if (streak == 0)
					//	std::cout << "Streak remains zero." << std::endl;
					streak = 0;
				}

				// if streak is 16, we assume that we just iterated through quaternion data (4 bytes each for 4 elements of the quaternion)
				if (streak == 16) {
					//std::cout << "Data found at start index " << std::to_string(streakStartIndex) << std::endl;

					// boolean telling us whether data from this sensor was read already in a previous segment of the byte stream
					bool dataAlreadyRead = false;

					// if streakStartIndex is not found in vector startIndices, push it in it
					if (std::find(startIndices.begin(), startIndices.end(), streakStartIndex) == startIndices.end())
						startIndices.push_back(streakStartIndex);

					// if firstStartIndex has not been set, set it to be streakStartIndex
					if (firstStartIndex == -1)
						firstStartIndex = streakStartIndex;

					// calculate the index (1-16) of the Delsys sensor with the assumption that the first quaternion that is read comes from sensor 1; this is not always true and thus we calculate an integer offset later
					int sensorIndex = ((streakStartIndex - firstStartIndex + dataGap) / dataGap) % 16;
					// modulus of 16/16 is 0, therefore if we get 0, we must make that 16
					if (sensorIndex == 0)
						sensorIndex = 16;
					// if sensorIndex is not found in vector sensorIndices, push it in it
					if (std::find(sensorIndices.begin(), sensorIndices.end(), sensorIndex) == sensorIndices.end())
					{
						//std::cout << "Pushing " << sensorIndex << " to sensorIndices." << std::endl;
						sensorIndices.push_back(sensorIndex);
					}
					else {
						//std::cout << "Data already read." << std::endl;
						dataAlreadyRead = true; // if it was already found, we have read data for this sensor previously in the same byte segment
					}

					// read data and store it as a quaternion only if it hasn't already been stored
					if (!dataAlreadyRead) {
						// array of 16 chars
						char dataBytes[16];
						// iterate to 16 and fill dataBytes with nonzero byte values
						for (unsigned int m = 0; m < 16; ++m) {
							dataBytes[m] = receivedBytes[streakStartIndex + m];
							//std::cout << "dataBytes[" << m << "] = " << (int)dataBytes[m] << std::endl;
						}
						// create a float array depicting a quaternion and populate it with floats that have been converted from bytes using convertBytesToFloat()
						float quaternion[4];
						quaternion[0] = convertBytesToFloat(dataBytes[0], dataBytes[1], dataBytes[2], dataBytes[3], reverse);
						quaternion[1] = convertBytesToFloat(dataBytes[4], dataBytes[5], dataBytes[6], dataBytes[7], reverse);
						quaternion[2] = convertBytesToFloat(dataBytes[8], dataBytes[9], dataBytes[10], dataBytes[11], reverse);
						quaternion[3] = convertBytesToFloat(dataBytes[12], dataBytes[13], dataBytes[14], dataBytes[15], reverse);
						// construct a quaternion out of the float array
						SimTK::Quaternion_<SimTK::Real> quat(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
						// place the quaternion in a dictionary/map
						quatMap[sensorIndex] = quat;

						//std::cout << "Quaternion for sensor " << std::to_string(sensorIndex) << ": [" << quaternion[0] << ", " << quaternion[1] << ", " << quaternion[2] << ", " << quaternion[3] << "]\n" << std::endl;
						nDetectedSensors++;
					}


				}
				// if nonzero byte values continue beyond 16 consequent values, something is wrong with the byte stream
				if (streak > 16) {
					std::cout << "WARNING: NONZERO DATA STREAK IS " << std::to_string(streak) << std::endl;
				}
			}

			if (nDetectedSensors != nActiveSensors_) {
				success = false;
			}
			else {
				//std::cout << "Number of unique quaternions read from received bytes: " << nDetectedSensors << "\n" << std::endl;

				// modify vector sensorIndices so that the same offset is applied to each of its elements, and the elements are sorted; this is done until sensorIndices == activeSensors_
				unsigned int offset = correctSensorIndex(activeSensors_, std::ref(sensorIndices));
				
				//std::cout << "Offset: " << offset << std::endl;

				// get labels in the order corrected by offset
				std::vector<std::string> labels = getSegmentLabelsForNumberLabels(sensorIndices, offset);
				// getSegmentLabelsForNumberLabels could optionally be called in the constructor, since we know activeSensors_; this would reduce stress on the loop; however then we should ensure that the number of elements in sensorIndices equals the number of elements in activeSensors_

				//std::cout << "Labels with " << labels.size() << " elements: " << labels[0] << ", " << labels[1] << ", " << labels[2] << std::endl;

				

				SimTK::Matrix_<SimTK::Quaternion> quatMatrix(1, nActiveSensors_);
				for (unsigned int m = 0; m < nActiveSensors_; ++m) {
					int x = m - offset + 1;
					if (x < 1)
						x = x + 16;
					quatMatrix.set(0, m, quatMap[x]);
				}

				/*for (unsigned int z = 0; z < nActiveSensors_; ++z) {
					std::cout << "Quaternion for " << labels[z] << ": " << quatMatrix.get(0,z) << std::endl;
				}*/

				std::vector<double> timeVector = { 0 };

				quatTable_ = std::make_unique<OpenSim::TimeSeriesTable_<SimTK::Quaternion>>(timeVector, quatMatrix, labels);
			}
			

		} // if statement for successful data retrieval ends


	} while (!success);
	//std::cout << "Finished loop" << std::endl;

}