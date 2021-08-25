#include <PointTracker.h>
#include <OpenSim.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>

using namespace OpenSimLive;

PointTracker::PointTracker() {}

PointTracker::PointTracker(SimTK::State state, std::string modelFileName) {
	//setState(state);
	//setModelByFileName(modelFileName);
}

PointTracker::PointTracker(SimTK::State state, std::string modelFileName, std::string bodyName) {
	//setState(state);
	//setModelByFileName(modelFileName);
	//setBodyName(bodyName);
}

PointTracker::~PointTracker() {
}

// This function performs all the necessary calculations to fetch the local position of the station, calculate it in another reference frame (body), mirror the position in that new reference frame, get the original body's orientation, mirror the orientation with respect to an axis and finally return a 6-element vector with mirrored positions and orientations.
std::array<double, 7> PointTracker::runTracker(const SimTK::State* s, OpenSim::Model* model, const std::string& bodyName, const std::string& referenceBodyName) {

	// if we use L-correction, calculate body-to-base rotation matrix
	if (useReferenceRotation_) {
		calculateBodyToBaseMatrix();
	}

	// Get the location of the station in its parent body's reference frame
	SimTK::Vec3 stationLocationLocal(findStationLocationInLocalFrame(model, bodyName));
		
	// Get a pointer to the body the station is located on
	OpenSim::Body* body = &(model->updBodySet().get(bodyName));
	// Get a pointer to the body we want to use as the new reference frame for the station's location
	OpenSim::Body* referenceBody = &(model->updBodySet().get(referenceBodyName));
	// Calculate the rotation of the station's original parent frame (body) and mirror those orientations with respect to an axis.
	SimTK::Rotation mirroredRotation = calculatePointRotation(s, model, body, referenceBody);
	
	// Calculate the location of the station in another body's reference frame
	SimTK::Vec3 pointLocation = calculatePointLocation(stationLocationLocal, *s, body, referenceBody);
	// Reflect the location of the station in another body's reference frame with respect to an axis
	reflectWithRespectToAxis(pointLocation, 2); // 0 for x, 1 for y, 2 for z

	// calculate and set transform for decoration generator
	if (visualize_)
	{
		// position for sphere
		SimTK::Transform_<SimTK::Real> mirroredTransform(mirroredRotation_, { pointLocation[0], pointLocation[1], pointLocation[2] });
		decGen_->setTransformInReferenceBody(mirroredTransform);		
	}

	// rotate points after feeding them to the visualization
	if (useReferenceRotation_) {
		std::cout << "Mirrored point location in OpenSim coordinate system before L-correction: " << pointLocation << std::endl;
		pointLocation = bodyToBase_ * pointLocation;
		std::cout << "Mirrored point location in OpenSim coordinate system after L-correction: " << pointLocation << std::endl;
	}

	std::array<double, 7> positionsAndRotations;

	if (outputFormat_ == EULER) {
		// get mirrored rotation as Euler angles
		SimTK::Vec3 mirroredEuler = mirroredRotation.convertThreeAxesRotationToThreeAngles(SimTK::BodyOrSpaceType::BodyRotationSequence, SimTK::ZAxis, SimTK::YAxis, SimTK::XAxis);
		positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2], 0 };
	}
	else if (outputFormat_ == QUATERNION) {
		SimTK::Quaternion mirroredQuaternion = mirroredRotation.convertRotationToQuaternion();
		positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredQuaternion[0], mirroredQuaternion[1], mirroredQuaternion[2], mirroredQuaternion[3] };
	}
		

	// Save the calculated results in a double array and return it
	//std::vector<double> positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2] };
	
	
	if (savePointTrackerResults_) {
		timeSeriesTimeVector_.push_back(timeSeriesCurrentTime_);
		timeSeriesDepData_.push_back(positionsAndRotations);
	}

	return positionsAndRotations;
}

// This function calculates the body-to-base rotation matrix for L-correction.
void PointTracker::calculateBodyToBaseMatrix() {
	// get quaternion orientations of the IMU in base and on station reference body as rotation matrices
	SimTK::Rotation referenceBaseRotation(referenceBaseRotation_);
	std::cout << "Reference base rotation: " << referenceBaseRotation << std::endl;
	SimTK::Rotation referenceBodyRotation(referenceBodyRotation_);
	std::cout << "Reference body rotation: " << referenceBodyRotation << std::endl;
	// calculate the rotation from the orientation of the IMU on the station reference body to the orientation of the IMU on the base of the robot arm
	// X * BODY = BASE
	// X = BASE * ~BODY
	bodyToBase_ = referenceBaseRotation * referenceBodyRotation.invert();
	std::cout << "Body to base rotation matrix: " << bodyToBase_ << std::endl;
}

// This function finds the value of station's XML attribute location and returns it as a SimTK::Vec3 vector
SimTK::Vec3 PointTracker::findStationLocationInLocalFrame(OpenSim::Model* model, const std::string& bodyName) {
	// Load the calibrated model file as an XML document
	SimTK::Xml::Document modelXML(model->getInputFileName());
	// Get all XML elements that represent bodies in the model
	SimTK::Array_<SimTK::Xml::Element> bodyElements = modelXML.getRootElement().getRequiredElement("Model").getRequiredElement("BodySet").getRequiredElement("objects").getAllElements("Body");
	// String to read the location of the station into
	std::string locationString;
	// Iterate through all body XML elements
	for (unsigned int i = 0; i < bodyElements.size(); ++i) {
		// If we find the right element
		if (bodyElements[i].getRequiredAttributeValue("name") == bodyName)
		{
			// Save the location value into locationString
			locationString = bodyElements[i].getRequiredElement("Station").getRequiredElementValue("location");
			break; // Stop iterating any further
		}
	}
	// If we didn't find the right element
	if (locationString.empty()) {
		std::cerr << "Error! Location string was empty!";
	}
	// Parse individual words (location coordinates as strings) from locationString and convert them to double with std::stod
	std::stringstream ss(locationString);
	std::string word;
	SimTK::Vec3 locationVector = { 0,0,0 };
	for (int j = 0; j < 3; ++j) {
		ss >> word;
		locationVector[j] = std::stod(word);
	}
	// Return the location as a SimTK::Vec3 vector
	return locationVector;
}

// This function calculates the location of the point in another body's reference frame
SimTK::Vec3 PointTracker::calculatePointLocation(const SimTK::Vec3& localLocation, const SimTK::State& s, const OpenSim::Body* body, const OpenSim::Body* referenceBody) {
	return body->findStationLocationInAnotherFrame(s, localLocation, *referenceBody);
}

// This function calculates the rotation of the station by getting the rotation of its parent frame (body) and mirroring it with respect to the sagittal plane
SimTK::Rotation PointTracker::calculatePointRotation(const SimTK::State* s, OpenSim::Model* model, const OpenSim::Body* body, const OpenSim::Body* referenceBody) {
	// Get the rotation of the parent body in ground reference frame
	SimTK::Rotation mirroringBodyRotation = body->getRotationInGround(*s);
	SimTK::Rotation referenceBodyRotation = referenceBody->getRotationInGround(*s);
	// calculate the rotation of the body with respect to the reference body's coordinate system
	SimTK::Rotation bodyWrtRefBodyRot = referenceBodyRotation.invert() * mirroringBodyRotation;
	// create a new rotation that we mirror w.r.t. the xy-plane of OpenSim coordinate system
	SimTK::Rotation mirroredRotation = bodyWrtRefBodyRot;
	mirroredRotation.set(2, 0, -bodyWrtRefBodyRot.get(2, 0));
	mirroredRotation.set(2, 1, -bodyWrtRefBodyRot.get(2, 1));
	mirroredRotation.set(2, 2, -bodyWrtRefBodyRot.get(2, 2));
	// Now we have calculated the mirrored rotation w.r.t. coordinates of the reference body.

	// set rotation for line in decoration generator
	mirroredRotation_ = mirroredRotation;

	// Now that we have the mirrored rotation w.r.t. to the OpenSim coordinate system, we have to rotate it so that it's in the KUKA coordinate system
	// create a rotation matrix to hold the rotation in KUKA coordinate system
	SimTK::Rotation mirroredRotationWrtKuka;
	// we must rotate the OpenSim coordinate system -90 degrees about X to match the coordinate axes with the KUKA coordinate system
	SimTK::Rotation deg90AboutX;
	deg90AboutX.setRotationFromAngleAboutX(-1.570796326794897);
	
	if (useReferenceRotation_) // if reference base rotation has been defined
	{
		// rotate mirroredRotation to correct for the difference between orientation on the base of the robot arm and current orientation of the station reference body (and 90 degrees to match OpenSim coordinate system to KUKA)
		mirroredRotationWrtKuka = bodyToBase_ * (deg90AboutX*mirroredRotation);
	}
	else // if reference base rotation has not been defined
	{
		// we assume that KUKA coordinate system is facing opposite the station reference body's coordinate system (180 degrees about the vertical axis)
		SimTK::Rotation deg180AboutZ;
		deg180AboutZ.setRotationFromAngleAboutZ(3.14159265358979323);
		mirroredRotationWrtKuka = deg180AboutZ * (deg90AboutX * mirroredRotation);
	}

	// convert the 3x3 rotation matrix into body fixed ZYX euler angles
	return mirroredRotationWrtKuka;//.convertThreeAxesRotationToThreeAngles(SimTK::BodyOrSpaceType::BodyRotationSequence, SimTK::ZAxis, SimTK::YAxis, SimTK::XAxis);
}

// This function reflects a point with respect to an axis by multiplying the element corresponding to that axis by -1
void PointTracker::reflectWithRespectToAxis(SimTK::Vec3& pointLocation, const int axisIndex) {
	// Change the point's coordinates in its local coordinate system by multiplying one of the coordinates by -1
	pointLocation[axisIndex] = -1 * pointLocation[axisIndex];
}

// This function takes a calibrated model file, creates a station element in under the desired body and then overwrites the .osim file
void PointTracker::addStationToBody(const std::string& bodyName, const SimTK::Vec3& pointLocation, const std::string& modelFile) {
	
	// Create the station element and its child elements and link them together
	SimTK::Xml::Element stationElement("Station");
	stationElement.setAttributeValue("name", "mirror_station");
	SimTK::Xml::Element locationElement("location", std::to_string(pointLocation[0]) + " "+ std::to_string(pointLocation[1]) + " " + std::to_string(pointLocation[2]));
	SimTK::Xml::Element parentFrameElement("parent_frame", bodyName);
	stationElement.appendNode(locationElement);
	stationElement.appendNode(parentFrameElement);

	// Create a Sphere element and its child elements and link them together
	SimTK::Xml::Element sphereComponentsElement("components");
	SimTK::Xml::Element spherePOFElement("PhysicalOffsetFrame");
	spherePOFElement.setAttributeValue("name", "offset_station");
	SimTK::Xml::Element sphereFGElement("FrameGeometry");
	sphereFGElement.setAttributeValue("name", "frame_geometry");
	SimTK::Xml::Element sphereFGSocketFrameElement("socket_frame", "..");
	SimTK::Xml::Element sphereFGScaleFactorsElement("scale_factors", "1 1 1");
	sphereFGElement.appendNode(sphereFGSocketFrameElement);
	sphereFGElement.appendNode(sphereFGScaleFactorsElement);
	SimTK::Xml::Element sphereAGElement("attached_geometry");
	SimTK::Xml::Element sphereElement("Sphere");
	sphereElement.setAttributeValue("name", "sphere_station");
	SimTK::Xml::Element sphereSocketFrameElement("socket_frame", "..");
	SimTK::Xml::Element sphereAppearanceElement("Appearance");
	SimTK::Xml::Element sphereColorElement("color", "1 0 0");
	sphereAppearanceElement.appendNode(sphereColorElement);
	SimTK::Xml::Element sphereRadiusElement("radius", "0.02");
	sphereElement.appendNode(sphereSocketFrameElement);
	sphereElement.appendNode(sphereAppearanceElement);
	sphereElement.appendNode(sphereRadiusElement);
	sphereAGElement.appendNode(sphereElement);
	SimTK::Xml::Element sphereSocketParentElement("socket_parent", "..");
	SimTK::Xml::Element sphereTranslationElement("translation", std::to_string(pointLocation[0]) + " " + std::to_string(pointLocation[1]) + " " + std::to_string(pointLocation[2]));
	SimTK::Xml::Element sphereOrientationElement("orientation", "0 0 0");
	spherePOFElement.appendNode(sphereFGElement);
	spherePOFElement.appendNode(sphereAGElement);
	spherePOFElement.appendNode(sphereSocketParentElement);
	spherePOFElement.appendNode(sphereTranslationElement);
	spherePOFElement.appendNode(sphereOrientationElement);
	sphereComponentsElement.appendNode(spherePOFElement);

	// open the .osim model as an XML document
	SimTK::Xml::Document calibratedModelFile(modelFile);
	// get the root element of the XML file, then its child elements etc
	SimTK::Xml::Element rootElement = calibratedModelFile.getRootElement();
	SimTK::Xml::Element modelElement = rootElement.getRequiredElement("Model");
	SimTK::Xml::Element bodySetElement = modelElement.getRequiredElement("BodySet");
	SimTK::Xml::Element objectsElement = bodySetElement.getRequiredElement("objects");
	// get all child elements with tag <Body>
	SimTK::Array_<SimTK::Xml::Element> bodyElements = objectsElement.getAllElements("Body");
	// get the number of bodies
	int numberOfBodies = bodyElements.size();

	// Iterate through all bodies
	for (int i = 0; i < numberOfBodies; ++i) {
		// If we find the desired body, append stationElement and sphereComponentsElement to it
		if (bodyElements.at(i).getRequiredAttributeValue("name") == bodyName)
		{
			bodyElements[i].appendNode(sphereComponentsElement);
			bodyElements[i].appendNode(stationElement);
		}
	}

	// Write the modified file into .osim
	calibratedModelFile.writeToFile(modelFile);
}

// This function saves the time points and the corresponding PointTracker outputs (that may be broadcasted to client) to file for later examination.
void PointTracker::savePointTrackerOutputToFile(std::string& rootDir, std::string& resultsDir) {
	
	std::string filePath(rootDir + "/" + resultsDir + "/" + "PointTrackerOutput.txt");
	std::ofstream outputFile;
	// open and set file to discard any contents that existed in the file previously (truncate mode)
	outputFile.open(filePath, std::ios_base::out|std::ios_base::trunc);
	if (outputFile.is_open())
	{
		outputFile << "Time series of PointTracker output that may have been sent to client:\n";
		outputFile << "Time (s)\tPos_X (m)\tPos_Y (m)\tPos_Z (m)\tEuler_X (rad)\tEuler_Y (rad)\tEuler_Z (rad)";

		for (unsigned int i = 0; i < timeSeriesDepData_.size(); ++i) { // iteration through rows
			outputFile << "\n" << timeSeriesTimeVector_[i] << "\t" << timeSeriesDepData_[i][0] << "\t" << timeSeriesDepData_[i][1] << "\t" << timeSeriesDepData_[i][2] << "\t" << timeSeriesDepData_[i][3] << "\t" << timeSeriesDepData_[i][4] << "\t" << timeSeriesDepData_[i][5];
		}
		outputFile.close();
		std::cout << "PointTracker output written to file " << filePath << std::endl;
	}
	else {
		std::cout << "Failed to open file " << filePath << std::endl;
	}
}





