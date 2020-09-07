#include <PointTracker.h>
#include <OpenSim.h>
#include <vector>

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
std::vector<double> PointTracker::runTracker(const SimTK::State* s, OpenSim::Model* model, const std::string& bodyName, const std::string& referenceBodyName) {

	// Get the location of the station in its parent body's reference frame
	SimTK::Vec3 stationLocationLocal(findStationLocationInLocalFrame(model, bodyName));
		
	// Get a pointer to the body the station is located on
	OpenSim::Body* body = &(model->updBodySet().get(bodyName));
	// Get a pointer to the body we want to use as the new reference frame for the station's location
	OpenSim::Body* referenceBody = &(model->updBodySet().get(referenceBodyName));
	// Calculate the rotation of the station's original parent frame (body) and mirror those orientations with respect to an axis.
	SimTK::Vec3 mirroredEuler = calculatePointRotation(s, model, 2, body, referenceBody);
		
	// Calculate the location of the station in another body's reference frame
	SimTK::Vec3 pointLocation = calculatePointLocation(stationLocationLocal, *s, body, referenceBody);
	// Reflect the location of the station in another body's reference frame with respect to an axis
	reflectWithRespectToAxis(pointLocation, 2); // 0 for x, 1 for y, 2 for z

	//std::cout << "Original point location in reference frame: " << pointLocation << std::endl;
	//std::cout << "Mirrored point location in reference frame: " << reflectedPointLocation << std::endl;

	// calculate and set transform for decoration generator
	if (visualize_)
	{
		// position for sphere
		SimTK::Transform_<SimTK::Real> mirroredTransform({ pointLocation[0], pointLocation[1], pointLocation[2] });
		decGen_->setTransformInReferenceBody(mirroredTransform);
	}

	// Save the calculated results in a vector and return it
	std::vector<double> positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2] };
	
	if (savePointTrackerResults_) {
		timeSeriesTimeVector_.push_back(timeSeriesCurrentTime_);
		timeSeriesDepData_.push_back(positionsAndRotations);
	}

	return positionsAndRotations;
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
SimTK::Vec3 PointTracker::calculatePointRotation(const SimTK::State* s, OpenSim::Model* model, const int axisIndex, const OpenSim::Body* body, const OpenSim::Body* referenceBody) {
	// Get the rotation of the parent body in ground reference frame
	SimTK::Rotation mirroringBodyRotation = body->getRotationInGround(*s);
	SimTK::Rotation referenceBodyRotation = referenceBody->getRotationInGround(*s);
	// calculate the rotation of the body with respect to the reference body's coordinate system
	SimTK::Rotation bodyWrtRefBodyRot = referenceBodyRotation.invert() * mirroringBodyRotation;
	// create a rotation of 180 degrees around a suitable axis, then transpose/invert it (same thing because we are working with rotation matrices)
	SimTK::Rotation mirroringRotation;
	mirroringRotation.setRotationFromAngleAboutAxis(3.14159265358979323, SimTK::CoordinateAxis(axisIndex)).transpose();
	// rotate the rotation of the body w.r.t. reference body coordinate system by the rotation we created
	SimTK::Rotation mirroredRotation = mirroringRotation * bodyWrtRefBodyRot;
	// Now we have calculated the mirrored rotation w.r.t. coordinates of the reference body.

	// rotation for arrow
	SimTK::Transform_<SimTK::Real> mirroredArrowDirection(mirroredRotation);
	decGen_->setArrowDirection(mirroredArrowDirection);

	SimTK::Rotation mirroredRotationWrtKuka;
	// we must rotate the OpenSim coordinate system -90 degrees about X to match the coordinate axes with the KUKA coordinate system
	SimTK::Rotation deg90AboutX;
	deg90AboutX.setRotationFromAngleAboutX(-1.570796326794897);
	if (getReferenceBaseRotation().isNaN()) // if reference base rotation has not been defined
	{
		// we assume that KUKA coordinate system is facing opposite the station reference body's coordinate system
		SimTK::Rotation deg180AboutZ;
		deg180AboutZ.setRotationFromAngleAboutZ(3.14159265358979323);
		mirroredRotationWrtKuka = deg180AboutZ*(deg90AboutX*mirroredRotation);
	}
	else
	{
		// get quaternion orientations of the IMU in base and on station reference body as rotation matrices
		SimTK::Rotation referenceBaseRotation(getReferenceBaseRotation());
		SimTK::Rotation referenceBodyRotation(getReferenceBodyRotation());
		// calculate the rotation from the orientation of the IMU on the station reference body to the orientation of the IMU on the base of the robot arm
		// X * BODY = BASE
		// X = BASE * ~BODY
		SimTK::Rotation bodyToBase(referenceBaseRotation * referenceBodyRotation.invert());
		// rotate mirroredRotation to correct for the difference between orientation on the base of the robot arm and current orientation of the station reference body (and 90 degrees to match OpenSim coordinate system to KUKA)
		mirroredRotationWrtKuka = bodyToBase * (deg90AboutX*mirroredRotation);
	}

	// convert the 3x3 rotation matrix into body fixed XYZ euler angles
	//return mirroredRotation.convertRotationToBodyFixedXYZ();
	//return mirroredRotation.convertThreeAxesRotationToThreeAngles(SimTK::BodyRotationSequence, SimTK::XAxis, SimTK::YAxis, SimTK::ZAxis);
	return mirroredRotationWrtKuka.convertThreeAxesRotationToThreeAngles(SimTK::BodyOrSpaceType::BodyRotationSequence, SimTK::ZAxis, SimTK::YAxis, SimTK::XAxis);
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

// This function creates a TimeSeriesTable that contains the time points and the corresponding PointTracker outputs (that are broadcasted to client), and saves that TimeSeriesTable to file for later examination.
void PointTracker::savePointTrackerOutputToFile(std::string& rootDir, std::string& resultsDir) {
	// name the column labels
	std::vector<std::string> labels = { "pos_X", "pos_Y", "pos_Z", "Eul_X", "Eul_Y", "Eul_Z" };
	// create a matrix of appropriate size
	SimTK::Matrix_<SimTK::Real> timeSeriesMatrix(timeSeriesDepData_.size(), 6);
	// fill the matrix with values (this is likely slow because we're using vectors, but that might not matter because this is done at the end of the program, not during IK)
	for (unsigned int i = 0; i < timeSeriesDepData_.size(); ++i) { // iteration through rows
		for (unsigned int j = 0; j < 6; ++j) { // iteration throughs columns
			timeSeriesMatrix.set(i, j, timeSeriesDepData_.at(i).at(j));
		}
	}
	// construct the TimeSeriesTable
	OpenSim::TimeSeriesTable_<double> outputTimeSeries(timeSeriesTimeVector_, timeSeriesMatrix, labels);
	// write it to file as PointTrackerOutput.sto
	OpenSim::STOFileAdapter_<double>::write(outputTimeSeries, rootDir + "/" + resultsDir + "/" + "PointTrackerOutput.sto");
}



