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

PointTracker::~PointTracker() {}

// This function performs all the necessary calculations to fetch the local position of the station, calculate it in another reference frame (body), mirror the position in that new reference frame, get the original body's orientation, mirror the orientation with respect to an axis and finally return a 6-element vector with mirrored positions and orientations.
std::vector<double> PointTracker::runTracker(SimTK::State* s, OpenSim::Model* model, std::string bodyName, std::string referenceBodyName, SimTK::Vec3 pointLocalCoordinates) {
	// Get a pointer to the body the station is located on
	OpenSim::Body* body = &(model->updBodySet().get(bodyName));
	// Get a pointer to the body we want to use as the new reference frame for the station's location
	OpenSim::Body* referenceBody = &(model->updBodySet().get(referenceBodyName));
	// Get the location of the station in its parent body's reference frame
	SimTK::Vec3 stationLocationLocal(findStationLocationInLocalFrame(model, bodyName));
	// Calculate the location of the station in another body's reference frame
	SimTK::Vec3 pointLocation = calculatePointLocation(stationLocationLocal, s, body, referenceBody);
	// Calculate the rotation of the station's original parent frame (body) and mirror those orientations with respect to an axis.
	SimTK::Vec3 mirroredEuler = calculatePointRotation(s, model, 2, body, referenceBody);
	// Reflect the location of the station in another body's reference frame with respect to an axis
	SimTK::Vec3 reflectedPointLocation = reflectWithRespectToAxis(pointLocation, 1); // 0 for x, 1 for y, 2 for z
	// Save the calculated results in a vector and return it
	std::vector<double> positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2] };
	return positionsAndRotations;
}

// This function finds the value of station's XML attribute location and returns it as a SimTK::Vec3 vector
SimTK::Vec3 PointTracker::findStationLocationInLocalFrame(OpenSim::Model* model, std::string bodyName) {
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


/*OpenSim::Station PointTracker::addStationToModel(OpenSim::Model model, std::string bodyName, SimTK::Vec3 pointLocalCoordinates) {
	OpenSim::BodySet bodySet = model.getBodySet();
	std::cout << "Getting " << bodyName << " from model " << model.getName() << std::endl;
	OpenSim::Body mirroringBody = bodySet.get(bodyName);
	std::cout << "Found " << mirroringBody.getName() << std::endl;
	OpenSim::Station mirroringPoint(mirroringBody, pointLocalCoordinates);
	return mirroringPoint;
}*/

// This function calculates the location of the point in another body's reference frame
SimTK::Vec3 PointTracker::calculatePointLocation(SimTK::Vec3 localLocation, SimTK::State* s, OpenSim::Body* body, OpenSim::Body* referenceBody) {
	return body->findStationLocationInAnotherFrame(*s, localLocation, *referenceBody);
}

// This function calculates the rotation of the station by getting the rotation of its parent frame (body) and mirroring it with respect to the sagittal plane
SimTK::Vec3 PointTracker::calculatePointRotation(SimTK::State* s, OpenSim::Model* model, int axisIndex, OpenSim::Body* body, OpenSim::Body* referenceBody) {
	// Get the rotation of the parent body in ground reference frame
	SimTK::Rotation mirroringBodyRotation = body->getRotationInGround(*s);
	// rotate the rotation by pi radians around a suitable axis and transposing/inverting it
	SimTK::Rotation mirroredRotation = mirroringBodyRotation.setRotationFromAngleAboutAxis(3.14159265258979323, SimTK::CoordinateAxis(axisIndex)).transpose();
	// convert the 3x3 rotation matrix into body fixed XYZ euler angles
	return mirroredRotation.convertRotationToBodyFixedXYZ();
}

// This function reflects a point with respect to an axis by multiplying the element corresponding to that axis by -1
SimTK::Vec3 PointTracker::reflectWithRespectToAxis(SimTK::Vec3 pointLocation, int axisIndex) {
	pointLocation[axisIndex] = -1 * pointLocation[axisIndex];
	return pointLocation;
}

// This function takes a calibrated model file, creates a station element in under the desired body and then overwrites the .osim file
void PointTracker::addStationToBody(std::string bodyName, SimTK::Vec3 pointLocation, std::string modelFile) {
	SimTK::Xml::Element stationElement("Station");
	stationElement.setAttributeValue("name", "mirror_station");
	SimTK::Xml::Element locationElement("location", std::to_string(pointLocation[0]) + " "+ std::to_string(pointLocation[1]) + " " + std::to_string(pointLocation[2]));
	SimTK::Xml::Element parentFrameElement("parent_frame", bodyName);
	stationElement.appendNode(locationElement);
	stationElement.appendNode(parentFrameElement);

	SimTK::Xml::Document calibratedModelFile(modelFile);
	// get the root element of the XML file
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
		// If we find the desired body, append stationElement to it
		if (bodyElements.at(i).getRequiredAttributeValue("name") == bodyName)
			bodyElements[i].appendNode(stationElement);
	}

	// Write the modified file into .osim
	calibratedModelFile.writeToFile(modelFile);
}
