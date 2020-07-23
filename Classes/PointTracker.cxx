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

std::vector<double> PointTracker::runTracker(SimTK::State* s, OpenSim::Model* model, std::string bodyName, std::string referenceBodyName, SimTK::Vec3 pointLocalCoordinates) {
	std::cout << "runTracker started" << std::endl;
	OpenSim::Body* body = &(model->updBodySet().get(bodyName));
	OpenSim::Body* referenceBody = &(model->updBodySet().get(referenceBodyName));
	std::cout << "runTracker 2" << std::endl;
	SimTK::Vec3 stationLocationLocal(findStationLocationInLocalFrame(model, bodyName));
	std::cout << "runTracker 3" << std::endl;
	SimTK::Vec3 pointLocation = calculatePointLocation(stationLocationLocal, s, body, referenceBody);
	std::cout << "runTracker 4" << std::endl;
	SimTK::Vec3 mirroredEuler = calculatePointRotation(s, model, 2, body, referenceBody);
	std::cout << "runTracker 5" << std::endl;
	SimTK::Vec3 reflectedPointLocation = reflectWithRespectToAxis(pointLocation, 1); // 0 for x, 1 for y, 2 for z
	std::cout << "runTracker 6" << std::endl;
	std::vector<double> positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2] };
	std::cout << "runTracker 7" << std::endl;
	return positionsAndRotations;
}

SimTK::Vec3 PointTracker::findStationLocationInLocalFrame(OpenSim::Model* model, std::string bodyName) {
	SimTK::Xml::Document modelXML(model->getInputFileName());
	SimTK::Array_<SimTK::Xml::Element> bodyElements = modelXML.getRootElement().getRequiredElement("Model").getRequiredElement("BodySet").getRequiredElement("objects").getAllElements("Body");
	std::string locationString;
	for (unsigned int i = 0; i < bodyElements.size(); ++i) {
		if (bodyElements[i].getRequiredAttributeValue("name") == bodyName)
		{
			locationString = bodyElements[i].getRequiredElement("Station").getRequiredElementValue("location");
			break;
		}
	}
	if (locationString.empty()) {
		std::cerr << "Error! Location string was empty!";
	}
	std::stringstream ss(locationString);
	std::string word;
	SimTK::Vec3 locationVector = { 0,0,0 };
	for (int j = 0; j < 3; ++j) {
		ss >> word;
		locationVector[j] = std::stoi(word);
	}
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

SimTK::Vec3 PointTracker::calculatePointLocation(SimTK::Vec3 localLocation, SimTK::State* s, OpenSim::Body* body, OpenSim::Body* referenceBody) {
	return body->findStationLocationInAnotherFrame(*s, localLocation, *referenceBody);
}

SimTK::Vec3 PointTracker::calculatePointRotation(SimTK::State* s, OpenSim::Model* model, int axisIndex, OpenSim::Body* body, OpenSim::Body* referenceBody) {
	//std::cout << "CalcPointRot" << std::endl;
	//OpenSim::BodySet bodySet = model->getBodySet();
	//OpenSim::Body mirroringBody = bodySet.get(bodyName);
	SimTK::Rotation mirroringBodyRotation = body->getRotationInGround(*s);
	// rotate the rotation by pi radians around a suitable axis and transposing/inverting it
	SimTK::Rotation mirroredRotation = mirroringBodyRotation.setRotationFromAngleAboutAxis(3.14159265258979323, SimTK::CoordinateAxis(axisIndex)).transpose();
	// convert the 3x3 rotation matrix into body fixed XYZ euler angles
	return mirroredRotation.convertRotationToBodyFixedXYZ();
}

SimTK::Vec3 PointTracker::reflectWithRespectToAxis(SimTK::Vec3 pointLocation, int axisIndex) {
	std::cout << "Reflect" << std::endl;
	pointLocation[axisIndex] = -1 * pointLocation[axisIndex];
	return pointLocation;
}

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

	for (int i = 0; i < numberOfBodies; ++i) {
		if (bodyElements.at(i).getRequiredAttributeValue("name") == bodyName)
			bodyElements[i].appendNode(stationElement);
	}

	// write the modified file into .osim
	calibratedModelFile.writeToFile(modelFile);
}
