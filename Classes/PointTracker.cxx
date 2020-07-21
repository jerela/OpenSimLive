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

std::vector<double> PointTracker::runTracker(OpenSim::Station station, SimTK::State s, OpenSim::Model model, std::string bodyName, SimTK::Vec3 pointLocalCoordinates) {
	SimTK::Vec3 pointLocation = calculatePointLocation(station, s);
	SimTK::Vec3 mirroredEuler = calculatePointRotation(s, model, 2, bodyName);
	SimTK::Vec3 reflectedPointLocation = reflectWithRespectToAxis(pointLocation, 1); // 0 for x, 1 for y, 2 for z
	std::vector<double> positionsAndRotations = { pointLocation[0], pointLocation[1], pointLocation[2], mirroredEuler[0], mirroredEuler[1], mirroredEuler[2] };
	return positionsAndRotations;
}

OpenSim::Station PointTracker::addStationToModel(OpenSim::Model model, std::string bodyName, SimTK::Vec3 pointLocalCoordinates) {
	OpenSim::BodySet bodySet = model.getBodySet();
	std::cout << "Getting " << bodyName << " from model " << model.getName() << std::endl;
	OpenSim::Body mirroringBody = bodySet.get(bodyName);
	std::cout << "Found " << mirroringBody.getName() << std::endl;
	OpenSim::Station mirroringPoint(mirroringBody, pointLocalCoordinates);
	//mirroringBody.addComponent(&mirroringPoint);
	//model.compo
	//OpenSim::Component& mirroringBody = model.updComponent(bodyName);
	//OpenSim::Station mirroringPoint(*mirroringBody, pointLocalCoordinates);
	return mirroringPoint;
}

SimTK::Vec3 PointTracker::calculatePointLocation(OpenSim::Station station, SimTK::State s) {
	//std::cout << "CalcPointLoc" << std::endl;
	//OpenSim::BodySet bodySet = model.getBodySet();
	//std::cout << "Getting " << bodyName << " from model " << model.getName() << std::endl;
	//OpenSim::Body mirroringBody = bodySet.get(bodyName);
	//std::cout << "Found " << mirroringBody.getName() << std::endl;
	//OpenSim::Station mirroringPoint(mirroringBody, pointLocalCoordinates);
	//s = model.initSystem();
	//model.addComponent(&mirroringPoint);
	// extendSetPropertiesFromState
	//mirroringBody.addComponent(&mirroringPoint);
	//model.finalizeFromProperties();
	std::cout << "finding location in frame" << std::endl;
	return station.findLocationInFrame(s, OpenSim::Ground());
}

SimTK::Vec3 PointTracker::calculatePointRotation(SimTK::State s, OpenSim::Model model, int axisIndex, std::string bodyName) {
	std::cout << "CalcPointRot" << std::endl;
	OpenSim::BodySet bodySet = model.getBodySet();
	OpenSim::Body mirroringBody = bodySet.get(bodyName);
	SimTK::Rotation mirroringBodyRotation = mirroringBody.getRotationInGround(s);
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