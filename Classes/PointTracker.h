#pragma once

#include <OpenSim.h>
#include <vector>

namespace OpenSimLive {

	class PointTracker {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		PointTracker();
		PointTracker(SimTK::State state, std::string modelFileName);
		PointTracker(SimTK::State state, std::string modelFileName, std::string bodyName);
		~PointTracker();

		// PUBLIC METHODS
		void addStationToBody(std::string bodyName, SimTK::Vec3 pointLocation, std::string modelFile);
		std::vector<double> runTracker(SimTK::State* s, OpenSim::Model* model, std::string bodyName, std::string referenceBodyName, SimTK::Vec3 pointLocalCoordinates);
		//void setModel(OpenSim::Model model) { model_ = model; }
		//void setModelByFileName(std::string modelFileName) { model_ = OpenSim::Model(modelFileName); }
		//void setState(SimTK::State state) { s_ = state; }
		//void setReferenceFrame(OpenSim::Body frameBody) { referenceFrame_ = &frameBody; }
		//void setBodyName(std::string bodyName) { bodyName_ = bodyName; }
		//void setPointLocalCoordinates(SimTK::Vec3 pointLocalCoordinates) { pointLocalCoordinates_ = pointLocalCoordinates; }

	protected:
		//OpenSim::Station addStationToModel(OpenSim::Model model, std::string bodyName, SimTK::Vec3 pointLocalCoordinates);
		void setStation(OpenSim::Station station) { station_ = station; }
		OpenSim::Station getStation() { return station_; }

	private:
		// PRIVATE METHODS
		SimTK::Vec3 findStationLocationInLocalFrame(OpenSim::Model* model, std::string bodyName);
		SimTK::Vec3 calculatePointLocation(SimTK::Vec3 localLocation, SimTK::State* s, OpenSim::Body* body, OpenSim::Body* referenceBody);
		SimTK::Vec3 calculatePointRotation(SimTK::State* s, OpenSim::Model* model, int axisIndex, OpenSim::Body* body, OpenSim::Body* referenceBody);
		SimTK::Vec3 reflectWithRespectToAxis(SimTK::Vec3 pointLocation, int axisIndex);
		//SimTK::Vec3 getPointLocation() { return pointLocation_; };

		//PRIVATE VARIABLES
		OpenSim::Station station_;
		//OpenSim::Model model_; // calibrated IMU model
		//SimTK::State s_; // the state of the system when the location of the point is calculated
		//SimTK::Vec3 pointLocalCoordinates_ = { 0,0,0 };
		//SimTK::Vec3 pointLocation_; // XYZ coordinates of the point (before and after mirroring)
		//SimTK::Vec3 mirroredEuler_; // Euler angles of the body of the point, after mirroring
		//OpenSim::Frame* referenceFrame_ = &OpenSim::Ground(); // reference frame to use as the coordinate system for point location
		//std::string bodyName_; // name of the body that the point is located on

	}; // end of class
}