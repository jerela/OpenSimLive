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
		void setPointTrackerEnabled(bool setting) { pointTrackerEnabled_ = setting; }

	protected:
		bool getPointTrackerEnabled() { return pointTrackerEnabled_; }
		
	private:
		// PRIVATE METHODS
		SimTK::Vec3 findStationLocationInLocalFrame(OpenSim::Model* model, std::string bodyName);
		SimTK::Vec3 calculatePointLocation(SimTK::Vec3 localLocation, SimTK::State* s, OpenSim::Body* body, OpenSim::Body* referenceBody);
		SimTK::Vec3 calculatePointRotation(SimTK::State* s, OpenSim::Model* model, int axisIndex, OpenSim::Body* body, OpenSim::Body* referenceBody);
		SimTK::Vec3 reflectWithRespectToAxis(SimTK::Vec3 pointLocation, int axisIndex);
		
		//PRIVATE VARIABLES
		std::string referenceBodyName_ = "pelvis";
		std::string bodyName_ = "";
		bool pointTrackerEnabled_ = true;

	}; // end of class
}