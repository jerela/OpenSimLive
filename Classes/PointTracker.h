// This class calculates the position of a mirrored point and the orientation of its parent body for mirror therapy applications.

#pragma once

#include <OpenSim.h>
#include <vector>
#include <DecorationGeneratorLive.h>

namespace OpenSimLive {

	class PointTracker {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		PointTracker();
		PointTracker(SimTK::State state, std::string modelFileName);
		PointTracker(SimTK::State state, std::string modelFileName, std::string bodyName);
		~PointTracker();

		// PUBLIC METHODS
		void addStationToBody(const std::string& bodyName, const SimTK::Vec3& pointLocation, const std::string& modelFile);
		std::vector<double> runTracker(const SimTK::State* s, OpenSim::Model* model, const std::string& bodyName, const std::string& referenceBodyName);
		void setPointTrackerEnabled(const bool setting) { pointTrackerEnabled_ = setting; }

	protected:
		// PROTECTED METHODS
		bool getPointTrackerEnabled() { return pointTrackerEnabled_; }
		void createDecorationGenerator() { decGen_ = new OpenSimLive::DecorationGeneratorLive(); }
		void setVisualize(bool setting) { visualize_ = setting; }

		// PROTECTED VARIABLES
		OpenSimLive::DecorationGeneratorLive* decGen_;

	private:
		// PRIVATE METHODS
		SimTK::Vec3 findStationLocationInLocalFrame(OpenSim::Model* model, const std::string& bodyName);
		SimTK::Vec3 calculatePointLocation(const SimTK::Vec3& localLocation, const SimTK::State& s, const OpenSim::Body* body, const OpenSim::Body* referenceBody);
		SimTK::Vec3 calculatePointRotation(const SimTK::State* s, OpenSim::Model* model, const int axisIndex, const OpenSim::Body* body, const OpenSim::Body* referenceBody);
		void reflectWithRespectToAxis(SimTK::Vec3& pointLocation, const int axisIndex);
		
		//PRIVATE VARIABLES
		std::string referenceBodyName_ = "pelvis";
		std::string bodyName_ = "";
		bool pointTrackerEnabled_ = true;
		bool visualize_ = false;

	}; // end of class
}