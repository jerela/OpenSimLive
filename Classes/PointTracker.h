#pragma once

//#include <OpenSim/Common/Object.h>
//#include <OpenSim/Common/ModelDisplayHints.h>
//#include <OpenSim/Common/Set.h>
//#include <OpenSim/Common/TimeSeriesTable.h>
//#include <OpenSim/Simulation/Model/Point.h>
//#include <OpenSim/Simulation/OrientationsReference.h>

namespace OpenSimLive {

	class PointTracker {
	
	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		PointTracker();
		~PointTracker();
		
		// PUBLIC METHODS
		void setModel(OpenSim::Model model) { model_ = model; }
	
	private:
		// PRIVATE METHODS
		
		//PRIVATE VARIABLES
		OpenSim::Model model_; // calibrated IMU model
		SimTK::State s_; // the state of the system when the location of the point is calculated
		SimTK::Vec3 pointLocation_; // XYZ coordinates of the point
		OpenSim::Frame referenceFrame_; // reference frame to use as the coordinate system for point location
		std::string bodyName_; // name of the body that the point is located on


		SimTK::Vec3 calculatePointLocation(SimTK::State state, OpenSim::Frame referenceFrame = OpenSim::Ground, std::string bodyName = "hand_r"){
			std::string bodyName = "hand_r"; // read from XML file?
			OpenSim::Model model(modelFileName);
			OpenSim::BodySet bodySet = model.getBodySet();
			OpenSim::Body mirroringBody = bodySet.get(bodyName);
			const SimTK::Vec3 pointLocalCoordinates(0,0,0); // read from XML file or leave as is, if it doesn't matter
			OpenSim::Station mirroringPoint(mirroringBody, pointLocalCoordinates);
			OpenSim::Frame referenceFrame = OpenSim::Ground; // read from XML file, by default pelvis or ground?
			SimTK::Vec3 pointLocation = mirroringPoint.findLocationInFrame(state, referenceFrame);
			return pointLocation;
			// use this as a private method that is called at the end of update method
			// this way you can directly use s_ and model_
			// use setter functions to define bodyName, referenceFrame and pointLocalCoordinates
			// create a public getter function to extract the point
			// mirror the point by multiplying its y-coordinate by -1
			// transform the point to coordinates used by KUKA and pass it on to KUKA API
			
			// ALTERNATIVELY to avoid bloating of IMUInverseKinematicsToolLive
			// create a public getter function to get state s_ at each update
			// use that as an input to a separate PointTracker object to call pointTracker()
			// use that class to mirror(std::char axis) and transform the point (and possibly to pass it on to KUKA API)
		}
		
		SimTK::Vec3 reflectWithRespectToAxis(SimTK::Vec3 vectorToReflect, int axisIndex) {
			SimTK::Vec3 reflectedVector = vectorToReflect;
			reflectedVector[axisIndex] = -1*reflectedVector[axisIndex];
			return reflectedVector;
		}
		
	}
}