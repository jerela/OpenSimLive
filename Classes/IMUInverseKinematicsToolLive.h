#pragma once

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Point.h>
#include <OpenSim/Simulation/OrientationsReference.h>

namespace OpenSimLive {

	class IMUInverseKinematicsToolLive {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		IMUInverseKinematicsToolLive();
		IMUInverseKinematicsToolLive(const std::string& modelFile);
		IMUInverseKinematicsToolLive(const std::string& modelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable);
		~IMUInverseKinematicsToolLive();

		// PUBLIC METHODS DEFINED IN THE .CXX FILE
		void runInverseKinematicsWithLiveOrientations(OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, bool visualizeResults = false);
		void updateInverseKinematics(OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, bool visualizeResults = false);
		bool IMUInverseKinematicsToolLive::run(bool visualizeResults);
		bool IMUInverseKinematicsToolLive::update(bool visualizeResults);
		void reportToFile();

		// PUBLIC METHODS DEFINED HERE
		std::vector<double> getQ() { return q_; }
		void setTime(double time) { time_ = time; }
		void setQuaternion(OpenSim::TimeSeriesTable_<SimTK::Quaternion> newQuat) { quat_ = newQuat; }
		void setModel(OpenSim::Model newModel) { model_ = newModel; }
		void setModelFile(std::string newModelFile) { model_ = OpenSim::Model(newModelFile); }
		void setOpenSimLiveRootDirectory(std::string directoryPath) { OpenSimLiveRootDirectory_ = directoryPath; }
		void setSensorToOpenSimRotations(SimTK::Vec3 newRotations) { sensor_to_opensim_rotations = newRotations; }
		
		SimTK::State getState() { return s_; }
		SimTK::Vec3 pointTracker(SimTK::State state, OpenSim::Frame referenceFrame = OpenSim::Ground, std::string bodyName = "hand_r"){
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

	private:
		// PRIVATE VARIABLES
		OpenSim::TimeSeriesTable* modelOrientationErrors_;
		OpenSim::TableReporter* ikReporter_;
		SimTK::State s_; // the state we use for visualization
		double time_ = 0; // time since calibration
		SimTK::Vec3 sensor_to_opensim_rotations = { -1.5707963267948966, 0, 0 }; // the rotations applied to IMU coordinate systems to match its axes to OpenSim coordinate system
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quat_; // quaternion data for IMU orientation
		OpenSim::Set<OpenSim::OrientationWeight> orientationWeightSet; // weights for individual IMUs
		bool report_errors = true;
		OpenSim::Model model_; // the OpenSim model that state s_ depicts
		std::vector<double> q_; // joint angles calculated from IK are stored here

		// PRIVATE METHODS
		std::string OpenSimLiveRootDirectory_ = "";
		SimTK::Vec3 get_sensor_to_opensim_rotations() { return sensor_to_opensim_rotations; }
		OpenSim::Set<OpenSim::OrientationWeight> get_orientation_weights() { return orientationWeightSet; }
		bool get_report_errors() { return report_errors; }
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> get_quat() { return quat_; }
		OpenSim::Model get_model() { return model_; }
		void setQ(std::vector<double> q) { q_ = q; }

	};  // end of class

}