#pragma once

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ModelDisplayHints.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Point.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <PointTracker.h>

namespace OpenSimLive {

	class IMUInverseKinematicsToolLive : public OpenSimLive::PointTracker {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		IMUInverseKinematicsToolLive();
		IMUInverseKinematicsToolLive(const std::string& modelFile);
		IMUInverseKinematicsToolLive(const std::string& modelFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable);
		~IMUInverseKinematicsToolLive();

		// PUBLIC METHODS DEFINED IN THE .CXX FILE
		void runInverseKinematicsWithLiveOrientations(OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, const bool visualizeResults = false);
		bool IMUInverseKinematicsToolLive::run(const bool visualizeResults);
		void IMUInverseKinematicsToolLive::update(const bool visualizeResults);
		void reportToFile();

		// PUBLIC METHODS DEFINED HERE
		std::vector<double> getQ() { return q_; }
		std::vector<double> getPointTrackerPositionsAndOrientations() { return pointTrackerPositionsAndOrientations_; }
		void setTime(const double time) { time_ = time; }
		void setQuaternion(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& newQuat) { quat_ = newQuat; }
		void setModel(const OpenSim::Model& newModel) { model_ = newModel; }
		void setModelFile(const std::string& newModelFile) { model_ = OpenSim::Model(newModelFile); }
		void setOpenSimLiveRootDirectory(const std::string& directoryPath) { OpenSimLiveRootDirectory_ = directoryPath; }
		void setSensorToOpenSimRotations(const SimTK::Vec3& newRotations) { sensor_to_opensim_rotations = newRotations; }
		void setPointTrackerReferenceBodyName(const std::string& referenceBodyName) { pointTrackerReferenceBodyName_ = referenceBodyName; }
		void setPointTrackerBodyName(const std::string& bodyName) { pointTrackerBodyName_ = bodyName; }
		void setSaveIKResults(bool save) { save_ik_results_ = save; }
		bool getSaveIKResults() { return save_ik_results_; }
		void setReportErrors(bool report) { report_errors = report; }
		void setAccuracy(double accuracy) { accuracy_ = accuracy; }

	private:
		// PRIVATE VARIABLES
		std::string OpenSimLiveRootDirectory_ = "";
		double accuracy_ = 1e-5;
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
		std::vector<double> pointTrackerPositionsAndOrientations_;
		std::string pointTrackerBodyName_ = "";
		std::string pointTrackerReferenceBodyName_ = "pelvis";
		bool save_ik_results_ = false;

		// PRIVATE METHODS DEFINED HERE
		SimTK::Vec3 get_sensor_to_opensim_rotations() { return sensor_to_opensim_rotations; }
		OpenSim::Set<OpenSim::OrientationWeight> get_orientation_weights() { return orientationWeightSet; }
		bool get_report_errors() { return report_errors; }
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> get_quat() { return quat_; }
		OpenSim::Model get_model() { return model_; }
		void setQ(const std::vector<double>& q) { q_ = q; }
		void setPointTrackerPositionsAndOrientations(const std::vector<double>& positionsAndOrientations) { pointTrackerPositionsAndOrientations_ = positionsAndOrientations; }
		std::string getPointTrackerBodyName() { return pointTrackerBodyName_; }
		std::string getPointTrackerReferenceBodyName() { return pointTrackerReferenceBodyName_; }

		// PRIVATE METHODS DEFINED IN THE .CXX FILE
		void updateInverseKinematics(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, const bool visualizeResults = false);
		void updateConcurrentInverseKinematics(OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, const bool visualizeResults = false);
		void updateJointAngleVariable(SimTK::State& s, OpenSim::Model& model);
		void updatePointTracker();

	};  // end of class

}