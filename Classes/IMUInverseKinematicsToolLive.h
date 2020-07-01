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
		IMUInverseKinematicsToolLive();
		IMUInverseKinematicsToolLive(const std::string& modelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable);
		~IMUInverseKinematicsToolLive();
		void runInverseKinematicsWithLiveOrientations(OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, bool visualizeResults = false);
		bool IMUInverseKinematicsToolLive::run(bool visualizeResults);
		std::vector<double> getQ() { return q_; }

	private:
		//void constructProperties();
		SimTK::Vec3 sensor_to_opensim_rotations = { -1.5707963267948966, 0, 0 };
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quat_;
		OpenSim::Set<OpenSim::OrientationWeight> orientationWeightSet;
		bool report_errors = true;
		OpenSim::Model model_;
		std::vector<double> q_;
		//std::string modelFileName = "gait2392_full_calibrated.osim";
		SimTK::Vec3 get_sensor_to_opensim_rotations() { return sensor_to_opensim_rotations; }
		OpenSim::Set<OpenSim::OrientationWeight> get_orientation_weights() { return orientationWeightSet; }
		bool get_report_errors() { return report_errors; }
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> get_quat() { return quat_; }
		OpenSim::Model get_model() { return model_; }
		void setQ(std::vector<double> q) { q_ = q; }

	};  // end of class

}