// This class calibrates the model by adding IMUs as XML elements on it, then saves the calibrated model as .osim file.

#pragma once

#include <OpenSim.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <Simbody.h>
#include <OpenSim/Simulation/OpenSense/IMUPlacer.h>
#include <PointTracker.h>

namespace OpenSim {
	class Model;
}

namespace OpenSimLive {

	class IMUPlacerLive : public OpenSim::IMUPlacer, public OpenSimLive::PointTracker {
	public:
		virtual ~IMUPlacerLive();
		IMUPlacerLive();
		IMUPlacerLive(const std::string& setupFile);
		bool run(bool visualizeResults = false);
		void setModel(OpenSim::Model& aModel) { _model = &aModel; };
		
		//OpenSim::Model& getCalibratedModel() const; // can be used directly from base class

		void setQuaternion(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quat) { quat_ = quat; };
		void setSubjectHeight(const double height) { subjectHeight_ = height; }
		void setModelHeight(const double height) { modelHeight_ = height; }


	private:
		void constructProperties();
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getQuaternion();
		/** Pointer to the model being _calibrated. */
		SimTK::ReferencePtr<OpenSim::Model> _model;
		/** Flag indicating if Calibration run has been invoked already */
		bool _calibrated;
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quat_;
		// This should become redundant after moving to OpenSim 4.2
		SimTK::Vec3 IMUPlacerLive::computeHeadingCorrection(
			OpenSim::Model& model,
			const SimTK::State& state,
			OpenSim::TimeSeriesTable_<SimTK::Quaternion_<double>>&
			quaternionsTable,
			const std::string& baseImuName,
			const SimTK::CoordinateDirection baseHeadingDirection);

		void scaleModel();

		// height of the human subject in cm, used in scaling the generic model
		double subjectHeight_ = 180;
		// height of the generic model, used in comparing to subject height for determining the scale factor
		double modelHeight_ = 180;


	}; // end of IMUPlacerLive

}