#pragma once

#include <OpenSim.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <Simbody.h>
#include <OpenSim/Simulation/OpenSense/IMUPlacer.h>

namespace OpenSim {

	class Model;
	class IMUPlacerLive : public IMUPlacer {
	public:
		virtual ~IMUPlacerLive();
		IMUPlacerLive();
		IMUPlacerLive(const std::string& setupFile);
		bool run(bool visualizeResults = false);
		void setModel(Model& aModel) { _model = &aModel; };
		
		Model& getCalibratedModel() const; // can be used directly from base class

		void setQuaternion(OpenSim::TimeSeriesTable_<SimTK::Quaternion> quat) { quat_ = quat; };


	private:
		void constructProperties();
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> getQuaternion();
		/** Pointer to the model being _calibrated. */
		SimTK::ReferencePtr<Model> _model;
		/** Flag indicating if Calibration run has been invoked already */
		bool _calibrated;
		OpenSim::TimeSeriesTable_<SimTK::Quaternion> quat_;


	}; // end of IMUPlacerLive

}