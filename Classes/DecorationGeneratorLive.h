// This class defines the geometry objects that are drawn in addition to the musculoskeletal model in the visualization window.
#pragma once

#include <OpenSim.h>

namespace OpenSimLive {

	class DecorationGeneratorLive : public SimTK::DecorationGenerator {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		// constructor
		DecorationGeneratorLive();
		// deconstructor
		~DecorationGeneratorLive();

		// PUBLIC METHODS
		// This function is called whenever a new state is about to be visualized. It generates the desired decorative geometries for each frame.
		void generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
		// sets referenceBodyId_ to hold the id of the reference body for mirror therapy
		void setReferenceBodyId(int id) { referenceBodyId_ = id; }
		// set the transform of the mirrored body with respect to the station reference body in the coordinate system of the station parent body
		void setTransformInReferenceBody(SimTK::Transform transform) { transform_ = transform; }

	protected:
		
	private:
		// PRIVATE METHODS
		
		//PRIVATE VARIABLES
		// id of the reference body (on the healthy limb that we are mirroring) on the musculoskeletal model
		int referenceBodyId_;
		// position and orientation of the mirrored body with respect to the station reference body (usually pelvis) in the coordinate system of the station parent body (such as hand_r)
		SimTK::Transform transform_;

	}; // end of class
}