#pragma once

#include <OpenSim.h>

namespace OpenSimLive {

	class DecorationGeneratorLive : public SimTK::DecorationGenerator {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		DecorationGeneratorLive();
		~DecorationGeneratorLive();

		// PUBLIC METHODS
		void generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
		void setReferenceBodyId(int id) { referenceBodyId_ = id; }
		void setTransformInReferenceBody(SimTK::Transform transform) { transform_ = transform; }
		void setArrowDirection(SimTK::Transform transform) { arrowDir_ = transform; }

	protected:
		
	private:
		// PRIVATE METHODS
		
		//PRIVATE VARIABLES
		int referenceBodyId_;
		SimTK::Transform transform_;
		SimTK::Transform arrowDir_;

	}; // end of class
}