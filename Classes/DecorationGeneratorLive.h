#pragma once

#include <OpenSim.h>

namespace OpenSimLive {

	class DecorationGeneratorLive : public SimTK::DecorationGenerator {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		DecorationGeneratorLive();
		DecorationGeneratorLive(double radius);
		~DecorationGeneratorLive();

		// PUBLIC METHODS
		void generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry);

	protected:
		
	private:
		// PRIVATE METHODS
		
		//PRIVATE VARIABLES

	}; // end of class
}