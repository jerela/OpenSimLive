#include <DecorationGeneratorLive.h>
#include <OpenSim.h>

using namespace OpenSimLive;

DecorationGeneratorLive::DecorationGeneratorLive() {}
DecorationGeneratorLive::DecorationGeneratorLive(double radius) {
	
}

DecorationGeneratorLive::~DecorationGeneratorLive() {}

void DecorationGeneratorLive::generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) {
	std::cout << "DecGenLive" << std::endl;
	SimTK::Array_<SimTK::DecorativeGeometry> geomArray(1);
	SimTK::DecorativeSphere sphere(1);
	geomArray[0] = sphere;
}
