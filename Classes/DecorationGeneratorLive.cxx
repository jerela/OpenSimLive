#include <DecorationGeneratorLive.h>
#include <OpenSim.h>

using namespace OpenSimLive;

DecorationGeneratorLive::DecorationGeneratorLive() {}

DecorationGeneratorLive::~DecorationGeneratorLive() {}

void DecorationGeneratorLive::generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) {
	// create an array to hold the decorative sphere
	SimTK::Array_<SimTK::DecorativeGeometry> geomArray(1);
	// create the decorative sphere
	SimTK::DecorativeSphere sphere(0.02);
	// make the sphere blue
	sphere.setColor({ 0, 0, 1 });
	// make the sphere partially transparent
	sphere.setOpacity(0.5);
	// set the sphere's coordinate system to the reference body's coordinate system
	sphere.setBodyId(referenceBodyId_);
	// set the sphere's location in its coordinate system
	sphere.setTransform(transform_);
	// put the sphere into an array
	geomArray[0] = sphere;
	// save it as the resulting geometry to be shown
	geometry = geomArray;
}
