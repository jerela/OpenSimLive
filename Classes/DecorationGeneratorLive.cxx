#include <DecorationGeneratorLive.h>
#include <OpenSim.h>

using namespace OpenSimLive;

DecorationGeneratorLive::DecorationGeneratorLive() {}

DecorationGeneratorLive::~DecorationGeneratorLive() {}

void DecorationGeneratorLive::generateDecorations(const SimTK::State& state, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) {
	// create an array to hold the decorative sphere
	SimTK::Array_<SimTK::DecorativeGeometry> geomArray(2);
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

	// let's also create a line from the pelvis, pointing along the mirrored rotation of the body
	SimTK::DecorativeLine arrow(SimTK::Vec3(0, 0, 0), SimTK::Vec3(1, 0, 0));
	// set it to blue
	arrow.setColor({ 0,0,1 });
	// set its coordinate system to that of the reference body
	arrow.setBodyId(referenceBodyId_);
	// set it to point directly downwards initially, like arms in the anatomical position
	arrow.setPoint1(SimTK::Vec3(0, 0, 0));
	arrow.setPoint2(SimTK::Vec3(0, -1, 0));
	// make it 50% transparent
	arrow.setOpacity(0.5);
	// make it thick enough for us to clearly see it in the visualization window
	arrow.setLineThickness(10);
	// set its position and orientation
	arrow.setTransform(transform_);
	// put it in the array of geometry objects that are drawn
	geomArray[1] = arrow;

	// save the array as the resulting geometry to be shown
	geometry = geomArray;
}
