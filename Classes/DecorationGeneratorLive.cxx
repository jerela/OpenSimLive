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

	// let's also create an arrow from the pelvis, pointing like the mirrored rotation of the body
	SimTK::DecorativeLine arrow(SimTK::Vec3(0, 0, 0), SimTK::Vec3(1, 0, 0));
	arrow.setColor({ 0,0,1 });
	arrow.setBodyId(referenceBodyId_);
	arrow.setPoint1(SimTK::Vec3(0, 0, 0));
	arrow.setPoint2(SimTK::Vec3(0, -1, 0));
	arrow.setOpacity(0.5);
	arrow.setLineThickness(10);
	//arrow.setTransform(arrowDir_);
	arrow.setTransform(transform_);
	geomArray[1] = arrow;




	// save the array as the resulting geometry to be shown
	geometry = geomArray;


}
