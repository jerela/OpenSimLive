
#include "IMUPlacerLive.h"
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSimLive;
using SimTK::Vec3;

IMUPlacerLive::IMUPlacerLive() {
    constructProperties();
    _calibrated = false;
}

IMUPlacerLive::IMUPlacerLive(const std::string& setupFile) : OpenSim::IMUPlacer(setupFile) {

}

//_____________________________________________________________________________
/**
 * Destructor.
 */
IMUPlacerLive::~IMUPlacerLive() {}

//=============================================================================
// CONSTRUCTION

//_____________________________________________________________________________
/**
 */
void IMUPlacerLive::constructProperties() {
    constructProperty_model_file("");
    constructProperty_base_imu_label("");
    constructProperty_base_heading_axis("");
    constructProperty_sensor_to_opensim_rotations(SimTK::Vec3(0));
//    constructProperty_orientation_file_for_calibration("");
    constructProperty_output_model_file("");
}


OpenSim::TimeSeriesTable_<SimTK::Quaternion> IMUPlacerLive::getQuaternion() {
    return quat_;
}

/**
 * This method runs the calibration method on the _model maintained by
 * this IMUPlacer
 */
bool IMUPlacerLive::run(bool visualizeResults) {

    std::cout << "IMUPlacerLive run initiated" << std::endl;
    _calibrated = false;
    // Check there's a model file specified before trying to open it
    if (get_model_file().size() == 0) {
        OPENSIM_THROW(OpenSim::Exception, "No model file specified for IMUPlacer.");
    }
    // If there is no model file loaded, load one
    if (_model.empty()) { _model.reset(new OpenSim::Model(get_model_file())); } // program failing here will indicate that one of the config .xml files points to the wrong OpenSim model file path
    // Define quatTable
    OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(getQuaternion());
    // Get sensor to OpenSim rotations to a vector from file
    const SimTK::Vec3& sensor_to_opensim_rotations = get_sensor_to_opensim_rotations();
    // Get those rotations as a rotation matrix
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
            sensor_to_opensim_rotations[0], SimTK::XAxis,
            sensor_to_opensim_rotations[1], SimTK::YAxis,
            sensor_to_opensim_rotations[2], SimTK::ZAxis);
    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);
    // Check consistent heading correction specification
    // both base_heading_axis and base_imu_label should be specified
    // finer error checking is done downstream
    bool performHeadingRequested = !get_base_heading_axis().empty() && !get_base_imu_label().empty();
    if (performHeadingRequested) {
        std::string imu_axis = OpenSim::IO::Lowercase(get_base_heading_axis());

        SimTK::CoordinateDirection directionOnIMU(SimTK::ZAxis);
        int direction = 1;
        if (imu_axis.front() == '-') direction = -1;
        // get the last char of string imu_axis, which is either x, y or z
        char& back = imu_axis.back();
        if (back == 'x')
            directionOnIMU = SimTK::CoordinateDirection(SimTK::XAxis, direction);
        else if (back == 'y')
            directionOnIMU = SimTK::CoordinateDirection(SimTK::YAxis, direction);
        else if (back == 'z')
            directionOnIMU = SimTK::CoordinateDirection(SimTK::ZAxis, direction);
        else { // Throw, invalid specification
            OPENSIM_THROW(OpenSim::Exception, "Invalid specification of heading axis '" +
                imu_axis + "' found.");
        }

        // Compute rotation matrix so that (e.g. "pelvis_imu"+ SimTK::ZAxis)
        // lines up with model forward (+X)
        SimTK::Vec3 headingRotationVec3 = OpenSim::OpenSenseUtilities::computeHeadingCorrection(*_model, quatTable, get_base_imu_label(), directionOnIMU);
        // rotation matrix describing rotation between OpenSim coordinate system and IMU coordinate system with base heading axis pointing along X axis of the OpenSim coordinate system
        SimTK::Rotation headingRotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, headingRotationVec3[0], SimTK::XAxis, headingRotationVec3[1], SimTK::YAxis, headingRotationVec3[2], SimTK::ZAxis);

        OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, headingRotation);
    }
    else
        cout << "No heading correction is applied." << endl;

    // This is now plain conversion, no Rotation or magic underneath
    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    auto imuLabels = orientationsData.getColumnLabels();
    auto& times = orientationsData.getIndependentColumn();

    // The rotations of the IMUs at the start time in order
    // the labels in the TimerSeriesTable of orientations
    auto rotations = orientationsData.updRowAtIndex(0);

    SimTK::State& s0 = _model->initSystem();
    s0.updTime() = times[0];

    // default pose of the model
    _model->realizePosition(s0);

    size_t imuix = 0;
    std::vector<OpenSim::PhysicalFrame*> bodies{ imuLabels.size(), nullptr };
    std::map<std::string, SimTK::Rotation> imuBodiesInGround;

    // First compute the transform of each of the imu bodies in ground
    for (auto& imuName : imuLabels) {
        auto ix = imuName.rfind("_imu");
        if (ix != std::string::npos) {
            auto bodyName = imuName.substr(0, ix);
            auto body = _model->findComponent<OpenSim::PhysicalFrame>(bodyName);
            if (body) {
                bodies[imuix] = const_cast<OpenSim::PhysicalFrame*>(body);
                imuBodiesInGround[imuName] = body->getTransformInGround(s0).R();
            }
        }
        ++imuix;
    }

    // Now cycle through each imu with a body and compute the relative
    // offset of the IMU measurement relative to the body and
    // update the modelOffset OR add an offset if none exists
    imuix = 0;
    for (auto& imuName : imuLabels) {
        cout << "Processing " << imuName << endl;
        if (imuBodiesInGround.find(imuName) != imuBodiesInGround.end()) {
            cout << "Computed offset for " << imuName << endl;
            SimTK::Rotation R_FB =
                ~imuBodiesInGround[imuName] * rotations[int(imuix)];
            cout << "Offset is " << R_FB << endl;
            OpenSim::PhysicalOffsetFrame* imuOffset = nullptr;
            const OpenSim::PhysicalOffsetFrame* mo = nullptr;
            if ((mo = _model->findComponent<OpenSim::PhysicalOffsetFrame>(imuName))) {
                imuOffset = const_cast<OpenSim::PhysicalOffsetFrame*>(mo);
                auto X = imuOffset->getOffsetTransform();
                X.updR() = R_FB;
                imuOffset->setOffsetTransform(X);
            }
            else {
                cout << "Creating offset frame for " << imuName << endl;
                OpenSim::Body* body =
                    dynamic_cast<OpenSim::Body*>(bodies[imuix]);
                SimTK::Vec3 p_FB(0);
                if (body) { p_FB = body->getMassCenter(); }

                imuOffset = new OpenSim::PhysicalOffsetFrame(
                    imuName, *bodies[imuix], SimTK::Transform(R_FB, p_FB));
                auto* brick = new OpenSim::Brick(Vec3(0.02, 0.01, 0.005));
                brick->setColor(SimTK::Orange);
                imuOffset->attachGeometry(brick);
                bodies[imuix]->addComponent(imuOffset);
                cout << "Added offset frame for " << imuName << endl;
            }
            cout << imuOffset->getName() << " offset computed from " << imuName << " data from file." << endl;
        }
        imuix++;
    }

    _model->finalizeConnections();

    if (!get_output_model_file().empty())
        _model->print(get_output_model_file());

    _calibrated = true;
    if (visualizeResults) {
        _model->setUseVisualizer(true);
        SimTK::State& s = _model->initSystem();

        s.updTime() = times[0];

        // create the solver given the input data
        OpenSim::MarkersReference mRefs{};
        OpenSim::OrientationsReference oRefs(orientationsData);
        SimTK::Array_<OpenSim::CoordinateReference> coordRefs{};

        const double accuracy = 1e-4;
        OpenSim::InverseKinematicsSolver ikSolver(*_model, mRefs, oRefs, coordRefs);
        ikSolver.setAccuracy(accuracy);

        SimTK::Visualizer& viz = _model->updVisualizer().updSimbodyVisualizer();
        // We use the input silo to get key presses.
        auto silo = &_model->updVisualizer().updInputSilo();
        silo->clear(); // Ignore any previous key presses.

        SimTK::DecorativeText help("Press any key to quit.");
        help.setIsScreenText(true);
        viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);
        _model->getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        ikSolver.assemble(s);
        _model->getVisualizer().show(s);

        unsigned key, modifiers;
        silo->waitForKeyHit(key, modifiers);
        viz.shutdown();
    }
    return true;
}

/*Model& IMUPlacer::getCalibratedModel() const {
    if (_calibrated) return *_model;
    OPENSIM_THROW(Exception, "Attempt to retrieve calibrated model without "
        "invoking IMU_Placer::run.");
}*/