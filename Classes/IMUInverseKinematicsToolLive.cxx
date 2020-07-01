#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <IMUInverseKinematicsToolLive.h>

using namespace OpenSimLive;
using namespace SimTK;
using namespace std;

// CONSTRUCTORS

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive() {
    //constructProperties();
}

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable) {
    model_ = OpenSim::Model(modelFile);
    quat_ = quatTable;
}

// DESTRUCTOR

IMUInverseKinematicsToolLive::~IMUInverseKinematicsToolLive()
{
}

// PUBLIC MEMBER FUNCTIONS

void IMUInverseKinematicsToolLive::runInverseKinematicsWithLiveOrientations(
    OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable,
    bool visualizeResults) {

    // Ideally if we add a Reporter, we also remove it at the end for good hygiene but 
    // at the moment there's no interface to remove Reporter so we'll reuse one if exists
    /*const auto reporterExists = model.findComponent<OpenSim::TableReporter>("ik_reporter");

    bool reuse_reporter = true;
    OpenSim::TableReporter* ikReporter = nullptr;
    if (reporterExists == nullptr) {
        // Add a reporter to get IK computed coordinate values out
        ikReporter = new OpenSim::TableReporter();
        ikReporter->setName("ik_reporter");
        reuse_reporter = false;
    }
    else
        ikReporter = &model.updComponent<OpenSim::TableReporter>("ik_reporter");*/

    // define the model's internal data members and structure according to its properties, so we can use updComponentList to find all of its <Coordinate> elements
    model.finalizeFromProperties();
    auto coordinates = model.updComponentList<OpenSim::Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        /*ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());*/
        if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

/*    if (!reuse_reporter) {
        model.addComponent(ikReporter);
    }*/
    //TimeSeriesTable_<SimTK::Quaternion> quatTable(orientationsFileName);
    //std::cout <<"Loading orientations as quaternions" << std::endl;
    // Will maintain only data in time range specified by the tool
    // If unspecified {-inf, inf} no trimming is done
//    quatTable.trim(getStartTime(), getEndTime());

    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence,
        rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis,
        rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);
    //Trim to time window required by Tool
    //quatTable.trim(getStartTime(), getEndTime());



    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData =
        OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());
    OpenSim::MarkersReference mRefs{};

    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;


    // visualize for debugging
    if (visualizeResults)
        model.setUseVisualizer(true);

    SimTK::State& s0 = model.initSystem();

    double t0 = s0.getTime();

    // create the solver given the input data
    const double accuracy = 1e-4;
    OpenSim::InverseKinematicsSolver ikSolver(model, mRefs, oRefs,
        coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();
    std::shared_ptr<OpenSim::TimeSeriesTable> modelOrientationErrors(
        get_report_errors() ? new OpenSim::TimeSeriesTable()
        : nullptr);
    s0.updTime() = times[0];
    ikSolver.assemble(s0);
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    int nos = ikSolver.getNumOrientationSensorsInUse();
    SimTK::Array_<double> orientationErrors(nos, 0.0);

    if (get_report_errors()) {
        SimTK::Array_<string> labels;
        for (int i = 0; i < nos; ++i) {
            labels.push_back(ikSolver.getOrientationSensorNameForIndex(i));
        }
        modelOrientationErrors->setColumnLabels(labels);
        modelOrientationErrors->updTableMetaData().setValueForKey<string>(
            "name", "OrientationErrors");
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
    }
    if (visualizeResults) {
        model.getVisualizer().show(s0);
        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
    }
    for (auto time : times) {
        s0.updTime() = time;
        ikSolver.track(s0);
        if (get_report_errors()) {
            ikSolver.computeCurrentOrientationErrors(orientationErrors);
            modelOrientationErrors->appendRow(
                s0.getTime(), orientationErrors);
        }
        if (visualizeResults)
            model.getVisualizer().show(s0);
        else
            std::cout << "Solved at time: " << time << " s" << std::endl;;
        // realize to report to get reporter to pull values from model
        model.realizeReport(s0);
    }


    // get coordinates from state s0
    SimTK::Vector stateQ(s0.getQ());
    // get number of coordinates (joint angles) in the model
    int numCoordinates = model.getNumCoordinates();
    // initialize vector
    std::vector<double> q(numCoordinates);
    for (int j = 0; j < numCoordinates; j++) {
        // fill q with angle values for different joint angles
        q[j] = stateQ[j];
    //    std::cout << "Q" << j << ": " << q[j] << std::endl;
    }
    // set private variable q_ to equal q
    setQ(q);



/*    auto report = ikReporter->getTable();
    
    std::string modelFileName("C:/Users/wksadmin/source/repos/OpenSimLive/Config/gait2392_full_calibrated.osim");
    
    auto eix = modelFileName.rfind(".");
    auto stix = modelFileName.rfind("/") + 1;

    std::string resultsDirectory("C:/Users/wksadmin/source/repos/OpenSimLive/Results");

    OpenSim::IO::makeDir(resultsDirectory);
    std::string outName = "ik_" + modelFileName.substr(stix, eix - stix);
    std::string outputFile = resultsDirectory + "/" + outName;

    // Convert to degrees to compare with marker-based IK
    // but only for rotational coordinates
    model.getSimbodyEngine().convertRadiansToDegrees(report);
    report.updTableMetaData().setValueForKey<string>("name", outName);

    OpenSim::STOFileAdapter_<double>::write(report, outputFile + ".mot");

    //log_info("Wrote IK with IMU tracking results to: '{}'.", outputFile);
    //if (get_report_errors()) {
    //    STOFileAdapter_<double>::write(*modelOrientationErrors,
    //        resultsDirectory + "/" +
    //        getName() + "_orientationErrors.sto");
    //}
    // Results written to file, clear in case we run again
    ikReporter->clearTable();*/
}

bool IMUInverseKinematicsToolLive::run(bool visualizeResults)
{
 /*   if (_model.empty()) {
        _model.reset(new Model(get_model_file()));
    }

    runInverseKinematicsWithLiveOrientations(*_model,
        get_quat,
        visualizeResults);

    return true;*/

    runInverseKinematicsWithLiveOrientations(get_model(), get_quat(), visualizeResults);
    return true;
}
