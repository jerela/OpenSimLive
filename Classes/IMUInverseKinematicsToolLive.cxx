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
#include <OpenSim.h>

using namespace OpenSimLive;
using namespace SimTK;
using namespace std;

// CONSTRUCTORS

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive() {
    
}

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile) {
    // create OpenSim::Model object model_ from the .osim file given in modelFile
    model_ = OpenSim::Model(modelFile);
    //model_.finalizeFromProperties();

    //ikReporter_->setName("ik_reporter");

    //auto coordinates = model_.updComponentList<OpenSim::Coordinate>();
    //for (auto& coord : coordinates) {
    //    ikReporter_->updInput("inputs").connect( coord.getOutput("value"), coord.getName() );
    //    if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
    //        coord.setDefaultLocked(true);
    //    }
    //}
    //s_ = model_.initSystem();
}

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable) {
    model_ = OpenSim::Model(modelFile);
    //model_.finalizeFromProperties();

    //ikReporter_->setName("ik_reporter");

    //auto coordinates = model_.updComponentList<OpenSim::Coordinate>();
    //for (auto& coord : coordinates) {
    //    ikReporter_->updInput("inputs").connect(coord.getOutput("value"), coord.getName());
    //    if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
    //        coord.setDefaultLocked(true);
    //    }
    //}
    //s_ = model_.initSystem();
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
    //const auto reporterExists = model.findComponent<OpenSim::TableReporter>("ik_reporter");

    //bool reuse_reporter = true;
    //OpenSim::TableReporter* ikReporter = nullptr;
    //if (reporterExists == nullptr) {
        // Add a reporter to get IK computed coordinate values out
        ikReporter_ = new OpenSim::TableReporter();
        ikReporter_->setName("ik_reporter");
        //reuse_reporter = false;
    //}
    //else
     //   ikReporter = &model.updComponent<OpenSim::TableReporter>("ik_reporter");

    // define the model's internal data members and structure according to its properties, so we can use updComponentList to find all of its <Coordinate> elements
    std::cout << "Finalizing model from properties." << std::endl;
    model_.finalizeFromProperties();
    auto coordinates = model_.updComponentList<OpenSim::Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        ikReporter_->updInput("inputs").connect(coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    //if (!reuse_reporter) {
        model_.addComponent(ikReporter_);
    //}
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



    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());
    OpenSim::MarkersReference mRefs{};

    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;

    if (visualizeResults)
        model_.setUseVisualizer(true);

    std::cout << "Initializing system." << std::endl;
    try {
        s_ = model_.initSystem(); // this creates the visualizer window; consists of buildSystem() and initializeState()
    }
    catch (std::exception& e) { std::cout << e.what(); }
    catch (...) { std::cout << "Initialization exception!" << std::endl; }
    //std::cout << "System initialized." << std::endl;

    // if we want to visualize results, set visualizer into sampling mode so we can update the joint angle values later
    if (visualizeResults) {
        //*viz = model_.updVisualizer().updSimbodyVisualizer();
        SimTK::Visualizer& viz = model_.updVisualizer().updSimbodyVisualizer();
        viz.setMode(SimTK::Visualizer::Mode::Sampling); // try RealTime mode instead for better FPS?
    }
    
    // create the solver given the input data
    const double accuracy = 1e-4;
    OpenSim::InverseKinematicsSolver ikSolver(model_, mRefs, oRefs, coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();
    //std::shared_ptr<OpenSim::TimeSeriesTable> modelOrientationErrors(
    //    get_report_errors() ? new OpenSim::TimeSeriesTable()
    //    : nullptr);
    if (get_report_errors())
        modelOrientationErrors_ = new OpenSim::TimeSeriesTable();
    // set the initial time for state s_
    s_.updTime() = times[0];
    // assemble the state
    ikSolver.assemble(s_);
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    int nos = ikSolver.getNumOrientationSensorsInUse();
    SimTK::Array_<double> orientationErrors(nos, 0.0);

    if (get_report_errors()) {
        SimTK::Array_<string> labels;
        for (int i = 0; i < nos; ++i) {
            labels.push_back(ikSolver.getOrientationSensorNameForIndex(i));
        }
        modelOrientationErrors_->setColumnLabels(labels);
        modelOrientationErrors_->updTableMetaData().setValueForKey<string>(
            "name", "OrientationErrors");
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
    }

    // draw the image in the visualizer window if desired
    if (visualizeResults) {
        model_.getVisualizer().show(s_);
        model_.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        model_.getVisualizer().getSimbodyVisualizer().setShowFrameRate(true);
    }
    std::cout << "Desired visualizer frame rate is " << model_.getVisualizer().getSimbodyVisualizer().getDesiredFrameRate() << std::endl;

    // track the state at time defined for it
    ikSolver.track(s_);
    // set state time as time after calibration
    s_.updTime() = time_;
    if (get_report_errors()) {
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
        modelOrientationErrors_->appendRow(s_.getTime(), orientationErrors);
    }
    if (visualizeResults) {
        model_.getVisualizer().show(s_);
    }
    model_.realizeReport(s_);

    // get coordinates from state s_
    SimTK::Vector stateQ(s_.getQ());
    // get number of coordinates (joint angles) in the model
    int numCoordinates = model_.getNumCoordinates();
    // initialize vector that holds the joint angles
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








// This function calculates the joint angle values for a new state s0, then updates state s with those values and redraws the visualization.
void IMUInverseKinematicsToolLive::updateInverseKinematics(OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, bool visualizeResults) {

    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);

    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());
    OpenSim::MarkersReference mRefs{};

    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;

    // make sure that we don't create an additional visualization window when we initialize system
    model.setUseVisualizer(false);
    SimTK::State& s0 = model.initSystem();
    //std::cout << model.updBodySet().get("femur_r").findStationLocationInAnotherFrame(s0, { 0,0,0 }, model.getGround()) << std::endl;
    //std::cout << "s0 after initSystem: " << s0.getSystemStage() << std::endl;
    //std::cout << "s_ :" << s_.getSystemStage() << std::endl;
    // create the solver given the input data
    const double accuracy = 1e-4;
    OpenSim::InverseKinematicsSolver ikSolver(model, mRefs, oRefs, coordinateReferences);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs.getTimes();
    std::shared_ptr<OpenSim::TimeSeriesTable> modelOrientationErrors(
        get_report_errors() ? new OpenSim::TimeSeriesTable()
        : nullptr);

    // set the time of state s0
    s0.updTime() = times[0];
    //std::cout << "s0 after updTime: " << s0.getSystemStage() << std::endl;
    //std::cout << "s_ :" << s_.getSystemStage() << std::endl;
    // assemble state s0, solving the initial joint angles in the least squares sense
    ikSolver.assemble(s0);
    //std::cout << "s0 after assemble: " << s0.getSystemStage() << std::endl;
    //std::cout << "s_ :" << s_.getSystemStage() << std::endl;
    
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    int nos = ikSolver.getNumOrientationSensorsInUse();
    SimTK::Array_<double> orientationErrors(nos, 0.0);
    // compute the orientation errors and save them into orientationErrors
    ikSolver.computeCurrentOrientationErrors(orientationErrors);

    // get coordinates from state s0
    SimTK::Vector stateQ(s0.getQ());
    // get number of coordinates (joint angles) in the model
    int numCoordinates = model.getNumCoordinates();
    // initialize vector that holds the joint angle values
    std::vector<double> q(numCoordinates);
    for (int j = 0; j < numCoordinates; j++) {
        // fill q with angle values for different joint angles
        q[j] = stateQ[j];
        //    std::cout << "Q" << j << ": " << q[j] << std::endl;
        // insert the joint angle values calculated from state s0 into state s, which we use for visualization
        s_.updQ()[j] = q[j];
    }

    // set private variable q_ to equal q so that we can get the latest joint angle values with getQ
    setQ(q);

    // calculate point location and orientation of its base body segment for mirror therapy
    //std::cout << "CHK1" << std::endl;
    ikSolver.track(s0);
    model.realizePosition(s0);
    //std::cout << "s0: " << s0.getSystemStage() << std::endl;
    //std::cout << "s_: " << s_.getSystemStage() << std::endl;
    try {
        std::vector<double> trackerResults = runTracker(&s0, &model, "femur_r", "pelvis", { 0, 0, 0 });
        setPointTrackerPositionsAndOrientations(trackerResults);
    }
    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "bloop" << std::endl;
    }
    //std::cout << "CHK3" << std::endl;



    // update the time to be shown in the visualization
    s_.updTime() = time_;
    // now insert q into the original visualized state and show them
    //model_.getVisualizer().getSimbodyVisualizer().flushFrames();
    model_.getVisualizer().show(s_);
    //model_.getVisualizer().getSimbodyVisualizer().drawFrameNow(s_);

    // update the time of s0 so that when we realize the report, the correct timestamp is used for the joint angle values
    s0.updTime() = time_;
    if (get_report_errors()) {
        // calculate orientation errors into orientationErrors
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
        // append orientationErrors into modelOrientationErrors_
        modelOrientationErrors_->appendRow(s0.getTime(), orientationErrors);
    }

    //std::cout << model.updBodySet().get("femur_r").findStationLocationInAnotherFrame(s0, { 0,0,0 }, model.getGround()) << std::endl;

    // update IK values to report for time defined for s0
    model_.realizeReport(s0);

}



void IMUInverseKinematicsToolLive::reportToFile() {

    //model_.addComponent(ikReporter_);
    auto report = ikReporter_->getTable();
    // set the name of the results directory and create it
    std::string resultsDirectoryName = "OpenSimLive-results";
    OpenSim::IO::makeDir(OpenSimLiveRootDirectory_+ "/" + resultsDirectoryName);

    // convert joint angles in the report from radians to degrees
    model_.getSimbodyEngine().convertRadiansToDegrees(report);
    // set the value for name (not the name of the file) in the .mot file to be created
    report.updTableMetaData().setValueForKey<string>("name", "IK-live");
    // write the .mot file to hard drive
    OpenSim::STOFileAdapter_<double>::write(report, OpenSimLiveRootDirectory_ + "/" + resultsDirectoryName + "/" + "IK-live.mot");




    if (get_report_errors()) {
            OpenSim::STOFileAdapter_<double>::write(*modelOrientationErrors_, OpenSimLiveRootDirectory_ + "/" + resultsDirectoryName + "/" + "IK-live" + "_orientationErrors.sto");
        }
        // Results written to file, clear in case we run again
    ikReporter_->clearTable(); 





}






// This function initially runs the IK and should only be called once per calibration.
bool IMUInverseKinematicsToolLive::run(bool visualizeResults)
{
    runInverseKinematicsWithLiveOrientations(get_model(), get_quat(), visualizeResults);
    return true;
}

// This function updates the IK after it's been initially run
bool IMUInverseKinematicsToolLive::update(bool visualizeResults)
{
    updateInverseKinematics(get_model(), get_quat(), visualizeResults);
    return true;
}
