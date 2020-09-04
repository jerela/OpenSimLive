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
#include <mutex>
#include <DecorationGeneratorLive.h>

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

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable) {
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

IMUInverseKinematicsToolLive::~IMUInverseKinematicsToolLive(){
}

// PUBLIC MEMBER FUNCTIONS

void IMUInverseKinematicsToolLive::runInverseKinematicsWithLiveOrientations(
    OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable,
    const bool visualizeResults) {
    

    // Ideally if we add a Reporter, we also remove it at the end for good hygiene but 
    // at the moment there's no interface to remove Reporter so we'll reuse one if exists
    //const auto reporterExists = model.findComponent<OpenSim::TableReporter>("ik_reporter");

    if (get_report_errors())
    {
        ikReporter_ = new OpenSim::TableReporter();
        ikReporter_->setName("ik_reporter");
    }

    // define the model's internal data members and structure according to its properties, so we can use updComponentList to find all of its <Coordinate> elements
    std::cout << "Finalizing model from properties." << std::endl;
    model_.finalizeFromProperties();
    auto coordinates = model_.updComponentList<OpenSim::Coordinate>();

    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : coordinates) {
        if (get_report_errors())
            ikReporter_->updInput("inputs").connect(coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    if (get_report_errors())
        model_.addComponent(ikReporter_);

    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence,
        rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis,
        rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);

    // convert quaternions to rotation data type
    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());
    OpenSim::MarkersReference mRefs{};

    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;

    if (visualizeResults){
        model_.setUseVisualizer(true);
    }

    std::cout << "Initializing system." << std::endl;
    try {
        s_ = model_.initSystem(); // this creates the visualizer window; consists of buildSystem() and initializeState()
    }
    catch (std::exception& e) { std::cout << e.what(); }
    catch (...) { std::cout << "Initialization exception!" << std::endl; }
    std::cout << "System initialized." << std::endl;
    
    // create the solver given the input data
    OpenSim::InverseKinematicsSolver ikSolver(model_, mRefs, oRefs, coordinateReferences);
    ikSolver.setAccuracy(accuracy_);

    auto& times = oRefs.getTimes();
    
    // set the initial time for state s_
    s_.updTime() = times[0];
    // assemble the state
    ikSolver.assemble(s_);
    // Create place holder for orientation errors, populate based on user pref.
    // according to report_errors property
    int nos = ikSolver.getNumOrientationSensorsInUse();
    SimTK::Array_<double> orientationErrors(nos, 0.0);

    // draw the image in the visualizer window if desired
    if (visualizeResults) {
        // if we want to visualize results, set visualizer into sampling mode so we can update the joint angle values later
        //SimTK::Visualizer& viz = model_.updVisualizer().updSimbodyVisualizer();
        model_.updVisualizer().updSimbodyVisualizer().setMode(SimTK::Visualizer::Mode::PassThrough); // try RealTime mode instead for better FPS?
        model_.updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(60);
        // prepare to visualize the mirrored point
        startDecorationGenerator();
        // visualize actually
        model_.getVisualizer().show(s_);
        model_.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        model_.getVisualizer().getSimbodyVisualizer().setShowFrameRate(true);
        std::cout << "Desired visualizer frame rate is " << model_.getVisualizer().getSimbodyVisualizer().getDesiredFrameRate() << std::endl;
    }

//    s_.updTime() = time_;

    if (get_report_errors()) {
        modelOrientationErrors_ = new OpenSim::TimeSeriesTable();
        SimTK::Array_<string> labels;
        for (int i = 0; i < nos; ++i) {
            labels.push_back(ikSolver.getOrientationSensorNameForIndex(i));
        }
        modelOrientationErrors_->setColumnLabels(labels);
        modelOrientationErrors_->updTableMetaData().setValueForKey<string>("name", "OrientationErrors");
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
        modelOrientationErrors_->appendRow(s_.getTime(), orientationErrors);
        model_.realizeReport(s_);
    }

    //updateJointAngleVariable(s_, model_);

}

// Define and add a decoration generator to the visualizer
void IMUInverseKinematicsToolLive::startDecorationGenerator() {
    // Make PointTracker construct a new decGen_
    createDecorationGenerator();
    // set visualize to true so PointTracker knows to calculate data for decoration generator
    setVisualize(true);
    // PointTracker must be run so we can obtain the location of the mirrored point
    updatePointTracker();
    // realize positions for adding decoration generator to visualizer
    model_.realizePosition(s_);
    // make the decoration generator's objects adopt the coordinate system of the reference body
    decGen_->setReferenceBodyId(model_.updBodySet().get(getPointTrackerReferenceBodyName()).getMobilizedBodyIndex());
    // add decoration generator to visualizer
    model_.updVisualizer().updSimbodyVisualizer().addDecorationGenerator(decGen_);
}


void IMUInverseKinematicsToolLive::updateJointAngleVariable(SimTK::State& s, OpenSim::Model& model) {
    
    // get coordinates from state s
    SimTK::Vector stateQ(s.getQ());
    // get number of coordinates (joint angles) in the model
    int numCoordinates = model.getNumCoordinates();
    // initialize vector that holds the joint angle values
    std::vector<double> q(numCoordinates);
    for (int j = 0; j < numCoordinates; j++) {
        // fill q with angle values for different joint angles
        q[j] = stateQ[j];
    }

    // set private variable q_ to equal q so that we can get the latest joint angle values with getQ
    setQ(q);
}


std::mutex m;

// This function calculates the joint angle values for a new state s0, then updates state s with those values and redraws the visualization.
void IMUInverseKinematicsToolLive::updateInverseKinematics(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, const bool visualizeResults) {
    
    // Convert to OpenSim Frame
    const SimTK::Vec3& rotations = get_sensor_to_opensim_rotations();
    SimTK::Rotation sensorToOpenSim = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0], SimTK::XAxis, rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);

    // Rotate data so Y-Axis is up
    OpenSim::OpenSenseUtilities::rotateOrientationTable(quatTable, sensorToOpenSim);

    // convert quaternion orientation data of IMUs to rotation matrix form
    OpenSim::TimeSeriesTable_<SimTK::Rotation> orientationsData = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(quatTable);

    OpenSim::OrientationsReference oRefs(orientationsData, &get_orientation_weights());
    OpenSim::MarkersReference mRefs{};

    SimTK::Array_<OpenSim::CoordinateReference> coordinateReferences;
    
    // create the solver given the input data
    OpenSim::InverseKinematicsSolver ikSolver(model_, mRefs, oRefs, coordinateReferences);
    ikSolver.setAccuracy(accuracy_);

    // set the time of state s0
    auto& times = oRefs.getTimes();
    std::unique_lock<std::mutex> concurrentIKMutex(m);
    s_.updTime() = times[0];
    concurrentIKMutex.unlock();
    // assemble state s0, solving the initial joint angles in the least squares sense
    concurrentIKMutex.lock();
    ikSolver.assemble(s_);

    // save joint angles to q_
    //updateJointAngleVariable(s_, model_);

    // update the time to be shown in the visualization and so that when we realize the report, the correct timestamp is used for the joint angle values
    s_.updTime() = time_;
    // now insert q into the original visualized state and show them
    if (visualizeResults) {
        try {
            model_.getVisualizer().show(s_);
        }
        catch (std::exception& e) {
            std::cerr << "Exception in visualizing: " << e.what() << std::endl;
        }
        catch (...) {
            std::cerr << "Unknown exception in visualizer" << std::endl;
        }
    }
    concurrentIKMutex.unlock();
    //model_.getVisualizer().getSimbodyVisualizer().drawFrameNow(s_);

    // update the time of s_
    if (get_report_errors()) {
        int nos = ikSolver.getNumOrientationSensorsInUse();
        SimTK::Array_<double> orientationErrors(nos, 0.0);
        // calculate orientation errors into orientationErrors
        ikSolver.computeCurrentOrientationErrors(orientationErrors);
        // append orientationErrors into modelOrientationErrors_
        concurrentIKMutex.lock();
        modelOrientationErrors_->appendRow(s_.getTime(), orientationErrors);
        model_.realizeReport(s_);
        concurrentIKMutex.unlock();
    }

    if (getPointTrackerEnabled() == true) {
        concurrentIKMutex.lock();
        updatePointTracker();
        concurrentIKMutex.unlock();
    }

}



void IMUInverseKinematicsToolLive::updatePointTracker() {
    // calculate point location and orientation of its base body segment for mirror therapy
    //s_.advanceSystemToStage(SimTK::Stage::Position);
    //model_.realizePosition(s_);
    model_.updMultibodySystem().realize(s_, SimTK::Stage::Position); // Required to advance (or move back) system to a stage where we can use pointTracker
    // Run PointTracker functions
    setPointTrackerCurrentTime(time_);
    std::vector<double> trackerResults = runTracker(&s_, &model_, getPointTrackerBodyName(), getPointTrackerReferenceBodyName());
    // Save the results to a private variable
    setPointTrackerPositionsAndOrientations(trackerResults);
}






void IMUInverseKinematicsToolLive::reportToFile() {

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
    OpenSim::STOFileAdapter_<double>::write(*modelOrientationErrors_, OpenSimLiveRootDirectory_ + "/" + resultsDirectoryName + "/" + "IK-live" + "_orientationErrors.sto");
    // Results written to file, clear in case we run again
    ikReporter_->clearTable(); 

    if (getSavePointTrackerResults()) {
        std::cout << "Writing PointTracker output to file..." << std::endl;
        savePointTrackerOutputToFile(OpenSimLiveRootDirectory_, resultsDirectoryName);
    }
    
}






// This function initially runs the IK and should only be called once per calibration.
bool IMUInverseKinematicsToolLive::run(const bool visualizeResults)
{
    runInverseKinematicsWithLiveOrientations(get_model(), get_quat(), visualizeResults);
    return true;
}

// This function updates the IK after it's been initially run
void IMUInverseKinematicsToolLive::update(const bool visualizeResults)
{
    updateInverseKinematics(get_quat(), visualizeResults);
}

