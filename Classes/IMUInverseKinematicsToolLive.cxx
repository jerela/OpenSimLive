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
#include <XMLFunctions.h>

using namespace OpenSimLive;
using namespace SimTK;
using namespace std;

// CONSTRUCTORS

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive() {
    
}

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile) {
    // create OpenSim::Model object model_ from the .osim file given in modelFile
    model_ = OpenSim::Model(modelFile);
}

IMUInverseKinematicsToolLive::IMUInverseKinematicsToolLive(const std::string& modelFile, const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable) {
    model_ = OpenSim::Model(modelFile);
    quat_ = quatTable;
}

// DESTRUCTOR

IMUInverseKinematicsToolLive::~IMUInverseKinematicsToolLive(){
}

// PUBLIC MEMBER FUNCTIONS

// Set point tracker output format (data that is sent to the Java client) to Euler angles or quaternions
void IMUInverseKinematicsToolLive::setPointTrackerOutputFormat(const std::string& outputFormat) {
    if (getPointTrackerEnabled())
    {
        if (outputFormat == "euler") {
            setPointTrackerOutputRotation(EULER);
        }
        else if (outputFormat == "quaternion") {
            setPointTrackerOutputRotation(QUATERNION);
        }
        else {
            std::cout << "ERROR! Output format for PointTracker not recognized in IMUInverseKinematicsToolLive::setPointTrackerOutputFormat()!" << std::endl;
            abort();
        }
    }
}

// Set point tracker euler convention (space or body fixed, and order of coordinate axes), used if output rotation is set to EULER.
void IMUInverseKinematicsToolLive::setPointTrackerEulerConvention(const std::string& eulerConvention) {
    if (getPointTrackerEnabled())
    {
        // Substringify string of type "body-xyz" into "body" and "xyz"
        std::string bodyOrSpaceType = eulerConvention.substr(0,eulerConvention.find("-"));
        std::string convention = eulerConvention.substr(eulerConvention.find("-") + 1);

        // Set private variable bodyOrSpaceType_ in PointTracker accordingly
        if (bodyOrSpaceType == "body") {
            setPointTrackerBodyOrSpaceType(SimTK::BodyOrSpaceType::BodyRotationSequence);
        }
        else if (bodyOrSpaceType == "space") {
            setPointTrackerBodyOrSpaceType(SimTK::BodyOrSpaceType::BodyRotationSequence);
        }
        else {
            std::cout << "ERROR! Body or space type for PointTracker not recognized in IMUInverseKinematicsToolLive::setPointTrackerEulerConvention()!" << std::endl;
            abort();
        }

        // set variable coordinateAxes_ accordingly
        for (size_t i = 0; i < 3; ++i) {
            if (convention[i] == 'x') {
                setPointTrackerCoordinateAxes(i, SimTK::XAxis);
            }
            else if (convention[i] == 'y') {
                setPointTrackerCoordinateAxes(i, SimTK::YAxis);
            }
            else if (convention[i] == 'z') {
                setPointTrackerCoordinateAxes(i, SimTK::ZAxis);
            }
            else {
                std::cout << "ERROR! Coordinate axis letter for PointTracker not recognized in IMUInverseKinematicsToolLive::setPointTrackerEulerConvention()!" << std::endl;
                abort();
            }

        }
        

    }
}



void IMUInverseKinematicsToolLive::runInverseKinematicsWithLiveOrientations(
    OpenSim::Model& model, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable,
    const bool visualizeResults) {
    
    // load the names of the coordinates to track from MainConfiguration.xml
    trackedCoordinateNames_ = ConfigReaderVector("MainConfiguration.xml", "tracked_coordinates");
    if (trackedCoordinateNames_[0] == "none") {
        trackedCoordinateNames_.resize(0);
    }
    else {
        // resize and reserve a proper number of elements in the vector that stores coordinate values
        trackedCoordinateValues_.resize(trackedCoordinateNames_.size());
    }
    printTrackedCoordinates_ = ("true" == ConfigReader("MainConfiguration.xml", "print_tracked_coordinates"));

    // Ideally if we add a Reporter, we also remove it at the end for good hygiene but 
    // at the moment there's no interface to remove Reporter so we'll reuse one if exists
    //const auto reporterExists = model.findComponent<OpenSim::TableReporter>("ik_reporter");

    // if we want to report errors, create a new reporter
    if (get_report_errors())
    {
        // if ikReporter_ already exists, delete it and create a new one
        if (ikReporter_ != NULL) {
            try {
                //delete ikReporter_;
            }
            catch (std::exception& e) {
                std::cerr << "Error while deleting ikReporter_: " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "Unknown error while deleting ikReporter_!" << std::endl;
            }
        }
        ikReporter_ = new OpenSim::TableReporter();
        ikReporter_->setName("ik_reporter");
    }

    // define the model's internal data members and structure according to its properties, so we can use updComponentList to find all of its <Coordinate> elements
    std::cout << "Finalizing model from properties." << std::endl;
    model_.finalizeFromProperties();
    modelCoordinates_ = std::make_unique<OpenSim::ComponentList<OpenSim::Coordinate>>(model_.updComponentList<OpenSim::Coordinate>());
    
    // Hookup reporter inputs to the individual coordinate outputs
    // and lock coordinates that are translational since they cannot be
    for (auto& coord : *modelCoordinates_) {
        if (get_report_errors())
            ikReporter_->updInput("inputs").connect(coord.getOutput("value"), coord.getName());
        if (coord.getMotionType() == OpenSim::Coordinate::Translational) {
            coord.setDefaultLocked(true);
        }
    }

    // add the reporter to the model
    if (get_report_errors()) {
        model_.addComponent(ikReporter_);
    }

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
    //SimTK::Array_<double> orientationErrors(nos, 0.0);

    // reset the values for ordered vectors so that we can start the time from the beginning whenever we recalibrate
    orderedTimeVector_.clear();
    orderedIndexVector_.clear();
    // reset atomic time index
    atomicTimeIndex_ = 0;

    // draw the image in the visualizer window if desired
    if (visualizeResults) {
        // if we want to visualize results, set visualizer into sampling mode so we can update the joint angle values later
        //SimTK::Visualizer& viz = model_.updVisualizer().updSimbodyVisualizer();
        model_.updVisualizer().updSimbodyVisualizer().setMode(SimTK::Visualizer::Mode::PassThrough); // try RealTime mode instead for better FPS?
        model_.updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(100);
        model_.updVisualizer().updSimbodyVisualizer().setDesiredBufferLengthInSec(0);

        if (trackedCoordinateNames_.size() > 0) {
            // add a slider to show the values of the selected coordinates
            // loop through all coordinates that have been named by the used in visualizedJointAnglesVector
            for (unsigned int i = 0; i < trackedCoordinateNames_.size(); ++i) {
                // loop through all coordinates found on the model itself
                for (auto& coord : *modelCoordinates_) {
                    // if we find a user-named coordinate on the model, initialize its slider in the visualization window
                    if (coord.getName() == trackedCoordinateNames_[i]) {
                        double minValue = SimTK::convertRadiansToDegrees(coord.getRangeMin());
                        double maxValue = SimTK::convertRadiansToDegrees(coord.getRangeMax());
                        double defaultValue = SimTK::convertRadiansToDegrees(coord.getDefaultValue());
                        // add the slider itself
                        model_.updVisualizer().updSimbodyVisualizer().addSlider(trackedCoordinateNames_[i], i, minValue, maxValue, defaultValue);
                        std::cout << "Added slider with ID " << i << " for coordinate " << coord.getName() << std::endl;
                        break;
                    }
                }
            }
        }

        // prepare to visualize the mirrored point
        if (getPointTrackerEnabled())
        {
            try {
                startDecorationGenerator();
            }
            catch (std::exception & e) {
                std::cout << "Error: " << e.what() << std::endl;
            }
            catch (...) {
                std::cout << "Error in starting decoration generator." << std::endl;
            }
        }
        // visualize actually
        model_.getVisualizer().show(s_);
        model_.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
        model_.getVisualizer().getSimbodyVisualizer().setShowFrameRate(true);
        std::cout << "Desired visualizer frame rate is " << model_.getVisualizer().getSimbodyVisualizer().getDesiredFrameRate() << std::endl;
    }


    // push calibration data to vectors
    orderedTimeVector_.push_back(time_);
    orderedIndexVector_.push_back(0);
    // add first entry into the reporter
    s_.updTime() = atomicTimeIndex_;
    model_.realizeReport(s_);

}

// Define and add a decoration generator to the visualizer
void IMUInverseKinematicsToolLive::startDecorationGenerator() {
    // Make PointTracker construct a new decGen_
    createDecorationGenerator();
    // set visualize to true so PointTracker knows to calculate data for decoration generator
    setVisualize(true);
    // PointTracker must be run so we can obtain the location of the mirrored point
    updatePointTracker(s_);
    // realize positions for adding decoration generator to visualizer
    model_.realizePosition(s_);
    // make the decoration generator's objects adopt the coordinate system of the reference body
    decGen_->setReferenceBodyId(model_.updBodySet().get(getPointTrackerReferenceBodyName()).getMobilizedBodyIndex());
    // add decoration generator to visualizer
    model_.updVisualizer().updSimbodyVisualizer().addDecorationGenerator(decGen_);
}


/*void IMUInverseKinematicsToolLive::updateJointAngleVariable(SimTK::State& s, OpenSim::Model& model) {
    
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
}*/


std::mutex IKMutex;

// This function calculates the joint angle values for a new state s0, then updates state s with those values and redraws the visualization.
void IMUInverseKinematicsToolLive::updateInverseKinematics(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, const bool visualizeResults, bool offline) {
    // this may prevent time_ getting updated mid-IK by another thread, resulting in PointTracker from that IK to get the time from the more recent IK from another thread
    double time = time_;
    
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
    
    // check if another thread has already set the time of state s_ to a more advanced time point
    bool threadExpired = false;

    // lock a part of the code from being run by several threads in parallel
    std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);
    if (!offline) {
        // give s_ an initial time for assembling it with ikSolver_
        s_.updTime() = times[0];
    }
    else if (offline) {
        // if we're doing offline IK, make the time of the state the current time
        s_.updTime() = time;
    }
    concurrentIKMutex.unlock();

    // assemble state s_, solving the initial joint angles in the least squares sense
    ikSolver.assemble(s_);
    // create a copy of s_ so we can use methods that modify the propreties of s without modifying the properties s_ (which may be modified by another thread, leading to exceptions if multiple threads modify it in parallel)
    SimTK::State s = s_;
    if (!offline)
    {
        // update the time to be shown in the visualization and so that when we realize the report, the correct timestamp is used for the joint angle values
        s.updTime() = time;
    }
    
    // save joint angles to q_
    //updateJointAngleVariable(s_, model_);

    // show a visualization of the state
    if (visualizeResults) {
        try {
            concurrentIKMutex.lock();
            // ikTool.assemble() is using s_ and needs its time value, so we couldn't change it for s_; instead we created s for the visualization
            model_.getVisualizer().show(s);
            concurrentIKMutex.unlock();
        }
        catch (std::exception& e) {
            std::cerr << "Exception in visualizing: " << e.what() << std::endl;
        }
        catch (...) {
            std::cerr << "Unknown exception in visualizer" << std::endl;
        }
    }
    
    /*
    // update the time of s_
    if (get_report_errors()) {
        concurrentIKMutex.lock();
        // if a newer thread hasn't already gotten here, continue; otherwise set threadExpired to true
        if (lastUpdatedTime_ < time) {
            // update lastUpdatedTime_ so other, older threads that are still running can check to see if this thread has passed them
            lastUpdatedTime_ = time;
            int nos = ikSolver.getNumOrientationSensorsInUse();
            SimTK::Array_<double> orientationErrors(nos, 0.0);
            // calculate orientation errors into orientationErrors
            ikSolver.computeCurrentOrientationErrors(orientationErrors);
            try {
                // append orientationErrors into modelOrientationErrors_
                modelOrientationErrors_->appendRow(time, orientationErrors);
                // set s into a stage where report is realized; required for IK reporting
                model_.realizeReport(s);
            }
            catch (std::exception& e) {
                std::cerr << "Error in reporting IK errors: " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "Unknown error in reporting IK errors." << std::endl;
            }
        }
        else {
            threadExpired = true;
        }
        concurrentIKMutex.unlock();
        
    }*/

    // if this thread hadn't expired already before visualization, run PointTracker
    if (getPointTrackerEnabled() && !threadExpired) {
        concurrentIKMutex.lock();
        // give time to PointTracker only if we need it
        if (getSavePointTrackerResults()) {
            setPointTrackerCurrentTime(time);
        }
        updatePointTracker(s);  
        concurrentIKMutex.unlock();
    }

}



void IMUInverseKinematicsToolLive::updatePointTracker(SimTK::State s) {
    // calculate point location and orientation of its base body segment for mirror therapy
    //s_.advanceSystemToStage(SimTK::Stage::Position);
    //model_.realizePosition(s_);
    model_.updMultibodySystem().realize(s, SimTK::Stage::Position); // Required to advance (or move back) system to a stage where we can use pointTracker
    // Run PointTracker functions
    std::vector<double> trackerResults = runTracker(&s, &model_, getPointTrackerBodyName(), getPointTrackerReferenceBodyName());
    // Save the results to a private variable
    setPointTrackerPositionsAndOrientations(trackerResults);
}








// This function calculates the joint angle values for a new state s0, then updates state s with those values and redraws the visualization.
void IMUInverseKinematicsToolLive::updateOrderedInverseKinematics(OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quatTable, unsigned int orderIndex, double time, const bool visualizeResults, bool offline) {
    
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

    // create a copy of s_ so we can use methods that modify the properties of s without modifying the properties s_ (which may be modified by another thread, leading to exceptions if multiple threads modify it in parallel)
    SimTK::State s = s_;
    s.updTime() = times[0];
    //++debugCounter1_;

    bool assemblySucceeded = false;

    // assemble state s_, solving the initial joint angles in the least squares sense
    if (offline) {
        std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);
        try {
            ikSolver.assemble(s); // occasionally throws an exception "KeyNotFound" when using offline_IK_tool, but not when using OSL_core
            assemblySucceeded = true;
        }
        catch (std::exception& e) {
            std::cerr << "Assemble failed: " << e.what() << std::endl;
        }
        catch (...) {
            std::cerr << "Unknown error in assemble()" << std::endl;
        }
        concurrentIKMutex.unlock();
    }
    else {
        try {
            ikSolver.assemble(s);
            assemblySucceeded = true;
        }
        catch (std::exception& e) {
            std::cout << "Error in assembling IK solver: " << e.what() << std::endl;
        }
        catch (...) {
            std::cout << "Unknown error in assembling IK solver!" << std::endl;
        }
    }
    
    //++debugCounter2_;

    s.updTime() = time;

    // update PointTracker
    if (getPointTrackerEnabled()) {

        std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);
        // if thread hasn't expired, send data to PointTracker
        if (assemblySucceeded) {
            // give time to PointTracker only if we need it
            if (getSavePointTrackerResults()) {
                setPointTrackerCurrentTime(time);
            }
            updatePointTracker(s);
        }
        concurrentIKMutex.unlock();
    }


    // populate trackedCoordinateValues_ with joint angle values for specified coordinates
    if (trackedCoordinateNames_.size() > 0) {
        // iterate through all coordinates that the user has named
        for (unsigned int i = 0; i < trackedCoordinateNames_.size(); ++i) {
            // iterate through all coordinates defined on the model
            for (auto& coord : *modelCoordinates_) {
                // if a user-specified coordinate is found on the model, update its value according to the latest solved state
                if (coord.getName() == trackedCoordinateNames_[i]) {
                    trackedCoordinateValues_[i] = SimTK::convertRadiansToDegrees(coord.getValue(s));
                }
            }
        }
    }

    // create and print a string that reports coordinate outputs
    if (trackedCoordinateNames_.size() > 0 && printTrackedCoordinates_) {
        std::string coordinateReportString = "";
        for (unsigned int i = 0; i < trackedCoordinateNames_.size(); ++i) {
            if (i > 0) {
                coordinateReportString += ",   ";
            }
            coordinateReportString += trackedCoordinateNames_[i] + "(" + std::to_string(trackedCoordinateValues_[i]) + ")";
        }
        // now print the string, mutex to avoid several threads printing in the same line
        {
            std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);
            std::cout << "\33[2K";
            std::cout << coordinateReportString << "\r";
            std::cout.flush();
        }
    }

    // show a visualization of the state
    if (visualizeResults) {
        // update the time to be shown in the visualization
        //s.updTime() = time;
        try {
            if (assemblySucceeded) {
                std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);
                // ikTool.assemble() is using s_ and needs its time value, so we couldn't change it for s_; instead we created s for the visualization
                model_.getVisualizer().show(s);
                concurrentIKMutex.unlock();
            }
        }
        catch (std::exception& e) {
            std::cerr << "Exception in visualizing: " << e.what() << std::endl;
        }
        catch (...) {
            std::cerr << "Unknown exception in visualizer" << std::endl;
        }

        // if we are tracking any coordinates, update their sliders
        if (trackedCoordinateNames_.size() > 0) {
            // iterate through all coordinates that the user has specified
            for (unsigned int i = 0; i < trackedCoordinateNames_.size(); ++i) {
                model_.updVisualizer().updSimbodyVisualizer().setSliderValue(i, round(trackedCoordinateValues_[i]*10)/10);
            }
        }


    }


    // update the time of s_
    if (get_report_errors()) {
        std::unique_lock<std::mutex> concurrentIKMutex(IKMutex);

        // push time and order index into vectors in the order that they are written by IK threads, so not necessarily in ascending (proper) order
        orderedTimeVector_.push_back(time);
        orderedIndexVector_.push_back(orderIndex);

        try {
            // update the time of s so that the report can be realized; atomicTimeIndex_ is used instead of actual time because unlike the time, atomic time index will always be ascending; it is matched to time later when writing IK to file
            s.updTime() = ++atomicTimeIndex_;
            model_.realizeReport(s);
        }
        catch (std::exception& e) {
            std::cerr << "Error in realizing report: " << e.what() << std::endl;
        }
        catch (...) {
            std::cerr << "Unknown error in realizing report!" << std::endl;
        }

        concurrentIKMutex.unlock();

    }

}














// Write a labelled time series table of joint angle values to file.
void IMUInverseKinematicsToolLive::reportToFile() {

    // if ordered time vector has no elements (no IK points have been saved), skip this whole function
    if (orderedTimeVector_.size() == 0) {
        std::cout << "Nothing to report, skipping reporting IK to file!" << std::endl;
        return;
    }

    std::cout << "Debug counter 1: " << debugCounter1_ << ", debug counter 2: " << debugCounter2_ << std::endl;

    // organized vectors should contain the measures in time-ascending order; we must create them
    std::vector<double> organizedTimeVector;

    // organizedReportQMatrix contains the joint angle values in time-ascending order
    SimTK::Matrix_<SimTK::Real> organizedReportQMatrix(orderedTimeVector_.size(), model_.getNumCoordinates());
    
    // get the time series table collected by ikReporter_
    auto report = ikReporter_->getTable();

    // if we used updateOrderedInverseKinematics, then ordered time vector has more than zero elements
    if (orderedTimeVector_.size() > 0) {
        // index in order of IK calculations that we are looking for
        unsigned int size = orderedIndexVector_.size();

        // create a matrix that will contain the IK values for the time series table that will be saved to file
        SimTK::Matrix_<SimTK::Real> reportQ(1, model_.getNumCoordinates());

        // create a variable to contain the time for i'th element in the time series matrix
        double time;

        // loop through all calculated IK points
        for (unsigned int i = 0; i < size; ++i) {
            //++index;
            // loop through the vector of ordered indices to find the correct time
            for (unsigned int j = 0; j < size; ++j) {
                // if the currently desired table index is found, break out of the nested for-loop and update variables for the time series table
                if (i == orderedIndexVector_[j]) {
                    // get the i'th time value
                    time = orderedTimeVector_[j];
                    //std::cout << time << std::endl;
                    // get the i'th row of joint angle values
                    reportQ = report.getRow(j);
                    break;
                }
                // if we looped through all indices without finding a match, inform the user
                if (j == size - 1) {
                    std::cout << "NO MATCH!! Size of orderedIndexVector: " << orderedIndexVector_.size() << ", size of orderedTimeVector: " << orderedTimeVector_.size() << std::endl;
                }
            }

            // check if a greater than equal time already exists right before this time entry in the vector
            if (organizedTimeVector.size() > 0) {
                if (!(time > organizedTimeVector.back())) {
                    std::cout << "WARNING: Current time is not greater than the time at the back of the vector. Current time: " << time << ", previous time: " << organizedTimeVector.back() << std::endl;
                }
            }

            // push i'th time as the i'th element in organizedTimeVector
            organizedTimeVector.push_back(time);

            // populate the i'th row of organizedReportQMatrix with the right joint angle values
            organizedReportQMatrix.updRow(i) = reportQ.getAsRowVectorView();

        }
        
    }

    std::cout << "Debug counter 1: " << debugCounter1_ << ", debug counter 2: " << debugCounter2_ << std::endl;

    std::cout << "Size of organizedTimeVector: " << organizedTimeVector.size() << std::endl;
    std::cout << "Number of time points in orderedTimeVector_: " << orderedTimeVector_.size() << std::endl;
    std::cout << "Number of rows in reporter: " << report.getNumRows() << std::endl;
    std::cout << "Number of rows of organizedReportQMatrix: " << organizedReportQMatrix.nrow() << std::endl;

    // construct the time series table that will be saved to file
    OpenSim::TimeSeriesTable organizedTimeSeriesTable(organizedTimeVector, organizedReportQMatrix, report.getColumnLabels());

    // set the name of the results directory and create it
    std::string resultsDirectoryName = "OpenSimLive-results";
    OpenSim::IO::makeDir(OpenSimLiveRootDirectory_ + "/" + resultsDirectoryName);

    // convert joint angles in the report from radians to degrees
    model_.getSimbodyEngine().convertRadiansToDegrees(organizedTimeSeriesTable);
    // set the value for name (not the name of the file) in the .mot file to be created
    organizedTimeSeriesTable.updTableMetaData().setValueForKey<string>("name", outputDataName_);
    try {
        // write the .mot file to hard drive
        OpenSim::STOFileAdapter_<double>::write(organizedTimeSeriesTable, OpenSimLiveRootDirectory_ + "/" + resultsDirectoryName + "/" + outputFileName_ +".mot");
    }
    catch (std::exception& e) {
        std::cerr << "Error in saving IK output to file: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown error in saving IK output to file!" << std::endl;
    }
    // Results written to file, clear in case we run again
    ikReporter_->clearTable();
        
    if (getSavePointTrackerResults()) {
        std::cout << "Writing PointTracker output to file..." << std::endl;
        try {
            savePointTrackerOutputToFile(OpenSimLiveRootDirectory_, resultsDirectoryName);
        }
        catch (std::exception& e) {
            std::cout << "Error while saving PointTracker output to file: " << e.what() << std::endl;
        }
        catch (...) {
            std::cout << "Unknown error while saving PointTracker output to file!" << std::endl;
        }
    }

    std::cout << "IMUInverseKinematicsToolLive::reportToFile() finished." << std::endl;

}






// This function initially runs the IK and should only be called once per calibration.
bool IMUInverseKinematicsToolLive::run(const bool visualizeResults)
{
    runInverseKinematicsWithLiveOrientations(get_model(), get_quat(), visualizeResults);
    return true;
}

// This function updates the IK after it's been initially run
void IMUInverseKinematicsToolLive::update(const bool visualizeResults, const bool offline)
{
    updateInverseKinematics(get_quat(), visualizeResults, offline);
}

// This function updates the IK after it's been initially run
void IMUInverseKinematicsToolLive::update(const bool visualizeResults, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quat, const bool offline)
{
    updateInverseKinematics(quat, visualizeResults, offline);
}

// This function updates the IK after it's been initially run
void IMUInverseKinematicsToolLive::updateOrdered(const bool visualizeResults, OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quat, unsigned int orderIndex, double time, const bool offline)
{
    updateOrderedInverseKinematics(quat, orderIndex, time, visualizeResults, offline);
}
