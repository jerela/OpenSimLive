# OpenSimLive

- [Getting started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installing](#installing)
  * [Running the tests](#running-the-tests)
- [How it works](#how-it-works)
- [Troubleshooting and FAQ](#troubleshooting-and-faq)
  * [General questions](#general-questions)
  * [Run-time issues](#run-time-issues)
- [Authors](#authors)
- [License](#license)
- [Acknowledgments](#acknowledgments)
<!-- toc -->

OpenSimLive is a C++ package that streams orientation data from Xsens MTw Awinda inertial measurement units and calculates inverse kinematics based on that data. It relies on OpenSim for biomechanical analyses and related tools. The current version uses OpenSim 4.1 API and XDA 4.6.

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

XDA 4.6 and OpenSim 4.1 are required for core functionality. CMake and Visual Studio are used to configure, generate and build the project. Vicon Datastream SDK 1.10 and Vicon Nexus 2.10+ are required if you wish to perform inverse kinematics on live marker data, but are otherwise optional.

```
Example
```

### Installing

Step by step instructions on how to install this project.

#### Windows (64-bit)

1. Download and unzip the package to a directory on your hard drive.
2. Open CMake and select that directory as the source code directory.
- Put YourFilePath/YourSourceCodeFolder-build or whatever else you want as the build folder and allow CMake to create a new folder when prompted.
- Select **x64** as the generator when prompted.
- Select **Configure**. CMake variables and their values should now be displayed. If any of them are not found, you can manually type in folders. An example file path of each is as follows:
   - CONFIG_PATH:               C:/Users/YourUserHere/Documents/OpenSimLive/Config
   - INCLUDE_CLASSES_PATH:      C:/Users/YourUserHere/Documents/OpenSimLive/Classes
   - INCLUDE_FUNCTIONS_PATH:    C:/Users/YourUserHere/Documents/OpenSimLive/Functions
   - MTSDK_PATH:                C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/include
   - MT_LIB_PATH:               C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/lib
   - OPENSIM_INCLUDE_PATH:      C:/OpenSim 4.1/sdk/include/OpenSim
   - OPENSIM_INCLUDE_PATH_TWO:  C:/OpenSim 4.1/sdk/include
   - OPENSIM_LIB_PATH:          C:/OpenSim 4.1/sdk/lib
   - SIMBODY_INCLUDE_PATH:      C:/OpenSim 4.1/sdk/Simbody/include
   - SIMBODY_LIB_PATH:          C:/OpenSim 4.1/sdk/Simbody/lib
   - VICONSDK_LIB:              C:/Program Files/Vicon/DataStream SDK/Win64/CPP/ViconDataStreamSDK_CPP.lib
   - VICONSDK_PATH:             C:/Program Files/Vicon/DataStream SDK/Win64/CPP
- The two last entries are not required for IMU IK calculations.
- Finally, select **Generate**.
3. Open Visual Studio. Open the solution you just generated in the build directory. Build **ALL_BUILD**. Visual Studio should now create the required executable(s) in a subdirectory in the build directory.
4. Copy **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib to the directory where **XsensReader.exe** is.
5. Go to .../OpenSimLive/Config and make sure the .xml files have the right values for your directory paths.
6. Installation complete. You are ready to run **XsensReader.exe**.


### Running the tests

There are currently two tests, **test_IK_speed** and **test_IK_speed_multithread**. They are found in OpenSimLive/Tests/ in the source directory and in a similar separate Tests folder in the build directory. Note that **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib must be copied to the folder where the test executables are.

**test_IK_speed** measures how many inverse kinematics operations are performed over a time that the user inputs in seconds.
**test_IK_speed_multithread** measures how many inverse kinematics operations are performed over a time that the user inputs in seconds. The user also inputs the number of threads to be used.

Both tests calculate point position tracking operations for mirror therapy if **station_parent_body** in OpenSimLive/Config/MainConfiguration.xml is not set to *none*. In the case of **test_IK_speed_multithread**, these operations are also multithreaded.

When the tests finish, results are printed on the command prompt. They include how many IK (and possibly point tracking) operations were performed in total, how much time was elapsed and how many operations were performed on average per second.



## How it works

IMU-based inverse kinematics and mirror therapy applications rely on a number of classes. The most important ones and some of their member functions are highlighted below.

### IMUPlacerLive

This class calibrates an OpenSim model (XML file saved as .osim that contains joint angle definitions, constraints etc) by taking applying the orientations of IMUs at the calibration time point and placing IMUs on the bodies of the model. Config/IMUPlacerSetup.xml contains information about the model file that is calibrated, the file name that the calibrated model will be saved as, information about the initial heading of the "base" IMU and orientation transformations between IMU and OpenSim coordinates. Config/SensorMappings.xml describes which IMU corresponds to which body on the OpenSim model.

IMUPlacerLive is based on the [IMUPlacer class from OpenSim 4.1](https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1IMUPlacer.html) and inherits it. It also inherits PointTracker, which is used to call inherited public methods through IMUPlacerLive.

### IMUInverseKinematicsToolLive

This class calculates inverse kinematics on the model based on IMU orientation data. It's based on the IMUInverseKinematicsTool class from an unpublished version of OpenSim that follows OpenSim 4.1. It supports multithreading, which is why it utilizes mutex locks in some of its methods. It inherits PointTracker, which is used to perform calculations related to mirror therapy when the state of the system is suitable.

#### IMUInverseKinematicsToolLive::runInverseKinematicsWithLiveOrientations()

This method is used to calculate the initial IK and establish the initial state of the system. It is supposed to be called only once per calibration, directly after the calibration, and it calculates the inverse kinematics at the initial time point. Subsequent IK calculations are done with IMUInverseKinematicsToolLive::updateInverseKinematics().
This method is private and should be invoked from other classes by IMUInverseKinematicsToolLive::run().

#### IMUInverseKinematicsToolLive::updateInverseKinematics()

This method is used to calculate inverse kinematics repeatedly. It can be run concurrently by different threads. If PointTracker is enabled, PointTracker calculations are performed at the end of this method.
This method is private and should be invoked from other classes by IMUInverseKinematicsToolLive::update().

### PointTracker

This class takes the position and parent body of a point in the model and a reference body as input. The position of the point is then expressed in the reference body's coordinate system and reflected with respect to the z axis of that coordinate system. The orientation of the point's parent body is similarly mirrored. The class is independent in the sense that it doesn't inherit other classes and isn't based on any existing classes, although its methods make extensive use of the OpenSim 4.1 API.

#### PointTracker::runTracker()

This method is the "main" method of this class and calls other methods to perform individual parts of the calculations. These calculations sometimes require writable references to OpenSim::Model and SimTK::State, which is why the method has to be invoked in IMUInverseKinematisToolLive::updateInverseKinematics().

#### PointTracker::addStationToBody()

This method is used by IMUPlacerLive during calibration to modify the XML file containing the OpenSim model. It adds XML elements describing the point of interest in mirror therapy to the OpenSim model.

### XsensDataReader

This class establishes the connection to Xsens MTw Awinda inertial measurement units, acquires orientation data from them and closes the connection.

#### XsensDataReader::GetQuaternionData()

This method takes a vector of XsQuaternion objects as its input. If any IMUs have new data, it updates the quaternion orientation values in the vector for that IMU. In any case the input vector is returned.

### ThreadPoolContainer

This class controls the number of worker threads during multithreading and works as an interface to access the ThreadPool class. When constructed with an integer parameter N, ThreadPoolContainer creates a ThreadPool object with N worker threads. This class ensures that the user has control over how many worker threads run at a time.
This class is not necessary if multithreading is not used.

#### ThreadPoolContainer::offerFuture()

This method sends a function to the thread pool. If there are N worker threads running already, the function will wait in queue until the oldest of them has finished, then it will be given to the worker thread. If there are less than N worker threads running, the function is immediately given to a vacant worker thread. The elements of the thread pool are managed in a vector.

## Troubleshooting and FAQ

### General questions

#### What operating systems are supported by OpenSimLive?

Currently OpenSimLive has been tested with 64-bit Windows 7 and 64-bit Windows 10.

### Run-time issues

#### The program crashes right after printing "IMUPlacerLive run initiated"

Make sure that **IMUPlacerSetup.xml** in OpenSimLive/Config/ has the correct file path for *model_file*. The file path should include the whole directory starting from the letter of your hard disk drive. Make sure that the same .osim file exists in OpenSimLive/Config/.

#### The program crashes right after printing "Loaded model ... from file ..."

Make sure that **IMUPlacerSetup.xml** in OpenSimLive/Config/ has the correct value for **base_imu_label**. The value should equal one of the *name_in_model* values in **SensorMappings_full.xml** in OpenSimLive/Config/. Additionally make sure that the IMU in question is in use.

#### The program is stuck after printing "Waiting for ack..."

OpenSimLive is waiting on client program that is receiving data to acknowledge it received data. This is an issue with data not getting back to OpenSimLive, likely because the client has stopped working. The issue is therefore in the client program and not in OpenSimLive.

## Authors

Jere Lavikainen, jere.lavikainen (at) uef.fi

## License

## Acknowledgments

Thanks to GitHub user PurpleBooth for providing an excellent [readme template](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2).
Thanks to Keith Vertanen for his [Java and C++ socket class](https://www.keithv.com/software/socket/).
Thanks to Jacob Progsch and Václav Zeman for their [thread pool class](https://github.com/progschj/ThreadPool).
