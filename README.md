# OpenSimLive

- [Getting started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installing](#installing)
  * [Running the tests](#running-the-tests)
- [How it works](#how-it-works)
- [Descriptions of files](#descriptions-of-files)
- [Troubleshooting and FAQ](#troubleshooting-and-faq)
  * [General questions](#general-questions)
  * [Run-time issues](#run-time-issues)
- [Authors](#authors)
- [License](#license)
- [Acknowledgments](#acknowledgments)
<!-- toc -->


OpenSimLive is a C++ package that streams orientation data from inertial measurement units and calculates inverse kinematics based on that data. It relies on OpenSim for biomechanical analyses and related tools. The current version uses OpenSim 4.1 API. Two types of IMUs are supported: Xsens MTw Awinda and Delsys Trigno Avanti. Xsens IMUs use XDA 4.6 and Delsys IMUs use Delsys Trigno Control Utility.

Some of OpenSimLive's features can be tested without actual IMUs by using fake IMU data that that OpenSimLive generates as random unit quaternions.

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

OpenSim 4.1 is required for core functionality. XDA 4.6 is required to use Xsens IMUs. Delsys Trigno Control Utility is required to communicate with Delsys SDK and read data from Delsys IMUs. CMake and Visual Studio are used to configure, generate and build the project.

Python 3.7+ is optional and enables using the PythonPlotter class, which can be used to plot OpenSimLive data.

```
Example
```

### Installing

Step by step instructions on how to install this project.

#### Windows (64-bit)

1. Download and unzip the package to a directory on your hard drive.
2. Open CMake and select the directory from the previous step as the source code directory.
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
   - PYTHON_LIB:				  C:/Users/YourUserHere/AppData/Local/Programs/Python38/libs
   - PYTHON_PATH:				  C:/Users/YourUserHere/AppData/Local/Programs/Python38
- The two last entries are not required unless you wish to plot EMG data with Delsys IMUs.
- Finally, select **Generate**.
3. Open Visual Studio. Open the solution you just generated in the build directory. Build **ALL_BUILD**. Visual Studio should now create the required executable(s) in a subdirectory in the build directory. It will probably be **.../BuildFolderName/MirrorTherapy/RelWithDebInfo/** for mirror therapy applications and **.../BuildFolderName/Tests/RelWithDebInfo/** for tests.
4. Copy **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib to the directories where **OSL_common.exe** and **test_IK_speed** are or add their locations to the *PATH* environmental variable. **This is required to run Xsens-related scripts, but is otherwise optional.**
5. Go to **.../OpenSimLive/Config** and make sure the .xml files have the right values for your directory paths.
6. Installation complete. You are ready to run OpenSimLive.

### Running the program

The main mirror therapy program (**OSL_common**) and most other programs and tests are controlled by keyboard input. Calibrating a model will open a visualization in another window, so make sure you select the command console window as the active window to ensure keyboard input is successfully read.

If you have set IMU manufacturer as "xsens" in MainConfiguration.xml, then when the program starts, it will look for active Xsens IMUs. Make sure your IMUs are on and not on standby mode (move them until a red light starts blinking). The program will list all IMUs it can connect to in a numbered order and finding an IMU may take a few seconds. When the program lists all the IMUs you want to use, press Y on your keyboard to continue.

If you have set IMU manufacturer as "delsys" in MainConfiguration.xml, then the program will start reading data stream from Delsys Trigno Control Utility.

Finally, you can run the program without any actual IMUs by setting manufactured as "simulated", in which case the program will create random quaternions for IMU orientations. The visualization of the skeletal model in this case will not look sensible, but will instead change to random and sometimes unnatural orientations with each new frame.

If **station_parent_body** in **Config/MainConfiguration.xml** is set to anything but "none", the program (acting as a server for socket communication) will wait for another program (the client) to connect to it. For testing purposes, you can run **JavaClient/run.bat** at this point to execute a program that receives data from the server. When the connection is made, the main program will automatically continue. Make sure that *socket_port* in **.../OpenSimLive/Config/MainConfiguration.xml** matches the first line of **.../OpenSimLive/JavaClient/conf.txt** so that the client can connect to the server with the right port.

The program should print input instructions in the command window. Pressing L will set a reference orientation for the base segment IMU and pressing C will calibrate the model.
Setting a reference orientation with L is not necessary, but is used to acknowledge differences in coordinate system rotations between the base body on the model and the rehabilitation robot. When L is pressed, the IMU on the base segment should be facing the same way as the robot's coordinate system. For example, if the robot is mounted on a wall 180 degrees opposite to the patient during the rehabilitation session, then the base segment IMU should first be placed on a surface that is parallel to the base of the robot and in a way that the IMU would be on the patient's base segment if the patient was sitting with their back to the base of the robot. After reference orientation is set, the IMU should be placed back on the patient's base segment and the model should then be calibrated. During inverse kinematics the current position of the base segment IMU is then used to correct the mirrored rotations and positions of the body to be rehabilitated, before they are sent to the client.
If a reference orientation is not set, during inverse kinematics the program will assume that there is a 180 degree rotation around the vertical axis between coordinate systems of the robot and the base segment of the patient, but position data won't be rotated. Therefore only rotation data is corrected in this case.

To enable sending data to the client, you must press V. Pressing B will disable this feature.

After the model is calibrated, you can enable continuous inverse kinematics and rotation/position mirroring operations with N and disable them with M. The visualization window will show the solved joint angles on the model. If data sending is enabled, the calculated data is automatically sent to the client. You can also press Z to calculate IK and rotation/position mirroring operations at a single time point.

Delsys sensors will allow you to measure EMG. Pressing A will begin EMG measurement and S will pause it.

When you are finished, pressing X will quit the program. At this point the program will save IK results, IK errors and calculated mirrored positions and rotations to **IK-live.mot**, **IK-live_orientationErrors.sto** and **PointTrackerOutput.txt**, respectively, in **.../OpenSimLive/OpenSimLive-results/**.

### Running the tests

There are currently four tests, **test_EMG**, **test_Xsens_IK_speed**, **test_Xsens_IK_speed_multithread** and **test_IK_speed_multithread**. They are found in /Tests/ in the source directory and in a similar separate Tests folder in the build directory. Note that **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib must be copied to the folder where the test executables are to run IK speed tests.

**test_Xsens_IK_speed** measures how many inverse kinematics operations are performed over a time that the user inputs in seconds while using Xsens sensors. **test_Xsens_IK_speed_multithread** does the same with user-given amount of CPU threads.

**test_IK_speed_multithread** measures how many inverse kinematics operations are performed over a time that the user inputs in seconds. The user also inputs the number of threads to be used. This test supports Xsens or Delsys IMUs or simulated data.

All tests except **test_EMG** calculate point position tracking operations for mirror therapy if **station_parent_body** in **.../OpenSimLive/Config/MainConfiguration.xml** is not set to *none*.
When the tests finish, results are printed on the command prompt. They include how many IK (and possibly point tracking) operations were performed in total, how much time was elapsed and how many operations were performed on average per second.

**test_EMG** reads EMG data from Delsys sensors and outputs the throughput (EMG values read per second) when you finish the program.


## How it works

IMU-based inverse kinematics and mirror therapy applications rely on a number of classes. The most important ones and some of their member functions are highlighted below.

### IMUPlacerLive

This class calibrates an OpenSim model (XML file saved as .osim that contains joint angle definitions, constraints etc) by taking applying the orientations of IMUs at the calibration time point and placing IMUs on the bodies of the model. Config/IMUPlacerSetup.xml contains information about the model file that is calibrated, the file name that the calibrated model will be saved as, information about the initial heading of the "base" IMU and orientation transformations between IMU and OpenSim coordinates. **.../OpenSimLive/Config/SensorMappings.xml** describes which IMU corresponds to which body on the OpenSim model.

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

### DelsysDataReader

This class establishes the connection to Delsys Trigno Avanti inertial measurement units and acquires orientation and EMG data from them.

#### DelsysDataReader::updateQuaternionData()

This method creates a time series table of quaternions for a single frame. The quaternions represent orientations of Delsys Trigno Avanti sensors, supporting a maximum of 16 sensors.

Although the quaternion bytes for each sensor in the byte stream are in ascending order, the method may start reading this bytestream at any point, not only in the beginning. Therefore the first quaternion this method reads does not necessarily represent the orientation of the first sensor. Initially the method will assume this to be the case, and form a vector of initial detected sensor indices. These indices are compared to the sensor indices given in <active_sensors> in OpenSimLive/Config/DelsysMappings.xml and incremented until they match, and the number of increments is used as an "offset" to determine the actual sensor indices from the detected byte stream.

For example, if we use sensors 1, 2 and 3, but the method starts reading the byte stream from the third quaternion, the detected sensor indices before correction are 1, 15 and 16. After each incrementation step, we compare the results to the indices given in the config file. Therefore after the first incrementation we have 2, 16 and 1, which doesn't match the actual sensor indices even after sorting. After the second incrementation we have 3, 1 and 2 which after sorting is 1, 2 and 3. This matches the sensor indices in the config file, and we now know that the first detected quaternion actually belonged to the third sensor.

Now imagine that we use sensors 1, 2, 3, 4, 9, 10, 11 and 12. If the method starts reading the byte stream from the quaternion of sensor 9, it is detected as sensor 1 and without any offset incrementation we detect sensors 1, 2, 3, 4, 9, 10, 11 and 12, even though the actual indices of the sensors are 9, 10, 11, 12, 1, 2, 3 and 4. The method will return quaternions assigned to the wrong sensor indices. Therefore when selecting your sensors, you should avoid index sequences that contain this kind of symmetry.

### SimulatedDataReader

This is a very simple class that creates quaternions from a random distribution and lets the program use those quaternions as IMU orientations. It can be used in **OSL_common** if you do not wish to use actual IMUs.

### IMUHandler

This class can be used as a "generic data reader" class for IMUs. It is used in **test_common** to invoke methods from DelsysDataReader or XsensDataReader. Because of slight differences in the working principles of DelsysDataReader and XsensDataReader, the methods invoked by IMUHandler are not always exactly identical to the methods invoked by the corresponding data reader class.

### ThreadPoolContainer

This class controls the number of worker threads during multithreading and works as an interface to access the ThreadPool class. When constructed with an integer parameter N, ThreadPoolContainer creates a ThreadPool object with N worker threads. This class ensures that the user has control over how many worker threads run at a time.
This class is not necessary if multithreading is not used.

#### ThreadPoolContainer::offerFuture()

This method sends a function to the thread pool. If there are N worker threads running already, the function will wait in queue until the oldest of them has finished, then it will be given to the worker thread. If there are less than N worker threads running, the function is immediately given to a vacant worker thread. The elements of the thread pool are managed in a vector.

### Client

This class is based on [Keith Vertanen's Java / C++ socket class](https://www.keithv.com/software/socket/) and enables socket communication between Delsys Trigno Control Utility (TCU) and this program. It is used to send commands to TCU and receive orientation and EMG data from it. In effect, this class is used to read bytes sent by TCU.

### Server

Like Client, this class is also based on Keith Vertanen's work. It enables socket communication between this program and the Java client that is used to control the robot arm in mirror therapy.

### PythonPlotter

This class can be used to plot data, but it requires Python3 with matplotplib. The main program freezes when embedded Python commands are interpreted and the procedure is not thread-safe. Thus using this class during any data loops will slow them down.

## Descriptions of files

OpenSimLive contains a number of configuration and output files. Configuration files can all be found in **.../OpenSimLive/Config/** and output files can be found in **.../OpenSimLive/OpenSimLive-results/** with the exception of EMG time series from Delsys, which can be found in **.../OpenSimLive/Delsys-data/**. A brief description of each file follows.

### Files in Config folder

#### gait2392_full.osim and gait2392_shoulders.osim

These are the musculoskeletal models that we use as a base for calibration. Note that you can change this by modifying **model_file** IMUPlacerSetup.xml.

#### gait2392_full_calibrated.osim and similar _calibrated.osim files

This is the same model as gait2392_full.osim, but after calibration IMUs have been added to it with their initial orientations. Note that you can change the name of this file by modifying **output_model_file** IMUPlacerSetup.xml

#### MainConfiguration.xml

This XML file contains most user-defined settings that can be easily changed with a text editor without a need to rebuilt the project. The settings are as follows:
- **desired_update_rate**: preferred orientations-per-second throughput that Xsens IMUs should send to OpenSimLive. Note that Xsens has a defined set of supported rates so your desired input might be corrected to the closest predefined rate. Relevant only with Xsens IMUs.
- **mappings_file**: The XML file that contains the serial numbers / IDs or Xsens IMUs and the name of the body that the IMU is connected to in the model. Relevant only with Xsens IMUs.

**WIP!!!**


## Troubleshooting and FAQ

### General questions

#### What operating systems are supported by OpenSimLive?

Currently OpenSimLive has been tested on 64-bit Windows 7 and 64-bit Windows 10.

### Run-time issues

#### The program crashes right after printing "IMUPlacerLive run initiated"

Make sure that **IMUPlacerSetup.xml** in **.../OpenSimLive/Config/** has the correct file path for *model_file*. The file path should include the whole directory starting from the letter of your hard disk drive. Make sure that the same .osim file exists in **.../OpenSimLive/Config/**.

#### The program crashes right after printing "Loaded model ... from file ..."

Make sure that **IMUPlacerSetup.xml** in **.../OpenSimLive/Config/** has the correct value for **base_imu_label**. The value should equal one of the *name_in_model* values in **SensorMappings_full.xml** in **.../OpenSimLive/Config/**. Additionally make sure that the IMU in question is in use.

#### The program is stuck after printing "Waiting for ack..."

OpenSimLive is waiting on client program that is receiving data to acknowledge it received data. This is an issue with data not getting back to OpenSimLive, likely because the client has stopped working. The issue is therefore in the client program and not in OpenSimLive.

## Authors

Jere Lavikainen, jere.lavikainen (at) uef.fi

## License

## Acknowledgments

Thanks to GitHub user PurpleBooth for providing an excellent [readme template](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2).
Thanks to Keith Vertanen for his [Java and C++ socket class](https://www.keithv.com/software/socket/).
Thanks to Jacob Progsch and Václav Zeman for their [thread pool class](https://github.com/progschj/ThreadPool).
