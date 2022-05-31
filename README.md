# OpenSimLive

- [Getting started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installing](#installing)
  * [Running the program](#running-the-program)
- [How it works](#how-it-works)
- [Descriptions of files](#descriptions-of-files)
- [Troubleshooting and FAQ](#troubleshooting-and-faq)
  * [General questions](#general-questions)
  * [Run-time issues](#run-time-issues)
- [Authors](#authors)
- [License and copyright](#license-and-copyright)
- [Acknowledgements](#acknowledgements)
- [Similar projects](#similar-projects)
<!-- toc -->


OpenSimLive is a C++ package that streams orientation data from inertial measurement units and calculates inverse kinematics based on that data. It relies on OpenSim for biomechanical analyses and related tools. The current version uses [OpenSim 4.1 API](https://simtk.org/api_docs/opensim/api_docs/index.html). Two types of IMUs are currently supported: Xsens MTw Awinda and Delsys Trigno Avanti. Xsens IMUs use XDA 4.6 and Delsys IMUs use Delsys Trigno Control Utility.

Some of OpenSimLive's features can be tested without actual IMUs by using simulated IMU data that that OpenSimLive generates as random unit quaternions.

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

OpenSim 4.1 is required for core functionality. XDA 4.6 is required to use Xsens IMUs. Delsys Trigno Control Utility is required to communicate with Delsys SDK and read data from Delsys IMUs. CMake and Visual Studio are used to configure, generate and build the project.

You can get XDA 4.6 from [the Xsens website](https://www.xsens.com/software-downloads) by downloading MT Software Suite under MTw Awinda. Make sure not to download MT Software Suite under MTi Products if you use MTw Awinda IMUs.

~~Python 3.7+ is optional and enables using the PythonPlotter class, which can be used to plot OpenSimLive data.~~ Python-based plotting is not supported in the latest version.

### Installing

Step by step instructions on how to install this project.

#### Windows (64-bit)

1. Download and unzip the package to a directory on your hard drive.
2. Open CMake and select the directory from the previous step as the source code directory.
- Put YourFilePath/YourSourceCodeFolder-build or whatever else you want as the build folder and allow CMake to create a new folder when prompted.
- Select **x64** as the generator when prompted.
- Select **Configure**. CMake variables and their values should now be displayed. If any of them are not found, you can manually type in folders. An example file path of each is as follows:
   - CMAKE_CONFIGURATION_TYPES  Debug;Release;MinSizeRel;RelWithDebInfo
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
3. Open Visual Studio. Open the solution you just generated in the build directory. Make sure that the solution configuration in the top bar is set to RelWithDebInfo or Release instead of Debug, as Debug builds will not work without including the external .pdb debug information databases. Build **ALL_BUILD**. Visual Studio should now create the required executable(s) in a subdirectory in the build directory. It will probably be **.../BuildFolderName/MirrorTherapy/RelWithDebInfo/** for mirror therapy applications and **.../BuildFolderName/Tests/RelWithDebInfo/** for tests.
4. Copy **xsensdeviceapi64.dll** and **xstypes64.dll** from .../Xsens/MT Software Suite 4.6/MT SDK/x64/lib to the directories where the executables are or add their locations to the *PATH* environmental variable. **This is required to run Xsens-related scripts, but is otherwise optional.**
5. Make sure **...\OpenSim 4.1\bin** is present in your *PATH* environmental variable. Otherwise you will receive an error message when trying to run any of the executables.
6. Go to **.../OpenSimLive/Config** and make sure the .xml files have the right values for your directory paths.
- You need to download an .osim model file to use with the program. You can find several models here: https://simtk-confluence.stanford.edu/display/OpenSim/Musculoskeletal+Models
- For testing, the gait2392 model and the Hamner full body model are recommended for lower-body and full-body kinematics, respectively.
7. Installation complete. You are ready to run OpenSimLive.

### Running the program

The main program (**OSL_core**) and most other programs and tests are controlled by keyboard input. Calibrating a model will open a visualization in another window, so make sure you select the command console window as the active window to ensure keyboard input is successfully read.

If you have set IMU manufacturer as "xsens" in MainConfiguration.xml, then when the program starts, it will look for active Xsens IMUs. Make sure your IMUs are on and not on standby mode (move them until a red light starts blinking). The program will list all IMUs it can connect to in a numbered order and finding an IMU may take a few seconds. When the program lists all the IMUs you want to use, press Y on your keyboard to continue.

If you have set IMU manufacturer as "delsys" in MainConfiguration.xml, then the program will start reading data stream from Delsys Trigno Control Utility.

Finally, you can run the program without any actual IMUs by setting manufactured as "simulated", in which case the program will create random quaternions for IMU orientations. The visualization of the skeletal model in this case will not look sensible, but will instead change to random and often unnatural orientations with each new frame.

The program should print input instructions in the command window. Pressing C will calibrate the model.

After the model is calibrated, you can enable continuous inverse kinematics with N and disable them with M. The visualization window will show the solved joint angles on the model.

When you are finished, pressing X will quit the program. At this point the program will save IK results to **IK-live.mot** and read quaternion data to **QuaternionTimeSeriesTableXsens.txt**, **QuaternionTimeSeriesTableDelsys.txt** or **QuaternionTimeSeriesTableSimulated.txt** in **.../OpenSimLive/OpenSimLive-results/**.

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

#### IMUInverseKinematicsToolLive::updateOrderedInverseKinematics()

This method is used to calculate inverse kinematics repeatedly. It can be run concurrently by different threads. If PointTracker is enabled, PointTracker calculations are performed at the end of this method.
This method is private and should be invoked from other classes through IMUInverseKinematicsToolLive::updateOrdered().

### PointTracker

This class is for mirror therapy applications. It takes the position and parent body of a point in the model and a reference body as input. The position of the point is then expressed in the reference body's coordinate system and reflected with respect to the z axis of that coordinate system. The orientation of the point's parent body is similarly mirrored. The class is independent in the sense that it doesn't inherit other classes and isn't based on any existing classes, although its methods make extensive use of the OpenSim 4.1 API.

#### PointTracker::runTracker()

This method is the "main" method of this class and calls other methods to perform individual parts of the calculations. These calculations sometimes require writable references to OpenSim::Model and SimTK::State, which is why the method has to be invoked in IMUInverseKinematisToolLive::updateOrderedInverseKinematics().

#### PointTracker::addStationToBody()

This method is used by IMUPlacerLive during calibration to modify the XML file containing the OpenSim model. It adds XML elements describing the point of interest in mirror therapy to the OpenSim model.

### XsensDataReader

This class establishes the connection to Xsens MTw Awinda inertial measurement units, acquires orientation data from them and closes the connection.

#### XsensDataReader::GetQuaternionData()

TO BE UPDATED

### DelsysDataReader

This class establishes the connection to Delsys Trigno Avanti inertial measurement units and acquires orientation and EMG data from them.

#### DelsysDataReader::updateQuaternionData()

This method creates a time series table of quaternions for a single frame. The quaternions represent orientations of Delsys Trigno Avanti sensors, supporting a maximum of 16 sensors.

Although the quaternion bytes for each sensor in the byte stream are in ascending order, the method may start reading this bytestream at any point, not only in the beginning. Therefore the first quaternion this method reads does not necessarily represent the orientation of the first sensor. Initially the method will assume this to be the case, and form a vector of initial detected sensor indices. These indices are compared to the sensor indices given in <active_sensors> in OpenSimLive/Config/DelsysMappings.xml and incremented until they match, and the number of increments is used as an "offset" to determine the actual sensor indices from the detected byte stream.

For example, if we use sensors 1, 2 and 3, but the method starts reading the byte stream from the third quaternion, the detected sensor indices before correction are 1, 15 and 16. After each incrementation step, we compare the results to the indices given in the config file. Therefore after the first incrementation we have 2, 16 and 1, which doesn't match the actual sensor indices even after sorting. After the second incrementation we have 3, 1 and 2 which after sorting is 1, 2 and 3. This matches the sensor indices in the config file, and we now know that the first detected quaternion actually belonged to the third sensor.

Now imagine that we use sensors 1, 2, 3, 4, 9, 10, 11 and 12. If the method starts reading the byte stream from the quaternion of sensor 9, it is detected as sensor 1 and without any offset incrementation we detect sensors 1, 2, 3, 4, 9, 10, 11 and 12, even though the actual indices of the sensors are 9, 10, 11, 12, 1, 2, 3 and 4. The method will return quaternions assigned to the wrong sensor indices. Therefore when selecting your sensors, you should avoid index sequences that contain this kind of symmetry.

### SimulatedDataReader

This is a very simple class that creates quaternions from a random distribution and lets the program use those quaternions as IMU orientations. It can be used in **OSL_core** if you do not wish to use actual IMUs.

### IMUHandler

This class can be used as a "generic data reader" class for IMUs. It is used to invoke methods from DelsysDataReader or XsensDataReader. Because of slight differences in the working principles of DelsysDataReader and XsensDataReader, the methods invoked by IMUHandler are not always exactly identical to the methods invoked by the corresponding data reader class.

### ThreadPoolContainer

This class controls the number of worker threads during multithreading and works as an interface to access the ThreadPool class. When constructed with an integer parameter N, ThreadPoolContainer creates a ThreadPool object with N worker threads. This class ensures that the user has control over how many worker threads run at a time.

#### ThreadPoolContainer::offerFuture()

This method sends a function to the thread pool. If there are N worker threads running already, the function will wait in queue until the oldest of them has finished, then it will be given to the worker thread. If there are less than N worker threads running, the function is immediately given to a vacant worker thread. The elements of the thread pool are managed in a vector.

### Client

This class is based on [Keith Vertanen's Java / C++ socket class](https://www.keithv.com/software/socket/) and enables socket communication between Delsys Trigno Control Utility (TCU) and this program. It is used to send commands to TCU and receive orientation and EMG data from it. In effect, this class is used to read bytes sent by TCU.

### Server

Like Client, this class is also based on Keith Vertanen's work. It enables socket communication between this program and the Java client that is used to control the robot arm during mirror therapy.

### PythonPlotter

This class can be used to plot data, but it requires Python3 with matplotplib. The main program freezes when embedded Python commands are interpreted and the procedure is not thread-safe. Thus using this class during any data loops will slow them down. CURRENTLY NOT IN USE, TO BE REPLACED/UPDATED

## Descriptions of files

OpenSimLive contains a number of configuration and output files. All configuration files can be found in **.../OpenSimLive/Config/** and output files can be found in **.../OpenSimLive/OpenSimLive-results/**. A brief description of each file follows.

### Files in Config folder

#### gait2392_full.osim and gait2392_shoulders.osim

These are the musculoskeletal models that we use as a base for calibration. Note that you can change this by modifying **model_file** IMUPlacerSetup.xml.

#### gait2392_full_calibrated.osim and similar _calibrated.osim files

This is the same model as gait2392_full.osim, but after calibration IMUs have been added to it with their initial orientations. Note that you can change the name of this file by modifying **output_model_file** IMUPlacerSetup.xml

#### MainConfiguration.xml

This XML file contains most user-defined settings that can be easily changed with a text editor without a need to rebuilt the project. The settings are as follows:
- **desired_update_rate**: preferred orientations-per-second throughput that Xsens IMUs should send to OpenSimLive. Note that Xsens has a defined set of supported rates so your desired input might be corrected to the closest predefined rate. Relevant only with Xsens IMUs.
- **mappings_file**: The XML file that contains the serial numbers / IDs or Xsens IMUs and the name of the body that the IMU is connected to in the model. Relevant only with Xsens IMUs.
- **imu_placer_setup_file**: The XML file that contains information for model calibration. It is IMUPlacerSetup.xml by default and will not require changes unless you wish to have several setup files and switch between them this way.
- **save_ik_results**: A simple true/false boolean to toggle if time series of solved joint angles and their errors should be saved to **.../OpenSimLive/OpenSimLive-results/** as IK_live.mot and IK-live_orientationErrors.sto, respectively.
- **continuous_mode_ms_delay**: When reading data continuously from IMUs, this sets the minimum delay (in milliseconds) between consecutive time points for IK. If visualizing IK and this is too low, visualizer will "clog up", which shows as low FPS and a delay between the actual movement and the model's movement. Aim for a value that allows the FPS you want, but isn't much lower.
- **print_roll_pitch_yaw**: When this is true, Xsens IMUs will print their roll, pitch and yaw angles to console. TO BE UPDATED
- **reset_clock_on_continuous_mode**: When this is true, the clock for IK time series will be restarted from zero whenever you re-enter continuous mode. When this is false, the clock will start running when you calibrate the model and then keep running no matter how many pauses from continuous mode you take. TO BE UPDATED
- **station_parent_body**: The body on the OpenSim model that we want to mirror, such as a rehabilitation patient's healthy hand whose movement we want to mirror. Relevant only if you use PointTracker for mirror therapy applications. Set to "none" if you don't need it.
- **station_location**: Location of the station to be mirrored in the coordinate system of station_parent_body. Given in metres. Relevant only if you use mirror therapy applications, ignored otherwise.
- **station_reference_body**: The body on the OpenSim model with respect to which we mirror station_parent_body. The mirroring is done with respect to the XY plane of station_reference_body. Relevant only if you use mirror therapy applications, ignored otherwise.
- **point_tracker_output_format**: "euler" or "quaternion" depending on which format you want to use while sending the mirrored rotations to the Java client. Relevant only if you use mirror therapy applications, ignored otherwise.
- **euler_convention**: Body or space-fixed coordinate axes and the order of rotations around the axes while converting rotations to Euler angles. Separate with a hyphen, e.g. "space-zyx". Relevant only if you use mirror therapy applications and **point_tracker_output_format** is "euler", ignored otherwise.
- **transform_rotations_to_kuka**: If true, the mirrored rotations are transformed from OpenSim coordinate system (Y up) to KUKA coordinate system (Z up) and then rotated 180 degrees around the vertical axis (because the rehabilitation robot is facing the patient). Relevant only if you use mirror therapy applications, ignored otherwise.
- **socket_port**: Port for socket communication between the server (OpenSimLive) and the client (such as a Java program that controls a robotic rehabilitation arm). Relevant only if you use mirror therapy applications, ignored otherwise.
- **threads**: The maximum allowed number of concurrent IK threads to use for multithreading.
- **max_buffer_size**: Maximum number of points that can be saved into the buffers that are shared between the producer thread and the consumer thread. It is recommended that this is at least the number of concurrent IK threads.
- **enable_EMG_plotting**: If set to true, real-time EMG data will be plotted as its read at a very low FPS. This will prevent the program to read EMG data at frequencies necessary for EMG analysis. Relevant only if you use EMG reading applications. TO BE UPDATED
- **IMU_manufacturer**: Either "simulated", "delsys" or "xsens". See [Running the program](#running-the-program) for more information.
- **simulated_bodies**: List of bodies on the OpenSim model, with suffix "\_imu". If we are simulating IMU data, random orientations will be generated for these bodies. Irrelevant otherwise.
- **save_quaternions_to_file**: Boolean that toggles if quaternion data is saved to file after the program finishes.
- **enable_imu_feedback**: If true, information about IMU drift and RPY angles is printed to the console during real-time IK.
- **tracked_coordinates**: List of OpenSim model coordinates (joint angles) that you want to track with a slider in the visualization window. Does not affect the calculations, the effect is only visual.
- **print_tracked_coordinates**: If true, the angles of tracked coordinates are printed to console whenever an IK operation finishes.
- **subject_height**: Height of the human subject in cm. Used in scaling the model after IMU calibration.
- **model_height**: Height of the generic musculoskeletal model in cm. Used in scaling the model after IMU calibration. 

### DelsysMappings.xml

This XML file contains settings specific to Delsys IMUs. The settings are as follows:
- **number_of_active_sensors**: The number of Delsys sensors that are online.
- **active_sensors_**: The indices of the slots on Trigno Control Utility for sensors that are online. The number of these indices must equal number_of_active_sensors.
- **sensor_x_label**: x is a number from 1 to 16, denoting the index of an active sensor. The value of these should be the name of a body on the OpenSim model with suffix "_imu". This links a sensor and its orientation output to a body on the OpenSim model. For example, if active_sensors is "1 2" then "sensor_1_label" could be "pelvis_imu", indicating that orientation from the sensor on TCU slot 1 is taken as the orientation of the IMU on pelvis.

### SensorMappings.xml

This XML file contains settings specific to Xsens IMUs. The settings are as follows:
- **trial_prefix**: Not used in OpenSimLive and can be safely ignored. The reason this exists is because OpenSense uses an identical format of XML files to calculate inverse kinematics from prerecorded trials.
- **name**: The serial code / ID on the physical IMU box.
- **name_in_model**: The body on the OpenSim model, with suffix "_imu", that this IMU is connected to.

## Troubleshooting and FAQ

### General questions

#### What operating systems are supported by OpenSimLive?

Currently OpenSimLive has been tested on 64-bit Windows 7 and 64-bit Windows 10.

#### Which project am I supposed to build and run?

**OSL_core** is the main project of the solution and probably the one you should be building and using. It supports real-time inverse kinematics with Delsys and Xsens IMUs and simulated IMU data (without real IMUs).
There is also **OSL_common**, which is used in [our research lab](https://sites.uef.fi/humea/) to control a KUKA iiwa robot arm for mirror therapy. It is practically **OSL_core** with mirror therapy features on top.
Then there are a bunch of projects starting with **test_**. They were created for performance testing for a publication about this repository, and while they might still work, they are meant to be used with the **study-measurements** branch.
Similarly, **OSL_Xsens** and **OSL_Delsys** might still work, but they are have been replaced by **OSL_core**, which can handle both IMU types.

### Run-time issues

#### The program crashes right after printing "IMUPlacerLive run initiated"

Make sure that **IMUPlacerSetup.xml** in **.../OpenSimLive/Config/** has the correct file path for *model_file*. The file path should include the whole directory starting from the letter of your hard disk drive. Make sure that the same .osim file exists in **.../OpenSimLive/Config/**.

#### The program crashes right after printing "Loaded model ... from file ..."

Make sure that **IMUPlacerSetup.xml** in **.../OpenSimLive/Config/** has the correct value for **base_imu_label**. The value should equal one of the *name_in_model* values in **SensorMappings_full.xml** in **.../OpenSimLive/Config/**. Additionally make sure that the IMU in question is in use.

#### The program is stuck after printing "Waiting for ack..."

OpenSimLive is waiting on client program that is receiving data to acknowledge it received data. This is an issue with data not getting back to OpenSimLive, likely because the client program has stopped working.

## Authors

Jere Lavikainen, jere.lavikainen (at) uef.fi

## License and copyright

The following copyright disclaimer applies to all files in this repository with the exception of **files under OpenSimLive/Config**, **ThreadPool.h**, **XsensDataReader.h**, **XsensDataReader.cpp**, **Client.h**, **Client.cpp**, **Server.h**, **Server.cpp**, **IMUInverseKinematicsToolLive.h**, **IMUInverseKinematicsToolLive.cpp**, **IMUPlacerLive.h** and **IMUPlacerLive.cpp**.

### Copyright disclaimer / EULA

Copyright 2021 University of Eastern Finland

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
```
http://www.apache.org/licenses/LICENSE-2.0
```

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

### Copyright for Client.h, Client.cpp, Server.h and Server.h

The author of OpenSimLive would like to acknowledge that Client.h, Client.cpp, Server.h and Server.cpp are originally the work of Keith Vertanen and may have been modified by the author of OpenSimLive in this distribution. Keith Vertanen has stated regarding his work: "You may use this code for whatever you like."

### Copyright for XsensDataReader.h and XsensDataReader.cpp

The author of OpenSimLive would like to state that XsensDataReader.h and XsensDataReader.cpp are based on example code provided by Xsens. The example code has been modified by the author of OpenSimLive in this distribution. The following disclaimer applies for modified versions of the example code:

```
Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.	Redistributions of source code must retain the above copyright notice,
this list of conditions, and the following disclaimer.

2.	Redistributions in binary form must reproduce the above copyright notice,
this list of conditions, and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3.	Neither the names of the copyright holders nor the names of their contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
```

### Copyright for ThreadPool.h

The author of OpenSimLive would like to state that ThreadPool.h is the work of Jakob Progsch and Václav Zeman and may have been modified by the author of OpenSimLive in this distribution. The following disclaimer applies for ThreadPool.h:

```
Copyright (c) 2012 Jakob Progsch, Václav Zeman

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.
```

### Copyright for IMUInversekinematicsToolLive.h, IMUInverseKinematicsToolLive.cpp, IMUPlacerLive.h and IMUPlacerLive.cpp

These four files are originally the work of the developers and contributors of [OpenSim](https://github.com/opensim-org/opensim-core), subject to [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0). The author of OpenSimLive has modified and renamed these files. The original file names are similar, but do not contain "Live" in the end before the file format suffix.

## Acknowledgements

Huge thanks to the people behind the [OpenSim](https://simtk.org/projects/opensim) project.
Thanks to GitHub user PurpleBooth for providing an excellent [readme template](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2).
Thanks to Keith Vertanen for his [Java and C++ socket class](https://www.keithv.com/software/socket/).
Thanks to Jacob Progsch and Václav Zeman for their [thread pool class](https://github.com/progschj/ThreadPool).

This software library was developed while working in an European Regional Development Fund project: Digital Technology RDI Environment (Digi Center) (Project ID: A74338, www.digicenterns.fi).

## Similar projects

For marker-based real-time inverse kinematics using OpenSim 3.3 API, see C. Pizzolato's [RTOSIM](https://github.com/RealTimeBiomechanics/rtosim/).
For marker and IMU-based real time kinematical and dynamical analysis using OpenSim 4.1 API, see D. Stanev's [OpenSimRT](https://github.com/mitkof6/OpenSimRT).
