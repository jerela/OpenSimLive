# OpenSimLive
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
- Select x64 as the generator when prompted.
- Select **Configure**. CMake variables and their values should now be displayed. If any of them are not found, you can manually type in folders. An example file path of each is as follows:
   - INCLUDE_CLASSES_PATH:      C:/Users/YourUserHere/Documents/OpenSimLive/Classes
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

Instructions on how to run the automated tests for this project.

## Authors

Jere Lavikainen, jere.lavikainen (at) uef.fi

## License

## Acknowledgments

Thanks to GitHub user PurpleBooth for providing an excellent [readme template](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2).
Thanks to Keith Vertanen for his [Java and C++ socket class](https://www.keithv.com/software/socket/).
Thanks to Jacob Progsch and Václav Zeman for their [thread pool class](https://github.com/progschj/ThreadPool).
