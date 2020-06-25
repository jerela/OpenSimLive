cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(OpenSimLive VERSION 0.2)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find path to Vicon SDK
find_path(VICONSDK_PATH DataStreamClient.h PATHS C:/Program Files/Vicon/DataStream SDK/Win64/CPP)

# find Vicon SDK library
find_library(VICONSDK_LIB ViconDataStreamSDK_CPP.lib PATHS "${VICONSDK_PATH}")

# find path to OpenSim include and lib folders
find_path(OPENSIM_INCLUDE_PATH OpenSim.h PATHS "$ENV{OPENSIM_HOME}/sdk/include/OpenSim")
find_path(SIMBODY_INCLUDE_PATH Simbody.h PATHS "$ENV{OPENSIM_HOME}/sdk/Simbody/include")
find_path(OPENSIM_LIB_PATH osimCommon.lib PATHS "$ENV{OPENSIM_HOME}/sdk/lib")
find_path(SIMBODY_LIB_PATH SimTKcommon.lib PATHS "$ENV{OPENSIM_HOME}/sdk/Simbody/lib")

# set variables for OpenSim and Simbody library name lists
set(OPENSIM_LIBRARY_LIST
    osimCommon osimSimulation osimAnalyses osimActuators osimTools)
set(SIMBODY_LIBRARY_LIST SimTKcommon SimTKmath SimTKsimbody
        OpenSim_SimTKcommon OpenSim_SimTKmath OpenSim_SimTKsimbody)

set(OPENSIM_LIBRARY)

# iterate through all elements in the OpenSim library list
foreach(LIB_NAME IN LISTS OPENSIM_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
        PATHS "${OPENSIM_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND OPENSIM_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()

set(SIMBODY_LIBRARY ${OPENSIM_LIBRARY})

# iterate through all elements in the Simbody library list
foreach(LIB_NAME IN LISTS SIMBODY_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
		PATHS "${OPENSIM_LIB_PATH}" "${SIMBODY_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND SIMBODY_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()

configure_file(OpenSimLiveConfig.h.in OpenSimLiveConfig.h)

set(OPENSIM_INCLUDE_PATH_TWO "$ENV{OPENSIM_HOME}/sdk/include")

# add and link the executables, libraries and directories for ViconMarkerReader
if(${VICONSDK_LIB})
	add_executable(ViconMarkerReader ViconMarkerReader.cxx)
	# link Vicon SDK library
	target_link_libraries(ViconMarkerReader PRIVATE "${VICONSDK_LIB}")
	# link OpenSim and Simbody libraries
	target_link_libraries(ViconMarkerReader PRIVATE "${OPENSIM_LIBRARY}")
	target_link_libraries(ViconMarkerReader PRIVATE "${SIMBODY_LIBRARY}")
	target_include_directories(ViconMarkerReader PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${VICONSDK_PATH}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   )
endif()
add_executable(XsensReader XsensReader.cxx)










# set variables for MT library name list
set(MT_LIBRARY_LIST
    xsensdeviceapi64 xstypes64)

set(MT_LIBRARY)

# find path to MT SDK libraries
find_path(MT_LIB_PATH xsensdeviceapi64.lib PATHS "C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/lib")

# iterate through all elements in the MT library list
foreach(LIB_NAME IN LISTS MT_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
        PATHS "${MT_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND MT_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()

# find path to MT SDK
find_path(MTSDK_PATH xsensdeviceapi.h PATHS "C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/include")

# link MT SDK libraries
target_link_libraries(XsensReader PRIVATE "${MT_LIBRARY}")
target_link_libraries(XsensReader PRIVATE "${OPENSIM_LIBRARY}")
target_link_libraries(XsensReader PRIVATE "${SIMBODY_LIBRARY}")



target_include_directories(XsensReader PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   "${MTSDK_PATH}"
						   "${MTSDK_XSENS_PATH}"
						   )

