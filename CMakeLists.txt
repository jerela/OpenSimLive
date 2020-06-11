cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(OpenSimLive VERSION 0.1)

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

# find OpenSim and Simbody libraries
#find_library(OPENSIM_LIB osimCommon.lib PATHS "${OPENSIM_LIB_PATH}")
#find_library(OPENSIM_LIB osimCommon.lib PATHS "${OPENSIM_LIB_PATH}")

configure_file(OpenSimLiveConfig.h.in OpenSimLiveConfig.h)

# add the executable
add_executable(OpenSimLive OpenSimLive.cxx)

# link Vicon SDK library
target_link_libraries(OpenSimLive PRIVATE "${VICONSDK_LIB}")

# link OpenSim and Simbody libraries
target_link_libraries(OpenSimLive PRIVATE "${OPENSIM_LIBRARY}")
target_link_libraries(OpenSimLive PRIVATE "${SIMBODY_LIBRARY}")

target_include_directories(OpenSimLive PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${VICONSDK_PATH}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${SIMBODY_INCLUDE_PATH}"
						   )

