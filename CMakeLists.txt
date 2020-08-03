cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(OpenSimLive VERSION 0.3)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# YOU MAY HAVE TO SET THIS MANUALLY IF YOU HAVE VICON SDK IN A DIFFERENT DIRECTORY
# find path to CPP folder of the Vicon SDK installation directory
find_path(VICONSDK_PATH DataStreamClient.h PATHS C:/Program Files/Vicon/DataStream SDK/Win64/CPP)

# find Vicon SDK library
find_library(VICONSDK_LIB ViconDataStreamSDK_CPP.lib PATHS "${VICONSDK_PATH}")

# find path to OpenSim include and lib folders (according to OpenSim 4.1 directory hierarchy)
find_path(OPENSIM_INCLUDE_PATH OpenSim.h PATHS "$ENV{OPENSIM_HOME}/sdk/include/OpenSim")
find_path(SIMBODY_INCLUDE_PATH Simbody.h PATHS "$ENV{OPENSIM_HOME}/sdk/Simbody/include")
find_path(OPENSIM_LIB_PATH osimCommon.lib PATHS "$ENV{OPENSIM_HOME}/sdk/lib")
find_path(SIMBODY_LIB_PATH SimTKcommon.lib PATHS "$ENV{OPENSIM_HOME}/sdk/Simbody/lib")

# set variables for OpenSim and Simbody library name lists (according to OpenSim 4.1)
set(OPENSIM_LIBRARY_LIST
    osimCommon osimSimulation osimAnalyses osimActuators osimTools)
set(SIMBODY_LIBRARY_LIST SimTKcommon SimTKmath SimTKsimbody
        OpenSim_SimTKcommon OpenSim_SimTKmath OpenSim_SimTKsimbody)

# declare variable
set(OPENSIM_LIBRARY)

# iterate through all elements in the OpenSim library list and append file to OPENSIM_LIBRARY if found
foreach(LIB_NAME IN LISTS OPENSIM_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
        PATHS "${OPENSIM_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND OPENSIM_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()

# set the found OpenSim libraries into SIMBODY_LIBRARY
set(SIMBODY_LIBRARY ${OPENSIM_LIBRARY})

# iterate through all elements in the Simbody library list and append to SIMBODY_LIBRARY if found
foreach(LIB_NAME IN LISTS SIMBODY_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
		PATHS "${OPENSIM_LIB_PATH}" "${SIMBODY_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND SIMBODY_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()


# add a definition that can be called as a variable in C++
add_definitions(-DOPENSIMLIVE_ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

configure_file(OpenSimLiveConfig.h.in OpenSimLiveConfig.h)

# another path variable for OpenSim includes according to OpenSim 4.1 hierarchy
#set(OPENSIM_INCLUDE_PATH_TWO "$ENV{OPENSIM_HOME}/sdk/include")
find_path(OPENSIM_INCLUDE_PATH_TWO OpenSim.h PATHS "$ENV{OPENSIM_HOME}/sdk/include/OpenSim")

# add and link the executables, libraries and directories for ViconMarkerReader only if VICONSDK_LIB was found
if(VICONSDK_LIB)
	message([STATUS] "Vicon SDK found.")
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
else()
	# if Vicon SDK is not found, inform the user but continue CMake
	message([STATUS] "Cannot find Vicon SDK.")	
endif()



# set variables for MT library name list
set(MT_LIBRARY_LIST
    xsensdeviceapi64 xstypes64)

set(MT_LIBRARY)

# YOU MAY HAVE TO SET THIS MANUALLY IF YOU HAVE MT SDK IN A DIFFERENT DIRECTORY
# find path to MT SDK libraries
find_path(MT_LIB_PATH xsensdeviceapi64.lib PATHS "C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/lib")

# iterate through all elements in the MT library list, append to MT_LIBRARY if found
foreach(LIB_NAME IN LISTS MT_LIBRARY_LIST)
    find_library(FOUND_LIB NAMES ${LIB_NAME}
        PATHS "${MT_LIB_PATH}"
        NO_DEFAULT_PATH)
    if(FOUND_LIB)
        list(APPEND MT_LIBRARY ${FOUND_LIB})
    endif()
    unset(FOUND_LIB CACHE)
endforeach()

# YOU MAY HAVE TO SET THIS MANUALLY IF YOU HAVE MT SDK IN A DIFFERENT DIRECTORY
# find path to MT SDK
find_path(MTSDK_PATH xsensdeviceapi.h PATHS "C:/Program Files/Xsens/MT Software Suite 4.6/MT SDK/x64/include")


# find path to classes folder
find_path(INCLUDE_CLASSES_PATH IMUPlacerLive.h PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Classes")
# same for functions folder
find_path(INCLUDE_FUNCTIONS_PATH XMLFunctions.h PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Functions")
# set all C++ files that XsensReader uses into XSENS_SOURCES
set(XSENS_SOURCES XsensReader.cxx "${INCLUDE_CLASSES_PATH}/IMUPlacerLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.cxx"
	"${INCLUDE_CLASSES_PATH}/PointTracker.h"
	"${INCLUDE_CLASSES_PATH}/PointTracker.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.cxx")



# add executable for C++ files in XSENS_SOURCES
add_executable(XsensReader "${XSENS_SOURCES}")

# link MT SDK libraries
target_link_libraries(XsensReader PRIVATE "${MT_LIBRARY}"
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}")

target_include_directories(XsensReader PRIVATE
						"${INCLUDE_CLASSES_PATH}"
						"${INCLUDE_FUNCTIONS_PATH}")
target_include_directories(XsensReader PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   "${MTSDK_PATH}"
						   "${MT_LIB_PATH}"
						   )
