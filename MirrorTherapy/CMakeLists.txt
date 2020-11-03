
# set all C++ files that OSL_Xsens uses into OSL_XSENS_SOURCES
set(OSL_XSENS_SOURCES OSL_Xsens.cxx "${INCLUDE_CLASSES_PATH}/IMUPlacerLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.cxx"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.h"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.cxx"
	"${INCLUDE_CLASSES_PATH}/PointTracker.h"
	"${INCLUDE_CLASSES_PATH}/PointTracker.cxx"
	"${INCLUDE_CLASSES_PATH}/Server.h"
	"${INCLUDE_CLASSES_PATH}/Server.cpp"
	"${INCLUDE_CLASSES_PATH}/XsensDataReader.h"
	"${INCLUDE_CLASSES_PATH}/XsensDataReader.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPool.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.tpp"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctionsXsens.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctionsXsens.cxx")



# add executable for C++ files in XSENS_SOURCES
add_executable(OSL_Xsens "${OSL_XSENS_SOURCES}")

# link MT SDK libraries
target_link_libraries(OSL_Xsens PRIVATE "${MT_LIBRARY}" # Xsens MT libraries for IMU data retrieval
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}"
						PRIVATE "ws2_32.lib") # winsock2 required for socket communication

target_include_directories(OSL_Xsens PRIVATE
						"${INCLUDE_CLASSES_PATH}"
						"${INCLUDE_FUNCTIONS_PATH}")
target_include_directories(OSL_Xsens PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   "${MTSDK_PATH}"
						   "${MT_LIB_PATH}"
						   )









						   
						   
						 
						 
set(OSL_DELSYS_SOURCES
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.cxx"
	"${INCLUDE_CLASSES_PATH}/PointTracker.h"
	"${INCLUDE_CLASSES_PATH}/PointTracker.cxx"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.h"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.cxx"
	"${INCLUDE_CLASSES_PATH}/DelsysDataReader.h"
	"${INCLUDE_CLASSES_PATH}/DelsysDataReader.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPool.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.tpp"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.cxx"
	"${INCLUDE_CLASSES_PATH}/Server.h"						   
	"${INCLUDE_CLASSES_PATH}/Server.cpp"						   
	"${INCLUDE_CLASSES_PATH}/Client.h"						   
	"${INCLUDE_CLASSES_PATH}/Client.cpp"
	"${INCLUDE_CLASSES_PATH}/PythonPlotter.h"						   
	"${INCLUDE_CLASSES_PATH}/PythonPlotter.cxx")	 



# add executable based on the source files determined in TEST_DELSYS_SOURCES
add_executable(OSL_Delsys
			   OSL_Delsys.cxx
			   "${OSL_DELSYS_SOURCES}")


# attempt to find Python path and libs
find_path(PYTHON_PATH python38.dll PATHS "C:/Users/wksadmin/AppData/Local/Programs/Python/Python38")
find_path(PYTHON_LIB python38.lib PATHS "${PYTHON_PATH}/libs")
# add and link the executables, libraries and directories for Python only if PYTHON_LIB was found
if(PYTHON_LIB)
	message([STATUS] "Python found.")
	target_link_libraries(OSL_Delsys PRIVATE "${PYTHON_LIB}/python38.lib"
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}"
						PRIVATE "ws2_32.lib")
	target_include_directories(OSL_Delsys PRIVATE "${PYTHON_PATH}/include")
	# add a preprocessor definition
	add_definitions(-DPYTHON_ENABLED)
else()
	# if Python is not found, inform the user but continue CMake
	message([STATUS] "Cannot find Python. Removing PYTHON_LIB and PYTHON_PATH from list of CMake variables.")
	unset(PYTHON_LIB)
	unset(PYTHON_PATH)
	target_link_libraries(OSL_Delsys PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}"
						PRIVATE "ws2_32.lib")
	# add a preprocessor definition
	add_definitions(-DPYTHON_DISABLED)
endif()



						
target_include_directories(OSL_Delsys PRIVATE
						"${INCLUDE_CLASSES_PATH}"
						"${INCLUDE_FUNCTIONS_PATH}")
						
target_include_directories(OSL_Delsys PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   )								   






















set(OSL_COMMON_SOURCES
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.cxx"
	"${INCLUDE_CLASSES_PATH}/PointTracker.h"
	"${INCLUDE_CLASSES_PATH}/PointTracker.cxx"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.h"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.cxx"
	"${INCLUDE_CLASSES_PATH}/DelsysDataReader.h"
	"${INCLUDE_CLASSES_PATH}/DelsysDataReader.cxx"
	"${INCLUDE_CLASSES_PATH}/XsensDataReader.h"
	"${INCLUDE_CLASSES_PATH}/XsensDataReader.cxx"
	"${INCLUDE_CLASSES_PATH}/SimulatedDataReader.h"
	"${INCLUDE_CLASSES_PATH}/SimulatedDataReader.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctionsXsens.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctionsXsens.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPool.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.tpp"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.cxx"
	"${INCLUDE_CLASSES_PATH}/Server.h"						   
	"${INCLUDE_CLASSES_PATH}/Server.cpp"						   
	"${INCLUDE_CLASSES_PATH}/Client.h"						   
	"${INCLUDE_CLASSES_PATH}/Client.cpp"
	"${INCLUDE_CLASSES_PATH}/PythonPlotter.h"						   
	"${INCLUDE_CLASSES_PATH}/PythonPlotter.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUHandler.h"						   
	"${INCLUDE_CLASSES_PATH}/IMUHandler.cxx")	 


# add a definition that can be called as a variable in C++
#add_definitions(-DOPENSIMLIVE_ROOT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# add executable based on the source files determined in OSL_COMMON_SOURCES
add_executable(OSL_common
			   OSL_common.cxx
			   "${OSL_COMMON_SOURCES}")


# attempt to find Python path and libs
find_path(PYTHON_PATH python38.dll PATHS "C:/Users/Jere/AppData/Local/Programs/Python/Python38")
find_path(PYTHON_LIB python38.lib PATHS "${PYTHON_PATH}/libs")
# add and link the executables, libraries and directories for Python only if PYTHON_LIB was found
if(PYTHON_LIB)
	message([STATUS] "Python found.")
	target_link_libraries(OSL_common PRIVATE "${PYTHON_LIB}/python38.lib"
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}"
						PRIVATE "${MT_LIBRARY}"
						PRIVATE "ws2_32.lib")
	target_include_directories(OSL_common PRIVATE "${PYTHON_PATH}/include")
	# add a preprocessor definition
	add_definitions(-DPYTHON_ENABLED)
else()
	# if Python is not found, inform the user but continue CMake
	message([STATUS] "Cannot find Python. Removing PYTHON_LIB and PYTHON_PATH from list of CMake variables.")
	unset(PYTHON_LIB)
	unset(PYTHON_PATH)
	target_link_libraries(OSL_common
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}"
						PRIVATE "${MT_LIBRARY}"
						PRIVATE "ws2_32.lib")
	# add a preprocessor definition
	add_definitions(-DPYTHON_DISABLED)
endif()



						
target_include_directories(OSL_common PRIVATE
						"${INCLUDE_CLASSES_PATH}"
						"${INCLUDE_FUNCTIONS_PATH}")
						
target_include_directories(OSL_common PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   "${MTSDK_PATH}"
						   "${MT_LIB_PATH}"
						   )						   








						   