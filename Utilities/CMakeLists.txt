set(OFFLINE_IK_TOOL_SOURCES
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUPlacerLive.cxx"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.h"
	"${INCLUDE_CLASSES_PATH}/IMUInverseKinematicsToolLive.cxx"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.h"
	"${INCLUDE_CLASSES_PATH}/DecorationGeneratorLive.cxx"
	"${INCLUDE_CLASSES_PATH}/PointTracker.h"
	"${INCLUDE_CLASSES_PATH}/PointTracker.cxx"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.h"
	"${INCLUDE_FUNCTIONS_PATH}/XMLFunctions.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPool.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.h"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.cxx"
	"${INCLUDE_CLASSES_PATH}/ThreadPoolContainer.tpp")

# add executable based on the source files determined in OFFLINE_IK_TOOL_SOURCES
add_executable(offline_IK_tool offline_IK_tool.cxx "${OFFLINE_IK_TOOL_SOURCES}")

target_link_libraries(offline_IK_tool
						PRIVATE "${OPENSIM_LIBRARY}"
						PRIVATE "${SIMBODY_LIBRARY}")
						
target_include_directories(offline_IK_tool PRIVATE
						"${INCLUDE_CLASSES_PATH}"
						"${INCLUDE_FUNCTIONS_PATH}")
target_include_directories(offline_IK_tool PUBLIC
						   "${PROJECT_BINARY_DIR}"
						   "${OPENSIM_INCLUDE_PATH}"
						   "${OPENSIM_INCLUDE_PATH_TWO}"
						   "${SIMBODY_INCLUDE_PATH}"
						   )
						   
						   
						   
						   
