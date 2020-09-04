
# find the path to OpenSimlive/Config/
find_path(CONFIG_PATH IMUPlacerSetup.xml.in PATHS "${CMAKE_CURRENT_SOURCE_DIR}")

# configure IMUPlacerSetup.xml to contain the correct file path
configure_file("${CONFIG_PATH}/IMUPlacerSetup.xml.in"
			   "${CONFIG_PATH}/IMUPlacerSetup.xml")