// This function calculates the values for all joint angles of the model based on live IMU data.
/*std::vector<double> OpenSimInverseKinematicsFromIMUs(std::string modelFileName, OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable, double duration, bool initialCall) {
	// Create a new IMUInverseKinematicsToolLive object
	OpenSimLive::IMUInverseKinematicsToolLive IKTool(modelFileName, quatTable);
	// set current time
	IKTool.setTime(duration);
	// run IK
	IKTool.run(true);
	// return the calculated joint angles
	return IKTool.getQ();
}*/


/*
std::vector<double> InverseKinematicsFromIMUs(std::vector<XsMatrix> matrixData, std::vector<MtwCallback*> mtwCallbacks, SimTK::Real timeInteger, std::string modelFileName, SimTK::Vec3 sensorToOpenSimRotations)
{

	int numberOfSensors = mtwCallbacks.size();

	std::cout << "Numbers of sensors: " << numberOfSensors << ", size of matrixData: " << matrixData.size() << std::endl;

	// create a vector for IMU names
	std::vector<std::string> sensorNameVector;
	// create a vector for time (in a single frame, so just a vector with a single value)
	std::vector<double> timeVector{ 0 };
	// create a matrix for IMU orientations
	SimTK::Matrix_<SimTK::Rotation_<double>> orientationDataMatrix(1, numberOfSensors); // marker positions go here

	// declare variables to be used in the loop
	double m11, m12, m13, m21, m22, m23, m31, m32, m33;
	SimTK::Rotation_<double> tempMatrix;
	XsMatrix xsMatrix;
	std::string currentSensorId; std::string sensorNameInModel;


	// insert orientation data from matrixData to orientationDataMatrix; note that orientationDataMatrix elements are 3x3 rotation matrices
	for (int k = 0; k < numberOfSensors; k++) {

		xsMatrix = matrixData[k];
		if (xsMatrix.empty()) {
			std::cout << "Matrix at k=" << k << " is empty. Returning." << std::endl;
			return { 0 };
		}

		// transform IMU orientation data to OpenSim coordinate system and set it to OrientationDataMatrix
		//orientationDataMatrix.set(0, k, transformOrientationToRotation(sensorToOpenSimRotations, xsMatrix));

		// give values to variables representing individual elements of a 3x3 rotation matrix
		m11 = xsMatrix.value(0, 0);
		m12 = xsMatrix.value(0, 1);
		m13 = xsMatrix.value(0, 2);
		m21 = xsMatrix.value(1, 0);
		m22 = xsMatrix.value(1, 1);
		m23 = xsMatrix.value(1, 2);
		m31 = xsMatrix.value(2, 0);
		m32 = xsMatrix.value(2, 1);
		m33 = xsMatrix.value(2, 2);
		std::cout << "[" << m11 << "; " << m12 << "; " << m13 << " " << std::endl;
		std::cout << " " << m21 << "; " << m22 << "; " << m23 << " " << std::endl;
		std::cout << " " << m31 << "; " << m32 << "; " << m33 << "]" << std::endl;

		// set these values to tempMatrix
		tempMatrix.set(0, 0, m11);
		tempMatrix.set(0, 1, m12);
		tempMatrix.set(0, 2, m13);
		tempMatrix.set(1, 0, m21);
		tempMatrix.set(1, 1, m22);
		tempMatrix.set(1, 2, m23);
		tempMatrix.set(2, 0, m31);
		tempMatrix.set(2, 1, m32);
		tempMatrix.set(2, 2, m33);
		
		// fill orientationDataMatrix with orientation matrices for each sensor
		orientationDataMatrix.set(0, k, tempMatrix);

		// populate sensorNameVector with names of the IMUs on the model

		// initialize currentSensorId with the serial of the IMU
		currentSensorId = mtwCallbacks[k]->device().deviceId().toString().toStdString();
		
		sensorNameInModel = sensorIdToLabel(currentSensorId, "C:/Users/wksadmin/source/repos/OpenSimLive/Config/SensorMappings.xml");

		if (sensorNameInModel != "NotFound"){
			// push the name of the IMU in the model into sensorNameVector
			sensorNameVector.push_back(sensorNameInModel);
		}
		else{
			std::cout << "No match found for sensor " << currentSensorId << std::endl;
		}
	}

	std::cout << "For-loop finished, preparing OpenSim API methods for IK." << std::endl;

	// form a timeseriestable of the IMU orientations
	OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> timeSeriesTable(timeVector, orientationDataMatrix, sensorNameVector);

	// load model
	OpenSim::Model model(modelFileName);

	//std::cout << "TimeSeriesTable and Model objects created, preparing to initialize state." << std::endl;

	// enable visualizer
	model.setUseVisualizer(true);

	// create state
	SimTK::State s = model.initSystem();

	SimTK::Visualizer &visualizer = model.updVisualizer().updSimbodyVisualizer();

	visualizer.setMode(SimTK::Visualizer::Mode::PassThrough);

	// update state time
	//s.updTime() = timeInteger/1000;

	//std::cout << "State initialized, creating Reference objects." << std::endl;

	// create MarkersReference object
	OpenSim::MarkersReference markersReference;

	// create OrientationsReference object
	OpenSim::OrientationsReference orientationsReference(timeSeriesTable);

	// create coordinate reference
	SimTK::Array_<OpenSim::CoordinateReference> coordinateReference;

	//std::cout << "Reference objects created, constructing InverseKinematicsSolver" << std::endl;

	// use inversekinematicssolver with markersreference to solve
	OpenSim::InverseKinematicsSolver iks(model, markersReference, orientationsReference, coordinateReference, SimTK::Infinity);

	iks.setAccuracy(1e-4);
	bool isAssembled = false;
	while (!isAssembled) {
		try {
			iks.assemble(s);
			isAssembled = true;
		}
		catch (...) {
			std::cerr << "Model not assembled" << std::endl;
		}
	}

	//std::cout << "Preparing to track state s" << std::endl;
	// calculate coordinates for state s
	iks.track(s);

	// get coordinates from state s
	SimTK::Vector stateQ(s.getQ());
	// get number of coordinates (joint angles) in the model
	int numCoordinates = model.getNumCoordinates();
	std::vector<double> q(numCoordinates);
	for (int j = 0; j < numCoordinates; j++) {
		q[j] = stateQ[j];
		std::cout << "Q" << j << ": " << q[j] << std::endl;
	}


	//OpenSim::ModelVisualizer::show(s);
	model.getVisualizer().show(s);

	std::cout << "IK finished successfully." << std::endl;
	return q;
}
*/








// this function creates a calibrated .osim model from live data (as opposed to data exported after recording from MT manager)
/*std::string calibrateOpenSimModel(std::vector<MtwCallback*> mtwCallbacks, std::vector<XsMatrix> initialMatrix, SimTK::Vec3 sensorToOpenSimRotations){

	size_t callbacksSize = mtwCallbacks.size();

	// model used as a base for the calibrated model
	OpenSim::Model baseModel("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full.osim");
	std::string baseIMU("pelvis_imu");
	std::string baseHeadingAxis("-z");
	std::string outModelFile("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full_calib.osim");
	
	// let's calculate that in OpenSim coordinate system

	// the next step is to add PhysicalOffsetFrames representing the IMUs under each body in the model

	SimTK::Xml::Document baseModelFile("C:/Users/wksadmin/source/repos/OpenSimLive/gait2392_full.osim");
	// get the root element of the XML file
	SimTK::Xml::Element rootElement = baseModelFile.getRootElement();
	// get the child element of the root element
	SimTK::Xml::Element modelElement = rootElement.getRequiredElement("Model");
	// get the child element of the child element of the root element
	SimTK::Xml::Element bodySetElement = modelElement.getRequiredElement("BodySet");
	// get the child element of the BodySet element
	SimTK::Xml::Element objectsElement = bodySetElement.getRequiredElement("objects");
	// get all child elements with tag <Body> of the element <objects>
	SimTK::Array_<SimTK::Xml::Element> bodyElements = objectsElement.getAllElements("Body");
	// get the number of sensors defined in the XML file
	int numberOfBodies = bodyElements.size();

	std::string sensorLabel; std::string currentSensorId; std::string bodyName; std::string currentBody;

	// iterate through all sensors that are active
	for (size_t k = 0; k < callbacksSize; ++k) {
		// get ID/serial of currently iterated sensor
		currentSensorId = mtwCallbacks[k]->device().deviceId().toString().toStdString();
		// get the corresponding label
		sensorLabel = sensorIdToLabel(currentSensorId, "C:/Users/wksadmin/source/repos/OpenSimLive/Config/SensorMappings.xml");
		bodyName = sensorLabel;
		// remove 4 trailing characters ("_imu") from bodyName
		for (int m = 0; m < 4; ++m) {
			bodyName.pop_back();
		}
		// iterate through bodies in the model until a match to bodyName is found
		for (int m = 0; m < numberOfBodies; ++m) {
			currentBody = bodyElements.at(m).getRequiredAttributeValue("name");
			// if we find a match, add PhysicalOffsetFrame for the IMU
			if (bodyName == currentBody) {
				// create components
				//SimTK::Xml::Element componentsElement = bodyElements[m].getRequiredElement("components");
				SimTK::Xml::Element componentsElement("components");
				// create the new XML element to be inserted
				SimTK::Xml::Element physicalOffsetFrameElement("PhysicalOffsetFrame");
				physicalOffsetFrameElement.setAttributeValue("name", sensorLabel);
				// create child elements of the new XML element
				SimTK::Xml::Element frameGeometryElement("FrameGeometry");
				frameGeometryElement.setAttributeValue("name", "frame_geometry");
				SimTK::Xml::Element socketFrameFGElement("socket_frame", "..");
				SimTK::Xml::Element scaleFactorsElement("scale_factors", "0.2 0.2 0.2");
				frameGeometryElement.appendNode(socketFrameFGElement);
				frameGeometryElement.appendNode(scaleFactorsElement);
				SimTK::Xml::Element attachedGeometryElement("attached_geometry");
				SimTK::Xml::Element brickElement("Brick");
				brickElement.setAttributeValue("name", sensorLabel+"_geom1");
				SimTK::Xml::Element socketFrameBElement("socket_frame", "..");
				SimTK::Xml::Element appearanceElement("Appearance");
				SimTK::Xml::Element colorElement("color", "1 0.5 0");
				appearanceElement.appendNode(colorElement);
				SimTK::Xml::Element halfLengthsElement("half_lengths", "0.02 0.01 0.005");
				brickElement.appendNode(socketFrameBElement);
				brickElement.appendNode(appearanceElement);
				brickElement.appendNode(halfLengthsElement);
				attachedGeometryElement.appendNode(brickElement);
				SimTK::Xml::Element socketParentElement("socket_parent", "..");
				SimTK::Xml::Element translationElement("translation", "-0.0707 0 0");
				// orientation must be calculated from initial IMU orientations and given as a string with 3 elements separated by whitespaces
				SimTK::Vec3 finalIMUOrientation = transformOrientationToEuler(sensorToOpenSimRotations, initialMatrix[k]);
				std::string orientationVector = std::to_string(finalIMUOrientation[0]) + " " + std::to_string(finalIMUOrientation[1]) + " " + std::to_string(finalIMUOrientation[2]);
				SimTK::Xml::Element orientationElement("orientation", orientationVector);
				// insert child elements into the XML element to be inserted
				physicalOffsetFrameElement.appendNode(frameGeometryElement);
				physicalOffsetFrameElement.appendNode(attachedGeometryElement);
				physicalOffsetFrameElement.appendNode(socketParentElement);
				physicalOffsetFrameElement.appendNode(translationElement);
				physicalOffsetFrameElement.appendNode(orientationElement);
				// insert the new XML element under the parent node
				componentsElement.appendNode(physicalOffsetFrameElement);
				bodyElements[m].appendNode(componentsElement);
			}
		}
	}

	// write the modified model file into an XML document
	baseModelFile.writeToFile(outModelFile);

	return outModelFile;
}*/






/*SimTK::Rotation_<double> transformOrientationToRotation(SimTK::Vec3 sensorToOpenSim, XsMatrix sensorMatrix) {

	std::cout << "Euler for sensor to OpenSim: [" << sensorToOpenSim[0] << ", " << sensorToOpenSim[1] << ", " << sensorToOpenSim[2] << "]" << std::endl;

	// convert Euler angles to rotation matrices
	SimTK::Matrix_<double> sensorToOpenSimMatrix = eulerToRotationMatrix(sensorToOpenSim);
	std::cout << "Euler angles converted to rotation matrices." << std::endl;

	std::cout << "Rotation matrix for IMU:" << std::endl;
	std::cout << "[" << sensorMatrix.value(0, 0) << ", " << sensorMatrix.value(0, 1) << ", " << sensorMatrix.value(0, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(1, 0) << ", " << sensorMatrix.value(1, 1) << ", " << sensorMatrix.value(1, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(2, 0) << ", " << sensorMatrix.value(2, 1) << ", " << sensorMatrix.value(2, 2) << "]" << std::endl;

	std::cout << "Rotation matrix for sensor to OpenSim:" << std::endl;
	std::cout << "[" << sensorToOpenSimMatrix.get(0, 0) << ", " << sensorToOpenSimMatrix.get(0, 1) << ", " << sensorToOpenSimMatrix.get(0, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(1, 0) << ", " << sensorToOpenSimMatrix.get(1, 1) << ", " << sensorToOpenSimMatrix.get(1, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(2, 0) << ", " << sensorToOpenSimMatrix.get(2, 1) << ", " << sensorToOpenSimMatrix.get(2, 2) << "]" << std::endl;

	// convert sensor rotation matrix from Xs to SimTK
	SimTK::Matrix_<double> sensorMatrixSimTK(3, 3);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sensorMatrixSimTK.set(i, j, sensorMatrix.value(i, j));
		}
	}


	// calculate the matrix product
	SimTK::Matrix_<double> productMatrix = matrixMultiplication(sensorToOpenSimMatrix, sensorMatrixSimTK);
	std::cout << "Matrix product calculated." << std::endl;

	SimTK::Rotation_<double> returnMatrix;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			returnMatrix.set(i, j, productMatrix.get(i, j));
		}
	}

	return returnMatrix;
}*/




/*SimTK::Vec3 transformOrientationToEuler(SimTK::Vec3 sensorToOpenSim, XsMatrix sensorMatrix) {

	std::cout << "Euler for sensor to OpenSim: [" << sensorToOpenSim[0] << ", " << sensorToOpenSim[1] << ", " << sensorToOpenSim[2] << "]" << std::endl;

	// convert Euler angles to rotation matrices
	SimTK::Matrix_<double> sensorToOpenSimMatrix = eulerToRotationMatrix(sensorToOpenSim);
	std::cout << "Euler angles converted to rotation matrices." << std::endl;

	std::cout << "Rotation matrix for IMU:" << std::endl;
	std::cout << "[" << sensorMatrix.value(0, 0) << ", " << sensorMatrix.value(0, 1) << ", " << sensorMatrix.value(0, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(1, 0) << ", " << sensorMatrix.value(1, 1) << ", " << sensorMatrix.value(1, 2) << std::endl;
	std::cout << " " << sensorMatrix.value(2, 0) << ", " << sensorMatrix.value(2, 1) << ", " << sensorMatrix.value(2, 2) << "]" << std::endl;

	std::cout << "Rotation matrix for sensor to OpenSim:" << std::endl;
	std::cout << "[" << sensorToOpenSimMatrix.get(0, 0) << ", " << sensorToOpenSimMatrix.get(0, 1) << ", " << sensorToOpenSimMatrix.get(0, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(1, 0) << ", " << sensorToOpenSimMatrix.get(1, 1) << ", " << sensorToOpenSimMatrix.get(1, 2) << std::endl;
	std::cout << " " << sensorToOpenSimMatrix.get(2, 0) << ", " << sensorToOpenSimMatrix.get(2, 1) << ", " << sensorToOpenSimMatrix.get(2, 2) << "]" << std::endl;

	// convert sensor rotation matrix from Xs to SimTK
	SimTK::Matrix_<double> sensorMatrixSimTK(3,3);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sensorMatrixSimTK.set(i, j, sensorMatrix.value(i, j));
		}
	}
	
	
	// calculate the matrix product
	SimTK::Matrix_<double> productMatrix = matrixMultiplication(sensorToOpenSimMatrix, sensorMatrixSimTK);
	std::cout << "Matrix product calculated." << std::endl;

	// convert it back into Euler angles
	SimTK::Vec3 finalEuler = rotationMatrixToEuler(productMatrix);
	std::cout << "Rotation matrix converted into Euler angles:" << std::endl;
	std::cout << "[" << finalEuler[0] << ", " << finalEuler[1] << ", " << finalEuler[2] << "]" << std::endl;
	return finalEuler;
}*/




/*
// this function takes as input two euler angle sets, converts them to rotation matrices, rotates the other set and returns it as Euler angles
SimTK::Vec3 transformEuler(SimTK::Vec3 sensorToOpenSim, SimTK::Vec3 sensorEuler) {
	
	std::cout << "Euler for sensor to OpenSim: [" << sensorToOpenSim[0] << ", " << sensorToOpenSim[1] << ", " << sensorToOpenSim[2] << "]" << std::endl;
	std::cout << "Euler for IMU sensor: [" << sensorEuler[0] << ", " << sensorEuler[1] << ", " << sensorEuler[2] << "]" << std::endl;

	// convert Euler angles to rotation matrices
	SimTK::Matrix_<double> sensorToOpenSimMatrix = eulerToRotationMatrix(sensorToOpenSim);
	SimTK::Matrix_<double> sensorMatrix = eulerToRotationMatrix(sensorEuler);
	std::cout << "Euler angles converted to rotation matrices." << std::endl;

	std::cout << "Rotation matrix for IMU:" << std::endl;
	std::cout << "[" << sensorMatrix.get(0, 0) << ", " << sensorMatrix.get(0, 1) << ", " << sensorMatrix.get(0, 2) << std::endl;
	std::cout << " " << sensorMatrix.get(1, 0) << ", " << sensorMatrix.get(1, 1) << ", " << sensorMatrix.get(1, 2) << std::endl;
	std::cout << " " << sensorMatrix.get(0, 0) << ", " << sensorMatrix.get(0, 1) << ", " << sensorMatrix.get(0, 2) << "]" << std::endl;

	// calculate the matrix product
	SimTK::Matrix_<double> productMatrix = matrixMultiplication(sensorToOpenSimMatrix, sensorMatrix);
	std::cout << "Matrix product calculated." << std::endl;

	// convert it back into Euler angles
	SimTK::Vec3 finalEuler = rotationMatrixToEuler(productMatrix);
	std::cout << "Rotation matrix converted into Euler angles:" << std::endl;
	std::cout << "[" << finalEuler[0] << ", " << finalEuler[1] << ", " << finalEuler[2] << "]" << std::endl;
	return finalEuler;
}
*/





// performs matrix multiplication on 3x3 matrices and returns the product
/*SimTK::Matrix_<double> matrixMultiplication(SimTK::Matrix_<double> firstMatrix, SimTK::Matrix_<double> secondMatrix) {

	int numberOfFirstRows = firstMatrix.nrow();
	int numberOfFirstCols = firstMatrix.ncol();
	int numberOfSecondRows = secondMatrix.nrow();
	int numberOfSecondCols = secondMatrix.ncol();

	if (numberOfFirstCols != numberOfSecondRows) {
		std::cerr << "Multiplication is not defined for matrix dimensions!" << std::endl;
	}

	// construct the product matrix
	//SimTK::Matrix_<double> productMatrix(3, 3);
	SimTK::Matrix_<double> productMatrix(numberOfFirstRows, numberOfSecondCols);
	double value;

	// for each row in the first matrix
	for (int row = 0; row < numberOfSecondRows; ++row) {
		// for each col in the second matrix
		for (int col = 0; col < numberOfFirstCols; ++col) {
			value = 0;
			// for each row in the second matrix
			for (int rows = 0; rows < numberOfSecondRows; ++rows) {
				value += firstMatrix.get(row, rows) * secondMatrix.get(rows, col);
			}
			productMatrix.set(row, col, value);
		}
	}

	return productMatrix;
}*/




/*SimTK::Vec3 rotationMatrixToEuler(SimTK::Matrix_<double> matrix) {
	double m11 = matrix.get(0, 0);
	double m12 = matrix.get(0, 1);
	double m13 = matrix.get(0, 2);
	double m21 = matrix.get(1, 0);
	double m22 = matrix.get(1, 1);
	double m23 = matrix.get(1, 2);
	double m31 = matrix.get(2, 0);
	double m32 = matrix.get(2, 1);
	double m33 = matrix.get(2, 2);

	// get Euler angles as radians
	double pitch = asin(m13);
	double yaw = acos(m11 / cos(pitch));
	double roll = acos(m33 / cos(pitch));
	SimTK::Vec3 euler(roll, pitch, yaw);
	return euler;
}*/





// this function creates a rotation matrix out of Euler angles
/*SimTK::Matrix_<double> eulerToRotationMatrix(SimTK::Vec3 euler) {
	// define the elements of the rotation matrix
	double c1 = cos(euler[0]);
	double c2 = cos(euler[1]);
	double c3 = cos(euler[2]);
	double s1 = sin(euler[0]);
	double s2 = sin(euler[1]);
	double s3 = sin(euler[2]);
	// create a 3x3 matrix
	SimTK::Matrix_<double> rotationMatrix(3, 3);
	// set the values of the matrix
	rotationMatrix.set(0, 0, c2 * c3);
	rotationMatrix.set(0, 1, -c2 * s3);
	rotationMatrix.set(0, 2, s2);
	rotationMatrix.set(1, 0, c1 * s3 + c3 * s1 * s2);
	rotationMatrix.set(1, 1, c1 * c3 - s1 * s2 * s3);
	rotationMatrix.set(1, 2, -c2 * s1);
	rotationMatrix.set(2, 0, s1 * s3 - c1 * c3 * s2);
	rotationMatrix.set(2, 1, c3 * s1 + c1 * s2 * s3);
	rotationMatrix.set(2, 2, c1 * c2);

	return rotationMatrix;
}*/





// this function converts degrees to radians
/*double deg2rad(double deg) {
	return (deg * acos(0.0) / 90);
}*/


