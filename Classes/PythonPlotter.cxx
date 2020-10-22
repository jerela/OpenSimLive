// PythonPlotter.cxx
#ifdef PYTHON_ENABLED
#include <Python.h>
#include <PythonPlotter.h>
#include <string>
#include <iostream>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
PythonPlotter::PythonPlotter() {
}

// DESTRUCTOR
PythonPlotter::~PythonPlotter() {
	finalizeGraph();
}

// Run preparatory Python script
void PythonPlotter::prepareGraph() {
	try {
		// read python code from file
		std::string pythonScriptPath = OPENSIMLIVE_ROOT + "/Python/prepareGraph.py";
		//char filename[] = pythonScriptPath.c_str();
		FILE* fp;

		// Python startup code
		Py_Initialize();

		// open the file containing python code
		//fp = _Py_fopen(filename, "r");
		fp = _Py_fopen(pythonScriptPath.c_str(), "r");
		// run the python script
		//PyRun_SimpleFile(fp, filename);
		PyRun_SimpleFile(fp, pythonScriptPath.c_str());
	}
	catch (std::exception& e) {
		std::cerr << "Error in PythonPlotter::prepareGraph(): " << e.what() << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown error in PythonPlotter::prepareGraph()" << std::endl;
	}

	for (unsigned int i = 1; i < numSubPlots_ + 1; ++i) {
		// create subplots
		std::string addSubPlot = "plt" + std::to_string(i) + " = fig.add_subplot(" + std::to_string(numSubPlots_) + ", 1, " + std::to_string(i) + ", autoscale_on=\"True\")";
		PyRun_SimpleString(addSubPlot.c_str());
		// set initial y-axis limits
		std::string yLimitUpdate = "plt" + std::to_string(i) + ".set_ylim(bottom=" + std::to_string(-YLimit_) + ", top=" + std::to_string(YLimit_) + ")";
		PyRun_SimpleString(yLimitUpdate.c_str());
		// create x and y deques for each subplot
		std::string createDequeX = "x" + std::to_string(i) + " = deque([0]," + std::to_string(maxSize_) + ")";
		PyRun_SimpleString(createDequeX.c_str());
		std::string createDequeY = "y" + std::to_string(i) + " = deque([0]," + std::to_string(maxSize_) + ")";
		PyRun_SimpleString(createDequeY.c_str());
		std::string plotString = "line" + std::to_string(i) + ", = plt" + std::to_string(i) + ".plot(x" + std::to_string(i) + ", y" + std::to_string(i) + ", 'r-')";
		PyRun_SimpleString(plotString.c_str());
	}
	


}

// Append X and Y data points to graph
void PythonPlotter::updateGraph() {
	for (unsigned int i = 1; i < numSubPlots_ + 1; ++i) {

		// append data to x and y axes
		std::string xAppend = "x" + std::to_string(i) + ".append(" + std::to_string(XData_[i - 1]) + ")";
		PyRun_SimpleString(xAppend.c_str());
		std::string yAppend = "y" + std::to_string(i) + ".append(" + std::to_string(YData_[i - 1]) + ")";
		PyRun_SimpleString(yAppend.c_str());

		// feed x and y data to the graph
		std::string setXDataString = "line" + std::to_string(i) + ".set_xdata(x" + std::to_string(i) + ")";
		PyRun_SimpleString(setXDataString.c_str());
		std::string setYDataString = "line" + std::to_string(i) + ".set_ydata(y" + std::to_string(i) + ")";
		PyRun_SimpleString(setYDataString.c_str());


		// update y-axis limit if required
		if (YLimit_ < YData_[i - 1])
		{
			YLimit_ = YData_[i - 1];
			std::string yLimitUpdate = "plt" + std::to_string(i) + ".set_ylim(bottom=" + std::to_string(-YLimit_) + ", top=" + std::to_string(YLimit_) + ")";
			PyRun_SimpleString(yLimitUpdate.c_str());
		}

		// automatically scale the graph and update the axis limits
		std::string autoScalePlotString = "plt" + std::to_string(i) + ".autoscale_view(scalex=True, scaley=False)";
		PyRun_SimpleString(autoScalePlotString.c_str());
		std::string relimString = "plt" + std::to_string(i) + ".relim()";
		PyRun_SimpleString(relimString.c_str());

		//PyRun_SimpleString("pyplot.text(-0.5, 0.3, 'This is text')");
		
	}
	// draw the graph
	PyRun_SimpleString("fig.canvas.draw()");
//	PyRun_SimpleString("pyplot.show(block=False)");
	// stop momentarily so the user can see the graph
	PyRun_SimpleString("print(len(x1))");
	PyRun_SimpleString("print(len(y1))");
	PyRun_SimpleString("pyplot.pause(0.001)");
}

void PythonPlotter::finalizeGraph() {
	Py_Finalize();
}

#endif