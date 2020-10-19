// PythonPlotter.cxx

#include <Python.h>
#include <PythonPlotter.h>
#include <string>
#include <iostream>

const std::string OPENSIMLIVE_ROOT = OPENSIMLIVE_ROOT_PATH;

using namespace OpenSimLive;

// CONSTRUCTOR
PythonPlotter::PythonPlotter() {
	prepareGraph();
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

}

// Append X and Y data points to graph
void PythonPlotter::updateGraph() {
	// append data to x and y axes
	std::string yAppend = "y.append(" + std::to_string(YData_) + ")";
	PyRun_SimpleString(yAppend.c_str());
	std::string xAppend = "x.append(" + std::to_string(XData_) + ")";
	PyRun_SimpleString(xAppend.c_str());

	// if there would be more than the maximum number of data points (maxSize) on the graph, pop the oldest data point
	if (numDataPoints_ >= maxSize_) {
		PyRun_SimpleString("y.popleft()");
		PyRun_SimpleString("x.popleft()");
	}

	//PyRun_SimpleString("print(x)");
	//PyRun_SimpleString("print(y)");

	// feed x and y data to the graph
	PyRun_SimpleString("line1.set_ydata(y)");
	PyRun_SimpleString("line1.set_xdata(x)");
	// automatically scale the graph and update the axis limits
	PyRun_SimpleString("ax.autoscale_view()");
	PyRun_SimpleString("ax.relim()");
	// draw the graph
	PyRun_SimpleString("fig.canvas.draw()");
	PyRun_SimpleString("pyplot.text(-0.5, 0.3, 'This is text')");
	// stop momentarily so the user can see the graph
	PyRun_SimpleString("pyplot.pause(0.001)");
}

void PythonPlotter::finalizeGraph() {
	Py_Finalize();
}

