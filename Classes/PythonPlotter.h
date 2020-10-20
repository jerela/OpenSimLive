#pragma once


namespace OpenSimLive {

	class PythonPlotter {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		PythonPlotter();
		~PythonPlotter();

		// PUBLIC METHODS
		// append XData_ and YData_ to graph, update and scale axes
		void updateGraph();
		// set the maximum number of data points on the graph at a time
		void setMaxSize(unsigned int newMaxSize) { maxSize_ = newMaxSize; }
		// set the x value of the next data point to append
		void setXData(float x) { XData_ = x; }
		// set the y value of the next data point to append
		void setYData(float y) { YData_ = y; }
		// set the next data point to append
		void setData(float x, float y) { XData_ = x; YData_ = y; }
		
	protected:
			
	private:
		// PRIVATE METHODS
		// run preparatory Python commands
		void prepareGraph();
		// stop running Python when everything is finished
		void finalizeGraph();

		// PRIVATE VARIABLES
		// how many data points we want to see in the graph at a time
		unsigned int maxSize_ = 20;
		// next x value to append
		float XData_;
		// next y value to append
		float YData_;
		// number of data points plotted
		unsigned int numDataPoints_;
		// the limit of y-axis values
		float YLimit_ = 0.002;

	}; // end of class
}