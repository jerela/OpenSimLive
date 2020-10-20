#pragma once

#include <array>

namespace OpenSimLive {

	class PythonPlotter {

	public:
		// CONSTRUCTORS AND DECONSTRUCTORS
		PythonPlotter();
		~PythonPlotter();

		// PUBLIC METHODS
		// run preparatory Python commands
		void prepareGraph();
		// append XData_ and YData_ to graph, update and scale axes
		void updateGraph();
		// set the maximum number of data points on the graph at a time
		void setMaxSize(unsigned int newMaxSize) { maxSize_ = newMaxSize; }
		// set the x value of the next data point to append
		void setXData(std::array<float, 16> x) { XData_ = x; }
		// set the y value of the next data point to append
		void setYData(std::array<float, 16> y) { YData_ = y; }
		// set the next data point to append
		void setData(std::array<float, 16> x, std::array<float, 16> y) { XData_ = x; YData_ = y; }
		// set the number of subplots
		void setSubPlots(unsigned int n) { numSubPlots_ = n; }
		
	protected:
			
	private:
		// PRIVATE METHODS
		// stop running Python when everything is finished
		void finalizeGraph();

		// PRIVATE VARIABLES
		// how many separate subplots we have
		unsigned int numSubPlots_ = 1;
		// how many data points we want to see in the graph at a time
		unsigned int maxSize_ = 20;
		// next x value to append
		std::array<float, 16> XData_ = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		// next y value to append
		std::array<float,16> YData_ = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		// number of data points plotted
		unsigned int numDataPoints_;
		// the limit of y-axis values
		float YLimit_ = 0.002;

		bool asd_ = true;

	}; // end of class
}