#include <ThreadPoolContainer.h>
#include <iostream>

using namespace OpenSimLive;

ThreadPoolContainer::ThreadPoolContainer() {}

// Set the value of maxThreads_ and make threadPool_ pointer point to a new ThreadPool object
ThreadPoolContainer::ThreadPoolContainer(unsigned int maxThreads) {
	threadPool_ = new ThreadPool(maxThreads);
	maxThreads_ = maxThreads;
}

// Wait until there are no more futures running (vector_ is empty)
void ThreadPoolContainer::waitForFinish() {

	bool allDone = false;
	while (!allDone) {
		if (vector_.size() == 0) {
			allDone = true;
		}
		else{
			std::cout << "Waiting for all threads in the thread pool to finish. Threads remaining: " << vector_.size() << std::endl;
			vector_[0].wait();
			vector_.erase(vector_.begin());
		}
		
	}
	std::cout << "All threads in the thread pool have finished." << std::endl;
	
}

// delete the object created with keyword new
ThreadPoolContainer::~ThreadPoolContainer() {
	delete threadPool_;
}

