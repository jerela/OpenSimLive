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
			std::cout << vector_.size() << std::endl;
			bool isReady = (vector_[0].wait_for(std::chrono::seconds(0)) == std::future_status::ready);
			if (isReady) {
				vector_.erase(vector_.begin());
			}
		}
		
	}
	
}

// delete the object created with keyword new
ThreadPoolContainer::~ThreadPoolContainer() {
	delete threadPool_;
}

