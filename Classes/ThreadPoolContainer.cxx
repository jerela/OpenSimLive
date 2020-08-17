#include <ThreadPoolContainer.h>

using namespace OpenSimLive;

ThreadPoolContainer::ThreadPoolContainer() {}

// Set the value of maxThreads_ and make threadPool_ pointer point to a new ThreadPool object
ThreadPoolContainer::ThreadPoolContainer(unsigned int maxThreads) {
	threadPool_ = new ThreadPool(maxThreads);
	maxThreads_ = maxThreads;
}

// delete the object created with keyword new
ThreadPoolContainer::~ThreadPoolContainer() {
	delete threadPool_;
}

