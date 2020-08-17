
#ifndef THREAD_POOL_CONTAINER_H
#define THREAD_POOL_CONTAINER_H

#include <ThreadPool.h>
#include <future>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>


namespace OpenSimLive
{

    class ThreadPoolContainer {
    public:
		// CONSTRUCTORS
        ThreadPoolContainer();
        ThreadPoolContainer(unsigned int maxThreads);
        ~ThreadPoolContainer();

		// METHODS
        template<class F, class... Types>
        void offerFuture(F&& function, Types&&... args); // we use this function to feed functions to the pool

    private:
        std::vector<std::future<void>> vector_; // this vector stores the future threads
        ThreadPool* threadPool_; // this points to the ThreadPool object that handles the actual pooling
        unsigned int maxThreads_; // the maximum number of concurrent threads

    }; // end of class
}

#endif



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

// template to make the method work with any datatype (function in this case) and its arguments
template<class F, class... Types>
void ThreadPoolContainer::offerFuture(F&& f, Types&&... args) {
	// enqueue the actual thread using ThreadPool class
	auto future = threadPool_->enqueue(std::forward<F>(f), std::forward<Types>(args)...);
	// limit the number of worker threads to maxThreads_
	if (vector_.size() < maxThreads_)
	{
		vector_.push_back(std::move(future));
	}
	else
	{
		vector_.front().wait(); // if there are maxThreads_ or more threads, wait until the oldest one finishes
		vector_.erase(vector_.begin()); // then remove it from the vector
		vector_.push_back(std::move(future)); // and push a new thread into the vector
	}
}