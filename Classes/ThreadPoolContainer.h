
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
        ThreadPoolContainer();
        ThreadPoolContainer(unsigned int maxThreads);
        ~ThreadPoolContainer();

        template<class F, class... Types>
        void offerFuture(F&& function, Types&&... args);



    private:
        std::vector<std::future<void>> vector_;
        ThreadPool* threadPool_;
        unsigned int maxThreads_;

    }; // end of class
}

#endif



using namespace OpenSimLive;

ThreadPoolContainer::ThreadPoolContainer() {}

ThreadPoolContainer::ThreadPoolContainer(unsigned int maxThreads) {
	threadPool_ = new ThreadPool(maxThreads);
	maxThreads_ = maxThreads;
}

ThreadPoolContainer::~ThreadPoolContainer() {
	delete threadPool_;
}


template<class F, class... Types>
void ThreadPoolContainer::offerFuture(F&& f, Types&&... args) {
	auto future = threadPool_->enqueue(std::forward<F>(f), std::forward<Types>(args)...);
	if (vector_.size() < maxThreads_)
	{
		vector_.push_back(std::move(future));
	}
	else
	{
		vector_.front().wait();
		vector_.erase(vector_.begin());
		vector_.push_back(std::move(future));
	}
}