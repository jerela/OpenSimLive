// This class is a user-friendly interface to operate ThreadPool. It limits the number of concurrent threads and forces the oldest thread to finish if another thread is offered while a maximum number of threads is already running.

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

#include <ThreadPoolContainer.tpp>

#endif