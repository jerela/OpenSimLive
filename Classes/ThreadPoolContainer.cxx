//ThreadPoolContainer.cxx
/*
#include <ThreadPoolContainer.h>
//#include <ThreadPool.h>


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
}*/