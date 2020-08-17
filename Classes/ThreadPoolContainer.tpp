// template implementation of the offerFuture method

using namespace OpenSimLive;

// template to make the method work with any datatype (function in this case) and its arguments
template<class F, class... Types>
void ThreadPoolContainer::offerFuture(F&& f, Types&&... args) {
	// limit the number of worker threads to maxThreads_
	if (vector_.size() < maxThreads_)
	{
		// enqueue the actual thread using ThreadPool class
		auto future = threadPool_->enqueue(std::forward<F>(f), std::forward<Types>(args)...);
		vector_.push_back(std::move(future));
	}
	else
	{
		vector_.front().wait(); // if there are maxThreads_ or more threads, wait until the oldest one finishes
		vector_.erase(vector_.begin()); // then remove it from the vector
		// enqueue the actual thread using ThreadPool class
		auto future = threadPool_->enqueue(std::forward<F>(f), std::forward<Types>(args)...);
		vector_.push_back(std::move(future)); // and push a new thread into the vector
	}
}