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
		// boolean to indicate if we've found a thread (future) that is finished
		bool isReady = false;
		// index for checking ready states in the vector of futures
		unsigned int index = 0;
		// loop until we find a finished thread
		do{	
			// if index equals the number of threads in the vector, set index back to 0 so we start checking from the oldest thread again
			if (index >= vector_.size()){
				index = 0;
			}
			// update the boolean by checking if the index'th thread in the vector is ready
			isReady = (vector_[index].wait_for(std::chrono::seconds(0)) == std::future_status::ready);
			// increment index to check the next thread in the vector in the next iteration
			++index;
			
		} while(!isReady);
		
		// after we've identified the finished thread, remove it from the vector
		vector_.erase(vector_.begin()+index-1);

		// enqueue the actual thread using ThreadPool class
		auto future = threadPool_->enqueue(std::forward<F>(f), std::forward<Types>(args)...);
		vector_.push_back(std::move(future)); // and push a new thread into the vector
	}
}