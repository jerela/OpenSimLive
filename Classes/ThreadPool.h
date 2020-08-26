/* https://github.com/progschj/ThreadPool

Copyright(c) 2012 Jakob Progsch, Václav Zeman

This software is provided 'as-is', without any express or implied
warranty.In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter itand redistribute it
freely, subject to the following restrictions :

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software.If you use this software
in a product, an acknowledgment in the product documentation would be
appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.

3. This notice may not be removed or altered from any source
distribution.*/

#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

class ThreadPool {
public:
    ThreadPool(size_t); // constructor
    template<class F, class... Args> // template to allow enqueue to work with any input functions and their arguments
    auto enqueue(F&& f, Args&&... args)
        ->std::future<typename std::result_of<F(Args...)>::type>; // arrow operator describes the return type of enqueue (trailing return type)
    ~ThreadPool(); // deconstructor
private:
    // need to keep track of threads so we can join them
    std::vector< std::thread > workers; // workers is a vector of threads
    // the task queue
    std::queue< std::function<void()> > tasks; // tasks is a queue of functions that return void

    // synchronization
    std::mutex queue_mutex;
    std::condition_variable condition; // can be used to block a thread or multiple threads until another thread modifies a shared variable
    bool stop; // if threadpool is unused or destructed, stop is set to true
}; // end of class

// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads)
    : stop(false) // set stop to false when constructing
{
    for (size_t i = 0; i < threads; ++i) // iterate through all threads
        workers.emplace_back( // similar to push_back, but instead of copying or moving, the new element is constructed into the vector
            [this] // "this" returns a pointer to the ThreadPool object; square brackets for lambda expression
            {
                for (;;) // two semicolons indicate an infinite for loop
                {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, // if stop is false or tasks is empty, waiting continues; therefore this waits until stop is true or a task is given
                            [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) // if stop is true and there are no tasks, return from this endless loop
                            return;
                        task = std::move(this->tasks.front()); // move this task to the front of tasks queue
                        this->tasks.pop(); // removes the next element in the queue (the one we just moved)
                    }

                    task();
                }
            } // lambda expression ends
            ); // emplace_back ends
}

// add new work item to the pool
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared< std::packaged_task<return_type()> >(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        // don't allow enqueueing after stopping the pool
        if (stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");

        tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all(); // unblock threads
    for (std::thread& worker : workers) // iterate through all workers
        worker.join(); // join each worker thread
}

#endif