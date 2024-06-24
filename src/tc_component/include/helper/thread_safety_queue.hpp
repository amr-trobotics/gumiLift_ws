#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <iostream>

template <typename T>
class ThreadSafeQueue 
{
private:
    std::mutex mutex_;
    std::queue<T> queue_;

public:
    void push(T item) 
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(item);
        }
    }

    T& front() 
    {
        std::lock_guard<std::mutex> lock(mutex_);        
        return queue_.front();
    }

    void pop() 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.pop();
    }

    int count()
    {
        return queue_.size();
    }
};

#endif