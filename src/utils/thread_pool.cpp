#include "multibotnet/utils/thread_pool.hpp"
#include <algorithm>

namespace multibotnet {

ThreadPool::ThreadPool(size_t num_threads) 
    : stop_(false), active_tasks_(0) {
    workers_.reserve(num_threads);
    for (size_t i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&ThreadPool::worker, this);
    }
}

ThreadPool::~ThreadPool() {
    stop();
}

size_t ThreadPool::pending() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return tasks_.size();
}

void ThreadPool::wait() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    finished_condition_.wait(lock, [this] { 
        return tasks_.empty() && active_tasks_ == 0; 
    });
}

void ThreadPool::stop() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (stop_) return;
        stop_ = true;
    }
    
    condition_.notify_all();
    
    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    
    workers_.clear();
}

void ThreadPool::resize(size_t new_size) {
    if (new_size == workers_.size()) {
        return;
    }
    
    if (new_size < workers_.size()) {
        // 减少线程数
        size_t threads_to_remove = workers_.size() - new_size;
        
        // 标记要停止的线程
        for (size_t i = 0; i < threads_to_remove; ++i) {
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                tasks_.emplace([]() {
                    throw std::runtime_error("Thread termination");
                });
            }
            condition_.notify_one();
        }
        
        // 等待线程结束
        for (size_t i = workers_.size() - threads_to_remove; i < workers_.size(); ++i) {
            if (workers_[i].joinable()) {
                workers_[i].join();
            }
        }
        
        // 移除已结束的线程
        workers_.resize(new_size);
    } else {
        // 增加线程数
        size_t threads_to_add = new_size - workers_.size();
        workers_.reserve(new_size);
        
        for (size_t i = 0; i < threads_to_add; ++i) {
            workers_.emplace_back(&ThreadPool::worker, this);
        }
    }
}

void ThreadPool::worker() {
    while (true) {
        std::function<void()> task;
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            condition_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
            
            if (stop_ && tasks_.empty()) {
                return;
            }
            
            task = std::move(tasks_.front());
            tasks_.pop();
            active_tasks_++;
        }
        
        try {
            task();
        } catch (const std::runtime_error& e) {
            // 线程终止信号
            if (std::string(e.what()) == "Thread termination") {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                active_tasks_--;
                finished_condition_.notify_all();
                return;
            }
        } catch (...) {
            // 忽略其他异常，继续执行
        }
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            active_tasks_--;
            if (tasks_.empty() && active_tasks_ == 0) {
                finished_condition_.notify_all();
            }
        }
    }
}

} // namespace multibotnet