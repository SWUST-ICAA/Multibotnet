#ifndef MULTIBOTNET_UTILS_THREAD_POOL_HPP
#define MULTIBOTNET_UTILS_THREAD_POOL_HPP

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>
#include <memory>

namespace multibotnet {

/**
 * @brief 高性能线程池实现
 */
class ThreadPool {
public:
    /**
     * @brief 构造函数
     * @param num_threads 线程数量，默认为硬件并发数
     */
    explicit ThreadPool(size_t num_threads = std::thread::hardware_concurrency());
    
    ~ThreadPool();
    
    /**
     * @brief 提交任务到线程池
     * @param f 任务函数
     * @param args 函数参数
     * @return 任务的future对象
     */
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    
    /**
     * @brief 获取线程池大小
     * @return 线程数量
     */
    size_t size() const { return workers_.size(); }
    
    /**
     * @brief 获取待处理任务数
     * @return 任务队列大小
     */
    size_t pending() const;
    
    /**
     * @brief 等待所有任务完成
     */
    void wait();
    
    /**
     * @brief 停止线程池
     */
    void stop();
    
    /**
     * @brief 调整线程池大小
     * @param new_size 新的线程数量
     */
    void resize(size_t new_size);
    
private:
    // 工作线程
    std::vector<std::thread> workers_;
    
    // 任务队列
    std::queue<std::function<void()>> tasks_;
    
    // 同步
    mutable std::mutex queue_mutex_;
    std::condition_variable condition_;
    std::condition_variable finished_condition_;
    
    // 状态
    std::atomic<bool> stop_;
    std::atomic<size_t> active_tasks_;
    
    // 工作线程函数
    void worker();
};

// 模板实现
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
        
    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        // 不允许在停止后加入新任务
        if(stop_) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }

        tasks_.emplace([task](){ (*task)(); });
    }
    condition_.notify_one();
    return res;
}

/**
 * @brief 任务优先级队列
 */
template<typename T>
class PriorityQueue {
public:
    struct Item {
        int priority;
        T data;
        
        bool operator<(const Item& other) const {
            return priority < other.priority; // 高优先级在前
        }
    };
    
    void push(const T& data, int priority = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push({priority, data});
        cv_.notify_one();
    }
    
    bool pop(T& data, int timeout_ms = -1) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (timeout_ms < 0) {
            cv_.wait(lock, [this] { return !queue_.empty() || stopped_; });
        } else {
            if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                            [this] { return !queue_.empty() || stopped_; })) {
                return false;
            }
        }
        
        if (stopped_ && queue_.empty()) {
            return false;
        }
        
        data = queue_.top().data;
        queue_.pop();
        return true;
    }
    
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        cv_.notify_all();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::priority_queue<Item> queue_;
    std::atomic<bool> stopped_{false};
};

} // namespace multibotnet

#endif // MULTIBOTNET_UTILS_THREAD_POOL_HPP