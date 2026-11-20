#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace navfast {

class JobCancelled : public std::runtime_error {
public:
    JobCancelled() : std::runtime_error("Cancelled") {}
};

void lowerThreadPriority();
unsigned workerCount(unsigned requested);

/// Executes one indexed batch at a time, with a barrier before returning.
class ThreadPool {
public:
    ThreadPool(unsigned threads, const std::atomic_bool& cancelled);
    ~ThreadPool();
    void run(std::size_t count, const std::function<void(std::size_t)>& body);
    void checkCancelled() const;

private:
    void work();
    void consume();
    void shutdown();

    const std::atomic_bool& cancelled_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable available_;
    std::condition_variable complete_;
    const std::function<void(std::size_t)>* body_ = nullptr;
    std::size_t count_ = 0;
    std::atomic_size_t next_{0};
    std::atomic_bool failed_{false};
    std::exception_ptr error_;
    std::size_t epoch_ = 0;
    std::size_t pending_ = 0;
    bool stopping_ = false;
};

} // namespace navfast
