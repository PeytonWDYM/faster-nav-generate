#include "job/thread_pool.h"

#include <algorithm>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace navfast {

void lowerThreadPriority() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#else
    nice(5);
#endif
}

unsigned workerCount(unsigned requested) {
    return requested == 0 ? std::max(1u, std::thread::hardware_concurrency()) : requested;
}

ThreadPool::ThreadPool(unsigned threads, const std::atomic_bool& cancelled)
    : cancelled_(cancelled) {
    // The job thread participates as one worker.
    workers_.reserve(threads - 1);
    try {
        for (unsigned i = 1; i < threads; ++i)
            workers_.emplace_back([this] { work(); });
    } catch (...) {
        shutdown();
        throw;
    }
}

ThreadPool::~ThreadPool() { shutdown(); }

void ThreadPool::shutdown() {
    {
        std::lock_guard lock(mutex_);
        stopping_ = true;
    }
    available_.notify_all();
    for (auto& thread : workers_) thread.join();
}

void ThreadPool::checkCancelled() const {
    if (cancelled_.load(std::memory_order_relaxed)) throw JobCancelled{};
}

void ThreadPool::run(std::size_t count, const std::function<void(std::size_t)>& body) {
    checkCancelled();
    if (workers_.empty() || count < 8) {
        for (std::size_t i = 0; i < count; ++i) {
            checkCancelled();
            body(i);
        }
        return;
    }

    {
        std::lock_guard lock(mutex_);
        body_ = &body;
        count_ = count;
        next_ = 0;
        failed_ = false;
        error_ = nullptr;
        pending_ = workers_.size();
        ++epoch_;
    }
    available_.notify_all();
    consume();
    std::unique_lock lock(mutex_);
    complete_.wait(lock, [this] { return pending_ == 0; });
    if (error_) std::rethrow_exception(error_);
    checkCancelled();
}

void ThreadPool::consume() {
    try {
        constexpr std::size_t batch = 4;
        while (!failed_.load(std::memory_order_relaxed)) {
            checkCancelled();
            const auto start = next_.fetch_add(batch, std::memory_order_relaxed);
            if (start >= count_) return;
            for (auto i = start; i < std::min(start + batch, count_); ++i) (*body_)(i);
        }
    } catch (...) {
        std::lock_guard lock(mutex_);
        if (!error_) error_ = std::current_exception();
        failed_.store(true, std::memory_order_relaxed);
    }
}

void ThreadPool::work() {
#ifdef _WIN32
    lowerThreadPriority();
#endif
    std::size_t seen = 0;
    std::unique_lock lock(mutex_);
    for (;;) {
        available_.wait(lock, [&] { return stopping_ || epoch_ != seen; });
        if (stopping_) return;
        seen = epoch_;
        lock.unlock();
        consume();
        lock.lock();
        if (--pending_ == 0) complete_.notify_one();
    }
}

} // namespace navfast
