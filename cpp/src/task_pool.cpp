#include "task_pool.hpp"

namespace bench {

namespace {

constexpr int spin_iterations = 4000;

} // namespace

void TaskPool::relax() {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#endif
}

TaskPool::TaskPool(uint32_t thread_count) {
    for (uint32_t i = 1; i < thread_count; ++i) workers_.emplace_back([this, i] { worker_loop(i); });
}

TaskPool::~TaskPool() {
    stopping_.store(true, std::memory_order_release);
    epoch_.fetch_add(1, std::memory_order_release);
    epoch_.notify_all();
    for (std::thread& worker : workers_) worker.join();
}

void TaskPool::worker_loop(uint32_t index) {
    uint32_t seen = 0;
    for (;;) {
        int spins = 0;
        while (epoch_.load(std::memory_order_acquire) == seen) {
            if (++spins < spin_iterations) {
                relax();
            } else {
                epoch_.wait(seen, std::memory_order_acquire);
            }
        }
        seen = epoch_.load(std::memory_order_acquire);
        if (stopping_.load(std::memory_order_acquire)) return;
        call_(data_, index);
        finished_.fetch_add(1, std::memory_order_release);
    }
}

void TaskPool::dispatch(Call call, void* data) {
    if (workers_.empty()) {
        call(data, 0);
        return;
    }
    call_ = call;
    data_ = data;
    finished_.store(0, std::memory_order_relaxed);
    epoch_.fetch_add(1, std::memory_order_release);
    epoch_.notify_all();
    call(data, 0);
    const auto expected = static_cast<uint32_t>(workers_.size());
    while (finished_.load(std::memory_order_acquire) != expected) relax();
}

} // namespace bench
