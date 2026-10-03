#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <vector>

/// The engine's task pool: spinning workers woken by an epoch counter, a
/// fork-join `run` and a block-claiming `parallel_for`.
namespace bench {

class TaskPool final {
public:
    /// Starts `thread_count - 1` workers; the caller is worker 0.
    explicit TaskPool(uint32_t thread_count);
    /// Stops and joins the workers.
    ~TaskPool();

    TaskPool(const TaskPool&) = delete;
    TaskPool& operator=(const TaskPool&) = delete;

    /// Workers plus the calling thread.
    uint32_t thread_count() const { return static_cast<uint32_t>(workers_.size()) + 1u; }

    /// Runs `fn(worker)` on every thread and waits for all of them.
    template <class Fn> void run(Fn&& fn) {
        dispatch([](void* data, uint32_t worker) { (*static_cast<std::remove_reference_t<Fn>*>(data))(worker); }, &fn);
    }

    /// Runs `fn(begin, end)` over [0, count) in blocks of `grain`, claimed
    /// by whichever thread is free.
    template <class Fn> void parallel_for(std::size_t count, std::size_t grain, Fn&& fn) {
        if (count == 0) return;
        const std::size_t blocks = (count + grain - 1) / grain;
        if (blocks <= 1 || workers_.empty()) {
            fn(std::size_t{0}, count);
            return;
        }
        std::atomic<std::size_t> next{0};
        run([&](uint32_t) {
            for (;;) {
                const std::size_t block = next.fetch_add(1, std::memory_order_relaxed);
                if (block >= blocks) return;
                const std::size_t begin = block * grain;
                fn(begin, std::min(count, begin + grain));
            }
        });
    }

    /// A spin-wait hint to the CPU.
    static void relax();

private:
    using Call = void (*)(void*, uint32_t);

    /// Publishes a call to the workers, runs it here as worker 0, and waits.
    void dispatch(Call call, void* data);
    /// Spins, then sleeps, until a new epoch, and runs the published call.
    void worker_loop(uint32_t index);

    std::vector<std::thread> workers_;
    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint32_t> finished_{0};
    std::atomic<bool> stopping_{false};
    Call call_{};
    void* data_{};
};

} // namespace bench
