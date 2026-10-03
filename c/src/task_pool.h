#ifndef BENCH_TASK_POOL_H
#define BENCH_TASK_POOL_H

#include <stddef.h>
#include <stdint.h>

/// The engine's task pool: spinning workers woken by an epoch counter, a
/// fork-join `run` and a block-claiming `parallel_for`.
typedef struct TaskPool TaskPool;

/// Work run on every thread, given the worker index; worker 0 is the caller.
typedef void (*TaskFn)(void* data, uint32_t worker);
/// Work over the index range [begin, end).
typedef void (*TaskRangeFn)(void* data, size_t begin, size_t end);

/// Starts `thread_count - 1` workers; the caller is worker 0.
TaskPool* task_pool_create(uint32_t thread_count);
/// Stops and joins the workers and frees the pool.
void task_pool_destroy(TaskPool* pool);
/// Workers plus the calling thread.
uint32_t task_pool_thread_count(const TaskPool* pool);
/// Runs `fn(data, worker)` on every thread and waits for all of them.
void task_pool_run(TaskPool* pool, TaskFn fn, void* data);
/// Runs `fn(data, begin, end)` over [0, count) in blocks of `grain`, claimed
/// by whichever thread is free.
void task_pool_parallel_for(TaskPool* pool, size_t count, size_t grain, TaskRangeFn fn, void* data);
/// A spin-wait hint to the CPU.
void task_pool_relax(void);

#endif
