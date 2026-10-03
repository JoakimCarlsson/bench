#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include "task_pool.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "alloc.h"

#if defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || defined(_M_IX86)
#define TASK_POOL_X86 1
#include <immintrin.h>
#endif

#ifdef _WIN32
#include <windows.h>
typedef HANDLE Thread;
#else
#include <pthread.h>
#include <sched.h>
typedef pthread_t Thread;
#endif

enum { SPIN_ITERATIONS = 4000 };

/// A worker thread's pool and index.
typedef struct {
    TaskPool* pool;
    uint32_t index;
} WorkerStart;

struct TaskPool {
    uint32_t worker_count;
    Thread* threads;
    WorkerStart* starts;
    atomic_uint epoch;
    atomic_uint finished;
    atomic_bool stopping;
    TaskFn call;
    void* data;
};

/// Shared state of one `parallel_for`.
typedef struct {
    atomic_size_t next;
    size_t blocks;
    size_t count;
    size_t grain;
    TaskRangeFn fn;
    void* data;
} ParallelFor;

void task_pool_relax(void) {
#if defined(TASK_POOL_X86)
    _mm_pause();
#elif defined(__aarch64__)
    __asm__ __volatile__("yield");
#endif
}

/// Gives the rest of the time slice to another thread.
static void yield_thread(void) {
#ifdef _WIN32
    SwitchToThread();
#else
    sched_yield();
#endif
}

/// Spins, then yields, until a new epoch, and runs the published call.
static void worker_loop(TaskPool* pool, uint32_t index) {
    uint32_t seen = 0;
    for (;;) {
        int spins = 0;
        while (atomic_load_explicit(&pool->epoch, memory_order_acquire) == seen) {
            if (++spins < SPIN_ITERATIONS) {
                task_pool_relax();
            } else {
                yield_thread();
            }
        }
        seen = atomic_load_explicit(&pool->epoch, memory_order_acquire);
        if (atomic_load_explicit(&pool->stopping, memory_order_acquire)) return;
        pool->call(pool->data, index);
        atomic_fetch_add_explicit(&pool->finished, 1, memory_order_release);
    }
}

#ifdef _WIN32
/// Thread entry point.
static DWORD WINAPI worker_main(LPVOID arg) {
    WorkerStart* start = arg;
    worker_loop(start->pool, start->index);
    return 0;
}
#else
/// Thread entry point.
static void* worker_main(void* arg) {
    WorkerStart* start = arg;
    worker_loop(start->pool, start->index);
    return NULL;
}
#endif

/// Exits the process after reporting that a thread could not start.
static void thread_failed(void) {
    fprintf(stderr, "task pool: could not start a thread\n");
    exit(5);
}

TaskPool* task_pool_create(uint32_t thread_count) {
    TaskPool* pool = xalloc(sizeof(TaskPool));
    pool->worker_count = thread_count > 1 ? thread_count - 1 : 0;
    pool->threads = xalloc((pool->worker_count + 1) * sizeof(Thread));
    pool->starts = xalloc((pool->worker_count + 1) * sizeof(WorkerStart));
    atomic_init(&pool->epoch, 0);
    atomic_init(&pool->finished, 0);
    atomic_init(&pool->stopping, false);
    for (uint32_t i = 0; i < pool->worker_count; ++i) {
        pool->starts[i] = (WorkerStart){ pool, i + 1 };
#ifdef _WIN32
        pool->threads[i] = CreateThread(NULL, 0, worker_main, &pool->starts[i], 0, NULL);
        if (pool->threads[i] == NULL) thread_failed();
#else
        if (pthread_create(&pool->threads[i], NULL, worker_main, &pool->starts[i]) != 0) thread_failed();
#endif
    }
    return pool;
}

void task_pool_destroy(TaskPool* pool) {
    atomic_store_explicit(&pool->stopping, true, memory_order_release);
    atomic_fetch_add_explicit(&pool->epoch, 1, memory_order_release);
    for (uint32_t i = 0; i < pool->worker_count; ++i) {
#ifdef _WIN32
        WaitForSingleObject(pool->threads[i], INFINITE);
        CloseHandle(pool->threads[i]);
#else
        pthread_join(pool->threads[i], NULL);
#endif
    }
    free(pool->threads);
    free(pool->starts);
    free(pool);
}

uint32_t task_pool_thread_count(const TaskPool* pool) { return pool->worker_count + 1u; }

void task_pool_run(TaskPool* pool, TaskFn fn, void* data) {
    if (pool->worker_count == 0) {
        fn(data, 0);
        return;
    }
    pool->call = fn;
    pool->data = data;
    atomic_store_explicit(&pool->finished, 0, memory_order_relaxed);
    atomic_fetch_add_explicit(&pool->epoch, 1, memory_order_release);
    fn(data, 0);
    while (atomic_load_explicit(&pool->finished, memory_order_acquire) != pool->worker_count) task_pool_relax();
}

/// Claims blocks of a `parallel_for` until none are left.
static void parallel_for_worker(void* data, uint32_t worker) {
    ParallelFor* job = data;
    (void)worker;
    for (;;) {
        size_t block = atomic_fetch_add_explicit(&job->next, 1, memory_order_relaxed);
        if (block >= job->blocks) return;
        size_t begin = block * job->grain;
        size_t end = begin + job->grain < job->count ? begin + job->grain : job->count;
        job->fn(job->data, begin, end);
    }
}

void task_pool_parallel_for(TaskPool* pool, size_t count, size_t grain, TaskRangeFn fn, void* data) {
    if (count == 0) return;
    size_t blocks = (count + grain - 1) / grain;
    if (blocks <= 1 || pool->worker_count == 0) {
        fn(data, 0, count);
        return;
    }
    ParallelFor job = { .blocks = blocks, .count = count, .grain = grain, .fn = fn, .data = data };
    atomic_init(&job.next, 0);
    task_pool_run(pool, parallel_for_worker, &job);
}
