//! The engine's task pool: spinning workers woken by an epoch counter, a
//! fork-join `run` and a block-claiming `parallel_for`. Also the shared view
//! the engine's parallel sections write through when the colouring, not the
//! borrow checker, keeps their writes apart.
use std::cell::UnsafeCell;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::thread::{self, JoinHandle};

const SPIN_ITERATIONS: u32 = 4000;

/// A published call: a type-erased closure and the function that runs it.
#[derive(Clone, Copy)]
struct Job {
    call: unsafe fn(*const (), u32),
    data: *const (),
}

/// State the dispatching thread and the workers share.
struct Shared {
    epoch: AtomicU32,
    finished: AtomicU32,
    stopping: AtomicBool,
    job: UnsafeCell<Option<Job>>,
}

// SAFETY: `job` is written only by the dispatching thread before it bumps
// `epoch` with release ordering, and read by workers only after they observe
// that epoch with acquire ordering; the dispatcher waits for every worker to
// report `finished` before it can write `job` again. The closure behind
// `data` is `Sync`, checked by `TaskPool::run`.
unsafe impl Sync for Shared {}
// SAFETY: as above; the raw pointer in `job` is only dereferenced under that
// protocol.
unsafe impl Send for Shared {}

pub struct TaskPool {
    shared: Arc<Shared>,
    workers: Vec<JoinHandle<()>>,
}

/// Runs the closure of type `F` behind `data` as `worker`.
///
/// # Safety
///
/// `data` must point to a live `F`.
unsafe fn call_closure<F: Fn(u32) + Sync>(data: *const (), worker: u32) {
    // SAFETY: the caller guarantees `data` points to a live `F`.
    let f = unsafe { &*(data as *const F) };
    f(worker);
}

/// A spin-wait hint to the CPU.
pub fn relax() {
    std::hint::spin_loop();
}

impl TaskPool {
    /// Starts `thread_count - 1` workers; the caller is worker 0.
    pub fn new(thread_count: u32) -> TaskPool {
        let shared = Arc::new(Shared { epoch: AtomicU32::new(0), finished: AtomicU32::new(0), stopping: AtomicBool::new(false), job: UnsafeCell::new(None) });
        let workers = (1..thread_count)
            .map(|index| {
                let shared = Arc::clone(&shared);
                thread::spawn(move || worker_loop(&shared, index))
            })
            .collect();
        TaskPool { shared, workers }
    }

    /// Workers plus the calling thread.
    pub fn thread_count(&self) -> u32 {
        self.workers.len() as u32 + 1
    }

    /// Runs `f(worker)` on every thread and waits for all of them.
    pub fn run<F: Fn(u32) + Sync>(&self, f: F) {
        if self.workers.is_empty() {
            f(0);
            return;
        }
        let job = Job { call: call_closure::<F>, data: &f as *const F as *const () };
        // SAFETY: no worker reads `job` until the epoch below changes, and the
        // previous dispatch waited for every worker to finish with it.
        unsafe { *self.shared.job.get() = Some(job) };
        self.shared.finished.store(0, Ordering::Relaxed);
        self.shared.epoch.fetch_add(1, Ordering::Release);
        for worker in &self.workers {
            worker.thread().unpark();
        }
        f(0);
        let expected = self.workers.len() as u32;
        while self.shared.finished.load(Ordering::Acquire) != expected {
            relax();
        }
    }

    /// Runs `f(begin, end)` over [0, count) in blocks of `grain`, claimed by
    /// whichever thread is free.
    pub fn parallel_for<F: Fn(usize, usize) + Sync>(&self, count: usize, grain: usize, f: F) {
        if count == 0 {
            return;
        }
        let blocks = count.div_ceil(grain);
        if blocks <= 1 || self.workers.is_empty() {
            f(0, count);
            return;
        }
        let next = AtomicUsize::new(0);
        self.run(|_| {
            loop {
                let block = next.fetch_add(1, Ordering::Relaxed);
                if block >= blocks {
                    return;
                }
                let begin = block * grain;
                f(begin, count.min(begin + grain));
            }
        });
    }
}

/// Spins, then parks, until a new epoch, and runs the published call.
fn worker_loop(shared: &Shared, index: u32) {
    let mut seen = 0u32;
    loop {
        let mut spins = 0u32;
        while shared.epoch.load(Ordering::Acquire) == seen {
            spins += 1;
            if spins < SPIN_ITERATIONS {
                relax();
            } else {
                thread::park();
            }
        }
        seen = shared.epoch.load(Ordering::Acquire);
        if shared.stopping.load(Ordering::Acquire) {
            return;
        }
        // SAFETY: the epoch moved, so the dispatcher published `job` and keeps
        // the closure alive until this worker reports `finished`.
        let job = unsafe { *shared.job.get() }.expect("a new epoch without a job");
        // SAFETY: `job.data` points to the closure `job.call` was built for,
        // alive until `finished` reaches the worker count.
        unsafe { (job.call)(job.data, index) };
        shared.finished.fetch_add(1, Ordering::Release);
    }
}

impl Drop for TaskPool {
    /// Stops and joins the workers.
    fn drop(&mut self) {
        self.shared.stopping.store(true, Ordering::Release);
        self.shared.epoch.fetch_add(1, Ordering::Release);
        for worker in &self.workers {
            worker.thread().unpark();
        }
        for worker in self.workers.drain(..) {
            worker.join().expect("a task pool worker panicked");
        }
    }
}

/// A slice that parallel blocks write at indices they own, when the
/// ownership comes from the engine's data (block ranges, graph colours,
/// contact ids) rather than from a split the borrow checker can see.
pub struct Disjoint<'a, T> {
    ptr: *mut T,
    len: usize,
    marker: PhantomData<&'a mut [T]>,
}

// SAFETY: a `Disjoint` only hands out element references through its unsafe
// accessors, whose callers promise no two threads touch one element at once
// with one of them writing.
unsafe impl<T: Send + Sync> Sync for Disjoint<'_, T> {}
// SAFETY: as above.
unsafe impl<T: Send + Sync> Send for Disjoint<'_, T> {}

impl<'a, T> Disjoint<'a, T> {
    /// Takes the slice for the lifetime of the view.
    pub fn new(slice: &'a mut [T]) -> Disjoint<'a, T> {
        Disjoint { ptr: slice.as_mut_ptr(), len: slice.len(), marker: PhantomData }
    }

    /// Reads one element.
    ///
    /// # Safety
    ///
    /// No other thread may write element `index` while the reference lives.
    pub unsafe fn get(&self, index: usize) -> &T {
        assert!(index < self.len, "index {index} out of {}", self.len);
        // SAFETY: in bounds, and the caller rules out a concurrent writer.
        unsafe { &*self.ptr.add(index) }
    }

    /// Writes one element.
    ///
    /// # Safety
    ///
    /// No other thread may access element `index` while the reference lives.
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn get_mut(&self, index: usize) -> &mut T {
        assert!(index < self.len, "index {index} out of {}", self.len);
        // SAFETY: in bounds, and the caller rules out any concurrent access.
        unsafe { &mut *self.ptr.add(index) }
    }

    /// Writes the elements in [begin, end).
    ///
    /// # Safety
    ///
    /// No other thread may access those elements while the slice lives.
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn range_mut(&self, begin: usize, end: usize) -> &mut [T] {
        assert!(begin <= end && end <= self.len, "range {begin}..{end} out of {}", self.len);
        // SAFETY: in bounds, and the caller rules out any concurrent access.
        unsafe { std::slice::from_raw_parts_mut(self.ptr.add(begin), end - begin) }
    }
}
