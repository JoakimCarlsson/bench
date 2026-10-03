//! The safe counterpart of the task pool: a rayon pool of fixed size, chunked
//! loops whose disjointness the borrow checker proves through `chunks_mut`,
//! and an `f32` that threads share through a relaxed atomic.
use rayon::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::sync::atomic::{AtomicU32, Ordering};

/// A rayon pool of `threads` workers; the caller blocks while it runs a job.
pub fn new_pool(threads: usize) -> ThreadPool {
    ThreadPoolBuilder::new().num_threads(threads).build().expect("the thread pool failed to start")
}

/// Runs `f(offset, chunk)` over `items` in chunks of `grain`, where `offset`
/// is the index of the chunk's first element; on the current rayon pool when
/// `parallel`, otherwise in order on the calling thread.
pub fn each_chunk_mut<T: Send>(parallel: bool, items: &mut [T], grain: usize, f: impl Fn(usize, &mut [T]) + Sync + Send) {
    if parallel {
        items.par_chunks_mut(grain).enumerate().for_each(|(block, chunk)| f(block * grain, chunk));
    } else {
        items.chunks_mut(grain).enumerate().for_each(|(block, chunk)| f(block * grain, chunk));
    }
}

/// Runs `f(begin, end)` over the index range `begin..end` in blocks of
/// `grain`; on the current rayon pool when `parallel`, otherwise in order on
/// the calling thread.
pub fn each_range(parallel: bool, begin: usize, end: usize, grain: usize, f: impl Fn(usize, usize) + Sync + Send) {
    let blocks = (end - begin).div_ceil(grain);
    let run_block = |block: usize| {
        let first = begin + block * grain;
        f(first, end.min(first + grain));
    };
    if parallel {
        (0..blocks).into_par_iter().for_each(run_block);
    } else {
        (0..blocks).for_each(run_block);
    }
}

/// An `f32` that several threads may read and write, stored as its bits with
/// relaxed ordering; the pool's fork-join provides the happens-before edges.
#[derive(Default)]
pub struct AtomicF32(AtomicU32);

impl AtomicF32 {
    /// The current value.
    #[inline(always)]
    pub fn load(&self) -> f32 {
        f32::from_bits(self.0.load(Ordering::Relaxed))
    }

    /// Replaces the value.
    #[inline(always)]
    pub fn store(&self, value: f32) {
        self.0.store(value.to_bits(), Ordering::Relaxed);
    }
}
