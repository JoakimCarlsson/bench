//! Runs one kernel and prints its line of the output protocol.
use std::time::Instant;

/// One kernel: how to build its state and how to run over it. `run` must be
/// deterministic; the harness rejects a kernel whose checksum changes between
/// repetitions.
pub trait Case {
    const NAME: &'static str;
    fn init() -> Self;
    fn run(&mut self) -> u64;
}

/// Set up, run `warmup + reps` times, and print one line:
/// `name min_ns median_ns checksum_hex`. Returns false on a nondeterministic
/// kernel.
pub fn run_case<C: Case>(reps: usize, warmup: usize) -> bool {
    let mut case = C::init();
    let mut times = vec![0u64; reps];

    let mut checksum = 0u64;
    for i in 0..warmup + reps {
        let t0 = Instant::now();
        let sum = case.run();
        let elapsed = t0.elapsed();
        if i == 0 {
            checksum = sum;
        } else if sum != checksum {
            eprintln!("{}: nondeterministic", C::NAME);
            return false;
        }
        if i >= warmup {
            times[i - warmup] = elapsed.as_nanos() as u64;
        }
    }

    times.sort_unstable();
    println!("{} {} {} {:016x}", C::NAME, times[0], times[reps / 2], checksum);
    true
}
