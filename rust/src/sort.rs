//! Standard-library sort of 2^19 random u64 keys: `qsort`, `std::sort`,
//! `std.mem.sortUnstable` and `sort_unstable`. The one kernel that measures a
//! library rather than the code written here.
use crate::harness::Case;
use crate::hash;

const KEY_COUNT: usize = 1 << 19;

pub struct Sort {
    keys: Vec<u64>,
    initial: Vec<u64>,
}

impl Case for Sort {
    const NAME: &'static str = "sort";

    fn init() -> Sort {
        let mut rng = hash::Rng::new(0x5027);
        let initial = (0..KEY_COUNT).map(|_| rng.next()).collect();
        Sort { keys: vec![0; KEY_COUNT], initial }
    }

    fn run(&mut self) -> u64 {
        self.keys.copy_from_slice(&self.initial);
        self.keys.sort_unstable();
        let mut h = 0u64;
        let mut i = 0usize;
        while i < KEY_COUNT {
            h = hash::add(h, self.keys[i]);
            i += 977;
        }
        hash::add(h, self.keys[KEY_COUNT - 1])
    }
}
