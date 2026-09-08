//! Generational slot map: 4M random insert, remove and lookup operations
//! over 65536 slots, with a stale-handle lookup on every hit. Branchy
//! integer code over a free list, the shape of an entity or body registry.
use crate::harness::Case;
use crate::hash;

const CAP: u32 = 65536;
const OPS: usize = 4_000_000;

#[derive(Clone, Copy, Default)]
pub struct Slot {
    generation: u32,
    next: u32,
    value: u64,
}

pub struct SlotMap {
    slots: Vec<Slot>,
    handles: Vec<u64>,
    head: u32,
    count: u32,
}

impl SlotMap {
    fn reset(&mut self) {
        for (i, s) in self.slots.iter_mut().enumerate() {
            *s = Slot { generation: 1, next: i as u32 + 1, value: 0 };
        }
        self.head = 0;
        self.count = 0;
    }

    fn insert(&mut self, value: u64) {
        if self.count >= CAP {
            return;
        }
        let idx = self.head;
        self.head = self.slots[idx as usize].next;
        self.slots[idx as usize].value = value;
        self.handles[self.count as usize] = ((self.slots[idx as usize].generation as u64) << 32) | idx as u64;
        self.count += 1;
    }

    fn remove_at(&mut self, k: u32) {
        let hd = self.handles[k as usize];
        self.count -= 1;
        self.handles[k as usize] = self.handles[self.count as usize];
        let idx = hd as u32;
        let slot = &mut self.slots[idx as usize];
        slot.generation = slot.generation.wrapping_add(1);
        slot.next = self.head;
        self.head = idx;
    }

    fn lookup(&self, hd: u64) -> Option<u64> {
        let idx = hd as u32;
        let slot = &self.slots[idx as usize];
        if slot.generation != (hd >> 32) as u32 {
            return None;
        }
        Some(slot.value)
    }
}

impl Case for SlotMap {
    const NAME: &'static str = "slotmap";

    fn init() -> SlotMap {
        SlotMap {
            slots: vec![Slot::default(); CAP as usize],
            handles: vec![0; CAP as usize],
            head: 0,
            count: 0,
        }
    }

    fn run(&mut self) -> u64 {
        self.reset();
        let mut sum = 0u64;
        let mut misses = 0u64;
        let mut rng = hash::Rng::new(0x510);
        for _ in 0..OPS {
            let r = rng.next();
            match r & 3 {
                0 | 1 => self.insert(r),
                2 => {
                    if self.count > 0 {
                        self.remove_at(((r >> 2) % self.count as u64) as u32);
                    }
                }
                _ => {
                    if self.count > 0 {
                        let hd = self.handles[((r >> 2) % self.count as u64) as usize];
                        if let Some(v) = self.lookup(hd) {
                            sum = sum.wrapping_add(v);
                        }
                        match self.lookup(hd ^ (1u64 << 32)) {
                            Some(v) => sum = sum.wrapping_add(v),
                            None => misses += 1,
                        }
                    }
                }
            }
        }
        hash::add(hash::add(sum, misses), self.count as u64)
    }
}
