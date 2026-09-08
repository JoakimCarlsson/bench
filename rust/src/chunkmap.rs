//! Chunk lookup: an open-addressing hash map from packed chunk coordinates to
//! slots. 200k inserts, then 2M lookups at a 50% hit rate.
use crate::harness::Case;
use crate::hash;

const CAP: u32 = 1 << 19;
const INSERTS: u32 = 200_000;
const LOOKUPS: usize = 2_000_000;
const EMPTY: u64 = u64::MAX;

pub struct ChunkMap {
    keys: Vec<u64>,
    values: Vec<u32>,
    coords: Vec<u64>,
}

fn pack(rng: &mut hash::Rng) -> u64 {
    let r = rng.next();
    let x = r & 0x3ff;
    let y = (r >> 10) & 0x3ff;
    let z = (r >> 20) & 0x3ff;
    x | (y << 21) | (z << 42)
}

impl ChunkMap {
    fn insert(&mut self, key: u64, value: u32) {
        let mut i = (hash::mix64(key) & (CAP - 1) as u64) as u32;
        while self.keys[i as usize] != EMPTY && self.keys[i as usize] != key {
            i = (i + 1) & (CAP - 1);
        }
        self.keys[i as usize] = key;
        self.values[i as usize] = value;
    }

    fn lookup(&self, key: u64) -> Option<u32> {
        let mut i = (hash::mix64(key) & (CAP - 1) as u64) as u32;
        while self.keys[i as usize] != EMPTY {
            if self.keys[i as usize] == key {
                return Some(self.values[i as usize]);
            }
            i = (i + 1) & (CAP - 1);
        }
        None
    }
}

impl Case for ChunkMap {
    const NAME: &'static str = "chunkmap";

    fn init() -> ChunkMap {
        let mut rng = hash::Rng::new(0xc4a4);
        let coords = (0..INSERTS).map(|_| pack(&mut rng)).collect();
        ChunkMap { keys: vec![0; CAP as usize], values: vec![0; CAP as usize], coords }
    }

    fn run(&mut self) -> u64 {
        self.keys.fill(EMPTY);
        for i in 0..INSERTS {
            let c = self.coords[i as usize];
            self.insert(c, i);
        }

        let mut rng = hash::Rng::new(0x100c);
        let mut sum = 0u64;
        let mut hits = 0u64;
        for _ in 0..LOOKUPS {
            let r = rng.next();
            let key = if r & 1 != 0 {
                self.coords[((r >> 1) % INSERTS as u64) as usize]
            } else {
                pack(&mut rng)
            };
            if let Some(v) = self.lookup(key) {
                sum += v as u64;
                hits += 1;
            }
        }
        hash::add(sum, hits)
    }
}
