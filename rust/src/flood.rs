//! Connected components: 6-connected breadth-first labelling of a 128^3
//! occupancy grid at 30% fill, the pass a collapse step runs to find what
//! still holds together.
use crate::harness::Case;
use crate::hash;

const N: u32 = 128;
const CELLS: u32 = N * N * N;

pub struct Flood {
    occ: Vec<u8>,
    label: Vec<u32>,
    queue: Vec<u32>,
}

impl Case for Flood {
    const NAME: &'static str = "flood";

    fn init() -> Flood {
        let cells = CELLS as usize;
        let occ = (0..cells)
            .map(|i| u8::from(hash::mix64(i as u64 ^ 0x5eed) & 0xff < 77))
            .collect();
        Flood { occ, label: vec![0; cells], queue: vec![0; cells] }
    }

    fn run(&mut self) -> u64 {
        self.label.fill(0);
        let mut h = 0u64;
        let mut comp = 0u32;
        for start in 0..CELLS {
            if self.occ[start as usize] == 0 || self.label[start as usize] != 0 {
                continue;
            }
            comp += 1;
            self.label[start as usize] = comp;
            let mut head = 0u32;
            let mut tail = 0u32;
            let mut size = 0u32;
            self.queue[tail as usize] = start;
            tail += 1;
            while head < tail {
                let c = self.queue[head as usize];
                head += 1;
                size += 1;
                let x = c / (N * N);
                let y = (c / N) % N;
                let z = c % N;
                let mut nb = [0u32; 6];
                let mut count = 0usize;
                if x > 0 {
                    nb[count] = c - N * N;
                    count += 1;
                }
                if x + 1 < N {
                    nb[count] = c + N * N;
                    count += 1;
                }
                if y > 0 {
                    nb[count] = c - N;
                    count += 1;
                }
                if y + 1 < N {
                    nb[count] = c + N;
                    count += 1;
                }
                if z > 0 {
                    nb[count] = c - 1;
                    count += 1;
                }
                if z + 1 < N {
                    nb[count] = c + 1;
                    count += 1;
                }
                for &nbr in &nb[..count] {
                    if self.occ[nbr as usize] != 0 && self.label[nbr as usize] == 0 {
                        self.label[nbr as usize] = comp;
                        self.queue[tail as usize] = nbr;
                        tail += 1;
                    }
                }
            }
            h = hash::add(h, size as u64);
        }
        hash::add(h, comp as u64)
    }
}
