//! Chunk occupancy rebuild: for 512 chunks of 32^3 material bytes, rebuild
//! the per-row occupancy bitset and the 4^3-block mip, then popcount both.
use crate::harness::Case;
use crate::hash;

const CHUNKS: usize = 512;
const DIM: usize = 32;
const VOXELS: usize = DIM * DIM * DIM;
const ROW_COUNT: usize = DIM * DIM;
const MIP_DIM: usize = DIM / 4;
const MIP_WORDS: usize = MIP_DIM * MIP_DIM * MIP_DIM / 64;

pub struct Mips {
    mat: Vec<u8>,
    rows: Vec<u32>,
    mip: Vec<u64>,
}

impl Case for Mips {
    const NAME: &'static str = "mips";

    fn init() -> Mips {
        let mat = (0..CHUNKS * VOXELS)
            .map(|i| u8::from(hash::mix64(i as u64 ^ 0x3ea) & 3 == 0))
            .collect();
        Mips { mat, rows: vec![0; CHUNKS * ROW_COUNT], mip: vec![0; CHUNKS * MIP_WORDS] }
    }

    fn run(&mut self) -> u64 {
        let mut h = 0u64;
        for c in 0..CHUNKS {
            let mat = &self.mat[c * VOXELS..][..VOXELS];
            let row_words = &mut self.rows[c * ROW_COUNT..][..ROW_COUNT];
            let mip = &mut self.mip[c * MIP_WORDS..][..MIP_WORDS];
            mip.fill(0);
            for z in 0..DIM {
                for y in 0..DIM {
                    let row = &mat[(z * DIM + y) * DIM..][..DIM];
                    let mut word = 0u32;
                    for (x, &v) in row.iter().enumerate() {
                        word |= u32::from(v != 0) << x;
                    }
                    row_words[z * DIM + y] = word;
                    if word == 0 {
                        continue;
                    }
                    for bx in 0..MIP_DIM {
                        if (word >> (bx * 4)) & 0xf == 0 {
                            continue;
                        }
                        let bit = ((z / 4) * MIP_DIM + (y / 4)) * MIP_DIM + bx;
                        mip[bit >> 6] |= 1u64 << (bit & 63);
                    }
                }
            }
            let solid: u32 = row_words.iter().map(|w| w.count_ones()).sum();
            let blocks: u32 = mip.iter().map(|w| w.count_ones()).sum();
            h = hash::add(h, ((solid as u64) << 32) | blocks as u64);
        }
        h
    }
}
