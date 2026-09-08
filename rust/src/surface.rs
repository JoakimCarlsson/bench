//! Surface extraction: count the exposed faces of every solid voxel in a
//! 128^3 grid at 75% fill, the pass that finds what a raymarcher can see and
//! what a contact can touch.
use crate::harness::Case;
use crate::hash;

const N: u32 = 128;
const CELLS: u32 = N * N * N;

pub struct Surface {
    occ: Vec<u8>,
}

fn empty_at(occ: &[u8], x: i32, y: i32, z: i32) -> u32 {
    if x as u32 >= N || y as u32 >= N || z as u32 >= N {
        return 1;
    }
    let i = (x as u32 * N + y as u32) * N + z as u32;
    u32::from(occ[i as usize] == 0)
}

impl Case for Surface {
    const NAME: &'static str = "surface";

    fn init() -> Surface {
        let occ = (0..CELLS as usize)
            .map(|i| u8::from(hash::mix64(i as u64 ^ 0xface) & 3 != 0))
            .collect();
        Surface { occ }
    }

    fn run(&mut self) -> u64 {
        let mut h = 0u64;
        let mut faces_total = 0u64;
        for x in 0..N as i32 {
            for y in 0..N as i32 {
                for z in 0..N as i32 {
                    let cell = (x as u32 * N + y as u32) * N + z as u32;
                    if self.occ[cell as usize] == 0 {
                        continue;
                    }
                    let faces = empty_at(&self.occ, x - 1, y, z)
                        + empty_at(&self.occ, x + 1, y, z)
                        + empty_at(&self.occ, x, y - 1, z)
                        + empty_at(&self.occ, x, y + 1, z)
                        + empty_at(&self.occ, x, y, z - 1)
                        + empty_at(&self.occ, x, y, z + 1);
                    if faces == 0 {
                        continue;
                    }
                    h = hash::add(h, ((cell as u64) << 3) | faces as u64);
                    faces_total += faces as u64;
                }
            }
        }
        hash::add(h, faces_total)
    }
}
