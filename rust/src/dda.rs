//! Voxel raycast: 100k rays walked cell by cell through a 128^3 occupancy
//! bitset until they hit a solid cell or leave the grid.
use crate::harness::Case;
use crate::hash;

const N: u32 = 128;
const RAYS: usize = 100_000;

pub struct Dda {
    words: Vec<u64>,
}

impl Dda {
    fn solid(&self, x: i32, y: i32, z: i32) -> bool {
        let i = (x as u32 * N + y as u32) * N + z as u32;
        (self.words[(i >> 6) as usize] >> (i & 63)) & 1 != 0
    }
}

impl Case for Dda {
    const NAME: &'static str = "dda";

    fn init() -> Dda {
        let cells = (N * N * N) as usize;
        let mut words = vec![0u64; cells / 64];
        for i in 0..cells {
            if hash::mix64(i as u64) & 7 == 0 {
                words[i >> 6] |= 1u64 << (i & 63);
            }
        }
        Dda { words }
    }

    fn run(&mut self) -> u64 {
        let mut rng = hash::Rng::new(0x1234);
        let mut h = 0u64;
        for _ in 0..RAYS {
            let ox = rng.unit() * N as f32;
            let oy = rng.unit() * N as f32;
            let oz = rng.unit() * N as f32;
            let mut dx = rng.unit() * 2.0 - 1.0;
            let mut dy = rng.unit() * 2.0 - 1.0;
            let mut dz = rng.unit() * 2.0 - 1.0;
            let mut len = (dx * dx + dy * dy + dz * dz).sqrt();
            if len < 1e-3 {
                dx = 1.0;
                dy = 0.0;
                dz = 0.0;
                len = 1.0;
            }
            dx /= len;
            dy /= len;
            dz /= len;
            if dx.abs() < 1e-6 {
                dx = 1e-6;
            }
            if dy.abs() < 1e-6 {
                dy = 1e-6;
            }
            if dz.abs() < 1e-6 {
                dz = 1e-6;
            }

            let mut ix = ox as i32;
            let mut iy = oy as i32;
            let mut iz = oz as i32;
            let sx: i32 = if dx > 0.0 { 1 } else { -1 };
            let sy: i32 = if dy > 0.0 { 1 } else { -1 };
            let sz: i32 = if dz > 0.0 { 1 } else { -1 };
            let invx = 1.0 / dx;
            let invy = 1.0 / dy;
            let invz = 1.0 / dz;
            let tdx = invx.abs();
            let tdy = invy.abs();
            let tdz = invz.abs();
            let mut tx = if dx > 0.0 { ((ix + 1) as f32 - ox) * invx } else { (ix as f32 - ox) * invx };
            let mut ty = if dy > 0.0 { ((iy + 1) as f32 - oy) * invy } else { (iy as f32 - oy) * invy };
            let mut tz = if dz > 0.0 { ((iz + 1) as f32 - oz) * invz } else { (iz as f32 - oz) * invz };

            let mut steps = 0u32;
            loop {
                if self.solid(ix, iy, iz) {
                    let cell = (ix as u64 * N as u64 + iy as u64) * N as u64 + iz as u64;
                    h = hash::add(h, cell | ((steps as u64) << 32));
                    break;
                }
                if tx < ty {
                    if tx < tz {
                        ix += sx;
                        tx += tdx;
                    } else {
                        iz += sz;
                        tz += tdz;
                    }
                } else if ty < tz {
                    iy += sy;
                    ty += tdy;
                } else {
                    iz += sz;
                    tz += tdz;
                }
                steps += 1;
                if ix as u32 >= N || iy as u32 >= N || iz as u32 >= N {
                    h = hash::add(h, 0xffffffff ^ steps as u64);
                    break;
                }
            }
        }
        h
    }
}
