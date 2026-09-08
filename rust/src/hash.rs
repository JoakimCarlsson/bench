//! The hash, the generator and the checksum fold every kernel shares, so a
//! checksum is comparable across the four implementations.

/// splitmix64 finaliser.
pub fn mix64(z0: u64) -> u64 {
    let mut z = z0;
    z ^= z >> 30;
    z = z.wrapping_mul(0xbf58476d1ce4e5b9);
    z ^= z >> 27;
    z = z.wrapping_mul(0x94d049bb133111eb);
    z ^= z >> 31;
    z
}

/// Deterministic generator: a counter through `mix64`.
pub struct Rng {
    s: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng { s: seed }
    }

    pub fn next(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix64(self.s)
    }

    /// Uniform in [0, 1) with 24 bits of precision, so it is exact in f32.
    pub fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 * (1.0 / 16777216.0)
    }
}

/// Fold `v` into the running checksum `h`.
pub fn add(h: u64, v: u64) -> u64 {
    mix64(h ^ v)
}

pub fn f32_bits(f: f32) -> u32 {
    f.to_bits()
}
