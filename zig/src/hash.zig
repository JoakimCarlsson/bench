//! The hash, the generator and the checksum fold every kernel shares, so a
//! checksum is comparable across the three implementations.

/// splitmix64 finaliser.
pub fn mix64(z0: u64) u64 {
    var z = z0;
    z ^= z >> 30;
    z *%= 0xbf58476d1ce4e5b9;
    z ^= z >> 27;
    z *%= 0x94d049bb133111eb;
    z ^= z >> 31;
    return z;
}

/// Deterministic generator: a counter through `mix64`.
pub const Rng = struct {
    s: u64,

    pub fn next(self: *Rng) u64 {
        self.s +%= 0x9e3779b97f4a7c15;
        return mix64(self.s);
    }

    /// Uniform in [0, 1) with 24 bits of precision, so it is exact in f32.
    pub fn unit(self: *Rng) f32 {
        return @as(f32, @floatFromInt(self.next() >> 40)) * (1.0 / 16777216.0);
    }
};

/// Fold `v` into the running checksum `h`.
pub fn add(h: u64, v: u64) u64 {
    return mix64(h ^ v);
}

pub fn f32Bits(f: f32) u32 {
    return @bitCast(f);
}
