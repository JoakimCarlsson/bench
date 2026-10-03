//! The engine's simd.hpp, trimmed to what the contact solver uses: four- and
//! eight-lane float vectors as `@Vector`, with selects instead of min and
//! max so every lane rounds the same as scalar code.

pub const lanes = 8;

/// Eight floats processed together.
pub const F8 = @Vector(lanes, f32);
/// Per-lane comparison result for F8.
pub const M8 = @Vector(lanes, bool);
/// Four floats processed together.
pub const F4 = @Vector(4, f32);

/// Every lane set to `s`.
pub inline fn splat(s: f32) F8 {
    return @splat(s);
}

/// `a` where a > b, otherwise `b`.
pub inline fn max(a: F8, b: F8) F8 {
    return @select(f32, a > b, a, b);
}

/// `a` where a < b, otherwise `b`.
pub inline fn min(a: F8, b: F8) F8 {
    return @select(f32, a < b, a, b);
}

/// Lanes set in both masks.
pub inline fn both(a: M8, b: M8) M8 {
    return @select(bool, a, b, @as(M8, @splat(false)));
}

/// Whether any lane of the mask is set.
pub inline fn any(mask: M8) bool {
    return @reduce(.Or, mask);
}

/// Reads one lane at a runtime index.
pub inline fn getLane(v: *const F8, lane: usize) f32 {
    const array: *const [lanes]f32 = @ptrCast(v);
    return array[lane];
}

/// Writes one lane at a runtime index.
pub inline fn setLane(v: *F8, lane: usize, x: f32) void {
    const array: *[lanes]f32 = @ptrCast(v);
    array[lane] = x;
}

/// Transposes a 4x4 block held as four row vectors into four column vectors.
pub inline fn transpose4(r: [4]F4) [4]F4 {
    const t0 = @shuffle(f32, r[0], r[1], [4]i32{ 0, ~@as(i32, 0), 1, ~@as(i32, 1) });
    const t1 = @shuffle(f32, r[2], r[3], [4]i32{ 0, ~@as(i32, 0), 1, ~@as(i32, 1) });
    const t2 = @shuffle(f32, r[0], r[1], [4]i32{ 2, ~@as(i32, 2), 3, ~@as(i32, 3) });
    const t3 = @shuffle(f32, r[2], r[3], [4]i32{ 2, ~@as(i32, 2), 3, ~@as(i32, 3) });
    return .{
        @shuffle(f32, t0, t1, [4]i32{ 0, 1, ~@as(i32, 0), ~@as(i32, 1) }),
        @shuffle(f32, t0, t1, [4]i32{ 2, 3, ~@as(i32, 2), ~@as(i32, 3) }),
        @shuffle(f32, t2, t3, [4]i32{ 0, 1, ~@as(i32, 0), ~@as(i32, 1) }),
        @shuffle(f32, t2, t3, [4]i32{ 2, 3, ~@as(i32, 2), ~@as(i32, 3) }),
    };
}

/// Joins two four-lane vectors, lo in lanes 0 to 3 and hi in lanes 4 to 7.
pub inline fn join(lo: F4, hi: F4) F8 {
    return @shuffle(f32, lo, hi, [8]i32{ 0, 1, 2, 3, ~@as(i32, 0), ~@as(i32, 1), ~@as(i32, 2), ~@as(i32, 3) });
}
