# bench

C, C++, Zig and Rust running the same nineteen kernels from a voxel physics
engine. Every kernel produces a checksum, and the runner only reports when all
four languages produce the same bits.

## Results

Intel Core i5-14600K, Arch Linux (kernel 7.2.6), clang 22.1.8, Zig 0.16.0,
rustc 1.98.1. Five interleaved rounds of five repetitions. Median milliseconds
per repetition, and the median relative to the fastest language for that
kernel.

| kernel    |     C |   C++ |   Zig |  Rust |     C |   C++ |   Zig |  Rust |
|-----------|------:|------:|------:|------:|------:|------:|------:|------:|
| dda       |  5.07 |  5.16 |  5.14 |  5.14 | 1.00x | 1.02x | 1.01x | 1.01x |
| flood     | 15.97 | 15.89 | 16.86 | 16.41 | 1.00x | 1.00x | 1.06x | 1.03x |
| surface   |  8.85 |  8.89 |  7.62 | 11.85 | 1.16x | 1.17x | 1.00x | 1.56x |
| mips      |  8.15 |  8.22 | 10.63 | 10.40 | 1.00x | 1.01x | 1.31x | 1.28x |
| mass      | 11.41 |  9.91 | 11.00 | 13.73 | 1.15x | 1.00x | 1.11x | 1.39x |
| sweep     |  6.99 |  7.37 |  7.97 |  6.89 | 1.02x | 1.07x | 1.16x | 1.00x |
| bvh       | 17.01 | 16.54 | 16.56 | 16.07 | 1.06x | 1.03x | 1.03x | 1.00x |
| islands   |  2.03 |  2.06 |  2.02 |  2.29 | 1.00x | 1.02x | 1.00x | 1.14x |
| colour    |  2.00 |  1.98 |  1.71 |  1.67 | 1.20x | 1.18x | 1.02x | 1.00x |
| solve     | 13.61 | 13.46 | 16.64 | 13.41 | 1.01x | 1.00x | 1.24x | 1.00x |
| integrate | 10.11 |  8.59 |  7.18 | 10.07 | 1.41x | 1.20x | 1.00x | 1.40x |
| transform | 11.94 | 11.93 |  9.76 | 11.55 | 1.22x | 1.22x | 1.00x | 1.18x |
| unproject |  9.75 |  9.50 |  6.99 |  7.10 | 1.39x | 1.36x | 1.00x | 1.02x |
| decompose |  9.37 |  9.16 |  7.10 |  7.18 | 1.32x | 1.29x | 1.00x | 1.01x |
| raycast   | 12.91 | 13.02 | 11.78 | 13.74 | 1.10x | 1.11x | 1.00x | 1.17x |
| boxbox    | 11.82 | 10.80 | 10.90 | 11.53 | 1.09x | 1.00x | 1.01x | 1.07x |
| slotmap   | 24.86 | 23.66 | 24.65 | 23.69 | 1.05x | 1.00x | 1.04x | 1.00x |
| chunkmap  | 26.01 | 26.24 | 26.68 | 26.23 | 1.00x | 1.01x | 1.03x | 1.01x |
| sort      | 41.06 | 22.23 | 38.24 |  5.26 | 7.81x | 4.23x | 7.27x | 1.00x |

Raw samples and best-of-run numbers: [results/latest.md](results/latest.md).

## Kernels

| kernel    | work                                                                                    |
|-----------|-----------------------------------------------------------------------------------------|
| dda       | 100k rays walked cell by cell through a 128^3 occupancy bitset                          |
| flood     | 6-connected breadth-first labelling of a 128^3 grid at 30% fill                          |
| surface   | exposed-face count of every solid voxel in a 128^3 grid at 75% fill                      |
| mips      | rebuild row bitsets and 4^3 mips for 512 chunks of 32^3, then popcount                    |
| mass      | mass, centre of mass and inertia tensor of 128 bodies of 32^3 voxels                     |
| sweep     | sort-and-sweep broadphase over 16384 boxes, using the standard library sort              |
| bvh       | build a BVH over 16384 boxes, then trace 20k rays through it                              |
| islands   | union-find over 524288 contacts between 65536 bodies, label by lowest slot               |
| colour    | greedy graph colouring of 524288 contacts so each colour solves in parallel              |
| solve     | 8 frames of substepped sequential impulses, 4096 bodies, 16384 contacts                  |
| integrate | 32 steps of position, velocity and quaternion integration over 65536 bodies              |
| transform | 8 frames of scene-graph propagation over 65536 nodes: rotate, compose, model-view-projection |
| unproject | look-at, perspective, 4x4 cofactor inverse and four unprojected points for 131072 cameras  |
| decompose | basis from rotation and scale, then scale, rotation, inverse and rotate back, 262144 times  |
| raycast   | closest hit of 1024 rays against 1024 oriented boxes with the engine's slab test          |
| boxbox    | 8 frames of box-box SAT, face clipping, reduction and warm starting over 8192 pairs        |
| slotmap   | 4M insert, remove and lookup operations on a 65536-slot generational slot map           |
| chunkmap  | open-addressing hash map: 200k inserts, 2M lookups at 50% hit rate                        |
| sort      | `qsort`, `std::sort`, `std.mem.sortUnstable` and `sort_unstable` over 2^19 random u64 keys |

All state is allocated before the timed region. The last five port the
engine's `math.cpp`, `ray_cast.cpp` and `box_collision.cpp` into a shared
`vecmath` module per language.

## Rules

- clang, Zig and rustc all lower through LLVM.
- `-O3 -march=native -ffp-contract=off` for C and C++; `ReleaseFast` on the native CPU for Zig; `opt-level = 3`, fat LTO, one codegen unit and `-C target-cpu=native` for Rust.
- Operations are written in the same order in all four sources.
- Rust keeps its bounds checks and its overflow semantics; that is what idiomatic means here.
- Only `+ - * /` and `sqrt` touch floats: `sin`, `acos` and `tan` differ in the last bit between glibc and Zig's own libm, so `slerp` is left out and `perspective` takes the tangent precomputed.
- Floats are folded into the checksum by bit pattern. A mismatch across languages or repetitions aborts the run.
- Each round rotates which executable runs first.
- Idiomatic code in each language, no hand tuning.

## Running

Requires clang and clang++, Zig 0.16, a stable Rust toolchain, GNU make and
Python 3.10+.

```
make                # build out/bench_{c,cpp,zig,rust}
make run            # build, then python run.py
python run.py --rounds 10 --reps 5
python run.py --langs c zig --no-build
```

`make ZIG=/path/to/zig CC=clang-20 CXX=clang++-20 CARGO=cargo` overrides the
toolchains.

Each executable takes `[reps] [warmup]` and prints one line per kernel:

```
name min_ns median_ns checksum_hex
```

## Layout

```
c/src/      one .c and .h per kernel; hash, clock, alloc, vecmath and harness shared
cpp/src/    one .cpp and .hpp per kernel; hash, clock, vecmath and harness shared
zig/src/    one .zig per kernel; hash, vecmath and harness shared; build.zig alongside
rust/src/   one .rs per kernel; hash, vecmath and harness shared; Cargo.toml alongside
run.py      builds, interleaves, verifies checksums, writes results/
Makefile    the build flags
results/    the last run
```
