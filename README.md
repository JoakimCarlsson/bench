# bench

C, C++, Zig and Rust running the same fourteen kernels from a voxel physics
engine. Every kernel produces a checksum, and the runner only reports when all
four languages produce the same bits.

## Results

Intel Core i5-14600K, Windows 11, clang 22.1.2, Zig 0.16.0, rustc 1.98.1. Five
interleaved rounds of five repetitions. Median milliseconds per repetition, and
the median relative to the fastest language for that kernel.

| kernel    |     C |   C++ |   Zig |  Rust |     C |   C++ |   Zig |  Rust |
|-----------|------:|------:|------:|------:|------:|------:|------:|------:|
| dda       |  5.13 |  5.14 |  5.09 |  5.29 | 1.01x | 1.01x | 1.00x | 1.04x |
| flood     | 16.17 | 15.90 | 17.02 | 16.80 | 1.02x | 1.00x | 1.07x | 1.06x |
| surface   |  8.87 |  9.17 |  7.57 | 12.49 | 1.17x | 1.21x | 1.00x | 1.65x |
| mips      |  8.64 |  8.54 | 10.85 | 10.77 | 1.01x | 1.00x | 1.27x | 1.26x |
| mass      | 11.86 |  9.98 | 11.20 | 13.27 | 1.19x | 1.00x | 1.12x | 1.33x |
| sweep     |  7.68 |  7.21 |  7.03 |  6.25 | 1.23x | 1.15x | 1.13x | 1.00x |
| bvh       | 16.93 | 16.79 | 16.62 | 16.46 | 1.03x | 1.02x | 1.01x | 1.00x |
| islands   |  2.07 |  2.03 |  2.14 |  2.22 | 1.02x | 1.00x | 1.05x | 1.09x |
| colour    |  2.03 |  1.98 |  1.88 |  1.70 | 1.19x | 1.16x | 1.10x | 1.00x |
| solve     | 14.16 | 13.93 | 16.99 | 14.25 | 1.02x | 1.00x | 1.22x | 1.02x |
| integrate | 10.33 |  8.73 |  6.92 | 10.12 | 1.49x | 1.26x | 1.00x | 1.46x |
| slotmap   | 24.92 | 23.66 | 24.40 | 24.61 | 1.05x | 1.00x | 1.03x | 1.04x |
| chunkmap  | 26.85 | 27.53 | 28.69 | 26.79 | 1.00x | 1.03x | 1.07x | 1.00x |
| sort      | 47.95 |  9.96 | 38.03 |  5.44 | 8.81x | 1.83x | 6.99x | 1.00x |

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
| slotmap   | 4M insert, remove and lookup operations on a 65536-slot generational slot map           |
| chunkmap  | open-addressing hash map: 200k inserts, 2M lookups at 50% hit rate                        |
| sort      | `qsort`, `std::sort`, `std.mem.sortUnstable` and `sort_unstable` over 2^19 random u64 keys |

All state is allocated before the timed region.

## Rules

- clang, Zig and rustc all lower through LLVM.
- `-O3 -march=native -ffp-contract=off` for C and C++; `ReleaseFast` on the native CPU for Zig; `opt-level = 3`, fat LTO, one codegen unit and `-C target-cpu=native` for Rust.
- Operations are written in the same order in all four sources.
- Rust keeps its bounds checks and its overflow semantics; that is what idiomatic means here.
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
c/src/      one .c and .h per kernel; hash, clock, alloc and harness shared
cpp/src/    one .cpp and .hpp per kernel; hash, clock and harness shared
zig/src/    one .zig per kernel; hash and harness shared; build.zig alongside
rust/src/   one .rs per kernel; hash and harness shared; Cargo.toml alongside
run.py      builds, interleaves, verifies checksums, writes results/
Makefile    the build flags
results/    the last run
```
