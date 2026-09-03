# bench

C, C++ and Zig running the same nine kernels from a voxel physics engine.
Every kernel produces a checksum, and the runner only reports when all three
languages produce the same bits.

## Results

Intel Core i5-14600K, Windows 11, clang 22.1.2, Zig 0.16.0. Five interleaved
rounds of five repetitions. Median milliseconds per repetition, and the
median relative to the fastest language for that kernel.

| kernel    | C     | C++   | Zig   | C     | C++   | Zig   |
|-----------|------:|------:|------:|------:|------:|------:|
| dda       | 5.27  | 5.22  | 5.19  | 1.01x | 1.01x | 1.00x |
| flood     | 17.66 | 16.82 | 17.69 | 1.05x | 1.00x | 1.05x |
| surface   | 9.39  | 9.37  | 7.90  | 1.19x | 1.19x | 1.00x |
| sweep     | 7.84  | 7.42  | 8.74  | 1.06x | 1.00x | 1.18x |
| solve     | 14.21 | 14.32 | 17.65 | 1.00x | 1.01x | 1.24x |
| integrate | 10.55 | 9.07  | 7.14  | 1.48x | 1.27x | 1.00x |
| slotmap   | 25.97 | 24.52 | 25.30 | 1.06x | 1.00x | 1.03x |
| chunkmap  | 28.85 | 28.63 | 29.85 | 1.01x | 1.00x | 1.04x |
| sort      | 49.01 | 9.93  | 35.91 | 4.94x | 1.00x | 3.62x |

Raw samples and best-of-run numbers: [results/latest.md](results/latest.md).

## Kernels

| kernel    | work                                                                                    |
|-----------|-----------------------------------------------------------------------------------------|
| dda       | 100k rays walked cell by cell through a 128^3 occupancy bitset                          |
| flood     | 6-connected breadth-first labelling of a 128^3 grid at 30% fill                          |
| surface   | exposed-face count of every solid voxel in a 128^3 grid at 75% fill                      |
| sweep     | sort-and-sweep broadphase over 16384 boxes, using the standard library sort              |
| solve     | 8 frames of substepped sequential impulses, 4096 bodies, 16384 contacts                  |
| integrate | 32 steps of position, velocity and quaternion integration over 65536 bodies              |
| slotmap   | 4M insert, remove and lookup operations on a 65536-slot generational slot map           |
| chunkmap  | open-addressing hash map: 200k inserts, 2M lookups at 50% hit rate                        |
| sort      | `qsort`, `std::sort` and `std.mem.sortUnstable` over 2^19 random u64 keys                |

All state is allocated before the timed region.

## Rules

- clang and Zig both lower through LLVM.
- `-O3 -march=native -ffp-contract=off` for C and C++; `ReleaseFast` on the native CPU for Zig.
- Operations are written in the same order in all three sources.
- Floats are folded into the checksum by bit pattern. A mismatch across languages or repetitions aborts the run.
- Each round rotates which executable runs first.
- Idiomatic code in each language, no hand tuning.

## Running

Requires clang and clang++, Zig 0.16, GNU make and Python 3.10+.

```
make                # build out/bench_c, out/bench_cpp, out/bench_zig
make run            # build, then python run.py
python run.py --rounds 10 --reps 5
python run.py --langs c zig --no-build
```

`make ZIG=/path/to/zig CC=clang-20 CXX=clang++-20` overrides the toolchains.

Each executable takes `[reps] [warmup]` and prints one line per kernel:

```
name min_ns median_ns checksum_hex
```

## Layout

```
c/src/      one .c and .h per kernel; hash, clock, alloc and harness shared
cpp/src/    one .cpp and .hpp per kernel; hash, clock and harness shared
zig/src/    one .zig per kernel; hash and harness shared; build.zig alongside
run.py      builds, interleaves, verifies checksums, writes results/
Makefile    the build flags
results/    the last run
```
