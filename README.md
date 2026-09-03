# bench

C, C++ and Zig running the same fourteen kernels from a voxel physics engine.
Every kernel produces a checksum, and the runner only reports when all three
languages produce the same bits.

## Results

Intel Core i5-14600K, Windows 11, clang 22.1.2, Zig 0.16.0. Five interleaved
rounds of five repetitions. Median milliseconds per repetition, and the
median relative to the fastest language for that kernel.

| kernel    | C     | C++   | Zig   | C     | C++   | Zig   |
|-----------|------:|------:|------:|------:|------:|------:|
| dda       | 5.13  | 5.10  | 5.13  | 1.01x | 1.00x | 1.01x |
| flood     | 16.31 | 16.27 | 17.31 | 1.00x | 1.00x | 1.06x |
| surface   | 8.93  | 9.11  | 7.64  | 1.17x | 1.19x | 1.00x |
| mips      | 8.65  | 8.75  | 10.97 | 1.00x | 1.01x | 1.27x |
| mass      | 11.59 | 10.20 | 11.34 | 1.14x | 1.00x | 1.11x |
| sweep     | 7.73  | 7.25  | 7.04  | 1.10x | 1.03x | 1.00x |
| bvh       | 17.37 | 16.92 | 16.56 | 1.05x | 1.02x | 1.00x |
| islands   | 2.09  | 2.02  | 2.14  | 1.04x | 1.00x | 1.06x |
| colour    | 2.00  | 1.99  | 1.85  | 1.09x | 1.08x | 1.00x |
| solve     | 14.09 | 13.79 | 17.56 | 1.02x | 1.00x | 1.27x |
| integrate | 10.32 | 9.11  | 6.71  | 1.54x | 1.36x | 1.00x |
| slotmap   | 25.10 | 24.13 | 24.69 | 1.04x | 1.00x | 1.02x |
| chunkmap  | 27.47 | 27.17 | 27.92 | 1.01x | 1.00x | 1.03x |
| sort      | 48.84 | 9.68  | 38.72 | 5.04x | 1.00x | 4.00x |

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
