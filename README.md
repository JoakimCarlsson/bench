# bench

C, C++, Zig and Rust running the same twenty-four kernels from a voxel physics
engine. Every kernel produces a checksum, and the runner only reports when all
four languages produce the same bits.

## Results

Intel Core i5-14600K, Arch Linux (kernel 7.2.6), clang 22.1.8, Zig 0.16.0,
rustc 1.98.1. Five interleaved rounds of five repetitions. Median milliseconds
per repetition, and the median relative to the fastest language for that
kernel.

![Average slowdown against the fastest language per kernel, by language](results/overview.svg)

![Heatmap of each kernel's time per language, relative to the fastest](results/kernels.svg)

The same numbers as a table:

| kernel    |     C |   C++ |   Zig |  Rust |     C |   C++ |   Zig |  Rust |
|-----------|------:|------:|------:|------:|------:|------:|------:|------:|
| dda       |  5.07 |  5.14 |  5.06 |  5.34 | 1.00x | 1.02x | 1.00x | 1.06x |
| flood     | 16.02 | 15.81 | 16.63 | 17.32 | 1.01x | 1.00x | 1.05x | 1.10x |
| surface   |  8.94 |  9.19 |  7.63 | 11.81 | 1.17x | 1.20x | 1.00x | 1.55x |
| mips      |  8.47 |  8.36 | 11.12 | 10.50 | 1.01x | 1.00x | 1.33x | 1.26x |
| mass      | 11.48 |  9.85 | 10.78 | 13.77 | 1.17x | 1.00x | 1.09x | 1.40x |
| sweep     |  7.83 |  7.40 |  7.12 |  6.13 | 1.28x | 1.21x | 1.16x | 1.00x |
| bvh       | 16.85 | 16.46 | 16.22 | 16.12 | 1.05x | 1.02x | 1.01x | 1.00x |
| islands   |  2.08 |  2.06 |  2.01 |  2.25 | 1.04x | 1.02x | 1.00x | 1.12x |
| colour    |  1.98 |  1.97 |  1.75 |  1.66 | 1.19x | 1.19x | 1.05x | 1.00x |
| solve     | 13.59 | 13.42 | 16.67 | 13.41 | 1.01x | 1.00x | 1.24x | 1.00x |
| integrate | 10.25 |  8.58 |  7.14 | 10.01 | 1.44x | 1.20x | 1.00x | 1.40x |
| transform | 11.98 | 11.97 |  9.80 | 11.39 | 1.22x | 1.22x | 1.00x | 1.16x |
| unproject |  9.75 |  9.39 |  7.01 |  7.11 | 1.39x | 1.34x | 1.00x | 1.01x |
| decompose |  9.43 |  9.09 |  7.37 |  7.20 | 1.31x | 1.26x | 1.02x | 1.00x |
| raycast   | 13.22 | 13.00 | 11.75 | 13.77 | 1.13x | 1.11x | 1.00x | 1.17x |
| boxbox    | 12.26 | 11.25 | 11.89 | 12.96 | 1.09x | 1.00x | 1.06x | 1.15x |
| wide      | 19.64 | 20.59 | 22.25 | 19.80 | 1.00x | 1.05x | 1.13x | 1.01x |
| broadphase | 22.63 | 23.09 | 22.08 | 21.28 | 1.06x | 1.09x | 1.04x | 1.00x |
| gas       | 26.37 | 24.18 | 29.73 | 21.48 | 1.23x | 1.13x | 1.38x | 1.00x |
| world     | 18.11 | 18.83 | 20.14 | 18.70 | 1.00x | 1.04x | 1.11x | 1.03x |
| world4    | 12.50 | 13.05 | 13.56 | 13.38 | 1.00x | 1.04x | 1.08x | 1.07x |
| slotmap   | 24.65 | 23.53 | 24.64 | 23.80 | 1.05x | 1.00x | 1.05x | 1.01x |
| chunkmap  | 25.95 | 25.45 | 27.24 | 26.83 | 1.02x | 1.00x | 1.07x | 1.05x |
| sort      | 40.98 | 22.07 | 38.25 |  5.24 | 7.82x | 4.21x | 7.30x | 1.00x |

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
| wide      | 4 steps of the engine's 8-lane contact solver: 8192 stacked bodies, 12k coloured contacts  |
| broadphase | 16 frames of 4096 tumbling boxes through the engine's dynamic AABB tree and pair map     |
| gas       | 4 substeps of the engine's particle gas solver over 8192 particles and 4 stirring boxes    |
| world     | 64 steps of the engine's `PhysicsWorld::step` on one thread: 900 boxes settle, sleep, wake  |
| world4    | the same 64 steps on the engine's task pool with 4 threads                                  |
| slotmap   | 4M insert, remove and lookup operations on a 65536-slot generational slot map           |
| chunkmap  | open-addressing hash map: 200k inserts, 2M lookups at 50% hit rate                        |
| sort      | `qsort`, `std::sort`, `std.mem.sortUnstable` and `sort_unstable` over 2^19 random u64 keys |

Kernels allocate before the timed region, except where the engine itself
allocates: `broadphase`, `world` and `world4` rebuild their world inside each
repetition and grow lists as the engine does. `transform` through `boxbox`
port the engine's `math.cpp`, `ray_cast.cpp` and `box_collision.cpp` into a
shared `vecmath` module per language.

`wide` ports `contact_solver.cpp`, `simd.hpp` and the constraint graph's
colouring, single-threaded: each step prepares scalar constraints, packs every
colour into eight-lane bundles, runs 4 substeps of warm start, biased solve,
position integration and relaxed solve with friction, then restitution, and
stores impulses and poses. Each language writes the SIMD the way a port would:
C++ keeps the engine's vector-extension `FloatW8`, C uses AVX intrinsics, Zig
uses `@Vector(8, f32)`, and Rust wraps `core::arch` AVX intrinsics in its own
`F8` type with `unsafe` inside, because `std::simd` is still nightly-only.

`broadphase` ports `aabb_tree.cpp` and `broad_phase.cpp`: each frame refits
the fat bounds that no longer hold, reinserts those leaves with the tree's
rotations, queries the moved proxies against the dynamic and static trees,
records new pairs in a hash map keyed by shape pair, and drops pairs whose fat
bounds parted. `gas` ports the particle system's gas solver: a sorted spatial
hash searched with `lower_bound` for pressure and viscosity, and moving boxes
that stir and push particles.

`world` and `world4` run the engine's whole fixed step for bodies of one box:
body changes and waking, proxy refits, pair finding, box collision with
contact recycling, contacts beginning and ending touching with island linking,
merging and graph colouring, the eight-lane solver, and the sleep pass that
splits islands and puts resting ones to sleep. 144 piles of six boxes settle
and fall asleep; 36 tumbling boxes land on sleeping piles and wake them; a
shove at step 45 wakes every eighth pile. `world4` must produce the same bits
as `world`. Each language ports the engine's spinning task pool rather than
reaching for a library, and uses its standard hash map for the contact index:
`std::unordered_map`, `std.AutoHashMap`, `rustc_hash::FxHashMap` (Rust's
default SipHash map is built to resist hash flooding, not for speed), and a
hand-written open-addressing map in C. Where the engine writes shared state
from several threads under an invariant the compiler cannot check (colours
touch disjoint bodies, collide blocks own disjoint contacts), Rust needs
`unsafe`.

## Rules

- clang, Zig and rustc all lower through LLVM.
- `-O3 -march=native -ffp-contract=off` for C and C++; `ReleaseFast` on the native CPU for Zig; `opt-level = 3`, fat LTO, one codegen unit and `-C target-cpu=native` for Rust.
- Operations are written in the same order in all four sources.
- Rust keeps its bounds checks and its overflow semantics; that is what idiomatic means here.
- SIMD min and max are selects (`a > b ? a : b`), which is exactly what MAXPS and MINPS compute, so every lane rounds the same in every language.
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
c/src/      one .c and .h per kernel and per engine module; hash, clock, alloc, vecmath and harness shared
cpp/src/    one .cpp and .hpp per kernel and per engine module; hash, clock, vecmath, simd and harness shared
zig/src/    one .zig per kernel and per engine module; hash, vecmath and harness shared; build.zig alongside
rust/src/   one .rs per kernel and per engine module; hash, vecmath, simd and harness shared; Cargo.toml alongside
run.py      builds, interleaves, verifies checksums, writes results/
chart.py    draws results/*.svg from results/latest.json
Makefile    the build flags
results/    the last run
```
