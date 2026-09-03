# bench

C, C++ and Zig running the same four kernels, compared on equal terms.

The kernels are the hot paths of a voxel physics engine: a voxel raycast, a
connected-components flood, a contact solver and a generational slot map. Each
is written three times, in the idiom of each language, and produces a checksum.
The runner refuses to report a comparison unless every language produces the
same checksum bit for bit, so what is being timed is provably the same work.

## Results

Intel Core i5-14600K, Windows 11, clang 22.1.2, Zig 0.16.0. Five interleaved
rounds of five repetitions. Milliseconds per repetition; `rel` is the median
relative to the fastest language for that kernel.

| kernel  | C     | C++   | Zig   | C rel | C++ rel | Zig rel |
|---------|------:|------:|------:|------:|--------:|--------:|
| dda     | 5.28  | 5.21  | 5.20  | 1.02x | 1.00x   | 1.00x   |
| flood   | 17.40 | 17.31 | 18.15 | 1.01x | 1.00x   | 1.05x   |
| solve   | 14.14 | 14.11 | 17.71 | 1.00x | 1.00x   | 1.26x   |
| slotmap | 25.79 | 24.77 | 26.00 | 1.04x | 1.00x   | 1.05x   |

Full output, including best-of-run numbers, is in [results/latest.md](results/latest.md).

Three of the four kernels are within noise of each other. The solver is the
exception: Zig is a quarter slower on it. Compiling the C solver with Zig's
bundled clang (LLVM 21) runs at C speed, so the gap is in what the Zig
frontend emits for that loop, not in the LLVM version. It is not yet explained;
see the open questions below.

## Kernels

| name    | what it does                                                                                   | shape                                   |
|---------|------------------------------------------------------------------------------------------------|-----------------------------------------|
| dda     | 100k rays walked cell by cell through a 128^3 occupancy bitset until a hit or exit             | float steps, bit tests, unpredictable branches |
| flood   | 6-connected breadth-first labelling of a 128^3 grid at 30% fill                                 | memory bound, queue churn               |
| solve   | 8 frames of substepped sequential impulses over 4096 bodies and 16384 contacts, impulse clamping | float math through indirect body indices |
| slotmap | 4M random insert, remove and lookup operations over 65536 generational slots                   | branchy integer code over a free list   |

All state is allocated before the timed region. No kernel allocates while it runs.

## Fairness

- **Same backend.** clang and Zig both lower through LLVM, so the comparison is
  between language frontends and idioms, not between compilers.
- **Same flags.** `-O3 -march=native -ffp-contract=off` for C and C++,
  `ReleaseFast` on the native CPU for Zig. Contraction is off because Zig's
  default float mode does not fuse multiply-adds; leaving it on for clang would
  produce different bits and a false comparison.
- **Same numbers.** Every kernel folds its output into a splitmix64 checksum.
  Floats are folded by their bit pattern. The runner aborts on any mismatch
  across languages or across repetitions.
- **Same work order.** Operations are written in the same order in all three
  sources, so no language benefits from a reassociation another is denied.
- **Interleaved runs.** The runner rotates which executable goes first in every
  round, so thermal and frequency drift lands on all of them equally.
- **Idiomatic, not contorted.** C uses structs and function pointers, C++ uses
  classes, `std::vector` and templates, Zig uses slices and comptime generics.
  Nobody hand-tunes a kernel to win.

## Running

Requires clang and clang++ (any LLVM 20+), Zig 0.16, GNU make and Python 3.10+.

```
make                # build out/bench_c, out/bench_cpp, out/bench_zig
make run            # build, then python run.py
python run.py --rounds 10 --reps 5
python run.py --langs c zig --no-build
```

Toolchains can be overridden: `make ZIG=/path/to/zig CC=clang-20 CXX=clang++-20`.

Each executable takes `[reps] [warmup]` and prints one line per kernel:

```
name min_ns median_ns checksum_hex
```

`run.py` writes `results/latest.md` and `results/latest.json`.

## Layout

```
c/src/       one .c and .h per kernel; hash, clock, alloc and harness shared
cpp/src/     one .cpp and .hpp per kernel; hash, clock and harness shared
zig/src/     one .zig per kernel; hash and harness shared; build.zig alongside
run.py       builds, interleaves, verifies checksums, renders the table
Makefile     the build flags, which are the fairness rules in executable form
results/     the last run's table and raw samples
```

Every kernel exposes the same three operations: set up its state, run once and
return a checksum, tear down. The harness in each language is generic over that.

## Open questions

- Why is the Zig solver slower? The C solver built with `zig cc` is not, so the
  suspects are the Zig frontend's codegen for that loop and the placement of the
  buffers by Zig's allocator. Neither has been confirmed.
- MSVC is not represented. It would add a second backend to the C++ column.
- Nothing here is multithreaded. A parallel kernel would show how each
  language's threading primitives cost, which matters for a job system.
