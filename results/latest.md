# Results

- date: 2026-09-03
- cpu: Intel(R) Core(TM) i5-14600K
- os: Windows 11
- c/c++: clang version 22.1.2 (https://github.com/llvm/llvm-project.git 1ab49a973e210e97d61e5db6557180dcb92c3e98)
- zig: 0.16.0
- rounds: 5 x 5 reps, 2 warmup

Times in milliseconds per repetition. `best` is the fastest single repetition seen; `median` is the median of each run's median. `rel` is `median` relative to the fastest language for that kernel.

## dda

| language | best | median | rel |
|---|---:|---:|---:|
| C | 5.13 | 5.28 | 1.02x |
| C++ | 5.13 | 5.21 | 1.00x |
| Zig | 5.06 | 5.20 | 1.00x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.99 | 17.40 | 1.01x |
| C++ | 16.55 | 17.31 | 1.00x |
| Zig | 17.46 | 18.15 | 1.05x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.71 | 14.14 | 1.00x |
| C++ | 13.52 | 14.11 | 1.00x |
| Zig | 16.78 | 17.71 | 1.26x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.81 | 25.79 | 1.04x |
| C++ | 24.13 | 24.77 | 1.00x |
| Zig | 24.80 | 26.00 | 1.05x |

