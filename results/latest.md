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
| C | 5.14 | 5.27 | 1.01x |
| C++ | 5.07 | 5.22 | 1.01x |
| Zig | 5.05 | 5.19 | 1.00x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.87 | 17.66 | 1.05x |
| C++ | 16.22 | 16.82 | 1.00x |
| Zig | 17.13 | 17.69 | 1.05x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.17 | 9.39 | 1.19x |
| C++ | 9.15 | 9.37 | 1.19x |
| Zig | 7.63 | 7.90 | 1.00x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 7.51 | 7.84 | 1.06x |
| C++ | 7.09 | 7.42 | 1.00x |
| Zig | 8.49 | 8.74 | 1.18x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.72 | 14.21 | 1.00x |
| C++ | 13.90 | 14.32 | 1.01x |
| Zig | 17.01 | 17.65 | 1.24x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 10.33 | 10.55 | 1.48x |
| C++ | 8.81 | 9.07 | 1.27x |
| Zig | 6.76 | 7.14 | 1.00x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.26 | 25.97 | 1.06x |
| C++ | 23.85 | 24.52 | 1.00x |
| Zig | 24.71 | 25.30 | 1.03x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 27.07 | 28.85 | 1.01x |
| C++ | 27.61 | 28.63 | 1.00x |
| Zig | 28.28 | 29.85 | 1.04x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 48.29 | 49.01 | 4.94x |
| C++ | 9.56 | 9.93 | 1.00x |
| Zig | 35.25 | 35.91 | 3.62x |

