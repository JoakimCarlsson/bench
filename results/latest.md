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
| C | 5.08 | 5.13 | 1.01x |
| C++ | 5.04 | 5.10 | 1.00x |
| Zig | 5.05 | 5.13 | 1.01x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.89 | 16.31 | 1.00x |
| C++ | 15.68 | 16.27 | 1.00x |
| Zig | 16.75 | 17.31 | 1.06x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.76 | 8.93 | 1.17x |
| C++ | 9.01 | 9.11 | 1.19x |
| Zig | 7.55 | 7.64 | 1.00x |

## mips

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.39 | 8.65 | 1.00x |
| C++ | 8.54 | 8.75 | 1.01x |
| Zig | 10.73 | 10.97 | 1.27x |

## mass

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.36 | 11.59 | 1.14x |
| C++ | 9.86 | 10.20 | 1.00x |
| Zig | 11.11 | 11.34 | 1.11x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 7.53 | 7.73 | 1.10x |
| C++ | 7.10 | 7.25 | 1.03x |
| Zig | 6.89 | 7.04 | 1.00x |

## bvh

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.65 | 17.37 | 1.05x |
| C++ | 16.27 | 16.92 | 1.02x |
| Zig | 16.20 | 16.56 | 1.00x |

## islands

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.99 | 2.09 | 1.04x |
| C++ | 1.97 | 2.02 | 1.00x |
| Zig | 2.07 | 2.14 | 1.06x |

## colour

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.98 | 2.00 | 1.09x |
| C++ | 1.97 | 1.99 | 1.08x |
| Zig | 1.83 | 1.85 | 1.00x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.63 | 14.09 | 1.02x |
| C++ | 13.46 | 13.79 | 1.00x |
| Zig | 16.76 | 17.56 | 1.27x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 10.09 | 10.32 | 1.54x |
| C++ | 8.57 | 9.11 | 1.36x |
| Zig | 6.57 | 6.71 | 1.00x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.61 | 25.10 | 1.04x |
| C++ | 23.63 | 24.13 | 1.00x |
| Zig | 24.14 | 24.69 | 1.02x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.87 | 27.47 | 1.01x |
| C++ | 26.08 | 27.17 | 1.00x |
| Zig | 27.35 | 27.92 | 1.03x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 47.25 | 48.84 | 5.04x |
| C++ | 9.42 | 9.68 | 1.00x |
| Zig | 37.63 | 38.72 | 4.00x |

