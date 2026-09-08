# Results

- date: 2026-09-08
- cpu: Intel(R) Core(TM) i5-14600K
- os: Windows 11
- c/c++: clang version 22.1.2 (https://github.com/llvm/llvm-project.git 1ab49a973e210e97d61e5db6557180dcb92c3e98)
- zig: 0.16.0
- rust: cargo 1.98.1 (797e8a9bc 2026-08-05)
- rounds: 5 x 5 reps, 2 warmup

Times in milliseconds per repetition. `best` is the fastest single repetition seen; `median` is the median of each run's median. `rel` is `median` relative to the fastest language for that kernel.

## dda

| language | best | median | rel |
|---|---:|---:|---:|
| C | 5.10 | 5.13 | 1.01x |
| C++ | 5.03 | 5.14 | 1.01x |
| Zig | 5.05 | 5.09 | 1.00x |
| Rust | 5.20 | 5.29 | 1.04x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.96 | 16.17 | 1.02x |
| C++ | 15.58 | 15.90 | 1.00x |
| Zig | 16.54 | 17.02 | 1.07x |
| Rust | 16.40 | 16.80 | 1.06x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.76 | 8.87 | 1.17x |
| C++ | 8.96 | 9.17 | 1.21x |
| Zig | 7.53 | 7.57 | 1.00x |
| Rust | 12.30 | 12.49 | 1.65x |

## mips

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.41 | 8.64 | 1.01x |
| C++ | 8.25 | 8.54 | 1.00x |
| Zig | 10.74 | 10.85 | 1.27x |
| Rust | 10.44 | 10.77 | 1.26x |

## mass

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.30 | 11.86 | 1.19x |
| C++ | 9.83 | 9.98 | 1.00x |
| Zig | 11.09 | 11.20 | 1.12x |
| Rust | 13.04 | 13.27 | 1.33x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 7.46 | 7.68 | 1.23x |
| C++ | 7.05 | 7.21 | 1.15x |
| Zig | 6.85 | 7.03 | 1.13x |
| Rust | 6.16 | 6.25 | 1.00x |

## bvh

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.64 | 16.93 | 1.03x |
| C++ | 16.18 | 16.79 | 1.02x |
| Zig | 16.14 | 16.62 | 1.01x |
| Rust | 16.07 | 16.46 | 1.00x |

## islands

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.99 | 2.07 | 1.02x |
| C++ | 1.97 | 2.03 | 1.00x |
| Zig | 2.08 | 2.14 | 1.05x |
| Rust | 2.18 | 2.22 | 1.09x |

## colour

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.98 | 2.03 | 1.19x |
| C++ | 1.97 | 1.98 | 1.16x |
| Zig | 1.84 | 1.88 | 1.10x |
| Rust | 1.69 | 1.70 | 1.00x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.64 | 14.16 | 1.02x |
| C++ | 13.46 | 13.93 | 1.00x |
| Zig | 16.69 | 16.99 | 1.22x |
| Rust | 13.57 | 14.25 | 1.02x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 10.11 | 10.33 | 1.49x |
| C++ | 8.58 | 8.73 | 1.26x |
| Zig | 6.55 | 6.92 | 1.00x |
| Rust | 9.91 | 10.12 | 1.46x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.52 | 24.92 | 1.05x |
| C++ | 23.39 | 23.66 | 1.00x |
| Zig | 24.05 | 24.40 | 1.03x |
| Rust | 24.26 | 24.61 | 1.04x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.89 | 26.85 | 1.00x |
| C++ | 25.63 | 27.53 | 1.03x |
| Zig | 27.19 | 28.69 | 1.07x |
| Rust | 26.16 | 26.79 | 1.00x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 47.11 | 47.95 | 8.81x |
| C++ | 9.39 | 9.96 | 1.83x |
| Zig | 37.31 | 38.03 | 6.99x |
| Rust | 5.38 | 5.44 | 1.00x |

