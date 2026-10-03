# Results

- date: 2026-10-03
- cpu: Intel(R) Core(TM) i5-14600K
- os: Linux 7.2.6-arch2-1
- c/c++: clang version 22.1.8
- zig: 0.16.0
- rust: cargo 1.98.1 (797e8a9bc 2026-08-05)
- rounds: 5 x 5 reps, 2 warmup

Times in milliseconds per repetition. `best` is the fastest single repetition seen; `median` is the median of each run's median. `rel` is `median` relative to the fastest language for that kernel.

## dda

| language | best | median | rel |
|---|---:|---:|---:|
| C | 5.04 | 5.07 | 1.00x |
| C++ | 5.12 | 5.16 | 1.02x |
| Zig | 5.11 | 5.14 | 1.01x |
| Rust | 5.09 | 5.14 | 1.01x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.82 | 15.97 | 1.00x |
| C++ | 15.50 | 15.89 | 1.00x |
| Zig | 16.52 | 16.86 | 1.06x |
| Rust | 16.08 | 16.41 | 1.03x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.82 | 8.85 | 1.16x |
| C++ | 8.81 | 8.89 | 1.17x |
| Zig | 7.58 | 7.62 | 1.00x |
| Rust | 11.79 | 11.85 | 1.56x |

## mips

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.06 | 8.15 | 1.00x |
| C++ | 8.14 | 8.22 | 1.01x |
| Zig | 10.50 | 10.63 | 1.31x |
| Rust | 10.35 | 10.40 | 1.28x |

## mass

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.33 | 11.41 | 1.15x |
| C++ | 9.82 | 9.91 | 1.00x |
| Zig | 10.89 | 11.00 | 1.11x |
| Rust | 13.63 | 13.73 | 1.39x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 6.87 | 6.99 | 1.02x |
| C++ | 7.25 | 7.37 | 1.07x |
| Zig | 7.83 | 7.97 | 1.16x |
| Rust | 6.75 | 6.89 | 1.00x |

## bvh

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.69 | 17.01 | 1.06x |
| C++ | 16.37 | 16.54 | 1.03x |
| Zig | 16.45 | 16.56 | 1.03x |
| Rust | 16.02 | 16.07 | 1.00x |

## islands

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.99 | 2.03 | 1.00x |
| C++ | 2.04 | 2.06 | 1.02x |
| Zig | 1.99 | 2.02 | 1.00x |
| Rust | 2.25 | 2.29 | 1.14x |

## colour

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.99 | 2.00 | 1.20x |
| C++ | 1.97 | 1.98 | 1.18x |
| Zig | 1.69 | 1.71 | 1.02x |
| Rust | 1.66 | 1.67 | 1.00x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.52 | 13.61 | 1.01x |
| C++ | 13.40 | 13.46 | 1.00x |
| Zig | 16.58 | 16.64 | 1.24x |
| Rust | 13.38 | 13.41 | 1.00x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 10.03 | 10.11 | 1.41x |
| C++ | 8.50 | 8.59 | 1.20x |
| Zig | 7.11 | 7.18 | 1.00x |
| Rust | 9.89 | 10.07 | 1.40x |

## transform

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.88 | 11.94 | 1.22x |
| C++ | 11.90 | 11.93 | 1.22x |
| Zig | 9.72 | 9.76 | 1.00x |
| Rust | 11.50 | 11.55 | 1.18x |

## unproject

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.67 | 9.75 | 1.39x |
| C++ | 9.31 | 9.50 | 1.36x |
| Zig | 6.97 | 6.99 | 1.00x |
| Rust | 7.07 | 7.10 | 1.02x |

## decompose

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.33 | 9.37 | 1.32x |
| C++ | 9.04 | 9.16 | 1.29x |
| Zig | 7.08 | 7.10 | 1.00x |
| Rust | 7.16 | 7.18 | 1.01x |

## raycast

| language | best | median | rel |
|---|---:|---:|---:|
| C | 12.85 | 12.91 | 1.10x |
| C++ | 12.97 | 13.02 | 1.11x |
| Zig | 11.65 | 11.78 | 1.00x |
| Rust | 13.71 | 13.74 | 1.17x |

## boxbox

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.77 | 11.82 | 1.09x |
| C++ | 10.73 | 10.80 | 1.00x |
| Zig | 10.85 | 10.90 | 1.01x |
| Rust | 11.48 | 11.53 | 1.07x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.48 | 24.86 | 1.05x |
| C++ | 23.50 | 23.66 | 1.00x |
| Zig | 24.43 | 24.65 | 1.04x |
| Rust | 23.42 | 23.69 | 1.00x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.03 | 26.01 | 1.00x |
| C++ | 25.65 | 26.24 | 1.01x |
| Zig | 26.10 | 26.68 | 1.03x |
| Rust | 25.75 | 26.23 | 1.01x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 40.95 | 41.06 | 7.81x |
| C++ | 22.03 | 22.23 | 4.23x |
| Zig | 38.03 | 38.24 | 7.27x |
| Rust | 5.21 | 5.26 | 1.00x |

