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
| C | 5.06 | 5.07 | 1.00x |
| C++ | 5.09 | 5.14 | 1.02x |
| Zig | 5.04 | 5.06 | 1.00x |
| Rust | 5.29 | 5.34 | 1.06x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.89 | 16.02 | 1.01x |
| C++ | 15.60 | 15.81 | 1.00x |
| Zig | 16.45 | 16.63 | 1.05x |
| Rust | 16.92 | 17.32 | 1.10x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.87 | 8.94 | 1.17x |
| C++ | 8.83 | 9.19 | 1.20x |
| Zig | 7.58 | 7.63 | 1.00x |
| Rust | 11.67 | 11.81 | 1.55x |

## mips

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.24 | 8.47 | 1.01x |
| C++ | 8.14 | 8.36 | 1.00x |
| Zig | 10.93 | 11.12 | 1.33x |
| Rust | 10.34 | 10.50 | 1.26x |

## mass

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.32 | 11.48 | 1.17x |
| C++ | 9.83 | 9.85 | 1.00x |
| Zig | 10.68 | 10.78 | 1.09x |
| Rust | 13.69 | 13.77 | 1.40x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 7.68 | 7.83 | 1.28x |
| C++ | 7.20 | 7.40 | 1.21x |
| Zig | 6.99 | 7.12 | 1.16x |
| Rust | 6.06 | 6.13 | 1.00x |

## bvh

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.69 | 16.85 | 1.05x |
| C++ | 16.35 | 16.46 | 1.02x |
| Zig | 16.17 | 16.22 | 1.01x |
| Rust | 16.07 | 16.12 | 1.00x |

## islands

| language | best | median | rel |
|---|---:|---:|---:|
| C | 2.06 | 2.08 | 1.04x |
| C++ | 2.03 | 2.06 | 1.02x |
| Zig | 1.99 | 2.01 | 1.00x |
| Rust | 2.24 | 2.25 | 1.12x |

## colour

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.95 | 1.98 | 1.19x |
| C++ | 1.96 | 1.97 | 1.19x |
| Zig | 1.73 | 1.75 | 1.05x |
| Rust | 1.65 | 1.66 | 1.00x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.52 | 13.59 | 1.01x |
| C++ | 13.37 | 13.42 | 1.00x |
| Zig | 16.63 | 16.67 | 1.24x |
| Rust | 13.38 | 13.41 | 1.00x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 10.16 | 10.25 | 1.44x |
| C++ | 8.52 | 8.58 | 1.20x |
| Zig | 7.11 | 7.14 | 1.00x |
| Rust | 9.91 | 10.01 | 1.40x |

## transform

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.88 | 11.98 | 1.22x |
| C++ | 11.90 | 11.97 | 1.22x |
| Zig | 9.73 | 9.80 | 1.00x |
| Rust | 11.31 | 11.39 | 1.16x |

## unproject

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.73 | 9.75 | 1.39x |
| C++ | 9.31 | 9.39 | 1.34x |
| Zig | 6.95 | 7.01 | 1.00x |
| Rust | 7.08 | 7.11 | 1.01x |

## decompose

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.34 | 9.43 | 1.31x |
| C++ | 9.04 | 9.09 | 1.26x |
| Zig | 7.34 | 7.37 | 1.02x |
| Rust | 7.14 | 7.20 | 1.00x |

## raycast

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.15 | 13.22 | 1.13x |
| C++ | 12.96 | 13.00 | 1.11x |
| Zig | 11.70 | 11.75 | 1.00x |
| Rust | 13.70 | 13.77 | 1.17x |

## boxbox

| language | best | median | rel |
|---|---:|---:|---:|
| C | 12.21 | 12.26 | 1.09x |
| C++ | 11.16 | 11.25 | 1.00x |
| Zig | 11.83 | 11.89 | 1.06x |
| Rust | 12.92 | 12.96 | 1.15x |

## wide

| language | best | median | rel |
|---|---:|---:|---:|
| C | 19.23 | 19.64 | 1.00x |
| C++ | 20.36 | 20.59 | 1.05x |
| Zig | 21.84 | 22.25 | 1.13x |
| Rust | 19.50 | 19.80 | 1.01x |

## broadphase

| language | best | median | rel |
|---|---:|---:|---:|
| C | 22.04 | 22.63 | 1.06x |
| C++ | 22.41 | 23.09 | 1.09x |
| Zig | 21.95 | 22.08 | 1.04x |
| Rust | 21.11 | 21.28 | 1.00x |

## gas

| language | best | median | rel |
|---|---:|---:|---:|
| C | 26.26 | 26.37 | 1.23x |
| C++ | 24.13 | 24.18 | 1.13x |
| Zig | 29.59 | 29.73 | 1.38x |
| Rust | 21.39 | 21.48 | 1.00x |

## world

| language | best | median | rel |
|---|---:|---:|---:|
| C | 18.04 | 18.11 | 1.00x |
| C++ | 18.75 | 18.83 | 1.04x |
| Zig | 19.99 | 20.14 | 1.11x |
| Rust | 18.57 | 18.70 | 1.03x |

## world4

| language | best | median | rel |
|---|---:|---:|---:|
| C | 12.25 | 12.50 | 1.00x |
| C++ | 11.83 | 13.05 | 1.04x |
| Zig | 13.27 | 13.56 | 1.08x |
| Rust | 12.84 | 13.38 | 1.07x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.38 | 24.65 | 1.05x |
| C++ | 23.35 | 23.53 | 1.00x |
| Zig | 24.41 | 24.64 | 1.05x |
| Rust | 23.66 | 23.80 | 1.01x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.58 | 25.95 | 1.02x |
| C++ | 25.21 | 25.45 | 1.00x |
| Zig | 26.78 | 27.24 | 1.07x |
| Rust | 26.10 | 26.83 | 1.05x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 40.82 | 40.98 | 7.82x |
| C++ | 21.96 | 22.07 | 4.21x |
| Zig | 38.08 | 38.25 | 7.30x |
| Rust | 5.19 | 5.24 | 1.00x |

