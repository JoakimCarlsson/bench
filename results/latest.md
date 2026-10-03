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
| C | 5.03 | 5.08 | 1.00x |
| C++ | 5.02 | 5.06 | 1.00x |
| Zig | 5.07 | 5.16 | 1.02x |
| Rust | 5.40 | 5.46 | 1.08x |

## flood

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.79 | 16.05 | 1.00x |
| C++ | 15.78 | 16.20 | 1.01x |
| Zig | 16.85 | 17.04 | 1.06x |
| Rust | 16.15 | 16.41 | 1.02x |

## surface

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.94 | 8.97 | 1.18x |
| C++ | 8.84 | 9.68 | 1.27x |
| Zig | 7.60 | 7.61 | 1.00x |
| Rust | 11.93 | 12.08 | 1.59x |

## mips

| language | best | median | rel |
|---|---:|---:|---:|
| C | 8.24 | 8.30 | 1.03x |
| C++ | 8.02 | 8.08 | 1.00x |
| Zig | 10.69 | 10.75 | 1.33x |
| Rust | 10.37 | 10.41 | 1.29x |

## mass

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.31 | 11.45 | 1.16x |
| C++ | 9.82 | 9.88 | 1.00x |
| Zig | 10.73 | 10.80 | 1.09x |
| Rust | 13.66 | 13.71 | 1.39x |

## sweep

| language | best | median | rel |
|---|---:|---:|---:|
| C | 7.71 | 7.83 | 1.23x |
| C++ | 7.19 | 7.39 | 1.16x |
| Zig | 7.08 | 7.24 | 1.13x |
| Rust | 6.29 | 6.39 | 1.00x |

## bvh

| language | best | median | rel |
|---|---:|---:|---:|
| C | 16.61 | 16.97 | 1.05x |
| C++ | 16.20 | 16.57 | 1.03x |
| Zig | 16.35 | 16.44 | 1.02x |
| Rust | 16.01 | 16.15 | 1.00x |

## islands

| language | best | median | rel |
|---|---:|---:|---:|
| C | 2.04 | 2.10 | 1.04x |
| C++ | 1.97 | 2.02 | 1.00x |
| Zig | 1.97 | 2.03 | 1.01x |
| Rust | 2.18 | 2.21 | 1.10x |

## colour

| language | best | median | rel |
|---|---:|---:|---:|
| C | 1.94 | 1.96 | 1.15x |
| C++ | 1.96 | 1.96 | 1.16x |
| Zig | 1.80 | 1.83 | 1.08x |
| Rust | 1.67 | 1.69 | 1.00x |

## solve

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.56 | 13.71 | 1.02x |
| C++ | 13.38 | 13.50 | 1.00x |
| Zig | 16.71 | 16.80 | 1.25x |
| Rust | 13.41 | 13.47 | 1.00x |

## integrate

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.99 | 10.11 | 1.53x |
| C++ | 8.53 | 8.63 | 1.31x |
| Zig | 6.56 | 6.60 | 1.00x |
| Rust | 9.94 | 10.03 | 1.52x |

## transform

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.91 | 11.94 | 1.22x |
| C++ | 11.89 | 11.96 | 1.22x |
| Zig | 9.67 | 9.78 | 1.00x |
| Rust | 11.32 | 11.35 | 1.16x |

## unproject

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.59 | 9.71 | 1.39x |
| C++ | 9.32 | 9.42 | 1.35x |
| Zig | 6.94 | 6.96 | 1.00x |
| Rust | 7.09 | 7.13 | 1.02x |

## decompose

| language | best | median | rel |
|---|---:|---:|---:|
| C | 9.40 | 9.44 | 1.32x |
| C++ | 9.07 | 9.13 | 1.28x |
| Zig | 7.33 | 7.36 | 1.03x |
| Rust | 7.11 | 7.15 | 1.00x |

## raycast

| language | best | median | rel |
|---|---:|---:|---:|
| C | 12.94 | 13.00 | 1.10x |
| C++ | 12.98 | 13.03 | 1.10x |
| Zig | 11.77 | 11.82 | 1.00x |
| Rust | 13.72 | 13.77 | 1.16x |

## boxbox

| language | best | median | rel |
|---|---:|---:|---:|
| C | 12.15 | 12.24 | 1.09x |
| C++ | 11.20 | 11.25 | 1.00x |
| Zig | 11.92 | 11.98 | 1.07x |
| Rust | 12.90 | 12.95 | 1.15x |

## wide

| language | best | median | rel |
|---|---:|---:|---:|
| C | 19.10 | 19.83 | 1.00x |
| C++ | 20.39 | 21.11 | 1.06x |
| Zig | 21.81 | 22.70 | 1.14x |
| Rust | 19.50 | 20.14 | 1.02x |

## broadphase

| language | best | median | rel |
|---|---:|---:|---:|
| C | 22.08 | 22.76 | 1.07x |
| C++ | 22.31 | 23.35 | 1.10x |
| Zig | 22.19 | 22.48 | 1.05x |
| Rust | 21.13 | 21.31 | 1.00x |

## gas

| language | best | median | rel |
|---|---:|---:|---:|
| C | 26.17 | 26.24 | 1.22x |
| C++ | 25.82 | 25.92 | 1.20x |
| Zig | 29.15 | 29.24 | 1.36x |
| Rust | 21.41 | 21.52 | 1.00x |

## world

| language | best | median | rel |
|---|---:|---:|---:|
| C | 17.98 | 18.16 | 1.00x |
| C++ | 18.78 | 18.91 | 1.04x |
| Zig | 20.10 | 20.27 | 1.12x |
| Rust | 18.53 | 18.65 | 1.03x |

## world2

| language | best | median | rel |
|---|---:|---:|---:|
| C | 15.06 | 15.50 | 1.00x |
| C++ | 15.53 | 15.73 | 1.01x |
| Zig | 16.64 | 16.81 | 1.08x |
| Rust | 16.27 | 16.51 | 1.07x |

## world4

| language | best | median | rel |
|---|---:|---:|---:|
| C | 11.60 | 12.40 | 1.00x |
| C++ | 12.06 | 12.74 | 1.03x |
| Zig | 12.53 | 13.06 | 1.05x |
| Rust | 12.17 | 12.91 | 1.04x |

## world8

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.70 | 14.19 | 1.03x |
| C++ | 12.87 | 13.81 | 1.00x |
| Zig | 13.38 | 14.10 | 1.02x |
| Rust | 13.83 | 15.06 | 1.09x |

## world16

| language | best | median | rel |
|---|---:|---:|---:|
| C | 13.34 | 13.95 | 1.02x |
| C++ | 13.02 | 13.68 | 1.00x |
| Zig | 13.74 | 14.59 | 1.07x |
| Rust | 13.79 | 14.68 | 1.07x |

## particles

| language | best | median | rel |
|---|---:|---:|---:|
| C | 26.24 | 26.35 | 1.00x |
| C++ | 28.32 | 28.41 | 1.08x |
| Zig | 28.96 | 29.20 | 1.11x |
| Rust | 29.19 | 29.35 | 1.11x |

## json

| language | best | median | rel |
|---|---:|---:|---:|
| C | 21.24 | 21.94 | 1.22x |
| C++ | 23.29 | 23.79 | 1.33x |
| Zig | 16.22 | 17.94 | 1.00x |
| Rust | 19.51 | 20.14 | 1.12x |

## anim

| language | best | median | rel |
|---|---:|---:|---:|
| C | 19.59 | 19.99 | 1.04x |
| C++ | 20.85 | 21.29 | 1.11x |
| Zig | 20.06 | 20.20 | 1.05x |
| Rust | 18.99 | 19.18 | 1.00x |

## ui

| language | best | median | rel |
|---|---:|---:|---:|
| C | 14.53 | 14.71 | 1.15x |
| C++ | 19.68 | 20.10 | 1.57x |
| Zig | 12.45 | 12.78 | 1.00x |
| Rust | 14.68 | 14.96 | 1.17x |

## slotmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 24.14 | 24.52 | 1.03x |
| C++ | 23.59 | 23.76 | 1.00x |
| Zig | 24.68 | 25.26 | 1.06x |
| Rust | 23.54 | 23.72 | 1.00x |

## chunkmap

| language | best | median | rel |
|---|---:|---:|---:|
| C | 25.28 | 26.09 | 1.00x |
| C++ | 25.47 | 26.33 | 1.01x |
| Zig | 25.56 | 26.33 | 1.01x |
| Rust | 25.83 | 26.63 | 1.02x |

## sort

| language | best | median | rel |
|---|---:|---:|---:|
| C | 41.07 | 41.21 | 7.82x |
| C++ | 22.11 | 22.20 | 4.21x |
| Zig | 36.20 | 36.30 | 6.89x |
| Rust | 5.24 | 5.27 | 1.00x |

## world4safe

A variant of `world4`, same checksum.

| language | best | median | rel to base |
|---|---:|---:|---:|
| Rust | 14.78 | 15.22 | 1.18x |

