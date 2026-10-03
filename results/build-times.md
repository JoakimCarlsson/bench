# Build times

- date: 2026-10-03
- cpu: Intel(R) Core(TM) i5-14600K
- threads: 20
- repeats: 3

Seconds, median of the repeats; the `-j` rows use every hardware thread.

| case | C | C++ | Zig | Rust |
|---|---:|---:|---:|---:|
| clean | 3.8 | 19 | 21 | 11 |
| clean -j | 0.5 | 2.9 | n/a | n/a |
| one file | 0.3 | 1.5 | 20 | 9.1 |
| one header | 0.5 | 1.9 | 21 | 9.0 |

Rust with Cargo's default release profile (no LTO, 16 codegen units) instead of the benchmark's (fat LTO, 1 codegen unit):

- clean: 3.5 s, one file: 2.0 s
- `cargo check` after one edit: 0.5 s

| | C | C++ | Zig | Rust |
|---|---:|---:|---:|---:|
| source lines | 15,234 | 13,960 | 13,772 | 17,283 |
| executable KiB | 386 | 594 | 7,418 | 1,238 |

Files recompiled when the math header changes: 26 of 50 in C, 21 of 45 in C++.
