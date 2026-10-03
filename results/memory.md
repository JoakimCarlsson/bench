# Memory

Peak resident MiB a kernel adds over an idle process of the same executable (one repetition, minimum of three runs).

| kernel | C | C++ | Zig | Rust |
|---|---:|---:|---:|---:|
| dda | 0.0 | 0.2 | 0.1 | -0.1 |
| flood | 12.0 | 17.7 | 10.2 | 11.7 |
| surface | 2.1 | 2.0 | 1.8 | 1.7 |
| mips | 17.7 | 18.0 | 17.8 | 17.8 |
| mass | 3.9 | 3.8 | 3.7 | 3.6 |
| sweep | 0.4 | 0.5 | 0.3 | 0.3 |
| bvh | 0.6 | 1.4 | 0.8 | 0.6 |
| islands | 4.2 | 4.1 | 4.1 | 4.1 |
| colour | 4.8 | 4.6 | 4.5 | 4.4 |
| solve | 0.6 | 0.8 | 0.8 | 0.6 |
| integrate | 6.4 | 6.2 | 6.5 | 6.2 |
| transform | 11.5 | 11.5 | 11.4 | 11.2 |
| unproject | 4.8 | 5.0 | 4.8 | 4.8 |
| decompose | 9.9 | 10.1 | 9.8 | 9.9 |
| raycast | 0.3 | 0.4 | 0.2 | 0.0 |
| boxbox | 3.5 | 3.7 | 3.8 | 3.5 |
| wide | 30.6 | 29.5 | 29.2 | 28.5 |
| broadphase | 2.5 | 2.3 | 2.5 | 2.2 |
| gas | 0.9 | 0.8 | 0.9 | 0.8 |
| world | 2.2 | 2.1 | 3.0 | 2.1 |
| world2 | 2.7 | 2.1 | 3.0 | 2.4 |
| world4 | 2.5 | 2.3 | 3.3 | 2.4 |
| world8 | 2.5 | 2.1 | 6.0 | 2.4 |
| world16 | 2.6 | 2.2 | 9.7 | 2.1 |
| particles | 5.9 | 5.9 | 6.0 | 5.7 |
| json | 24.5 | 39.0 | 26.3 | 24.8 |
| anim | 3.6 | 2.4 | 3.6 | 2.4 |
| ui | 18.6 | 19.3 | 12.7 | 13.1 |
| slotmap | 1.6 | 1.6 | 1.4 | 1.2 |
| chunkmap | 7.4 | 7.4 | 7.4 | 7.3 |
| sort | 11.9 | 8.0 | 8.0 | 7.8 |

| idle process (MiB) | 2.2 | 3.9 | 0.7 | 2.6 |
