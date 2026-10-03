#ifndef BENCH_WORLD_H
#define BENCH_WORLD_H

#include <stddef.h>
#include <stdint.h>

#include "harness.h"

/// The engine's PhysicsWorld step for bodies of one box each on a static
/// ground: body changes and waking, proxy refits, pair finding into a hash
/// map, box collision with contact recycling, contact begin and end with
/// island linking and graph colouring, the eight-lane solver, and sleep with
/// island splitting.
typedef struct World World;

/// A world stepped on `threads` threads, one meaning no pool.
World* world_create(uint32_t threads);
/// Frees the world, its pool and everything in it.
void world_destroy(World* world);
/// Removes everything and builds the piles again.
void world_reset(World* world);
/// Advances one fixed step.
void world_step(World* world);
/// Wakes the top body of every eighth pile with a sideways shove.
void world_shove(World* world);
/// Checksum of the bodies and the bookkeeping counts.
uint64_t world_checksum(const World* world);
/// Number of awake bodies.
size_t world_awake_count(const World* world);

/// The world step: 64 steps of 864 boxes in 144 piles of six settling on a
/// ground and falling asleep island by island, 36 tumbling boxes dropped onto
/// every fourth pile that wake it on landing, and every eighth pile woken by a
/// shove at step 45. `world` runs on one thread, `world4` on four.
extern const Case world_case;
extern const Case world4_case;

#endif
