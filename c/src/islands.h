#ifndef BENCH_ISLANDS_H
#define BENCH_ISLANDS_H

#include "harness.h"

/// Island partition: union-find over 524288 contacts between 65536 bodies,
/// joining only pairs where both are dynamic, then labelling every dynamic
/// body by the lowest slot in its island.
extern const Case islands_case;

#endif
