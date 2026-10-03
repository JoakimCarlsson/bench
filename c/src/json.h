#ifndef BENCH_JSON_H
#define BENCH_JSON_H

#include "harness.h"

/// Scene load: the engine's JSON reader over a generated scene document of a
/// few megabytes. Setup writes the text with integer arithmetic only, so it
/// is the same bytes in every language. Each run parses it into a value tree
/// with the engine's strict recursive descent parser, walks the entities the
/// way the entity document code does (components looked up by key, vectors
/// and transforms read from number arrays), folds everything it reads into a
/// checksum, then drops the tree.
extern const Case json_case;

#endif
