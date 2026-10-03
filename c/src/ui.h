#ifndef BENCH_UI_H
#define BENCH_UI_H

#include "harness.h"

/// The engine's game-facing UI: a tree of about sixteen thousand controls
/// (an editor screen with a toolbar, a long hierarchy, a tabbed asset grid
/// and an inspector with long property lists) laid out and drawn into a
/// batched vertex, index and command list. Each of the frames resizes the
/// root, changes the text and minimum size of some controls so the
/// invalidation climbs the tree, runs the layout pass, draws the whole
/// tree, and folds every control rect and all draw data into the checksum.
///
/// The C port keeps the object structure of the C++ one: heap-allocated
/// controls with vtables of function pointers, owned children, a canvas
/// with a vtable, and string-keyed theme maps walked at every lookup.
extern const Case ui_case;

#endif
