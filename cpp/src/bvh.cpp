#include "bvh.hpp"

#include <cmath>
#include <utility>

#include "hash.hpp"

namespace bench {

namespace {

float centroid(const Bvh::Box& b, int axis) {
    return (b.min[axis] + b.max[axis]) * 0.5f;
}

bool slab(const float* min, const float* max, const float* o, const float* inv) {
    float tmin = 0.0f, tmax = 1000.0f;
    for (int k = 0; k < 3; k++) {
        float t1 = (min[k] - o[k]) * inv[k], t2 = (max[k] - o[k]) * inv[k];
        float lo = t1 < t2 ? t1 : t2, hi = t1 < t2 ? t2 : t1;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
    }
    return tmax >= tmin;
}

} // namespace

Bvh::Bvh() : boxes_(boxes_n), nodes_(max_nodes), order_(boxes_n) {
    Rng rng{0xb4};
    for (Box& b : boxes_) {
        for (int k = 0; k < 3; k++) {
            b.min[k] = rng.unit() * 200.0f;
            b.max[k] = b.min[k] + 0.5f + rng.unit() * 3.0f;
        }
    }
}

void Bvh::build(uint32_t node, uint32_t lo, uint32_t hi) {
    Node& n = nodes_[node];
    float cmin[3], cmax[3];
    for (int k = 0; k < 3; k++) { n.min[k] = 1e30f; n.max[k] = -1e30f; cmin[k] = 1e30f; cmax[k] = -1e30f; }
    for (uint32_t i = lo; i < hi; i++) {
        const Box& b = boxes_[order_[i]];
        for (int k = 0; k < 3; k++) {
            if (b.min[k] < n.min[k]) n.min[k] = b.min[k];
            if (b.max[k] > n.max[k]) n.max[k] = b.max[k];
            float c = centroid(b, k);
            if (c < cmin[k]) cmin[k] = c;
            if (c > cmax[k]) cmax[k] = c;
        }
    }
    if (hi - lo <= leaf) { n.first = lo; n.count = hi - lo; return; }

    int axis = 0;
    if (cmax[1] - cmin[1] > cmax[axis] - cmin[axis]) axis = 1;
    if (cmax[2] - cmin[2] > cmax[axis] - cmin[axis]) axis = 2;
    float split = (cmin[axis] + cmax[axis]) * 0.5f;
    uint32_t i = lo, j = hi;
    while (i < j) {
        if (centroid(boxes_[order_[i]], axis) < split) {
            i++;
        } else {
            j--;
            std::swap(order_[i], order_[j]);
        }
    }
    uint32_t mid = i;
    if (mid == lo || mid == hi) mid = lo + (hi - lo) / 2;

    n.first = node_count_;
    n.count = 0;
    node_count_ += 2;
    build(n.first, lo, mid);
    build(n.first + 1, mid, hi);
}

uint32_t Bvh::trace(const float* o, const float* inv) const {
    uint32_t stack[stack_depth];
    uint32_t sp = 0, hits = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& n = nodes_[stack[--sp]];
        if (!slab(n.min, n.max, o, inv)) continue;
        if (n.count == 0) {
            stack[sp++] = n.first + 1;
            stack[sp++] = n.first;
            continue;
        }
        for (uint32_t i = 0; i < n.count; i++) {
            const Box& b = boxes_[order_[n.first + i]];
            hits += slab(b.min, b.max, o, inv);
        }
    }
    return hits;
}

uint64_t Bvh::run() {
    for (uint32_t i = 0; i < boxes_n; i++) order_[i] = i;
    node_count_ = 1;
    build(0, 0, boxes_n);

    Rng rng{0x7ace};
    uint64_t h = 0, total = 0;
    for (int r = 0; r < rays; r++) {
        float o[3], d[3], inv[3];
        for (int k = 0; k < 3; k++) o[k] = rng.unit() * 200.0f;
        for (int k = 0; k < 3; k++) d[k] = rng.unit() * 2.0f - 1.0f;
        float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len < 1e-3f) { d[0] = 1.0f; d[1] = 0.0f; d[2] = 0.0f; len = 1.0f; }
        for (int k = 0; k < 3; k++) {
            d[k] /= len;
            if (std::fabs(d[k]) < 1e-6f) d[k] = 1e-6f;
            inv[k] = 1.0f / d[k];
        }
        uint32_t hits = trace(o, inv);
        h = hash_add(h, hits);
        total += hits;
    }
    return hash_add(hash_add(h, total), node_count_);
}

} // namespace bench
