//! Bounding volume hierarchy: build a tree over 16384 boxes by midpoint
//! partition on the longest centroid axis, leaves of at most four, then trace
//! 20k rays through it counting the boxes each one hits.
use crate::harness::Case;
use crate::hash;

const BOX_COUNT: u32 = 16384;
const RAYS: usize = 20_000;
const LEAF: u32 = 4;
const MAX_NODES: u32 = 2 * BOX_COUNT;
const STACK_DEPTH: usize = 64;

#[derive(Clone, Copy)]
pub struct Box3 {
    min: [f32; 3],
    max: [f32; 3],
}

#[derive(Clone, Copy)]
pub struct Node {
    min: [f32; 3],
    max: [f32; 3],
    first: u32,
    count: u32,
}

pub struct Bvh {
    boxes: Vec<Box3>,
    nodes: Vec<Node>,
    order: Vec<u32>,
    node_count: u32,
}

fn centroid(b: &Box3, axis: usize) -> f32 {
    (b.min[axis] + b.max[axis]) * 0.5
}

fn slab(min: &[f32; 3], max: &[f32; 3], o: &[f32; 3], inv: &[f32; 3]) -> bool {
    let mut tmin = 0.0f32;
    let mut tmax = 1000.0f32;
    for k in 0..3 {
        let t1 = (min[k] - o[k]) * inv[k];
        let t2 = (max[k] - o[k]) * inv[k];
        let lo = if t1 < t2 { t1 } else { t2 };
        let hi = if t1 < t2 { t2 } else { t1 };
        if lo > tmin {
            tmin = lo;
        }
        if hi < tmax {
            tmax = hi;
        }
    }
    tmax >= tmin
}

impl Bvh {
    fn build(&mut self, node: u32, lo: u32, hi: u32) {
        let mut cmin = [1e30f32; 3];
        let mut cmax = [-1e30f32; 3];
        let mut nmin = [1e30f32; 3];
        let mut nmax = [-1e30f32; 3];
        for &ib in &self.order[lo as usize..hi as usize] {
            let b = &self.boxes[ib as usize];
            for k in 0..3 {
                if b.min[k] < nmin[k] {
                    nmin[k] = b.min[k];
                }
                if b.max[k] > nmax[k] {
                    nmax[k] = b.max[k];
                }
                let c = centroid(b, k);
                if c < cmin[k] {
                    cmin[k] = c;
                }
                if c > cmax[k] {
                    cmax[k] = c;
                }
            }
        }
        self.nodes[node as usize].min = nmin;
        self.nodes[node as usize].max = nmax;
        if hi - lo <= LEAF {
            self.nodes[node as usize].first = lo;
            self.nodes[node as usize].count = hi - lo;
            return;
        }

        let mut axis = 0usize;
        if cmax[1] - cmin[1] > cmax[axis] - cmin[axis] {
            axis = 1;
        }
        if cmax[2] - cmin[2] > cmax[axis] - cmin[axis] {
            axis = 2;
        }
        let split = (cmin[axis] + cmax[axis]) * 0.5;
        let mut i = lo;
        let mut j = hi;
        while i < j {
            if centroid(&self.boxes[self.order[i as usize] as usize], axis) < split {
                i += 1;
            } else {
                j -= 1;
                self.order.swap(i as usize, j as usize);
            }
        }
        let mut mid = i;
        if mid == lo || mid == hi {
            mid = lo + (hi - lo) / 2;
        }

        let first = self.node_count;
        self.nodes[node as usize].first = first;
        self.nodes[node as usize].count = 0;
        self.node_count += 2;
        self.build(first, lo, mid);
        self.build(first + 1, mid, hi);
    }

    fn trace(&self, o: &[f32; 3], inv: &[f32; 3]) -> u32 {
        let mut stack = [0u32; STACK_DEPTH];
        let mut sp = 1usize;
        let mut hits = 0u32;
        stack[0] = 0;
        while sp > 0 {
            sp -= 1;
            let n = &self.nodes[stack[sp] as usize];
            if !slab(&n.min, &n.max, o, inv) {
                continue;
            }
            if n.count == 0 {
                stack[sp] = n.first + 1;
                stack[sp + 1] = n.first;
                sp += 2;
                continue;
            }
            for &ib in &self.order[n.first as usize..(n.first + n.count) as usize] {
                let b = &self.boxes[ib as usize];
                hits += u32::from(slab(&b.min, &b.max, o, inv));
            }
        }
        hits
    }
}

impl Case for Bvh {
    const NAME: &'static str = "bvh";

    fn init() -> Bvh {
        let mut rng = hash::Rng::new(0xb4);
        let mut boxes = vec![Box3 { min: [0.0; 3], max: [0.0; 3] }; BOX_COUNT as usize];
        for b in boxes.iter_mut() {
            for k in 0..3 {
                b.min[k] = rng.unit() * 200.0;
                b.max[k] = b.min[k] + 0.5 + rng.unit() * 3.0;
            }
        }
        let node = Node { min: [0.0; 3], max: [0.0; 3], first: 0, count: 0 };
        Bvh {
            boxes,
            nodes: vec![node; MAX_NODES as usize],
            order: vec![0; BOX_COUNT as usize],
            node_count: 0,
        }
    }

    fn run(&mut self) -> u64 {
        for (i, o) in self.order.iter_mut().enumerate() {
            *o = i as u32;
        }
        self.node_count = 1;
        self.build(0, 0, BOX_COUNT);

        let mut rng = hash::Rng::new(0x7ace);
        let mut h = 0u64;
        let mut total = 0u64;
        for _ in 0..RAYS {
            let mut o = [0.0f32; 3];
            let mut d = [0.0f32; 3];
            let mut inv = [0.0f32; 3];
            for k in 0..3 {
                o[k] = rng.unit() * 200.0;
            }
            for k in 0..3 {
                d[k] = rng.unit() * 2.0 - 1.0;
            }
            let mut len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            if len < 1e-3 {
                d = [1.0, 0.0, 0.0];
                len = 1.0;
            }
            for k in 0..3 {
                d[k] /= len;
                if d[k].abs() < 1e-6 {
                    d[k] = 1e-6;
                }
                inv[k] = 1.0 / d[k];
            }
            let hits = self.trace(&o, &inv);
            h = hash::add(h, hits as u64);
            total += hits as u64;
        }
        hash::add(hash::add(h, total), self.node_count as u64)
    }
}
