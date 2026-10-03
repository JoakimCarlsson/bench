//! Broad phase: 16 frames of 4096 tumbling boxes over a field of 1024
//! static tiles, through the engine's dynamic AABB tree. Each frame refits
//! fat bounds that no longer hold, reinserts those leaves with rotations,
//! finds the new pairs of every moved proxy, records them in a hash map
//! keyed by shape pair, and drops pairs whose fat bounds parted.
use crate::box_collision::box_aabb;
use crate::broad_phase::{BroadPhase, ProxyId};
use crate::contact::{ShapeRef, pair_key};
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{self, Aabb, Basis, BoxPose, Quat, Vec3};
use rustc_hash::FxHashMap;

const BODY_COUNT: usize = 4096;
const TILES_SIDE: u32 = 32;
const FRAMES: usize = 16;
const SPECULATIVE: f32 = 0.02;
const MAX_AABB_MARGIN: f32 = 0.05;
const AABB_MARGIN_FRACTION: f32 = 0.125;
const DT: f32 = 1.0 / 60.0;
const EXTENT_XZ: f32 = 45.0;
const EXTENT_Y: f32 = 16.0;

#[derive(Clone, Copy, Default)]
struct Body {
    pose: BoxPose,
    rotation: Quat,
    velocity: Vec3,
    spin: Vec3,
}

#[derive(Clone, Copy)]
struct Pair {
    proxy_a: ProxyId,
    proxy_b: ProxyId,
    key: u64,
    alive: bool,
}

pub struct BroadPhaseCase {
    initial: Vec<Body>,
    bodies: Vec<Body>,
    tiles: Vec<BoxPose>,
    body_proxies: Vec<ProxyId>,
    broad_phase: BroadPhase,
    pairs: Vec<Pair>,
    free_pairs: Vec<u32>,
    pair_index: FxHashMap<u64, u32>,
}

/// Fat margin for a box, the engine's shape_margin.
pub fn shape_margin(h: Vec3) -> f32 {
    vecmath::min(MAX_AABB_MARGIN, AABB_MARGIN_FRACTION * 2.0 * vecmath::max3(h.x, h.y, h.z))
}

/// Bounds the broad phase tests against: the box grown by the speculative
/// distance.
fn tight_aabb(pose: &BoxPose) -> Aabb {
    box_aabb(pose).grow(SPECULATIVE)
}

/// `value` wrapped into [0, extent).
fn wrap(value: f32, extent: f32) -> f32 {
    if value < 0.0 {
        return value + extent;
    }
    if value >= extent {
        return value - extent;
    }
    value
}

impl BroadPhaseCase {
    /// Creates every proxy from the initial poses.
    fn reset(&mut self) {
        self.broad_phase = BroadPhase::default();
        self.pairs.clear();
        self.free_pairs.clear();
        self.pair_index.clear();
        self.bodies.copy_from_slice(&self.initial);
        for (i, tile) in self.tiles.iter().enumerate() {
            self.broad_phase.create_proxy(ShapeRef { body: i as u32, is_static: true }, &tight_aabb(tile), false);
        }
        for (i, body) in self.bodies.iter().enumerate() {
            let fat = tight_aabb(&body.pose).grow(shape_margin(body.pose.half_extents));
            self.body_proxies[i] = self.broad_phase.create_proxy(ShapeRef { body: i as u32, is_static: false }, &fat, false);
        }
    }

    /// Advances the bodies and moves the proxies whose fat bounds they left.
    fn move_bodies(&mut self) {
        for (b, &proxy) in self.bodies.iter_mut().zip(self.body_proxies.iter()) {
            let c = b.pose.center + b.velocity * DT;
            b.pose.center = Vec3::new(wrap(c.x, EXTENT_XZ), wrap(c.y, EXTENT_Y), wrap(c.z, EXTENT_XZ));
            b.rotation = b.rotation.integrate(b.spin * DT);
            b.pose.basis = Basis::from_quat(b.rotation);
            let tight = tight_aabb(&b.pose);
            if !self.broad_phase.fat_aabb(proxy).contains(&tight) {
                self.broad_phase.move_proxy(proxy, &tight.grow(shape_margin(b.pose.half_extents)));
            }
        }
    }

    /// Records the new pairs of the moved proxies.
    fn update_pairs(&mut self) {
        let broad_phase = &self.broad_phase;
        let (pairs, free_pairs, pair_index) = (&mut self.pairs, &mut self.free_pairs, &mut self.pair_index);
        broad_phase.query_moved(0, broad_phase.moved_count(), |a, b| {
            let key = pair_key(broad_phase.shape(a), broad_phase.shape(b));
            if pair_index.contains_key(&key) {
                return;
            }
            let pair = Pair { proxy_a: a, proxy_b: b, key, alive: true };
            let id = match free_pairs.pop() {
                None => {
                    pairs.push(pair);
                    (pairs.len() - 1) as u32
                }
                Some(id) => {
                    pairs[id as usize] = pair;
                    id
                }
            };
            pair_index.entry(key).or_insert(id);
        });
        self.broad_phase.clear_moves();
    }

    /// Drops pairs whose fat bounds no longer overlap.
    fn drop_parted_pairs(&mut self) {
        for (id, pair) in self.pairs.iter_mut().enumerate() {
            if !pair.alive {
                continue;
            }
            if self.broad_phase.fat_aabb(pair.proxy_a).overlaps(self.broad_phase.fat_aabb(pair.proxy_b)) {
                continue;
            }
            self.pair_index.remove(&pair.key);
            pair.alive = false;
            self.free_pairs.push(id as u32);
        }
    }
}

impl Case for BroadPhaseCase {
    const NAME: &'static str = "broadphase";

    /// Draws the bodies and lays out the tiles.
    fn init() -> BroadPhaseCase {
        let mut rng = hash::Rng::new(0xb40ad);
        let mut initial = vec![Body::default(); BODY_COUNT];
        for b in initial.iter_mut() {
            b.pose.half_extents = Vec3::random(&mut rng, 0.25, 0.75);
            let x = rng.unit() * EXTENT_XZ;
            let y = rng.unit() * EXTENT_Y;
            let z = rng.unit() * EXTENT_XZ;
            b.pose.center = Vec3::new(x, y, z);
            b.rotation = Quat::random(&mut rng);
            b.pose.basis = Basis::from_quat(b.rotation);
            b.velocity = Vec3::random(&mut rng, -1.0, 1.0);
            b.spin = Vec3::random(&mut rng, -0.5, 0.5);
        }
        let mut tiles = Vec::new();
        for z in 0..TILES_SIDE {
            for x in 0..TILES_SIDE {
                let center = Vec3::new(x as f32 * 2.0 + 1.0, -0.5, z as f32 * 2.0 + 1.0);
                tiles.push(BoxPose { half_extents: Vec3::new(1.0, 0.5, 1.0), center, basis: Basis::default() });
            }
        }
        BroadPhaseCase {
            bodies: initial.clone(),
            initial,
            tiles,
            body_proxies: vec![0; BODY_COUNT],
            broad_phase: BroadPhase::default(),
            pairs: Vec::new(),
            free_pairs: Vec::new(),
            pair_index: FxHashMap::default(),
        }
    }

    /// Steps every frame and folds the pair counts, pairs and fat bounds.
    fn run(&mut self) -> u64 {
        self.reset();
        let mut h = 0u64;
        for _ in 0..FRAMES {
            self.move_bodies();
            self.update_pairs();
            self.drop_parted_pairs();
            h = hash::add(h, self.pair_index.len() as u64);
        }
        for pair in &self.pairs {
            h = hash::add(h, if pair.alive { pair.key } else { 0 });
        }
        let bits = hash::f32_bits;
        for &proxy in &self.body_proxies {
            let fat = self.broad_phase.fat_aabb(proxy);
            h = hash::add(h, bits(fat.min.x) as u64 | ((bits(fat.max.y) as u64) << 32));
        }
        h
    }
}
