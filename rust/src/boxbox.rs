//! Box-box narrowphase: 8 frames over 8192 pairs of oriented boxes, a
//! quarter of them stacked face to face. Separating-axis tests over 15 axes,
//! incident-face clipping against the reference face, reduction to four
//! points, and warm starting from the previous frame's manifold.
use crate::box_collision::{CollisionTolerances, collide_boxes};
use crate::contact::Manifold;
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{Basis, BoxPose, Quat, Vec3};

const PAIR_COUNT: usize = 8192;
const FRAMES: usize = 8;

#[derive(Clone, Copy, Default)]
struct Pair {
    a: BoxPose,
    b: BoxPose,
    velocity: Vec3,
}

pub struct BoxBox {
    pairs: Vec<Pair>,
    manifolds: Vec<Manifold>,
}

impl Case for BoxBox {
    const NAME: &'static str = "boxbox";

    /// Draws the box pairs, a quarter of them stacked.
    fn init() -> BoxBox {
        let mut rng = hash::Rng::new(0xb0b0);
        let mut pairs = vec![Pair::default(); PAIR_COUNT];
        for (i, p) in pairs.iter_mut().enumerate() {
            let rotation = Quat::random(&mut rng);
            p.a.half_extents = Vec3::random(&mut rng, 0.25, 1.5);
            p.a.center = Vec3::random(&mut rng, 0.0, 1000.0);
            p.a.basis = Basis::from_quat(rotation);
            p.b.half_extents = Vec3::random(&mut rng, 0.25, 1.5);
            if i % 4 == 0 {
                let yaw = Quat { x: 0.0, y: rng.unit() - 0.5, z: 0.0, w: 1.0 }.normalize();
                p.b.basis = Basis::from_quat(rotation * yaw);
                let slide = Vec3::random(&mut rng, -0.3, 0.3);
                let lift = p.a.half_extents.y + p.b.half_extents.y - 0.01;
                p.b.center = (p.a.center + p.a.basis.y * lift) + (p.a.basis.x * slide.x + p.a.basis.z * slide.z);
            } else {
                p.b.basis = Basis::from_quat(Quat::random(&mut rng));
                let reach = (p.a.half_extents.length() + p.b.half_extents.length()) * 0.6;
                p.b.center = p.a.center + Vec3::random(&mut rng, -1.0, 1.0) * reach;
            }
            p.velocity = Vec3::random(&mut rng, -0.02, 0.02);
        }
        BoxBox { pairs, manifolds: vec![Manifold::default(); PAIR_COUNT] }
    }

    /// Collides every pair for every frame and folds the manifolds.
    fn run(&mut self) -> u64 {
        self.manifolds.fill(Manifold::default());
        for f in 0..FRAMES {
            let t = f as f32;
            for (p, manifold) in self.pairs.iter().zip(self.manifolds.iter_mut()) {
                let mut b = p.b;
                b.center = b.center + p.velocity * t;
                collide_boxes(&p.a, &b, &CollisionTolerances::default(), manifold);
            }
        }
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for m in &self.manifolds {
            h = hash::add(h, bits(m.normal.x) as u64 | ((bits(m.normal.z) as u64) << 32));
            h = hash::add(h, m.point_count as u64);
            for p in m.active() {
                h = hash::add(h, bits(p.point.y) as u64 | ((bits(p.separation) as u64) << 32));
                h = hash::add(h, p.feature_id as u64 | ((p.persisted as u64) << 32));
            }
        }
        h
    }
}
