//! Transform decomposition: for 262144 rotation-and-scale pairs, a quarter of
//! them mirrored, build the basis, recover its scale and rotation through
//! the four-branch basis-to-quaternion conversion, invert it, and rotate a
//! point both ways.
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{Basis, Quat, Vec3};

const ITEM_COUNT: usize = 262144;

#[derive(Clone, Copy, Default)]
struct Item {
    rotation: Quat,
    scale: Vec3,
    point: Vec3,
}

pub struct Decompose {
    items: Vec<Item>,
}

impl Case for Decompose {
    const NAME: &'static str = "decompose";

    fn init() -> Decompose {
        let mut rng = hash::Rng::new(0xdec0);
        let mut items = vec![Item::default(); ITEM_COUNT];
        for (i, it) in items.iter_mut().enumerate() {
            it.rotation = Quat::random(&mut rng);
            it.scale = Vec3::random(&mut rng, 0.25, 4.0);
            if i % 4 == 0 {
                it.scale.x = -it.scale.x;
            }
            it.point = Vec3::random(&mut rng, -10.0, 10.0);
        }
        Decompose { items }
    }

    fn run(&mut self) -> u64 {
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for it in &self.items {
            let basis = Basis::from_rotation_scale(it.rotation, it.scale);
            let scale = basis.scale();
            let rotation = basis.rotation();
            let inv = basis.inverse();
            let rotated = rotation.rotate(it.point);
            let round_trip = inv * (basis * it.point);
            h = hash::add(h, bits(scale.x) as u64 | ((bits(scale.z) as u64) << 32));
            h = hash::add(h, bits(rotation.x) as u64 | ((bits(rotation.w) as u64) << 32));
            h = hash::add(h, bits(rotated.y) as u64 | ((bits(round_trip.z) as u64) << 32));
        }
        h
    }
}
