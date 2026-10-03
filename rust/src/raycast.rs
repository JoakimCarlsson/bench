//! Closest-hit ray casts: 1024 rays, each tested against 1024 oriented boxes
//! with the slab test in box space, shrinking the search to the closest hit
//! so far.
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{Basis, BoxPose, Quat, Vec3};

const BOX_COUNT: usize = 1024;
const RAY_COUNT: usize = 1024;

#[derive(Clone, Copy, Default)]
struct Ray {
    origin: Vec3,
    translation: Vec3,
}

#[derive(Clone, Copy, Default)]
struct Hit {
    point: Vec3,
    normal: Vec3,
    fraction: f32,
}

pub struct Raycast {
    boxes: Vec<BoxPose>,
    rays: Vec<Ray>,
}

/// Slab test of a ray against an oriented box, in the box's frame. A hit
/// needs an entry fraction in (0, max_fraction].
fn ray_cast_box(ray: &Ray, b: &BoxPose, max_fraction: f32) -> Option<Hit> {
    let inverse_basis = b.basis.transposed();
    let origin = inverse_basis * (ray.origin - b.center);
    let translation = inverse_basis * ray.translation;
    let origins = [origin.x, origin.y, origin.z];
    let translations = [translation.x, translation.y, translation.z];
    let extents = [b.half_extents.x, b.half_extents.y, b.half_extents.z];
    let mut entry_fraction = 0.0f32;
    let mut exit_fraction = max_fraction;
    let mut entry_axis: Option<usize> = None;
    let mut entry_sign = 0.0f32;

    for axis in 0..3 {
        let origin_axis = origins[axis];
        let translation_axis = translations[axis];
        let extent = extents[axis];
        if translation_axis.abs() <= 1e-8 {
            if origin_axis < -extent || origin_axis > extent {
                return None;
            }
            continue;
        }
        let mut first = (-extent - origin_axis) / translation_axis;
        let mut last = (extent - origin_axis) / translation_axis;
        let mut normal_sign = -1.0f32;
        if first > last {
            std::mem::swap(&mut first, &mut last);
            normal_sign = 1.0;
        }
        if first > entry_fraction {
            entry_fraction = first;
            entry_axis = Some(axis);
            entry_sign = normal_sign;
        }
        exit_fraction = exit_fraction.min(last);
        if entry_fraction > exit_fraction {
            return None;
        }
    }

    let axis = entry_axis?;
    if entry_fraction <= 0.0 || entry_fraction > max_fraction {
        return None;
    }
    let mut local_normal = Vec3::default();
    match axis {
        0 => local_normal.x = entry_sign,
        1 => local_normal.y = entry_sign,
        _ => local_normal.z = entry_sign,
    }
    Some(Hit {
        point: ray.origin + ray.translation * entry_fraction,
        normal: b.basis * local_normal,
        fraction: entry_fraction,
    })
}

impl Case for Raycast {
    const NAME: &'static str = "raycast";

    fn init() -> Raycast {
        let mut rng = hash::Rng::new(0x7a1c);
        let mut boxes = vec![BoxPose::default(); BOX_COUNT];
        for b in boxes.iter_mut() {
            b.half_extents = Vec3::random(&mut rng, 0.25, 2.0);
            b.center = Vec3::random(&mut rng, 0.0, 64.0);
            b.basis = Basis::from_quat(Quat::random(&mut rng));
        }
        let mut rays = vec![Ray::default(); RAY_COUNT];
        for r in rays.iter_mut() {
            r.origin = Vec3::random(&mut rng, -8.0, 72.0);
            r.translation = Vec3::random(&mut rng, -1.0, 1.0) * 80.0;
        }
        Raycast { boxes, rays }
    }

    fn run(&mut self) -> u64 {
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for ray in &self.rays {
            let mut closest = Hit { fraction: 1.0, ..Hit::default() };
            let mut closest_box = u32::MAX;
            for (j, b) in self.boxes.iter().enumerate() {
                if let Some(hit) = ray_cast_box(ray, b, closest.fraction) {
                    closest = hit;
                    closest_box = j as u32;
                }
            }
            h = hash::add(h, bits(closest.fraction) as u64 | ((closest_box as u64) << 32));
            h = hash::add(h, bits(closest.point.x) as u64 | ((bits(closest.normal.y) as u64) << 32));
        }
        h
    }
}
