//! Camera picking: for 131072 cameras build the view from a look-at, the
//! perspective projection and their product, invert it by cofactors, and
//! unproject four NDC points to world space and back.
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{Mat4, Transform, Vec3};

const CAMERA_COUNT: usize = 131072;

const NDC: [Vec3; 4] = [
    Vec3::new(-0.5, -0.5, -1.0),
    Vec3::new(0.5, -0.5, 0.0),
    Vec3::new(0.5, 0.5, 0.5),
    Vec3::new(-0.25, 0.75, 0.999),
];

#[derive(Clone, Copy, Default)]
struct Camera {
    eye: Vec3,
    target: Vec3,
    tan_half_fov: f32,
    aspect: f32,
    z_near: f32,
    z_far: f32,
}

impl Camera {
    /// View-projection matrix of the camera.
    fn view_projection(&self) -> Mat4 {
        let view = Transform::looking_at(self.eye, self.target, Vec3::new(0.0, 1.0, 0.0)).inverse_orthonormal().to_mat4();
        Mat4::perspective(self.tan_half_fov, self.aspect, self.z_near, self.z_far) * view
    }
}

pub struct Unproject {
    cameras: Vec<Camera>,
}

impl Case for Unproject {
    const NAME: &'static str = "unproject";

    fn init() -> Unproject {
        let mut rng = hash::Rng::new(0xca3e);
        let mut cameras = vec![Camera::default(); CAMERA_COUNT];
        for c in cameras.iter_mut() {
            c.eye = Vec3::random(&mut rng, -50.0, 50.0);
            c.target = c.eye + Vec3::random(&mut rng, -10.0, 10.0);
            c.tan_half_fov = 0.3 + rng.unit() * 1.0;
            c.aspect = 1.0 + rng.unit() * 1.0;
            c.z_near = 0.05 + rng.unit() * 0.45;
            c.z_far = 100.0 + rng.unit() * 3900.0;
        }
        Unproject { cameras }
    }

    fn run(&mut self) -> u64 {
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for c in &self.cameras {
            let vp = c.view_projection();
            let inv = vp.inverse();
            for p in &NDC {
                let world = inv.transform_point(*p);
                let back = vp.transform_point(world);
                h = hash::add(h, bits(world.x) as u64 | ((bits(world.y) as u64) << 32));
                h = hash::add(h, bits(world.z) as u64 | ((bits(back.x) as u64) << 32));
            }
        }
        h
    }
}
