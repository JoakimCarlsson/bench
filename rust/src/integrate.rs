//! Rigid-body integration: 32 steps over 65536 bodies, each with a position,
//! a velocity, an angular velocity and a quaternion that is renormalised every
//! step. Streaming float math with no indirection.
use crate::harness::Case;
use crate::hash;

const BODY_COUNT: usize = 65536;
const STEPS: usize = 32;

#[derive(Clone, Copy, Default)]
pub struct Body {
    px: f32,
    py: f32,
    pz: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    wx: f32,
    wy: f32,
    wz: f32,
    qx: f32,
    qy: f32,
    qz: f32,
    qw: f32,
}

pub struct Integrate {
    bodies: Vec<Body>,
    initial: Vec<Body>,
}

fn step(bodies: &mut [Body], dt: f32) {
    let half = 0.5 * dt;
    for b in bodies.iter_mut() {
        b.vy += -9.81 * dt;
        b.px += b.vx * dt;
        b.py += b.vy * dt;
        b.pz += b.vz * dt;
        b.wx *= 0.999;
        b.wy *= 0.999;
        b.wz *= 0.999;
        let qx = b.qx + half * (b.wx * b.qw + b.wy * b.qz - b.wz * b.qy);
        let qy = b.qy + half * (b.wy * b.qw + b.wz * b.qx - b.wx * b.qz);
        let qz = b.qz + half * (b.wz * b.qw + b.wx * b.qy - b.wy * b.qx);
        let qw = b.qw + half * (-b.wx * b.qx - b.wy * b.qy - b.wz * b.qz);
        let inv = 1.0 / (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
        b.qx = qx * inv;
        b.qy = qy * inv;
        b.qz = qz * inv;
        b.qw = qw * inv;
    }
}

impl Case for Integrate {
    const NAME: &'static str = "integrate";

    fn init() -> Integrate {
        let mut rng = hash::Rng::new(0x1a7e);
        let mut initial = vec![Body::default(); BODY_COUNT];
        for b in initial.iter_mut() {
            b.px = rng.unit() * 100.0;
            b.py = rng.unit() * 100.0;
            b.pz = rng.unit() * 100.0;
            b.vx = rng.unit() * 4.0 - 2.0;
            b.vy = rng.unit() * 4.0 - 2.0;
            b.vz = rng.unit() * 4.0 - 2.0;
            b.wx = rng.unit() * 2.0 - 1.0;
            b.wy = rng.unit() * 2.0 - 1.0;
            b.wz = rng.unit() * 2.0 - 1.0;
            b.qx = 0.0;
            b.qy = 0.0;
            b.qz = 0.0;
            b.qw = 1.0;
        }
        Integrate { bodies: vec![Body::default(); BODY_COUNT], initial }
    }

    fn run(&mut self) -> u64 {
        self.bodies.copy_from_slice(&self.initial);
        let dt = 1.0f32 / 60.0f32;
        for _ in 0..STEPS {
            step(&mut self.bodies, dt);
        }
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for b in &self.bodies {
            h = hash::add(h, bits(b.px) as u64 | ((bits(b.py) as u64) << 32));
            h = hash::add(h, bits(b.qx) as u64 | ((bits(b.qw) as u64) << 32));
        }
        h
    }
}
