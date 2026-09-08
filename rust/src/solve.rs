//! Contact solver: 8 frames of substepped sequential impulses over 4096
//! bodies and 16384 contacts, with accumulated-impulse clamping. Float math
//! through indirect body indices, the shape of a rigid-body solve loop.
use crate::harness::Case;
use crate::hash;

const BODY_COUNT: usize = 4096;
const CONTACT_COUNT: usize = 16384;
const SUBSTEPS: usize = 8;
const ITERS: usize = 4;
const FRAMES: usize = 8;

#[derive(Clone, Copy, Default)]
pub struct Body {
    px: f32,
    py: f32,
    pz: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    inv_mass: f32,
}

#[derive(Clone, Copy, Default)]
pub struct Contact {
    a: u32,
    b: u32,
    nx: f32,
    ny: f32,
    nz: f32,
    depth: f32,
    impulse: f32,
}

pub struct Solve {
    bodies: Vec<Body>,
    initial: Vec<Body>,
    contacts: Vec<Contact>,
}

fn integrate_gravity(bodies: &mut [Body], h: f32) {
    for b in bodies.iter_mut() {
        if b.inv_mass > 0.0 {
            b.vy += -9.81 * h;
        }
    }
}

/// `a` and `b` are always distinct, so reading both velocities and writing
/// both back is the same sequence of operations as updating them in place.
fn solve_contacts(bodies: &mut [Body], contacts: &mut [Contact], h: f32) {
    for c in contacts.iter_mut() {
        let (ia, ib) = (c.a as usize, c.b as usize);
        let a = bodies[ia];
        let b = bodies[ib];
        let k = a.inv_mass + b.inv_mass;
        if k == 0.0 {
            continue;
        }
        let rvx = b.vx - a.vx;
        let rvy = b.vy - a.vy;
        let rvz = b.vz - a.vz;
        let vn = rvx * c.nx + rvy * c.ny + rvz * c.nz;
        let bias = c.depth * 0.2 / h;
        let mut lambda = (-vn + bias) / k;
        let mut acc = c.impulse + lambda;
        if acc < 0.0 {
            acc = 0.0;
        }
        lambda = acc - c.impulse;
        c.impulse = acc;
        bodies[ia].vx = a.vx - c.nx * lambda * a.inv_mass;
        bodies[ia].vy = a.vy - c.ny * lambda * a.inv_mass;
        bodies[ia].vz = a.vz - c.nz * lambda * a.inv_mass;
        bodies[ib].vx = b.vx + c.nx * lambda * b.inv_mass;
        bodies[ib].vy = b.vy + c.ny * lambda * b.inv_mass;
        bodies[ib].vz = b.vz + c.nz * lambda * b.inv_mass;
    }
}

fn integrate_positions(bodies: &mut [Body], h: f32) {
    for b in bodies.iter_mut() {
        b.px += b.vx * h;
        b.py += b.vy * h;
        b.pz += b.vz * h;
    }
}

fn checksum(bodies: &[Body]) -> u64 {
    let bits = hash::f32_bits;
    let mut h = 0u64;
    for b in bodies {
        h = hash::add(h, bits(b.px) as u64 | ((bits(b.vx) as u64) << 32));
        h = hash::add(h, bits(b.py) as u64 | ((bits(b.vy) as u64) << 32));
        h = hash::add(h, bits(b.pz) as u64 | ((bits(b.vz) as u64) << 32));
    }
    h
}

impl Case for Solve {
    const NAME: &'static str = "solve";

    fn init() -> Solve {
        let mut rng = hash::Rng::new(0xb0d1e5);
        let mut initial = vec![Body::default(); BODY_COUNT];
        for (i, b) in initial.iter_mut().enumerate() {
            b.px = rng.unit() * 64.0;
            b.py = rng.unit() * 64.0;
            b.pz = rng.unit() * 64.0;
            b.vx = rng.unit() * 2.0 - 1.0;
            b.vy = rng.unit() * 2.0 - 1.0;
            b.vz = rng.unit() * 2.0 - 1.0;
            b.inv_mass = if i % 8 == 0 { 0.0 } else { 1.0 / (0.5 + rng.unit() * 2.0) };
        }
        let mut contacts = vec![Contact::default(); CONTACT_COUNT];
        for c in contacts.iter_mut() {
            c.a = (rng.next() % BODY_COUNT as u64) as u32;
            c.b = (rng.next() % BODY_COUNT as u64) as u32;
            if c.b == c.a {
                c.b = (c.a + 1) % BODY_COUNT as u32;
            }
            let mut nx = rng.unit() * 2.0 - 1.0;
            let mut ny = rng.unit() * 2.0 - 1.0;
            let mut nz = rng.unit() * 2.0 - 1.0;
            let mut len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len < 1e-3 {
                nx = 0.0;
                ny = 1.0;
                nz = 0.0;
                len = 1.0;
            }
            c.nx = nx / len;
            c.ny = ny / len;
            c.nz = nz / len;
            c.depth = rng.unit() * 0.05;
            c.impulse = 0.0;
        }
        Solve { bodies: vec![Body::default(); BODY_COUNT], initial, contacts }
    }

    fn run(&mut self) -> u64 {
        self.bodies.copy_from_slice(&self.initial);
        for c in self.contacts.iter_mut() {
            c.impulse = 0.0;
        }
        let h = 1.0f32 / 60.0f32 / SUBSTEPS as f32;
        for _ in 0..FRAMES {
            for _ in 0..SUBSTEPS {
                integrate_gravity(&mut self.bodies, h);
                for _ in 0..ITERS {
                    solve_contacts(&mut self.bodies, &mut self.contacts, h);
                }
                integrate_positions(&mut self.bodies, h);
            }
        }
        checksum(&self.bodies)
    }
}
