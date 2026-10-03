//! Particle gas: 4 substeps of the engine's gas solver over 8192 particles
//! stirred by 4 moving boxes. Each substep hashes particles into cells,
//! sorts the keys, searches the 27 neighbouring cells with a binary search
//! for pressure and viscosity, pushes particles out of and along the movers,
//! then integrates positions.
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{self, Basis, Quat, Transform, Vec3};

const PARTICLE_COUNT: usize = 8192;
const MOVER_COUNT: usize = 4;
const SUBSTEPS: usize = 4;
const GOLDEN: u32 = 0x9E37_79B9;
const GAS_KEY_OFFSET: i64 = 1 << 20;
const GAS_KEY_MASK: u64 = (1 << 21) - 1;
const MOVER_REACH: f32 = 1.5;
const MOVER_WAKE_RATE: f32 = 6.0;
const SPACING: f32 = 0.8;
const PRESSURE: f32 = 1.5;
const VISCOSITY: f32 = 0.5;
const STIR_STRENGTH: f32 = 1.0;
const STEP: f32 = 1.0 / 120.0;
const RISE: f32 = 0.5;
const EXTENT: f32 = 12.0;

#[derive(Clone, Copy, Default)]
struct Particle {
    position: Vec3,
    velocity: Vec3,
    size: f32,
    seed: u32,
}

#[derive(Clone, Copy, Default)]
struct Mover {
    transform: Transform,
    half_extents: Vec3,
    velocity: Vec3,
    angular_velocity: Vec3,
}

/// A particle index keyed by its spatial hash cell.
#[derive(Clone, Copy)]
struct Entry {
    key: u64,
    index: u32,
}

pub struct Gas {
    initial: Vec<Particle>,
    particles: Vec<Particle>,
    movers: [Mover; MOVER_COUNT],
    entries: Vec<Entry>,
    push: Vec<Vec3>,
    blend: Vec<Vec3>,
    weight: Vec<f32>,
}

/// The engine's 32-bit bit mixer.
fn mix_bits(mut value: u32) -> u32 {
    value ^= value >> 16;
    value = value.wrapping_mul(0x7FEB_352D);
    value ^= value >> 15;
    value = value.wrapping_mul(0x846C_A68B);
    value ^= value >> 16;
    value
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
fn unit_random(seed: u32, salt: u32) -> f32 {
    (mix_bits(seed ^ mix_bits(salt.wrapping_add(GOLDEN))) >> 8) as f32 / 16777216.0
}

/// Unit vector along `value`, or `fallback` when it is too short.
fn safe_normalize(value: Vec3, fallback: Vec3) -> Vec3 {
    let size = value.length();
    if size > 1e-6 { value / size } else { fallback }
}

/// Packs three cell coordinates into one 63-bit key.
fn gas_key(x: i64, y: i64, z: i64) -> u64 {
    let pack = |value: i64| (value + GAS_KEY_OFFSET) as u64 & GAS_KEY_MASK;
    pack(x) | (pack(y) << 21) | (pack(z) << 42)
}

/// Integer cell containing `position` for cell size `reach`.
fn gas_cell(position: Vec3, reach: f32) -> [i64; 3] {
    [(position.x / reach).floor() as i64, (position.y / reach).floor() as i64, (position.z / reach).floor() as i64]
}

/// Unit direction pushing `body` away from `other`; a seeded random one when
/// they coincide.
fn separation(body: &Particle, other: &Particle, offset: Vec3) -> Vec3 {
    let size = offset.length();
    if size > 1e-6 {
        return offset / size;
    }
    let mixed = mix_bits(body.seed ^ mix_bits(other.seed));
    let x = unit_random(mixed, 1) - 0.5;
    let y = unit_random(mixed, 2) - 0.5;
    let z = unit_random(mixed, 3) - 0.5;
    safe_normalize(Vec3::new(x, y, z), Vec3::new(0.0, 1.0, 0.0))
}

/// Sign of a coordinate as a unit, positive for zero.
fn unit_sign(value: f32) -> f32 {
    if value < 0.0 { -1.0 } else { 1.0 }
}

/// Outward face normal of the box face nearest to an inside point.
fn nearest_face_normal(local: Vec3, extent: Vec3) -> Vec3 {
    let depth = extent - local.abs();
    if depth.x <= depth.y && depth.x <= depth.z {
        Vec3::new(unit_sign(local.x), 0.0, 0.0)
    } else if depth.y <= depth.z {
        Vec3::new(0.0, unit_sign(local.y), 0.0)
    } else {
        Vec3::new(0.0, 0.0, unit_sign(local.z))
    }
}

/// Pushes a particle with a moving box, adding surface velocity and moving it
/// out when it penetrates.
fn stir(body: &mut Particle, mover: &Mover) {
    let rotation = Basis {
        x: safe_normalize(mover.transform.basis.x, Vec3::new(1.0, 0.0, 0.0)),
        y: safe_normalize(mover.transform.basis.y, Vec3::new(0.0, 1.0, 0.0)),
        z: safe_normalize(mover.transform.basis.z, Vec3::new(0.0, 0.0, 1.0)),
    };
    let local = rotation.transposed() * (body.position - mover.transform.origin);
    let extent = mover.half_extents;
    let closest = Vec3::new(vecmath::clamp(local.x, -extent.x, extent.x), vecmath::clamp(local.y, -extent.y, extent.y), vecmath::clamp(local.z, -extent.z, extent.z));
    let radius = body.size * 0.5;
    let reach = vecmath::max3(extent.x, extent.y, extent.z) * MOVER_REACH + radius;
    let outside = local - closest;
    let distance = outside.length();
    if distance > reach {
        return;
    }
    let arm = rotation * closest;
    let surface_velocity = mover.velocity + mover.angular_velocity.cross(arm);
    let weight = 1.0 - (distance / reach);
    let wake = vecmath::clamp(STIR_STRENGTH * weight * STEP * MOVER_WAKE_RATE, 0.0, 1.0);
    body.velocity = body.velocity + (surface_velocity - body.velocity) * wake;
    if distance >= radius {
        return;
    }
    let normal = if distance > 1e-6 { outside / distance } else { nearest_face_normal(local, extent) };
    let face = if distance > 1e-6 {
        closest
    } else {
        Vec3::new(
            if normal.x != 0.0 { normal.x * extent.x } else { local.x },
            if normal.y != 0.0 { normal.y * extent.y } else { local.y },
            if normal.z != 0.0 { normal.z * extent.z } else { local.z },
        )
    };
    let world_normal = rotation * normal;
    body.position = mover.transform.origin + (rotation * face) + (world_normal * radius);
    let into = (body.velocity - surface_velocity).dot(world_normal);
    if into < 0.0 {
        body.velocity = body.velocity - (world_normal * (into * STIR_STRENGTH));
    }
}

impl Gas {
    /// Pairwise push, velocity blend and weight from overlapping particles.
    fn accumulate_pressure(&mut self) {
        let mut reach = 0.0f32;
        for body in &self.particles {
            reach = vecmath::max(reach, body.size * SPACING);
        }
        if reach <= 0.0 {
            return;
        }
        self.entries.clear();
        for (index, particle) in self.particles.iter().enumerate() {
            let [x, y, z] = gas_cell(particle.position, reach);
            self.entries.push(Entry { key: gas_key(x, y, z), index: index as u32 });
        }
        self.entries.sort_unstable_by_key(|entry| (entry.key, entry.index));
        let entries = &self.entries;
        for entry in entries {
            let me = entry.index as usize;
            let body = &self.particles[me];
            let cell = gas_cell(body.position, reach);
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let key = gas_key(cell[0] + dx, cell[1] + dy, cell[2] + dz);
                        let first = entries.partition_point(|e| e.key < key);
                        for next in entries[first..].iter().take_while(|e| e.key == key) {
                            if next.index == entry.index {
                                continue;
                            }
                            let other = &self.particles[next.index as usize];
                            let offset = body.position - other.position;
                            let range = 0.5 * (body.size + other.size) * SPACING;
                            let distance = offset.length();
                            if range <= 0.0 || distance >= range {
                                continue;
                            }
                            let overlap = 1.0 - (distance / range);
                            self.push[me] = self.push[me] + (separation(body, other, offset) * overlap);
                            self.blend[me] = self.blend[me] + ((other.velocity - body.velocity) * overlap);
                            self.weight[me] += overlap;
                        }
                    }
                }
            }
        }
    }

    /// Pressure, viscosity and stirring for every particle.
    fn resolve(&mut self) {
        self.push.fill(Vec3::default());
        self.blend.fill(Vec3::default());
        self.weight.fill(0.0);
        self.accumulate_pressure();
        for (index, body) in self.particles.iter_mut().enumerate() {
            body.velocity = body.velocity + (self.push[index] * (PRESSURE * STEP));
            if self.weight[index] > 0.0 {
                let blend = vecmath::clamp(VISCOSITY * STEP * 10.0, 0.0, 1.0);
                body.velocity = body.velocity + (self.blend[index] * (blend / self.weight[index]));
            }
            for mover in &self.movers {
                stir(body, mover);
            }
        }
    }
}

impl Case for Gas {
    const NAME: &'static str = "gas";

    /// Draws the particles and the movers.
    fn init() -> Gas {
        let mut rng = hash::Rng::new(0x6a5);
        let mut initial = vec![Particle::default(); PARTICLE_COUNT];
        for p in initial.iter_mut() {
            p.position = Vec3::random(&mut rng, 0.0, EXTENT);
            p.velocity = Vec3::random(&mut rng, -0.5, 0.5);
            p.size = 0.3 + rng.unit() * 0.3;
            p.seed = rng.next() as u32;
        }
        let mut movers = [Mover::default(); MOVER_COUNT];
        for m in movers.iter_mut() {
            m.transform.basis = Basis::from_quat(Quat::random(&mut rng));
            m.transform.origin = Vec3::random(&mut rng, 4.0, EXTENT - 4.0);
            m.half_extents = Vec3::random(&mut rng, 1.0, 2.0);
            m.velocity = Vec3::random(&mut rng, -3.0, 3.0);
            m.angular_velocity = Vec3::random(&mut rng, -1.0, 1.0);
        }
        Gas {
            particles: initial.clone(),
            initial,
            movers,
            entries: Vec::with_capacity(PARTICLE_COUNT),
            push: vec![Vec3::default(); PARTICLE_COUNT],
            blend: vec![Vec3::default(); PARTICLE_COUNT],
            weight: vec![0.0; PARTICLE_COUNT],
        }
    }

    /// Restores the particles and runs every substep.
    fn run(&mut self) -> u64 {
        self.particles.copy_from_slice(&self.initial);
        for _ in 0..SUBSTEPS {
            self.resolve();
            for p in self.particles.iter_mut() {
                p.velocity.y += RISE * STEP;
                p.position = p.position + p.velocity * STEP;
            }
        }
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for p in &self.particles {
            h = hash::add(h, bits(p.position.x) as u64 | ((bits(p.position.y) as u64) << 32));
            h = hash::add(h, bits(p.velocity.z) as u64 | ((bits(p.position.z) as u64) << 32));
        }
        h
    }
}
