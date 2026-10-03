//! The engine's eight-lane contact solver, single-threaded: 4 steps of a
//! 32x32 field of 8-box stacks, 8192 bodies and about 12k contacts coloured
//! by the engine's constraint graph. Each step prepares constraints, packs
//! each colour into eight-lane bundles, runs 4 substeps of warm start, soft
//! biased solve, position integration and relaxed solve with friction, then
//! restitution, and writes impulses and poses back.
use crate::constraint_graph::{ConstraintGraph, GRAPH_COLOR_COUNT, GraphBodies};
use crate::contact::{Contact, ShapeRef};
use crate::contact_solver::{BodyDelta, ContactSolver, KinematicMotion, SolverContext, SolverInputs};
use crate::harness::Case;
use crate::hash;
use crate::rigid_body::RigidBody;
use crate::vecmath::{Basis, Quat, Vec3};

const GRID: u32 = 32;
const HEIGHT: u32 = 8;
const BODY_COUNT: usize = (GRID * GRID * HEIGHT) as usize;
const STEPS: usize = 4;

pub struct Wide {
    context: SolverContext,
    initial_bodies: Vec<RigidBody>,
    initial_contacts: Vec<Contact>,
    bodies: Vec<RigidBody>,
    contacts: Vec<Contact>,
    graph: ConstraintGraph,
    body_local: Vec<u32>,
    active: Vec<u32>,
    static_motions: Vec<KinematicMotion>,
    deltas: Vec<BodyDelta>,
    solver: ContactSolver,
}

/// Appends a contact of body `a` against `b` with the scene's material.
fn add_contact(contacts: &mut Vec<Contact>, a: u32, b: ShapeRef, normal: Vec3) -> &mut Contact {
    let index = contacts.len();
    let mut contact = Contact {
        shape_a: ShapeRef { body: a, is_static: false },
        shape_b: b,
        alive: true,
        friction: 0.6,
        restitution: if index % 8 == 0 { 0.3 } else { 0.0 },
        rolling_resistance: if index % 5 == 0 { 0.05 } else { 0.0 },
        ..Contact::default()
    };
    contact.manifold.normal = normal;
    contacts.push(contact);
    &mut contacts[index]
}

/// Appends a manifold point with a random separation.
fn add_point(contact: &mut Contact, rng: &mut hash::Rng, point: Vec3) {
    let manifold = &mut contact.manifold;
    let p = &mut manifold.points[manifold.point_count as usize];
    manifold.point_count += 1;
    p.point = point;
    p.separation = rng.unit() * 0.025 - 0.02;
}

/// A tilted box of a stack, resting at its place in the grid.
fn stack_body(rng: &mut hash::Rng, cx: u32, cz: u32, level: u32, index: u32) -> RigidBody {
    let mut b = RigidBody::default();
    let half = Vec3::random(rng, 0.4, 0.5);
    let x = rng.unit() * 0.1 - 0.05;
    let y = rng.unit() * 0.2 - 0.1;
    let z = rng.unit() * 0.1 - 0.05;
    b.rotation = Quat { x, y, z, w: 1.0 }.normalize();
    b.transform.basis = Basis::from_quat(b.rotation);
    b.transform.origin = Vec3::new(cx as f32 * 1.1, level as f32 * 1.0 + 0.5, cz as f32 * 1.1);
    b.set_box_mass(half);
    b.linear_velocity = Vec3::random(rng, -0.1, 0.1);
    if index % 16 == 0 {
        b.linear_velocity.y = -2.0;
    }
    b.angular_velocity = Vec3::random(rng, -0.1, 0.1);
    b.angular_damp = 0.05;
    b
}

/// Contacts of every box with the one below it and, at random, with its
/// neighbours in x and z.
fn stack_contacts(bodies: &[RigidBody], rng: &mut hash::Rng) -> Vec<Contact> {
    let corners = [(1.0f32, 1.0f32), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
    let half = Vec3::new(0.45, 0.45, 0.45);
    let mut contacts = Vec::new();
    for cz in 0..GRID {
        for cx in 0..GRID {
            for level in 0..HEIGHT {
                let i = (cz * GRID + cx) * HEIGHT + level;
                let center = bodies[i as usize].transform.origin;
                let below = if level == 0 { ShapeRef { body: 0, is_static: true } } else { ShapeRef { body: i - 1, is_static: false } };
                let down = add_contact(&mut contacts, i, below, Vec3::new(0.0, -1.0, 0.0));
                for (sx, sz) in corners {
                    add_point(down, rng, center + Vec3::new(sx * half.x, -half.y, sz * half.z));
                }
                if cx + 1 < GRID && rng.next() % 4 == 0 {
                    let side = add_contact(&mut contacts, i, ShapeRef { body: i + HEIGHT, is_static: false }, Vec3::new(1.0, 0.0, 0.0));
                    add_point(side, rng, center + Vec3::new(half.x, 0.5 * half.y, 0.0));
                    add_point(side, rng, center + Vec3::new(half.x, -0.5 * half.y, 0.0));
                }
                if cz + 1 < GRID && rng.next() % 4 == 0 {
                    let side = add_contact(&mut contacts, i, ShapeRef { body: i + GRID * HEIGHT, is_static: false }, Vec3::new(0.0, 0.0, 1.0));
                    add_point(side, rng, center + Vec3::new(0.0, 0.5 * half.y, half.z));
                    add_point(side, rng, center + Vec3::new(0.0, -0.5 * half.y, half.z));
                }
            }
        }
    }
    contacts
}

impl Case for Wide {
    const NAME: &'static str = "wide";

    /// Builds the stacks, their contacts and the colouring.
    fn init() -> Wide {
        let mut rng = hash::Rng::new(0x501e);
        let mut initial_bodies = Vec::with_capacity(BODY_COUNT);
        for cz in 0..GRID {
            for cx in 0..GRID {
                for level in 0..HEIGHT {
                    let i = (cz * GRID + cx) * HEIGHT + level;
                    initial_bodies.push(stack_body(&mut rng, cx, cz, level, i));
                }
            }
        }
        let mut initial_contacts = stack_contacts(&initial_bodies, &mut rng);

        let mut graph = ConstraintGraph::default();
        graph.reserve_bodies(BODY_COUNT);
        for (index, contact) in initial_contacts.iter_mut().enumerate() {
            let b_static = contact.shape_b.is_static;
            let slot = graph.add(index as u32, &GraphBodies { a: contact.shape_a.body, b: if b_static { 0 } else { contact.shape_b.body }, b_is_static: b_static });
            contact.color = slot.color;
            contact.local = slot.local;
        }

        Wide {
            context: SolverContext::engine_default(),
            bodies: initial_bodies.clone(),
            contacts: initial_contacts.clone(),
            initial_bodies,
            initial_contacts,
            graph,
            body_local: (0..BODY_COUNT as u32).collect(),
            active: (0..BODY_COUNT as u32).collect(),
            static_motions: vec![KinematicMotion::default()],
            deltas: vec![BodyDelta::default(); BODY_COUNT],
            solver: ContactSolver::default(),
        }
    }

    /// Restores the scene and solves it for every step.
    fn run(&mut self) -> u64 {
        self.bodies.copy_from_slice(&self.initial_bodies);
        self.contacts.copy_from_slice(&self.initial_contacts);
        let graph = &self.graph;
        let mut inputs = SolverInputs {
            contacts: &mut self.contacts,
            colors: std::array::from_fn::<_, GRAPH_COLOR_COUNT, _>(|color| graph.contacts(color)),
            body_local: &self.body_local,
            active_bodies: &self.active,
            bodies: &mut self.bodies,
            static_motions: &self.static_motions,
            deltas: &mut self.deltas,
            context: self.context,
            pool: None,
        };
        for _ in 0..STEPS {
            self.solver.solve(&mut inputs);
        }
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for (b, delta) in self.bodies.iter().zip(self.deltas.iter()) {
            h = hash::add(h, bits(b.linear_velocity.x) as u64 | ((bits(b.linear_velocity.y) as u64) << 32));
            h = hash::add(h, bits(b.transform.origin.y) as u64 | ((bits(b.rotation.w) as u64) << 32));
            h = hash::add(h, bits(b.angular_velocity.z) as u64 | ((bits(delta.rotation.x) as u64) << 32));
        }
        for c in &self.contacts {
            for p in c.manifold.active() {
                h = hash::add(h, bits(p.normal_impulse) as u64 | ((bits(p.total_normal_impulse) as u64) << 32));
                h = hash::add(h, bits(p.peak_normal_impulse) as u64 | ((bits(p.relative_velocity) as u64) << 32));
            }
            let f = &c.friction_impulses;
            h = hash::add(h, bits(f.tangent_x) as u64 | ((bits(f.tangent_y) as u64) << 32));
            h = hash::add(h, bits(f.twist) as u64 | ((bits(f.rolling.y) as u64) << 32));
        }
        h
    }
}
