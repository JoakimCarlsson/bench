//! Scene-graph propagation: 8 frames over a 4-ary tree of 65536 nodes. Each
//! node advances its rotation, builds a scaled local transform, composes it
//! with its parent's world transform, and multiplies the result into a
//! model-view-projection matrix.
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{Basis, Mat4, Quat, Transform as Xform, Vec3};

const NODE_COUNT: usize = 65536;
const FRAMES: usize = 8;

#[derive(Clone, Copy, Default)]
struct Node {
    position: Vec3,
    scale: Vec3,
    spin: Vec3,
    parent: u32,
}

pub struct Transform {
    nodes: Vec<Node>,
    initial: Vec<Quat>,
    rotations: Vec<Quat>,
    world: Vec<Xform>,
    mvp: Vec<Mat4>,
    view_projection: Mat4,
}

impl Transform {
    /// Advance every node one frame; parents come before their children.
    fn frame(&mut self) {
        for i in 0..NODE_COUNT {
            let n = self.nodes[i];
            self.rotations[i] = self.rotations[i].integrate(n.spin);
            let local = Xform { basis: Basis::from_rotation_scale(self.rotations[i], n.scale), origin: n.position };
            self.world[i] = if i == 0 { local } else { self.world[n.parent as usize] * local };
            self.mvp[i] = self.view_projection * self.world[i].to_mat4();
        }
    }
}

impl Case for Transform {
    const NAME: &'static str = "transform";

    fn init() -> Transform {
        let mut rng = hash::Rng::new(0x7f0e);
        let mut nodes = vec![Node::default(); NODE_COUNT];
        let mut initial = vec![Quat::default(); NODE_COUNT];
        for (i, (n, q)) in nodes.iter_mut().zip(initial.iter_mut()).enumerate() {
            n.position = Vec3::random(&mut rng, -2.0, 2.0);
            n.scale = Vec3::random(&mut rng, 0.9, 1.1);
            n.spin = Vec3::random(&mut rng, -0.05, 0.05);
            n.parent = if i == 0 { 0 } else { ((i - 1) / 4) as u32 };
            *q = Quat::random(&mut rng);
        }
        let eye = Xform::looking_at(Vec3::new(40.0, 30.0, 40.0), Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let view = eye.inverse_orthonormal().to_mat4();
        Transform {
            nodes,
            initial,
            rotations: vec![Quat::default(); NODE_COUNT],
            world: vec![Xform::default(); NODE_COUNT],
            mvp: vec![Mat4::default(); NODE_COUNT],
            view_projection: Mat4::perspective(0.75, 16.0 / 9.0, 0.05, 4000.0) * view,
        }
    }

    fn run(&mut self) -> u64 {
        self.rotations.copy_from_slice(&self.initial);
        for _ in 0..FRAMES {
            self.frame();
        }
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for mvp in &self.mvp {
            let m = &mvp.m;
            h = hash::add(h, bits(m[0]) as u64 | ((bits(m[5]) as u64) << 32));
            h = hash::add(h, bits(m[12]) as u64 | ((bits(m[13]) as u64) << 32));
            h = hash::add(h, bits(m[14]) as u64 | ((bits(m[15]) as u64) << 32));
        }
        h
    }
}
