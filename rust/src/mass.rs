//! Mass properties: for 128 bodies of 32^3 voxels with per-material density,
//! accumulate mass, centre of mass and the inertia tensor voxel by voxel, then
//! shift the tensor to the centre of mass.
use crate::harness::Case;
use crate::hash;

const BODY_COUNT: usize = 128;
const DIM: usize = 32;
const VOXELS: usize = DIM * DIM * DIM;

const VOXEL_SIZE: f32 = 0.1;
const DENSITY: [f32; 16] = [
    0.0, 2400.0, 700.0, 7800.0, 1600.0, 2500.0, 900.0, 1200.0, 1800.0, 8900.0, 500.0, 1500.0,
    2700.0, 300.0, 1100.0, 2000.0,
];

pub struct Props {
    mass: f32,
    com: [f32; 3],
    inertia: [f32; 6],
}

pub struct Mass {
    mat: Vec<u8>,
}

fn properties(mat: &[u8]) -> Props {
    let mut m = 0.0f32;
    let mut cx = 0.0f32;
    let mut cy = 0.0f32;
    let mut cz = 0.0f32;
    let mut ixx = 0.0f32;
    let mut iyy = 0.0f32;
    let mut izz = 0.0f32;
    let mut ixy = 0.0f32;
    let mut ixz = 0.0f32;
    let mut iyz = 0.0f32;
    let cube = VOXEL_SIZE * VOXEL_SIZE / 6.0;
    for z in 0..DIM {
        for y in 0..DIM {
            for x in 0..DIM {
                let id = mat[(z * DIM + y) * DIM + x];
                if id == 0 {
                    continue;
                }
                let dm = DENSITY[id as usize] * (VOXEL_SIZE * VOXEL_SIZE * VOXEL_SIZE);
                let px = (x as f32 + 0.5) * VOXEL_SIZE;
                let py = (y as f32 + 0.5) * VOXEL_SIZE;
                let pz = (z as f32 + 0.5) * VOXEL_SIZE;
                m += dm;
                cx += dm * px;
                cy += dm * py;
                cz += dm * pz;
                ixx += dm * (py * py + pz * pz + cube);
                iyy += dm * (px * px + pz * pz + cube);
                izz += dm * (px * px + py * py + cube);
                ixy -= dm * px * py;
                ixz -= dm * px * pz;
                iyz -= dm * py * pz;
            }
        }
    }
    let inv = if m > 0.0 { 1.0 / m } else { 0.0 };
    let ox = cx * inv;
    let oy = cy * inv;
    let oz = cz * inv;
    Props {
        mass: m,
        com: [ox, oy, oz],
        inertia: [
            ixx - m * (oy * oy + oz * oz),
            iyy - m * (ox * ox + oz * oz),
            izz - m * (ox * ox + oy * oy),
            ixy + m * ox * oy,
            ixz + m * ox * oz,
            iyz + m * oy * oz,
        ],
    }
}

impl Case for Mass {
    const NAME: &'static str = "mass";

    fn init() -> Mass {
        let mat = (0..BODY_COUNT * VOXELS)
            .map(|i| {
                let r = hash::mix64(i as u64 ^ 0x3a55);
                if r & 3 == 0 { 0 } else { ((r >> 2) & 15) as u8 }
            })
            .collect();
        Mass { mat }
    }

    fn run(&mut self) -> u64 {
        let mut h = 0u64;
        for b in 0..BODY_COUNT {
            let p = properties(&self.mat[b * VOXELS..][..VOXELS]);
            let bits = hash::f32_bits;
            h = hash::add(h, bits(p.mass) as u64 | ((bits(p.com[0]) as u64) << 32));
            h = hash::add(h, bits(p.com[1]) as u64 | ((bits(p.com[2]) as u64) << 32));
            h = hash::add(h, bits(p.inertia[0]) as u64 | ((bits(p.inertia[1]) as u64) << 32));
            h = hash::add(h, bits(p.inertia[2]) as u64 | ((bits(p.inertia[3]) as u64) << 32));
            h = hash::add(h, bits(p.inertia[4]) as u64 | ((bits(p.inertia[5]) as u64) << 32));
        }
        h
    }
}
