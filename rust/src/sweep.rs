//! Sort-and-sweep broadphase: 16384 boxes sorted on min x with the standard
//! library sort, then swept for overlapping pairs.
use crate::harness::Case;
use crate::hash;

const BOX_COUNT: u32 = 16384;

#[derive(Clone, Copy)]
pub struct Box3 {
    min: [f32; 3],
    max: [f32; 3],
}

pub struct Sweep {
    boxes: Vec<Box3>,
    order: Vec<u32>,
}

impl Case for Sweep {
    const NAME: &'static str = "sweep";

    fn init() -> Sweep {
        let mut rng = hash::Rng::new(0xb0c5);
        let mut boxes = vec![Box3 { min: [0.0; 3], max: [0.0; 3] }; BOX_COUNT as usize];
        for b in boxes.iter_mut() {
            for k in 0..3 {
                b.min[k] = rng.unit() * 200.0;
                b.max[k] = b.min[k] + 0.5 + rng.unit() * 3.0;
            }
        }
        Sweep { boxes, order: vec![0; BOX_COUNT as usize] }
    }

    fn run(&mut self) -> u64 {
        for (i, o) in self.order.iter_mut().enumerate() {
            *o = i as u32;
        }
        let boxes = &self.boxes;
        self.order.sort_unstable_by(|&a, &b| {
            let xa = boxes[a as usize].min[0];
            let xb = boxes[b as usize].min[0];
            if xa != xb { xa.partial_cmp(&xb).unwrap() } else { a.cmp(&b) }
        });

        let mut h = 0u64;
        let mut pairs = 0u64;
        for (i, &ia) in self.order.iter().enumerate() {
            let a = &self.boxes[ia as usize];
            for &ib in &self.order[i + 1..] {
                let b = &self.boxes[ib as usize];
                if b.min[0] > a.max[0] {
                    break;
                }
                if b.min[1] > a.max[1] || a.min[1] > b.max[1] {
                    continue;
                }
                if b.min[2] > a.max[2] || a.min[2] > b.max[2] {
                    continue;
                }
                h = hash::add(h, ((ia as u64) << 32) | ib as u64);
                pairs += 1;
            }
        }
        hash::add(h, pairs)
    }
}
