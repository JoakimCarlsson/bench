//! Island partition: union-find over 524288 contacts between 65536 bodies,
//! joining only pairs where both are dynamic, then labelling every dynamic
//! body by the lowest slot in its island.
use crate::harness::Case;
use crate::hash;

const BODY_COUNT: u32 = 65536;
const CONTACT_COUNT: usize = 524288;

#[derive(Clone, Copy)]
pub struct Pair {
    a: u32,
    b: u32,
}

pub struct Islands {
    dynamic: Vec<u8>,
    contacts: Vec<Pair>,
    parent: Vec<u32>,
}

fn find(parent: &mut [u32], start: u32) -> u32 {
    let mut i = start;
    while parent[i as usize] != i {
        parent[i as usize] = parent[parent[i as usize] as usize];
        i = parent[i as usize];
    }
    i
}

impl Case for Islands {
    const NAME: &'static str = "islands";

    fn init() -> Islands {
        let dynamic = (0..BODY_COUNT as usize)
            .map(|i| u8::from(hash::mix64(i as u64 ^ 0x15) & 7 != 0))
            .collect();
        let mut rng = hash::Rng::new(0x151a);
        let contacts = (0..CONTACT_COUNT)
            .map(|_| {
                let a = (rng.next() % BODY_COUNT as u64) as u32;
                let mut b = (rng.next() % BODY_COUNT as u64) as u32;
                if b == a {
                    b = (a + 1) % BODY_COUNT;
                }
                Pair { a, b }
            })
            .collect();
        Islands { dynamic, contacts, parent: vec![0; BODY_COUNT as usize] }
    }

    fn run(&mut self) -> u64 {
        for (i, p) in self.parent.iter_mut().enumerate() {
            *p = i as u32;
        }
        for c in self.contacts.iter() {
            if self.dynamic[c.a as usize] == 0 || self.dynamic[c.b as usize] == 0 {
                continue;
            }
            let ra = find(&mut self.parent, c.a);
            let rb = find(&mut self.parent, c.b);
            if ra == rb {
                continue;
            }
            if ra < rb {
                self.parent[rb as usize] = ra;
            } else {
                self.parent[ra as usize] = rb;
            }
        }
        let mut h = 0u64;
        let mut islands = 0u64;
        for i in 0..BODY_COUNT {
            if self.dynamic[i as usize] == 0 {
                continue;
            }
            let root = find(&mut self.parent, i);
            if root == i {
                islands += 1;
            }
            h = hash::add(h, ((i as u64) << 32) | root as u64);
        }
        hash::add(h, islands)
    }
}
