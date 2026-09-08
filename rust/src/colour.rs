//! Graph colouring: assign each of 524288 contacts the lowest colour not used
//! by another contact on either of its dynamic bodies, so every colour is a
//! set of contacts a solver can run in parallel. Bitmask per body, 31 colours
//! plus an overflow bucket.
use crate::harness::Case;
use crate::hash;

const BODY_COUNT: u32 = 65536;
const CONTACT_COUNT: usize = 524288;
const OVERFLOW: u32 = 31;

#[derive(Clone, Copy)]
pub struct Pair {
    a: u32,
    b: u32,
}

pub struct Colour {
    dynamic: Vec<u8>,
    contacts: Vec<Pair>,
    used: Vec<u32>,
    colour: Vec<u8>,
}

impl Case for Colour {
    const NAME: &'static str = "colour";

    fn init() -> Colour {
        let dynamic = (0..BODY_COUNT as usize)
            .map(|i| u8::from(hash::mix64(i as u64 ^ 0xc0) & 7 != 0))
            .collect();
        let mut rng = hash::Rng::new(0xc01c);
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
        Colour {
            dynamic,
            contacts,
            used: vec![0; BODY_COUNT as usize],
            colour: vec![0; CONTACT_COUNT],
        }
    }

    fn run(&mut self) -> u64 {
        self.used.fill(0);
        let mut highest = 0u32;
        for (p, out) in self.contacts.iter().zip(self.colour.iter_mut()) {
            let dyn_a = self.dynamic[p.a as usize] != 0;
            let dyn_b = self.dynamic[p.b as usize] != 0;
            let mask = if dyn_a { self.used[p.a as usize] } else { 0 }
                | if dyn_b { self.used[p.b as usize] } else { 0 };
            let mut c = (!mask).trailing_zeros();
            if c >= OVERFLOW {
                c = OVERFLOW;
            }
            if dyn_a {
                self.used[p.a as usize] |= 1u32 << c;
            }
            if dyn_b {
                self.used[p.b as usize] |= 1u32 << c;
            }
            *out = c as u8;
            if c > highest {
                highest = c;
            }
        }
        let mut h = 0u64;
        for chunk in self.colour.chunks_exact(8) {
            let mut word = 0u64;
            for (k, &c) in chunk.iter().enumerate() {
                word |= (c as u64) << (k * 8);
            }
            h = hash::add(h, word);
        }
        hash::add(h, highest as u64)
    }
}
