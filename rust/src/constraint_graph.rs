//! The engine's constraint graph: colours contacts so that no two in a colour
//! share a dynamic body, keeping static contacts out of colour 0 and sending
//! what does not fit to the overflow colour.
use crate::contact::NULL_LINK;

pub const GRAPH_COLOR_COUNT: usize = 24;
pub const OVERFLOW_COLOR: usize = GRAPH_COLOR_COUNT - 1;
pub const DYNAMIC_COLOR_COUNT: usize = GRAPH_COLOR_COUNT - 4;

pub struct GraphBodies {
    pub a: u32,
    pub b: u32,
    pub b_is_static: bool,
}

#[derive(Clone, Copy)]
pub struct GraphSlot {
    pub color: u32,
    pub local: u32,
}

#[derive(Default)]
struct Color {
    bodies: Vec<u64>,
    contacts: Vec<u32>,
}

impl Color {
    /// Whether `body` is not yet in this colour.
    fn is_free(&self, body: u32) -> bool {
        self.bodies[body as usize / 64] & (1u64 << (body % 64)) == 0
    }

    /// Marks `body` as in this colour.
    fn take(&mut self, body: u32) {
        self.bodies[body as usize / 64] |= 1u64 << (body % 64);
    }

    /// Clears `body` from this colour.
    fn release(&mut self, body: u32) {
        self.bodies[body as usize / 64] &= !(1u64 << (body % 64));
    }
}

#[derive(Default)]
pub struct ConstraintGraph {
    colors: [Color; GRAPH_COLOR_COUNT],
    words: usize,
    size: usize,
}

impl ConstraintGraph {
    /// Grows every colour's body bitset to cover `body_count` bodies.
    pub fn reserve_bodies(&mut self, body_count: usize) {
        let words = body_count.div_ceil(64);
        if words <= self.words {
            return;
        }
        self.words = words;
        for color in &mut self.colors[..OVERFLOW_COLOR] {
            color.bodies.resize(words, 0);
        }
    }

    /// Adds a contact to the first colour free for its bodies.
    pub fn add(&mut self, contact: u32, bodies: &GraphBodies) -> GraphSlot {
        let mut chosen = OVERFLOW_COLOR;
        if bodies.b_is_static {
            for color in (1..OVERFLOW_COLOR).rev() {
                if self.colors[color].is_free(bodies.a) {
                    self.colors[color].take(bodies.a);
                    chosen = color;
                    break;
                }
            }
        } else {
            for color in 0..DYNAMIC_COLOR_COUNT {
                let target = &mut self.colors[color];
                if target.is_free(bodies.a) && target.is_free(bodies.b) {
                    target.take(bodies.a);
                    target.take(bodies.b);
                    chosen = color;
                    break;
                }
            }
        }
        let target = &mut self.colors[chosen];
        let local = target.contacts.len() as u32;
        target.contacts.push(contact);
        self.size += 1;
        GraphSlot { color: chosen as u32, local }
    }

    /// Removes a contact; returns the contact moved into its place, or
    /// NULL_LINK.
    pub fn remove(&mut self, slot: GraphSlot, bodies: &GraphBodies) -> u32 {
        let target = &mut self.colors[slot.color as usize];
        if slot.color as usize != OVERFLOW_COLOR {
            target.release(bodies.a);
            if !bodies.b_is_static {
                target.release(bodies.b);
            }
        }
        let last = *target.contacts.last().expect("removing from an empty colour");
        let mut moved = NULL_LINK;
        if slot.local as usize + 1 != target.contacts.len() {
            target.contacts[slot.local as usize] = last;
            moved = last;
        }
        target.contacts.pop();
        self.size -= 1;
        moved
    }

    /// Contacts of one colour.
    pub fn contacts(&self, color: usize) -> &[u32] {
        &self.colors[color].contacts
    }

    /// Number of contacts in the graph.
    pub fn size(&self) -> usize {
        self.size
    }
}
