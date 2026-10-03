//! The engine's broad phase: static, kinematic and dynamic AABB trees, a
//! move buffer, and pair finding for the proxies that moved.
use crate::aabb_tree::{AabbTree, NULL_NODE, NodeId};
use crate::contact::ShapeRef;
use crate::vecmath::Aabb;

pub type ProxyId = i32;

#[derive(Clone, Copy)]
struct Proxy {
    shape: ShapeRef,
    fat_aabb: Aabb,
    node: NodeId,
    is_static: bool,
    kinematic: bool,
    moved: bool,
    alive: bool,
}

impl Default for Proxy {
    /// A dead proxy outside every tree.
    fn default() -> Proxy {
        Proxy { shape: ShapeRef::default(), fat_aabb: Aabb::default(), node: NULL_NODE, is_static: false, kinematic: false, moved: false, alive: false }
    }
}

#[derive(Default)]
pub struct BroadPhase {
    proxies: Vec<Proxy>,
    free: Vec<ProxyId>,
    move_buffer: Vec<ProxyId>,
    static_tree: AabbTree,
    kinematic_tree: AabbTree,
    dynamic_tree: AabbTree,
}

impl BroadPhase {
    /// Adds a proxy to the tree for its kind and buffers it as moved.
    pub fn create_proxy(&mut self, shape: ShapeRef, fat_aabb: &Aabb, kinematic: bool) -> ProxyId {
        let id = match self.free.pop() {
            None => {
                self.proxies.push(Proxy::default());
                (self.proxies.len() - 1) as ProxyId
            }
            Some(id) => id,
        };
        let is_static = shape.is_static;
        let mut proxy = Proxy { shape, fat_aabb: *fat_aabb, is_static, kinematic: kinematic && is_static, alive: true, ..Proxy::default() };
        proxy.node = self.tree_of_mut(&proxy).create_proxy(fat_aabb, id as u32);
        self.proxies[id as usize] = proxy;
        self.buffer_move(id);
        id
    }

    /// Gives a proxy new fat bounds and buffers it as moved.
    pub fn move_proxy(&mut self, id: ProxyId, fat_aabb: &Aabb) {
        self.proxies[id as usize].fat_aabb = *fat_aabb;
        let proxy = self.proxies[id as usize];
        self.tree_of_mut(&proxy).move_proxy(proxy.node, fat_aabb);
        self.buffer_move(id);
    }

    /// Fat bounds of a proxy.
    pub fn fat_aabb(&self, id: ProxyId) -> &Aabb {
        &self.proxies[id as usize].fat_aabb
    }

    /// Shape a proxy stands for.
    pub fn shape(&self, id: ProxyId) -> ShapeRef {
        self.proxies[id as usize].shape
    }

    /// Number of buffered moves.
    pub fn moved_count(&self) -> usize {
        self.move_buffer.len()
    }

    /// Calls `on_pair` once for every new overlap of the moved proxies in
    /// [begin, end), dynamic before static and lower id first.
    pub fn query_moved(&self, begin: usize, end: usize, mut on_pair: impl FnMut(ProxyId, ProxyId)) {
        for &query_id in &self.move_buffer[begin..end] {
            let query = &self.proxies[query_id as usize];
            if !query.alive {
                continue;
            }
            let mut visit = |found_tree: &AabbTree, node: NodeId| {
                let found_id = found_tree.user_data(node) as ProxyId;
                if found_id == query_id {
                    return true;
                }
                let found = &self.proxies[found_id as usize];
                if !query.is_static && !found.is_static && query.shape.body == found.shape.body {
                    return true;
                }
                if found.moved && found_id < query_id {
                    return true;
                }
                let query_first = !query.is_static && (found.is_static || query_id < found_id);
                if query_first {
                    on_pair(query_id, found_id);
                } else {
                    on_pair(found_id, query_id);
                }
                true
            };
            self.dynamic_tree.query(&query.fat_aabb, |node| visit(&self.dynamic_tree, node));
            if !query.is_static {
                self.static_tree.query(&query.fat_aabb, |node| visit(&self.static_tree, node));
                self.kinematic_tree.query(&query.fat_aabb, |node| visit(&self.kinematic_tree, node));
            }
        }
    }

    /// Clears the move buffer and the proxies' moved flags.
    pub fn clear_moves(&mut self) {
        for &id in &self.move_buffer {
            self.proxies[id as usize].moved = false;
        }
        self.move_buffer.clear();
    }

    /// Tree that holds a proxy of this kind.
    fn tree_of_mut(&mut self, proxy: &Proxy) -> &mut AabbTree {
        if !proxy.is_static {
            return &mut self.dynamic_tree;
        }
        if proxy.kinematic { &mut self.kinematic_tree } else { &mut self.static_tree }
    }

    /// Adds a proxy to the move buffer once.
    fn buffer_move(&mut self, id: ProxyId) {
        let proxy = &mut self.proxies[id as usize];
        if proxy.moved {
            return;
        }
        proxy.moved = true;
        self.move_buffer.push(id);
    }
}
