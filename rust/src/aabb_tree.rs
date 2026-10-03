//! The engine's dynamic AABB tree: surface-area-heuristic insertion with
//! AVL-style rotations and AABB queries.
use crate::vecmath::Aabb;

pub type NodeId = i32;
pub const NULL_NODE: NodeId = -1;

#[derive(Clone, Copy)]
struct Node {
    aabb: Aabb,
    user_data: u32,
    parent: NodeId,
    child1: NodeId,
    child2: NodeId,
    height: i32,
}

impl Default for Node {
    /// A detached leaf.
    fn default() -> Node {
        Node { aabb: Aabb::default(), user_data: 0, parent: NULL_NODE, child1: NULL_NODE, child2: NULL_NODE, height: 0 }
    }
}

impl Node {
    /// Whether the node has no children.
    fn is_leaf(&self) -> bool {
        self.child1 == NULL_NODE
    }
}

pub struct AabbTree {
    nodes: Vec<Node>,
    free_list: Vec<NodeId>,
    root: NodeId,
}

impl Default for AabbTree {
    /// An empty tree.
    fn default() -> AabbTree {
        AabbTree { nodes: Vec::new(), free_list: Vec::new(), root: NULL_NODE }
    }
}

impl AabbTree {
    /// Node by id.
    fn node(&self, id: NodeId) -> &Node {
        &self.nodes[id as usize]
    }

    /// Node by id, writable.
    fn node_mut(&mut self, id: NodeId) -> &mut Node {
        &mut self.nodes[id as usize]
    }

    /// Inserts a leaf for `aabb` carrying `user_data`.
    pub fn create_proxy(&mut self, aabb: &Aabb, user_data: u32) -> NodeId {
        let id = self.allocate_node();
        let leaf = self.node_mut(id);
        leaf.aabb = *aabb;
        leaf.user_data = user_data;
        leaf.height = 0;
        self.insert_leaf(id);
        id
    }

    /// Reinserts a leaf with new bounds.
    pub fn move_proxy(&mut self, proxy: NodeId, aabb: &Aabb) {
        self.remove_leaf(proxy);
        self.node_mut(proxy).aabb = *aabb;
        self.insert_leaf(proxy);
    }

    /// User data of a leaf.
    pub fn user_data(&self, proxy: NodeId) -> u32 {
        self.node(proxy).user_data
    }

    /// Calls `callback` with every leaf overlapping `aabb` until it returns
    /// false.
    pub fn query(&self, aabb: &Aabb, mut callback: impl FnMut(NodeId) -> bool) {
        let mut stack: Vec<NodeId> = Vec::with_capacity(64);
        stack.push(self.root);
        while let Some(id) = stack.pop() {
            if id == NULL_NODE {
                continue;
            }
            let node = self.node(id);
            if !node.aabb.overlaps(aabb) {
                continue;
            }
            if node.is_leaf() {
                if !callback(id) {
                    return;
                }
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
        }
    }

    /// A node from the free list, or a new one.
    fn allocate_node(&mut self) -> NodeId {
        match self.free_list.pop() {
            None => {
                self.nodes.push(Node::default());
                (self.nodes.len() - 1) as NodeId
            }
            Some(id) => {
                *self.node_mut(id) = Node::default();
                id
            }
        }
    }

    /// Returns a node to the free list.
    fn free_node(&mut self, id: NodeId) {
        *self.node_mut(id) = Node { height: -1, ..Node::default() };
        self.free_list.push(id);
    }

    /// Cost of descending into `child` to insert a leaf with `leaf_aabb`.
    fn child_cost(&self, child: NodeId, leaf_aabb: &Aabb, inheritance_cost: f32) -> f32 {
        let c = self.node(child);
        let merged = leaf_aabb.merge(&c.aabb).surface_area();
        if c.is_leaf() {
            return merged + inheritance_cost;
        }
        merged - c.aabb.surface_area() + inheritance_cost
    }

    /// Descends by the surface area heuristic, adds a parent over the chosen
    /// sibling, and refits and balances up to the root.
    fn insert_leaf(&mut self, leaf: NodeId) {
        if self.root == NULL_NODE {
            self.root = leaf;
            self.node_mut(leaf).parent = NULL_NODE;
            return;
        }

        let leaf_aabb = self.node(leaf).aabb;
        let mut index = self.root;
        while !self.node(index).is_leaf() {
            let child1 = self.node(index).child1;
            let child2 = self.node(index).child2;

            let area = self.node(index).aabb.surface_area();
            let combined_area = self.node(index).aabb.merge(&leaf_aabb).surface_area();
            let cost = 2.0 * combined_area;
            let inheritance_cost = 2.0 * (combined_area - area);

            let cost1 = self.child_cost(child1, &leaf_aabb, inheritance_cost);
            let cost2 = self.child_cost(child2, &leaf_aabb, inheritance_cost);

            if cost < cost1 && cost < cost2 {
                break;
            }
            index = if cost1 < cost2 { child1 } else { child2 };
        }

        let sibling = index;
        let old_parent = self.node(sibling).parent;
        let new_parent = self.allocate_node();
        let sibling_node = *self.node(sibling);
        let parent = self.node_mut(new_parent);
        parent.parent = old_parent;
        parent.aabb = leaf_aabb.merge(&sibling_node.aabb);
        parent.height = sibling_node.height + 1;

        if old_parent != NULL_NODE {
            self.replace_child(old_parent, sibling, new_parent);
        } else {
            self.root = new_parent;
        }
        let parent = self.node_mut(new_parent);
        parent.child1 = sibling;
        parent.child2 = leaf;
        self.node_mut(sibling).parent = new_parent;
        self.node_mut(leaf).parent = new_parent;

        let mut index = self.node(leaf).parent;
        while index != NULL_NODE {
            index = self.balance(index);
            let child1 = *self.node(self.node(index).child1);
            let child2 = *self.node(self.node(index).child2);
            let node = self.node_mut(index);
            node.height = 1 + child1.height.max(child2.height);
            node.aabb = child1.aabb.merge(&child2.aabb);
            index = node.parent;
        }
    }

    /// Points whichever child of `parent` is `old` at `new` instead.
    fn replace_child(&mut self, parent: NodeId, old: NodeId, new: NodeId) {
        let node = self.node_mut(parent);
        if node.child1 == old {
            node.child1 = new;
        } else {
            node.child2 = new;
        }
    }

    /// Replaces the leaf's parent with its sibling and refits up to the root.
    fn remove_leaf(&mut self, leaf: NodeId) {
        if leaf == self.root {
            self.root = NULL_NODE;
            return;
        }
        let parent = self.node(leaf).parent;
        let grand_parent = self.node(parent).parent;
        let sibling = if self.node(parent).child1 == leaf { self.node(parent).child2 } else { self.node(parent).child1 };

        if grand_parent != NULL_NODE {
            self.replace_child(grand_parent, parent, sibling);
            self.node_mut(sibling).parent = grand_parent;
            self.free_node(parent);

            let mut index = grand_parent;
            while index != NULL_NODE {
                index = self.balance(index);
                let child1 = *self.node(self.node(index).child1);
                let child2 = *self.node(self.node(index).child2);
                let node = self.node_mut(index);
                node.aabb = child1.aabb.merge(&child2.aabb);
                node.height = 1 + child1.height.max(child2.height);
                index = node.parent;
            }
        } else {
            self.root = sibling;
            self.node_mut(sibling).parent = NULL_NODE;
            self.free_node(parent);
        }
    }

    /// Rotates a node whose children differ in height by more than one;
    /// returns the node now in its place.
    fn balance(&mut self, a_id: NodeId) -> NodeId {
        let a = *self.node(a_id);
        if a.is_leaf() || a.height < 2 {
            return a_id;
        }
        let b_id = a.child1;
        let c_id = a.child2;
        let imbalance = self.node(c_id).height - self.node(b_id).height;
        if imbalance > 1 {
            return self.rotate_up(a_id, c_id, b_id, false);
        }
        if imbalance < -1 {
            return self.rotate_up(a_id, b_id, c_id, true);
        }
        a_id
    }

    /// Lifts child `up_id` of `a_id` into its place: `up_id` takes `a_id` as
    /// its first child, keeps its taller child, and hands the shorter one to
    /// `a_id` in the slot `up_id` vacated. `keep_id` is the other child of
    /// `a_id`; `up_is_child1` tells which slot `up_id` held.
    fn rotate_up(&mut self, a_id: NodeId, up_id: NodeId, keep_id: NodeId, up_is_child1: bool) -> NodeId {
        let a_parent = self.node(a_id).parent;
        let first_id = self.node(up_id).child1;
        let second_id = self.node(up_id).child2;

        self.node_mut(up_id).child1 = a_id;
        self.node_mut(up_id).parent = a_parent;
        self.node_mut(a_id).parent = up_id;
        if a_parent != NULL_NODE {
            self.replace_child(a_parent, a_id, up_id);
        } else {
            self.root = up_id;
        }

        let (taller_id, shorter_id) =
            if self.node(first_id).height > self.node(second_id).height { (first_id, second_id) } else { (second_id, first_id) };
        self.node_mut(up_id).child2 = taller_id;
        if up_is_child1 {
            self.node_mut(a_id).child1 = shorter_id;
        } else {
            self.node_mut(a_id).child2 = shorter_id;
        }
        self.node_mut(shorter_id).parent = a_id;

        let keep = *self.node(keep_id);
        let shorter = *self.node(shorter_id);
        let taller = *self.node(taller_id);
        let a_aabb = keep.aabb.merge(&shorter.aabb);
        let a_height = 1 + keep.height.max(shorter.height);
        let a = self.node_mut(a_id);
        a.aabb = a_aabb;
        a.height = a_height;
        let up = self.node_mut(up_id);
        up.aabb = a_aabb.merge(&taller.aabb);
        up.height = 1 + a_height.max(taller.height);
        up_id
    }
}
