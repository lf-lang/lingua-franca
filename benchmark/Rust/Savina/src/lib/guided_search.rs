use rand::rngs::SmallRng;
use rand::{RngCore, SeedableRng};

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;

// Node in a grid of nodes.
// Neighbors and parents are represented by their IDs.
pub struct GridNode {
    pub id: u32,
    pub i: u32,
    pub j: u32,
    pub k: u32,
    neighbors: Vec<u32>,
    parent_in_path: Option<u32>,
}

impl GridNode {
    pub fn new(id: u32, i: u32, j: u32, k: u32) -> Self {
        Self {
            id,
            i,
            j,
            k,
            neighbors: vec![],
            parent_in_path: None,
        }
    }

    pub fn get_neighbor_ids(&self) -> &[u32] {
        &self.neighbors
    }

    pub fn get_parent_id(&self) -> Option<u32> {
        self.parent_in_path
    }

    pub fn set_parent(&mut self, parent: u32) -> bool {
        if self.parent_in_path.is_none() {
            self.parent_in_path = Some(parent);
            true
        } else {
            false
        }
    }

    pub fn add_neighbor(&mut self, neighbor: u32) -> bool {
        if self.id == neighbor {
            return false;
        }

        if !self.neighbors.contains(&neighbor) {
            self.neighbors.push(neighbor);
            true
        } else {
            false
        }
    }

    pub fn distance_from(&self, node: &GridNode) -> f64 {
        let i_diff: i64 = (self.i - node.i).into();
        let j_diff: i64 = (self.j - node.j).into();
        let k_diff: i64 = (self.k - node.k).into();
        let diff_squares = ((i_diff * i_diff) + (j_diff * j_diff) + (k_diff * k_diff)) as f64;
        diff_squares.sqrt()
    }
}

impl fmt::Display for GridNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}, {}, {}]", self.i, self.j, self.k)
    }
}

#[derive(Default)]
pub struct Grid {
    size: u32,
    priorities: u32,
    all_nodes: BTreeMap<u32, GridNode>,
}

impl Grid {
    pub fn new(size: u32, priorities: u32) -> Self {
        Grid {
            size,
            priorities,
            all_nodes: BTreeMap::new(),
        }
    }

    pub fn get_node(&self, node_id: u32) -> &GridNode {
        self.all_nodes.get(&node_id).unwrap()
    }

    pub fn get_node_mut(&mut self, node_id: u32) -> &mut GridNode {
        self.all_nodes.get_mut(&node_id).unwrap()
    }

    pub fn initialize_data(&mut self) {
        self.all_nodes.clear();

        let mut id = 0;
        for i in 0..self.size {
            for j in 0..self.size {
                for k in 0..self.size {
                    self.all_nodes.insert(id, GridNode::new(id, i, j, k));
                    id += 1;
                }
            }
        }

        let mut rng = SmallRng::from_entropy();
        let keys: Vec<u32> = self.all_nodes.keys().map(|x| *x).collect();
        for key in keys {
            let mut iter_count = 0;
            let mut neighbor_count = 0;

            for i in 0..2 {
                for j in 0..2 {
                    for k in 0..2 {
                        iter_count += 1;
                        if iter_count == 1 || iter_count == 8 {
                            continue;
                        }

                        let add_neighbor: bool =
                            (iter_count == 7 && neighbor_count == 0) || (rng.next_u32() % 2 == 1);
                        if add_neighbor {
                            let (old_i, old_j, old_k) = {
                                let node = self.all_nodes.get(&key).unwrap();
                                (node.i, node.j, node.k)
                            };
                            let new_i = (self.size - 1).min(old_i + i);
                            let new_j = (self.size - 1).min(old_j + j);
                            let new_k = (self.size - 1).min(old_k + k);

                            let new_id =
                                (self.size * self.size * new_i) + (self.size * new_j) + new_k;
                            if self.all_nodes.get_mut(&key).unwrap().add_neighbor(new_id) {
                                neighbor_count += 1;
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn get_origin_node_id(&self) -> u32 {
        *self.all_nodes.keys().next().unwrap()
    }

    pub fn calc_priorities(&self, node: &GridNode) -> u32 {
        let dist_diff = (self.size - node.i).max((self.size - node.j).max(self.size - node.k));
        let priority_unit = dist_diff / 8;
        f64::from(self.priorities - priority_unit).abs() as u32
    }

    pub fn target_node_id(&self) -> u32 {
        let axis_val = (0.8 * self.size as f32) as u32;
        (axis_val * self.size * self.size) + (axis_val * self.size) + axis_val
    }

    pub fn validate(&self) -> bool {
        let mut parent_node_id = self.target_node_id();
        let mut parent_node = self.all_nodes.get(&parent_node_id).unwrap();
        while let Some(parent) = parent_node.get_parent_id() {
            parent_node_id = parent;
            parent_node = self.all_nodes.get(&parent_node_id).unwrap();
        }

        parent_node_id == self.get_origin_node_id()
    }
}

#[derive(Clone, Copy)]
pub struct WorkMessage {
    pub node_id: u32,
    pub target_id: u32,
}

impl WorkMessage {
    pub fn new(node_id: u32, target_id: u32) -> Self {
        Self { node_id, target_id }
    }
}
