//! Pre-routing analysis for the GUI: what is already on the board and what is still open.

use crate::engine::{Router, Wire};
use std::collections::HashMap;
use crate::geom::*;
use crate::plane::PlaneInfo;

/// Copper or keep-out that is on the board but belongs to no net of the board: vias, arcs,
/// circles, rectangles and solid regions without a net (or with an unknown one), drill holes,
/// board cut-outs. One entry per shape; `layers` are the copper layers it is on.
pub struct Obstacle {
    pub layers: Vec<usize>,
    pub geo: Geo,
    /// kept clear (holes, slots, regions) as opposed to copper of no net
    pub keepout: bool,
}

pub struct Preview {
    pub obstacles: Vec<Obstacle>,
    /// open connections (net, from, to) still to be routed
    pub airwires: Vec<(usize, P, P)>,
    pub pre_wires: Vec<(usize, Wire)>,
    pub pre_vias: Vec<(usize, P, f64)>,
    pub planes: Vec<PlaneInfo>,
    pub pairs: Vec<(usize, usize)>,
    /// number of groups per net (1 = already fully connected)
    pub groups: Vec<usize>,
    pub class_width: Vec<f64>,
    pub class_clearance: Vec<f64>,
}

impl Router {
    pub fn preview(&self) -> Preview {
        let mut airwires = vec![];
        let mut pre_wires = vec![];
        let mut pre_vias = vec![];
        let mut pairs = vec![];
        for (ni, n) in self.nets.iter().enumerate() {
            for w in &n.pre.wires {
                pre_wires.push((ni, w.clone()));
            }
            for v in &n.pre.vias {
                pre_vias.push((ni, *v, n.via_r));
            }
            if let Some(m) = n.pair {
                if ni < m {
                    pairs.push((ni, m));
                }
            }
            // connect every group to its nearest already-connected group
            let k = n.comps.len();
            if k < 2 || n.comps[0].is_empty() {
                continue;
            }
            let mut done = vec![0usize];
            let mut left: Vec<usize> = (1..k).collect();
            while !left.is_empty() {
                let mut best = (f64::MAX, 0usize, P::default(), P::default());
                for (li, &c) in left.iter().enumerate() {
                    for &p in &n.comps[c] {
                        for &d in &done {
                            for &q in &n.comps[d] {
                                let (a, b) = (self.pins[p].c, self.pins[q].c);
                                let dd = dist(a, b);
                                if dd < best.0 {
                                    best = (dd, li, a, b);
                                }
                            }
                        }
                    }
                }
                let c = left.remove(best.1);
                airwires.push((ni, best.2, best.3));
                done.push(c);
            }
        }
        // the board outline is drawn on its own; identical shapes on several layers are one entry
        let mut obstacles: Vec<Obstacle> = vec![];
        let mut index: HashMap<String, usize> = HashMap::new();
        for p in &self.fixed {
            if p.net == -2 && matches!(p.geo, Geo::Seg { hw, .. } if hw == 0.0) {
                continue;
            }
            let key = format!("{:?}{}", p.geo, p.net == -2);
            match index.get(&key) {
                Some(&i) => obstacles[i].layers.push(p.layer),
                None => {
                    index.insert(key, obstacles.len());
                    obstacles.push(Obstacle { layers: vec![p.layer], geo: p.geo.clone(), keepout: p.net == -2 });
                }
            }
        }
        Preview {
            obstacles,
            airwires,
            pre_wires,
            pre_vias,
            planes: self.planes.clone(),
            pairs,
            groups: self.nets.iter().map(|n| n.comps.len().max(1)).collect(),
            class_width: self.nets.iter().map(|n| n.width).collect(),
            class_clearance: self.nets.iter().map(|n| n.clr).collect(),
        }
    }
}
