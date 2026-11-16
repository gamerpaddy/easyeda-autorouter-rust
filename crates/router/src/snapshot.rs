//! Routing snapshots (JSON) consumed by the viewer.

use crate::engine::{Router, Wire};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone)]
pub struct SnapWire {
    pub net: usize,
    pub wire: Wire,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct SnapVia {
    pub net: usize,
    pub x: f64,
    pub y: f64,
    pub r: f64,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Snapshot {
    pub label: String,
    pub elapsed: f32,
    pub wires: Vec<SnapWire>,
    pub vias: Vec<SnapVia>,
    /// unrouted connections as (net, pin name)
    pub unrouted: Vec<(usize, String)>,
    pub length: f64,
    pub connections: usize,
    pub routed: usize,
    /// copper areas that will be exported (net, layer, outline)
    #[serde(default)]
    pub pours: Vec<(usize, usize, Vec<(f64, f64)>)>,
}

impl Snapshot {
    pub fn capture(r: &Router, label: &str) -> Snapshot {
        let mut wires = vec![];
        let mut vias = vec![];
        let mut unrouted = vec![];
        for (i, n) in r.nets.iter().enumerate() {
            for w in &n.r.wires {
                wires.push(SnapWire {
                    net: i,
                    wire: w.clone(),
                });
            }
            for v in &n.r.vias {
                vias.push(SnapVia {
                    net: i,
                    x: v.0,
                    y: v.1,
                    r: n.via_r,
                });
            }
            for &p in &n.unconnected {
                unrouted.push((i, r.pins[p].name.clone()));
            }
        }
        Snapshot {
            label: label.to_string(),
            elapsed: r.t0.elapsed().as_secs_f32(),
            wires,
            vias,
            length: r.total_length(),
            connections: r.connection_count(),
            routed: r.connection_count() - r.unrouted_count(),
            unrouted,
            pours: r.fill_polys.clone(),
        }
    }
}
