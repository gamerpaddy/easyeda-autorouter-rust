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
    /// Open connections as lines (net, from, to): every cut-off piece of a net joined to the nearest
    /// piece that is connected. Only filled in once few connections are open.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub airwires: Vec<(usize, (f64, f64), (f64, f64))>,
    /// Actual pad centres for a moved layout; absent in older snapshots.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pin_positions: Vec<(String, (f64, f64))>,
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
        // the nearest-piece lines of the open nets (not while hundreds are open: too slow, and
        // the lines of the old kind do for the early snapshots)
        let mut airwires = vec![];
        if r.unrouted_count() <= 40 {
            for (i, n) in r.nets.iter().enumerate() {
                // a net with a generated plane: the pieces on the plane are one, which the
                // geometric islands do not know; its open pins keep the simple line
                if n.unconnected.is_empty() || n.skip || n.plane.is_some() {
                    continue;
                }
                let isl = crate::drc::net_islands(r, i);
                if isl.len() < 2 {
                    continue;
                }
                // grow from the piece with the most pads: each next piece joins the nearest pad
                let root = (0..isl.len()).max_by_key(|&k| isl[k].pins.len()).unwrap_or(0);
                let mut done = vec![root];
                let mut left: Vec<usize> = (0..isl.len()).filter(|&k| k != root).collect();
                while !left.is_empty() {
                    let mut best: Option<(f64, usize, (f64, f64), (f64, f64))> = None;
                    for (li, &k) in left.iter().enumerate() {
                        for &a in &isl[k].pins {
                            for &d in &done {
                                for &b in &isl[d].pins {
                                    let (pa, pb) = (r.pins[a].c, r.pins[b].c);
                                    let dd = (pa.0 - pb.0).hypot(pa.1 - pb.1);
                                    if best.is_none_or(|x| dd < x.0) {
                                        best = Some((dd, li, pa, pb));
                                    }
                                }
                            }
                        }
                    }
                    let Some((_, li, pa, pb)) = best else { break };
                    airwires.push((i, pa, pb));
                    done.push(left.remove(li));
                }
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
            airwires,
            pours: r.fill_polys.clone(),
            pin_positions: if r.part_moves.is_empty() { vec![] } else {
                r.pins.iter().map(|p| (p.name.clone(), p.c)).collect()
            },
        }
    }
}
