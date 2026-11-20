//! Push and shove. The open net `n` is routed through a spot that is too crowded for it, where
//! routed copper only counts at a fraction of its clearance (fixed copper stays strict); then the
//! copper around it is pushed away: every via and every trace vertex of the nearby nets is a
//! movable node, each clearance violation pushes the offending piece away from what it hits, which
//! pushes the next one in turn, until everything fits again or the budget is used up. Wires move
//! with the vias they end in, pads and wire ends on pads stay. The result is checked with the
//! exact clearance test and the connectivity of every net that moved; otherwise everything is put
//! back as it was.

use crate::engine::*;
use crate::geom::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Node {
    pub pos: P,
    pub orig: P,
    pub fixed: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Kind {
    Seg { n0: usize, n1: usize, hw: f64 },
    Via { node: usize, r: f64 },
}

/// A piece of routed copper that may move.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Item {
    pub net: usize,
    /// layer of a wire piece (a via is on all layers)
    pub layer: usize,
    pub kind: Kind,
    pub clr: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Shape {
    Circle { c: P, r: f64 },
    Seg { a: P, b: P, hw: f64 },
    Poly { pts: Vec<P>, c: P },
}

/// Copper that stays where it is.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Obst {
    pub shape: Shape,
    pub layer: usize,
    pub net: i32,
    pub clr: f64,
    pub bb: (f64, f64, f64, f64),
}

#[derive(Clone, Copy)]
enum Core {
    Pt(P),
    Sg(P, P),
}

fn closest_pt_seg(p: P, a: P, b: P) -> (P, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-18 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) };
    ((a.0 + t * dx, a.1 + t * dy), t)
}

/// Closest points of two segments, with their parameters: (point on a, ta, point on b, tb).
fn closest_seg_seg(a0: P, a1: P, b0: P, b1: P) -> (P, f64, P, f64) {
    let (d1, d2) = ((a1.0 - a0.0, a1.1 - a0.1), (b1.0 - b0.0, b1.1 - b0.1));
    let den = d1.0 * d2.1 - d1.1 * d2.0;
    if den.abs() > 1e-18 {
        let (ex, ey) = (b0.0 - a0.0, b0.1 - a0.1);
        let ta = (ex * d2.1 - ey * d2.0) / den;
        let tb = (ex * d1.1 - ey * d1.0) / den;
        if (0.0..=1.0).contains(&ta) && (0.0..=1.0).contains(&tb) {
            let p = (a0.0 + ta * d1.0, a0.1 + ta * d1.1);
            return (p, ta, p, tb);
        }
    }
    let mut best: Option<(f64, P, f64, P, f64)> = None;
    let mut keep = |qa: P, ta: f64, qb: P, tb: f64| {
        let d = dist(qa, qb);
        if best.is_none_or(|x| d < x.0) {
            best = Some((d, qa, ta, qb, tb));
        }
    };
    for (t, p) in [(0.0, a0), (1.0, a1)] {
        let (q, tb) = closest_pt_seg(p, b0, b1);
        keep(p, t, q, tb);
    }
    for (t, p) in [(0.0, b0), (1.0, b1)] {
        let (q, ta) = closest_pt_seg(p, a0, a1);
        keep(q, ta, p, t);
    }
    let b = best.unwrap();
    (b.1, b.2, b.3, b.4)
}

fn closest(a: Core, b: Core) -> (P, f64, P, f64) {
    match (a, b) {
        (Core::Pt(p), Core::Pt(q)) => (p, 0.0, q, 0.0),
        (Core::Pt(p), Core::Sg(b0, b1)) => {
            let (q, tb) = closest_pt_seg(p, b0, b1);
            (p, 0.0, q, tb)
        }
        (Core::Sg(a0, a1), Core::Pt(q)) => {
            let (p, ta) = closest_pt_seg(q, a0, a1);
            (p, ta, q, 0.0)
        }
        (Core::Sg(a0, a1), Core::Sg(b0, b1)) => closest_seg_seg(a0, a1, b0, b1),
    }
}

fn unit(v: P, fallback: P) -> P {
    let l = (v.0 * v.0 + v.1 * v.1).sqrt();
    if l < 1e-12 {
        fallback
    } else {
        (v.0 / l, v.1 / l)
    }
}

/// Result of measuring a piece of copper against another.
pub(crate) struct Hit {
    /// edge to edge distance (negative: overlapping)
    pub gap: f64,
    /// unit vector pointing from the other piece to this one
    pub u: P,
    /// parameter of the closest point along this piece / along the other one (segments)
    pub ta: f64,
    pub tb: f64,
}

fn measure(a: Core, ra: f64, o: &Shape) -> Hit {
    let normal = match a {
        Core::Sg(p, q) => unit((-(q.1 - p.1), q.0 - p.0), (0.0, 1.0)),
        Core::Pt(_) => (0.0, 1.0),
    };
    match o {
        Shape::Circle { c, r } => {
            let (qa, ta, qb, tb) = closest(a, Core::Pt(*c));
            let d = dist(qa, qb);
            Hit { gap: d - ra - r, u: unit((qa.0 - qb.0, qa.1 - qb.1), normal), ta, tb }
        }
        Shape::Seg { a: b0, b: b1, hw } => {
            let (qa, ta, qb, tb) = closest(a, Core::Sg(*b0, *b1));
            let d = dist(qa, qb);
            Hit { gap: d - ra - hw, u: unit((qa.0 - qb.0, qa.1 - qb.1), normal), ta, tb }
        }
        Shape::Poly { pts, c } => {
            let mut best: Option<(f64, P, f64, P)> = None;
            let n = pts.len();
            for i in 0..n {
                let (qa, ta, qb, _) = closest(a, Core::Sg(pts[i], pts[(i + 1) % n]));
                let d = dist(qa, qb);
                if best.is_none_or(|x| d < x.0) {
                    best = Some((d, qa, ta, qb));
                }
            }
            let Some((d, qa, ta, qb)) = best else {
                return Hit { gap: f64::MAX, u: normal, ta: 0.0, tb: 0.0 };
            };
            let inside = match a {
                Core::Pt(p) => point_in_poly(p, pts),
                Core::Sg(p, q) => point_in_poly(p, pts) || point_in_poly(q, pts) || point_in_poly(((p.0 + q.0) / 2.0, (p.1 + q.1) / 2.0), pts),
            };
            if inside {
                Hit { gap: -d - ra, u: unit((qa.0 - c.0, qa.1 - c.1), normal), ta, tb: 0.0 }
            } else {
                Hit { gap: d - ra, u: unit((qa.0 - qb.0, qa.1 - qb.1), normal), ta, tb: 0.0 }
            }
        }
    }
}

fn item_core(it: &Item, nodes: &[Node]) -> (Core, f64) {
    match it.kind {
        Kind::Seg { n0, n1, hw } => (Core::Sg(nodes[n0].pos, nodes[n1].pos), hw),
        Kind::Via { node, r } => (Core::Pt(nodes[node].pos), r),
    }
}

fn item_shape(it: &Item, nodes: &[Node]) -> Shape {
    match it.kind {
        Kind::Seg { n0, n1, hw } => Shape::Seg { a: nodes[n0].pos, b: nodes[n1].pos, hw },
        Kind::Via { node, r } => Shape::Circle { c: nodes[node].pos, r },
    }
}

/// Push piece `i` by `amount` along `u`: the nodes it hangs on move, weighted by where the
/// closest point lies. False when nothing of it can move.
fn push(nodes: &mut [Node], it: &Item, t: f64, u: P, amount: f64, cap_mult: f64) -> bool {
    let cap = cap_mult * amount.abs() + 1e-9;
    match it.kind {
        Kind::Via { node, .. } => {
            if nodes[node].fixed {
                return false;
            }
            nodes[node].pos = (nodes[node].pos.0 + u.0 * amount, nodes[node].pos.1 + u.1 * amount);
            true
        }
        Kind::Seg { n0, n1, .. } => {
            let (w0, w1) = (1.0 - t, t);
            let (m0, m1) = (!nodes[n0].fixed, !nodes[n1].fixed);
            let den = (if m0 { w0 * w0 } else { 0.0 }) + (if m1 { w1 * w1 } else { 0.0 });
            if den < 1e-4 {
                return false;
            }
            for (n, w, m) in [(n0, w0, m0), (n1, w1, m1)] {
                if !m {
                    continue;
                }
                let k = (w / den * amount).clamp(-cap, cap);
                nodes[n].pos = (nodes[n].pos.0 + u.0 * k, nodes[n].pos.1 + u.1 * k);
            }
            true
        }
    }
}

/// Solver settings.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Params {
    /// farthest a node may end up from where it was
    pub max_shove: f64,
    pub sweeps: usize,
    /// extra gap kept beyond the clearance
    pub margin: f64,
    /// fraction of a violation that is resolved per push (1 = all of it)
    pub relax: f64,
    /// a single push moves a node at most this many times the violation
    pub cap_mult: f64,
}

/// Resolve every clearance violation of the movable pieces by pushing them, sweep after sweep.
/// True when no violation is left; false when a violation cannot be resolved, a node would move
/// farther than `max_shove`, or the sweeps run out.
pub fn solve(nodes: &mut [Node], items: &[Item], obst: &[Obst], max_shove: f64, sweeps: usize, margin: f64) -> bool {
    let p = Params { max_shove, sweeps, margin, relax: 1.0, cap_mult: 3.0 };
    solve_with(nodes, items, obst, &p).is_ok()
}

pub fn solve_why(nodes: &mut [Node], items: &[Item], obst: &[Obst], max_shove: f64, sweeps: usize, margin: f64) -> Result<(), String> {
    let p = Params { max_shove, sweeps, margin, relax: 1.0, cap_mult: 3.0 };
    solve_with(nodes, items, obst, &p)
}

/// `solve`, saying why it failed: the net of the piece that could not move (and what it hit),
/// or that a node went too far, or that the sweeps ran out.
pub fn solve_with(nodes: &mut [Node], items: &[Item], obst: &[Obst], prm: &Params) -> Result<(), String> {
    let (max_shove, sweeps, margin) = (prm.max_shove, prm.sweeps, prm.margin);
    let mut last = String::new();
    for _ in 0..sweeps {
        let mut worst = 0.0f64;
        for i in 0..items.len() {
            let (core, ra) = item_core(&items[i], nodes);
            let need_base = items[i].clr;
            let bb = core_bb(core, ra + need_base + margin);
            // against copper that stays
            for o in obst {
                if o.net == items[i].net as i32 {
                    continue;
                }
                if let Kind::Seg { .. } = items[i].kind {
                    if o.layer != items[i].layer {
                        continue;
                    }
                }
                let need = need_base.max(o.clr);
                if o.bb.0 > bb.2 + need || o.bb.2 < bb.0 - need || o.bb.1 > bb.3 + need || o.bb.3 < bb.1 - need {
                    continue;
                }
                let (core, ra) = item_core(&items[i], nodes);
                let h = measure(core, ra, &o.shape);
                let deficit = need + margin - h.gap;
                if deficit > 1e-6 && h.gap < need - 1e-6 {
                    worst = worst.max(deficit);
                    if !push(nodes, &items[i], h.ta, h.u, deficit * prm.relax, prm.cap_mult) {
                        return Err(format!("net {} piece (layer {}) at {:?} cannot move away from fixed copper of net {} (layer {}, deficit {:.3})", items[i].net, items[i].layer, core_bb(core, 0.0).0, o.net, o.layer, deficit));
                    }
                    last = format!("last violation: net {} vs fixed net {} deficit {:.3}", items[i].net, o.net, deficit);
                }
            }
            // against other movable pieces: both give way
            for j in i + 1..items.len() {
                if items[j].net == items[i].net {
                    continue;
                }
                if let (Kind::Seg { .. }, Kind::Seg { .. }) = (&items[i].kind, &items[j].kind) {
                    if items[i].layer != items[j].layer {
                        continue;
                    }
                }
                let need = need_base.max(items[j].clr);
                let (core, ra) = item_core(&items[i], nodes);
                let (cj, rb) = item_core(&items[j], nodes);
                {
                    let (bi, bj) = (core_bb(core, ra + need + margin), core_bb(cj, rb));
                    if bi.0 > bj.2 || bj.0 > bi.2 || bi.1 > bj.3 || bj.1 > bi.3 {
                        continue;
                    }
                }
                let shape = item_shape(&items[j], nodes);
                let h = measure(core, ra, &shape);
                let _ = rb;
                let deficit = need + margin - h.gap;
                if deficit > 1e-6 && h.gap < need - 1e-6 {
                    worst = worst.max(deficit);
                    let (a_ok, b_ok) = (can_move(&items[i], nodes, h.ta), can_move(&items[j], nodes, h.tb));
                    let (sa, sb) = match (a_ok, b_ok) {
                        (true, true) => (0.5, 0.5),
                        (true, false) => (1.0, 0.0),
                        (false, true) => (0.0, 1.0),
                        (false, false) => return Err(format!("nets {} and {} both stuck, deficit {:.3}", items[i].net, items[j].net, deficit)),
                    };
                    if sa > 0.0 && !push(nodes, &items[i], h.ta, h.u, deficit * sa * prm.relax, prm.cap_mult) {
                        return Err(format!("net {} cannot be pushed off net {}", items[i].net, items[j].net));
                    }
                    if sb > 0.0 && !push(nodes, &items[j], h.tb, (-h.u.0, -h.u.1), deficit * sb * prm.relax, prm.cap_mult) {
                        return Err(format!("net {} cannot be pushed off net {}", items[j].net, items[i].net));
                    }
                    last = format!("last violation: net {} vs net {} deficit {:.3}", items[i].net, items[j].net, deficit);
                }
            }
        }
        if let Some(far) = nodes.iter().map(|n| dist(n.pos, n.orig)).fold(None, |a: Option<f64>, d| Some(a.map_or(d, |x| x.max(d)))).filter(|d| *d > max_shove) {
            return Err(format!("a node was pushed {:.2} away (limit {:.2})", far, max_shove));
        }
        if worst < 1e-6 {
            return Ok(());
        }
    }
    Err(format!("sweeps used up, {}", last))
}

/// Can piece `it` take a push at its closest point `t`? (Same test as `push` makes.)
fn can_move(it: &Item, nodes: &[Node], t: f64) -> bool {
    match it.kind {
        Kind::Via { node, .. } => !nodes[node].fixed,
        Kind::Seg { n0, n1, .. } => {
            let (w0, w1) = (1.0 - t, t);
            let den = (if nodes[n0].fixed { 0.0 } else { w0 * w0 }) + (if nodes[n1].fixed { 0.0 } else { w1 * w1 });
            den >= 1e-4
        }
    }
}

fn core_bb(c: Core, pad: f64) -> (f64, f64, f64, f64) {
    match c {
        Core::Pt(p) => (p.0 - pad, p.1 - pad, p.0 + pad, p.1 + pad),
        Core::Sg(a, b) => (a.0.min(b.0) - pad, a.1.min(b.1) - pad, a.0.max(b.0) + pad, a.1.max(b.1) + pad),
    }
}

fn obst_of(p: &Prim) -> Obst {
    let shape = match &p.geo {
        Geo::Circle { c, r } => Shape::Circle { c: *c, r: *r },
        Geo::Seg { a, b, hw } => Shape::Seg { a: *a, b: *b, hw: *hw },
        Geo::Poly(v) => {
            let n = v.len().max(1) as f64;
            let c = (v.iter().map(|q| q.0).sum::<f64>() / n, v.iter().map(|q| q.1).sum::<f64>() / n);
            Shape::Poly { pts: v.clone(), c }
        }
    };
    Obst { shape, layer: p.layer, net: p.net, clr: p.clr, bb: p.bb }
}

fn key(p: P) -> (i64, i64) {
    ((p.0 * 1e4).round() as i64, (p.1 * 1e4).round() as i64)
}

impl Router {
    /// Route the open net `n` through the crowded spot and shove what is in the way aside.
    /// Everything is checked and put back when it does not work. True when `n` is connected now.
    pub(crate) fn shove_route(&mut self, n: usize) -> bool {
        // the less the new path squeezes, the less has to be shoved: try the gentlest first
        let t0 = std::time::Instant::now();
        for squeeze in [0.8, 0.55, 0.3] {
            if self.cancelled() || t0.elapsed().as_secs() > 8 {
                break;
            }
            if self.shove_try(n, squeeze) {
                return true;
            }
        }
        false
    }

    /// One try with routed copper counting at `squeeze` times its clearance for the new path.
    fn shove_try(&mut self, n: usize, squeeze: f64) -> bool {
        // the new path is squeezed through at the narrowest width (and widened afterwards where there is
        // room): not when narrowing is switched off, and never for a wide net (a power rail)
        if !self.cfg.allow_neckdown || self.nets[n].width > 3.0 * self.fields[self.nets[n].narrow_tf].p * 2.0 {
            return false;
        }
        if self.nets[n].pair.is_some() || self.nets[n].bundle.is_some() || self.nets[n].coupled || self.nets[n].guided {
            return false;
        }
        let isl = crate::drc::net_islands(self, n);
        if isl.len() < 2 || self.fields.is_empty() || self.fields[0].cnt.is_empty() {
            return false;
        }
        // how many pieces every net has now: a net that moves must not be cut by it
        let before: Vec<usize> = (0..self.nets.len())
            .map(|m| if self.nets[m].r.wires.is_empty() { 0 } else { crate::drc::net_islands(self, m).len() })
            .collect();
        // everything that can change: the whole state of the routes, put back on failure
        let saved: Vec<(Routed, Vec<usize>)> = self.nets.iter().map(|x| (x.r.clone(), x.unconnected.clone())).collect();
        let revert = |me: &mut Router| {
            for m in 0..me.nets.len() {
                let prims = std::mem::take(&mut me.nets[m].r.dynp);
                me.stamp_dyn(&prims, -1);
            }
            for (m, (r, unc)) in saved.iter().enumerate() {
                me.nets[m].r = r.clone();
                me.nets[m].unconnected = unc.clone();
                let prims = me.nets[m].r.dynp.clone();
                me.stamp_dyn(&prims, 1);
            }
        };
        // 1. route `n` on a field where routed copper only counts at a fraction of its clearance
        let narrow = self.nets[n].narrow_tf;
        let mut sq = Field::new(&self.grid, self.nets[n].clr, self.fields[narrow].p, false);
        sq.dyn_k = squeeze;
        {
            let g = self.grid.clone();
            for pin in &self.pins {
                for pr in &pin.prims {
                    sq.stamp(&g, pr, 1, false);
                }
            }
            for pr in &self.fixed {
                sq.stamp(&g, pr, 1, false);
            }
            for ns in &self.nets {
                for pr in &ns.pre.dynp {
                    sq.stamp(&g, pr, 1, false);
                }
                for pr in &ns.r.dynp {
                    sq.stamp(&g, pr, 1, true);
                }
            }
        }
        self.fields.push(sq);
        let sq_idx = self.fields.len() - 1;
        let saved_ladder = std::mem::replace(&mut self.nets[n].ladder, vec![sq_idx]);
        let saved_flags = (self.cfg.narrow_first, self.cfg.allow_neckdown);
        self.cfg.narrow_first = false;
        self.cfg.allow_neckdown = true;
        self.rip_net(n);
        self.route_net(n, false);
        self.cfg.narrow_first = saved_flags.0;
        self.cfg.allow_neckdown = saved_flags.1;
        self.nets[n].ladder = saved_ladder;
        self.fields.pop();
        let dbg = std::env::var_os("AR_DEBUG_SHOVE").is_some();
        if !self.nets[n].unconnected.is_empty() {
            if dbg {
                eprintln!("shove {} (squeeze {}): the squeezed field finds no path", self.nets[n].name, squeeze);
            }
            revert(self);
            return false;
        }
        // 2. the pieces that can move
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &self.nets[n].r.dynp {
            x0 = x0.min(p.bb.0);
            y0 = y0.min(p.bb.1);
            x1 = x1.max(p.bb.2);
            y1 = y1.max(p.bb.3);
        }
        let via_d = 2.0 * self.nets[n].via_r;
        let pad = (4.0 / self.unit_mm.max(1e-12)).max(4.0 * via_d);
        let region = (x0 - pad, y0 - pad, x1 + pad, y1 + pad);
        let inside = |p: P| p.0 >= region.0 && p.0 <= region.2 && p.1 >= region.1 && p.1 <= region.3;
        let max_shove = (3.0 / self.unit_mm.max(1e-12)).max(2.0 * via_d);
        let margin = 0.02;

        let mut nodes: Vec<Node> = vec![];
        // nodes are shared only by pieces of the same net that meet at a point: on the same layer, or
        // through a via (layer -1); a bend of another net or layer at the same point is another node
        let mut index: HashMap<(usize, i64, i64, i64), usize> = HashMap::new();
        let mut node_of = |nodes: &mut Vec<Node>, net: usize, layer: i64, p: P| -> usize {
            let k = key(p);
            *index.entry((net, layer, k.0, k.1)).or_insert_with(|| {
                nodes.push(Node { pos: p, orig: p, fixed: false });
                nodes.len() - 1
            })
        };
        let mut items: Vec<Item> = vec![];
        let mut obst: Vec<Obst> = vec![];
        // per moved net: for every wire, the node of every vertex; for every via, its node
        let mut layout: Vec<(usize, Vec<Vec<usize>>, Vec<usize>)> = vec![];
        let near = |p: &Prim| p.bb.2 >= region.0 - 2.0 * via_d && p.bb.0 <= region.2 + 2.0 * via_d && p.bb.3 >= region.1 - 2.0 * via_d && p.bb.1 <= region.3 + 2.0 * via_d;
        for pin in &self.pins {
            for p in pin.prims.iter().filter(|p| near(p)) {
                obst.push(obst_of(p));
            }
        }
        for p in self.fixed.iter().filter(|p| near(p)) {
            obst.push(obst_of(p));
        }
        for (m, ns) in self.nets.iter().enumerate() {
            for p in ns.pre.dynp.iter().filter(|p| !p.trace_only && near(p)) {
                obst.push(obst_of(p));
            }
            let touches = ns.r.wires.iter().any(|w| !w.fill && w.pts.iter().any(|q| inside(*q))) || ns.r.vias.iter().any(|v| inside(*v));
            let movable = m != n && !ns.skip && !ns.coupled && !ns.guided && ns.pair.is_none() && ns.bundle.is_none() && touches && ns.r.jumpers.is_empty();
            if !movable {
                for p in ns.r.dynp.iter().filter(|p| near(p)) {
                    obst.push(obst_of(p));
                }
                continue;
            }
            // own copper that wire ends and vias may be fixed to
            let mut own: Vec<&Prim> = vec![];
            for &pi in &ns.pins {
                own.extend(self.pins[pi].prims.iter());
            }
            own.extend(ns.pre.dynp.iter().filter(|p| !p.trace_only));
            let on_own = |p: P, layer: Option<usize>| own.iter().any(|c| layer.is_none_or(|l| c.layer == l) && c.dist(p) <= 1e-6);
            let via_keys: std::collections::HashSet<(i64, i64)> = ns.r.vias.iter().map(|&v| key(v)).collect();
            let layer_of = |w: &Wire, p: P| -> i64 { if via_keys.contains(&key(p)) { -1 } else { w.layer as i64 } };
            let mut wire_nodes: Vec<Vec<usize>> = vec![];
            for w in &ns.r.wires {
                if w.fill {
                    wire_nodes.push(vec![]);
                    continue;
                }
                let ids: Vec<usize> = w.pts.iter().map(|&p| node_of(&mut nodes, m, layer_of(w, p), p)).collect();
                for (k, &p) in w.pts.iter().enumerate() {
                    if !inside(p) || on_own(p, Some(w.layer)) {
                        nodes[ids[k]].fixed = true;
                    }
                }
                for k in 0..w.pts.len().saturating_sub(1) {
                    let (a, b) = (w.pts[k], w.pts[k + 1]);
                    if inside(a) || inside(b) {
                        items.push(Item { net: m, layer: w.layer, kind: Kind::Seg { n0: ids[k], n1: ids[k + 1], hw: w.width / 2.0 }, clr: ns.clr });
                    } else {
                        // far from the work: stays
                        let hw = w.width / 2.0;
                        obst.push(Obst { shape: Shape::Seg { a, b, hw }, layer: w.layer, net: m as i32, clr: ns.clr, bb: (a.0.min(b.0) - hw, a.1.min(b.1) - hw, a.0.max(b.0) + hw, a.1.max(b.1) + hw) });
                    }
                }
                wire_nodes.push(ids);
            }
            let mut via_nodes: Vec<usize> = vec![];
            for &v in &ns.r.vias {
                let id = node_of(&mut nodes, m, -1, v);
                if !inside(v) || on_own(v, None) {
                    nodes[id].fixed = true;
                }
                if inside(v) {
                    items.push(Item { net: m, layer: 0, kind: Kind::Via { node: id, r: ns.via_r }, clr: ns.clr });
                } else {
                    // far from the work: stays, as copper on every layer
                    for l in 0..self.grid.nl {
                        obst.push(Obst { shape: Shape::Circle { c: v, r: ns.via_r }, layer: l, net: m as i32, clr: ns.clr, bb: (v.0 - ns.via_r, v.1 - ns.via_r, v.0 + ns.via_r, v.1 + ns.via_r) });
                    }
                }
                via_nodes.push(id);
            }
            // a wire end that lies in the middle of another wire of the net: both stay
            for (wi, w) in ns.r.wires.iter().enumerate().filter(|(_, w)| !w.fill) {
                for (wj, o) in ns.r.wires.iter().enumerate().filter(|(_, o)| !o.fill && o.layer == w.layer) {
                    if wi == wj {
                        continue;
                    }
                    for &end in [o.pts[0], *o.pts.last().unwrap()].iter() {
                        let inner = w.pts.windows(2).any(|s| dist_pt_seg(end, s[0], s[1]) < 1e-6 && dist(end, s[0]) > 1e-6 && dist(end, s[1]) > 1e-6);
                        if inner {
                            for &id in &wire_nodes[wi] {
                                nodes[id].fixed = true;
                            }
                            let id = node_of(&mut nodes, m, layer_of(o, end), end);
                            nodes[id].fixed = true;
                        }
                    }
                }
            }
            layout.push((m, wire_nodes, via_nodes));
        }
        if layout.is_empty() {
            // nothing to shove: the squeezed route has to be legal as it is
            let prims = self.nets[n].r.dynp.clone();
            if self.prims_conflict(&prims).is_none() {
                return true;
            }
            revert(self);
            return false;
        }
        // 3. push
        let backup = nodes.clone();
        let prm = Params { max_shove, sweeps: 150, margin, relax: 0.8, cap_mult: 3.0 };
        if let Err(why) = solve_with(&mut nodes, &items, &obst, &prm) {
            if let Some(dir) = std::env::var_os("AR_DUMP_SHOVE") {
                let dir = std::path::PathBuf::from(dir);
                let _ = std::fs::create_dir_all(&dir);
                let k = std::fs::read_dir(&dir).map_or(0, |d| d.count());
                if k < 12 {
                    let doc = serde_json::json!({"net": self.nets[n].name, "why": why, "nodes": backup, "items": items, "obst": obst, "max_shove": max_shove, "margin": margin});
                    let _ = std::fs::write(dir.join(format!("shove{k}.json")), doc.to_string());
                }
            }
            if dbg {
                eprintln!("shove {} (squeeze {}): the pieces cannot be pushed apart ({} nets, {} pieces, {} nodes): {}", self.nets[n].name, squeeze, layout.len(), items.len(), nodes.len(), why);
            }
            revert(self);
            return false;
        }
        // 4. apply, then check with the exact clearance test and the connectivity of every moved net
        let moved: Vec<usize> = layout
            .iter()
            .filter(|(_, wn, vn)| wn.iter().flatten().chain(vn.iter()).any(|&id| dist(nodes[id].pos, nodes[id].orig) > 1e-9))
            .map(|(m, _, _)| *m)
            .collect();
        for (m, wn, vn) in &layout {
            if !moved.contains(m) {
                continue;
            }
            self.stamp_net(*m, -1);
            for (wi, ids) in wn.iter().enumerate() {
                for (k, &id) in ids.iter().enumerate() {
                    self.nets[*m].r.wires[wi].pts[k] = nodes[id].pos;
                }
            }
            for (vi, &id) in vn.iter().enumerate() {
                self.nets[*m].r.vias[vi] = nodes[id].pos;
            }
            self.rebuild_route_prims(*m);
            self.stamp_net(*m, 1);
        }
        let mut ok = true;
        'check: for &m in moved.iter().chain(std::iter::once(&n)) {
            let prims = self.nets[m].r.dynp.clone();
            if self.prims_conflict(&prims).is_some() {
                ok = false;
                break 'check;
            }
        }
        if ok {
            for &m in &moved {
                for v in &self.nets[m].r.vias {
                    if !point_in_poly(*v, &self.boundary) {
                        ok = false;
                    }
                }
            }
        }
        // a net that moved must not have been cut in two by it
        if ok {
            for &m in &moved {
                if crate::drc::net_islands(self, m).len() > before[m] {
                    ok = false;
                    break;
                }
            }
        }
        if ok && crate::drc::net_islands(self, n).len() > 1 {
            ok = false;
        }
        if !ok {
            if dbg {
                eprintln!("shove {} (squeeze {}): pushed apart, but the exact check or the connectivity fails", self.nets[n].name, squeeze);
            }
            revert(self);
            return false;
        }
        if dbg {
            eprintln!("shove {} (squeeze {}): WORKS, {} net(s) moved", self.nets[n].name, squeeze, moved.len());
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixed(p: P) -> Node {
        Node { pos: p, orig: p, fixed: true }
    }
    fn free(p: P) -> Node {
        Node { pos: p, orig: p, fixed: false }
    }

    #[test]
    fn a_trace_is_pushed_off_a_pad_that_is_too_close() {
        // a wire 0,0 - 5,0 - 10,0 with a movable middle; a round obstacle just above the middle
        let mut nodes = vec![fixed((0.0, 0.0)), free((5.0, 0.0)), fixed((10.0, 0.0))];
        let items = vec![
            Item { net: 0, layer: 0, kind: Kind::Seg { n0: 0, n1: 1, hw: 0.1 }, clr: 0.2 },
            Item { net: 0, layer: 0, kind: Kind::Seg { n0: 1, n1: 2, hw: 0.1 }, clr: 0.2 },
        ];
        let obst = vec![Obst { shape: Shape::Circle { c: (5.0, 0.4), r: 0.3 }, layer: 0, net: 1, clr: 0.2, bb: (4.7, 0.1, 5.3, 0.7) }];
        assert!(solve(&mut nodes, &items, &obst, 3.0, 60, 0.01));
        // the middle moved down, away from the obstacle, and the gap is legal now
        assert!(nodes[1].pos.1 < -0.05, "{:?}", nodes[1].pos);
        let h = measure(Core::Sg(nodes[0].pos, nodes[1].pos), 0.1, &obst[0].shape);
        assert!(h.gap >= 0.2 - 1e-6, "{}", h.gap);
    }

    #[test]
    fn a_push_cascades_from_a_trace_to_the_next_one() {
        // trace A is crowded by a pad above and pushes trace B, which sits below it
        let mut nodes = vec![fixed((0.0, 0.0)), free((5.0, 0.0)), fixed((10.0, 0.0)), fixed((0.0, -0.6)), free((5.0, -0.6)), fixed((10.0, -0.6))];
        let items = vec![
            Item { net: 0, layer: 0, kind: Kind::Seg { n0: 0, n1: 1, hw: 0.1 }, clr: 0.2 },
            Item { net: 0, layer: 0, kind: Kind::Seg { n0: 1, n1: 2, hw: 0.1 }, clr: 0.2 },
            Item { net: 2, layer: 0, kind: Kind::Seg { n0: 3, n1: 4, hw: 0.1 }, clr: 0.2 },
            Item { net: 2, layer: 0, kind: Kind::Seg { n0: 4, n1: 5, hw: 0.1 }, clr: 0.2 },
        ];
        let obst = vec![Obst { shape: Shape::Circle { c: (5.0, 0.4), r: 0.3 }, layer: 0, net: 1, clr: 0.2, bb: (4.7, 0.1, 5.3, 0.7) }];
        assert!(solve(&mut nodes, &items, &obst, 3.0, 200, 0.01));
        assert!(nodes[1].pos.1 < 0.0 && nodes[4].pos.1 < nodes[1].pos.1, "{:?} {:?}", nodes[1].pos, nodes[4].pos);
    }

    #[test]
    fn nothing_that_cannot_move_is_moved() {
        let mut nodes = vec![fixed((0.0, 0.0)), fixed((10.0, 0.0))];
        let items = vec![Item { net: 0, layer: 0, kind: Kind::Seg { n0: 0, n1: 1, hw: 0.1 }, clr: 0.2 }];
        let obst = vec![Obst { shape: Shape::Circle { c: (5.0, 0.3), r: 0.3 }, layer: 0, net: 1, clr: 0.2, bb: (4.7, 0.0, 5.3, 0.6) }];
        assert!(!solve(&mut nodes, &items, &obst, 3.0, 20, 0.01));
    }
}
