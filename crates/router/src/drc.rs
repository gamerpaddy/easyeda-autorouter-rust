//! Independent geometric design-rule / connectivity check of a routed result.

use crate::engine::Router;
use crate::geom::*;
use std::collections::{HashMap, HashSet};

pub struct DrcReport {
    pub clearance_violations: Vec<String>,
    pub open_nets: Vec<String>,
    pub shorts: usize,
    /// where each clearance violation is (same order as `clearance_violations`)
    pub violation_pos: Vec<P>,
    /// net index and a pin of a cut-off island for each entry of `open_nets`
    pub open_pos: Vec<(usize, P)>,
}

fn find(uf: &mut Vec<usize>, mut x: usize) -> usize {
    while uf[x] != x {
        uf[x] = uf[uf[x]];
        x = uf[x];
    }
    x
}

pub fn check(r: &Router) -> DrcReport {
    let mut all: Vec<Prim> = vec![];
    for p in &r.pins {
        all.extend(p.prims.iter().cloned());
    }
    for n in &r.nets {
        all.extend(n.pre.dynp.iter().filter(|p| !p.trace_only).cloned());
        all.extend(n.r.dynp.iter().cloned());
    }
    all.extend(r.fixed.iter().cloned());
    let mut viol = vec![];
    let mut viol_pos: Vec<P> = vec![];
    let mut shorts = 0;
    let cs = 10.0;
    let key = |x: f64| (x / cs).floor() as i64;
    let mut buckets: HashMap<(i64, i64, usize), Vec<usize>> = HashMap::new();
    for (i, p) in all.iter().enumerate() {
        for bx in key(p.bb.0 - 3.0)..=key(p.bb.2 + 3.0) {
            for by in key(p.bb.1 - 3.0)..=key(p.bb.3 + 3.0) {
                buckets.entry((bx, by, p.layer)).or_default().push(i);
            }
        }
    }
    let mut seen = HashSet::new();
    let name = |p: &Prim| {
        if p.net >= 0 {
            r.nets[p.net as usize].name.clone()
        } else if p.net == -2 {
            "<edge>".to_string()
        } else {
            "<pad>".to_string()
        }
    };
    for list in buckets.values() {
        for (ai, &a) in list.iter().enumerate() {
            for &b in &list[ai + 1..] {
                let (pa, pb) = (&all[a], &all[b]);
                if (pa.net == pb.net && pa.net >= 0) || (pa.net < 0 && pb.net < 0) {
                    continue;
                }
                if !seen.insert((a.min(b), a.max(b))) {
                    continue;
                }
                let need = pa.clr.max(pb.clr);
                let gap = pa.gap(pb);
                if gap < need - 1e-3 {
                    if gap < 0.0 {
                        shorts += 1;
                    }
                    let mid = |p: &Prim| ((p.bb.0 + p.bb.2) / 2.0, (p.bb.1 + p.bb.3) / 2.0);
                    // locate it at the smaller of the two pieces
                    let size = |p: &Prim| (p.bb.2 - p.bb.0) + (p.bb.3 - p.bb.1);
                    viol_pos.push(if size(pa) <= size(pb) { mid(pa) } else { mid(pb) });
                    viol.push(format!(
                        "L{} {} vs {}: gap {:.3} < {:.3}",
                        pa.layer,
                        name(pa),
                        name(pb),
                        gap,
                        need
                    ));
                }
            }
        }
    }
    // connectivity per net
    let mut open = vec![];
    let mut open_pos: Vec<(usize, P)> = vec![];
    for (ni, n) in r.nets.iter().enumerate() {
        if n.pins.len() < 2 || n.skip {
            continue;
        }
        let isl = net_islands(r, ni);
        if isl.len() > 1 {
            // the smallest island is the one that got cut off
            let small = isl.iter().min_by_key(|i| i.pins.len()).unwrap();
            open_pos.push((ni, r.pins[small.pins[0]].c));
            open.push(format!("{} ({} islands)", r.nets[ni].name, isl.len()));
        }
    }
    DrcReport {
        clearance_violations: viol,
        open_nets: open,
        shorts,
        violation_pos: viol_pos,
        open_pos,
    }
}

/// Copper that is electrically one piece, as seen by the independent geometric check.
pub struct Island {
    /// indices into `Router::pins`
    pub pins: Vec<usize>,
    pub prims: Vec<Prim>,
}

/// Connected pieces of net `ni` that contain at least one of its pins.
pub fn net_islands(r: &Router, ni: usize) -> Vec<Island> {
    let n = &r.nets[ni];
    let mut items: Vec<Prim> = vec![];
    let mut pin_of: Vec<Option<usize>> = vec![];
    for (k, &pi) in n.pins.iter().enumerate() {
        for p in &r.pins[pi].prims {
            items.push(p.clone());
            pin_of.push(Some(k));
        }
    }
    for p in n
        .pre
        .dynp
        .iter()
        .filter(|p| !p.trace_only)
        .chain(n.r.dynp.iter())
    {
        items.push(p.clone());
        pin_of.push(None);
    }
    for pl in r.planes.iter().filter(|pl| pl.net == ni && !pl.synthetic) {
        items.push(Prim::new(
            pl.layer,
            Geo::Poly(pl.poly.clone()),
            ni as i32,
            0.0,
        ));
        pin_of.push(None);
    }
    let m = items.len();
    let mut uf: Vec<usize> = (0..m).collect();
    // sweep on x so large strokes counts stay cheap
    let mut order: Vec<usize> = (0..m).collect();
    order.sort_by(|&a, &b| items[a].bb.0.total_cmp(&items[b].bb.0));
    for (oi, &i) in order.iter().enumerate() {
        for &j in &order[oi + 1..] {
            if items[j].bb.0 > items[i].bb.2 + 1e-6 {
                break;
            }
            if items[i].layer == items[j].layer
                && items[i].bb.1 <= items[j].bb.3 + 1e-6
                && items[j].bb.1 <= items[i].bb.3 + 1e-6
                && items[i].gap(&items[j]) <= 1e-6
            {
                let (a, b) = (find(&mut uf, i), find(&mut uf, j));
                uf[a] = b;
            }
        }
    }
    let mut by_pin: Vec<Vec<usize>> = vec![vec![]; n.pins.len()];
    for i in 0..m {
        if let Some(k) = pin_of[i] {
            by_pin[k].push(i);
        }
    }
    for v in &by_pin {
        for w in v.windows(2) {
            let (a, b) = (find(&mut uf, w[0]), find(&mut uf, w[1]));
            uf[a] = b;
        }
    }
    let mut by_pos: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for i in 0..m {
        if let Geo::Circle { c, .. } = items[i].geo {
            if pin_of[i].is_none() {
                by_pos
                    .entry(((c.0 * 1000.0).round() as i64, (c.1 * 1000.0).round() as i64))
                    .or_default()
                    .push(i);
            }
        }
    }
    for v in by_pos.values() {
        for w in v.windows(2) {
            let (a, b) = (find(&mut uf, w[0]), find(&mut uf, w[1]));
            uf[a] = b;
        }
    }
    // the two pads of a jumper resistor are one piece electrically (the part joins them)
    for j in &n.r.jumpers {
        let [pa, pb] = j.pads(ni as i32, 0.0);
        let find_pad = |p: &Prim| {
            items.iter().position(|it| it.layer == p.layer && matches!((&it.geo, &p.geo), (Geo::Poly(a), Geo::Poly(b)) if a == b))
        };
        if let (Some(a), Some(b)) = (find_pad(&pa), find_pad(&pb)) {
            let (a, b) = (find(&mut uf, a), find(&mut uf, b));
            uf[a] = b;
        }
    }
    let mut groups: HashMap<usize, Island> = HashMap::new();
    let mut order_roots: Vec<usize> = vec![];
    for k in 0..n.pins.len() {
        if by_pin[k].is_empty() {
            continue;
        }
        let root = find(&mut uf, by_pin[k][0]);
        let e = groups.entry(root).or_insert_with(|| {
            order_roots.push(root);
            Island {
                pins: vec![],
                prims: vec![],
            }
        });
        e.pins.push(n.pins[k]);
    }
    for i in 0..m {
        let root = find(&mut uf, i);
        if let Some(e) = groups.get_mut(&root) {
            e.prims.push(items[i].clone());
        }
    }
    order_roots
        .into_iter()
        .filter_map(|r| groups.remove(&r))
        .collect()
}
