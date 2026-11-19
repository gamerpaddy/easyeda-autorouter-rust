//! Differential pair routing: a single "swath" centreline is routed with a probe wide enough
//! for both traces and the gap, then split into two parallel offset traces. If the coupled
//! result is not clean (verified geometrically) the pair falls back to two independent nets.

use crate::engine::*;
use crate::geom::*;
use std::collections::HashSet;

type Pt = P;

fn normal(a: Pt, b: Pt) -> Pt {
    let d = dist(a, b).max(1e-12);
    (-(b.1 - a.1) / d, (b.0 - a.0) / d)
}

pub(crate) fn simplify(pts: &[Pt]) -> Vec<Pt> {
    let mut out: Vec<Pt> = vec![];
    for &p in pts {
        while out.len() >= 2 {
            let (a, b) = (out[out.len() - 2], out[out.len() - 1]);
            let cr = (b.0 - a.0) * (p.1 - b.1) - (b.1 - a.1) * (p.0 - b.0);
            if cr.abs() < 1e-9 && (b.0 - a.0) * (p.0 - b.0) + (b.1 - a.1) * (p.1 - b.1) > 0.0 {
                out.pop();
            } else {
                break;
            }
        }
        if out.last().map_or(true, |&q| dist(q, p) > 1e-9) {
            out.push(p);
        }
    }
    out
}

fn offset_line(pts: &[Pt], s: f64, off: f64) -> Option<Vec<Pt>> {
    let n = pts.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let nm = if i == 0 {
            normal(pts[0], pts[1])
        } else if i == n - 1 {
            normal(pts[n - 2], pts[n - 1])
        } else {
            let (n1, n2) = (normal(pts[i - 1], pts[i]), normal(pts[i], pts[i + 1]));
            let k = 1.0 + n1.0 * n2.0 + n1.1 * n2.1;
            if k < 0.3 {
                return None; // too sharp
            }
            ((n1.0 + n2.0) / k, (n1.1 + n2.1) / k)
        };
        out.push((pts[i].0 + s * off * nm.0, pts[i].1 + s * off * nm.1));
    }
    Some(out)
}

pub struct PairReport {
    pub p: String,
    pub n: String,
    pub coupled: bool,
    pub len_p: f64,
    pub len_n: f64,
    pub matched: bool,
    /// "coupled" (swath), "guided" (follower kept parallel to its lead) or "independent"
    pub kind: &'static str,
    /// member net indices
    pub nets: Vec<usize>,
    /// a pin of the shortest member (where to look)
    pub pos: P,
}

impl Router {
    /// All nets that route together with `n` (itself included); just `[n]` when ungrouped.
    pub fn group_members(&self, n: usize) -> Vec<usize> {
        if let Some(m) = self.nets[n].pair {
            return vec![n.min(m), n.max(m)];
        }
        if let Some(b) = self.nets[n].bundle {
            return self.bundles[b].clone();
        }
        vec![n]
    }

    fn report_of(&self, members: &[usize]) -> PairReport {
        let lens: Vec<f64> = members.iter().map(|&m| self.wire_length(m)).collect();
        let hi = lens.iter().cloned().fold(f64::MIN, f64::max);
        let lo = lens.iter().cloned().fold(f64::MAX, f64::min);
        let coupled = members.iter().all(|&m| self.nets[m].coupled);
        let guided = members
            .iter()
            .all(|&m| self.nets[m].coupled || self.nets[m].guided);
        let tol = self.cfg.tolerance_for(&self.nets[members[0]].name);
        let connected = members.iter().all(|&m| self.nets[m].unconnected.is_empty());
        let shortest = members
            .iter()
            .copied()
            .min_by(|&a, &b| self.wire_length(a).total_cmp(&self.wire_length(b)))
            .unwrap();
        let pos = self.nets[shortest]
            .pins
            .first()
            .map(|&p| self.pins[p].c)
            .unwrap_or((0.0, 0.0));
        PairReport {
            nets: members.to_vec(),
            pos,
            p: if members.len() == 2 {
                self.nets[members[0]].name.clone()
            } else {
                members
                    .iter()
                    .map(|&m| self.nets[m].name.clone())
                    .collect::<Vec<_>>()
                    .join(",")
            },
            n: if members.len() == 2 {
                self.nets[members[1]].name.clone()
            } else {
                String::new()
            },
            coupled: guided,
            len_p: hi,
            len_n: lo,
            matched: guided && connected && hi - lo <= tol,
            kind: if coupled {
                "coupled"
            } else if guided {
                "guided"
            } else {
                "independent"
            },
        }
    }

    pub fn pair_report(&self) -> Vec<PairReport> {
        let mut v = vec![];
        for (i, n) in self.nets.iter().enumerate() {
            if let Some(m) = n.pair {
                if i < m {
                    v.push(self.report_of(&[i, m]));
                }
            }
        }
        for b in &self.bundles {
            v.push(self.report_of(b));
        }
        v
    }

    /// Cells near the copper of `leads` are cheap for the follower `n`, others expensive.
    fn build_guide(&self, leads: &[usize], n: usize) -> Guide {
        let g = &self.grid;
        let nn = g.n();
        let mut near = vec![false; nn * g.nl];
        let gap = self.cfg.pair_gap.unwrap_or(self.nets[n].clr);
        let reach = |w: f64| ((w * 0.5 + gap + self.nets[n].width * 1.5) / g.g).ceil() as i32 + 1;
        let mut mark = |l: usize, p: P, rad: i32| {
            let (cx, cy) = g.cell(p);
            for y in (cy - rad).max(0)..=(cy + rad).min(g.ny as i32 - 1) {
                for x in (cx - rad).max(0)..=(cx + rad).min(g.nx as i32 - 1) {
                    near[l * nn + y as usize * g.nx + x as usize] = true;
                }
            }
        };
        for &m in leads {
            for w in &self.nets[m].r.wires {
                let rad = reach(w.width);
                for s in w.pts.windows(2) {
                    let steps = (dist(s[0], s[1]) / g.g).ceil().max(1.0) as usize;
                    for i in 0..=steps {
                        let t = i as f64 / steps as f64;
                        mark(
                            w.layer,
                            (
                                s[0].0 + t * (s[1].0 - s[0].0),
                                s[0].1 + t * (s[1].1 - s[0].1),
                            ),
                            rad,
                        );
                    }
                }
            }
            for &v in &self.nets[m].r.vias {
                for l in 0..g.nl {
                    mark(l, v, reach(self.nets[m].via_r * 2.0));
                }
            }
        }
        Guide {
            near,
            bonus: 0.7,
            penalty: 1.6,
        }
    }

    /// Route a group one net after another; every follower is pulled towards the nets already
    /// routed so the traces run side by side instead of wandering apart.
    fn route_guided(&mut self, members: &[usize], allow_rip: bool) -> Vec<usize> {
        for &m in members {
            if !self.nets[m].r.wires.is_empty() || !self.nets[m].r.vias.is_empty() {
                self.rip_net(m);
            }
        }
        let mut ripped = vec![];
        let mut done: Vec<usize> = vec![];
        for &m in members {
            if done.is_empty() {
                ripped.extend(self.route_net_plain(m, allow_rip));
            } else {
                self.guide = Some(self.build_guide(&done, m));
                ripped.extend(self.route_net_plain(m, false));
                self.guide = None;
            }
            done.push(m);
        }
        for &m in members {
            self.nets[m].guided = true;
        }
        ripped
    }

    /// Route a net (or its whole differential group).
    pub fn route_net(&mut self, n: usize, allow_rip: bool) -> Vec<usize> {
        if self.cfg.pairs {
            let members: Vec<usize> = self
                .group_members(n)
                .into_iter()
                .filter(|&m| !self.nets[m].skip && self.nets[m].comps.len() >= 2)
                .collect();
            if members.len() >= 2 {
                let healthy = members.iter().all(|&m| {
                    (self.nets[m].coupled || self.nets[m].guided)
                        && self.nets[m].unconnected.is_empty()
                });
                if healthy {
                    return vec![];
                }
                let simple = |x: usize| {
                    let ns = &self.nets[x];
                    ns.pins.len() >= 2
                        && ns.pins.len() <= 16
                        && ns.comps.len() == ns.pins.len()
                        && ns.pre.wires.is_empty()
                        && ns.pre.vias.is_empty()
                        && ns.plane.is_none()
                };
                if members.len() == 2
                    && simple(members[0])
                    && simple(members[1])
                    && self.nets[members[0]].pins.len() == self.nets[members[1]].pins.len()
                {
                    let (a, b) = (members[0], members[1]);
                    for &m in &[a, b] {
                        if !self.nets[m].r.wires.is_empty() || !self.nets[m].r.vias.is_empty() {
                            self.rip_net(m);
                        }
                    }
                    self.cur_net = a;
                    let ok = if self.nets[a].pins.len() == 2 {
                        self.route_pair(a, b)
                    } else {
                        self.route_pair_tree(a, b)
                    };
                    self.cur_net = usize::MAX;
                    if ok {
                        return vec![];
                    }
                }
                return self.route_guided(&members, allow_rip);
            }
        }
        self.route_net_plain(n, allow_rip)
    }

    fn route_pair_tree(&mut self, p: usize, q: usize) -> bool {
        let pp = self.nets[p].pins.clone();
        let qp = self.nets[q].pins.clone();
        let mut candidates = Vec::new();
        for &a in &pp {
            for &b in &qp {
                candidates.push((dist(self.pins[a].c, self.pins[b].c), a, b));
            }
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for (_, a, b) in candidates {
            if !pairs.iter().any(|&(x, y)| x == a || y == b) {
                pairs.push((a, b));
            }
        }
        let mid = |i: usize| {
            let (a, b) = pairs[i];
            (
                (self.pins[a].c.0 + self.pins[b].c.0) / 2.0,
                (self.pins[a].c.1 + self.pins[b].c.1) / 2.0,
            )
        };
        let mut connected = vec![0];
        let mut edges = Vec::new();
        while connected.len() < pairs.len() {
            let mut best = (f64::INFINITY, 0, 0);
            for &a in &connected {
                for b in 0..pairs.len() {
                    if !connected.contains(&b) {
                        let d = dist(mid(a), mid(b));
                        if d < best.0 {
                            best = (d, a, b);
                        }
                    }
                }
            }
            connected.push(best.2);
            edges.push((best.1, best.2));
        }
        for (a, b) in edges {
            let oldp = std::mem::take(&mut self.nets[p].r);
            let oldq = std::mem::take(&mut self.nets[q].r);
            // Keep earlier branches as fixed obstacles while routing another coupled swath.
            let fixed_len = self.fixed.len();
            self.fixed
                .extend(oldp.dynp.iter().chain(oldq.dynp.iter()).cloned());
            self.nets[p].pins = vec![pairs[a].0, pairs[b].0];
            self.nets[q].pins = vec![pairs[a].1, pairs[b].1];
            let ok = self.route_pair(p, q);
            self.nets[p].pins = pp.clone();
            self.nets[q].pins = qp.clone();
            self.fixed.truncate(fixed_len);
            for (n, old) in [(p, oldp), (q, oldq)] {
                self.nets[n].r.wires.extend(old.wires);
                self.nets[n].r.vias.extend(old.vias);
                self.nets[n].r.dynp.extend(old.dynp);
                self.nets[n].r.tree.extend(old.tree);
            }
            if !ok {
                self.rip_net(p);
                self.rip_net(q);
                return false;
            }
        }
        true
    }

    fn route_pair(&mut self, p: usize, q: usize) -> bool {
        let (pp, qp) = (self.nets[p].pins.clone(), self.nets[q].pins.clone());
        let d = |x: usize, y: usize| dist(self.pins[x].c, self.pins[y].c);
        let straight = d(pp[0], qp[0]) + d(pp[1], qp[1]);
        let cross = d(pp[0], qp[1]) + d(pp[1], qp[0]);
        let (a0, a1) = (pp[0], pp[1]);
        let (b0, b1) = if straight <= cross {
            (qp[0], qp[1])
        } else {
            (qp[1], qp[0])
        };
        let mid = |x: usize, y: usize| {
            (
                (self.pins[x].c.0 + self.pins[y].c.0) / 2.0,
                (self.pins[x].c.1 + self.pins[y].c.1) / 2.0,
            )
        };
        let (m0, m1) = (mid(a0, b0), mid(a1, b1));
        let layers_of =
            |pi: usize| -> HashSet<usize> { self.pins[pi].cells.iter().map(|c| c.0).collect() };
        let common = |x: usize, y: usize| -> Vec<usize> {
            let (lx, ly) = (layers_of(x), layers_of(y));
            let mut v: Vec<usize> = lx
                .intersection(&ly)
                .copied()
                .filter(|&l| !self.excl[l])
                .collect();
            v.sort();
            v
        };
        let (c0, c1) = (common(a0, b0), common(a1, b1));
        if c0.is_empty() || c1.is_empty() {
            return false;
        }
        self.stamp_net(p, -1);
        self.stamp_net(q, -1);
        let fi = (self.nets[p].ptf, self.nets[p].pvf);
        let breakout =
            |this: &Router, ax: usize, bx: usize, m: Pt, layers: &[usize]| -> Vec<Cell> {
                // axis between the two pads and its perpendicular (the exit direction)
                let (ca, cb) = (this.pins[ax].c, this.pins[bx].c);
                let dv = dist(ca, cb).max(1e-9);
                let v = ((cb.0 - ca.0) / dv, (cb.1 - ca.1) / dv);
                let nrm = (-v.1, v.0);
                let np = &this.nets[p];
                let nq = &this.nets[q];
                let w = np.width.max(nq.width);
                let clr = np.clr.max(nq.clr);
                let off = (w + this.cfg.pair_gap.unwrap_or(clr)) / 2.0;
                let mut cells = vec![];
                for sign in [1.0, -1.0] {
                    let dir = (nrm.0 * sign, nrm.1 * sign);
                    let mut ext: f64 = 0.0;
                    for &pi in &[ax, bx] {
                        for pr in &this.pins[pi].prims {
                            match &pr.geo {
                                Geo::Circle { c, r } => {
                                    ext = ext.max((c.0 - m.0) * dir.0 + (c.1 - m.1) * dir.1 + r)
                                }
                                Geo::Poly(pts) => {
                                    for q in pts {
                                        ext = ext.max((q.0 - m.0) * dir.0 + (q.1 - m.1) * dir.1);
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                    let l = ext + clr + off + w / 2.0 + this.grid.g * 2.0;
                    let bpt = (m.0 + dir.0 * l, m.1 + dir.1 * l);
                    let (x, y) = this.grid.cell(bpt);
                    if x < 0 || y < 0 || x >= this.grid.nx as i32 || y >= this.grid.ny as i32 {
                        continue;
                    }
                    for &layer in layers {
                        cells.push((layer, x, y));
                    }
                }
                cells
            };
        let seeds: Vec<Cell> = breakout(self, a0, b0, m0, &c0);
        let targets: Vec<Cell> = breakout(self, a1, b1, m1, &c1);
        if seeds.is_empty() || targets.is_empty() {
            self.stamp_net(p, 1);
            self.stamp_net(q, 1);
            return false;
        }
        let (x0, y0) = (seeds[0].1, seeds[0].2);
        let (x1, y1) = (targets[0].1, targets[0].2);
        let (nx, ny) = (self.grid.nx as i32, self.grid.ny as i32);
        let mar = (40.0 / self.grid.g) as i32;
        let win = (
            (x0.min(x1) - mar).max(0),
            (y0.min(y1) - mar).max(0),
            (x0.max(x1) + mar).min(nx - 1),
            (y0.max(y1) + mar).min(ny - 1),
        );
        let mut path = self.astar(fi, &seeds, &targets, false, win, 0.0);
        if path.is_none() {
            path = self.astar(fi, &seeds, &targets, false, (0, 0, nx - 1, ny - 1), 0.0);
        }
        let mut ok = false;
        let verbose = self.cfg.verbose;
        if path.is_none() && verbose {
            eprintln!(
                "  pair {}/{}: swath search found no path",
                self.nets[p].name, self.nets[q].name
            );
        }
        if let Some(path) = path {
            for variant in 0..16 {
                let g = self.pair_geometry(p, q, &path, (a0, a1), (b0, b1), m0, m1, variant);
                if g.is_none() && verbose {
                    eprintln!(
                        "  pair {}/{}: geometry rejected (corner too sharp / run too short)",
                        self.nets[p].name, self.nets[q].name
                    );
                }
                if let Some(geo) = g {
                    let (wp, vp, wq, vq) = geo;
                    let build = |r: &Router, n: usize, wires: &[Wire], vias: &[P]| -> Vec<Prim> {
                        let ns = &r.nets[n];
                        let mut v = vec![];
                        for w in wires {
                            for s in w.pts.windows(2) {
                                v.push(Prim::new(
                                    w.layer,
                                    Geo::Seg {
                                        a: s[0],
                                        b: s[1],
                                        hw: w.width / 2.0,
                                    },
                                    n as i32,
                                    ns.clr,
                                ));
                            }
                        }
                        for &c in vias {
                            for l in 0..r.grid.nl {
                                v.push(Prim::new(
                                    l,
                                    Geo::Circle { c, r: ns.via_r },
                                    n as i32,
                                    ns.clr,
                                ));
                            }
                        }
                        v
                    };
                    let (dp, dq) = (build(self, p, &wp, &vp), build(self, q, &wq, &vq));
                    let mut all = dp.clone();
                    all.extend(dq.iter().cloned());
                    let conflict = self.prims_conflict(&all);
                    if let (Some(c), true) = (&conflict, verbose && variant == 15) {
                        eprintln!(
                            "  pair {}/{}: coupled geometry violates clearance: {}",
                            self.nets[p].name, self.nets[q].name, c
                        );
                    }
                    if conflict.is_none() {
                        self.nets[p].r = Routed {
                            wires: wp,
                            vias: vp,
                            tree: vec![],
                            dynp: dp,
                            jumpers: vec![],
                        };
                        self.nets[q].r = Routed {
                            wires: wq,
                            vias: vq,
                            tree: vec![],
                            dynp: dq,
                            jumpers: vec![],
                        };
                        for &x in &[p, q] {
                            self.nets[x].unconnected = vec![];
                            self.nets[x].coupled = true;
                        }
                        ok = true;
                        break;
                    }
                }
            }
        }
        self.stamp_net(p, 1);
        self.stamp_net(q, 1);
        ok
    }

    /// true when any new primitive violates clearance against existing copper or another new net.
    pub(crate) fn prims_conflict(&self, newp: &[Prim]) -> Option<String> {
        let hit = |a: &Prim, o: &Prim| -> bool {
            if o.trace_only && matches!(a.geo, Geo::Circle { .. }) {
                return false;
            }
            if o.layer != a.layer || (o.net == a.net && o.net >= 0) {
                return false;
            }
            let need = a.clr.max(o.clr);
            if a.bb.0 > o.bb.2 + need
                || o.bb.0 > a.bb.2 + need
                || a.bb.1 > o.bb.3 + need
                || o.bb.1 > a.bb.3 + need
            {
                return false;
            }
            a.gap(o) < need - 1e-3
        };
        let nm = |x: &Prim| {
            if x.net >= 0 {
                self.nets[x.net as usize].name.clone()
            } else if x.net == -2 {
                "<edge>".to_string()
            } else {
                "<pad>".to_string()
            }
        };
        let desc = |a: &Prim, o: &Prim| {
            format!(
                "L{} {} {:?} vs {} {:?} gap {:.3}",
                a.layer,
                nm(a),
                a.bb,
                nm(o),
                o.bb,
                a.gap(o)
            )
        };
        for a in newp {
            if !self.cfg.allow_vias_in_pads && matches!(a.geo, Geo::Circle { .. }) {
                for pin in &self.pins {
                    if pin
                        .prims
                        .iter()
                        .any(|o| o.layer == a.layer && a.gap(o) < 0.0)
                    {
                        return Some("via overlaps pad".into());
                    }
                }
            }
            for o in self.fixed.iter() {
                if hit(a, o) {
                    return Some(desc(a, o));
                }
            }
            for pin in &self.pins {
                for o in &pin.prims {
                    if hit(a, o) {
                        return Some(desc(a, o));
                    }
                }
            }
            for n in &self.nets {
                for o in n.r.dynp.iter().chain(n.pre.dynp.iter()) {
                    if hit(a, o) {
                        return Some(desc(a, o));
                    }
                }
            }
            for o in newp {
                if hit(a, o) {
                    return Some(desc(a, o));
                }
            }
        }
        None
    }

    #[allow(clippy::type_complexity)]
    fn pair_geometry(
        &self,
        p: usize,
        q: usize,
        path: &[Cell],
        a: (usize, usize),
        b: (usize, usize),
        m0: Pt,
        m1: Pt,
        variant: u8,
    ) -> Option<(Vec<Wire>, Vec<P>, Vec<Wire>, Vec<P>)> {
        let g = &self.grid;
        let mut runs: Vec<(usize, Vec<Pt>)> = vec![];
        let mut vias_c: Vec<Pt> = vec![];
        for (i, &(l, x, y)) in path.iter().enumerate() {
            let c = g.center(x, y);
            if i == 0 || path[i - 1].0 != l {
                if i > 0 {
                    vias_c.push(c);
                }
                runs.push((l, vec![]));
            }
            runs.last_mut().unwrap().1.push(c);
        }
        // Offset only the octilinear grid trunk. Off-grid pad-pair midpoints
        // produce tiny arbitrary-angle bends and crossing offsets at the ends.
        let _ = m1;
        for r in runs.iter_mut() {
            r.1 = simplify(
                &r.1.windows(2)
                    .flat_map(|s| crate::finish::octilinear(s[0], s[1]))
                    .collect::<Vec<_>>(),
            );
            if r.1.len() < 2 {
                return None;
            }
        }
        let (np, nq) = (&self.nets[p], &self.nets[q]);
        let w = np.width.max(nq.width);
        let clr = np.clr.max(nq.clr);
        let gap = self.cfg.pair_gap.unwrap_or(clr);
        let off = (w + gap) / 2.0;
        let vr = np.via_r.max(nq.via_r);
        let hv = (2.0 * vr + clr + 0.05) / 2.0;
        let n0 = normal(runs[0].1[0], runs[0].1[1]);
        let pa = self.pins[a.0].c;
        let left_is_p = (pa.0 - m0.0) * n0.0 + (pa.1 - m0.1) * n0.1 > 0.0;
        let mut res: Vec<(Vec<Wire>, Vec<P>)> = vec![];
        for (net, start_pin, end_pin, s) in [
            (p, a.0, a.1, if left_is_p { 1.0 } else { -1.0 }),
            (q, b.0, b.1, if left_is_p { -1.0 } else { 1.0 }),
        ] {
            let mut wires = vec![];
            let mut vias = vec![];
            let nr = runs.len();
            for (k, (layer, pts)) in runs.iter().enumerate() {
                let mut line = offset_line(pts, s, off)?;
                if k == 0 {
                    line.insert(0, self.pins[start_pin].c);
                } else {
                    let prev = &runs[k - 1].1;
                    let n_in = normal(prev[prev.len() - 2], prev[prev.len() - 1]);
                    let c = vias_c[k - 1];
                    line.insert(0, (c.0 + s * hv * n_in.0, c.1 + s * hv * n_in.1));
                }
                if k == nr - 1 {
                    line.push(self.pins[end_pin].c);
                } else {
                    let n_in = normal(pts[pts.len() - 2], pts[pts.len() - 1]);
                    let c = vias_c[k];
                    let v = (c.0 + s * hv * n_in.0, c.1 + s * hv * n_in.1);
                    line.push(v);
                    vias.push(v);
                }
                let bits = if net == p { variant & 3 } else { variant >> 2 };
                let mut oct = Vec::new();
                for (i, s) in line.windows(2).enumerate() {
                    let reverse =
                        (i == 0 && bits & 1 != 0) || (i + 2 == line.len() && bits & 2 != 0);
                    let mut pts = if reverse {
                        crate::finish::octilinear(s[1], s[0])
                    } else {
                        crate::finish::octilinear(s[0], s[1])
                    };
                    if reverse {
                        pts.reverse();
                    }
                    oct.extend(pts);
                }
                wires.push(Wire {
                    fill: false,
                    layer: *layer,
                    width: self.nets[net].width,
                    pts: simplify(&oct),
                });
            }
            res.push((wires, vias));
        }
        let (wq, vq) = res.pop().unwrap();
        let (wp, vp) = res.pop().unwrap();
        Some((wp, vp, wq, vq))
    }
}
