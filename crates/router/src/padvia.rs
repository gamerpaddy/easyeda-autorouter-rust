//! Vias that tie the pads of a plane net to its plane: placed before any routing, in the pad,
//! half overlapping its edge, or just beside it (with a short stub), so that those pads never need
//! a trace. They work for nets that are not routed ("no route") as well: the plane is theirs.

use crate::engine::*;
use crate::geom::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DropKind {
    /// centre of the via inside the pad
    Inside,
    /// centre of the via on the edge of the pad, so the via is half over the pad
    Edge,
    /// beside the pad, joined by a short stub
    Beside,
}

#[derive(Clone, Debug)]
pub struct PlaneDrop {
    pub net: usize,
    pub pin: usize,
    pub via: P,
    /// (layer, from, to, width) of the stub that joins a via beside the pad
    pub stub: Option<(usize, P, P, f64)>,
    pub kind: DropKind,
}

struct Cand {
    pos: P,
    score: f64,
}

impl Router {
    /// Does the pad touch the generated plane `pi` (a pad cell on the plane's layer, inside it)?
    pub(crate) fn pin_on_plane(&self, pin: usize, pi: usize) -> bool {
        let l = self.planes[pi].layer;
        let nx = self.grid.nx;
        if self.plane_id[l].is_empty() {
            return false;
        }
        self.pins[pin]
            .cells
            .iter()
            .any(|&(cl, x, y)| cl == l && self.plane_id[l][y as usize * nx + x as usize] as usize == pi + 1)
    }

    /// Mask (in the via fields) that keeps new vias off the pads of `pins`, for the pad-via mode
    /// in use: not at all in a pad (default), or the pad's own area only (`vias_at_pad_edge`: the
    /// centre of a via stays outside the pad, the via may still overlap it), or nothing.
    pub(crate) fn pad_via_mask(&mut self, pins: &[usize], delta: i32) {
        if self.cfg.allow_vias_in_pads {
            return;
        }
        let grid = self.grid.clone();
        let edge = self.cfg.vias_at_pad_edge;
        for &pi in pins {
            for pr in &self.pins[pi].prims {
                for f in self.fields.iter_mut().filter(|f| f.via) {
                    if edge {
                        f.stamp_t(&grid, pr, delta, false, 1e-7);
                    } else {
                        f.stamp(&grid, pr, delta, false);
                    }
                }
            }
        }
    }

    /// A new via (and stub) may not violate clearance against what is on the board or the drops
    /// placed so far.
    fn drop_conflict(&mut self, prims: &[Prim], placed: &[Prim], kind: DropKind) -> bool {
        let saved = (self.cfg.allow_vias_in_pads, self.cfg.vias_at_pad_edge);
        self.cfg.allow_vias_in_pads = kind == DropKind::Inside;
        self.cfg.vias_at_pad_edge = kind == DropKind::Edge;
        let bad = self.prims_conflict(prims).is_some();
        self.cfg.allow_vias_in_pads = saved.0;
        self.cfg.vias_at_pad_edge = saved.1;
        if bad {
            return true;
        }
        for a in prims {
            for o in placed {
                if o.layer != a.layer || o.net == a.net {
                    continue;
                }
                let need = a.clr.max(o.clr);
                if a.bb.0 > o.bb.2 + need || o.bb.0 > a.bb.2 + need || a.bb.1 > o.bb.3 + need || o.bb.1 > a.bb.3 + need {
                    continue;
                }
                if a.gap(o) < need - 1e-3 {
                    return true;
                }
            }
        }
        false
    }

    /// Give every SMD pad of a plane net that is not on its plane a via into the plane. Tried in
    /// this order: inside the pad (only when allowed), half over the edge of the pad (when
    /// allowed), beside the pad with a short stub. A pad where none fits is left to the router.
    pub(crate) fn place_plane_drops(&mut self) {
        if self.cfg.single_sided || self.grid.nl < 2 || std::env::var_os("AR_NO_DROPS").is_some() {
            return;
        }
        let nx = self.grid.nx as i32;
        let ny = self.grid.ny as i32;
        let mm = 1.0 / self.unit_mm;
        let step = 0.05 * mm;
        let mut placed: Vec<Prim> = vec![];
        let t0 = std::time::Instant::now();
        let mut kinds_all = vec![];
        if self.cfg.allow_vias_in_pads {
            kinds_all.push(DropKind::Inside);
        }
        if self.cfg.allow_vias_in_pads || self.cfg.vias_at_pad_edge {
            kinds_all.push(DropKind::Edge);
        }
        kinds_all.push(DropKind::Beside);
        for n in 0..self.nets.len() {
            let Some(pi) = self.planes.iter().position(|p| p.net == n && p.synthetic && !p.filler) else {
                continue;
            };
            // vias beside a pad (never touching it) are for nets that are not routed; the router
            // places those of the others itself, after the signals
            if !self.cfg.allow_vias_in_pads && !self.cfg.vias_at_pad_edge && !self.nets[n].skip {
                continue;
            }
            // a net that is to have several vias side by side at each layer change gets them from
            // the router, not from here
            if self.cfg.net_overrides.get(&self.nets[n].name).and_then(|o| o.via_count).is_some_and(|c| c > 1) {
                continue;
            }
            let l = self.planes[pi].layer;
            let (via_r, clr) = (self.nets[n].via_r, self.nets[n].clr);
            let narrow = (self.fields[self.nets[n].narrow_tf].p * 2.0).min(self.nets[n].width);
            let widths: Vec<f64> = if self.nets[n].width - narrow > 1e-9 { vec![self.nets[n].width, narrow] } else { vec![narrow] };
            let pins = self.nets[n].pins.clone();
            for pin in pins {
                if self.pin_on_plane(pin, pi) {
                    continue;
                }
                let prs: Vec<Prim> = self.pins[pin].prims.clone();
                let Some(first) = prs.first() else { continue };
                let lp = first.layer;
                // through-hole pads (several layers) are left to the router
                if prs.iter().any(|p| p.layer != lp) {
                    continue;
                }
                let c0 = self.pins[pin].c;
                let pad_d = |pos: P| prs.iter().map(|p| p.dist(pos)).fold(f64::MAX, f64::min);
                let half = prs.iter().map(|p| (p.bb.2 - p.bb.0).max(p.bb.3 - p.bb.1)).fold(0.0, f64::max) / 2.0;
                let rmax = half * 1.5 + via_r + clr + 1.5 * mm;
                // per direction: where the ray leaves the pad
                let ndir = 24;
                let mut rays: Vec<(P, f64, Vec<(P, f64)>)> = vec![]; // (dir, s0, [(pos, d)])
                for k in 0..ndir {
                    let a = k as f64 * std::f64::consts::TAU / ndir as f64;
                    let dir = (a.cos(), a.sin());
                    let mut pts: Vec<(P, f64)> = vec![];
                    let mut s0 = None;
                    let mut s = 0.0;
                    while s <= rmax {
                        let pos = (c0.0 + dir.0 * s, c0.1 + dir.1 * s);
                        let d = pad_d(pos);
                        if s0.is_none() && d > 1e-6 {
                            s0 = Some(s);
                        }
                        if s0.is_some() {
                            pts.push((pos, d));
                        }
                        s += step;
                    }
                    rays.push((dir, s0.unwrap_or(0.0), pts));
                }
                let mut done = false;
                let dbg = std::env::var_os("AR_DEBUG_DROP").is_some();
                let mut stat = (0usize, 0usize, 0usize); // candidates, off the plane, conflicts
                for &kind in &kinds_all {
                    let mut cands: Vec<Cand> = vec![];
                    match kind {
                        DropKind::Inside => {
                            if prs.iter().any(|p| p.dist(c0) <= 1e-9) {
                                cands.push(Cand { pos: c0, score: 0.0 });
                            }
                        }
                        DropKind::Edge => {
                            for (_, _, pts) in &rays {
                                for (pos, d) in pts.iter() {
                                    if *d <= 0.6 * via_r {
                                        cands.push(Cand { pos: *pos, score: *d });
                                    }
                                }
                            }
                        }
                        DropKind::Beside => {
                            let gap = via_r + clr + 0.02 * mm;
                            for (_, _, pts) in &rays {
                                for (pos, d) in pts.iter() {
                                    if *d >= gap && *d <= gap + 1.2 * mm {
                                        cands.push(Cand { pos: *pos, score: *d });
                                    }
                                }
                            }
                        }
                    }
                    cands.sort_by(|a, b| a.score.total_cmp(&b.score));
                    let mut seen: std::collections::HashSet<(i64, i64)> = std::collections::HashSet::new();
                    let mut tries = 0;
                    for cand in cands {
                        let key = ((cand.pos.0 / step * 0.5).round() as i64, (cand.pos.1 / step * 0.5).round() as i64);
                        if !seen.insert(key) {
                            continue;
                        }
                        tries += 1;
                        if tries > 250 || t0.elapsed().as_secs() > 20 {
                            break;
                        }
                        let (cx, cy) = self.grid.cell(cand.pos);
                        if cx < 0 || cy < 0 || cx >= nx || cy >= ny {
                            continue;
                        }
                        // the via must land in this net's plane
                        stat.0 += 1;
                        if self.plane_id[l][cy as usize * self.grid.nx + cx as usize] as usize != pi + 1 {
                            stat.1 += 1;
                            continue;
                        }
                        let vias: Vec<Prim> = (0..self.grid.nl)
                            .map(|ly| Prim::new(ly, Geo::Circle { c: cand.pos, r: via_r }, n as i32, clr))
                            .collect();
                        let stubs: Vec<Option<f64>> = if kind == DropKind::Beside { widths.iter().map(|&w| Some(w)).collect() } else { vec![None] };
                        for sw in stubs {
                            let mut new = vias.clone();
                            if let Some(w) = sw {
                                new.push(Prim::new(lp, Geo::Seg { a: c0, b: cand.pos, hw: w / 2.0 }, n as i32, clr));
                            }
                            if self.drop_conflict(&new, &placed, kind) {
                                stat.2 += 1;
                                continue;
                            }
                            // accepted: fixed copper from now on (the net's own routing unstamps it)
                            let grid = self.grid.clone();
                            for pr in &new {
                                for f in self.fields.iter_mut() {
                                    f.stamp(&grid, pr, 1, false);
                                }
                            }
                            self.nets[n].pre.dynp.extend(new.iter().cloned());
                            placed.extend(new);
                            self.drops.push(PlaneDrop {
                                net: n,
                                pin,
                                via: cand.pos,
                                stub: sw.map(|w| (lp, c0, cand.pos, w)),
                                kind,
                            });
                            done = true;
                            break;
                        }
                        if done {
                            break;
                        }
                    }
                    if done {
                        break;
                    }
                }
                if dbg && !done {
                    eprintln!(
                        "plane via: {} pin {} found no place ({} candidates, {} off the plane, {} in conflict)",
                        self.nets[n].name, self.pins[pin].name, stat.0, stat.1, stat.2
                    );
                }
            }
        }
        if self.cfg.verbose && !self.drops.is_empty() {
            let k = |kind| self.drops.iter().filter(|d| d.kind == kind).count();
            eprintln!(
                "plane vias: {} pad(s) tied to their plane ({} in the pad, {} half over the pad edge, {} beside the pad)",
                self.drops.len(),
                k(DropKind::Inside),
                k(DropKind::Edge),
                k(DropKind::Beside)
            );
        }
    }

    /// Connected pieces (4-neighbour) of the cells of layer `l` for which `free` holds:
    /// (label per cell, 0 = not free, else piece number + 1; cell count per piece).
    pub(crate) fn free_pieces(&self, l: usize, free: &dyn Fn(usize) -> bool) -> (Vec<u32>, Vec<usize>) {
        let nn = self.grid.n();
        let (nx, ny) = (self.grid.nx, self.grid.ny);
        let _ = l;
        let mut label = vec![0u32; nn];
        let mut sizes: Vec<usize> = vec![];
        let mut q: Vec<usize> = vec![];
        for start in 0..nn {
            if label[start] != 0 || !free(start) {
                continue;
            }
            sizes.push(0);
            let id = sizes.len() as u32;
            label[start] = id;
            q.push(start);
            while let Some(c) = q.pop() {
                sizes[id as usize - 1] += 1;
                let (x, y) = (c % nx, c / nx);
                let mut visit = |cc: usize| {
                    if label[cc] == 0 && free(cc) {
                        label[cc] = id;
                        q.push(cc);
                    }
                };
                if x > 0 {
                    visit(c - 1);
                }
                if x + 1 < nx {
                    visit(c + 1);
                }
                if y > 0 {
                    visit(c - nx);
                }
                if y + 1 < ny {
                    visit(c + nx);
                }
            }
        }
        (label, sizes)
    }

    /// After routing: a via that ties a pad to a shared plane is only worth something when it
    /// lands in the main piece of the plane. Where other nets' vias and traces have cut the plane
    /// apart, a via in a small pocket reaches nothing: it is removed and the pad is routed like any
    /// other (to the main piece). Returns the number of vias taken back. Nets that are not routed
    /// keep theirs.
    pub(crate) fn revalidate_drops(&mut self) -> usize {
        if self.drops.is_empty() {
            return 0;
        }
        let mut nets: Vec<usize> = self.drops.iter().map(|d| d.net).collect();
        nets.sort_unstable();
        nets.dedup();
        let mut taken = 0;
        for n in nets {
            if self.nets[n].skip || self.cancelled() {
                continue;
            }
            let Some(pi) = self.planes.iter().position(|p| p.net == n && p.synthetic && !p.filler) else {
                continue;
            };
            let l = self.planes[pi].layer;
            // the same blocking as the copper area that is generated later (fill_copper): foreign
            // copper plus clearance plus half a stroke and 1.5 cells of margin
            let g = self.grid.clone();
            let g1 = Grid { nl: 1, ..g.clone() };
            let mut f = Field::new(&g1, self.nets[n].clr, g.g * 1.1 / 2.0 + g.g * 1.5, false);
            for pr in self
                .fixed
                .iter()
                .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                .chain(self.nets.iter().flat_map(|ns| ns.pre.dynp.iter().chain(ns.r.dynp.iter())))
            {
                if pr.layer == l && pr.net != n as i32 {
                    let mut q = pr.clone();
                    q.layer = 0;
                    f.stamp(&g1, &q, 1, false);
                }
            }
            let bad: Vec<usize> = {
                let pid = &self.plane_id[l];
                let want = (pi + 1) as u8;
                let free = |c: usize| pid[c] == want && !f.fx(c);
                let (label, sizes) = self.free_pieces(l, &free);
                let main = sizes.iter().enumerate().max_by_key(|(_, &s)| s).map(|(i, _)| i as u32 + 1).unwrap_or(0);
                let (via_r, clr) = (self.nets[n].via_r, self.nets[n].clr);
                self.drops
                    .iter()
                    .enumerate()
                    .filter(|(_, d)| d.net == n)
                    .filter(|(_, d)| {
                        // any cell of the via that lies in the main piece of the plane
                        let pr = Prim::new(l, Geo::Circle { c: d.via, r: via_r }, n as i32, clr);
                        !crate::prewire::prim_cells(&g, &pr).iter().any(|&(_, x, y)| label[y as usize * g.nx + x as usize] == main)
                    })
                    .map(|(i, _)| i)
                    .collect()
            };
            if bad.is_empty() {
                continue;
            }
            let gone: Vec<PlaneDrop> = bad.iter().map(|&i| self.drops[i].clone()).collect();
            self.stamp_net(n, -1);
            for d in &gone {
                self.nets[n].pre.dynp.retain(|pr| match pr.geo {
                    Geo::Circle { c, .. } => dist(c, d.via) > 1e-9,
                    Geo::Seg { b, .. } => dist(b, d.via) > 1e-9,
                    _ => true,
                });
            }
            self.drops.retain(|d| !gone.iter().any(|g| g.net == d.net && g.pin == d.pin));
            taken += gone.len();
            if self.cfg.verbose {
                eprintln!("plane vias: {} of {} on {} lie in a cut-off piece of the plane, those pads are routed", gone.len(), gone.len() + self.drops.iter().filter(|d| d.net == n).count(), self.nets[n].name);
            }
            self.stamp_net(n, 1);
            // the pads are separate groups again
            let routed = std::mem::take(&mut self.nets[n].r);
            let prims = routed.dynp;
            self.stamp_dyn(&prims, -1);
            self.init_net_components(n);
            self.route_net(n, false);
        }
        taken
    }

    /// Put the plane vias into the routes of their nets, so they are written out with the result.
    pub(crate) fn merge_drops(&mut self) {
        for d in self.drops.clone() {
            let (via_r, clr) = (self.nets[d.net].via_r, self.nets[d.net].clr);
            let nl = self.grid.nl;
            let have = self.nets[d.net].r.vias.iter().any(|v| dist(*v, d.via) < 1e-6);
            if have {
                continue;
            }
            let (x, y) = self.grid.cell(d.via);
            let ns = &mut self.nets[d.net];
            for l in 0..nl {
                ns.r.dynp.push(Prim::new(l, Geo::Circle { c: d.via, r: via_r }, d.net as i32, clr));
                ns.r.tree.push((l, x, y));
            }
            ns.r.vias.push(d.via);
            if let Some((l, a, b, w)) = d.stub {
                ns.r.wires.push(Wire { layer: l, width: w, pts: vec![a, b], fill: false });
                ns.r.dynp.push(Prim::new(l, Geo::Seg { a, b, hw: w / 2.0 }, d.net as i32, clr));
            }
        }
    }
}
