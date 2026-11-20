//! Checked post-processing: octilinear smoothing, pair tuning and connected ground fill.
use crate::engine::*;
use crate::geom::*;
use std::collections::VecDeque;

pub fn octilinear(a: P, b: P) -> Vec<P> {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let d = dx.abs().min(dy.abs());
    let m = (a.0 + dx.signum() * d, a.1 + dy.signum() * d);
    if dist(a, m) < 1e-9 || dist(m, b) < 1e-9 {
        vec![a, b]
    } else {
        vec![a, m, b]
    }
}

use std::collections::HashMap;

/// The last piece of a wire that ends in a via that is being moved.
struct ViaStub {
    layer: usize,
    from: P,
    width: f64,
}

impl Router {
    fn wire_prims(&self, n: usize, w: &Wire) -> Vec<Prim> {
        w.pts
            .windows(2)
            .map(|s| {
                Prim::new(
                    w.layer,
                    Geo::Seg {
                        a: s[0],
                        b: s[1],
                        hw: w.width / 2.0,
                    },
                    n as i32,
                    self.nets[n].clr,
                )
            })
            .collect()
    }

    pub(crate) fn smooth_wire(&self, n: usize, w: &mut Wire) {
        // Farthest visible shortcut, limited to 0/45/90 degrees and no added length.
        let mut i = 0;
        while i + 2 < w.pts.len() {
            let mut changed = false;
            for j in (i + 2..w.pts.len()).rev() {
                let old: f64 = w.pts[i..=j].windows(2).map(|s| dist(s[0], s[1])).sum();
                for rev in [false, true] {
                    let pts = if rev {
                        let mut p = octilinear(w.pts[j], w.pts[i]);
                        p.reverse();
                        p
                    } else {
                        octilinear(w.pts[i], w.pts[j])
                    };
                    if pts.len() >= j - i + 1
                        || pts.windows(2).map(|s| dist(s[0], s[1])).sum::<f64>() > old + 1e-8
                    {
                        continue;
                    }
                    let candidate = Wire {
                        fill: false,
                        layer: w.layer,
                        width: w.width,
                        pts: pts.clone(),
                    };
                    if self
                        .prims_conflict(&self.wire_prims(n, &candidate))
                        .is_none()
                    {
                        w.pts.splice(i..=j, pts);
                        changed = true;
                        break;
                    }
                }
                if changed {
                    break;
                }
            }
            i += 1;
        }
    }

    pub fn finish(&mut self) {
        if self.cancelled() {
            self.merge_drops();
            return;
        }
        let _t = std::time::Instant::now();
        self.smooth_routes();
        if self.cfg.verbose { eprintln!("finish: smooth_routes {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.center_on_pads();
        if self.cfg.verbose { eprintln!("finish: center_on_pads {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.multi_vias();
        if self.cfg.verbose { eprintln!("finish: multi_vias {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.tune_pairs();
        if self.cfg.verbose { eprintln!("finish: tune_pairs {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.repair_trace_clearance();
        if self.cfg.verbose { eprintln!("finish: repair_trace_clearance {:.2}s", _t.elapsed().as_secs_f32()); }
        // optional: pull the traces tight (shortest path), before widths and pours are made
        if self.cfg.tighten > 0 {
            let _t = std::time::Instant::now();
            self.tighten_traces();
            if self.cfg.verbose { eprintln!("finish: tighten_traces {:.2}s", _t.elapsed().as_secs_f32()); }
        }
        if self.cfg.pour_polygons {
            // copper areas are cut around the final copper, so widths are final before they are made
            let _t = std::time::Instant::now();
        self.final_widths();
        if self.cfg.verbose { eprintln!("finish: final_widths {:.2}s", _t.elapsed().as_secs_f32()); }
            let _t = std::time::Instant::now();
        self.repair_trace_clearance();
        if self.cfg.verbose { eprintln!("finish: repair_trace_clearance {:.2}s", _t.elapsed().as_secs_f32()); }
        }
        let _t = std::time::Instant::now();
        self.fill_copper(true);
        if self.cfg.verbose { eprintln!("finish: fill_copper(true) {:.2}s", _t.elapsed().as_secs_f32()); }
        // plane nets are bridged before pours take the free space on signal layers; nets that a
        // pour will join anyway are left alone
        let poured: Vec<usize> = self
            .fill_jobs()
            .iter()
            .filter(|j| j.2.is_none())
            .map(|j| j.1)
            .collect();
        let plane_nets: Vec<usize> = self
            .planes
            .iter()
            .filter(|p| p.synthetic && !poured.contains(&p.net))
            .map(|p| p.net)
            .collect();
        let _t = std::time::Instant::now();
        self.bridge_islands(Some(&plane_nets));
        if self.cfg.verbose { eprintln!("finish: bridge_islands(planes) {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.fill_copper(false);
        if self.cfg.verbose { eprintln!("finish: fill_copper(false) {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.bridge_islands(None);
        if self.cfg.verbose { eprintln!("finish: bridge_islands(all) {:.2}s", _t.elapsed().as_secs_f32()); }
        if !self.cfg.pour_polygons {
            let _t = std::time::Instant::now();
        self.final_widths();
        if self.cfg.verbose { eprintln!("finish: final_widths {:.2}s", _t.elapsed().as_secs_f32()); }
        }
        let _t = std::time::Instant::now();
        self.repair_trace_clearance();
        if self.cfg.verbose { eprintln!("finish: repair_trace_clearance {:.2}s", _t.elapsed().as_secs_f32()); }
        let _t = std::time::Instant::now();
        self.resolve_via_overlaps();
        if self.cfg.verbose { eprintln!("finish: resolve_via_overlaps {:.2}s", _t.elapsed().as_secs_f32()); }
        if self.cfg.trace_routing && self.cfg.pour_polygons {
            let _t = std::time::Instant::now();
        self.make_pour_polygons();
        if self.cfg.verbose { eprintln!("finish: make_pour_polygons {:.2}s", _t.elapsed().as_secs_f32()); }
            let _t = std::time::Instant::now();
        self.stitch_fill_vias();
        if self.cfg.verbose { eprintln!("finish: stitch_fill_vias {:.2}s", _t.elapsed().as_secs_f32()); }
        }
        if self.cfg.trace_routing && self.cfg.pour_outlines {
            self.draw_pour_outlines();
        }
        // no redundant vertices in the output
        for n in 0..self.nets.len() {
            for w in self.nets[n].r.wires.iter_mut().filter(|w| !w.fill) {
                let p = crate::pair::simplify(&w.pts);
                if p.len() >= 2 {
                    w.pts = p;
                }
            }
        }
        // the vias that tie pads to their plane go into the result
        self.merge_drops();
        if std::env::var_os("AR_DEBUG_ISL").is_some() {
            for n in 0..self.nets.len() {
                if self.nets[n].skip || self.nets[n].pins.len() < 2 {
                    continue;
                }
                let isl = crate::drc::net_islands(self, n);
                if isl.len() > 1 {
                    for (k, i) in isl.iter().enumerate() {
                        for &p in i.pins.iter().take(6) {
                            let c = self.pins[p].c;
                            let drop = self.drops.iter().find(|d| d.pin == p).map(|d| d.via);
                            eprintln!(
                                "ISL {} island {} pin {} at ({:.2},{:.2}) mm drop {:?} ({} pins, {} prims)",
                                self.nets[n].name, k, self.pins[p].name, c.0 * self.unit_mm, c.1 * self.unit_mm,
                                drop.map(|d| (d.0 * self.unit_mm, d.1 * self.unit_mm)), i.pins.len(), i.prims.len()
                            );
                        }
                    }
                }
            }
        }
        self.snapshot("Finished: connectivity and clearance checks");
    }

    /// Restore requested widths wherever they fit, then widen power traces (if enabled).
    fn final_widths(&mut self) {
        if !self.cfg.trace_routing {
            return;
        }
        // Establish connectivity first; widening must not consume another
        // terminal's only escape before that terminal has been connected.
        if self.cfg.allow_neckdown {
            for n in 0..self.nets.len() {
                self.grow_net(n, self.nets[n].width);
            }
        }
        if self.cfg.keep_track_widths {
            self.widen_power();
        }
    }

    fn grow_net(&mut self, n: usize, cap: f64) {
        if self.nets[n].coupled || self.nets[n].guided || self.nets[n].skip || self.cancelled() {
            return;
        }
        let Some(wires) = self.grown_wires(n, cap) else { return };
        self.stamp_net(n, -1);
        self.nets[n].r.wires = wires;
        self.rebuild_route_prims(n);
        self.stamp_net(n, 1);
    }

    /// Every grid-step piece of the net's traces at the largest width (up to `cap`) that keeps
    /// clearance to all foreign copper and the board edge. The room is measured exactly from
    /// the centre line, so no trial widths are needed. Pieces are never narrowed.
    fn grown_wires(&self, n: usize, cap: f64) -> Option<Vec<Wire>> {
        let g = self.grid.g;
        let clr = self.nets[n].clr;
        let coarse = 0.25 / self.unit_mm.max(1e-12);
        let restore = cap <= self.nets[n].width + 1e-9;
        let mut foreign: std::collections::HashMap<usize, Vec<&Prim>> = Default::default();
        let mut out: Vec<Wire> = vec![];
        let mut changed = false;
        for w in &self.nets[n].r.wires {
            if w.fill || w.width >= cap {
                out.push(w.clone());
                continue;
            }
            let others = foreign.entry(w.layer).or_insert_with(|| {
                self.fixed
                    .iter()
                    .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                    .chain(self.nets.iter().flat_map(|ns| ns.r.dynp.iter().chain(ns.pre.dynp.iter())))
                    .filter(|o| o.layer == w.layer && o.net != n as i32)
                    .collect()
            });
            for s in w.pts.windows(2) {
                let steps = (dist(s[0], s[1]) / g).ceil().max(1.0) as usize;
                let point = |i: usize| {
                    let t = i as f64 / steps as f64;
                    (s[0].0 + t * (s[1].0 - s[0].0), s[0].1 + t * (s[1].1 - s[0].1))
                };
                for i in 0..steps {
                    let (a, b) = (point(i), point(i + 1));
                    let line = Prim::new(w.layer, Geo::Seg { a, b, hw: 0.0 }, n as i32, clr);
                    let mut hw = cap / 2.0 + 0.01;
                    for o in others.iter() {
                        let r = hw + clr.max(o.clr);
                        if o.bb.0 > line.bb.2 + r
                            || o.bb.2 < line.bb.0 - r
                            || o.bb.1 > line.bb.3 + r
                            || o.bb.3 < line.bb.1 - r
                        {
                            continue;
                        }
                        hw = hw.min(line.gap(o) - clr.max(o.clr));
                        if 2.0 * hw <= w.width {
                            break;
                        }
                    }
                    let room = 2.0 * hw - 2e-3;
                    // coarser steps above the requested width: fewer, longer pieces to import
                    let width = if room >= cap {
                        cap
                    } else if restore {
                        // back to the requested width or stay as is: no in-between widths
                        w.width
                    } else {
                        ((room / g.max(coarse)).floor() * g.max(coarse)).max(w.width)
                    };
                    changed |= width > w.width;
                    if let Some(last) = out.last_mut().filter(|last| {
                        last.layer == w.layer
                            && !last.fill
                            && last.width == width
                            && dist(*last.pts.last().unwrap(), a) < 1e-8
                    }) {
                        last.pts.push(b);
                    } else {
                        out.push(Wire { fill: false, layer: w.layer, width, pts: vec![a, b] });
                    }
                }
            }
        }
        changed.then_some(out)
    }

    /// Nets that would get a pour (layer assignment or width override).
    fn power_nets(&self) -> Vec<usize> {
        (0..self.nets.len())
            .filter(|&n| {
                let name = &self.nets[n].name;
                !self.nets[n].skip
                    && (self.cfg.layers.values().any(|p| p.nets.contains(name))
                        || self.cfg.net_overrides.get(name).is_some_and(|o| o.width.is_some()))
            })
            .collect()
    }

    /// Pours cannot be exported, so power traces take the free room instead. The limit rises
    /// in steps and every net gets a turn per step, so neighbours share the space between them.
    fn widen_power(&mut self) {
        let max = self.cfg.power_width_max_mm / self.unit_mm.max(1e-12);
        let nets = self.power_nets();
        if max <= 0.0 || nets.is_empty() {
            return;
        }
        let mut level = nets.iter().map(|&n| self.nets[n].width).fold(f64::MAX, f64::min);
        while level < max {
            level = (level * 1.5).min(max);
            for &n in &nets {
                if self.nets[n].width < level {
                    self.grow_net(n, level);
                }
            }
        }
    }

    /// The cells a pour of every layer-assigned net would cover: per (net, layer) a mask of grid
    /// cells that keep clearance (`half` + `stray` probe) from all foreign copper. Nets sharing
    /// a layer split the free room by distance and stay a clearance apart.
    fn pour_masks(&self, half: &dyn Fn(usize) -> f64, stray: f64, planes_too: bool) -> Vec<(usize, usize, Vec<bool>)> {
        let g = self.grid.clone();
        let g1 = Grid { nl: 1, ..g.clone() };
        let (nx, ny) = (g.nx, g.ny);
        let inside: Vec<bool> = (0..nx * ny)
            .map(|i| point_in_poly(g.center((i % nx) as i32, (i / nx) as i32), &self.boundary))
            .collect();
        let mut result = vec![];
        for l in 0..g.nl {
            if (self.excl[l] && !planes_too) || self.cancelled() {
                continue;
            }
            let nets: Vec<usize> = self
                .cfg
                .layers
                .get(&self.layer_names[l])
                .map(|p| {
                    p.nets
                        .iter()
                        .filter_map(|nm| self.nets.iter().position(|n| &n.name == nm && (!n.skip || n.power)))
                        .filter(|&n| {
                            !self.pour_skip.iter().any(|(a, b)| *a == self.nets[n].name && *b == self.layer_names[l])
                        })
                        .take(250)
                        .collect()
                })
                .unwrap_or_default();
            if nets.is_empty() {
                continue;
            }
            let free: Vec<Field> = nets
                .iter()
                .map(|&n| {
                    let mut f = Field::new(&g1, self.nets[n].clr, half(n) + stray, false);
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
                    f
                })
                .collect();
            let mut owner = vec![u8::MAX; nx * ny];
            let mut queue = VecDeque::new();
            for (k, &n) in nets.iter().enumerate() {
                for pr in self
                    .pins
                    .iter()
                    .filter(|p| p.net == n as i32)
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets[n].pre.dynp.iter())
                    .chain(self.nets[n].r.dynp.iter())
                    .filter(|p| p.layer == l && !p.trace_only)
                {
                    for (_, x, y) in crate::prewire::prim_cells(&g, pr) {
                        let i = y as usize * nx + x as usize;
                        if owner[i] == u8::MAX && inside[i] && !free[k].fx(i) {
                            owner[i] = k as u8;
                            queue.push_back(i);
                        }
                    }
                }
            }
            while let Some(i) = queue.pop_front() {
                let k = owner[i];
                let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                        continue;
                    }
                    let j = yy as usize * nx + xx as usize;
                    if owner[j] == u8::MAX && inside[j] && !free[k as usize].fx(j) {
                        owner[j] = k;
                        queue.push_back(j);
                    }
                }
            }
            // keep the regions of different nets a clearance apart
            let clr = nets.iter().map(|&n| self.nets[n].clr).fold(0.0, f64::max);
            let r = (((clr + 2.0 * stray) / g.g).ceil() as i32 + 2) / 2;
            let mut keep = vec![true; nx * ny];
            for y in 0..ny as i32 {
                for x in 0..nx as i32 {
                    let k = owner[y as usize * nx + x as usize];
                    if k == u8::MAX {
                        continue;
                    }
                    let mut foreign = false;
                    for yy in (y - r).max(0)..=(y + r).min(ny as i32 - 1) {
                        for xx in (x - r).max(0)..=(x + r).min(nx as i32 - 1) {
                            let o = owner[yy as usize * nx + xx as usize];
                            foreign |= o != u8::MAX && o != k;
                        }
                    }
                    keep[y as usize * nx + x as usize] = !foreign;
                }
            }
            for (k, &n) in nets.iter().enumerate() {
                let mask: Vec<bool> = (0..nx * ny).map(|i| owner[i] == k as u8 && keep[i]).collect();
                result.push((n, l, mask));
            }
        }
        result
    }

    /// Cells where copper areas may be: inside the board, an edge clearance away from its edge.
    fn board_ok_mask(&self) -> Vec<bool> {
        let g = self.grid.clone();
        let (nx, ny) = (g.nx, g.ny);
        let edge = (((self.cfg.rules.min_edge_clearance_mm + 0.4) / self.unit_mm.max(1e-12)) / g.g).ceil() as i32;
        let inside: Vec<bool> = (0..nx * ny)
            .map(|i| point_in_poly(g.center((i % nx) as i32, (i / nx) as i32), &self.boundary))
            .collect();
        let outside: Vec<bool> = inside.iter().map(|&b| !b).collect();
        let near_edge = crate::outline::dilate(&g, &outside, edge);
        (0..nx * ny).map(|i| inside[i] && !near_edge[i]).collect()
    }

    /// The net whose copper areas fill the empty space (`fill_empty`), if the mode applies.
    pub(crate) fn fill_net_index(&self) -> Option<usize> {
        if !self.cfg.fill_empty || !self.cfg.pour_polygons || !self.cfg.trace_routing {
            return None;
        }
        self.nets
            .iter()
            .position(|n| n.name.eq_ignore_ascii_case(self.cfg.fill_net.trim()) && (!n.skip || n.power) && !n.pins.is_empty())
    }

    /// Does the fill net get a generated area on layer `l`? Not where it is assigned to the
    /// layer anyway. An area of the fill net that is already in the file does not stop it: the
    /// room around that area is filled, and the two are the same net, so they simply merge.
    pub(crate) fn fill_applies(&self, l: usize, f: usize) -> bool {
        !self
            .cfg
            .layers
            .get(&self.layer_names[l])
            .is_some_and(|p| p.nets.iter().any(|n| *n == self.nets[f].name))
    }

    /// Free room of layer `l` for the fill net: inside the board, not taken by another net's
    /// area and not under an existing copper area.
    fn fill_mask(&self, l: usize, ok: &[bool], taken: &[bool]) -> Vec<bool> {
        let pid = &self.plane_id[l];
        (0..ok.len())
            .map(|i| {
                ok[i]
                    && !taken[i]
                    && (pid.is_empty() || pid[i] == 0 || self.planes.get(pid[i] as usize - 1).is_some_and(|p| p.synthetic))
            })
            .collect()
    }

    /// Where copper areas go: per layer, every assigned net owns the cells nearest to its own
    /// pads, vias and traces, up to `pour_reach_mm` away. Foreign copper is ignored on purpose
    /// (the CAD cuts around it when it rebuilds the area); only the board edge and the room of
    /// neighbouring pours are kept free. Notches are closed so the edge stays smooth.
    fn region_masks(&self) -> (Vec<(usize, usize, Vec<bool>)>, Vec<String>) {
        let g = self.grid.clone();
        let (nx, ny) = (g.nx, g.ny);
        let cells = |mm: f64| mm / self.unit_mm.max(1e-12) / g.g;
        let reach = cells(self.cfg.pour_reach_mm);
        let edge = cells(self.cfg.rules.min_edge_clearance_mm + 0.4).ceil() as i32;
        // areas touch: EasyEDA cuts the clearance between them itself
        let gap = 0;
        let close = cells(1.2).ceil().max(2.0) as i32;
        let ok = self.board_ok_mask();
        let _ = edge;
        let fill_net = self.fill_net_index();
        let mut result = vec![];
        let mut warnings: Vec<String> = vec![];
        for l in 0..g.nl {
            if self.cancelled() {
                break;
            }
            // single sided: copper areas only on the one copper layer
            if self.cfg.single_sided && self.excl[l] {
                continue;
            }
            let nets: Vec<usize> = self
                .cfg
                .layers
                .get(&self.layer_names[l])
                .map(|p| {
                    p.nets
                        .iter()
                        .filter_map(|nm| self.nets.iter().position(|n| &n.name == nm && (!n.skip || n.power)))
                        .filter(|&n| !self.pour_skip.iter().any(|(a, b)| *a == self.nets[n].name && *b == self.layer_names[l]))
                        .take(250)
                        .collect()
                })
                .unwrap_or_default();
            let fill = fill_net.filter(|&f| !nets.contains(&f) && self.fill_applies(l, f));
            if nets.is_empty() {
                if let Some(f) = fill {
                    result.push((f, l, self.fill_mask(l, &ok, &vec![false; nx * ny])));
                }
                continue;
            }
            // nearest-copper flood fill (4-neighbour), limited to `reach` from the source cell
            let mut owner = vec![u8::MAX; nx * ny];
            let mut src = vec![0u32; nx * ny];
            let mut queue = VecDeque::new();
            let mut seeds: Vec<Vec<usize>> = vec![vec![]; nets.len()];
            for (k, &n) in nets.iter().enumerate() {
                for pr in self
                    .pins
                    .iter()
                    .filter(|p| p.net == n as i32)
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets[n].pre.dynp.iter())
                    .chain(self.nets[n].r.dynp.iter())
                    .filter(|p| p.layer == l && !p.trace_only && !p.fill)
                {
                    for (_, x, y) in crate::prewire::prim_cells(&g, pr) {
                        let i = y as usize * nx + x as usize;
                        if owner[i] == u8::MAX && ok[i] {
                            owner[i] = k as u8;
                            src[i] = i as u32;
                            queue.push_back(i);
                            seeds[k].push(i);
                        }
                    }
                }
            }
            while let Some(i) = queue.pop_front() {
                let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                let s = src[i] as usize;
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                        continue;
                    }
                    let j = yy as usize * nx + xx as usize;
                    if owner[j] != u8::MAX || !ok[j] {
                        continue;
                    }
                    let (sx, sy) = ((s % nx) as f64, (s / nx) as f64);
                    if ((xx as f64 - sx).powi(2) + (yy as f64 - sy).powi(2)).sqrt() > reach {
                        continue;
                    }
                    owner[j] = owner[i];
                    src[j] = src[i];
                    queue.push_back(j);
                }
            }
            // A ground net with the most pins on the layer is the background pour: it fills
            // whatever the other nets did not claim, as long as it stays connected to itself.
            let pins_on = |n: usize| self.nets[n].pins.len();
            let background = nets
                .iter()
                .enumerate()
                .filter(|(_, &n)| self.nets[n].name.to_uppercase().ends_with("GND"))
                .max_by_key(|(_, &n)| pins_on(n))
                .map(|(k, _)| k);
            let mut masks: Vec<Vec<bool>> = (0..nets.len())
                .map(|k| (0..nx * ny).map(|i| owner[i] == k as u8).collect())
                .collect();
            // On a plane layer the copper area IS the connection between the net's vias and
            // pads: join the pieces with corridors, most important (most pins) nets first.
            let plane_layer = self.cfg.layers.get(&self.layer_names[l]).is_some_and(|p| p.role == LayerRole::Plane);
            // Plane layer: the router's own plane regions (one connected patch per net, built
            // from the nets' pins) are the copper areas; leftover "filler" copper is not exported.
            let from_partition = plane_layer && !self.plane_id[l].is_empty();
            let mut part: Vec<Vec<bool>> = vec![];
            if from_partition {
                let pid = &self.plane_id[l];
                for (k, &n) in nets.iter().enumerate() {
                    if let Some(pi) = self.planes.iter().position(|p| p.net == n && p.layer == l && p.synthetic && !p.filler) {
                        masks[k] = (0..nx * ny).map(|i| pid[i] as usize == pi + 1 && ok[i]).collect();
                    } else {
                        masks[k] = vec![false; nx * ny];
                    }
                }
                part = masks.clone();
                // Only near its own copper (the ground background keeps its whole region); the
                // pieces are joined again through the net's own region, so it stays connected.
                let r = cells(self.cfg.pour_reach_mm).ceil().max(1.0) as i32;
                for k in 0..nets.len() {
                    if Some(k) == background {
                        continue;
                    }
                    let mut seed_mask = vec![false; nx * ny];
                    for &i in &seeds[k] {
                        seed_mask[i] = true;
                    }
                    let near = crate::outline::dilate(&g, &seed_mask, r);
                    for i in 0..nx * ny {
                        masks[k][i] = part[k][i] && near[i];
                    }
                }
            }
            if plane_layer {
                let corridor = cells(1.6).ceil().max(1.0) as i32;
                let mut order: Vec<usize> = (0..nets.len()).collect();
                order.sort_by_key(|&k| std::cmp::Reverse(self.nets[nets[k]].pins.len()));
                for k in order {
                    if Some(k) == background {
                        continue;
                    }
                    for _ in 0..16 {
                        // pieces of this net that hold its own copper
                        let mut label = vec![u32::MAX; nx * ny];
                        let mut pieces: Vec<Vec<usize>> = vec![];
                        for &s0 in &seeds[k] {
                            if label[s0] != u32::MAX || !masks[k][s0] {
                                continue;
                            }
                            let id = pieces.len() as u32;
                            let mut piece = vec![s0];
                            label[s0] = id;
                            let mut qi = 0;
                            while qi < piece.len() {
                                let i = piece[qi];
                                qi += 1;
                                let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                                    let (xx, yy) = (x + dx, y + dy);
                                    if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                                        continue;
                                    }
                                    let j = yy as usize * nx + xx as usize;
                                    if masks[k][j] && label[j] == u32::MAX {
                                        label[j] = id;
                                        piece.push(j);
                                    }
                                }
                            }
                            pieces.push(piece);
                        }
                        if pieces.len() < 2 {
                            break;
                        }
                        let mut foreign = vec![false; nx * ny];
                        for (m, mm) in masks.iter().enumerate() {
                            if m != k {
                                for (f, &c) in foreign.iter_mut().zip(mm) {
                                    *f |= c;
                                }
                            }
                        }
                        let band = crate::outline::dilate(&g, &foreign, gap);
                        // First choice: one plain rectangle around two pieces, if nothing of
                        // another net is in the way (nearest pair first).
                        let bbox = |p: &Vec<usize>| {
                            p.iter().fold((usize::MAX, usize::MAX, 0usize, 0usize), |a, &i| {
                                let (x, y) = (i % nx, i / nx);
                                (a.0.min(x), a.1.min(y), a.2.max(x), a.3.max(y))
                            })
                        };
                        let boxes: Vec<_> = pieces.iter().map(&bbox).collect();
                        let mut pairs: Vec<(usize, usize, i64)> = vec![];
                        for a in 0..pieces.len() {
                            for b in a + 1..pieces.len() {
                                let dx = (boxes[a].0 as i64 - boxes[b].2 as i64).max(boxes[b].0 as i64 - boxes[a].2 as i64).max(0);
                                let dy = (boxes[a].1 as i64 - boxes[b].3 as i64).max(boxes[b].1 as i64 - boxes[a].3 as i64).max(0);
                                pairs.push((a, b, dx * dx + dy * dy));
                            }
                        }
                        pairs.sort_by_key(|p| p.2);
                        let mut merged = false;
                        for &(a, b, _) in &pairs {
                            let (x0, y0) = (boxes[a].0.min(boxes[b].0), boxes[a].1.min(boxes[b].1));
                            let (x1, y1) = (boxes[a].2.max(boxes[b].2), boxes[a].3.max(boxes[b].3));
                            let mut blocked = false;
                            'scan: for y in y0..=y1 {
                                for x in x0..=x1 {
                                    let i = y * nx + x;
                                    if ok[i] && band[i] && !masks[k][i] {
                                        blocked = true;
                                        break 'scan;
                                    }
                                }
                            }
                            if !blocked {
                                for y in y0..=y1 {
                                    for x in x0..=x1 {
                                        let i = y * nx + x;
                                        if ok[i] {
                                            masks[k][i] = true;
                                        }
                                    }
                                }
                                merged = true;
                                break;
                            }
                        }
                        if merged {
                            continue;
                        }
                        // breadth-first from the first piece to the nearest other piece
                        let mut prev = vec![u32::MAX; nx * ny];
                        let mut seen = vec![false; nx * ny];
                        let mut q: VecDeque<usize> = VecDeque::new();
                        for &i in &pieces[0] {
                            seen[i] = true;
                            q.push_back(i);
                        }
                        let mut hit = None;
                        'bfs: while let Some(i) = q.pop_front() {
                            let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                                let (xx, yy) = (x + dx, y + dy);
                                if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                                    continue;
                                }
                                let j = yy as usize * nx + xx as usize;
                                if seen[j] {
                                    continue;
                                }
                                let own_piece = label[j] != u32::MAX;
                                let free = if from_partition { part[k][j] } else { ok[j] && !band[j] };
                                if !own_piece && !free {
                                    continue;
                                }
                                seen[j] = true;
                                prev[j] = i as u32;
                                if own_piece && label[j] != 0 {
                                    hit = Some(j);
                                    break 'bfs;
                                }
                                q.push_back(j);
                            }
                        }
                        let Some(mut c) = hit else {
                            warnings.push(format!(
                                "{} on layer {}: {} separate pieces cannot be joined (no free corridor)",
                                self.nets[nets[k]].name,
                                self.layer_names[l],
                                pieces.len()
                            ));
                            break;
                        };
                        let mut path = vec![false; nx * ny];
                        while label[c] != 0 {
                            path[c] = true;
                            let p = prev[c];
                            if p == u32::MAX {
                                break;
                            }
                            c = p as usize;
                        }
                        let wide = crate::outline::dilate(&g, &path, corridor);
                        for i in 0..nx * ny {
                            let free = if from_partition { part[k][i] } else { ok[i] && !band[i] };
                            if wide[i] && free {
                                masks[k][i] = true;
                            }
                        }
                    }
                }
            }
            // ground fills whatever is left once every other net has its (joined) area
            if !from_partition {
            for k in 0..nets.len() {
                for i in 0..nx * ny {
                    if masks[k][i] && owner[i] == u8::MAX {
                        owner[i] = k as u8;
                    }
                }
            }
            if let Some(k) = background {
                let mut q: VecDeque<usize> = (0..nx * ny).filter(|&i| owner[i] == k as u8).collect();
                while let Some(i) = q.pop_front() {
                    let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                    for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                        let (xx, yy) = (x + dx, y + dy);
                        if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                            continue;
                        }
                        let j = yy as usize * nx + xx as usize;
                        if owner[j] == u8::MAX && ok[j] {
                            owner[j] = k as u8;
                            q.push_back(j);
                        }
                    }
                }
            }
            for k in 0..nets.len() {
                for i in 0..nx * ny {
                    masks[k][i] = owner[i] == k as u8;
                }
            }
            }
            // ground takes back everything the other nets gave up (and every unclaimed cell)
            if from_partition {
                if let Some(bg) = background {
                    let mut taken = vec![false; nx * ny];
                    for (m, mm) in masks.iter().enumerate() {
                        if m != bg {
                            for (t, &c) in taken.iter_mut().zip(mm) {
                                *t |= c;
                            }
                        }
                    }
                    for i in 0..nx * ny {
                        masks[bg][i] = ok[i] && !taken[i];
                    }
                }
            }
            let mut layer_masks: Vec<Vec<bool>> = vec![];
            for k in 0..nets.len() {
                // smooth: close notches, but never into the room of another net
                let mut foreign = vec![false; nx * ny];
                for (m, mm) in masks.iter().enumerate() {
                    if m != k {
                        for (f, &c) in foreign.iter_mut().zip(mm) {
                            *f |= c;
                        }
                    }
                }
                let band = crate::outline::dilate(&g, &foreign, gap);
                let grown = crate::outline::dilate(&g, &masks[k], close);
                let not_grown: Vec<bool> = grown.iter().map(|&b| !b).collect();
                let shrunk_away = crate::outline::dilate(&g, &not_grown, close);
                let closed: Vec<bool> = (0..nx * ny)
                    .map(|i| (masks[k][i] || (grown[i] && !shrunk_away[i])) && ok[i] && !band[i])
                    .collect();
                layer_masks.push(closed);
            }
            // Inflate every area until it touches another one (or the board edge, or 8 mm).
            let grow = cells(8.0).ceil() as i32;
            let mut owner2 = vec![u8::MAX; nx * ny];
            for (k, m) in layer_masks.iter().enumerate() {
                for i in 0..nx * ny {
                    if m[i] {
                        owner2[i] = k as u8;
                    }
                }
            }
            let mut dist = vec![0i32; nx * ny];
            let mut q: VecDeque<usize> = (0..nx * ny).filter(|&i| owner2[i] != u8::MAX).collect();
            while let Some(i) = q.pop_front() {
                let d = dist[i];
                if d >= grow {
                    continue;
                }
                let (x, y) = ((i % nx) as i32, (i / nx) as i32);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                        continue;
                    }
                    let j = yy as usize * nx + xx as usize;
                    if owner2[j] == u8::MAX && ok[j] {
                        owner2[j] = owner2[i];
                        dist[j] = d + 1;
                        q.push_back(j);
                    }
                }
            }
            let mut taken = vec![false; nx * ny];
            for (k, m) in layer_masks.into_iter().enumerate() {
                let grown: Vec<bool> = (0..nx * ny).map(|i| owner2[i] == k as u8 || (m[i] && owner2[i] == u8::MAX)).collect();
                for (t, &c) in taken.iter_mut().zip(&grown) {
                    *t |= c;
                }
                result.push((nets[k], l, grown));
            }
            if let Some(f) = fill {
                result.push((f, l, self.fill_mask(l, &ok, &taken)));
            }
        }
        (result, warnings)
    }

    /// Copper areas for CADs that import them (EasyEDA JSON): per net and layer, every connected
    /// piece of the pour region as one polygon with the holes around foreign copper cut in.
    /// The CAD re-pours inside the polygon with its own clearances.
    fn make_pour_polygons(&mut self) {
        let g = self.grid.clone();
        let (nx, ny) = (g.nx, g.ny);
        let (masks, warns) = self.region_masks();
        self.pour_warnings.extend(warns);
        // Outlines are traced on a coarse grid of ~1 mm blocks: squares and 45 degree lines only
        // (finer on a layer where vias of different nets are closer together than a block).
        let b0 = ((1.0 / self.unit_mm.max(1e-12) / g.g).round() as usize).max(3);
        let min_plane = self.min_pour_area();
        let min_trace = (6.0 / self.unit_mm.max(1e-12)).powi(2);
        let board_ok = self.board_ok_mask();
        let fill_net = self.fill_net_index();
        let mut layers: Vec<usize> = masks.iter().map(|m| m.1).collect();
        layers.sort_unstable();
        layers.dedup();
        for l in layers {
            let group: Vec<&(usize, usize, Vec<bool>)> = masks.iter().filter(|m| m.1 == l).collect();
            let kn = group.len();
            let min_area = if self.cfg.layers.get(&self.layer_names[l]).is_some_and(|p| p.role == LayerRole::Plane) {
                min_plane
            } else {
                min_trace
            };
            // Vias, pads and traces of a net that sit inside its own plane region are connected to
            // the plane through that copper, so the coarse outline has to cover every one of them.
            // `foot`: the grid cells of each net's such copper; `check`: points that must end up
            // inside the net's polygons (circle outline and centre, corners of other shapes).
            let mut foot: Vec<Vec<(usize, usize)>> = vec![vec![]; kn];
            let mut check: Vec<Vec<P>> = vec![vec![]; kn];
            for (k, m) in group.iter().enumerate() {
                let n = m.0;
                for pr in self
                    .pins
                    .iter()
                    .filter(|p| p.net == n as i32)
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets[n].pre.dynp.iter())
                    .chain(self.nets[n].r.dynp.iter())
                    .filter(|p| p.layer == l && !p.trace_only && !p.fill)
                {
                    let cells = crate::prewire::prim_cells(&g, pr);
                    if !cells.iter().any(|&(_, x, y)| m.2[y as usize * nx + x as usize]) {
                        continue;
                    }
                    if let Geo::Circle { c, r } = &pr.geo {
                        check[k].push(*c);
                        for a in 0..8 {
                            let t = a as f64 * std::f64::consts::FRAC_PI_4;
                            check[k].push((c.0 + r * t.cos(), c.1 + r * t.sin()));
                        }
                    }
                    for &(_, x, y) in &cells {
                        if !matches!(pr.geo, Geo::Circle { .. }) {
                            let (cx, cy) = (g.x0 + x as f64 * g.g, g.y0 + y as f64 * g.g);
                            for (sx, sy) in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)] {
                                check[k].push((cx + sx * g.g, cy + sy * g.g));
                            }
                        }
                        foot[k].push((x as usize, y as usize));
                    }
                }
                foot[k].sort_unstable();
                foot[k].dedup();
            }
            let b = b0;
            let (cnx, cny) = (nx.div_ceil(b), ny.div_ceil(b));
            let block_area = (b as f64 * g.g).powi(2);
            // blocks that touch a copper area already in the file: nothing grows into them, so
            // generated areas leave a hole there (and a margin of up to one block)
            let mut exist_blk = vec![false; cnx * cny];
            if !self.plane_id[l].is_empty() {
                let mut cnt = vec![0usize; cnx * cny];
                for i in 0..nx * ny {
                    let v = self.plane_id[l][i] as usize;
                    if v != 0 && self.planes.get(v - 1).is_some_and(|p| !p.synthetic) {
                        cnt[(i / nx / b) * cnx + (i % nx) / b] += 1;
                    }
                }
                for k in 0..cnx * cny {
                    exist_blk[k] = cnt[k] > 0;
                }
            }
            let mut okb = vec![false; cnx * cny];
            for i in 0..nx * ny {
                if board_ok[i] {
                    okb[(i / nx / b) * cnx + (i % nx) / b] = true;
                }
            }
            // each block belongs to the net that covers most of it
            let mut count = vec![0u16; cnx * cny * kn];
            for (k, m) in group.iter().enumerate() {
                for i in 0..nx * ny {
                    if m.2[i] {
                        count[((i / nx / b) * cnx + (i % nx) / b) * kn + k] += 1;
                    }
                }
            }
            let mut label = vec![u8::MAX; cnx * cny];
            for bi in 0..cnx * cny {
                let mut best = (0u16, u8::MAX);
                for k in 0..kn {
                    let c = count[bi * kn + k];
                    if c > best.0 {
                        best = (c, k as u8);
                    }
                }
                if best.0 as f64 > 0.4 * (b * b) as f64 {
                    label[bi] = best.1;
                }
            }
            // majority filter: nibs and notches of one block disappear, areas keep touching
            for _ in 0..2 {
                let prev = label.clone();
                for by in 0..cny {
                    for bx in 0..cnx {
                        let mut tally: Vec<(u8, u32)> = vec![];
                        for yy in by.saturating_sub(1)..=(by + 1).min(cny - 1) {
                            for xx in bx.saturating_sub(1)..=(bx + 1).min(cnx - 1) {
                                let v = prev[yy * cnx + xx];
                                match tally.iter_mut().find(|t| t.0 == v) {
                                    Some(t) => t.1 += 1,
                                    None => tally.push((v, 1)),
                                }
                            }
                        }
                        let cur = prev[by * cnx + bx];
                        let top = tally.iter().map(|t| t.1).max().unwrap_or(0);
                        if tally.iter().find(|t| t.0 == cur).map_or(0, |t| t.1) < top {
                            let pick = tally.iter().filter(|t| t.1 == top).map(|t| t.0).min().unwrap_or(cur);
                            label[by * cnx + bx] = pick;
                        }
                    }
                }
            }
            // Hand the blocks under that copper to the net (the footprint itself first, then a one
            // cell margin wherever no other net needs the room), whatever the majority said.
            let mut forced = vec![vec![false; cnx * cny]; kn];
            let mut forced_by = vec![u8::MAX; cnx * cny];
            let mut margin_blocks: Vec<Vec<usize>> = vec![vec![]; kn];
            for k in 0..kn {
                for &(x, y) in &foot[k] {
                    let bi = (y / b) * cnx + x / b;
                    if okb[bi] && forced_by[bi] == u8::MAX {
                        forced_by[bi] = k as u8;
                        forced[k][bi] = true;
                    }
                    for dy in -1i32..=1 {
                        for dx in -1i32..=1 {
                            let (xx, yy) = (x as i32 + dx, y as i32 + dy);
                            if xx < 0 || yy < 0 || xx >= nx as i32 || yy >= ny as i32 {
                                continue;
                            }
                            let bj = (yy as usize / b) * cnx + xx as usize / b;
                            if okb[bj] {
                                margin_blocks[k].push(bj);
                            }
                        }
                    }
                }
            }
            for k in 0..kn {
                for &bj in &margin_blocks[k] {
                    if forced_by[bj] == u8::MAX {
                        forced_by[bj] = k as u8;
                        forced[k][bj] = true;
                    }
                }
            }
            for bi in 0..cnx * cny {
                if forced_by[bi] != u8::MAX {
                    label[bi] = forced_by[bi];
                } else if exist_blk[bi] {
                    // the smoothing above would close a small hole around an existing area
                    label[bi] = u8::MAX;
                }
            }
            for (k, m) in group.iter().enumerate() {
                let n = m.0;
                // blocks holding copper of the net itself
                let mut own = vec![false; cnx * cny];
                for pr in self
                    .pins
                    .iter()
                    .filter(|p| p.net == n as i32)
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets[n].pre.dynp.iter())
                    .chain(self.nets[n].r.dynp.iter())
                    .filter(|p| p.layer == l && !p.trace_only && !p.fill)
                {
                    for (_, x, y) in crate::prewire::prim_cells(&g, pr) {
                        own[(y as usize / b) * cnx + x as usize / b] = true;
                    }
                }
                // keep pieces that touch own copper and are big enough
                let mut mask: Vec<bool> = label.iter().map(|&v| v == k as u8).collect();
                let mut seen = vec![false; cnx * cny];
                for start in 0..cnx * cny {
                    if !mask[start] || seen[start] {
                        continue;
                    }
                    let mut piece = vec![start];
                    seen[start] = true;
                    let mut keep = own[start];
                    let mut must = forced[k][start];
                    let mut qi = 0;
                    while qi < piece.len() {
                        let i = piece[qi];
                        qi += 1;
                        let (x, y) = ((i % cnx) as i32, (i / cnx) as i32);
                        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                            let (xx, yy) = (x + dx, y + dy);
                            if xx < 0 || yy < 0 || xx >= cnx as i32 || yy >= cny as i32 {
                                continue;
                            }
                            let j = yy as usize * cnx + xx as usize;
                            if mask[j] && !seen[j] {
                                seen[j] = true;
                                keep |= own[j];
                                must |= forced[k][j];
                                piece.push(j);
                            }
                        }
                    }
                    // a piece that carries vias or pads of its plane is never dropped; fill areas
                    // need no copper of their own yet (stitching vias or pruning decide later)
                    let fill_piece = fill_net == Some(n) && self.fill_applies(l, n);
                    if std::env::var_os("AR_DEBUG_POUR").is_some() {
                        let (sx, sy) = (piece[0] % cnx, piece[0] / cnx);
                        eprintln!(
                            "pour piece {} layer {}: {} blocks ({:.1} mm2) at block ({},{}) keep {} must {} fill_piece {} min {:.1}",
                            self.nets[n].name, self.layer_names[l], piece.len(),
                            piece.len() as f64 * block_area * self.unit_mm * self.unit_mm, sx, sy, keep, must, fill_piece,
                            min_area * self.unit_mm * self.unit_mm
                        );
                    }
                    if !must && ((!keep && !fill_piece) || piece.len() as f64 * block_area < min_area) {
                        for i in piece {
                            mask[i] = false;
                            label[i] = u8::MAX;
                        }
                    }
                }
            }
            // blocks left over by dropped fragments go to the nearest surviving area (so no
            // holes remain), as far as the board allows
            let mut q: VecDeque<usize> = (0..cnx * cny).filter(|&i| label[i] != u8::MAX).collect();
            while let Some(i) = q.pop_front() {
                let (x, y) = ((i % cnx) as i32, (i / cnx) as i32);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= cnx as i32 || yy >= cny as i32 {
                        continue;
                    }
                    let j = yy as usize * cnx + xx as usize;
                    if label[j] == u8::MAX && okb[j] && !exist_blk[j] {
                        label[j] = label[i];
                        q.push_back(j);
                    }
                }
            }
            // Cell level: the block map is smooth but too coarse for vias of different nets that sit
            // closer together than a block. Every net takes the cells under its own vias and pads
            // (and one cell more all round, where no other net has copper), then the boundaries
            // between the areas are traced once and shared, so neighbouring areas fit exactly.
            let mut lab = vec![u8::MAX; nx * ny];
            for i in 0..nx * ny {
                if board_ok[i] {
                    lab[i] = label[(i / nx / b) * cnx + (i % nx) / b];
                }
            }
            let mut is_foot = vec![false; nx * ny];
            for k in 0..kn {
                for &(x, y) in &foot[k] {
                    let i = y * nx + x;
                    if board_ok[i] {
                        lab[i] = k as u8;
                        is_foot[i] = true;
                    }
                }
            }
            for k in 0..kn {
                for &(x, y) in &foot[k] {
                    for yy in y.saturating_sub(1)..=(y + 1).min(ny - 1) {
                        for xx in x.saturating_sub(1)..=(x + 1).min(nx - 1) {
                            let j = yy * nx + xx;
                            if board_ok[j] && !is_foot[j] {
                                lab[j] = k as u8;
                            }
                        }
                    }
                }
            }
            // Two areas (or an area and a gap) that meet only at a corner leave a ring that touches
            // itself there: an invalid polygon that CADs fill wrongly. A 2x2 window with the same
            // label on one diagonal and another on the other gets one cell changed, never one
            // under a via or pad.
            crate::outline::fix_pinches(&mut lab, nx, ny, &is_foot, &board_ok);
            let chains = crate::outline::trace_chains(&lab, nx, ny);
            let to_pt = |&(x, y): &(i32, i32)| -> P { (g.x0 + (x as f64 - 0.5) * g.g, g.y0 + (y as f64 - 0.5) * g.g) };
            // The simplification tolerance is coarse (a staircase of one block per step becomes one
            // 45 degree line) and halves, only around the spot, every time a via or pad there is
            // not covered: `zones` holds (centre, level) of such spots per boundary.
            let base_tol = 1.2 * b as f64;
            let zone_r = 3 * b as i32;
            let mut zones: Vec<Vec<((i32, i32), u32)>> = vec![vec![]; chains.len()];
            let level_at = |zones: &Vec<((i32, i32), u32)>, q: (i32, i32)| -> u32 {
                zones
                    .iter()
                    .filter(|(c, _)| (c.0 - q.0).abs() <= zone_r && (c.1 - q.1).abs() <= zone_r)
                    .map(|z| z.1)
                    .max()
                    .unwrap_or(0)
            };
            let covered = |rings: &Vec<Vec<Vec<(i32, i32)>>>, k: usize, p: P| -> bool {
                rings[k]
                    .iter()
                    .filter(|r| point_in_poly(p, &r.iter().map(to_pt).collect::<Vec<P>>()))
                    .count()
                    % 2
                    == 1
            };
            let mut rings: Vec<Vec<Vec<(i32, i32)>>> = vec![];
            for _round in 0..60 {
                let mut simp: Vec<Vec<(i32, i32)>> = vec![vec![]; chains.len()];
                for (i, c) in chains.iter().enumerate() {
                    if c.master == i {
                        // a boundary to nothing needs less care than one shared with another area
                        let t = if c.nbr == u8::MAX { base_tol * 0.75 } else { base_tol };
                        let zs = &zones[i];
                        simp[i] = crate::outline::simplify_open_var(&c.pts, &|q| {
                            let lv = level_at(zs, q);
                            if lv >= 5 { 0.0 } else { t / (1u32 << lv) as f64 }
                        });
                    }
                }
                for (i, c) in chains.iter().enumerate() {
                    if c.master != i {
                        let mut v = simp[c.master].clone();
                        v.reverse();
                        simp[i] = v;
                    }
                }
                rings = crate::outline::assemble_rings(&chains, &simp, kn);
                // points of vias / pads that the areas fail to cover make the boundary near them finer
                let mut worse = false;
                for k in 0..kn {
                    let polys: Vec<Vec<P>> = rings[k].iter().map(|r| r.iter().map(to_pt).collect()).collect();
                    for &p in &check[k] {
                        let inside = polys.iter().filter(|poly| point_in_poly(p, poly)).count() % 2 == 1;
                        if inside {
                            continue;
                        }
                        // only the boundary point closest to the spot gets finer
                        let mut nearest: Option<(usize, (i32, i32), f64)> = None;
                        for (i, c) in chains.iter().enumerate() {
                            for q in &c.pts {
                                if level_at(&zones[c.master], *q) >= 5 {
                                    continue;
                                }
                                let d = dist(to_pt(q), p);
                                if nearest.as_ref().is_none_or(|n| d < n.2) {
                                    nearest = Some((i, *q, d));
                                }
                            }
                        }
                        if let Some((i, q, _)) = nearest {
                            let z = &mut zones[chains[i].master];
                            match z.iter_mut().find(|(c, _)| (c.0 - q.0).abs() <= zone_r && (c.1 - q.1).abs() <= zone_r) {
                                Some(zz) => zz.1 += 1,
                                None => z.push((q, 1)),
                            }
                            worse = true;
                        }
                    }
                }
                if !worse {
                    break;
                }
            }
            for k in 0..kn {
                let n = group[k].0;
                let uncovered = check[k].iter().filter(|&&p| !covered(&rings, k, p)).count();
                if uncovered > 0 {
                    self.pour_warnings.push(format!(
                        "{} on layer {}: {} point(s) of its vias/pads are not covered by the copper area",
                        self.nets[n].name, self.layer_names[l], uncovered
                    ));
                }
                let mut outers: Vec<Vec<P>> = vec![];
                let mut holes: Vec<Vec<P>> = vec![];
                for r in &rings[k] {
                    let area: i64 = (0..r.len())
                        .map(|i| {
                            let (a, b2) = (r[i], r[(i + 1) % r.len()]);
                            a.0 as i64 * b2.1 as i64 - b2.0 as i64 * a.1 as i64
                        })
                        .sum();
                    let pts: Vec<P> = r.iter().map(to_pt).collect();
                    if area > 0 {
                        outers.push(pts);
                    } else if area < 0 {
                        holes.push(pts);
                    }
                }
                for o in outers {
                    let inside: Vec<Vec<P>> = holes.iter().filter(|h| point_in_poly(h[0], &o)).cloned().collect();
                    let (poly, failed) = crate::outline::merge_holes_checked(o.clone(), inside);
                    self.fill_polys.push((n, l, if failed > 0 { o } else { poly }));
                    self.outline_stats.0 += 1;
                }
            }
        }
    }

    /// Smallest copper area worth a polygon (about 3 mm x 3 mm): fewer, larger polygons.
    fn min_pour_area(&self) -> f64 {
        (3.0 / self.unit_mm.max(1e-12)).powi(2)
    }

    /// The area a pour of every layer-assigned net would cover on its layer, drawn as thin
    /// closed traces of that net, for CADs that cannot import copper areas. Holes are left out:
    /// the CAD cuts those itself when the outline is re-traced as a copper area.
    fn draw_pour_outlines(&mut self) {
        let g = self.grid.clone();
        let min_area = (2.0 / self.unit_mm.max(1e-12)).powi(2).max((g.g * 8.0).powi(2));
        // Few corners matter more than hugging every obstacle: the outline is re-traced by
        // hand. It may stray `stray` from the cells it encloses, so keep that free.
        let tol = (0.4 / self.unit_mm.max(1e-12)).max(g.g * 0.8);
        let stray = tol + g.g * 0.8;
        let width_of = |me: &Router, n: usize| me.fields[me.nets[n].narrow_tf].p * 2.0;
        let masks = self.pour_masks(&|n| width_of(self, n) / 2.0, stray, false);
        let (nx, ny) = (g.nx, g.ny);
        for (n, l, mask) in masks {
            let width = width_of(self, n);
            let (outers, _holes) = crate::outline::contours(&mask, nx, ny);
            for lp in outers {
                let raw: Vec<P> = lp
                    .iter()
                    .map(|&(x, y)| (g.x0 + (x as f64 - 0.5) * g.g, g.y0 + (y as f64 - 0.5) * g.g))
                    .collect();
                if crate::outline::ring_area(&raw).abs() < min_area {
                    continue;
                }
                let mut done = false;
                for tol in [tol, tol * 0.5, g.g * 0.8, 0.0] {
                    let mut pts = if tol > 0.0 {
                        crate::outline::simplify_closed(&raw, tol)
                    } else {
                        crate::pair::simplify(&raw)
                    };
                    if pts.len() < 3 {
                        continue;
                    }
                    pts.push(pts[0]);
                    let w = Wire { fill: false, layer: l, width, pts };
                    let prims = self.wire_prims(n, &w);
                    if self.prims_conflict(&prims).is_none() {
                        self.ground_fill_length += w.pts.windows(2).map(|s| dist(s[0], s[1])).sum::<f64>();
                        self.stamp_dyn(&prims, 1);
                        self.nets[n].r.dynp.extend(prims);
                        self.nets[n].r.wires.push(w);
                        done = true;
                        break;
                    }
                }
                if done {
                    self.outline_stats.0 += 1;
                } else {
                    self.outline_stats.1 += 1;
                }
            }
        }
    }

    /// Grid searches test sample points; verify the continuous copper after
    /// finishing as well. Neck down only offending segments, never below the rules.
    fn repair_trace_clearance(&mut self) {
        if !self.cfg.allow_neckdown { return; }
        for n in 0..self.nets.len() {
            if self.cancelled() { return; }
            if self.nets[n].coupled || self.nets[n].guided { continue; }
            let old = self.nets[n].r.wires.clone();
            let mut result = Vec::with_capacity(old.len());
            let mut changed = false;
            for w in old {
                if w.fill || self.prims_conflict(&self.wire_prims(n, &w)).is_none() {
                    result.push(w); continue;
                }
                for pts in w.pts.windows(2) {
                    let mut piece = Wire { pts: pts.to_vec(), ..w.clone() };
                    if self.prims_conflict(&self.wire_prims(n, &piece)).is_some() {
                        let floor = self.min_width.max(self.grid.g * 0.05).min(w.width);
                        piece.width = floor;
                        if self.prims_conflict(&self.wire_prims(n, &piece)).is_none() {
                            let (mut lo, mut hi) = (floor, w.width);
                            for _ in 0..20 {
                                piece.width = (lo + hi) * 0.5;
                                if self.prims_conflict(&self.wire_prims(n, &piece)).is_none() { lo = piece.width; }
                                else { hi = piece.width; }
                            }
                            piece.width = lo;
                            changed = true;
                        } else { piece.width = w.width; }
                    }
                    result.push(piece);
                }
            }
            if changed {
                self.stamp_net(n, -1);
                self.nets[n].r.wires = result;
                self.rebuild_route_prims(n);
                self.stamp_net(n, 1);
            }
        }
    }

    fn smooth_routes(&mut self) {
        for n in 0..self.nets.len() {
            if self.nets[n].coupled || self.nets[n].guided || self.cancelled() {
                continue;
            }
            let old = self.nets[n].r.wires.clone();
            let anchors: Vec<(usize, P)> = old
                .iter()
                .flat_map(|w| [(w.layer, w.pts[0]), (w.layer, *w.pts.last().unwrap())])
                .collect();
            let mut result = Vec::new();
            for w in &old {
                let mut run = vec![w.pts[0]];
                for seg in w.pts.windows(2) {
                    let mut points: Vec<P> = anchors
                        .iter()
                        .filter(|(l, p)| {
                            *l == w.layer
                                && dist(*p, seg[0]) > 1e-8
                                && dist_pt_seg(*p, seg[0], seg[1]) < 1e-8
                        })
                        .map(|(_, p)| *p)
                        .collect();
                    points.push(seg[1]);
                    points.sort_by(|a, b| dist(seg[0], *a).total_cmp(&dist(seg[0], *b)));
                    points.dedup_by(|a, b| dist(*a, *b) < 1e-8);
                    for p in points {
                        run.push(p);
                        if anchors
                            .iter()
                            .any(|(l, a)| *l == w.layer && dist(*a, p) < 1e-8)
                        {
                            let mut chunk = Wire {
                                fill: false,
                                layer: w.layer,
                                width: w.width,
                                pts: run,
                            };
                            self.smooth_wire(n, &mut chunk);
                            result.push(chunk);
                            run = vec![p];
                        }
                    }
                }
                if run.len() > 1 {
                    let mut chunk = Wire {
                        fill: false,
                        layer: w.layer,
                        width: w.width,
                        pts: run,
                    };
                    self.smooth_wire(n, &mut chunk);
                    result.push(chunk);
                }
            }
            self.stamp_net(n, -1);
            self.nets[n].r.wires = result;
            self.rebuild_route_prims(n);
            self.stamp_net(n, 1);
        }
    }

    fn tune_pairs(&mut self) {
        // every group (pair or bundle) is matched to its longest member
        let mut groups: Vec<Vec<usize>> = vec![];
        for p in 0..self.nets.len() {
            if let Some(q) = self.nets[p].pair {
                if p < q {
                    groups.push(vec![p, q]);
                }
            }
        }
        groups.extend(self.bundles.iter().cloned());
        for members in groups {
            if !members
                .iter()
                .all(|&m| self.nets[m].coupled || self.nets[m].guided)
            {
                continue;
            }
            let tol = self.cfg.tolerance_for(&self.nets[members[0]].name);
            let longest = members
                .iter()
                .map(|&m| self.wire_length(m))
                .fold(f64::MIN, f64::max);
            for &n in &members {
                // several small 45-degree bumps when no single segment is long enough
                let mut step = longest - self.wire_length(n);
                let min_step = (tol * 0.5).max(self.grid.g * 0.1);
                for _ in 0..40 {
                    let delta = longest - self.wire_length(n);
                    if delta <= tol || step < min_step {
                        break;
                    }
                    if self.add_length(n, step.min(delta)) {
                        continue;
                    }
                    step *= 0.5;
                }
            }
        }
    }

    /// A four-corner 45-degree trombone adds exactly 2*h*(sqrt(2)-1).
    /// One 45-degree bump (squiggle) of height h adds exactly 2*h*(sqrt(2)-1). Tried on every
    /// segment, on both sides, at several positions and with a long or a short flat top.
    fn add_length(&mut self, n: usize, delta: f64) -> bool {
        let h = delta / (2.0 * (2.0_f64.sqrt() - 1.0));
        let mut replacement = None;
        'wires: for (wi, w) in self.nets[n].r.wires.iter().enumerate() {
            if w.fill {
                continue;
            }
            for (si, s) in w.pts.windows(2).enumerate() {
                let len = dist(s[0], s[1]);
                let u = ((s[1].0 - s[0].0) / len.max(1e-12), (s[1].1 - s[0].1) / len.max(1e-12));
                for top in [2.0 * h, w.width.max(self.grid.g)] {
                    let foot = 2.0 * h + top;
                    if len < foot + 2.0 * w.width {
                        continue;
                    }
                    for frac in [0.5, 0.25, 0.75] {
                        let begin = (len - foot) * frac;
                        for sign in [-1.0, 1.0] {
                            let v = (-u.1 * sign, u.0 * sign);
                            let pt = |d: f64, k: f64| {
                                (s[0].0 + u.0 * d + v.0 * k, s[0].1 + u.1 * d + v.1 * k)
                            };
                            let mut nw = w.clone();
                            nw.pts.splice(
                                si + 1..si + 1,
                                [
                                    pt(begin, 0.0),
                                    pt(begin + h, h),
                                    pt(begin + h + top, h),
                                    pt(begin + foot, 0.0),
                                ],
                            );
                            if self.prims_conflict(&self.wire_prims(n, &nw)).is_none() {
                                replacement = Some((wi, nw));
                                break 'wires;
                            }
                        }
                    }
                }
            }
        }
        if let Some((wi, w)) = replacement {
            self.stamp_net(n, -1);
            self.nets[n].r.wires[wi] = w;
            self.rebuild_route_prims(n);
            self.stamp_net(n, 1);
            return true;
        }
        false
    }

    /// Nets that ask for several vias per layer change get extra vias next to each via, tied
    /// to it with a short stub on every layer the via joins.
    fn multi_vias(&mut self) {
        for n in 0..self.nets.len() {
            let want = self
                .cfg
                .net_overrides
                .get(&self.nets[n].name)
                .and_then(|o| o.via_count)
                .unwrap_or(1)
                .min(8);
            if want <= 1 || self.nets[n].skip || self.cfg.single_sided {
                continue;
            }
            let (vr, clr, width) = (self.nets[n].via_r, self.nets[n].clr, self.nets[n].width);
            let d = 2.0 * vr * 1.1;
            let originals = self.nets[n].r.vias.clone();
            for v in originals {
                let mut layers: Vec<usize> = self.nets[n]
                    .r
                    .wires
                    .iter()
                    .filter(|w| !w.fill && w.pts.iter().any(|p| dist(*p, v) < 1e-6))
                    .map(|w| w.layer)
                    .collect();
                layers.sort();
                layers.dedup();
                let mut placed = 1;
                'ring: for ring in 1..=2 {
                    for k in 0..8 {
                        if placed >= want {
                            break 'ring;
                        }
                        let ang = k as f64 * std::f64::consts::FRAC_PI_4;
                        let c = (
                            v.0 + ring as f64 * d * ang.cos(),
                            v.1 + ring as f64 * d * ang.sin(),
                        );
                        if !point_in_poly(c, &self.boundary)
                            || self.nets[n].r.vias.iter().chain(self.nets[n].pre.vias.iter()).any(|o| dist(*o, c) < d - 1e-6)
                        {
                            continue;
                        }
                        let mut prims: Vec<Prim> = (0..self.grid.nl)
                            .map(|l| Prim::new(l, Geo::Circle { c, r: vr }, n as i32, clr))
                            .collect();
                        let stubs: Vec<Wire> = layers
                            .iter()
                            .map(|&l| Wire {
                                fill: false,
                                layer: l,
                                width,
                                pts: vec![v, c],
                            })
                            .collect();
                        for w in &stubs {
                            prims.extend(self.wire_prims(n, w));
                        }
                        if self.prims_conflict(&prims).is_some() {
                            continue;
                        }
                        self.stamp_net(n, -1);
                        self.nets[n].r.vias.push(c);
                        self.nets[n].r.wires.extend(stubs);
                        self.rebuild_route_prims(n);
                        self.stamp_net(n, 1);
                        placed += 1;
                    }
                }
                if placed < want {
                    self.via_shortfall += 1;
                }
            }
        }
    }

    /// Vias of one net must never overlap. Two routed vias closer than a via diameter are merged
    /// into one (at either of them, or in between) when the wires that end there still keep
    /// their clearances; identical vias are simply dropped. What cannot be merged is reported.
    fn resolve_via_overlaps(&mut self) {
        for n in 0..self.nets.len() {
            let (vr, clr) = (self.nets[n].via_r, self.nets[n].clr);
            let dia = 2.0 * vr;
            let mut guard = 0;
            'again: loop {
                guard += 1;
                if guard > 200 {
                    break;
                }
                let vias = self.nets[n].r.vias.clone();
                let fixed: Vec<P> = self.nets[n].pre.vias.clone();
                for i in 0..vias.len() {
                    // (via to replace, optional second routed via, positions to try)
                    let mut pair: Option<(P, Option<P>, Vec<P>)> = None;
                    if let Some(&f) = fixed.iter().find(|f| dist(**f, vias[i]) < dia - 1e-6) {
                        // against an existing via the routed one moves onto it
                        pair = Some((vias[i], None, vec![f]));
                    }
                    for j in i + 1..vias.len() {
                        if pair.is_none() && dist(vias[i], vias[j]) < dia - 1e-6 {
                            let mid = ((vias[i].0 + vias[j].0) / 2.0, (vias[i].1 + vias[j].1) / 2.0);
                            pair = Some((vias[i], Some(vias[j]), vec![vias[i], vias[j], mid]));
                        }
                    }
                    let Some((a, b, cands)) = pair else { continue };
                    let olds: Vec<P> = std::iter::once(a).chain(b).collect();
                    for p in cands {
                        let mut wires: Vec<Wire> = self.nets[n].r.wires.clone();
                        for w in wires.iter_mut() {
                            for q in w.pts.iter_mut() {
                                if olds.iter().any(|o| dist(*o, *q) < 1e-6) {
                                    *q = p;
                                }
                            }
                            w.pts.dedup_by(|x, y| dist(*x, *y) < 1e-9);
                        }
                        wires.retain(|w| w.fill || w.pts.len() >= 2);
                        let mut prims: Vec<Prim> = (0..self.grid.nl)
                            .map(|l| Prim::new(l, Geo::Circle { c: p, r: vr }, n as i32, clr))
                            .collect();
                        for w in wires.iter().filter(|w| !w.fill && w.pts.iter().any(|q| dist(*q, p) < 1e-9)) {
                            prims.extend(self.wire_prims(n, w));
                        }
                        if self.prims_conflict(&prims).is_some() {
                            continue;
                        }
                        self.stamp_net(n, -1);
                        let mut keep: Vec<P> = self.nets[n]
                            .r
                            .vias
                            .iter()
                            .copied()
                            .filter(|v| !olds.iter().any(|o| dist(*o, *v) < 1e-6))
                            .collect();
                        if b.is_some() {
                            keep.push(p);
                        }
                        self.nets[n].r.vias = keep;
                        self.nets[n].r.wires = wires;
                        self.rebuild_route_prims(n);
                        self.stamp_net(n, 1);
                        continue 'again;
                    }
                }
                break;
            }
            // whatever is still overlapping is reported
            let vias = &self.nets[n].r.vias;
            let left = (0..vias.len())
                .flat_map(|i| (i + 1..vias.len()).map(move |j| (i, j)))
                .filter(|&(i, j)| dist(vias[i], vias[j]) < dia - 1e-6)
                .count();
            if left > 0 {
                self.pour_warnings
                    .push(format!("{}: {} overlapping via pair(s) could not be merged", self.nets[n].name, left));
            }
        }
    }

    /// Stitching vias between the fill net's copper areas: wherever areas of that net on two
    /// layers lie on top of each other, a via on a regular pitch joins them, but only where the
    /// via fits (clearance to everything else) and lies completely inside the area on every layer
    /// it touches, and never inside the copper area of another net. Fill areas that end up without
    /// any copper of the net are removed again.
    fn stitch_fill_vias(&mut self) {
        if self.cfg.single_sided {
            return;
        }
        let Some(f) = self.fill_net_index() else { return };
        let nl = self.grid.nl;
        let pitch = self.cfg.stitch_pitch_mm / self.unit_mm.max(1e-12);
        let (vr, clr) = (self.nets[f].via_r, self.nets[f].clr);
        let mut polys: Vec<Vec<Vec<P>>> = vec![vec![]; nl];
        for (n, l, p) in &self.fill_polys {
            if *n == f {
                polys[*l].push(p.clone());
            }
        }
        for pl in self.planes.iter().filter(|p| !p.synthetic && p.net == f) {
            polys[pl.layer].push(pl.poly.clone());
        }
        let mut placed = 0usize;
        if pitch > 0.0 && polys.iter().filter(|p| !p.is_empty()).count() >= 2 {
            // Copper areas of other nets. A via is first placed where it touches none of them;
            // where that leaves no room (ground on one layer, power planes on the next) it may cut
            // them, but each area only as often as its size allows: one cut per (2 x pitch)^2 of
            // area, and cuts at least two pitches apart.
            let mut foreign: Vec<Vec<(usize, Vec<P>)>> = vec![vec![]; nl];
            let mut budget: Vec<usize> = vec![];
            let mm2 = self.unit_mm * self.unit_mm;
            let per_cut = (2.0 * pitch) * (2.0 * pitch) * mm2;
            let others: Vec<(usize, Vec<P>)> = self
                .fill_polys
                .iter()
                .filter(|p| p.0 != f)
                .map(|p| (p.1, p.2.clone()))
                .chain(self.planes.iter().filter(|p| !p.synthetic && p.net != f).map(|p| (p.layer, p.poly.clone())))
                .collect();
            for (l, poly) in others {
                budget.push(((crate::outline::ring_area(&poly).abs() * mm2) / per_cut.max(1e-9)).floor() as usize);
                foreign[l].push((budget.len() - 1, poly));
            }
            let mut used = vec![0usize; budget.len()];
            let mut cutters: Vec<P> = vec![];
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &(x, y) in &self.boundary {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
            }
            let mut existing: Vec<P> =
                self.nets[f].r.vias.iter().chain(self.nets[f].pre.vias.iter()).copied().collect();
            let sep = (pitch * 0.7).max(2.0 * vr * 1.2);
            let rim = |c: P| -> Vec<P> {
                let mut v = vec![c];
                for a in 0..8 {
                    let t = a as f64 * std::f64::consts::FRAC_PI_4;
                    v.push((c.0 + vr * 1.05 * t.cos(), c.1 + vr * 1.05 * t.sin()));
                }
                v
            };
            let q = pitch / 4.0;
            // Where something is: copper of any net (pads, traces, vias) as seen before any
            // stitching via exists, in buckets of the search distance.
            let near = self.cfg.stitch_near_mm / self.unit_mm.max(1e-12);
            let bucket = near.max(1e-9);
            let mut stuff: HashMap<(i64, i64), Vec<Prim>> = HashMap::new();
            if near > 0.0 {
                let all = self
                    .pins
                    .iter()
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets.iter().flat_map(|n| n.pre.dynp.iter().chain(n.r.dynp.iter())))
                    .filter(|p| !p.fill && !p.trace_only);
                for pr in all {
                    for bx in (pr.bb.0 / bucket).floor() as i64..=(pr.bb.2 / bucket).floor() as i64 {
                        for by in (pr.bb.1 / bucket).floor() as i64..=(pr.bb.3 / bucket).floor() as i64 {
                            stuff.entry((bx, by)).or_default().push(pr.clone());
                        }
                    }
                }
            }
            let something_near = |c: P| -> bool {
                if near <= 0.0 {
                    return true;
                }
                let (bx, by) = ((c.0 / bucket).floor() as i64, (c.1 / bucket).floor() as i64);
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        if stuff.get(&(bx + dx, by + dy)).is_some_and(|v| v.iter().any(|p| p.dist(c) <= near)) {
                            return true;
                        }
                    }
                }
                false
            };
            for allow_cuts in [false, true] {
                let mut row = 0;
                let mut y = y0 + pitch / 2.0;
                while y < y1 {
                    let mut x = x0 + if row % 2 == 0 { pitch / 2.0 } else { pitch };
                    while x < x1 {
                        for off in [(0.0, 0.0), (q, 0.0), (-q, 0.0), (0.0, q), (0.0, -q)] {
                            let c = (x + off.0, y + off.1);
                            if existing.iter().any(|o| dist(*o, c) < sep) {
                                break;
                            }
                            if !point_in_poly(c, &self.boundary) || !something_near(c) {
                                continue;
                            }
                            let pts = rim(c);
                            let mut joined = 0;
                            let mut bad = false;
                            let mut hit: Vec<usize> = vec![];
                            for l in 0..nl {
                                let inside =
                                    pts.iter().filter(|&&p| polys[l].iter().any(|poly| point_in_poly(p, poly))).count();
                                if inside == pts.len() {
                                    joined += 1;
                                } else if inside > 0 {
                                    bad = true;
                                    break;
                                }
                                for (id, poly) in &foreign[l] {
                                    if pts.iter().any(|&p| point_in_poly(p, poly)) {
                                        hit.push(*id);
                                    }
                                }
                            }
                            if bad || joined < 2 {
                                continue;
                            }
                            if !hit.is_empty()
                                && (!allow_cuts
                                    || hit.iter().any(|&id| used[id] >= budget[id])
                                    || cutters.iter().any(|o| dist(*o, c) < 2.0 * pitch))
                            {
                                continue;
                            }
                            let prims: Vec<Prim> =
                                (0..nl).map(|l| Prim::new(l, Geo::Circle { c, r: vr }, f as i32, clr)).collect();
                            if self.prims_conflict(&prims).is_some() {
                                continue;
                            }
                            self.stamp_dyn(&prims, 1);
                            self.nets[f].r.dynp.extend(prims);
                            self.nets[f].r.vias.push(c);
                            existing.push(c);
                            if !hit.is_empty() {
                                cutters.push(c);
                                for &id in &hit {
                                    used[id] += 1;
                                }
                            }
                            placed += 1;
                            break;
                        }
                        x += pitch;
                    }
                    y += pitch * 0.866;
                    row += 1;
                }
            }
        }
        // fill areas that hold no copper of the net are floating: drop them
        let keep: Vec<bool> = self
            .fill_polys
            .iter()
            .map(|(n, l, poly)| {
                if *n != f || !self.fill_applies(*l, f) {
                    return true;
                }
                self.pins
                    .iter()
                    .filter(|p| p.net == f as i32)
                    .flat_map(|p| p.prims.iter())
                    .chain(self.nets[f].pre.dynp.iter())
                    .chain(self.nets[f].r.dynp.iter())
                    .filter(|p| p.layer == *l && !p.trace_only && !p.fill)
                    .any(|p| point_in_poly(((p.bb.0 + p.bb.2) / 2.0, (p.bb.1 + p.bb.3) / 2.0), poly))
            })
            .collect();
        let dropped = keep.iter().filter(|k| !**k).count();
        let mut it = keep.into_iter();
        self.fill_polys.retain(|_| it.next().unwrap_or(true));
        if self.cfg.verbose {
            eprintln!(
                "fill {}: {} stitching via(s), {} floating area(s) dropped",
                self.nets[f].name, placed, dropped
            );
        }
    }

    /// Pull every routed wire tight (`cfg.tighten`): vertices that can be bypassed with a straight
    /// piece are dropped and the others slide toward the line between their neighbours as far as
    /// the exact clearances allow, which is the shortest path around what is in the way. With
    /// `tighten == 1` the result is then cut into 0/45/90 degree pieces; a wire for which that
    /// does not fit keeps its old shape. Ends, vias and the points where another wire attaches
    /// never move.
    pub(crate) fn tighten_traces(&mut self) {
        let mode = self.cfg.tighten;
        if mode == 0 {
            return;
        }
        let t0 = std::time::Instant::now();
        let (mut shortened, mut saved_len) = (0usize, 0.0f64);
        for n in 0..self.nets.len() {
            if self.cancelled() {
                return;
            }
            if self.nets[n].coupled || self.nets[n].guided || self.nets[n].skip {
                continue;
            }
            let old = self.nets[n].r.wires.clone();
            // points of this net that must stay: ends of wires, vias
            let mut fixed: Vec<(usize, P)> = vec![];
            for w in old.iter().filter(|w| !w.fill) {
                fixed.push((w.layer, w.pts[0]));
                fixed.push((w.layer, *w.pts.last().unwrap()));
            }
            for &v in &self.nets[n].r.vias {
                for l in 0..self.grid.nl {
                    fixed.push((l, v));
                }
            }
            let mut own_copper: Vec<Prim> = vec![];
            for &pi in &self.nets[n].pins {
                own_copper.extend(self.pins[pi].prims.iter().cloned());
            }
            own_copper.extend(self.nets[n].pre.dynp.iter().filter(|p| !p.trace_only).cloned());
            for j in &self.nets[n].r.jumpers {
                own_copper.extend(j.pads(n as i32, 0.0));
            }
            let mut result = old.clone();
            let mut changed = false;
            for (wi, w) in old.iter().enumerate() {
                if w.fill || w.pts.len() < 3 {
                    continue;
                }
                // something attached in the middle of a piece: leave the wire alone
                let attached_inside = fixed.iter().any(|(l, p)| {
                    *l == w.layer
                        && w.pts.windows(2).any(|s| dist_pt_seg(*p, s[0], s[1]) < 1e-6 && dist(*p, s[0]) > 1e-6 && dist(*p, s[1]) > 1e-6)
                });
                if attached_inside {
                    continue;
                }
                let is_fixed = |i: usize| -> bool {
                    i == 0 || i + 1 == w.pts.len() || fixed.iter().any(|(l, p)| *l == w.layer && dist(*p, w.pts[i]) < 1e-6)
                };
                let before: f64 = w.pts.windows(2).map(|s| dist(s[0], s[1])).sum();
                // copper of this net that the wire touches on its way (a pad it passes over, existing
                // wiring, a jumper pad): it has to be touched afterwards as well
                let touching: Vec<Prim> = own_copper
                    .iter()
                    .filter(|c| c.layer == w.layer && self.wire_touches(n, w, &w.pts, c))
                    .cloned()
                    .collect();
                let mut pts = self.tighten_pts(n, w, &is_fixed);
                if mode == 1 {
                    match self.octilinear_pts(n, w, &pts) {
                        Some(p) => pts = p,
                        None => continue,
                    }
                }
                if touching.iter().any(|c| !self.wire_touches(n, w, &pts, c)) {
                    continue;
                }
                let after: f64 = pts.windows(2).map(|s| dist(s[0], s[1])).sum();
                if after < before - 1e-6 && pts.len() >= 2 {
                    saved_len += before - after;
                    shortened += 1;
                    result[wi].pts = pts;
                    changed = true;
                }
            }
            if changed {
                self.stamp_net(n, -1);
                self.nets[n].r.wires = result;
                self.rebuild_route_prims(n);
                self.stamp_net(n, 1);
            }
        }
        if self.cfg.verbose {
            eprintln!(
                "tighten: {} wire(s) shortened by {:.1} in total ({:.2}s)",
                shortened,
                saved_len,
                t0.elapsed().as_secs_f32()
            );
        }
    }

    /// Does the polyline `pts` (width and layer of `w`) touch the copper `c`?
    fn wire_touches(&self, n: usize, w: &Wire, pts: &[P], c: &Prim) -> bool {
        pts.windows(2).any(|s| {
            let seg = Prim::new(w.layer, Geo::Seg { a: s[0], b: s[1], hw: w.width / 2.0 }, n as i32, 0.0);
            seg.gap(c) <= 1e-6
        })
    }

    /// Is the straight piece a-b of wire `w` (same layer and width) free of conflicts?
    fn piece_ok(&self, n: usize, w: &Wire, a: P, b: P) -> bool {
        let piece = Wire { fill: false, layer: w.layer, width: w.width, pts: vec![a, b] };
        self.prims_conflict(&self.wire_prims(n, &piece)).is_none()
    }

    /// Taut-string version of the polyline of `w`, free angles.
    fn tighten_pts(&self, n: usize, w: &Wire, is_fixed: &dyn Fn(usize) -> bool) -> Vec<P> {
        let mut pts = w.pts.clone();
        // the fixed flags travel with the vertices
        let mut fix: Vec<bool> = (0..pts.len()).map(is_fixed).collect();
        for _pass in 0..10 {
            let mut improved = false;
            // 1. bypass vertices with a straight piece
            let mut i = 0;
            while i + 2 < pts.len() {
                // the farthest vertex that can be reached before the next fixed vertex
                let mut jmax = pts.len() - 1;
                for (k, f) in fix.iter().enumerate().skip(i + 1) {
                    if *f {
                        jmax = k;
                        break;
                    }
                }
                let mut j = jmax;
                while j >= i + 2 {
                    if self.piece_ok(n, w, pts[i], pts[j]) {
                        pts.drain(i + 1..j);
                        fix.drain(i + 1..j);
                        improved = true;
                        break;
                    }
                    j -= 1;
                }
                i += 1;
            }
            // 2. slide the others toward the line between their neighbours
            for k in 1..pts.len().saturating_sub(1) {
                if fix[k] {
                    continue;
                }
                let (p, q, v) = (pts[k - 1], pts[k + 1], pts[k]);
                // closest point of p-q to v
                let (dx, dy) = (q.0 - p.0, q.1 - p.1);
                let l2 = dx * dx + dy * dy;
                if l2 < 1e-12 {
                    continue;
                }
                let t = (((v.0 - p.0) * dx + (v.1 - p.1) * dy) / l2).clamp(0.0, 1.0);
                let target = (p.0 + t * dx, p.1 + t * dy);
                if dist(v, target) < 1e-6 {
                    continue;
                }
                let at = |f: f64| (v.0 + f * (target.0 - v.0), v.1 + f * (target.1 - v.1));
                let ok = |f: f64| {
                    let c = at(f);
                    self.piece_ok(n, w, p, c) && self.piece_ok(n, w, c, q)
                };
                let f = if ok(1.0) {
                    1.0
                } else {
                    let (mut lo, mut hi) = (0.0, 1.0);
                    for _ in 0..10 {
                        let mid = (lo + hi) / 2.0;
                        if ok(mid) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    lo
                };
                if f > 1e-3 {
                    pts[k] = at(f);
                    improved = true;
                }
            }
            if !improved {
                break;
            }
        }
        pts
    }

    /// The polyline `pts` (the tightened version of `w`) cut into 0/45/90 degree pieces; None
    /// when a piece does not fit.
    fn octilinear_pts(&self, n: usize, w: &Wire, pts: &[P]) -> Option<Vec<P>> {
        let is_oct = |a: P, b: P| {
            let (dx, dy) = ((b.0 - a.0).abs(), (b.1 - a.1).abs());
            dx < 1e-6 || dy < 1e-6 || (dx - dy).abs() < 1e-6
        };
        let mut out = vec![pts[0]];
        for s in pts.windows(2) {
            let (a, b) = (s[0], s[1]);
            if is_oct(a, b) {
                out.push(b);
                continue;
            }
            // the two ways round the corner: diagonal first or straight first
            let first = octilinear(a, b);
            let mut second = octilinear(b, a);
            second.reverse();
            let mut done = false;
            for cand in [first, second] {
                let piece = Wire { fill: false, layer: w.layer, width: w.width, pts: cand.clone() };
                if self.prims_conflict(&self.wire_prims(n, &piece)).is_none() {
                    out.extend(cand.into_iter().skip(1));
                    done = true;
                    break;
                }
            }
            if !done {
                return None;
            }
        }
        Some(out)
    }

    /// Last resort for connections that stay open (with the option to nudge parts): routed vias of
    /// other nets that stand near the gap between the pieces of an open net are moved a little,
    /// one by one, in pairs or all together by the same offset (a via can only move together
    /// with the wires that end in it), the exact clearances are checked, and the open net is
    /// routed again. A move stays only when that net then connects; nothing else is re-routed.
    pub(crate) fn nudge_vias(&mut self) {
        let t0 = std::time::Instant::now();
        let failing: Vec<usize> = (0..self.nets.len())
            .filter(|&n| {
                !self.nets[n].unconnected.is_empty()
                    && !self.nets[n].skip
                    && !self.nets[n].coupled
                    && !self.nets[n].guided
                    && self.nets[n].pair.is_none()
                    && self.nets[n].bundle.is_none()
            })
            .collect();
        for n in failing {
            if self.cancelled() || t0.elapsed().as_secs() > 12 {
                return;
            }
            if self.nets[n].unconnected.is_empty() {
                continue;
            }
            let isl = crate::drc::net_islands(self, n);
            if isl.len() < 2 {
                continue;
            }
            // the shortest stretch between two pieces of the net
            let mut best: Option<(f64, P, P)> = None;
            for (i, a) in isl.iter().enumerate() {
                for b in isl.iter().skip(i + 1) {
                    for &pa in &a.pins {
                        for &pb in &b.pins {
                            let (ca, cb) = (self.pins[pa].c, self.pins[pb].c);
                            let d = dist(ca, cb);
                            if best.is_none_or(|x| d < x.0) {
                                best = Some((d, ca, cb));
                            }
                        }
                    }
                }
            }
            let Some((_, a, b)) = best else { continue };
            // first the way a person does it: lift the vias out, route, put them back
            if self.push_vias_route(n) {
                if self.cfg.verbose {
                    eprintln!("vias pushed aside: {} is routed", self.nets[n].name);
                }
                continue;
            }
            // vias of other nets near it
            let via_d = 2.0 * self.nets[n].via_r;
            let reach = (3.0 / self.unit_mm.max(1e-12)).max(3.0 * via_d);
            let mut near: Vec<(f64, usize, usize)> = vec![];
            for m in 0..self.nets.len() {
                if m == n || self.nets[m].coupled || self.nets[m].guided || self.nets[m].skip {
                    continue;
                }
                for (vi, &v) in self.nets[m].r.vias.iter().enumerate() {
                    let d = dist_pt_seg(v, a, b);
                    if d < reach {
                        near.push((d, m, vi));
                    }
                }
            }
            near.sort_by(|x, y| x.0.total_cmp(&y.0));
            near.truncate(4);
            if near.is_empty() {
                continue;
            }
            // groups of vias moved by the same offset: each alone, every pair, all of them
            let mut groups: Vec<Vec<(usize, usize)>> = near.iter().map(|&(_, m, vi)| vec![(m, vi)]).collect();
            for i in 0..near.len() {
                for j in i + 1..near.len() {
                    groups.push(vec![(near[i].1, near[i].2), (near[j].1, near[j].2)]);
                }
            }
            if near.len() > 2 {
                groups.push(near.iter().map(|&(_, m, vi)| (m, vi)).collect());
            }
            'groups: for group in &groups {
                for step_f in [0.25, 0.5, 0.75, 1.0] {
                    for d in 0..8 {
                        if self.cancelled() || t0.elapsed().as_secs() > 12 {
                            return;
                        }
                        let ang = d as f64 * std::f64::consts::FRAC_PI_4;
                        let off = (via_d * step_f * ang.cos(), via_d * step_f * ang.sin());
                        if self.try_via_move(n, group, off) {
                            if self.cfg.verbose {
                                eprintln!(
                                    "nudged {} via(s) by {:.2} mm: {} is routed",
                                    group.len(),
                                    via_d * step_f * self.unit_mm,
                                    self.nets[n].name
                                );
                            }
                            break 'groups;
                        }
                    }
                }
            }
        }
    }

    /// Via nudging, the way a person does it. The routed vias of other nets near the gap of the
    /// open net `n` are moved a little (the last piece of the wires that end in a via follows it)
    /// and `n` is routed again in the room that is made. Orders tried, each checked against the
    /// exact clearances, and only kept when `n` ends up connected (otherwise everything is put
    /// back exactly as it was):
    ///  1. every via stays as a smaller "ghost" while `n` is routed, so the path keeps off its
    ///     centre, then each via is pushed out by just what is needed (ghost sizes 0.85, 0.6, 0.35);
    ///  2. every via is moved just clear of the straight line between the two pieces of `n`, all
    ///     to the same side of it, first one side, then the other;
    ///  3. the vias are lifted out entirely, `n` is routed, the vias go back as near as possible.
    pub(crate) fn push_vias_route(&mut self, n: usize) -> bool {
        for ghost in [0.85, 0.6, 0.35] {
            if self.push_vias_try(n, 3, ghost) {
                return true;
            }
        }
        if self.push_vias_try(n, 0, 0.0) || self.push_vias_try(n, 1, 0.0) || self.push_vias_try(n, 2, 0.0) {
            return true;
        }
        // 4. push and shove: the net squeezes through, the vias and traces around it are pushed
        //    aside, which pushes their neighbours in turn
        self.shove_route(n)
    }

    /// Put a lifted via of net `m` (and the wire pieces `stubs` that end in it) at the nearest
    /// place to `v` where it is clean; with `clear_of` = (a, b, distance, side), only places at
    /// least that far from the segment a-b and on that side of the line (side 0: either). False
    /// when there is none.
    fn place_lifted_via(&mut self, m: usize, v: P, stubs: &[ViaStub], clear_of: Option<(P, P, f64, f64)>) -> bool {
        let via_d = 2.0 * self.nets[m].via_r;
        let mut spots: Vec<P> = vec![v];
        for k in 1..=16 {
            let ring = 0.1 * k as f64;
            for d in 0..16 {
                let ang = d as f64 * std::f64::consts::FRAC_PI_8;
                spots.push((v.0 + via_d * ring * ang.cos(), v.1 + via_d * ring * ang.sin()));
            }
        }
        let (clr, via_r) = (self.nets[m].clr, self.nets[m].via_r);
        for p in spots {
            if !point_in_poly(p, &self.boundary) {
                continue;
            }
            if let Some((a, b, need, side)) = clear_of {
                if dist_pt_seg(p, a, b) < need {
                    continue;
                }
                let cross = (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0);
                if side != 0.0 && cross * side < 0.0 {
                    continue;
                }
            }
            let mut prims: Vec<Prim> = (0..self.grid.nl).map(|l| Prim::new(l, Geo::Circle { c: p, r: via_r }, m as i32, clr)).collect();
            for s in stubs {
                if dist(s.from, p) > 1e-9 {
                    prims.push(Prim::new(s.layer, Geo::Seg { a: s.from, b: p, hw: s.width / 2.0 }, m as i32, clr));
                }
            }
            if self.prims_conflict(&prims).is_some() {
                continue;
            }
            self.stamp_net(m, -1);
            self.nets[m].r.vias.push(p);
            for s in stubs {
                if dist(s.from, p) > 1e-9 {
                    self.nets[m].r.wires.push(Wire { fill: false, layer: s.layer, width: s.width, pts: vec![s.from, p] });
                }
            }
            self.rebuild_route_prims(m);
            self.stamp_net(m, 1);
            return true;
        }
        false
    }

    /// One order of `push_vias_route`: `strategy` 3 with a ghost size, 0 / 1 (one side / the other
    /// side of the line), or 2 (lift, route, put back).
    fn push_vias_try(&mut self, n: usize, strategy: u8, ghost: f64) -> bool {
        let isl = crate::drc::net_islands(self, n);
        if isl.len() < 2 {
            return false;
        }
        // the shortest stretch between two pieces of the net
        let mut best: Option<(f64, P, P)> = None;
        for (i, a) in isl.iter().enumerate() {
            for b in isl.iter().skip(i + 1) {
                for &pa in &a.pins {
                    for &pb in &b.pins {
                        let (ca, cb) = (self.pins[pa].c, self.pins[pb].c);
                        let d = dist(ca, cb);
                        if best.is_none_or(|x| d < x.0) {
                            best = Some((d, ca, cb));
                        }
                    }
                }
            }
        }
        let Some((_, a, b)) = best else { return false };
        let reach = (3.0 / self.unit_mm.max(1e-12)).max(3.0 * 2.0 * self.nets[n].via_r);
        // the nearest vias of other nets, and where they are (positions, not indices: indices move)
        let mut near: Vec<(f64, usize, P)> = vec![];
        for m in 0..self.nets.len() {
            if m == n || self.nets[m].coupled || self.nets[m].guided || self.nets[m].skip || self.nets[m].pair.is_some() || self.nets[m].bundle.is_some() {
                continue;
            }
            for &v in &self.nets[m].r.vias {
                let d = dist_pt_seg(v, a, b);
                if d < reach {
                    near.push((d, m, v));
                }
            }
        }
        near.sort_by(|x, y| x.0.total_cmp(&y.0));
        near.truncate(8);
        if near.is_empty() {
            return false;
        }
        // a wire that runs from one lifted via straight to another one would lose both of its ends
        for &(_, m, v) in &near {
            for w in self.nets[m].r.wires.iter().filter(|w| !w.fill && w.pts.len() == 2) {
                let other = if dist(w.pts[0], v) < 1e-6 { Some(w.pts[1]) } else if dist(w.pts[1], v) < 1e-6 { Some(w.pts[0]) } else { None };
                if let Some(o) = other {
                    if near.iter().any(|&(_, m2, v2)| m2 == m && dist(v2, v) > 1e-6 && dist(v2, o) < 1e-6) {
                        return false;
                    }
                }
            }
        }
        let mut affected: Vec<usize> = near.iter().map(|x| x.1).collect();
        affected.sort_unstable();
        affected.dedup();
        // everything that can change, to put it back exactly (copper, stamps, what is still open)
        let saved: Vec<(usize, Routed, Vec<usize>)> =
            affected.iter().map(|&m| (m, self.nets[m].r.clone(), self.nets[m].unconnected.clone())).collect();
        let saved_n = (self.nets[n].r.clone(), self.nets[n].unconnected.clone());
        // how many pieces each owner has now: moving a via must not cut a net in two
        let pieces_before: Vec<(usize, usize)> = affected.iter().map(|&m| (m, crate::drc::net_islands(self, m).len())).collect();
        let revert = |me: &mut Router| {
            for (m, r, unc) in &saved {
                let prims = std::mem::take(&mut me.nets[*m].r.dynp);
                me.stamp_dyn(&prims, -1);
                me.nets[*m].r = r.clone();
                me.nets[*m].unconnected = unc.clone();
                let prims = me.nets[*m].r.dynp.clone();
                me.stamp_dyn(&prims, 1);
            }
            let prims = std::mem::take(&mut me.nets[n].r.dynp);
            me.stamp_dyn(&prims, -1);
            me.nets[n].r = saved_n.0.clone();
            me.nets[n].unconnected = saved_n.1.clone();
            let prims = me.nets[n].r.dynp.clone();
            me.stamp_dyn(&prims, 1);
        };
        // lift the vias out: remove each via and the last piece of the wires that end in it
        let mut lifted: Vec<(usize, P, Vec<ViaStub>)> = vec![];
        for &(_, m, v) in &near {
            self.stamp_net(m, -1);
            let mut stubs: Vec<ViaStub> = vec![];
            let mut keep: Vec<Wire> = vec![];
            for mut w in std::mem::take(&mut self.nets[m].r.wires) {
                let k = w.pts.len();
                if w.fill || k < 2 || (dist(w.pts[0], v) > 1e-6 && dist(w.pts[k - 1], v) > 1e-6) || dist(w.pts[0], w.pts[k - 1]) < 1e-6 {
                    keep.push(w);
                    continue;
                }
                if dist(w.pts[0], v) < 1e-6 {
                    stubs.push(ViaStub { layer: w.layer, from: w.pts[1], width: w.width });
                    w.pts.remove(0);
                } else {
                    stubs.push(ViaStub { layer: w.layer, from: w.pts[k - 2], width: w.width });
                    w.pts.pop();
                }
                if w.pts.len() >= 2 {
                    keep.push(w);
                }
            }
            self.nets[m].r.wires = keep;
            if let Some(i) = self.nets[m].r.vias.iter().position(|q| dist(*q, v) < 1e-6) {
                self.nets[m].r.vias.remove(i);
            }
            self.rebuild_route_prims(m);
            self.stamp_net(m, 1);
            lifted.push((m, v, stubs));
        }
        // strategies 0 and 1: put the vias just clear of the straight line first, all on one side
        if strategy <= 1 {
            let side = if strategy == 0 { 1.0 } else { -1.0 };
            let half = self.fields[self.nets[n].narrow_tf].p;
            for (m, v, stubs) in &lifted {
                let need = self.nets[*m].via_r + self.nets[*m].clr.max(self.nets[n].clr) + half + 0.02;
                let ok = self.place_lifted_via(*m, *v, stubs, Some((a, b, need, side)))
                    // none clears the line: leave it where it was, the router may go round
                    || self.place_lifted_via(*m, *v, stubs, None);
                if !ok {
                    revert(self);
                    return false;
                }
            }
        }
        // strategy 3: every via stays as a smaller ghost (a circle of `ghost` times its radius)
        // while the net is routed, so the path keeps off its centre
        let mut ghosts: Vec<Prim> = vec![];
        if strategy == 3 {
            for (m, v, _) in &lifted {
                for l in 0..self.grid.nl {
                    ghosts.push(Prim::new(l, Geo::Circle { c: *v, r: self.nets[*m].via_r * ghost }, *m as i32, self.nets[*m].clr));
                }
            }
            self.stamp_dyn(&ghosts, 1);
        }
        // route the open net through the room; a corridor can be narrower than the grid can
        // express, so the fine local search gets its turn too
        self.rip_net(n);
        self.route_net(n, false);
        if !self.nets[n].unconnected.is_empty() {
            self.adaptive_net(n);
        }
        if !ghosts.is_empty() {
            self.stamp_dyn(&ghosts, -1);
        }
        if !self.nets[n].unconnected.is_empty() {
            revert(self);
            return false;
        }
        let prims = self.nets[n].r.dynp.clone();
        if self.prims_conflict(&prims).is_some() {
            revert(self);
            return false;
        }
        // strategies 2 and 3: the vias go back as near as possible to where they were
        if strategy >= 2 {
            for (m, v, stubs) in &lifted {
                if !self.place_lifted_via(*m, *v, stubs, None) {
                    revert(self);
                    return false;
                }
            }
        }
        // the owners of the moved vias must still be in as few pieces as before, and the open net
        // must really be one piece now (the geometric check, not only the router's bookkeeping)
        for &(m, before) in &pieces_before {
            if crate::drc::net_islands(self, m).len() > before {
                revert(self);
                return false;
            }
        }
        if crate::drc::net_islands(self, n).len() > 1 {
            revert(self);
            return false;
        }
        true
    }

    /// Move the vias `group` ((net, index in its via list)) by `off`, route net `n` again and
    /// keep it all when `n` connects; otherwise everything is put back. Returns whether it stayed.
    fn try_via_move(&mut self, n: usize, group: &[(usize, usize)], off: P) -> bool {
        let mut moved_nets: Vec<usize> = group.iter().map(|g| g.0).collect();
        moved_nets.sort_unstable();
        moved_nets.dedup();
        let saved: Vec<(usize, Routed)> = moved_nets.iter().map(|&m| (m, self.nets[m].r.clone())).collect();
        let saved_n = (self.nets[n].r.clone(), self.nets[n].unconnected.clone());
        let revert = |me: &mut Router, with_n: bool| {
            if with_n {
                let prims = std::mem::take(&mut me.nets[n].r.dynp);
                me.stamp_dyn(&prims, -1);
                me.nets[n].r = saved_n.0.clone();
                me.nets[n].unconnected = saved_n.1.clone();
                let prims = me.nets[n].r.dynp.clone();
                me.stamp_dyn(&prims, 1);
            }
            for (m, r) in &saved {
                let prims = std::mem::take(&mut me.nets[*m].r.dynp);
                me.stamp_dyn(&prims, -1);
                me.nets[*m].r = r.clone();
                let prims = me.nets[*m].r.dynp.clone();
                me.stamp_dyn(&prims, 1);
            }
        };
        // move the vias and the wire ends in them
        for &m in &moved_nets {
            self.stamp_net(m, -1);
            for &(gm, vi) in group.iter().filter(|g| g.0 == m) {
                let old = saved.iter().find(|s| s.0 == gm).map(|s| s.1.vias[vi]).unwrap_or((0.0, 0.0));
                let new = (old.0 + off.0, old.1 + off.1);
                self.nets[m].r.vias[vi] = new;
                for w in self.nets[m].r.wires.iter_mut().filter(|w| !w.fill) {
                    for q in w.pts.iter_mut() {
                        if dist(*q, old) < 1e-6 {
                            *q = new;
                        }
                    }
                }
            }
            self.rebuild_route_prims(m);
            self.stamp_net(m, 1);
        }
        // exact check of everything that changed, and the new places must lie on the board
        let mut ok = true;
        for &m in &moved_nets {
            for &(gm, vi) in group.iter().filter(|g| g.0 == m) {
                if !point_in_poly(self.nets[gm].r.vias[vi], &self.boundary) {
                    ok = false;
                }
            }
            let prims = self.nets[m].r.dynp.clone();
            if ok && self.prims_conflict(&prims).is_some() {
                ok = false;
            }
        }
        if !ok {
            revert(self, false);
            return false;
        }
        // route the open net again in the room that was made; a corridor between traces can be
        // narrower than the grid can express, so the fine local search gets its turn too
        self.rip_net(n);
        self.route_net(n, false);
        if !self.nets[n].unconnected.is_empty() {
            self.adaptive_net(n);
        }
        let connected = self.nets[n].unconnected.is_empty();
        let clean = connected && {
            let prims = self.nets[n].r.dynp.clone();
            self.prims_conflict(&prims).is_none()
        };
        if clean {
            return true;
        }
        revert(self, true);
        false
    }

    pub(crate) fn rebuild_route_prims(&mut self, n: usize) {
        let mut prims: Vec<Prim> = self.nets[n]
            .r
            .wires
            .iter()
            .flat_map(|w| {
                let mut v = self.wire_prims(n, w);
                for p in v.iter_mut() {
                    p.fill = w.fill;
                }
                v
            })
            .collect();
        for &v in &self.nets[n].r.vias {
            for l in 0..self.grid.nl {
                prims.push(Prim::new(
                    l,
                    Geo::Circle {
                        c: v,
                        r: self.nets[n].via_r,
                    },
                    n as i32,
                    self.nets[n].clr,
                ));
            }
        }
        for j in &self.nets[n].r.jumpers {
            prims.extend(j.pads(n as i32, self.nets[n].clr));
        }
        self.nets[n].r.dynp = prims;
    }

    /// End every trace in the middle of the pad it touches instead of at whichever grid cell
    /// inside the pad the search happened to stop at (checked against clearance).
    fn center_on_pads(&mut self) {
        for n in 0..self.nets.len() {
            if self.cancelled() {
                return;
            }
            let pads: Vec<(usize, P, Prim)> = self.nets[n]
                .pins
                .iter()
                .flat_map(|&pi| self.pins[pi].prims.iter())
                .map(|pr| (pr.layer, ((pr.bb.0 + pr.bb.2) / 2.0, (pr.bb.1 + pr.bb.3) / 2.0), pr.clone()))
                .collect();
            if pads.is_empty() {
                continue;
            }
            let mut changed = false;
            for wi in 0..self.nets[n].r.wires.len() {
                if self.nets[n].r.wires[wi].fill {
                    continue;
                }
                for at_end in [false, true] {
                    let w = self.nets[n].r.wires[wi].clone();
                    let e = if at_end { *w.pts.last().unwrap() } else { w.pts[0] };
                    let Some((_, c, _)) = pads
                        .iter()
                        .find(|(l, c, pr)| *l == w.layer && pr.dist(e) <= 1e-9 && pr.dist(*c) <= 1e-9)
                    else {
                        continue;
                    };
                    if dist(e, *c) < 1e-6 {
                        continue;
                    }
                    let stub = octilinear(e, *c);
                    let mut nw = w.clone();
                    if at_end {
                        nw.pts.extend_from_slice(&stub[1..]);
                    } else {
                        let mut p: Vec<P> = stub.iter().rev().cloned().collect();
                        p.extend_from_slice(&w.pts[1..]);
                        nw.pts = p;
                    }
                    let stub_wire = Wire {
                        fill: false,
                        layer: w.layer,
                        width: w.width,
                        pts: stub.clone(),
                    };
                    // the stub must not touch foreign copper (it lies inside our own pad)
                    if self.prims_conflict(&self.wire_prims(n, &stub_wire)).is_none() {
                        self.nets[n].r.wires[wi] = nw;
                        changed = true;
                    }
                }
            }
            if changed {
                self.stamp_net(n, -1);
                self.rebuild_route_prims(n);
                self.stamp_net(n, 1);
            }
        }
    }

    /// Nets that ended up in several pieces (e.g. a plane cut by vias): join the pieces with a
    /// trace on a routable layer using at most `bridge_vias` new vias per bridge. Pieces are
    /// joined as a tree, so no loops are created.
    pub(crate) fn bridge_islands(&mut self, only: Option<&[usize]>) {
        if self.cfg.bridge_vias == 0 {
            return;
        }
        let full = (0, 0, self.grid.nx as i32 - 1, self.grid.ny as i32 - 1);
        let maxv = self.cfg.bridge_vias;
        for n in 0..self.nets.len() {
            if self.nets[n].pins.len() < 2
                || self.nets[n].skip
                || only.is_some_and(|o| !o.contains(&n))
            {
                continue;
            }
            for _ in 0..8 {
                if self.cancelled() {
                    return;
                }
                let isl = crate::drc::net_islands(self, n);
                if isl.len() < 2 {
                    break;
                }
                // grow the first piece towards the closest other piece
                let mut best = (f64::MAX, 1usize);
                for (j, other) in isl.iter().enumerate().skip(1) {
                    for &a in &isl[0].pins {
                        for &b in &other.pins {
                            let d = dist(self.pins[a].c, self.pins[b].c);
                            if d < best.0 {
                                best = (d, j);
                            }
                        }
                    }
                }
                let cells = |me: &Router, i: &crate::drc::Island| -> Vec<Cell> {
                    let mut v: Vec<Cell> = vec![];
                    for pr in &i.prims {
                        if me.excl[pr.layer] {
                            continue;
                        }
                        if pr.fill {
                            continue;
                        }
                        v.extend(crate::prewire::prim_cells(&me.grid, pr));
                    }
                    v.sort();
                    v.dedup();
                    v
                };
                let seeds = cells(self, &isl[0]);
                let targets = cells(self, &isl[best.1]);
                if self.cfg.verbose {
                    eprintln!(
                        "bridge {}: {} islands, pins {:?}, seeds {} targets {}",
                        self.nets[n].name,
                        isl.len(),
                        isl.iter().map(|i| i.pins.len()).collect::<Vec<_>>(),
                        seeds.len(),
                        targets.len()
                    );
                }
                if seeds.is_empty() || targets.is_empty() {
                    break;
                }
                self.stamp_net(n, -1);
                self.cur_net = n;
                let fi = (self.nets[n].tf, self.nets[n].vf);
                let base = self.cfg.via_cost;
                let mut found = None;
                for mult in [1.0f32, 4.0, 16.0, 64.0] {
                    self.cfg.via_cost = base * mult;
                    let p = self.astar(fi, &seeds, &targets, false, full, 0.0);
                    self.cfg.via_cost = base;
                    if let Some(p) = p {
                        let nv = p.windows(2).filter(|w| w[0].0 != w[1].0).count();
                        if nv <= maxv {
                            found = Some(p);
                            break;
                        }
                    }
                }
                let requested = self.nets[n].width;
                let mut use_width = requested;
                if found.is_none() && self.cfg.allow_neckdown && self.min_width > 0.0 {
                    let thin = self.min_width.min(requested);
                    let mut f = Field::new(&self.grid, self.nets[n].clr, thin / 2.0, false);
                    for pr in self
                        .fixed
                        .iter()
                        .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                        .chain(
                            self.nets
                                .iter()
                                .flat_map(|ns| ns.pre.dynp.iter().chain(ns.r.dynp.iter())),
                        )
                    {
                        if pr.net != n as i32 {
                            f.stamp(&self.grid, pr, 1, false);
                        }
                    }
                    let nf = self.fields.len();
                    self.fields.push(f);
                    if let Some(p) = self.astar((nf, fi.1), &seeds, &targets, false, full, 0.0) {
                        if p.windows(2).filter(|w| w[0].0 != w[1].0).count() <= maxv {
                            found = Some(p);
                            use_width = thin;
                        }
                    }
                    self.fields.pop();
                }
                let mut stripped = false;
                if found.is_none() && self.cfg.allow_plane_jumps {
                    // generated plane copper is not an obstacle for a jump: it is poured again
                    // around the new trace afterwards
                    self.strip_plane_fill(n);
                    stripped = true;
                    self.jump_mode = true;
                    let p = self.astar(fi, &seeds, &targets, false, full, 0.0);
                    self.jump_mode = false;
                    if self.cfg.verbose {
                        eprintln!(
                            "bridge jump: {:?}",
                            p.as_ref().map(|p| (
                                p.len(),
                                p.iter().filter(|c| self.excl[c.0]).count(),
                                p.windows(2).filter(|w| w[0].0 != w[1].0).count(),
                                p[0],
                                p[p.len() - 1]
                            ))
                        );
                    }
                    if let Some(p) = p {
                        let on_plane = p.iter().filter(|c| self.excl[c.0]).count();
                        let limit = self.cfg.plane_jump_mm / self.unit_mm.max(1e-12) / self.grid.g;
                        if on_plane as f64 <= limit
                            && p.windows(2).filter(|w| w[0].0 != w[1].0).count() <= maxv + 2
                        {
                            found = Some(p);
                            self.jumps_used += 1;
                        }
                    }
                }
                self.cur_net = usize::MAX;
                match found {
                    Some(p) => {
                        self.nets[n].width = use_width;
                        self.commit_path(n, &p, requested);
                        self.nets[n].width = requested;
                        self.stamp_net(n, 1);
                        if stripped {
                            self.fill_copper(true);
                        }
                    }
                    None => {
                        self.stamp_net(n, 1);
                        if stripped {
                            self.fill_copper(true);
                        }
                        break;
                    }
                }
            }
        }
    }

    /// Plane copper for the CAD as a few hundred wide tracks instead of tens of thousands of
    /// thin strokes: big rectangles of whole blocks (one thick track each) plus thin connectors
    /// from the net's pads / vias to them. Everything lies inside the verified free region.
    fn compact_fill(&self, reach: &[bool], seeds: &[usize], width: f64, layer: usize) -> Vec<Wire> {
        let g = &self.grid;
        let (nx, ny) = (g.nx, g.ny);
        let nn = nx * ny;
        let b = ((self.cfg.plane_block_mm / self.unit_mm.max(1e-12) / g.g).round() as usize).max(2);
        let (cw, ch) = (nx / b, ny / b);
        let mut coarse = vec![false; cw * ch];
        for by in 0..ch {
            for bx in 0..cw {
                coarse[by * cw + bx] = (0..b).all(|dy| {
                    let row = (by * b + dy) * nx + bx * b;
                    reach[row..row + b].iter().all(|&r| r)
                });
            }
        }
        let mut used = vec![false; cw * ch];
        let mut rects: Vec<(usize, usize, usize, usize)> = vec![];
        for by in 0..ch {
            for bx in 0..cw {
                let i = by * cw + bx;
                if !coarse[i] || used[i] {
                    continue;
                }
                let mut x1 = bx;
                while x1 + 1 < cw && coarse[by * cw + x1 + 1] && !used[by * cw + x1 + 1] {
                    x1 += 1;
                }
                let mut y1 = by;
                'grow: while y1 + 1 < ch {
                    for x in bx..=x1 {
                        let j = (y1 + 1) * cw + x;
                        if !coarse[j] || used[j] {
                            break 'grow;
                        }
                    }
                    y1 += 1;
                }
                for y in by..=y1 {
                    for x in bx..=x1 {
                        used[y * cw + x] = true;
                    }
                }
                rects.push((bx, by, x1, y1));
            }
        }
        let mut wires: Vec<Wire> = vec![];
        let stroke = |a: P, c: P, w: f64| Wire {
            fill: true,
            layer,
            width: w,
            pts: vec![a, if dist(a, c) < 1e-9 { (c.0 + g.g * 0.001, c.1) } else { c }],
        };
        let mut covered = vec![false; nn];
        for &(bx, by, x1, y1) in &rects {
            for y in by * b..(y1 + 1) * b {
                for x in bx * b..(x1 + 1) * b {
                    covered[y * nx + x] = true;
                }
            }
            let xmin = g.x0 + (bx * b) as f64 * g.g;
            let xmax = g.x0 + ((x1 + 1) * b - 1) as f64 * g.g;
            let ymin = g.y0 + (by * b) as f64 * g.g;
            let ymax = g.y0 + ((y1 + 1) * b - 1) as f64 * g.g;
            let (w, h) = (xmax - xmin, ymax - ymin);
            if w >= h {
                let ym = (ymin + ymax) / 2.0;
                wires.push(stroke((xmin + h / 2.0, ym), (xmax - h / 2.0, ym), h));
            } else {
                let xm = (xmin + xmax) / 2.0;
                wires.push(stroke((xm, ymin + w / 2.0), (xm, ymax - w / 2.0), w));
            }
        }
        // thin connectors: breadth-first from the big rectangles through free cells
        let mut parent = vec![u32::MAX; nn];
        let mut q: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        for c in 0..nn {
            if covered[c] {
                parent[c] = c as u32;
                q.push_back(c);
            }
        }
        while let Some(c) = q.pop_front() {
            let (x, y) = (c % nx, c / nx);
            let nb = [
                if x > 0 { c - 1 } else { usize::MAX },
                if x + 1 < nx { c + 1 } else { usize::MAX },
                if y > 0 { c - nx } else { usize::MAX },
                if y + 1 < ny { c + nx } else { usize::MAX },
            ];
            for cc in nb {
                if cc != usize::MAX && reach[cc] && parent[cc] == u32::MAX {
                    parent[cc] = c as u32;
                    q.push_back(cc);
                }
            }
        }
        let center = |c: usize| g.center((c % nx) as i32, (c / nx) as i32);
        let mut seen_island = vec![false; nn];
        for &s in seeds {
            if covered[s] {
                continue;
            }
            if parent[s] != u32::MAX {
                let mut pts = vec![center(s)];
                let mut c = s;
                while !covered[c] {
                    covered[c] = true;
                    c = parent[c] as usize;
                    pts.push(center(c));
                }
                let pts = crate::pair::simplify(&pts);
                if pts.len() >= 2 {
                    wires.push(Wire {
                        fill: true,
                        layer,
                        width,
                        pts,
                    });
                }
            } else if !seen_island[s] {
                // a free pocket without room for any big rectangle: fill it cell row by row
                let mut cells: Vec<usize> = vec![];
                let mut q2 = std::collections::VecDeque::new();
                seen_island[s] = true;
                q2.push_back(s);
                while let Some(c) = q2.pop_front() {
                    cells.push(c);
                    let (x, y) = (c % nx, c / nx);
                    let nb = [
                        if x > 0 { c - 1 } else { usize::MAX },
                        if x + 1 < nx { c + 1 } else { usize::MAX },
                        if y > 0 { c - nx } else { usize::MAX },
                        if y + 1 < ny { c + nx } else { usize::MAX },
                    ];
                    for cc in nb {
                        if cc != usize::MAX && reach[cc] && !seen_island[cc] {
                            seen_island[cc] = true;
                            q2.push_back(cc);
                        }
                    }
                }
                if cells.len() > 20_000 {
                    continue;
                }
                cells.sort();
                let mut i = 0;
                while i < cells.len() {
                    let mut j = i;
                    while j + 1 < cells.len() && cells[j + 1] == cells[j] + 1 && cells[j + 1] / nx == cells[i] / nx {
                        j += 1;
                    }
                    wires.push(stroke(center(cells[i]), center(cells[j]), width));
                    i = j + 1;
                }
            }
        }
        wires
    }

    /// Remove generated plane copper (plane layers only). `unstamped` is a net whose copper is
    /// currently out of the obstacle fields.
    fn strip_plane_fill(&mut self, unstamped: usize) {
        let excl0 = self.excl.clone();
        self.fill_export.retain(|(_, w)| !excl0[w.layer]);
        self.fill_polys.retain(|(_, l, _)| !excl0[*l]);
        for m in 0..self.nets.len() {
            let excl = self.excl.clone();
            if !self.nets[m].r.wires.iter().any(|w| w.fill && excl[w.layer]) {
                continue;
            }
            if m != unstamped {
                self.stamp_net(m, -1);
            }
            self.nets[m].r.wires.retain(|w| !(w.fill && excl[w.layer]));
            self.rebuild_route_prims(m);
            if m != unstamped {
                self.stamp_net(m, 1);
            }
        }
    }

    /// Cells of the exported plane polygons that lie closer than the clearance to foreign copper
    /// (should be 0). Independent of the fill search: polygons are rasterised again.
    pub fn check_fill_polygons(&self) -> usize {
        let g = &self.grid;
        let g1 = Grid { nl: 1, ..g.clone() };
        let mut bad = 0;
        let mut fields: std::collections::HashMap<(usize, usize), Field> = Default::default();
        for (n, l, poly) in &self.fill_polys {
            let f = fields.entry((*n, *l)).or_insert_with(|| {
                let mut f = Field::new(&g1, self.nets[*n].clr, 0.0, false);
                for pr in self
                    .fixed
                    .iter()
                    .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                    .chain(self.nets.iter().flat_map(|ns| ns.pre.dynp.iter().chain(ns.r.dynp.iter())))
                {
                    if pr.layer == *l && pr.net != *n as i32 && !pr.fill {
                        let mut q = pr.clone();
                        q.layer = 0;
                        f.stamp(&g1, &q, 1, false);
                    }
                }
                f
            });
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &(x, y) in poly {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
            }
            let cx0 = (((x0 - g.x0) / g.g).floor() as i32).max(0);
            let cx1 = (((x1 - g.x0) / g.g).ceil() as i32).min(g.nx as i32 - 1);
            let cy0 = (((y0 - g.y0) / g.g).floor() as i32).max(0);
            let cy1 = (((y1 - g.y0) / g.g).ceil() as i32).min(g.ny as i32 - 1);
            for y in cy0..=cy1 {
                for x in cx0..=cx1 {
                    if f.fx(y as usize * g.nx + x as usize) && point_in_poly(g.center(x, y), poly) {
                        bad += 1;
                        if self.cfg.verbose && bad <= 6 {
                            let c = g.center(x, y);
                            let culprit = self
                                .fixed
                                .iter()
                                .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                                .chain(self.nets.iter().flat_map(|ns| ns.pre.dynp.iter().chain(ns.r.dynp.iter())))
                                .filter(|pr| pr.layer == *l && pr.net != *n as i32 && !pr.fill)
                                .min_by(|a, b| a.dist(c).total_cmp(&b.dist(c)));
                            eprintln!(
                                "copper area of {} on layer {} too close at {:?}: nearest {} {:?} dist {:.3} need {:.3}",
                                self.nets[*n].name,
                                self.layer_names[*l],
                                c,
                                culprit.map_or(String::new(), |p| if p.net >= 0 { self.nets[p.net as usize].name.clone() } else { "<fixed>".into() }),
                                culprit.map(|p| match &p.geo { Geo::Circle { .. } => "circle", Geo::Seg { .. } => "seg", Geo::Poly(_) => "poly" }),
                                culprit.map_or(0.0, |p| p.dist(c)),
                                self.nets[*n].clr
                            );
                        }
                    }
                }
            }
        }
        bad
    }

    /// Clearance violations of the compact plane export (should be 0).
    pub fn check_fill_export(&self) -> usize {
        let mut bad = 0;
        for (n, w) in &self.fill_export {
            let prims = self.wire_prims(*n, w);
            if let Some(c) = self.prims_conflict(&prims) {
                if self.cfg.verbose {
                    eprintln!("fill export conflict: {} width {:.2} pts {:?}", c, w.width, w.pts);
                }
                bad += 1;
            }
        }
        bad
    }

    fn net_has_copper(&self, n: usize, l: usize) -> bool {
        self.nets[n]
            .r
            .dynp
            .iter()
            .chain(self.nets[n].pre.dynp.iter())
            .any(|p| p.layer == l && !p.trace_only)
            || self
                .pins
                .iter()
                .any(|p| p.net == n as i32 && p.prims.iter().any(|p| p.layer == l))
    }

    /// A clear through via gives the net copper on every layer so a plane/pour can attach.
    fn add_fill_via(&mut self, n: usize) -> bool {
        if self.cfg.single_sided {
            return false;
        }
        let g = self.grid.g;
        let mut cands: Vec<P> = vec![];
        for w in self.nets[n].r.wires.iter().filter(|w| !w.fill) {
            for s in w.pts.windows(2) {
                let steps = (dist(s[0], s[1]) / g).ceil().max(1.0) as usize;
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    cands.push((
                        s[0].0 + t * (s[1].0 - s[0].0),
                        s[0].1 + t * (s[1].1 - s[0].1),
                    ));
                }
            }
        }
        for &pi in &self.nets[n].pins {
            let c = self.pins[pi].c;
            for ring in 1..=10 {
                let rad = ring as f64 * self.nets[n].via_r * 1.5;
                for k in 0..16 {
                    let a = k as f64 * std::f64::consts::TAU / 16.0;
                    cands.push((c.0 + rad * a.cos(), c.1 + rad * a.sin()));
                }
            }
        }
        for v in cands.into_iter().take(6000) {
            let prims: Vec<_> = (0..self.grid.nl)
                .map(|l| {
                    Prim::new(
                        l,
                        Geo::Circle {
                            c: v,
                            r: self.nets[n].via_r,
                        },
                        n as i32,
                        self.nets[n].clr,
                    )
                })
                .collect();
            if point_in_poly(v, &self.boundary) && self.prims_conflict(&prims).is_none() {
                self.stamp_net(n, -1);
                self.nets[n].r.vias.push(v);
                self.rebuild_route_prims(n);
                self.stamp_net(n, 1);
                return true;
            }
        }
        false
    }

    /// Generated planes (layer plan) and pours. Existing DSN planes/areas are respected: nothing is
    /// poured over them and a layer that already has a plane of the same net is left alone.
    pub(crate) fn fill_jobs(&self) -> Vec<(usize, usize, Option<usize>)> {
        if self.cfg.trace_routing && !self.cfg.pour_polygons { return vec![]; }
        let nl = self.grid.nl;
        let gnd = self
            .nets
            .iter()
            .position(|n| n.name.eq_ignore_ascii_case("GND") && (!n.skip || n.power));
        let mut jobs: Vec<(usize, usize, Option<usize>)> = vec![]; // (layer, net, plane)
        for l in 0..nl {
            for (pi, pl) in self.planes.iter().enumerate() {
                if pl.layer == l && pl.synthetic && (!self.nets[pl.net].skip || self.nets[pl.net].power) {
                    jobs.push((l, pl.net, Some(pi)));
                }
            }
            let plan = self.cfg.layers.get(&self.layer_names[l]);
            if plan.is_some_and(|p| p.role == LayerRole::Plane) {
                continue;
            }
            // pours on trace layers come from `make_pour_polygons` in trace-routing mode
            if self.cfg.trace_routing {
                continue;
            }
            let mut pours: Vec<usize> = plan
                .map(|p| {
                    p.nets
                        .iter()
                        .filter_map(|nm| self.nets.iter().position(|n| &n.name == nm))
                        .collect()
                })
                .unwrap_or_default();
            if pours.is_empty() && self.cfg.add_gnd_planes && !self.excl[l] {
                pours.extend(gnd);
            }
            for n in pours {
                if (self.nets[n].skip && !self.nets[n].power)
                    || self
                        .planes
                        .iter()
                        .any(|p| p.layer == l && p.net == n && !p.synthetic)
                {
                    continue;
                }
                jobs.push((l, n, None));
            }
        }
        jobs
    }

    /// `planes`: only generated plane layers (before bridging), otherwise only pours.
    fn fill_copper(&mut self, planes: bool) {
        let jobs: Vec<_> = self
            .fill_jobs()
            .into_iter()
            .filter(|j| j.2.is_some() == planes)
            .collect();
        let g = self.grid.clone();
        let g1 = Grid { nl: 1, ..g.clone() };
        let width = g.g * 1.1;
        for (l, n, region) in jobs {
            if self.cancelled() {
                return;
            }
            if !self.net_has_copper(n, l) {
                self.add_fill_via(n);
                if !self.net_has_copper(n, l) {
                    continue;
                }
            }
            // one extra cell of margin: lets the exported polygons be simplified without ever
            // getting closer to foreign copper than the clearance
            let mut f = Field::new(&g1, self.nets[n].clr, width / 2.0 + g.g * 1.5, false);
            let to0 = |p: &Prim| {
                let mut q = p.clone();
                q.layer = 0;
                q
            };
            for pr in self
                .fixed
                .iter()
                .chain(self.pins.iter().flat_map(|p| p.prims.iter()))
                .chain(
                    self.nets
                        .iter()
                        .flat_map(|ns| ns.pre.dynp.iter().chain(ns.r.dynp.iter())),
                )
            {
                if pr.layer == l && pr.net != n as i32 {
                    f.stamp(&g1, &to0(pr), 1, false);
                }
            }
            for pl in self.planes.iter().filter(|p| p.layer == l && !p.synthetic) {
                if pl.net != n {
                    let pr = Prim::new(
                        0,
                        Geo::Poly(pl.poly.clone()),
                        pl.net as i32,
                        self.nets[pl.net].clr,
                    );
                    f.stamp(&g1, &pr, 1, false);
                }
            }
            let pid = &self.plane_id[l];
            let allowed = |i: usize| -> bool {
                match region {
                    Some(pi) => pid[i] as usize == pi + 1,
                    None => pid.is_empty() || pid[i] == 0,
                }
            };
            let mut reachable = vec![false; g.n()];
            let mut queue = VecDeque::new();
            let mut seed_cells: Vec<usize> = vec![];
            for pr in self
                .pins
                .iter()
                .filter(|p| p.net == n as i32)
                .flat_map(|p| p.prims.iter())
                .chain(self.nets[n].pre.dynp.iter())
                .chain(self.nets[n].r.dynp.iter())
                .filter(|p| p.layer == l && !p.trace_only)
            {
                for (_, x, y) in crate::prewire::prim_cells(&g, pr) {
                    let i = y as usize * g.nx + x as usize;
                    if !f.fx(i) && allowed(i) && !reachable[i] {
                        reachable[i] = true;
                        seed_cells.push(i);
                        queue.push_back(i);
                    }
                }
            }
            while let Some(i) = queue.pop_front() {
                let (x, y) = (i % g.nx, i / g.nx);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (xx, yy) = (x as i32 + dx, y as i32 + dy);
                    if xx < 0 || yy < 0 || xx >= g.nx as i32 || yy >= g.ny as i32 {
                        continue;
                    }
                    let j = yy as usize * g.nx + xx as usize;
                    if !reachable[j]
                        && !f.fx(j)
                        && allowed(j)
                        && point_in_poly(g.center(xx, yy), &self.boundary)
                    {
                        reachable[j] = true;
                        queue.push_back(j);
                    }
                }
            }
            let compact = if self.cfg.write_fill {
                self.compact_fill(&reachable, &seed_cells, width, l)
            } else {
                vec![]
            };
            // Rows of copper are merged into rectangles; a rectangle is a thin stroke along its
            // first and last row (keeps contact with the rows above/below) plus one fat stroke.
            let mut wires = vec![];
            let stroke = |a: P, b: P, w: f64| Wire {
                fill: true,
                layer: l,
                width: w,
                pts: vec![a, if dist(a, b) < 1e-9 { (b.0 + g.g * 0.001, b.1) } else { b }],
            };
            let emit = |xs: usize, xe: usize, y0: usize, y1: usize, wires: &mut Vec<Wire>| {
                let (a, b) = (g.center(xs as i32, y0 as i32), g.center(xe as i32, y0 as i32));
                wires.push(stroke(a, b, width));
                if y1 > y0 {
                    let (c, d) = (g.center(xs as i32, y1 as i32), g.center(xe as i32, y1 as i32));
                    wires.push(stroke(c, d, width));
                    if y1 > y0 + 1 {
                        let (h, wd) = (c.1 - a.1, b.0 - a.0);
                        if wd >= h {
                            let ym = (a.1 + c.1) / 2.0;
                            wires.push(stroke(
                                (a.0 + h / 2.0, ym),
                                (b.0 - h / 2.0, ym),
                                h + width,
                            ));
                        } else {
                            let xm = (a.0 + b.0) / 2.0;
                            wires.push(stroke(
                                (xm, a.1 + wd / 2.0),
                                (xm, c.1 - wd / 2.0),
                                wd + width,
                            ));
                        }
                    }
                }
            };
            let mut active: Vec<(usize, usize, usize)> = vec![]; // (xs, xe, first row)
            for y in 0..=g.ny {
                let mut runs: Vec<(usize, usize)> = vec![];
                if y < g.ny {
                    let mut x = 0;
                    while x < g.nx {
                        if !reachable[y * g.nx + x] {
                            x += 1;
                            continue;
                        }
                        let start = x;
                        while x + 1 < g.nx && reachable[y * g.nx + x + 1] {
                            x += 1;
                        }
                        runs.push((start, x));
                        x += 1;
                    }
                }
                let mut next: Vec<(usize, usize, usize)> = vec![];
                for &(xs, xe, y0) in &active {
                    if runs.contains(&(xs, xe)) {
                        next.push((xs, xe, y0));
                    } else {
                        emit(xs, xe, y0, y - 1, &mut wires);
                    }
                }
                for &(xs, xe) in &runs {
                    if !active.iter().any(|a| a.0 == xs && a.1 == xe) {
                        next.push((xs, xe, y));
                    }
                }
                active = next;
            }
            self.stamp_net(n, -1);
            self.fill_export.extend(compact.into_iter().map(|w| (n, w)));
            if self.cfg.write_polygons {
                let (polys, dropped) = crate::outline::mask_polygons(&g, &reachable);
                if dropped > 0 && self.cfg.verbose {
                    eprintln!("fill polygons dropped (could not cut holes): {}", dropped);
                }
                let min_area = self.min_pour_area();
                self.fill_polys.extend(
                    polys
                        .into_iter()
                        .filter(|p| !self.cfg.pour_polygons || crate::outline::ring_area(p).abs() >= min_area)
                        .map(|p| (n, l, p)),
                );
            }
            if reachable.iter().any(|&r| r) && !self.fill_done.contains(&(l, n, region)) {
                self.fill_done.push((l, n, region));
            }
            self.nets[n].r.wires.extend(wires);
            self.rebuild_route_prims(n);
            self.stamp_net(n, 1);
        }
    }
}

#[cfg(test)]
mod clearance_tests {
    use super::*;
    #[test]
    fn finished_copper_necks_down_only_where_clearance_requires_it() {
        let b = dsn::parse_dsn(r#"(pcb test (resolution mil 1000)
          (structure (boundary (path pcb 0 0 0 40 0 40 30 0 30))
            (layer top (type signal)) (layer bottom (type signal))
            (via via0) (rule (width 1) (clearance 1)))
          (placement (component img (place U 0 0 front 0)))
          (library (image img (pin pad 1 10 10) (pin pad 2 25 10) (pin pad 3 15 13))
            (padstack pad (shape (circle top 2)))
            (padstack via0 (shape (circle top 1)) (shape (circle bottom 1))))
          (network (net A (pins U-1 U-2)) (net B (pins U-3))))"#).unwrap();
        let mut cfg = Config { verbose: false, unit_mm: Some(1.0), ..Config::default() };
        cfg.rules.min_trace_mm = 1.0;
        cfg.use_trace_routing();
        let mut r = Router::new(&b, cfg);
        let n = r.nets.iter().position(|n| n.name == "A").unwrap();
        r.stamp_net(n, -1);
        r.nets[n].r.wires.push(Wire { layer: 0, width: 4.0, pts: vec![(10.0,10.0),(20.0,10.0),(25.0,10.0)], fill: false });
        r.rebuild_route_prims(n);
        r.stamp_net(n, 1);
        assert!(!crate::drc::check(&r).clearance_violations.is_empty());
        r.repair_trace_clearance();
        let report = crate::drc::check(&r);
        assert!(report.clearance_violations.is_empty(), "{:?}", report.clearance_violations);
        assert!(report.open_nets.is_empty());
        assert!(r.nets[n].r.wires.iter().all(|w| w.width >= r.min_width));
        assert!(r.nets[n].r.wires.iter().any(|w| w.width == 4.0));
        assert!(r.nets[n].r.wires.iter().any(|w| w.width < 4.0));
    }
}
