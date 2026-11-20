//! Pre-existing copper (wiring, vias, planes): ingestion, connectivity groups, plane landing,
//! and plane cut reporting / repair.

use crate::engine::*;
use crate::geom::*;
use crate::prewire::{components, prim_cells};
use dsn::Board;
use std::collections::{HashMap, VecDeque};

#[derive(Clone, Debug)]
pub struct PlaneInfo {
    pub name: String,
    pub net: usize,
    pub layer: usize,
    pub poly: Vec<P>,
    pub exclusive: bool,
    /// pins that touch the plane copper
    pub members: Vec<usize>,
    /// generated from the layer plan (not present in the DSN): filled copper is written out
    pub synthetic: bool,
    /// left-over copper handed to ground; never used as a landing target
    pub filler: bool,
}

#[derive(Clone, Debug)]
pub struct PlaneStatus {
    pub name: String,
    pub members: usize,
    /// pieces of the plane that contain member pins (1 = intact)
    pub islands: usize,
    /// sizeable copper pieces cut off from every pin (loops / dead islands)
    pub orphans: usize,
}

const ORPHAN_MIN_CELLS: usize = 400;

impl Router {
    /// Take over wires, vias and planes that already exist in the DSN. They are never removed.
    pub(crate) fn ingest_existing(&mut self, board: &Board) {
        let net_idx: HashMap<&str, usize> = board
            .nets
            .iter()
            .enumerate()
            .map(|(i, n)| (n.name.as_str(), i))
            .collect();
        let nl = self.grid.nl;
        let default_clr = if board.rule_clear > 0.0 {
            board.rule_clear
        } else {
            self.grid.g * 2.0
        };
        for w in &board.wires {
            let Some(l) = board.layer_index(&w.layer) else {
                continue;
            };
            let ni = net_idx.get(w.net.as_str()).copied();
            let clr = ni.map_or(default_clr, |i| self.nets[i].clr);
            let net = ni.map_or(-1, |i| i as i32);
            let prims: Vec<Prim> = w
                .pts
                .windows(2)
                .map(|s| {
                    Prim::new(
                        l,
                        Geo::Seg {
                            a: s[0],
                            b: s[1],
                            hw: w.width / 2.0,
                        },
                        net,
                        clr,
                    )
                })
                .collect();
            match ni {
                Some(i) => {
                    self.nets[i].pre.wires.push(Wire {
                        fill: false,
                        layer: l,
                        width: w.width,
                        pts: w.pts.clone(),
                    });
                    self.nets[i].pre.dynp.extend(prims);
                }
                None => self.fixed.extend(prims),
            }
        }
        for v in &board.vias {
            let ni = net_idx.get(v.net.as_str()).copied();
            let r = via_radius(board, &v.name);
            let clr = ni.map_or(default_clr, |i| self.nets[i].clr);
            let net = ni.map_or(-1, |i| i as i32);
            let prims: Vec<Prim> = (0..nl)
                .map(|l| Prim::new(l, Geo::Circle { c: (v.x, v.y), r }, net, clr))
                .collect();
            match ni {
                Some(i) => {
                    self.nets[i].pre.vias.push((v.x, v.y));
                    self.nets[i].pre.dynp.extend(prims);
                }
                None => self.fixed.extend(prims),
            }
        }
        for pl in &board.planes {
            let Some(layer) = board.layer_index(&pl.layer) else {
                continue;
            };
            let Some(&net) = net_idx.get(pl.net.as_str()) else {
                continue;
            };
            if self.planes.len() >= 250 {
                break;
            }
            let name = format!("{}@{}", pl.net, pl.layer);
            let exclusive =
                self.cfg.protect_planes || self.cfg.exclusive_planes.iter().any(|e| e == &name);
            let idx = self.planes.len();
            // raster
            let n = self.grid.n();
            if self.plane_id[layer].is_empty() {
                self.plane_id[layer] = vec![0u8; n];
            }
            let g = self.grid.clone();
            let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &(x, y) in &pl.pts {
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
            }
            let (cx0, cx1) = (
                (((x0 - g.x0) / g.g).floor() as i32).max(0),
                (((x1 - g.x0) / g.g).ceil() as i32).min(g.nx as i32 - 1),
            );
            let (cy0, cy1) = (
                (((y0 - g.y0) / g.g).floor() as i32).max(0),
                (((y1 - g.y0) / g.g).ceil() as i32).min(g.ny as i32 - 1),
            );
            for y in cy0..=cy1 {
                for x in cx0..=cx1 {
                    if point_in_poly(g.center(x, y), &pl.pts) {
                        self.plane_id[layer][y as usize * g.nx + x as usize] = (idx + 1) as u8;
                    }
                }
            }
            if exclusive {
                let mut pr = Prim::new(
                    layer,
                    Geo::Poly(pl.pts.clone()),
                    net as i32,
                    self.nets[net].clr,
                );
                pr.trace_only = true;
                self.nets[net].pre.dynp.push(pr);
            }
            self.planes.push(PlaneInfo {
                name,
                net,
                layer,
                poly: pl.pts.clone(),
                exclusive,
                members: vec![],
                synthetic: false,
                filler: false,
            });
        }
    }

    /// Turn the layer plan (layer -> plane nets) into planes.
    ///
    /// Every net gets exactly ONE connected region per layer (the patch holding most of its
    /// pins), so a plane can never start out in separate pieces; pins outside it are wired to it
    /// with ordinary traces. Copper nobody uses (far from the owner's pins, or dropped patches)
    /// becomes a ground filler region.
    pub(crate) fn build_layer_planes(&mut self, board: &Board) {
        const FILL: u8 = 254;
        const NONE: u8 = u8::MAX;
        let nn = self.grid.n();
        let g = self.grid.clone();
        let gnd = self
            .nets
            .iter()
            .position(|n| n.name.eq_ignore_ascii_case("GND") && (!n.skip || n.power) && !n.pins.is_empty());
        let mut order: Vec<(usize, Vec<usize>)> = vec![];
        for (l, layer) in board.layers.iter().enumerate() {
            let Some(plan) = self.cfg.layers.get(&layer.name) else {
                continue;
            };
            if plan.role != LayerRole::Plane {
                continue;
            }
            let mut nets: Vec<usize> = vec![];
            for name in &plan.nets {
                if let Some(i) = board.nets.iter().position(|n| &n.name == name) {
                    if !self.nets[i].pins.is_empty() && !nets.contains(&i) && nets.len() < 200 {
                        nets.push(i);
                    }
                }
            }
            if !nets.is_empty() {
                order.push((l, nets));
            }
        }
        for (l, nets) in order {
            if self.planes.len() + nets.len() + 1 > 250 {
                break;
            }
            if self.plane_id[l].is_empty() {
                self.plane_id[l] = vec![0u8; nn];
            }
            let inside: Vec<bool> = (0..nn)
                .map(|c| {
                    let (x, y) = ((c % g.nx) as i32, (c / g.nx) as i32);
                    self.plane_id[l][c] == 0 && point_in_poly(g.center(x, y), &board.boundary)
                })
                .collect();
            let nbrs = |c: usize| -> [usize; 4] {
                let (x, y) = (c % g.nx, c / g.nx);
                [
                    if x > 0 { c - 1 } else { usize::MAX },
                    if x + 1 < g.nx { c + 1 } else { usize::MAX },
                    if y > 0 { c - g.nx } else { usize::MAX },
                    if y + 1 < g.ny { c + g.nx } else { usize::MAX },
                ]
            };
            let pin_cell = |me: &Router, pi: usize| -> usize {
                let (x, y) = g.cell(me.pins[pi].c);
                y.clamp(0, g.ny as i32 - 1) as usize * g.nx + x.clamp(0, g.nx as i32 - 1) as usize
            };
            // multi-source flood from every pin of the assigned nets (nearest pin wins)
            let mut owner = vec![NONE; nn];
            let mut src = vec![0u32; nn];
            let mut q: VecDeque<usize> = VecDeque::new();
            for (k, &n) in nets.iter().enumerate() {
                for &pi in &self.nets[n].pins {
                    let c = pin_cell(self, pi);
                    if owner[c] == NONE {
                        owner[c] = k as u8;
                        src[c] = c as u32;
                        q.push_back(c);
                    }
                }
            }
            while let Some(c) = q.pop_front() {
                for cc in nbrs(c) {
                    if cc != usize::MAX && owner[cc] == NONE {
                        owner[cc] = owner[c];
                        src[cc] = src[c];
                        q.push_back(cc);
                    }
                }
            }
            let filler_ok = self.cfg.plane_refine && gnd.is_some();
            // unused copper far away from a power net's pins goes to ground
            if filler_ok {
                // within `reach` of one of the owner's own pins (a disc, not a diamond)
                let reach = self.cfg.plane_reach_mm / self.unit_mm / g.g;
                let is_ground: Vec<bool> = nets
                    .iter()
                    .map(|&n| self.nets[n].name.to_uppercase().ends_with("GND"))
                    .collect();
                for c in 0..nn {
                    let k = owner[c];
                    if k != NONE && !is_ground[k as usize] {
                        let s = src[c] as usize;
                        let (dx, dy) = (
                            (c % g.nx) as f64 - (s % g.nx) as f64,
                            (c / g.nx) as f64 - (s / g.nx) as f64,
                        );
                        if (dx * dx + dy * dy).sqrt() > reach {
                            owner[c] = FILL;
                        }
                    }
                }
            }
            // one connected patch per net: keep the one with the most pins
            let mut label = vec![0u32; nn];
            let mut next = 1u32;
            for (k, &n) in nets.iter().enumerate() {
                let mut comps: Vec<(u32, usize, usize)> = vec![]; // (label, pins, area)
                for start in 0..nn {
                    if label[start] != 0 || owner[start] != k as u8 || !inside[start] {
                        continue;
                    }
                    let id = next;
                    next += 1;
                    let mut area = 0;
                    label[start] = id;
                    q.push_back(start);
                    while let Some(c) = q.pop_front() {
                        area += 1;
                        for cc in nbrs(c) {
                            if cc != usize::MAX
                                && label[cc] == 0
                                && owner[cc] == k as u8
                                && inside[cc]
                            {
                                label[cc] = id;
                                q.push_back(cc);
                            }
                        }
                    }
                    comps.push((id, 0, area));
                }
                for &pi in &self.nets[n].pins {
                    let lb = label[pin_cell(self, pi)];
                    if let Some(c) = comps.iter_mut().find(|c| c.0 == lb) {
                        c.1 += 1;
                    }
                }
                let keep = comps
                    .iter()
                    .max_by_key(|c| (c.1, c.2))
                    .map(|c| c.0)
                    .unwrap_or(0);
                for c in 0..nn {
                    if owner[c] == k as u8 && label[c] != keep {
                        owner[c] = if filler_ok { FILL } else { NONE };
                    }
                }
            }
            let base = self.planes.len();
            for (k, &n) in nets.iter().enumerate() {
                for c in 0..nn {
                    if inside[c] && owner[c] == k as u8 {
                        self.plane_id[l][c] = (base + k + 1) as u8;
                    }
                }
                self.planes.push(PlaneInfo {
                    name: format!("{}@{}", self.nets[n].name, self.layer_names[l]),
                    net: n,
                    layer: l,
                    poly: board.boundary.clone(),
                    exclusive: true,
                    members: vec![],
                    synthetic: true,
                    filler: false,
                });
            }
            if let (true, Some(gn)) = (filler_ok, gnd) {
                let id = (self.planes.len() + 1) as u8;
                let mut any = false;
                for c in 0..nn {
                    if inside[c] && owner[c] == FILL {
                        self.plane_id[l][c] = id;
                        any = true;
                    }
                }
                if any {
                    self.planes.push(PlaneInfo {
                        name: format!("{} fill@{}", self.nets[gn].name, self.layer_names[l]),
                        net: gn,
                        layer: l,
                        poly: board.boundary.clone(),
                        exclusive: true,
                        members: vec![],
                        synthetic: true,
                        filler: true,
                    });
                }
            }
        }
    }

    /// Group each net's pins into electrically connected components (pads touching, existing
    /// wiring, vias, plane copper) and rasterise the cells of each group.
    pub(crate) fn init_components(&mut self) {
        for n in 0..self.nets.len() {
            self.init_net_components(n);
        }
    }

    /// `init_components` for one net.
    pub(crate) fn init_net_components(&mut self, n: usize) {
        let nl = self.grid.nl;
        {
            let pins = self.nets[n].pins.clone();
            let mut prims: Vec<Prim> = vec![];
            let mut owner: Vec<Option<usize>> = vec![];
            for (k, &pi) in pins.iter().enumerate() {
                for pr in &self.pins[pi].prims {
                    prims.push(pr.clone());
                    owner.push(Some(k));
                }
            }
            let n_pin_items = prims.len();
            let mut pre_idx: Vec<usize> = vec![]; // index into nets[n].pre.dynp for each pre item
            for (j, pr) in self.nets[n].pre.dynp.iter().enumerate() {
                if pr.trace_only {
                    continue;
                }
                prims.push(pr.clone());
                owner.push(None);
                pre_idx.push(j);
            }
            let mut plane_items: Vec<(usize, usize)> = vec![]; // (item index, plane idx)
            for (pi, pl) in self.planes.iter().enumerate() {
                if pl.net == n && !pl.synthetic {
                    plane_items.push((prims.len(), pi));
                    prims.push(Prim::new(
                        pl.layer,
                        Geo::Poly(pl.poly.clone()),
                        n as i32,
                        0.0,
                    ));
                    owner.push(None);
                }
            }
            // forced unions: layers of one pin; all layers of one via
            let mut extra: Vec<(usize, usize)> = vec![];
            let mut first_of_pin: HashMap<usize, usize> = HashMap::new();
            for i in 0..n_pin_items {
                let k = owner[i].unwrap();
                match first_of_pin.get(&k) {
                    Some(&f) => extra.push((f, i)),
                    None => {
                        first_of_pin.insert(k, i);
                    }
                }
            }
            let mut via_first: HashMap<(i64, i64), usize> = HashMap::new();
            for (off, &j) in pre_idx.iter().enumerate() {
                if let Geo::Circle { c, .. } = self.nets[n].pre.dynp[j].geo {
                    let key = ((c.0 * 1000.0).round() as i64, (c.1 * 1000.0).round() as i64);
                    let item = n_pin_items + off;
                    match via_first.get(&key) {
                        Some(&f) => extra.push((f, item)),
                        None => {
                            via_first.insert(key, item);
                        }
                    }
                }
            }
            let refs: Vec<&Prim> = prims.iter().collect();
            let root = components(&refs, &extra);
            // group pins
            let mut groups: Vec<(usize, Vec<usize>)> = vec![]; // (root, pins)
            for (k, &pi) in pins.iter().enumerate() {
                let r = root[first_of_pin[&k]];
                match groups.iter_mut().find(|g| g.0 == r) {
                    Some(g) => g.1.push(pi),
                    None => groups.push((r, vec![pi])),
                }
                let _ = k;
            }
            // plane component first
            let mut plane_of_net: Option<usize> = None;
            let mut plane_root: Option<usize> = None;
            for &(item, pi) in &plane_items {
                let r = root[item];
                let members: Vec<usize> = groups
                    .iter()
                    .filter(|g| g.0 == r)
                    .flat_map(|g| g.1.clone())
                    .collect();
                self.planes[pi].members = members.clone();
                if self.cfg.verbose {
                    eprintln!(
                        "plane {}: {} member pin(s) of {}",
                        self.planes[pi].name,
                        members.len(),
                        pins.len()
                    );
                }
                if plane_of_net.is_none() && !members.is_empty() {
                    plane_of_net = Some(pi);
                    plane_root = Some(r);
                }
            }
            // generated plane: its member pins form one group; with none, an empty
            // pseudo group stands for the plane itself and every pin group must reach it
            let syn = self
                .planes
                .iter()
                .position(|p| p.net == n && p.synthetic && !p.filler);
            let mut syn_first = false;
            if let Some(pi) = syn {
                let l = self.planes[pi].layer;
                let nx = self.grid.nx;
                let on_plane = |me: &Router, p: usize| {
                    me.pins[p].cells.iter().any(|&(cl, x, y)| {
                        cl == l && me.plane_id[l][y as usize * nx + x as usize] as usize == pi + 1
                    }) || me.drops.iter().any(|d| d.net == n && d.pin == p)
                };
                let mut merged: Vec<usize> = vec![];
                let mut k = 0;
                while k < groups.len() {
                    if groups[k].1.iter().any(|&p| on_plane(self, p)) {
                        merged.extend(groups.remove(k).1);
                    } else {
                        k += 1;
                    }
                }
                self.planes[pi].members = merged.clone();
                groups.insert(0, (usize::MAX, merged));
                plane_of_net = Some(pi);
                plane_root = Some(usize::MAX);
                syn_first = true;
            }
            let _ = syn_first;
            let start_root =
                plane_root.or_else(|| groups.iter().find(|g| g.1.contains(&pins[0])).map(|g| g.0));
            if let Some(sr) = start_root {
                if let Some(pos) = groups.iter().position(|g| g.0 == sr) {
                    let g = groups.remove(pos);
                    groups.insert(0, g);
                }
            }
            self.nets[n].plane = plane_of_net;
            // cells per component
            let mut comp_cells: Vec<Vec<Cell>> = vec![];
            for (r, gp) in &groups {
                let mut cells: Vec<Cell> = vec![];
                for &pi in gp {
                    cells.extend_from_slice(&self.pins[pi].cells);
                }
                for (off, &j) in pre_idx.iter().enumerate() {
                    if root[n_pin_items + off] == *r {
                        cells.extend(prim_cells(&self.grid, &self.nets[n].pre.dynp[j]));
                    }
                }
                cells.sort();
                cells.dedup();
                comp_cells.push(cells);
            }
            self.nets[n].comps = groups.into_iter().map(|g| g.1).collect();
            self.nets[n].comp_cells = comp_cells;
            self.nets[n].unconnected = self.reps(n);
            let _ = nl;
        }
    }

    /// Free plane cells (for the plane owned by net `n`) reachable from its member pins.
    /// Returns (layer, mask over layer cells).
    pub(crate) fn plane_landing(&self, n: usize) -> Option<(usize, Vec<bool>)> {
        let pi = self.nets[n].plane?;
        let pl = &self.planes[pi];
        let l = pl.layer;
        let nn = self.grid.n();
        let pid = &self.plane_id[l];
        let tf = &self.fields[self.nets[n].tf];
        let want = (pi + 1) as u8;
        let free = |c: usize| pid[c] == want && !tf.fx(l * nn + c) && !tf.dy(l * nn + c);
        let mut mask = vec![false; nn];
        let mut q: VecDeque<usize> = VecDeque::new();
        // no pad of the net touches the generated plane (its pads may be tied to it with vias): any
        // free cell of the plane
        if pl.synthetic && !self.nets[n].comp_cells[0].iter().any(|&(cl, _, _)| cl == l) {
            // only the big connected pieces: a pocket that other copper has cut off from the rest
            // of the plane is no place to land
            let (label, sizes) = self.free_pieces(l, &free);
            let biggest = sizes.iter().copied().max().unwrap_or(0);
            for c in 0..nn {
                if label[c] != 0 && sizes[label[c] as usize - 1] * 4 >= biggest {
                    mask[c] = true;
                }
            }
            return Some((l, mask));
        }
        for &(cl, x, y) in &self.nets[n].comp_cells[0] {
            if cl == l {
                let c = y as usize * self.grid.nx + x as usize;
                if free(c) && !mask[c] {
                    mask[c] = true;
                    q.push_back(c);
                }
            }
        }
        let nx = self.grid.nx;
        while let Some(c) = q.pop_front() {
            let (x, y) = (c % nx, c / nx);
            let mut visit = |cc: usize| {
                if !mask[cc] && free(cc) {
                    mask[cc] = true;
                    q.push_back(cc);
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
            if y + 1 < self.grid.ny {
                visit(c + nx);
            }
        }
        Some((l, mask))
    }

    /// Landing cells of a mask inside a window.
    pub(crate) fn landing_cells(
        &self,
        layer: usize,
        mask: &[bool],
        win: (i32, i32, i32, i32),
    ) -> Vec<Cell> {
        let mut v = vec![];
        for y in win.1..=win.3 {
            for x in win.0..=win.2 {
                if mask[y as usize * self.grid.nx + x as usize] {
                    v.push((layer, x, y));
                }
            }
        }
        v
    }

    /// How is each plane doing: still in one piece, or cut apart by routed traces/vias?
    pub fn plane_report(&self) -> Vec<PlaneStatus> {
        let mut out = vec![];
        let g1 = Grid {
            nl: 1,
            ..self.grid.clone()
        };
        let nx = self.grid.nx;
        let nn = self.grid.n();
        for (pi, pl) in self.planes.iter().enumerate() {
            if pl.synthetic {
                // generated planes sit on trace-free layers; their pieces are joined by bridging
                out.push(PlaneStatus {
                    name: pl.name.clone(),
                    members: pl.members.len(),
                    islands: 1,
                    orphans: 0,
                });
                continue;
            }
            let l = pl.layer;
            let mut f = Field::new(&g1, self.nets[pl.net].clr, 0.0, false);
            let to0 = |p: &Prim| {
                let mut q = p.clone();
                q.layer = 0;
                q
            };
            let mut stamp = |p: &Prim| {
                if p.layer == l && p.net != pl.net as i32 && !p.trace_only {
                    f.stamp(&g1, &to0(p), 1, false);
                }
            };
            for p in &self.fixed {
                stamp(p);
            }
            for pin in &self.pins {
                for p in &pin.prims {
                    stamp(p);
                }
            }
            for n in &self.nets {
                for p in n.pre.dynp.iter().chain(n.r.dynp.iter()) {
                    stamp(p);
                }
            }
            let pid = &self.plane_id[l];
            let want = (pi + 1) as u8;
            let inside_free = |c: usize| pid[c] == want && f.cnt[c] == 0;
            let mut label = vec![0u32; nn];
            let mut areas: Vec<usize> = vec![0];
            for start in 0..nn {
                if label[start] != 0 || !inside_free(start) {
                    continue;
                }
                let id = areas.len() as u32;
                areas.push(0);
                let mut q = VecDeque::new();
                label[start] = id;
                q.push_back(start);
                while let Some(c) = q.pop_front() {
                    areas[id as usize] += 1;
                    let (x, y) = (c % nx, c / nx);
                    let mut nb = [usize::MAX; 4];
                    if x > 0 {
                        nb[0] = c - 1;
                    }
                    if x + 1 < nx {
                        nb[1] = c + 1;
                    }
                    if y > 0 {
                        nb[2] = c - nx;
                    }
                    if y + 1 < self.grid.ny {
                        nb[3] = c + nx;
                    }
                    for &cc in &nb {
                        if cc != usize::MAX && label[cc] == 0 && inside_free(cc) {
                            label[cc] = id;
                            q.push_back(cc);
                        }
                    }
                }
            }
            let mut member_labels = std::collections::HashSet::new();
            for &m in &pl.members {
                for &(cl, x, y) in &self.pins[m].cells {
                    if cl == l {
                        let lb = label[y as usize * nx + x as usize];
                        if lb != 0 {
                            member_labels.insert(lb);
                        }
                    }
                }
            }
            let orphans = (1..areas.len())
                .filter(|&i| !member_labels.contains(&(i as u32)) && areas[i] >= ORPHAN_MIN_CELLS)
                .count();
            out.push(PlaneStatus {
                name: pl.name.clone(),
                members: pl.members.len(),
                islands: member_labels.len().max(1),
                orphans,
            });
        }
        out
    }

    /// Re-route the nets that cut a plane the most, with a heavier cut penalty, until the plane
    /// is whole again (or the round limit is reached).
    pub fn plane_repair(&mut self) {
        if self.planes.iter().all(|p| p.synthetic) {
            return;
        }
        for round in 0..self.cfg.plane_repair_rounds {
            if self.cancelled() {
                return;
            }
            let rep = self.plane_report();
            let bad: Vec<usize> = (0..rep.len())
                .filter(|&i| rep[i].islands > 1 || rep[i].orphans > 0)
                .collect();
            if self.cfg.verbose {
                for r in &rep {
                    eprintln!(
                        "  plane {}: {} piece(s) with pins, {} dead piece(s) (round {})",
                        r.name, r.islands, r.orphans, round
                    );
                }
            }
            if bad.is_empty() {
                return;
            }
            for pi in bad {
                let pl = self.planes[pi].clone();
                // foreign nets ranked by trace length inside the plane
                let mut cut: Vec<(f64, usize)> = vec![];
                for (n, ns) in self.nets.iter().enumerate() {
                    if n == pl.net || ns.skip {
                        continue;
                    }
                    let mut len = 0.0;
                    for w in &ns.r.wires {
                        if w.layer != pl.layer {
                            continue;
                        }
                        for s in w.pts.windows(2) {
                            let m = ((s[0].0 + s[1].0) / 2.0, (s[0].1 + s[1].1) / 2.0);
                            if point_in_poly(m, &pl.poly) {
                                len += dist(s[0], s[1]);
                            }
                        }
                    }
                    if len > 0.0 {
                        cut.push((len, n));
                    }
                }
                cut.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
                for &(_, n) in cut.iter().take(2) {
                    self.rip_net(n);
                    self.nets[n].plane_mul *= 5.0;
                    self.route_net(n, false);
                }
            }
        }
    }
}
