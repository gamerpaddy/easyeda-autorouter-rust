//! The placer: constructive placement (greedy, connectivity ordered) -> bypass capacitor pass ->
//! simulated annealing on weighted wire length with hard legality (inside the outline, no
//! overlaps, room around big chips, locked parts never touched).
use crate::geom::*;
use crate::model::*;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Sides {
    Front,
    Back,
    Both,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Size {
    NoRoom,
    Tight,
    Normal,
    Relaxed,
}

impl Size {
    pub fn density(self) -> f64 {
        match self {
            Size::NoRoom => 0.85,
            Size::Tight => 0.65,
            Size::Normal => 0.45,
            Size::Relaxed => 0.30,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Settings {
    pub sides: Sides,
    /// board size preset (only used when the file has no outline)
    pub size: Size,
    /// fixed board size W x H in mm (only used when the file has no outline)
    pub max_size: Option<(f64, f64)>,
    /// gap between parts, mm
    pub clearance: f64,
    /// distance from the board edge, mm
    pub edge_margin: f64,
    /// extra room around chips with many pins, mm (0 = off)
    pub room: f64,
    /// pin count from which a part counts as a big chip
    pub big_pins: usize,
    pub bypass: bool,
    /// 1..10
    pub effort: u32,
    pub seed: u64,
    pub allow_rotate: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings { sides: Sides::Front, size: Size::Normal, max_size: None, clearance: 0.25, edge_margin: 0.3, room: 1.0, big_pins: 16, bypass: true, effort: 3, seed: 1, allow_rotate: true }
    }
}

#[derive(Clone, Debug)]
pub struct Outcome {
    pub pl: Vec<Pl>,
    pub region: Vec<P>,
    /// bounding box of everything placed
    pub used: [f64; 4],
    pub unplaced: Vec<usize>,
    pub hpwl_before: f64,
    pub hpwl_after: f64,
    /// bypass capacitors: (cap, host, distance of its power pad to the host pin in mm)
    pub caps: Vec<(usize, usize, f64)>,
    pub notes: Vec<String>,
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

const GRID: f64 = 0.05;
fn snap(v: f64) -> f64 {
    (v / GRID).round() * GRID
}

pub fn net_weight(n: &Net) -> f64 {
    let deg = n.pins.len();
    if deg < 2 {
        return 0.0;
    }
    match n.kind {
        NetKind::Gnd => 0.0,
        NetKind::Power => 0.35,
        NetKind::Signal => {
            if deg > 40 {
                0.2
            } else {
                1.0
            }
        }
    }
}

fn is_cap_name(p: &Part) -> bool {
    let r = p.refdes.to_uppercase();
    let mut ch = r.chars();
    ch.next() == Some('C') && ch.next().map_or(false, |c| c.is_ascii_digit())
}

struct Sat {
    cap: usize,
    host: usize,
    cpwr: usize,
    cgnd: usize,
    hpin: usize,
    hgnd: Vec<usize>,
    /// pads of the host on the supply net
    pins: Vec<usize>,
}

pub struct Eng<'a> {
    b: &'a Board,
    s: &'a Settings,
    n: usize,
    region: Region,
    esc: Vec<f64>,
    big: Vec<bool>,
    fixed: Vec<bool>,
    nets_of: Vec<Vec<usize>>,
    net_w: Vec<f64>,
    sats: Vec<Sat>,
    sat_of: Vec<Vec<usize>>,
    hosted: Vec<Vec<usize>>,
    by_host: Vec<Vec<usize>>,
    pl: Vec<Pl>,
    placed: Vec<bool>,
    rects: Vec<[f64; 4]>,
    masks: Vec<u8>,
    rng: Rng,
    nstamp: Vec<u32>,
    sstamp: Vec<u32>,
    stamp: u32,
}

impl<'a> Eng<'a> {
    fn new(b: &'a Board, s: &'a Settings, region: Region) -> Eng<'a> {
        let n = b.parts.len();
        let mut nets_of = vec![vec![]; n];
        for (ni, net) in b.nets.iter().enumerate() {
            for &(p, _) in &net.pins {
                if !nets_of[p].contains(&ni) {
                    nets_of[p].push(ni);
                }
            }
        }
        let big: Vec<bool> = b.parts.iter().map(|p| s.room > 0.0 && p.pads.len() >= s.big_pins).collect();
        let esc: Vec<f64> = b.parts.iter().map(|p| if s.room > 0.0 && p.pads.len() >= s.big_pins { s.room * (0.5 + (p.pads.len().min(200) as f64) / 64.0) } else { 0.0 }).collect();
        let fixed: Vec<bool> = b.parts.iter().map(|p| p.fixed()).collect();
        let mut e = Eng {
            b,
            s,
            n,
            region,
            esc,
            big,
            fixed,
            nets_of,
            net_w: b.nets.iter().map(net_weight).collect(),
            sats: vec![],
            sat_of: vec![vec![]; n],
            hosted: vec![vec![]; n],
            by_host: vec![vec![]; n],
            pl: vec![],
            placed: vec![false; n],
            rects: vec![[0.0; 4]; n],
            masks: vec![0; n],
            rng: Rng(s.seed.wrapping_mul(0x9E3779B97F4A7C15) | 1),
            nstamp: vec![0; b.nets.len()],
            sstamp: vec![0; 0],
            stamp: 0,
        };
        e.pl = (0..n)
            .map(|i| {
                let mut p = Pl::of(&b.parts[i]);
                p.flip = e.forced_flip(i);
                p
            })
            .collect();
        for i in 0..n {
            if e.fixed[i] {
                e.placed[i] = true;
                e.pl[i] = Pl::of(&b.parts[i]);
            }
            e.refresh(i);
        }
        e
    }

    fn forced_flip(&self, i: usize) -> bool {
        let p = &self.b.parts[i];
        if self.fixed[i] || !p.flip_ok || p.tht {
            return false;
        }
        match self.s.sides {
            Sides::Front => p.bottom,
            Sides::Back => !p.bottom,
            Sides::Both => false,
        }
    }

    fn can_flip(&self, i: usize) -> bool {
        self.s.sides == Sides::Both && self.b.parts[i].flip_ok && !self.fixed[i] && !self.b.parts[i].tht
    }

    fn can_rot(&self, i: usize) -> bool {
        self.s.allow_rotate && self.b.parts[i].rot_ok && !self.fixed[i]
    }

    fn refresh(&mut self, i: usize) {
        let p = &self.b.parts[i];
        self.rects[i] = p.rect(&self.pl[i]);
        self.masks[i] = mask(p, &self.pl[i]);
    }

    fn set_pl(&mut self, i: usize, pl: Pl) {
        self.pl[i] = pl;
        self.refresh(i);
    }

    fn need(&self, i: usize, j: usize) -> f64 {
        let mut g = self.s.clearance;
        if self.big[i] && self.big[j] {
            g += self.esc[i].max(self.esc[j]);
        }
        g
    }

    fn fits_at(&self, i: usize, pl: &Pl) -> bool {
        let p = &self.b.parts[i];
        let r = p.rect(pl);
        if !self.region.contains_box(r, self.s.edge_margin) {
            return false;
        }
        let m = mask(p, pl);
        for j in 0..self.n {
            if j == i || !self.placed[j] || self.masks[j] & m == 0 {
                continue;
            }
            let nd = self.need(i, j);
            let o = &self.rects[j];
            if r[0] < o[2] + nd && o[0] < r[2] + nd && r[1] < o[3] + nd && o[1] < r[3] + nd {
                return false;
            }
        }
        true
    }

    fn net_cost(&self, ni: usize) -> f64 {
        let w = self.net_w[ni];
        if w == 0.0 {
            return 0.0;
        }
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        let mut cnt = 0;
        for &(p, k) in &self.b.nets[ni].pins {
            if !self.placed[p] {
                continue;
            }
            let q = self.b.parts[p].pad_xy(k, &self.pl[p]);
            x0 = x0.min(q.0);
            x1 = x1.max(q.0);
            y0 = y0.min(q.1);
            y1 = y1.max(q.1);
            cnt += 1;
        }
        if cnt < 2 {
            0.0
        } else {
            w * ((x1 - x0) + (y1 - y0))
        }
    }

    /// the pin of the chip on the capacitor's supply net that is nearest to `a`
    fn link_pin(&self, s: &Sat, a: P) -> P {
        let hp = &self.b.parts[s.host];
        let mut best = (f64::MAX, hp.pad_xy(s.hpin, &self.pl[s.host]));
        for &k in &s.pins {
            let q = hp.pad_xy(k, &self.pl[s.host]);
            let d = (q.0 - a.0).powi(2) + (q.1 - a.1).powi(2);
            if d < best.0 {
                best = (d, q);
            }
        }
        best.1
    }

    fn sat_cost(&self, si: usize) -> f64 {
        let s = &self.sats[si];
        if !self.placed[s.cap] || !self.placed[s.host] {
            return 0.0;
        }
        let cp = &self.b.parts[s.cap];
        let a = cp.pad_xy(s.cpwr, &self.pl[s.cap]);
        let h = self.link_pin(s, a);
        let mut c = 5.0 * ((a.0 - h.0).powi(2) + (a.1 - h.1).powi(2)).sqrt();
        // ground goes to a plane: the ground pad is not placed for
        if self.masks[s.cap] & self.masks[s.host] == 0 {
            c += 10.0;
        }
        // links of the same chip must not cross
        for &o in &self.by_host[s.host] {
            if o == si || !self.placed[self.sats[o].cap] {
                continue;
            }
            let so = &self.sats[o];
            let a2 = self.b.parts[so.cap].pad_xy(so.cpwr, &self.pl[so.cap]);
            let h2 = self.link_pin(so, a2);
            if seg_cross(a, h, a2, h2) {
                c += 14.0;
            }
        }
        c
    }

    fn cost_set(&mut self, set: &[usize]) -> f64 {
        self.stamp += 1;
        let sn = self.stamp;
        let mut c = 0.0;
        for &i in set {
            for k in 0..self.nets_of[i].len() {
                let n = self.nets_of[i][k];
                if self.nstamp[n] != sn {
                    self.nstamp[n] = sn;
                    c += self.net_cost(n);
                }
            }
            for k in 0..self.sat_of[i].len() {
                let h = self.sats[self.sat_of[i][k]].host;
                for m in 0..self.by_host[h].len() {
                    let s = self.by_host[h][m];
                    if self.sstamp[s] != sn {
                        self.sstamp[s] = sn;
                        c += self.sat_cost(s);
                    }
                }
            }
            if self.s.sides == Sides::Both && self.masks[i] == 2 {
                c += 0.4;
            }
        }
        c
    }

    fn total_cost(&self) -> f64 {
        (0..self.b.nets.len()).map(|n| self.net_cost(n)).sum::<f64>() + (0..self.sats.len()).map(|s| self.sat_cost(s)).sum::<f64>()
    }

    // ------------------------------------------------------------ constructive

    fn orients(&self, i: usize) -> Vec<(u8, bool)> {
        let ds: Vec<u8> = if self.can_rot(i) { vec![0, 1, 2, 3] } else { vec![0] };
        let fl: Vec<bool> = if self.can_flip(i) { vec![false, true] } else { vec![self.forced_flip(i)] };
        let mut v = vec![];
        for &d in &ds {
            for &f in &fl {
                v.push((d, f));
            }
        }
        v
    }

    fn target(&self, i: usize) -> P {
        let (mut sx, mut sy, mut sw) = (0.0, 0.0, 0.0);
        for &n in &self.nets_of[i] {
            let net = &self.b.nets[n];
            let w = self.net_w[n] / (net.pins.len().max(2) - 1) as f64;
            if w == 0.0 {
                continue;
            }
            for &(p, k) in &net.pins {
                if p != i && self.placed[p] {
                    let q = self.b.parts[p].pad_xy(k, &self.pl[p]);
                    sx += q.0 * w;
                    sy += q.1 * w;
                    sw += w;
                }
            }
        }
        let bb = self.region.bbox;
        if sw > 0.0 {
            ((sx / sw).clamp(bb[0], bb[2]), (sy / sw).clamp(bb[1], bb[3]))
        } else {
            ((bb[0] + bb[2]) / 2.0, (bb[1] + bb[3]) / 2.0)
        }
    }

    fn place_one(&mut self, i: usize) -> bool {
        let t = self.target(i);
        let (tx, ty) = (snap(t.0), snap(t.1));
        let step = 0.5;
        let bb = self.region.bbox;
        let rmax = (((bb[2] - bb[0]).max(bb[3] - bb[1]) / step) as i32 + 2).max(4);
        let ors = self.orients(i);
        let mut best: Option<(f64, Pl)> = None;
        let mut found_at: Option<i32> = None;
        for m in 0..=rmax {
            if let Some(f) = found_at {
                if m > f + 3 {
                    break;
                }
            }
            let mut ring: Vec<(i32, i32)> = vec![];
            if m == 0 {
                ring.push((0, 0));
            } else {
                for k in -m..=m {
                    ring.push((k, -m));
                    ring.push((k, m));
                    if k > -m && k < m {
                        ring.push((-m, k));
                        ring.push((m, k));
                    }
                }
            }
            for (kx, ky) in ring {
                let (x, y) = (tx + kx as f64 * step, ty + ky as f64 * step);
                for &(d, f) in &ors {
                    let c = Pl { x, y, d, flip: f };
                    if !self.fits_at(i, &c) {
                        continue;
                    }
                    let old = (self.pl[i], self.placed[i]);
                    self.pl[i] = c;
                    self.placed[i] = true;
                    let rr = self.b.parts[i].rect(&c);
                    let old_r = self.rects[i];
                    let old_m = self.masks[i];
                    self.rects[i] = rr;
                    self.masks[i] = mask(&self.b.parts[i], &c);
                    let cost = self.cost_set(&[i]) + 0.01 * ((x - tx).abs() + (y - ty).abs());
                    self.pl[i] = old.0;
                    self.placed[i] = old.1;
                    self.rects[i] = old_r;
                    self.masks[i] = old_m;
                    if best.map_or(true, |b| cost < b.0) {
                        best = Some((cost, c));
                    }
                    if found_at.is_none() {
                        found_at = Some(m);
                    }
                }
            }
        }
        match best {
            Some((_, c)) => {
                self.set_pl(i, c);
                self.placed[i] = true;
                true
            }
            None => false,
        }
    }

    fn constructive(&mut self, cancel: &mut dyn FnMut(&[Pl], &str, f32) -> bool) -> Vec<usize> {
        let mut conn = vec![0.0f64; self.n];
        let mut todo: Vec<usize> = (0..self.n).filter(|&i| !self.placed[i]).collect();
        let mut unplaced = vec![];
        // connectivity from the parts that are fixed already
        for i in 0..self.n {
            if self.placed[i] {
                self.bump(&mut conn, i);
            }
        }
        for i in 0..self.n {
            if self.fixed[i] && !self.hosted[i].is_empty() {
                self.place_hosted(i);
            }
        }
        for i in 0..self.n {
            if self.placed[i] {
                self.bump(&mut conn, i);
            }
        }
        todo.retain(|&i| !self.placed[i]);
        let total = todo.len().max(1);
        let mut done = 0;
        while !todo.is_empty() {
            // most connected part first, big chips before small ones on ties
            let mut bi = 0;
            let mut bs = f64::MIN;
            for (k, &i) in todo.iter().enumerate() {
                let sc = conn[i] * 100.0 + self.b.parts[i].pads.len() as f64 * 0.01 + {
                    let r = self.b.parts[i].body;
                    ((r[2] - r[0]) * (r[3] - r[1])) * 0.001
                };
                if sc > bs {
                    bs = sc;
                    bi = k;
                }
            }
            let i = todo.swap_remove(bi);
            if self.placed[i] {
                continue;
            }
            if self.place_one(i) {
                self.bump(&mut conn, i);
                self.place_hosted(i);
                let caps = self.hosted[i].clone();
                for c in caps {
                    if self.placed[c] {
                        self.bump(&mut conn, c);
                    }
                }
            } else {
                unplaced.push(i);
            }
            done += 1;
            if done % 8 == 0 && !cancel(&self.pl, "placing", 0.3 * done as f32 / total as f32) {
                break;
            }
        }
        unplaced.extend(todo);
        unplaced.retain(|&i| !self.placed[i]);
        unplaced
    }

    fn bump(&self, conn: &mut [f64], i: usize) {
        for &n in &self.nets_of[i] {
            let net = &self.b.nets[n];
            let w = self.net_w[n] / (net.pins.len().max(2) - 1) as f64;
            for &(p, _) in &net.pins {
                if p != i {
                    conn[p] += w;
                }
            }
        }
    }

    // ------------------------------------------------------------ bypass capacitors

    /// Pair every decoupling capacitor (a C between a supply net and ground or another supply)
    /// with the chip pin it decouples (independent of positions).
    fn build_sats(&mut self) {
        let b = self.b;
        let pass = |p: &Part| {
            let r = p.refdes.to_uppercase();
            is_cap_name(p) || r.starts_with('R') || r.starts_with('L') || r.starts_with('F') || r.starts_with('D') || r.starts_with("Y") || r.starts_with("TP") || r.starts_with('H')
        };
        // chips first, other parts with supply pins (connectors, modules) only when there is no chip
        let class = |h: usize| -> usize {
            let r = b.parts[h].refdes.to_uppercase();
            if r.starts_with('U') || r.starts_with('Q') || r.starts_with("IC") || r.starts_with('M') {
                0
            } else {
                1
            }
        };
        let is_ic: Vec<bool> = b.parts.iter().map(|p| p.pads.len() >= 3 && !pass(p)).collect();
        let mut uses: std::collections::HashMap<(usize, usize), usize> = std::collections::HashMap::new();
        let mut caps: Vec<usize> = (0..self.n).filter(|&i| !self.fixed[i] && b.parts[i].pads.len() == 2 && is_cap_name(&b.parts[i])).collect();
        caps.sort_by(|&a, &c| b.parts[a].refdes.cmp(&b.parts[c].refdes));
        for i in caps {
            let p = &b.parts[i];
            let kind = |k: usize| if p.pads[k].net >= 0 { b.nets[p.pads[k].net as usize].kind } else { NetKind::Signal };
            let has_ic = |k: usize| p.pads[k].net >= 0 && b.nets[p.pads[k].net as usize].pins.iter().any(|&(h, _)| h != i && is_ic[h]);
            let (cpwr, cgnd) = match (kind(0), kind(1)) {
                (NetKind::Power, NetKind::Gnd) => (0, 1),
                (NetKind::Gnd, NetKind::Power) => (1, 0),
                (NetKind::Power, NetKind::Power) => {
                    if has_ic(0) || !has_ic(1) {
                        (0, 1)
                    } else {
                        (1, 0)
                    }
                }
                _ => continue,
            };
            let net = &b.nets[p.pads[cpwr].net as usize];
            let other = p.pads[cgnd].net;
            let mut best: Option<((usize, usize, i64), usize, usize)> = None;
            for &(h, k) in &net.pins {
                if h == i || !is_ic[h] {
                    continue;
                }
                let key = (class(h), uses.get(&(h, k)).copied().unwrap_or(0), -(b.parts[h].pads.len() as i64));
                if best.map_or(true, |x| key < x.0) {
                    best = Some((key, h, k));
                }
            }
            let Some((_, h, k)) = best else { continue };
            *uses.entry((h, k)).or_insert(0) += 1;
            let hgnd: Vec<usize> = (0..b.parts[h].pads.len())
                .filter(|&q| {
                    let n = b.parts[h].pads[q].net;
                    n >= 0 && (n == other || b.nets[n as usize].kind == NetKind::Gnd)
                })
                .collect();
            let si = self.sats.len();
            let pins: Vec<usize> = (0..b.parts[h].pads.len()).filter(|&q| b.parts[h].pads[q].net == p.pads[cpwr].net).collect();
            self.sats.push(Sat { cap: i, host: h, cpwr, cgnd, hpin: k, hgnd, pins });
            self.sat_of[i].push(si);
            self.sat_of[h].push(si);
            self.hosted[h].push(i);
            self.by_host[h].push(si);
        }
        self.sstamp = vec![0; self.sats.len()];
    }

    /// Put one bypass capacitor as close to its pin as the rules allow (the host is placed).
    fn place_sat(&mut self, si: usize) -> bool {
        let (cap, host, hpin) = (self.sats[si].cap, self.sats[si].host, self.sats[si].hpin);
        let hp = self.b.parts[host].pad_xy(hpin, &self.pl[host]);
        let (hx, hy) = (snap(hp.0), snap(hp.1));
        let host_mask = self.masks[host];
        let ors = self.orients(cap);
        let st = 0.2;
        let mut best: Option<(f64, Pl)> = None;
        let mut found_at: Option<i32> = None;
        let was = self.placed[cap];
        self.placed[cap] = true;
        for m in 0..90 {
            if let Some(f) = found_at {
                if m > f + 4 {
                    break;
                }
            }
            let mut ring: Vec<(i32, i32)> = vec![];
            if m == 0 {
                ring.push((0, 0));
            } else {
                for k in -m..=m {
                    ring.push((k, -m));
                    ring.push((k, m));
                    if k > -m && k < m {
                        ring.push((-m, k));
                        ring.push((m, k));
                    }
                }
            }
            for (kx, ky) in ring {
                let (x, y) = (hx + kx as f64 * st, hy + ky as f64 * st);
                for &(d, f) in &ors {
                    let cand = Pl { x, y, d, flip: f };
                    let mk = mask(&self.b.parts[cap], &cand);
                    if mk & host_mask == 0 || !self.fits_at(cap, &cand) {
                        continue;
                    }
                    let old = self.pl[cap];
                    let (old_r, old_m) = (self.rects[cap], self.masks[cap]);
                    self.pl[cap] = cand;
                    self.rects[cap] = self.b.parts[cap].rect(&cand);
                    self.masks[cap] = mk;
                    let c = self.sat_cost(si);
                    self.pl[cap] = old;
                    self.rects[cap] = old_r;
                    self.masks[cap] = old_m;
                    if best.map_or(true, |b| c < b.0) {
                        best = Some((c, cand));
                    }
                    if found_at.is_none() {
                        found_at = Some(m);
                    }
                }
            }
        }
        match best {
            Some((_, c)) => {
                self.set_pl(cap, c);
                true
            }
            None => {
                self.placed[cap] = was;
                false
            }
        }
    }

    /// Place the capacitors of a chip that is placed already.
    fn place_hosted(&mut self, host: usize) {
        let sats: Vec<usize> = self.sat_of[host].iter().cloned().filter(|&si| self.sats[si].host == host).collect();
        for si in sats {
            if !self.placed[self.sats[si].cap] {
                self.place_sat(si);
            }
        }
    }

    // ------------------------------------------------------------ annealing

    /// Apply the new placements, keep them when legal and accepted. Returns true when kept.
    fn try_move(&mut self, set: &[(usize, Pl)], temp: f64) -> bool {
        let idx: Vec<usize> = set.iter().map(|s| s.0).collect();
        let before = self.cost_set(&idx);
        let old: Vec<Pl> = idx.iter().map(|&i| self.pl[i]).collect();
        for &(i, pl) in set {
            self.set_pl(i, pl);
        }
        let mut ok = true;
        for &(i, pl) in set {
            if !self.fits_at(i, &pl) {
                ok = false;
                break;
            }
        }
        if ok {
            let after = self.cost_set(&idx);
            let d = after - before;
            if d <= 0.0 || (temp > 0.0 && self.rng.f() < (-d / temp).exp()) {
                return true;
            }
        }
        for (k, &i) in idx.iter().enumerate() {
            self.set_pl(i, old[k]);
        }
        false
    }

    fn anneal(&mut self, cancel: &mut dyn FnMut(&[Pl], &str, f32) -> bool) {
        let mv: Vec<usize> = (0..self.n).filter(|&i| !self.fixed[i] && self.placed[i]).collect();
        if mv.len() < 2 {
            return;
        }
        let bb = self.region.bbox;
        let span = (bb[2] - bb[0]).max(bb[3] - bb[1]);
        let r0 = (span * 0.25).clamp(2.0, 60.0);
        let iters = ((mv.len() as u64) * 1500 * self.s.effort as u64).clamp(30_000, 6_000_000);
        // start temperature from the size of typical uphill moves
        let mut sum = 0.0;
        let mut cnt = 0;
        for _ in 0..400 {
            let i = mv[self.rng.range(mv.len())];
            let cur = self.pl[i];
            let c = Pl { x: snap(cur.x + (self.rng.f() - 0.5) * r0), y: snap(cur.y + (self.rng.f() - 0.5) * r0), ..cur };
            let before = self.cost_set(&[i]);
            self.set_pl(i, c);
            let ok = self.fits_at(i, &c);
            let after = self.cost_set(&[i]);
            self.set_pl(i, cur);
            if ok && after > before {
                sum += after - before;
                cnt += 1;
            }
        }
        let t0 = if cnt > 0 { 0.6 * sum / cnt as f64 } else { 1.0 }.max(0.05);
        let t1 = t0 * 0.002;
        let alpha = (t1 / t0).powf(1.0 / iters as f64);
        let mut t = t0;
        let report = (iters / 40).max(1);
        for it in 0..iters {
            t *= alpha;
            if it % report == 0 {
                let frac = 0.3 + 0.7 * it as f32 / iters as f32;
                if !cancel(&self.pl, "optimising", frac) {
                    return;
                }
            }
            let temp_ratio = t / t0;
            let rad = (r0 * temp_ratio.powf(0.6)).max(0.4);
            let i = mv[self.rng.range(mv.len())];
            let cur = self.pl[i];
            let roll = self.rng.f();
            if roll < 0.55 {
                let (dx, dy) = (snap((self.rng.f() - 0.5) * 2.0 * rad), snap((self.rng.f() - 0.5) * 2.0 * rad));
                if dx == 0.0 && dy == 0.0 {
                    continue;
                }
                let mut set = vec![(i, Pl { x: cur.x + dx, y: cur.y + dy, ..cur })];
                for &c in &self.hosted[i] {
                    let p = self.pl[c];
                    set.push((c, Pl { x: p.x + dx, y: p.y + dy, ..p }));
                }
                self.try_move(&set, t);
            } else if roll < 0.72 {
                // towards the pins it is connected to
                let tg = self.target(i);
                let j = 1.0 + rad * 0.2;
                let nx = snap(cur.x + (tg.0 - (self.rects[i][0] + self.rects[i][2]) / 2.0) * (0.3 + 0.7 * self.rng.f()) + (self.rng.f() - 0.5) * j);
                let ny = snap(cur.y + (tg.1 - (self.rects[i][1] + self.rects[i][3]) / 2.0) * (0.3 + 0.7 * self.rng.f()) + (self.rng.f() - 0.5) * j);
                let (dx, dy) = (nx - cur.x, ny - cur.y);
                let mut set = vec![(i, Pl { x: nx, y: ny, ..cur })];
                for &c in &self.hosted[i] {
                    let p = self.pl[c];
                    set.push((c, Pl { x: p.x + dx, y: p.y + dy, ..p }));
                }
                self.try_move(&set, t);
            } else if roll < 0.82 {
                if self.can_rot(i) && self.hosted[i].is_empty() {
                    let d = (cur.d + 1 + self.rng.range(3) as u8) & 3;
                    self.try_move(&[(i, Pl { d, ..cur })], t);
                }
            } else if roll < 0.86 {
                if self.can_flip(i) && self.hosted[i].is_empty() {
                    self.try_move(&[(i, Pl { flip: !cur.flip, ..cur })], t);
                }
            } else if roll >= 0.94 {
                // two capacitors of one chip trade their pins
                if self.sats.len() < 2 {
                    continue;
                }
                let a = self.rng.range(self.sats.len());
                let h = self.sats[a].host;
                if self.by_host[h].len() < 2 {
                    continue;
                }
                let b = self.by_host[h][self.rng.range(self.by_host[h].len())];
                if a == b || !self.placed[self.sats[a].cap] || !self.placed[self.sats[b].cap] {
                    continue;
                }
                let set = [self.sats[a].cap, self.sats[b].cap];
                let before = self.cost_set(&set);
                let (pa, pb) = (self.sats[a].hpin, self.sats[b].hpin);
                self.sats[a].hpin = pb;
                self.sats[b].hpin = pa;
                let after = self.cost_set(&set);
                let d = after - before;
                if !(d <= 0.0 || self.rng.f() < (-d / t).exp()) {
                    self.sats[a].hpin = pa;
                    self.sats[b].hpin = pb;
                }
            } else {
                let j = mv[self.rng.range(mv.len())];
                if j == i || !self.hosted[i].is_empty() || !self.hosted[j].is_empty() {
                    continue;
                }
                let (ri, rj) = (self.rects[i], self.rects[j]);
                let (ci, cj) = (((ri[0] + ri[2]) / 2.0, (ri[1] + ri[3]) / 2.0), ((rj[0] + rj[2]) / 2.0, (rj[1] + rj[3]) / 2.0));
                let pj = self.pl[j];
                let a = Pl { x: snap(cur.x + cj.0 - ci.0), y: snap(cur.y + cj.1 - ci.1), ..cur };
                let b = Pl { x: snap(pj.x + ci.0 - cj.0), y: snap(pj.y + ci.1 - cj.1), ..pj };
                self.try_move(&[(i, a), (j, b)], t);
            }
        }
        // final greedy pass: only improving moves
        for _ in 0..(mv.len() * 200) {
            let i = mv[self.rng.range(mv.len())];
            let cur = self.pl[i];
            let (dx, dy) = (snap((self.rng.f() - 0.5) * 1.0), snap((self.rng.f() - 0.5) * 1.0));
            let mut set = vec![(i, Pl { x: cur.x + dx, y: cur.y + dy, ..cur })];
            for &c in &self.hosted[i] {
                let p = self.pl[c];
                set.push((c, Pl { x: p.x + dx, y: p.y + dy, ..p }));
            }
            self.try_move(&set, 0.0);
        }
    }
}

/// do the segments a-b and c-d cross (shared end points do not count)?
fn seg_cross(a: P, b: P, c: P, d: P) -> bool {
    let close = |p: P, q: P| (p.0 - q.0).abs() < 1e-6 && (p.1 - q.1).abs() < 1e-6;
    if close(a, c) || close(a, d) || close(b, c) || close(b, d) {
        return false;
    }
    let o = |p: P, q: P, r: P| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
    let (d1, d2, d3, d4) = (o(a, b, c), o(a, b, d), o(c, d, a), o(c, d, b));
    d1 * d2 < 0.0 && d3 * d4 < 0.0
}

fn mask(p: &Part, pl: &Pl) -> u8 {
    if p.tht {
        3
    } else if p.bottom ^ pl.flip {
        2
    } else {
        1
    }
}

/// Wire length of the placements as they are (all parts placed).
pub fn hpwl(b: &Board, pl: &[Pl]) -> f64 {
    let mut t = 0.0;
    for net in &b.nets {
        let w = net_weight(net);
        if w == 0.0 {
            continue;
        }
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for &(p, k) in &net.pins {
            let q = b.parts[p].pad_xy(k, &pl[p]);
            x0 = x0.min(q.0);
            x1 = x1.max(q.0);
            y0 = y0.min(q.1);
            y1 = y1.max(q.1);
        }
        t += w * ((x1 - x0) + (y1 - y0));
    }
    t
}

fn make_region(b: &Board, s: &Settings, grow: f64) -> (Region, bool) {
    if let Some(o) = &b.outline {
        return (Region::from_poly(o.clone(), b.cutouts.clone()), true);
    }
    let pad = s.clearance;
    let mut area = 0.0;
    let (mut sx, mut sy, mut sn) = (0.0, 0.0, 0.0);
    let mut fixed_pts: Vec<P> = vec![];
    for p in &b.parts {
        let r = p.rect(&Pl::of(p));
        let (w, h) = (r[2] - r[0], r[3] - r[1]);
        if p.fixed() {
            fixed_pts.push((r[0], r[1]));
            fixed_pts.push((r[2], r[3]));
        } else {
            sx += (r[0] + r[2]) / 2.0;
            sy += (r[1] + r[3]) / 2.0;
            sn += 1.0;
        }
        let e = if s.room > 0.0 && p.pads.len() >= s.big_pins { s.room * (0.5 + p.pads.len().min(200) as f64 / 64.0) * 0.5 } else { 0.0 };
        area += (w + pad + e) * (h + pad + e);
    }
    let sides = if s.sides == Sides::Both { 1.7 } else { 1.0 };
    let board_area = area / sides / s.size.density();
    let (w, h) = match s.max_size {
        Some((w, h)) => (w, h),
        None => {
            let a = board_area * grow;
            let side = a.sqrt();
            (side * 1.15, side / 1.15)
        }
    };
    let (cx, cy) = if sn > 0.0 { (sx / sn, sy / sn) } else { (0.0, 0.0) };
    let mut r = [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0];
    if !fixed_pts.is_empty() {
        let fb = bbox_of(&fixed_pts);
        let m = s.edge_margin;
        if s.max_size.is_some() {
            // keep the user's size, slide it over the locked parts
            let (fw, fh) = (fb[2] - fb[0] + 2.0 * m, fb[3] - fb[1] + 2.0 * m);
            let (w2, h2) = (w.max(fw), h.max(fh));
            let (mx, my) = ((fb[0] + fb[2]) / 2.0, (fb[1] + fb[3]) / 2.0);
            r = [mx - w2 / 2.0, my - h2 / 2.0, mx + w2 / 2.0, my + h2 / 2.0];
        } else {
            r = [r[0].min(fb[0] - m), r[1].min(fb[1] - m), r[2].max(fb[2] + m), r[3].max(fb[3] + m)];
        }
    }
    (Region::from_rect(r), s.max_size.is_some())
}

/// Run the placer. `progress(placements, phase, fraction)` is called now and then; returning false
/// stops the run (the result is still legal).
pub fn place(b: &Board, s: &Settings, progress: &mut dyn FnMut(&[Pl], &str, f32) -> bool) -> Result<Outcome, String> {
    let before: Vec<Pl> = b.parts.iter().map(Pl::of).collect();
    let hpwl_before = hpwl(b, &before);
    let mut notes: Vec<String> = vec![];
    let mut grow = 1.0;
    let mut eng;
    let mut unplaced;
    let mut tries = 0;
    loop {
        let (region, fixed_region) = make_region(b, s, grow);
        eng = Eng::new(b, s, region);
        if s.bypass {
            eng.build_sats();
        }
        unplaced = eng.constructive(progress);
        tries += 1;
        if unplaced.is_empty() || fixed_region || tries >= 30 {
            break;
        }
        grow *= 1.06;
    }
    if tries > 1 {
        notes.push(format!("board area grown {}x to fit all parts", tries - 1));
    }
    if !unplaced.is_empty() {
        notes.push(format!(
            "{} part(s) did not fit and stay where they were: {}",
            unplaced.len(),
            unplaced.iter().take(12).map(|&i| b.parts[i].refdes.clone()).collect::<Vec<_>>().join(", ")
        ));
        for &i in &unplaced {
            eng.placed[i] = false;
        }
    }
    progress(&eng.pl, "optimising", 0.3);
    eng.anneal(progress);
    // result
    let mut pl = eng.pl.clone();
    for &i in &unplaced {
        pl[i] = Pl::of(&b.parts[i]);
    }
    let mut caps = vec![];
    for s in &eng.sats {
        let a = b.parts[s.cap].pad_xy(s.cpwr, &pl[s.cap]);
        let hp = &b.parts[s.host];
        let mut h = hp.pad_xy(s.hpin, &pl[s.host]);
        let mut bd = f64::MAX;
        for &k in &s.pins {
            let q = hp.pad_xy(k, &pl[s.host]);
            let d = (q.0 - a.0).powi(2) + (q.1 - a.1).powi(2);
            if d < bd {
                bd = d;
                h = q;
            }
        }
        caps.push((s.cap, s.host, ((a.0 - h.0).powi(2) + (a.1 - h.1).powi(2)).sqrt()));
    }
    let mut pts = vec![];
    for (i, p) in b.parts.iter().enumerate() {
        let r = p.rect(&pl[i]);
        pts.push((r[0], r[1]));
        pts.push((r[2], r[3]));
    }
    let used = bbox_of(&pts);
    let hpwl_after = hpwl(b, &pl);
    progress(&pl, "done", 1.0);
    Ok(Outcome { pl, region: eng.region.poly.clone(), used, unplaced, hpwl_before, hpwl_after, caps, notes })
}

/// Independent check of a result: parts inside the outline, no overlaps, locked parts unmoved.
pub fn verify(b: &Board, s: &Settings, o: &Outcome) -> Vec<String> {
    let mut errs = vec![];
    let region = if let Some(out) = &b.outline { Region::from_poly(out.clone(), b.cutouts.clone()) } else { Region::from_poly(o.region.clone(), vec![]) };
    let rects: Vec<[f64; 4]> = b.parts.iter().enumerate().map(|(i, p)| p.rect(&o.pl[i])).collect();
    for (i, p) in b.parts.iter().enumerate() {
        if o.unplaced.contains(&i) {
            continue;
        }
        if p.fixed() {
            if o.pl[i] != Pl::of(p) {
                errs.push(format!("{} is locked but moved", p.refdes));
            }
            continue;
        }
        if !region.contains_box(rects[i], s.edge_margin - 1e-6) {
            errs.push(format!("{} is outside the board", p.refdes));
        }
        for j in 0..i {
            if o.unplaced.contains(&j) || (b.parts[j].fixed() && p.fixed()) {
                continue;
            }
            if mask(p, &o.pl[i]) & mask(&b.parts[j], &o.pl[j]) == 0 {
                continue;
            }
            let (a, c) = (rects[i], rects[j]);
            let g = s.clearance - 1e-6;
            if a[0] < c[2] + g && c[0] < a[2] + g && a[1] < c[3] + g && c[1] < a[3] + g {
                errs.push(format!("{} overlaps {}", p.refdes, b.parts[j].refdes));
            }
        }
    }
    errs
}
