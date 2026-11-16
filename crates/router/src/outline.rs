//! Plane / pour outlines for the CAD: a few clean polygons per net and layer instead of
//! thousands of fill strokes. The CAD pours copper into them itself (clearances, thermals).

use crate::engine::*;
use crate::geom::*;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct PlaneOutline {
    pub layer: String,
    pub net: String,
    /// "plane" (generated plane region), "filler" (left-over ground) or "pour" (whole layer)
    pub kind: &'static str,
    /// closed polygon, DSN units
    pub pts: Vec<P>,
}

/// Boundaries of the true cells of `mask`: (outer loops counter-clockwise, hole loops clockwise).
pub(crate) fn contours(mask: &[bool], nx: usize, ny: usize) -> (Vec<Vec<(i32, i32)>>, Vec<Vec<(i32, i32)>>) {
    let at = |x: i32, y: i32| -> bool {
        x >= 0 && y >= 0 && (x as usize) < nx && (y as usize) < ny && mask[y as usize * nx + x as usize]
    };
    let mut next: HashMap<(i32, i32), Vec<(i32, i32)>> = HashMap::new();
    for y in 0..ny as i32 {
        for x in 0..nx as i32 {
            if !at(x, y) {
                continue;
            }
            if !at(x, y - 1) {
                next.entry((x, y)).or_default().push((x + 1, y));
            }
            if !at(x + 1, y) {
                next.entry((x + 1, y)).or_default().push((x + 1, y + 1));
            }
            if !at(x, y + 1) {
                next.entry((x + 1, y + 1)).or_default().push((x, y + 1));
            }
            if !at(x - 1, y) {
                next.entry((x, y + 1)).or_default().push((x, y));
            }
        }
    }
    let mut loops = vec![];
    let mut holes = vec![];
    let mut keys: Vec<(i32, i32)> = next.keys().copied().collect();
    // Stable start vertices also make simplification and hole cuts reproducible.
    keys.sort_unstable();
    for k in keys {
        while next.get(&k).is_some_and(|v| !v.is_empty()) {
            let mut pts = vec![k];
            let mut cur = k;
            loop {
                let Some(n) = next.get_mut(&cur).and_then(|v| v.pop()) else {
                    break;
                };
                if n == k {
                    break;
                }
                pts.push(n);
                cur = n;
            }
            let mut area = 0.0;
            for i in 0..pts.len() {
                let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
                area += (a.0 as f64) * (b.1 as f64) - (b.0 as f64) * (a.1 as f64);
            }
            if area > 0.0 {
                loops.push(pts); // counter-clockwise = outer boundary
            } else if area < 0.0 {
                holes.push(pts);
            }
        }
    }
    (loops, holes)
}

pub(crate) fn ring_area(pts: &[P]) -> f64 {
    (0..pts.len())
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            a.0 * b.1 - b.0 * a.1
        })
        .sum::<f64>()
        / 2.0
}

/// One simple polygon from an outer ring and the holes inside it: every hole is joined to the
/// ring by a zero-width cut, so the result can be poured as a single area.
fn merge_holes(outer: Vec<P>, holes: Vec<Vec<P>>) -> Vec<P> {
    merge_holes_checked(outer, holes).0
}

/// Like `merge_holes`; the count is the number of holes that could not be joined (those are
/// left out of the polygon, so the caller must not use it as copper).
pub(crate) fn merge_holes_checked(mut outer: Vec<P>, mut holes: Vec<Vec<P>>) -> (Vec<P>, usize) {
    let mut failed = 0;
    holes.sort_by(|a, b| {
        let mx = |h: &Vec<P>| h.iter().map(|p| p.0).fold(f64::MIN, f64::max);
        mx(b).total_cmp(&mx(a))
    });
    while !holes.is_empty() {
        let hole = holes.remove(0);
        let hi = (0..hole.len()).max_by(|&i, &j| hole[i].0.total_cmp(&hole[j].0)).unwrap();
        let h = hole[hi];
        let mut cand: Vec<usize> = (0..outer.len()).collect();
        cand.sort_by(|&i, &j| dist(outer[i], h).total_cmp(&dist(outer[j], h)));
        let blocked = |a: P, b: P, ring: &[P]| {
            (0..ring.len()).any(|i| {
                let (c, d) = (ring[i], ring[(i + 1) % ring.len()]);
                dist(c, a) > 1e-9 && dist(c, b) > 1e-9 && dist(d, a) > 1e-9 && dist(d, b) > 1e-9 && segs_cross(a, b, c, d)
            })
        };
        let pick = cand
            .iter()
            .copied()
            .take(400)
            .find(|&i| {
                !blocked(outer[i], h, &outer)
                    && !blocked(outer[i], h, &hole)
                    && !holes.iter().any(|o| blocked(outer[i], h, o))
            });
        let Some(pick) = pick else {
            failed += 1;
            continue;
        };
        let o = outer[pick];
        let mut ring: Vec<P> = outer[..=pick].to_vec();
        for k in 0..=hole.len() {
            ring.push(hole[(hi + k) % hole.len()]);
        }
        ring.push(o);
        ring.extend_from_slice(&outer[pick + 1..]);
        outer = ring;
    }
    (outer, failed)
}

/// Smallest octagon with axis-parallel and diagonal sides that contains the ring (clockwise
/// output when the input ring is clockwise). Empty when degenerate.
fn enclosing_octagon(ring: &[P]) -> Vec<P> {
    if ring.len() < 3 {
        return vec![];
    }
    let dirs: Vec<P> = (0..8)
        .map(|k| {
            let a = k as f64 * std::f64::consts::FRAC_PI_4;
            (a.cos(), a.sin())
        })
        .collect();
    let h: Vec<f64> = dirs
        .iter()
        .map(|d| ring.iter().map(|p| p.0 * d.0 + p.1 * d.1).fold(f64::MIN, f64::max))
        .collect();
    let mut out = vec![];
    for k in 0..8 {
        let (d1, d2) = (dirs[k], dirs[(k + 1) % 8]);
        let det = d1.0 * d2.1 - d1.1 * d2.0;
        if det.abs() < 1e-9 {
            return vec![];
        }
        let (h1, h2) = (h[k], h[(k + 1) % 8]);
        out.push(((h1 * d2.1 - h2 * d1.1) / det, (d1.0 * h2 - d2.0 * h1) / det));
    }
    // vertices come out counter-clockwise; holes are clockwise
    if ring_area(ring) < 0.0 {
        out.reverse();
    }
    out
}

/// Exact copper shape of a fill mask: every connected piece as one polygon, with the holes
/// (foreign pads, vias, traces and their clearance) cut in by zero-width slits. Returns the
/// polygons and the number of pieces that had to be dropped.
pub(crate) fn mask_polygons(g: &Grid, mask: &[bool]) -> (Vec<Vec<P>>, usize) {
    mask_polygons_tol(g, mask, g.g * 1.2, true)
}

/// Square dilation of a mask by `d` cells.
pub(crate) fn dilate(g: &Grid, mask: &[bool], d: i32) -> Vec<bool> {
    let (nx, ny) = (g.nx as i32, g.ny as i32);
    let mut rows = vec![false; mask.len()];
    for y in 0..ny {
        let mut last = i32::MIN / 2;
        for x in 0..nx {
            if mask[(y * nx + x) as usize] {
                last = x;
            }
            if x - last <= d {
                rows[(y * nx + x) as usize] = true;
            }
        }
        let mut last = i32::MAX / 2;
        for x in (0..nx).rev() {
            if mask[(y * nx + x) as usize] {
                last = x;
            }
            if last - x <= d {
                rows[(y * nx + x) as usize] = true;
            }
        }
    }
    let mut out = vec![false; mask.len()];
    for x in 0..nx {
        let mut last = i32::MIN / 2;
        for y in 0..ny {
            if rows[(y * nx + x) as usize] {
                last = y;
            }
            if y - last <= d {
                out[(y * nx + x) as usize] = true;
            }
        }
        let mut last = i32::MAX / 2;
        for y in (0..ny).rev() {
            if rows[(y * nx + x) as usize] {
                last = y;
            }
            if last - y <= d {
                out[(y * nx + x) as usize] = true;
            }
        }
    }
    out
}

/// `octagons`: replace compact cut-outs (vias, pads) by their enclosing octagon.
pub(crate) fn mask_polygons_tol(g: &Grid, mask: &[bool], tol0: f64, octagons: bool) -> (Vec<Vec<P>>, usize) {
    let to_p = |&(x, y): &(i32, i32)| -> P {
        (g.x0 + (x as f64 - 0.5) * g.g, g.y0 + (y as f64 - 0.5) * g.g)
    };
    let (outers_raw, holes_raw) = contours(mask, g.nx, g.ny);
    // A hole next to the edge of its region can fall outside the simplified outline and would
    // silently vanish (copper over a foreign via). Retry with a finer outline until every hole
    // is inside one; the last attempt keeps the exact cell outline.
    let mut chosen = None;
    for tol in [tol0, tol0 * 0.5, tol0 * 0.2, 0.0] {
        let outers: Vec<Vec<P>> = outers_raw
            .iter()
            .map(|lp| {
                let ring: Vec<P> = lp.iter().map(to_p).collect();
                if tol > 0.0 { simplify_closed(&ring, tol) } else { ring }
            })
            .filter(|p| p.len() >= 3)
            .collect();
        let hole_tol = if tol > 0.0 { tol } else { g.g * 0.5 };
        let holes: Vec<Vec<P>> = holes_raw
            .iter()
            .map(|lp| {
                let ring: Vec<P> = lp.iter().map(to_p).collect();
                // compact round-ish cut-outs (vias, pads): enclosing octagon, 8 vertices at most
                let oct = enclosing_octagon(&ring);
                let simple = if tol > 0.0 { simplify_closed(&ring, hole_tol) } else { ring.clone() };
                if octagons && tol > 0.0 && oct.len() == 8 && ring_area(&oct).abs() <= 1.3 * ring_area(&ring).abs() && simple.len() > 10 {
                    oct
                } else {
                    simple
                }
            })
            .filter(|p| p.len() >= 3)
            .collect();
        // each hole belongs to the smallest outer ring that contains it
        let mut owned: Vec<Vec<Vec<P>>> = vec![vec![]; outers.len()];
        let mut lost = 0;
        for h in holes {
            let best = (0..outers.len())
                .filter(|&i| h.iter().all(|&q| point_in_poly(q, &outers[i])))
                .min_by(|&a, &b| ring_area(&outers[a]).abs().total_cmp(&ring_area(&outers[b]).abs()));
            match best {
                Some(i) => owned[i].push(h),
                None => lost += 1,
            }
        }
        let last = tol == 0.0;
        if lost == 0 || last {
            chosen = Some((outers, owned));
            break;
        }
    }
    let (outers, owned) = chosen.unwrap_or_default();
    let mut polys = vec![];
    let mut dropped = 0;
    for (o, hs) in outers.into_iter().zip(owned) {
        let (p, failed) = merge_holes_checked(o, hs);
        if failed > 0 {
            dropped += 1;
        } else {
            polys.push(p);
        }
    }
    (polys, dropped)
}

fn dp(pts: &[P], tol: f64, out: &mut Vec<P>) {
    // Douglas-Peucker on an open chain; first point is kept, last is appended by the caller
    if pts.len() < 3 {
        out.push(pts[0]);
        return;
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let (mut idx, mut dmax) = (0, 0.0);
    for (i, &p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let d = dist_pt_seg(p, a, b);
        if d > dmax {
            dmax = d;
            idx = i;
        }
    }
    if dmax > tol {
        dp(&pts[..=idx], tol, out);
        dp(&pts[idx..], tol, out);
    } else {
        out.push(a);
    }
}

pub(crate) fn simplify_closed(pts: &[P], tol: f64) -> Vec<P> {
    if pts.len() < 4 {
        return pts.to_vec();
    }
    // split the ring at the two points farthest apart so both halves are open chains
    let a = pts[0];
    let far = (0..pts.len())
        .max_by(|&i, &j| dist(a, pts[i]).total_cmp(&dist(a, pts[j])))
        .unwrap();
    let mut first: Vec<P> = pts[..=far].to_vec();
    let mut second: Vec<P> = pts[far..].to_vec();
    second.push(pts[0]);
    let mut out = vec![];
    dp(&first, tol, &mut out);
    dp(&second, tol, &mut out);
    first.clear();
    out.dedup_by(|p, q| dist(*p, *q) < 1e-9);
    out
}

/// A stretch of boundary between two labelled regions of a cell map (or a region and the
/// outside, label `u8::MAX`), traced along the lattice lines between the cells.
#[derive(Clone, Debug)]
pub(crate) struct Chain {
    /// lattice points; the first equals the last on a closed loop
    pub pts: Vec<(i32, i32)>,
    /// region on whose side the chain runs counter-clockwise (outer boundaries)
    pub owner: u8,
    /// region on the other side
    pub nbr: u8,
    /// index of the chain whose simplification this one shares (itself when it is the master)
    pub master: usize,
    /// this chain runs the other way round than its master
    pub reversed: bool,
}

/// All boundaries between differently labelled cells, cut into chains at the points where three
/// labels meet. Two neighbouring regions share one master chain, so whatever is done to it
/// (simplification) leaves no gap and no overlap between them.
pub(crate) fn trace_chains(lab: &[u8], nx: usize, ny: usize) -> Vec<Chain> {
    let at = |x: i32, y: i32| -> u8 {
        if x < 0 || y < 0 || x >= nx as i32 || y >= ny as i32 {
            u8::MAX
        } else {
            lab[y as usize * nx + x as usize]
        }
    };
    // directed edges owned by the region on the inside (same convention as `contours`)
    let mut edges: Vec<((i32, i32), (i32, i32), u8, u8)> = vec![];
    for y in 0..ny as i32 {
        for x in 0..nx as i32 {
            let a = at(x, y);
            if a == u8::MAX {
                continue;
            }
            let mut push = |s: (i32, i32), e: (i32, i32), b: u8| {
                if b != a {
                    edges.push((s, e, a, b));
                }
            };
            push((x, y), (x + 1, y), at(x, y - 1));
            push((x + 1, y), (x + 1, y + 1), at(x + 1, y));
            push((x + 1, y + 1), (x, y + 1), at(x, y + 1));
            push((x, y + 1), (x, y), at(x - 1, y));
        }
    }
    let mut out_of: HashMap<(u8, (i32, i32)), Vec<usize>> = HashMap::new();
    let mut in_of: HashMap<(u8, (i32, i32)), usize> = HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        out_of.entry((e.2, e.0)).or_default().push(i);
        *in_of.entry((e.2, e.1)).or_default() += 1;
    }
    let next = |i: usize| -> Option<usize> {
        let e = &edges[i];
        let o = out_of.get(&(e.2, e.1))?;
        if o.len() == 1 && in_of.get(&(e.2, e.1)) == Some(&1) && edges[o[0]].3 == e.3 {
            Some(o[0])
        } else {
            None
        }
    };
    let mut has_prev = vec![false; edges.len()];
    for i in 0..edges.len() {
        if let Some(j) = next(i) {
            has_prev[j] = true;
        }
    }
    let mut seen = vec![false; edges.len()];
    let mut chains: Vec<Chain> = vec![];
    let mut walk = |start: usize, seen: &mut Vec<bool>, closed: bool| {
        let mut pts = vec![edges[start].0];
        let mut cur = start;
        loop {
            seen[cur] = true;
            pts.push(edges[cur].1);
            match next(cur) {
                Some(n) if !seen[n] => cur = n,
                _ => break,
            }
        }
        if closed {
            // start every loop at its smallest point so both sides agree
            let m = (0..pts.len() - 1).min_by_key(|&i| pts[i]).unwrap_or(0);
            let body: Vec<(i32, i32)> = pts[..pts.len() - 1].to_vec();
            pts = (0..body.len()).map(|k| body[(m + k) % body.len()]).collect();
            pts.push(pts[0]);
        }
        chains.push(Chain { pts, owner: edges[start].2, nbr: edges[start].3, master: usize::MAX, reversed: false });
    };
    for i in 0..edges.len() {
        if !seen[i] && !has_prev[i] {
            walk(i, &mut seen, false);
        }
    }
    for i in 0..edges.len() {
        if !seen[i] {
            walk(i, &mut seen, true);
        }
    }
    // masters: the chain of the smaller label; the other side follows it reversed
    let mut open_key: HashMap<((i32, i32), (i32, i32)), usize> = HashMap::new();
    let mut closed_key: HashMap<(i32, i32), usize> = HashMap::new();
    for (i, c) in chains.iter().enumerate() {
        if c.owner < c.nbr {
            if c.pts.first() == c.pts.last() {
                closed_key.insert(c.pts[0], i);
            } else {
                open_key.insert((c.pts[0], c.pts[1]), i);
            }
        }
    }
    for i in 0..chains.len() {
        let c = &chains[i];
        let found = if c.owner < c.nbr {
            Some(i)
        } else if c.pts.first() == c.pts.last() {
            closed_key.get(&c.pts[0]).copied()
        } else {
            open_key.get(&(c.pts[c.pts.len() - 1], c.pts[c.pts.len() - 2])).copied()
        };
        match found {
            Some(m) if m == i => chains[i].master = i,
            Some(m) => {
                chains[i].master = m;
                chains[i].reversed = true;
            }
            None => chains[i].master = i,
        }
    }
    chains
}

/// Octilinear simplification of an open chain of lattice points: the ends stay, every piece is
/// horizontal, vertical or diagonal and no skipped point is farther than `tol` from it.
pub(crate) fn simplify_open(pts: &[(i32, i32)], tol: f64) -> Vec<(i32, i32)> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let dev = |p: (i32, i32), a: (i32, i32), b: (i32, i32)| -> f64 {
        let (px, py, ax, ay, bx, by) = (p.0 as f64, p.1 as f64, a.0 as f64, a.1 as f64, b.0 as f64, b.1 as f64);
        let (dx, dy) = (bx - ax, by - ay);
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1e-12 { 0.0 } else { (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0) };
        ((px - ax - t * dx).powi(2) + (py - ay - t * dy).powi(2)).sqrt()
    };
    let mut out = vec![pts[0]];
    let mut i = 0;
    while i < n - 1 {
        let mut best = i + 1;
        let mut miss = 0;
        let mut j = i + 2;
        while j < n && miss <= 64 {
            let (a, b) = (pts[i], pts[j]);
            let (dx, dy) = ((b.0 - a.0).abs(), (b.1 - a.1).abs());
            if (dx == 0 || dy == 0 || dx == dy) && (i + 1..j).all(|m| dev(pts[m], a, b) <= tol) {
                best = j;
                miss = 0;
            } else {
                miss += 1;
            }
            j += 1;
        }
        out.push(pts[best]);
        i = best;
    }
    out
}

/// Rings of every label from the (simplified) chains: `simplified[i]` runs along chain `i`.
/// Counter-clockwise rings are outer boundaries, clockwise ones are holes.
pub(crate) fn assemble_rings(chains: &[Chain], simplified: &[Vec<(i32, i32)>], labels: usize) -> Vec<Vec<Vec<(i32, i32)>>> {
    let mut result: Vec<Vec<Vec<(i32, i32)>>> = vec![vec![]; labels];
    for k in 0..labels {
        let mine: Vec<usize> = (0..chains.len()).filter(|&i| chains[i].owner as usize == k).collect();
        let mut by_start: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for &i in &mine {
            by_start.entry(simplified[i][0]).or_default().push(i);
        }
        let mut used: std::collections::HashSet<usize> = Default::default();
        for &first in &mine {
            if used.contains(&first) {
                continue;
            }
            let mut ring: Vec<(i32, i32)> = vec![];
            let mut cur = first;
            let origin = simplified[first][0];
            loop {
                used.insert(cur);
                let s = &simplified[cur];
                ring.extend_from_slice(&s[..s.len() - 1]);
                let end = *s.last().unwrap();
                if end == origin {
                    break;
                }
                match by_start.get(&end).and_then(|v| v.iter().copied().find(|c| !used.contains(c))) {
                    Some(n) => cur = n,
                    None => break,
                }
            }
            ring.dedup();
            if ring.len() >= 3 {
                result[k].push(ring);
            }
        }
    }
    result
}

impl Router {
    pub fn plane_outlines(&self) -> Vec<PlaneOutline> {
        let g = &self.grid;
        let tol = g.g * 1.2;
        let mut out = vec![];
        for &(l, n, region) in &self.fill_done {
            let layer = self.layer_names[l].clone();
            let net = self.nets[n].name.clone();
            match region {
                None => out.push(PlaneOutline {
                    layer,
                    net,
                    kind: "pour",
                    pts: self.boundary.clone(),
                }),
                Some(pi) => {
                    let pid = &self.plane_id[l];
                    let mask: Vec<bool> = pid.iter().map(|&v| v as usize == pi + 1).collect();
                    let to_mm = |&(x, y): &(i32, i32)| -> P {
                        (g.x0 + (x as f64 - 0.5) * g.g, g.y0 + (y as f64 - 0.5) * g.g)
                    };
                    let (outers, holes) = contours(&mask, g.nx, g.ny);
                    let outers: Vec<Vec<P>> = outers
                        .iter()
                        .map(|lp| simplify_closed(&lp.iter().map(to_mm).collect::<Vec<_>>(), tol))
                        .filter(|p| p.len() >= 3 && ring_area(p).abs() >= (g.g * 6.0).powi(2))
                        .collect();
                    let holes: Vec<Vec<P>> = holes
                        .iter()
                        .map(|lp| simplify_closed(&lp.iter().map(to_mm).collect::<Vec<_>>(), tol))
                        .filter(|p| p.len() >= 3 && ring_area(p).abs() >= (g.g * 4.0).powi(2))
                        .collect();
                    for o in &outers {
                        let inside: Vec<Vec<P>> = holes
                            .iter()
                            .filter(|h| point_in_poly(h[0], o))
                            .cloned()
                            .collect();
                        out.push(PlaneOutline {
                            layer: layer.clone(),
                            net: net.clone(),
                            kind: if self.planes[pi].filler { "filler" } else { "plane" },
                            pts: merge_holes(o.clone(), inside),
                        });
                    }
                }
            }
        }
        out
    }
}

/// ASCII DXF (R12 polylines, millimetres); one closed polyline per outline, layer = LAYER_NET.
pub fn write_dxf(r: &Router, outlines: &[PlaneOutline]) -> String {
    use std::fmt::Write;
    let mm = r.unit_mm;
    let mut o = String::new();
    let _ = writeln!(o, "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES");
    for pl in outlines {
        let name: String = format!("{}_{}_{}", pl.layer, pl.net, pl.kind)
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
            .collect();
        let _ = writeln!(o, "0\nPOLYLINE\n8\n{}\n66\n1\n70\n1", name);
        for p in &pl.pts {
            let _ = writeln!(o, "0\nVERTEX\n8\n{}\n10\n{:.4}\n20\n{:.4}", name, p.0 * mm, p.1 * mm);
        }
        let _ = writeln!(o, "0\nSEQEND\n8\n{}", name);
    }
    let _ = writeln!(o, "0\nENDSEC\n0\nEOF");
    o
}

/// Human readable list: layer, net, kind and the vertices in millimetres.
pub fn write_csv(r: &Router, outlines: &[PlaneOutline]) -> String {
    use std::fmt::Write;
    let mm = r.unit_mm;
    let mut o = String::from("layer,net,kind,vertices_mm\n");
    for pl in outlines {
        let v: Vec<String> = pl.pts.iter().map(|p| format!("{:.3} {:.3}", p.0 * mm, p.1 * mm)).collect();
        let _ = writeln!(o, "\"{}\",\"{}\",{},\"{}\"", pl.layer, pl.net, pl.kind, v.join("; "));
    }
    o
}

/// Best-effort KiCad board with only the board outline and the plane / pour zones, for CADs that
/// import KiCad boards together with their copper zones (copy the zones into the real board).
pub fn write_kicad(r: &Router, outlines: &[PlaneOutline]) -> String {
    use std::fmt::Write;
    let mm = r.unit_mm;
    let kname = |l: &str| -> String {
        match l {
            "1" => "F.Cu".into(),
            "2" => "B.Cu".into(),
            "21" => "In1.Cu".into(),
            "22" => "In2.Cu".into(),
            other => {
                let n: String = other.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
                format!("In_{}.Cu", n)
            }
        }
    };
    let mut nets: Vec<String> = vec![];
    for pl in outlines {
        if !nets.contains(&pl.net) {
            nets.push(pl.net.clone());
        }
    }
    let mut o = String::new();
    let _ = writeln!(o, "(kicad_pcb (version 20221018) (generator autorouter)");
    let _ = writeln!(o, "  (general (thickness 1.6))");
    let _ = writeln!(o, "  (paper \"A3\")");
    let _ = writeln!(o, "  (layers (0 \"F.Cu\" signal) (1 \"In1.Cu\" signal) (2 \"In2.Cu\" signal) (31 \"B.Cu\" signal) (44 \"Edge.Cuts\" user))");
    let _ = writeln!(o, "  (setup (pad_to_mask_clearance 0))");
    let _ = writeln!(o, "  (net 0 \"\")");
    for (i, n) in nets.iter().enumerate() {
        let _ = writeln!(o, "  (net {} \"{}\")", i + 1, n.replace('"', ""));
    }
    let b = &r.boundary;
    for i in 0..b.len() {
        let (p, q) = (b[i], b[(i + 1) % b.len()]);
        let _ = writeln!(
            o,
            "  (gr_line (start {:.4} {:.4}) (end {:.4} {:.4}) (layer \"Edge.Cuts\") (width 0.1))",
            p.0 * mm, -p.1 * mm, q.0 * mm, -q.1 * mm
        );
    }
    for pl in outlines {
        let id = nets.iter().position(|n| *n == pl.net).unwrap() + 1;
        let _ = writeln!(
            o,
            "  (zone (net {}) (net_name \"{}\") (layer \"{}\") (hatch edge 0.5)\n    (connect_pads (clearance {:.3}))\n    (min_thickness {:.3}) (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))\n    (polygon (pts",
            id,
            pl.net.replace('"', ""),
            kname(&pl.layer),
            r.cfg.rules.min_clearance_mm.max(0.1),
            r.cfg.rules.min_trace_mm.max(0.1)
        );
        for p in &pl.pts {
            let _ = writeln!(o, "      (xy {:.4} {:.4})", p.0 * mm, -p.1 * mm);
        }
        let _ = writeln!(o, "    )))");
    }
    let _ = writeln!(o, ")");
    o
}
