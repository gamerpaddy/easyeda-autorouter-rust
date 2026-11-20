//! Small geometry helpers: polygons, regions, arc flattening, path parsing.
use crate::model::P;
use serde_json::Value;

pub fn point_in_poly(p: P, poly: &[P]) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Does the closed segment a-b touch the (closed) box?
fn seg_hits_box(a: P, b: P, bx: [f64; 4]) -> bool {
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    let d = (b.0 - a.0, b.1 - a.1);
    for (p, q) in [(-d.0, a.0 - bx[0]), (d.0, bx[2] - a.0), (-d.1, a.1 - bx[1]), (d.1, bx[3] - a.1)] {
        if p == 0.0 {
            if q < 0.0 {
                return false;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                if r > t1 {
                    return false;
                }
                t0 = t0.max(r);
            } else {
                if r < t0 {
                    return false;
                }
                t1 = t1.min(r);
            }
        }
    }
    true
}

pub fn edges_hit_box(poly: &[P], bx: [f64; 4]) -> bool {
    let n = poly.len();
    (0..n).any(|i| seg_hits_box(poly[i], poly[(i + 1) % n], bx))
}

pub fn bbox_of(pts: &[P]) -> [f64; 4] {
    let mut r = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    for p in pts {
        r[0] = r[0].min(p.0);
        r[1] = r[1].min(p.1);
        r[2] = r[2].max(p.0);
        r[3] = r[3].max(p.1);
    }
    r
}

pub fn area(poly: &[P]) -> f64 {
    let n = poly.len();
    let mut s = 0.0;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        s += a.0 * b.1 - b.0 * a.1;
    }
    s.abs() / 2.0
}

/// The area parts may use: an outline (or a rectangle) minus cut-outs.
#[derive(Clone, Debug)]
pub struct Region {
    pub poly: Vec<P>,
    pub cut: Vec<Vec<P>>,
    /// set when the polygon is an axis aligned rectangle
    pub rect: Option<[f64; 4]>,
    pub bbox: [f64; 4],
}

impl Region {
    pub fn from_rect(r: [f64; 4]) -> Region {
        Region { poly: vec![(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])], cut: vec![], rect: Some(r), bbox: r }
    }

    pub fn from_poly(mut poly: Vec<P>, cut: Vec<Vec<P>>) -> Region {
        if poly.len() > 1 && (poly[0].0 - poly[poly.len() - 1].0).abs() < 1e-6 && (poly[0].1 - poly[poly.len() - 1].1).abs() < 1e-6 {
            poly.pop();
        }
        let bb = bbox_of(&poly);
        let is_rect = poly.len() == 4 && {
            let eps = 1e-3;
            poly.iter().all(|p| ((p.0 - bb[0]).abs() < eps || (p.0 - bb[2]).abs() < eps) && ((p.1 - bb[1]).abs() < eps || (p.1 - bb[3]).abs() < eps))
        };
        Region { poly, cut, rect: if is_rect { Some(bb) } else { None }, bbox: bb }
    }

    /// Is the box, grown by `margin`, inside the outline and clear of the cut-outs?
    pub fn contains_box(&self, b: [f64; 4], margin: f64) -> bool {
        let g = [b[0] - margin, b[1] - margin, b[2] + margin, b[3] + margin];
        if let Some(r) = self.rect {
            if g[0] < r[0] - 1e-9 || g[1] < r[1] - 1e-9 || g[2] > r[2] + 1e-9 || g[3] > r[3] + 1e-9 {
                return false;
            }
        } else {
            if g[0] < self.bbox[0] || g[1] < self.bbox[1] || g[2] > self.bbox[2] || g[3] > self.bbox[3] {
                return false;
            }
            for c in [(g[0], g[1]), (g[2], g[1]), (g[2], g[3]), (g[0], g[3])] {
                if !point_in_poly(c, &self.poly) {
                    return false;
                }
            }
            let s = [g[0] + 1e-6, g[1] + 1e-6, g[2] - 1e-6, g[3] - 1e-6];
            if edges_hit_box(&self.poly, s) {
                return false;
            }
        }
        let s = [g[0] + 1e-6, g[1] + 1e-6, g[2] - 1e-6, g[3] - 1e-6];
        for c in &self.cut {
            let cb = bbox_of(c);
            if cb[2] < g[0] || cb[0] > g[2] || cb[3] < g[1] || cb[1] > g[3] {
                continue;
            }
            if point_in_poly(((g[0] + g[2]) / 2.0, (g[1] + g[3]) / 2.0), c) || edges_hit_box(c, s) {
                return false;
            }
        }
        true
    }
}

/// Points of the arc from `a` to `b` that turns by `sweep_deg` (positive: counter-clockwise),
/// without `a`, with `b`.
pub fn arc_points(a: P, b: P, sweep_deg: f64) -> Vec<P> {
    let sweep = sweep_deg.to_radians();
    let chord = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    if chord < 1e-9 || sweep.abs() < 1e-6 {
        return vec![b];
    }
    let r = chord / (2.0 * (sweep.abs() / 2.0).sin());
    let m = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
    let h = (r * r - chord * chord / 4.0).max(0.0).sqrt();
    let dir = ((b.0 - a.0) / chord, (b.1 - a.1) / chord);
    // left normal; the centre lies left of a->b for a counter-clockwise sweep shorter than 180
    let n = (-dir.1, dir.0);
    let sgn = if sweep > 0.0 { 1.0 } else { -1.0 };
    let off = if sweep.abs() > std::f64::consts::PI { -h } else { h };
    let c = (m.0 + n.0 * off * sgn, m.1 + n.1 * off * sgn);
    let a0 = (a.1 - c.1).atan2(a.0 - c.0);
    let steps = ((sweep.abs().to_degrees() / 10.0).ceil() as usize).max(2);
    let mut v = vec![];
    for k in 1..=steps {
        let t = a0 + sweep * k as f64 / steps as f64;
        v.push((c.0 + r * t.cos(), c.1 + r * t.sin()));
    }
    if let Some(l) = v.last_mut() {
        *l = b;
    }
    v
}

fn num(v: &Value) -> Option<f64> {
    v.as_f64()
}

/// Contours of an EasyEDA Pro path value: flat `[x,y,"L",x,y,...,"ARC",sweep,x,y]`, nested lists
/// of those, `["R",x,y,w,h,rot,radius]` and `["CIRCLE",x,y,r]`. Coordinates as in the file.
pub fn pro_path_contours(v: &Value) -> Vec<Vec<P>> {
    let mut out = vec![];
    let Some(arr) = v.as_array() else { return out };
    if arr.iter().any(|e| e.is_array()) {
        for e in arr {
            out.extend(pro_path_contours(e));
        }
        return out;
    }
    let mut cur: Vec<P> = vec![];
    let mut i = 0;
    let n = arr.len();
    let mut pend: Vec<f64> = vec![];
    while i < n {
        match &arr[i] {
            Value::String(s) => match s.as_str() {
                "L" => {}
                "ARC" | "CARC" => {
                    let sweep = arr.get(i + 1).and_then(num).unwrap_or(0.0);
                    let ex = arr.get(i + 2).and_then(num).unwrap_or(0.0);
                    let ey = arr.get(i + 3).and_then(num).unwrap_or(0.0);
                    if let Some(&a) = cur.last() {
                        cur.extend(arc_points(a, (ex, ey), sweep));
                    }
                    i += 3;
                }
                "R" => {
                    let g = |k: usize| arr.get(i + k).and_then(num).unwrap_or(0.0);
                    let (x, y, w, h) = (g(1), g(2), g(3), g(4));
                    // top-left corner, y up: extends downwards
                    out.push(vec![(x, y), (x + w, y), (x + w, y - h), (x, y - h)]);
                    i += 6;
                }
                "CIRCLE" => {
                    let g = |k: usize| arr.get(i + k).and_then(num).unwrap_or(0.0);
                    let (cx, cy, r) = (g(1), g(2), g(3));
                    out.push((0..24).map(|k| {
                        let t = k as f64 * std::f64::consts::TAU / 24.0;
                        (cx + r * t.cos(), cy + r * t.sin())
                    }).collect());
                    i += 3;
                }
                _ => {}
            },
            other => {
                if let Some(x) = num(other) {
                    pend.push(x);
                    if pend.len() == 2 {
                        cur.push((pend[0], pend[1]));
                        pend.clear();
                    }
                }
            }
        }
        i += 1;
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

/// Chain loose segments into closed loops.
pub fn chain_segments(segs: &[(P, P)]) -> Vec<Vec<P>> {
    let eps = 0.02;
    let mut used = vec![false; segs.len()];
    let mut loops = vec![];
    let close = |a: P, b: P| (a.0 - b.0).abs() < eps && (a.1 - b.1).abs() < eps;
    for s in 0..segs.len() {
        if used[s] {
            continue;
        }
        used[s] = true;
        let mut pts = vec![segs[s].0, segs[s].1];
        loop {
            let end = *pts.last().unwrap();
            if pts.len() > 2 && close(end, pts[0]) {
                break;
            }
            let mut found = false;
            for k in 0..segs.len() {
                if used[k] {
                    continue;
                }
                if close(segs[k].0, end) {
                    pts.push(segs[k].1);
                } else if close(segs[k].1, end) {
                    pts.push(segs[k].0);
                } else {
                    continue;
                }
                used[k] = true;
                found = true;
                break;
            }
            if !found {
                break;
            }
        }
        if pts.len() > 3 && close(*pts.last().unwrap(), pts[0]) {
            pts.pop();
            loops.push(pts);
        }
    }
    loops
}

/// Outline = largest loop, cut-outs = loops inside it.
pub fn split_outline(mut loops: Vec<Vec<P>>) -> (Option<Vec<P>>, Vec<Vec<P>>) {
    loops.retain(|l| l.len() >= 3);
    if loops.is_empty() {
        return (None, vec![]);
    }
    loops.sort_by(|a, b| area(b).partial_cmp(&area(a)).unwrap());
    let outline = loops.remove(0);
    let cut = loops.into_iter().filter(|l| point_in_poly(l[0], &outline)).collect();
    (Some(outline), cut)
}
