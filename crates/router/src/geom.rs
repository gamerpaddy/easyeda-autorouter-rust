//! Plane geometry helpers and copper primitives.

pub type P = (f64, f64);

#[inline]
pub fn dist(a: P, b: P) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

pub fn dist_pt_seg(p: P, a: P, b: P) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-18 {
        return dist(p, a);
    }
    let t = (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0);
    dist(p, (a.0 + t * dx, a.1 + t * dy))
}

pub fn point_in_poly(p: P, pts: &[P]) -> bool {
    let mut inside = false;
    let n = pts.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (pts[i], pts[j]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        j = i;
    }
    inside
}

pub fn dist_pt_poly(p: P, pts: &[P]) -> f64 {
    if point_in_poly(p, pts) {
        return 0.0;
    }
    let n = pts.len();
    let mut d = f64::MAX;
    for i in 0..n {
        d = d.min(dist_pt_seg(p, pts[i], pts[(i + 1) % n]));
    }
    d
}

fn ccw(a: P, b: P, c: P) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

pub fn segs_cross(a: P, b: P, c: P, d: P) -> bool {
    let (d1, d2, d3, d4) = (ccw(c, d, a), ccw(c, d, b), ccw(a, b, c), ccw(a, b, d));
    ((d1 > 0.0) != (d2 > 0.0))
        && ((d3 > 0.0) != (d4 > 0.0))
        && d1 != 0.0
        && d2 != 0.0
        && d3 != 0.0
        && d4 != 0.0
}

pub fn seg_seg_dist(a: P, b: P, c: P, d: P) -> f64 {
    if segs_cross(a, b, c, d) {
        return 0.0;
    }
    dist_pt_seg(a, c, d)
        .min(dist_pt_seg(b, c, d))
        .min(dist_pt_seg(c, a, b))
        .min(dist_pt_seg(d, a, b))
}

#[derive(Clone, Debug)]
pub enum Geo {
    Circle {
        c: P,
        r: f64,
    },
    Poly(Vec<P>),
    /// segment with half width (a trace)
    Seg {
        a: P,
        b: P,
        hw: f64,
    },
}

/// A piece of copper on one layer.
#[derive(Clone, Debug)]
pub struct Prim {
    pub layer: usize,
    pub geo: Geo,
    /// owning net id, -1 = no net, -2 = board outline / keepout (never rippable)
    pub net: i32,
    pub clr: f64,
    pub bb: (f64, f64, f64, f64),
    /// blocks traces only (exclusive plane regions): vias may still pass through
    pub trace_only: bool,
    /// generated plane / pour copper
    pub fill: bool,
}

impl Prim {
    pub fn new(layer: usize, geo: Geo, net: i32, clr: f64) -> Prim {
        let bb = match &geo {
            Geo::Circle { c, r } => (c.0 - r, c.1 - r, c.0 + r, c.1 + r),
            Geo::Seg { a, b, hw } => (
                a.0.min(b.0) - hw,
                a.1.min(b.1) - hw,
                a.0.max(b.0) + hw,
                a.1.max(b.1) + hw,
            ),
            Geo::Poly(p) => {
                let mut bb = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                for &(x, y) in p {
                    bb = (bb.0.min(x), bb.1.min(y), bb.2.max(x), bb.3.max(y));
                }
                bb
            }
        };
        Prim {
            layer,
            geo,
            net,
            clr,
            bb,
            trace_only: false,
            fill: false,
        }
    }

    /// Distance from point to the copper edge (0 when inside).
    pub fn dist(&self, p: P) -> f64 {
        match &self.geo {
            Geo::Circle { c, r } => (dist(p, *c) - r).max(0.0),
            Geo::Poly(pts) => dist_pt_poly(p, pts),
            Geo::Seg { a, b, hw } => (dist_pt_seg(p, *a, *b) - hw).max(0.0),
        }
    }

    /// Edge-to-edge distance to another primitive (for DRC). Negative = overlap.
    pub fn gap(&self, o: &Prim) -> f64 {
        use Geo::*;
        match (&self.geo, &o.geo) {
            (Circle { c, r }, _) => o.dist(*c) - r,
            (_, Circle { c, r }) => self.dist(*c) - r,
            (Seg { a, b, hw }, Seg { a: c, b: d, hw: h2 }) => {
                seg_seg_dist(*a, *b, *c, *d) - hw - h2
            }
            (Seg { a, b, hw }, Poly(p)) | (Poly(p), Seg { a, b, hw }) => {
                if point_in_poly(*a, p) || point_in_poly(*b, p) {
                    return -*hw;
                }
                let n = p.len();
                let mut d = f64::MAX;
                for i in 0..n {
                    d = d.min(seg_seg_dist(*a, *b, p[i], p[(i + 1) % n]));
                }
                d - hw
            }
            (Poly(p), Poly(q)) => {
                if point_in_poly(p[0], q) || point_in_poly(q[0], p) {
                    return 0.0;
                }
                let (n, m) = (p.len(), q.len());
                let mut d = f64::MAX;
                for i in 0..n {
                    for j in 0..m {
                        d = d.min(seg_seg_dist(p[i], p[(i + 1) % n], q[j], q[(j + 1) % m]));
                    }
                }
                d
            }
        }
    }
}
