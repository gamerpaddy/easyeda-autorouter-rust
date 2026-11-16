//! Helpers for copper that already exists in the DSN (pads, wiring, vias, planes):
//! rasterising it to grid cells and finding electrically connected groups.

use crate::engine::{Cell, Grid};
use crate::geom::*;

/// Grid cells that lie well inside a primitive (used as seeds/targets for connecting to it).
pub fn prim_cells(grid: &Grid, pr: &Prim) -> Vec<Cell> {
    let (core, lim): (Box<dyn Fn(P) -> f64>, f64) = match &pr.geo {
        Geo::Seg { a, b, hw } => {
            let (a, b) = (*a, *b);
            (Box::new(move |p| dist_pt_seg(p, a, b)), hw * 0.9)
        }
        Geo::Circle { c, r } => {
            let c = *c;
            (Box::new(move |p| dist(p, c)), r * 0.9)
        }
        Geo::Poly(pts) => {
            let pts = pts.clone();
            (
                Box::new(move |p| {
                    if point_in_poly(p, &pts) {
                        0.0
                    } else {
                        f64::MAX
                    }
                }),
                0.0,
            )
        }
    };
    let x0 = (((pr.bb.0 - grid.x0) / grid.g).floor() as i32).max(0);
    let x1 = (((pr.bb.2 - grid.x0) / grid.g).ceil() as i32).min(grid.nx as i32 - 1);
    let y0 = (((pr.bb.1 - grid.y0) / grid.g).floor() as i32).max(0);
    let y1 = (((pr.bb.3 - grid.y0) / grid.g).ceil() as i32).min(grid.ny as i32 - 1);
    let mut v = vec![];
    for y in y0..=y1 {
        for x in x0..=x1 {
            if core(grid.center(x, y)) <= lim {
                v.push((pr.layer, x, y));
            }
        }
    }
    if v.is_empty() {
        let c = ((pr.bb.0 + pr.bb.2) / 2.0, (pr.bb.1 + pr.bb.3) / 2.0);
        let (x, y) = grid.cell(c);
        v.push((
            pr.layer,
            x.clamp(0, grid.nx as i32 - 1),
            y.clamp(0, grid.ny as i32 - 1),
        ));
    }
    v
}

pub fn find(uf: &mut Vec<usize>, mut x: usize) -> usize {
    while uf[x] != x {
        uf[x] = uf[uf[x]];
        x = uf[x];
    }
    x
}

/// Union-find over primitives: touching copper on the same layer is connected; `extra` forces
/// additional unions (e.g. the layers of one through-hole pad, or via copper on all layers).
pub fn components(prims: &[&Prim], extra: &[(usize, usize)]) -> Vec<usize> {
    let m = prims.len();
    let mut uf: Vec<usize> = (0..m).collect();
    for i in 0..m {
        for j in i + 1..m {
            let (a, b) = (prims[i], prims[j]);
            if a.layer == b.layer
                && a.bb.0 <= b.bb.2 + 1e-6
                && b.bb.0 <= a.bb.2 + 1e-6
                && a.bb.1 <= b.bb.3 + 1e-6
                && b.bb.1 <= a.bb.3 + 1e-6
                && a.gap(b) <= 1e-6
            {
                let (ra, rb) = (find(&mut uf, i), find(&mut uf, j));
                uf[ra] = rb;
            }
        }
    }
    for &(a, b) in extra {
        let (ra, rb) = (find(&mut uf, a), find(&mut uf, b));
        uf[ra] = rb;
    }
    (0..m).map(|i| find(&mut uf, i)).collect()
}
