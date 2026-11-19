//! Bounded fine-grid repairs. Existing copper stays in world coordinates; only the
//! search window is refined. Never round the accepted geometry back to the coarse grid.
use crate::{drc, engine::*, geom::*, prewire::prim_cells};

const MAX_CELLS: usize = 8_000_000;

fn copper_cells(grid: &Grid, prims: &[Prim]) -> Vec<Cell> {
    let mut cells: Vec<_> = prims.iter().filter(|p| !p.trace_only)
        .flat_map(|p| prim_cells(grid, p).into_iter().filter(move |&(_, x, y)| {
            // prim_cells has a nearest-cell fallback, which is not a valid terminal
            // for a cropped window or a pad narrower than the search grid.
            p.dist(grid.center(x, y)) <= 1e-9
        })).collect();
    cells.sort_unstable();
    cells.dedup();
    cells
}

impl Router {
    pub(crate) fn adaptive_repair(&mut self) {
        let t0 = std::time::Instant::now();
        self.adaptive_repair_inner();
        crate::engine::PROF_ADAPTIVE_MS.fetch_add(t0.elapsed().as_millis() as u64, std::sync::atomic::Ordering::Relaxed);
    }

    fn adaptive_repair_inner(&mut self) {
        let failing: Vec<_> = (0..self.nets.len()).filter(|&n|
            !self.nets[n].skip && !self.nets[n].unconnected.is_empty()
                && self.nets[n].plane.is_none()).collect();
        for n in failing {
            if self.cancelled() { return; }
            self.adaptive_net(n);
        }
    }

    fn adaptive_net(&mut self, n: usize) {
        let mut islands = drc::net_islands(self, n);
        if islands.len() == 1 {
            self.nets[n].unconnected.clear();
            return;
        }
        let max_bridges = islands.len().saturating_sub(1);
        for _ in 0..max_bridges {
            if self.cancelled() || islands.len() < 2 { break; }
            // Start with the smallest disconnected piece; all other copper of this
            // net is a target, including already-routed branches and vias.
            islands.sort_by_key(|i| (i.pins.len(), i.prims.len()));
            let source = &islands[0];
            let targets: Vec<_> = islands[1..].iter().flat_map(|i| i.prims.iter().cloned()).collect();
            let mut closest = (f64::INFINITY, (0.0, 0.0), (0.0, 0.0));
            for &a in &source.pins {
                for b in islands[1..].iter().flat_map(|i| i.pins.iter()) {
                    let (a, b) = (self.pins[a].c, self.pins[*b].c);
                    let d = dist(a, b);
                    if d < closest.0 { closest = (d, a, b); }
                }
            }
            if !closest.0.is_finite() { break; }
            let mut accepted = false;
            for (factor, margin_mm) in [(2.0, 1.0), (4.0, 1.0), (2.0, 4.0)] {
                if self.cancelled() { return; }
                let step = self.grid.g / factor;
                // Below 5 microns further refinement is unlikely to help, and can
                // explode memory. This limit never relaxes a design rule.
                if step * self.unit_mm < 0.005 { continue; }
                let margin = margin_mm / self.unit_mm;
                let (a, b) = (closest.1, closest.2);
                let x0 = (a.0.min(b.0) - margin).max(self.grid.x0);
                let y0 = (a.1.min(b.1) - margin).max(self.grid.y0);
                let x1 = (a.0.max(b.0) + margin).min(self.grid.center(self.grid.nx as i32 - 1, 0).0);
                let y1 = (a.1.max(b.1) + margin).min(self.grid.center(0, self.grid.ny as i32 - 1).1);
                let grid = Grid { x0, y0, g: step,
                    nx: ((x1 - x0) / step).floor().max(0.0) as usize + 1,
                    ny: ((y1 - y0) / step).floor().max(0.0) as usize + 1,
                    nl: self.grid.nl };
                if grid.n().saturating_mul(grid.nl) > MAX_CELLS { continue; }
                if self.cfg.verbose {
                    eprintln!("adaptive search {}: {}x{}x{}, {:.4} mm", self.nets[n].name,
                        grid.nx, grid.ny, grid.nl, step * self.unit_mm);
                }
                self.snapshot(&format!("Adaptive repair {}: {:.4} mm local grid", self.nets[n].name, step * self.unit_mm));
                let Some(mut route) = self.adaptive_search(n, grid, &source.prims, &targets) else { continue; };
                // Rasterisation is only a search aid. Check complete segments and
                // through-via geometry against the original, unsnapped obstacles.
                if self.prims_conflict(&route.dynp).is_some() { continue; }
                let old = self.nets[n].r.clone();
                route.tree = copper_cells(&self.grid, &route.dynp);
                self.stamp_dyn(&route.dynp, 1);
                self.nets[n].r.wires.extend(route.wires);
                self.nets[n].r.vias.extend(route.vias);
                self.nets[n].r.tree.extend(route.tree);
                self.nets[n].r.dynp.extend(route.dynp.iter().cloned());
                let next = drc::net_islands(self, n);
                if next.len() >= islands.len() {
                    self.stamp_dyn(&route.dynp, -1);
                    self.nets[n].r = old;
                    continue;
                }
                if self.cfg.verbose {
                    eprintln!("adaptive {}: islands {} -> {}, local grid {:.4} mm", self.nets[n].name, islands.len(), next.len(), step * self.unit_mm);
                }
                // Keep the component containing the original root as the connected
                // tree; each other island contributes one open connection.
                let root = self.nets[n].comps.first().and_then(|c| c.first()).copied();
                self.nets[n].unconnected = next.iter()
                    .filter(|i| root.is_none_or(|p| !i.pins.contains(&p)))
                    .filter_map(|i| i.pins.first().copied()).collect();
                islands = next;
                accepted = true;
                break;
            }
            if !accepted { break; }
        }
    }

    fn adaptive_search(&self, n: usize, grid: Grid, source: &[Prim], targets: &[Prim]) -> Option<Routed> {
        let seeds = copper_cells(&grid, source);
        let targets = copper_cells(&grid, targets);
        if seeds.is_empty() || targets.is_empty() { return None; }
        let ns = &self.nets[n];
        let width = if self.cfg.allow_neckdown { self.fields[ns.narrow_tf].p * 2.0 } else { ns.width };
        let mut fields = vec![Field::new(&grid, ns.clr, width / 2.0, false), Field::new(&grid, ns.clr, ns.via_r, true)];
        for p in self.fixed.iter().chain(self.pins.iter().flat_map(|p| &p.prims))
            .chain(self.nets.iter().flat_map(|n| n.pre.dynp.iter().chain(&n.r.dynp))) {
            if p.net != n as i32 {
                for f in &mut fields { f.stamp(&grid, p, 1, false); }
            }
        }
        if !self.cfg.allow_vias_in_pads {
            for p in self.pins.iter().flat_map(|p| &p.prims) { fields[1].stamp(&grid, p, 1, false); }
        }
        // Preserve plane preferences on the local lattice, without changing the
        // exclusion of plane layers from signal routing.
        let plane_id = self.plane_id.iter().map(|ids| {
            if ids.is_empty() { return vec![]; }
            (0..grid.n()).map(|i| {
                let (x, y) = self.grid.cell(grid.center((i % grid.nx) as i32, (i / grid.nx) as i32));
                ids[y.clamp(0, self.grid.ny as i32 - 1) as usize * self.grid.nx + x.clamp(0, self.grid.nx as i32 - 1) as usize]
            }).collect()
        }).collect();
        let mut local = self.copy_for_grid(grid.clone(), fields,
            vec![0; grid.n() * grid.nl], plane_id, vec![]);
        local.guide = None;
        local.cfg.via_cost *= (self.grid.g / grid.g) as f32;
        local.grid = grid;
        local.cur_net = n;
        local.cur_land = false;
        local.jump_mode = false;
        local.h_weight = 1.0;
        local.pop_budget = 500_000;
        let window = (0, 0, local.grid.nx as i32 - 1, local.grid.ny as i32 - 1);
        let path = local.astar((0, 1), &seeds, &targets, false, window, 0.0)?;
        local.nets[n].width = width;
        local.nets[n].r = Routed::default();
        local.commit_path(n, &path, width);
        Some(std::mem::take(&mut local.nets[n].r))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corridor(gap_top: f64) -> Router {
        let mut b = dsn::parse_dsn(r#"(pcb adaptive (resolution mm 1)
          (structure (boundary (path pcb 0 0 0 20 0 20 10 0 10))
            (layer top (type signal)) (rule (width 0.2) (clearance 0.1)))
          (placement (component img (place U 0 0 front 0)))
          (library (image img (pin pad 1 2 5.15) (pin pad 2 18 5.15))
            (padstack pad (shape (circle top 0.4))))
          (network (net DATA (pins U-1 U-2))))"#).unwrap();
        for (y1, y2) in [(0.0, 4.9), (gap_top, 10.0)] {
            b.keepouts.push(dsn::Keepout { layer: None,
                shape: dsn::Shape::Rect { layer: "top".into(), x1: 8.0, y1, x2: 12.0, y2 },
                via_only: false, wire_only: false });
        }
        let mut cfg = Config { grid: Some(0.5), unit_mm: Some(1.0),
            allow_neckdown: false, verbose: false, ..Config::default() };
        cfg.rules.enabled = false;
        Router::new(&b, cfg)
    }

    #[test]
    fn local_refinement_finds_corridor_between_coarse_rows() {
        let mut r = corridor(5.4);
        r.route_net_plain(0, false);
        assert_eq!(r.unrouted_count(), 1, "coarse lattice must miss the corridor");
        let grid = r.grid.g;
        r.adaptive_repair();
        assert_eq!(r.grid.g, grid, "the global lattice must not change");
        assert_eq!(r.unrouted_count(), 0);
        let report = drc::check(&r);
        assert!(report.open_nets.is_empty());
        assert!(report.clearance_violations.is_empty(), "{:?}", report.clearance_violations);
        assert!(r.nets[0].r.wires.iter().all(|w| (w.width - 0.2).abs() < 1e-9));
    }

    #[test]
    fn refinement_cannot_squeeze_through_an_illegal_gap() {
        let mut r = corridor(5.2);
        let fields: Vec<_> = r.fields.iter().map(|f| f.cnt.clone()).collect();
        r.adaptive_repair();
        assert_eq!(r.unrouted_count(), 1);
        assert!(r.nets[0].r.dynp.is_empty());
        assert_eq!(fields, r.fields.iter().map(|f| f.cnt.clone()).collect::<Vec<_>>());
    }

    #[test]
    fn cropped_terminals_do_not_snap_to_window_edges() {
        let grid = Grid { x0: 0.0, y0: 0.0, g: 0.5, nx: 10, ny: 10, nl: 1 };
        let outside = Prim::new(0, Geo::Circle { c: (8.0, 3.0), r: 0.2 }, 0, 0.1);
        assert!(copper_cells(&grid, &[outside]).is_empty());
    }
}
