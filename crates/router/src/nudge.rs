//! Last resort for connections that stay open: nudge small passives a little (at most a few part
//! widths) to open a path, then repair the affected routes. Off by default. Only small surface
//! mount parts move: no ICs, no through-hole parts, no connectors, switches or oscillators, and
//! no part that already has wiring attached.

use crate::drc;
use crate::engine::*;
use crate::geom::*;
use dsn::Board;
use std::collections::HashSet;

/// A footprint as far as nudging is concerned.
#[derive(Clone, Debug)]
pub struct PartSpec {
    pub name: String,
    /// pin numbers (`N` of `u1-N`) of its pads
    pub pads: Vec<usize>,
    /// surface-mount only, no holes, and every object of it can be moved in the document
    pub movable: bool,
}

pub fn specs_from_doc(doc: &dsn::easyeda::EasyedaDoc) -> Vec<PartSpec> {
    doc.parts
        .iter()
        .map(|p| PartSpec {
            name: if p.designator.is_empty() { p.package.clone() } else { format!("{} ({})", p.designator, p.package) },
            pads: p.pads.clone(),
            movable: p.smd && p.movable && !excluded_package(&p.package),
        })
        .collect()
}

/// Packages that never move even though they are small: connectors, switches, oscillators,
/// crystals, adjustable parts and anything with a plug or socket in its name.
fn excluded_package(name: &str) -> bool {
    let n = name.to_uppercase();
    [
        "HDR", "HEADER", "IDC", "JACK", "CONN", "USB", "JST", "SMA-TH", "KEY", "SW-", "SWITCH", "OSC", "XTAL",
        "CRYSTAL", "ADJ", "POT", "RJ", "SOCKET", "BGA", "QFN", "QFP", "LFCSP", "SOIC", "SSOP", "TSSOP", "DIP",
        "SOP-", "MSOP", "LGA", "WLCSP", "ANTENNA", "TO-2", "SOT-223", "DPAK", "D2PAK", "FUSE", "BATTERY", "RELAY",
        "BUZZER", "SPEAKER", "MOTOR", "-TH", "TH_",
    ]
    .iter()
    .any(|k| n.contains(k))
}

/// The board with the parts moved by their total offsets (part index, dx, dy).
pub fn moved_board(board: &Board, parts: &[PartSpec], moves: &[(usize, f64, f64)]) -> Board {
    let mut b = board.clone();
    let Some(img_name) = b.components.first().map(|c| c.image.clone()) else {
        return b;
    };
    if let Some(img) = b.images.get_mut(&img_name) {
        for &(pi, dx, dy) in moves {
            let Some(p) = parts.get(pi) else { continue };
            for pin in img.pins.iter_mut() {
                if pin.id.parse::<usize>().is_ok_and(|n| p.pads.contains(&n)) {
                    pin.x += dx;
                    pin.y += dy;
                }
            }
        }
    }
    b
}

fn translate(p: &Prim, dx: f64, dy: f64) -> Prim {
    let geo = match &p.geo {
        Geo::Circle { c, r } => Geo::Circle { c: (c.0 + dx, c.1 + dy), r: *r },
        Geo::Seg { a, b, hw } => Geo::Seg { a: (a.0 + dx, a.1 + dy), b: (b.0 + dx, b.1 + dy), hw: *hw },
        Geo::Poly(v) => Geo::Poly(v.iter().map(|q| (q.0 + dx, q.1 + dy)).collect()),
    };
    let mut q = Prim::new(p.layer, geo, p.net, p.clr);
    q.trace_only = p.trace_only;
    q.fill = p.fill;
    q
}

fn total(moves: &[(usize, f64, f64)], pi: usize) -> (f64, f64) {
    moves.iter().filter(|m| m.0 == pi).fold((0.0, 0.0), |a, m| (a.0 + m.1, a.1 + m.2))
}

fn merge(moves: &[(usize, f64, f64)], pi: usize, dx: f64, dy: f64) -> Vec<(usize, f64, f64)> {
    let mut v: Vec<(usize, f64, f64)> = moves.iter().copied().filter(|m| m.0 != pi).collect();
    let t = total(moves, pi);
    v.push((pi, t.0 + dx, t.1 + dy));
    v.sort_by_key(|m| m.0);
    v
}

pub struct Solved {
    pub router: Router,
    /// total offset of every part that moved: (part index, dx, dy) in DSN units
    pub moves: Vec<(usize, f64, f64)>,
    pub log: Vec<String>,
}

struct Run {
    router: Router,
    routes: SavedRoutes,
    report: drc::DrcReport,
    fail: usize,
    violations: usize,
}

fn run_once(board: &Board, parts: &[PartSpec], cfg: &Config, skip: &[(String, String)], moves: &[(usize, f64, f64)], previous: Option<&Run>) -> Run {
    let b = moved_board(board, parts, moves);
    let mut r = Router::new(&b, cfg.clone());
    r.pour_skip = skip.to_vec();
    r.part_moves = moves.to_vec();
    if let Some(previous) = previous {
        let moved: Vec<_> = r.pins.iter().zip(&previous.router.pins)
            .filter(|(a, b)| a.c != b.c && a.net >= 0)
            .map(|(p, _)| p.net as usize).collect();
        r.repair_after_move(&previous.routes, &moved);
    } else {
        r.route_all();
        r.refine();
    }
    let routes = if cfg.nudge_parts { r.save_routes() } else { vec![] };
    r.finish();
    let report = drc::check(&r);
    let fail = r.unrouted_count().max(report.open_nets.len());
    let violations = report.clearance_violations.len();
    Run { router: r, routes, report, fail, violations }
}

/// Route the board; with `cfg.nudge_parts` and connections left open, move small parts and try
/// again, keeping every move that helps.
pub fn solve(board: &Board, cfg: Config, parts: &[PartSpec], pour_skip: &[(String, String)]) -> Solved {
    let started = std::time::Instant::now();
    let mut best = run_once(board, parts, &cfg, pour_skip, &[], None);
    let mut moves: Vec<(usize, f64, f64)> = vec![];
    let mut log: Vec<String> = vec![];
    if !cfg.nudge_parts || parts.is_empty() || board.components.len() != 1 {
        return Solved { router: best.router, moves, log };
    }
    let mut tried: HashSet<(usize, i64, i64)> = HashSet::new();
    let mut tried_parts = HashSet::new();
    let mut attempts = 0;
    while best.fail > 0 && attempts < cfg.nudge_attempts && !best.router.cancelled() {
        let cands = candidates(&best, board, parts, &cfg, &moves);
        let mut available: Vec<_> = cands.into_iter().filter(|c| !tried.contains(&(c.part, (c.dx * 1000.0).round() as i64, (c.dy * 1000.0).round() as i64))).collect();
        // Do not spend the entire attempt budget on several distant positions of
        // the same part while other parts along the blocked escape are untried.
        let candidate = available.iter().position(|c| !tried_parts.contains(&c.part)).unwrap_or(0);
        let c = if available.is_empty() {
            log.push("no further position to try for the parts around the open connections".into());
            best.router.snapshot("Nudging: no legal movable part near the remaining connections");
            break;
        } else { available.remove(candidate) };
        attempts += 1;
        tried_parts.insert(c.part);
        let key = (c.part, (c.dx * 1000.0).round() as i64, (c.dy * 1000.0).round() as i64);
        tried.insert(key);
        let trial_moves = merge(&moves, c.part, c.dx, c.dy);
        let unit = best.router.unit_mm;
        let label = format!("Nudge {}/{}: {} by {:.2} mm", attempts, cfg.nudge_attempts,
            parts[c.part].name, dist((0.0, 0.0), (c.dx, c.dy)) * unit);
        best.router.snapshot(&label);
        if cfg.verbose { eprintln!("{label}"); }
        let mut trial_cfg = cfg.clone();
        if let Some(cb) = cfg.on_snapshot.clone() {
            trial_cfg.on_snapshot = Some(std::sync::Arc::new(move |mut snap| {
                snap.label = format!("{label}: {}", snap.label);
                cb(snap);
            }));
        }
        if let Some(dir) = &cfg.snapshot_dir {
            trial_cfg.snapshot_dir = Some(dir.join(format!("nudge-{attempts}")));
        }
        let run = run_once(board, parts, &trial_cfg, pour_skip, &trial_moves, Some(&best));
        let better = run.violations <= best.violations
            && (run.fail, run.violations) < (best.fail, best.violations);
        let message = format!(
            "nudge {} by {:.2} mm: open {} -> {}, clearance errors {} -> {}{}",
            parts[c.part].name,
            (c.dx * c.dx + c.dy * c.dy).sqrt() * unit,
            best.fail,
            run.fail,
            best.violations,
            run.violations,
            if better { "  (kept)" } else { "  (undone)" }
        );
        if cfg.verbose { eprintln!("{message}"); }
        log.push(message);
        if better {
            best = run;
            best.router.cfg.on_snapshot = cfg.on_snapshot.clone();
            best.router.cfg.snapshot_dir = cfg.snapshot_dir.clone();
            moves = trial_moves;
        }
        // A rejected trial must not remain visible as if it were the chosen layout.
        best.router.snapshot(if better { "Nudge kept: improved routing" } else { "Nudge undone: restored best routing" });
    }
    best.router.t0 = started;
    Solved { router: best.router, moves, log }
}

struct Candidate {
    part: usize,
    dx: f64,
    dy: f64,
    score: f64,
}

/// Positions to try, best first: small parts near an open connection, moved in eight
/// directions in steps of a quarter of their size, never farther than `nudge_max_widths` part
/// sizes from where they started, never closer than the clearance to anything else.
fn candidates(run: &Run, board: &Board, parts: &[PartSpec], cfg: &Config, moves: &[(usize, f64, f64)]) -> Vec<Candidate> {
    let r = &run.router;
    let unit = r.unit_mm.max(1e-12);
    let mut out: Vec<Candidate> = vec![];
    // Every open net: the stretch between the cut-off piece and the nearest pin of another piece
    // is where something is in the way.
    let mut segs: Vec<(P, P)> = vec![];
    for &(n, pos) in &run.report.open_pos {
        let isl = drc::net_islands(r, n);
        let mine: Vec<P> = isl.iter().filter(|i| i.pins.iter().any(|&k| dist(r.pins[k].c, pos) < 1e-6)).flat_map(|i| i.pins.iter().map(|&k| r.pins[k].c)).collect();
        let rest: Vec<P> = isl.iter().filter(|i| !i.pins.iter().any(|&k| dist(r.pins[k].c, pos) < 1e-6)).flat_map(|i| i.pins.iter().map(|&k| r.pins[k].c)).collect();
        let mut best: Option<(f64, P, P)> = None;
        for &a in &mine {
            for &b in &rest {
                let d = dist(a, b);
                if best.is_none_or(|x| d < x.0) {
                    best = Some((d, a, b));
                }
            }
        }
        match best {
            Some((_, a, b)) => segs.push((a, b)),
            None => segs.push((pos, pos)),
        }
    }
    if segs.is_empty() {
        return out;
    }
    let part_gap = 0.25 / unit;
    // pads of each part at their current positions
    let pin_of = |name: &str| r.pins.iter().position(|p| p.name == name);
    let comp = board.components.first().map(|c| c.reference.clone()).unwrap_or_default();
    let pins_of: Vec<Vec<usize>> = parts
        .iter()
        .map(|p| p.pads.iter().filter_map(|n| pin_of(&format!("{comp}-{n}"))).collect())
        .collect();
    let mut owner_of_pin: Vec<Option<usize>> = vec![None; r.pins.len()];
    for (pi, v) in pins_of.iter().enumerate() {
        for &k in v {
            owner_of_pin[k] = Some(pi);
        }
    }
    // everything a moved pad has to stay clear of
    let others = |me: usize| -> Vec<(Prim, bool)> {
        let mut v: Vec<(Prim, bool)> = vec![];
        for (k, pin) in r.pins.iter().enumerate() {
            if owner_of_pin[k] == Some(me) {
                continue;
            }
            for p in &pin.prims {
                v.push((p.clone(), owner_of_pin[k].is_some()));
            }
        }
        for p in &r.fixed {
            v.push((p.clone(), false));
        }
        for n in &r.nets {
            for p in n.pre.dynp.iter().filter(|p| !p.trace_only) {
                v.push((p.clone(), false));
            }
        }
        v
    };
    for (pi, spec) in parts.iter().enumerate() {
        if !spec.movable || spec.pads.len() > 6 || pins_of[pi].len() != spec.pads.len() {
            continue;
        }
        let prims: Vec<Prim> = pins_of[pi].iter().flat_map(|&k| r.pins[k].prims.iter().cloned()).collect();
        if prims.is_empty() {
            continue;
        }
        // wiring that already touches a pad stays put, so the part does too
        let wired = pins_of[pi].iter().any(|&k| {
            let n = r.pins[k].net;
            n >= 0
                && r.nets[n as usize]
                    .pre
                    .dynp
                    .iter()
                    .filter(|p| !p.trace_only)
                    .any(|w| r.pins[k].prims.iter().any(|p| p.layer == w.layer && p.gap(w) <= 1e-6))
        });
        if wired {
            continue;
        }
        let bb = prims.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |a, p| {
            (a.0.min(p.bb.0), a.1.min(p.bb.1), a.2.max(p.bb.2), a.3.max(p.bb.3))
        });
        let size = (bb.2 - bb.0).max(bb.3 - bb.1);
        if size * unit > 5.0 {
            continue;
        }
        let centre = ((bb.0 + bb.2) / 2.0, (bb.1 + bb.3) / 2.0);
        let near = segs.iter().map(|s| dist_pt_seg(centre, s.0, s.1)).fold(f64::MAX, f64::min);
        let reach = (3.0 * size).max(6.0 / unit);
        if near > reach {
            continue;
        }
        let owns_fail = pins_of[pi].iter().any(|&k| segs.iter().any(|s| dist(s.0, r.pins[k].c) < 1e-6 || dist(s.1, r.pins[k].c) < 1e-6));
        let obstacles = others(pi);
        let cur = total(moves, pi);
        let limit = cfg.nudge_max_widths.clamp(0.0, 3.0) * size;
        let step = (size / 8.0).max(0.1 / unit);
        let nsteps = (limit / step).floor() as i32;
        for s in 1..=nsteps.max(0) {
            for d in 0..8 {
                let a = d as f64 * std::f64::consts::FRAC_PI_4;
                let (dx, dy) = (step * s as f64 * a.cos(), step * s as f64 * a.sin());
                let new_total = (cur.0 + dx, cur.1 + dy);
                if (new_total.0 * new_total.0 + new_total.1 * new_total.1).sqrt() > limit + 1e-9 {
                    continue;
                }
                let moved: Vec<Prim> = prims.iter().map(|p| translate(p, dx, dy)).collect();
                // A large candidate displacement can jump completely over the edge
                // keepout without intersecting it. Reject destinations off the board.
                if moved.iter().any(|p| {
                    let (x0, y0, x1, y1) = p.bb;
                    [(x0, y0), (x1, y0), (x1, y1), (x0, y1)].iter()
                        .any(|&q| !point_in_poly(q, &r.boundary))
                }) { continue; }
                let mut min_gap = f64::MAX;
                let mut legal = true;
                'check: for m in &moved {
                    for (o, is_part) in &obstacles {
                        if o.layer != m.layer
                            || m.bb.0 > o.bb.2 + 4.0 * part_gap + m.clr
                            || o.bb.0 > m.bb.2 + 4.0 * part_gap + m.clr
                            || m.bb.1 > o.bb.3 + 4.0 * part_gap + m.clr
                            || o.bb.1 > m.bb.3 + 4.0 * part_gap + m.clr
                        {
                            continue;
                        }
                        let g = m.gap(o);
                        let need = if *is_part { m.clr.max(o.clr).max(part_gap) } else { m.clr.max(o.clr) };
                        if g < need {
                            legal = false;
                            break 'check;
                        }
                        min_gap = min_gap.min(g);
                    }
                }
                if !legal {
                    continue;
                }
                let new_centre = (centre.0 + dx, centre.1 + dy);
                let away = segs.iter().map(|s| dist_pt_seg(new_centre, s.0, s.1)).fold(f64::MAX, f64::min) - near;
                // more room around the part helps the part itself; moving away helps a neighbour
                let room = min_gap.min(2.0 / unit) * unit;
                let benefit = if owns_fail { room * 2.0 } else { room + away.max(0.0) * unit };
                let score = benefit / (1.0 + dist((0.0, 0.0), (dx, dy)) * unit / 0.25);
                out.push(Candidate { part: pi, dx, dy, score });
            }
        }
    }
    out.sort_by(|a, b| b.score.total_cmp(&a.score));
    out
}
