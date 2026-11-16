//! Grid router: obstacle fields, A* search, net routing, rip-up and refinement.

use crate::geom::*;
use dsn::{AbsShape, Board};
use serde::{Deserialize, Serialize};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;
use std::time::Instant;

const MARGIN: f64 = 0.02;
const SQ2: f32 = std::f32::consts::SQRT_2;
/// A* heuristic inflation: trades a few percent of path length for much less searching.
/// A* heuristic inflation: trades a few percent of path length for much less searching
/// (higher weights were measured: no consistent gain).
fn h_weight() -> f32 {
    1.2
}
fn full_h_weight() -> f32 {
    1.2
}
/// Extra via cost (fraction of a via) per pad whose escape room the via would take.
const CROWD_VIA: f32 = 0.3;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct NetOverride {
    pub diff_group: String,
    pub width: Option<f64>,
    pub clearance: Option<f64>,
    /// leave this net unrouted
    pub skip: bool,
    /// via padstack to use for this net (one of the DSN's via names)
    pub via: Option<String>,
    /// vias placed side by side at every layer change (power nets); None/1 = single via
    pub via_count: Option<usize>,
}

/// Global design-rule floors (millimetres). Anything the DSN or the user asks for below these
/// values is raised to them. Defaults: JLCPCB 4-layer, economic service.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct DrcRules {
    pub enabled: bool,
    pub min_trace_mm: f64,
    pub min_clearance_mm: f64,
    pub min_via_diameter_mm: f64,
    pub min_via_drill_mm: f64,
    /// copper to board edge
    pub min_edge_clearance_mm: f64,
}

impl Default for DrcRules {
    fn default() -> Self {
        DrcRules {
            enabled: true,
            min_trace_mm: 0.09,
            min_clearance_mm: 0.09,
            min_via_diameter_mm: 0.45,
            min_via_drill_mm: 0.20,
            min_edge_clearance_mm: 0.30,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum LayerRole {
    /// traces are routed here; assigned nets become a pour in the free space afterwards
    #[default]
    Signal,
    /// no traces; assigned nets become planes that pins reach through vias
    Plane,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct LayerPlan {
    pub role: LayerRole,
    pub nets: Vec<String>,
}

pub type SnapshotFn = std::sync::Arc<dyn Fn(crate::snapshot::Snapshot) + Send + Sync>;

#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// layer names on which no traces may be routed
    pub exclusive: Vec<String>,
    /// planes (as NET@LAYER) in which no traces may be routed
    pub exclusive_planes: Vec<String>,
    pub net_overrides: HashMap<String, NetOverride>,
    /// cost of a via, in grid cells of trace length
    pub via_cost: f32,
    pub max_rip_events: usize,
    pub refine_passes: usize,
    #[serde(skip)]
    pub snapshot_dir: Option<PathBuf>,
    /// write a snapshot every N routed nets (0 = only at phase ends)
    pub snapshot_every: usize,
    pub pair_gap: Option<f64>,
    /// global design-rule minimums
    pub rules: DrcRules,
    /// layer name -> role and assigned plane/pour nets
    pub layers: HashMap<String, LayerPlan>,
    /// millimetres per DSN unit; None = detect from the DSN
    pub unit_mm: Option<f64>,
    /// max new vias per bridge when a plane/net is split into islands (0 = no bridging)
    pub bridge_vias: usize,
    /// multi-net plane layers: copper farther than this (mm) from the owning net's pins becomes GND
    pub plane_reach_mm: f64,
    /// shrink plane regions to where they are used (remainder = GND)
    pub plane_refine: bool,
    /// via cost factor while connecting a pad to a plane (small = one via next to every pad)
    pub plane_via_factor: f32,
    /// also write generated plane / pour copper into the SES (large; most CAD tools choke on it)
    pub write_fill: bool,
    /// Application mode: real routed traces only; layer nets are routing preferences.
    pub trace_routing: bool,
    /// Clean-board retries (failing nets first) after the local repairs left connections
    /// open; at least one is always allowed, at most three.
    pub restart_attempts: usize,
    /// write plane / pour copper into the SES as polygons (EasyEDA imports them as solid regions)
    pub write_polygons: bool,
    /// edge length (mm) of the building blocks of the compact plane export
    pub plane_block_mm: f64,
    /// last resort for open nets: short trace jumps on plane / no-routing layers
    pub allow_plane_jumps: bool,
    /// last resort for open nets: a via inside a pad of the net itself
    pub last_resort_pad_vias: bool,
    /// longest such jump per connection (mm)
    pub plane_jump_mm: f64,
    #[serde(skip)]
    pub verbose: bool,
    pub grid: Option<f64>,
    pub pairs: bool,
    pub allow_vias_in_pads: bool,
    pub allow_neckdown: bool,
    pub add_gnd_planes: bool,
    pub protect_planes: bool,
    pub pair_tolerance: f64,
    pub group_tolerances: HashMap<String, f64>,
    /// extra cost multiplier for each grid step a trace spends inside a foreign plane (cuts it)
    pub plane_penalty: f32,
    pub plane_repair_rounds: usize,
    /// also write the pre-existing wiring into the SES
    pub include_existing: bool,
    /// Stand-in for copper pours: nets with a layer assignment or a width override are widened
    /// beyond their width wherever there is room, up to this many millimetres (0 = off).
    pub power_width_max_mm: f64,
    /// Draw the outline of the area a pour of each layer-assigned net would cover as a thin
    /// closed trace, to be re-traced as a copper area in the CAD.
    pub pour_outlines: bool,
    /// Copper areas are available (EasyEDA JSON): layer-assigned nets get pours on trace layers
    /// and plane layers are filled. Few, large polygons: dust is dropped.
    pub pour_polygons: bool,
    /// Copper areas reach at most this far (mm) from copper of their own net.
    pub pour_reach_mm: f64,
    /// EasyEDA JSON: after everything else, fill the empty space of every layer with a copper
    /// area of `fill_net` and tie it to the other layers' areas of that net with stitching vias.
    pub fill_empty: bool,
    /// Net used by `fill_empty` (default GND).
    pub fill_net: String,
    /// Last resort for connections that stay open: nudge small passives (EasyEDA JSON only) and
    /// route again. Off by default.
    pub nudge_parts: bool,
    /// How far a part may end up from where it started, in part sizes (at most 3).
    pub nudge_max_widths: f64,
    /// How many nudged layouts are routed at most.
    pub nudge_attempts: usize,
    /// Distance (mm) between the stitching vias of overlapping fill areas (0 = no vias).
    pub stitch_pitch_mm: f64,
    /// The CAD keeps the width of imported tracks. EasyEDA does not (every track gets its rule
    /// width), so by default traces are written as plain centre lines with as few segments as
    /// possible and power traces are not widened.
    pub keep_track_widths: bool,
    #[serde(skip)]
    pub cancel: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    #[serde(skip)]
    pub on_snapshot: Option<SnapshotFn>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            exclusive: vec![],
            exclusive_planes: vec![],
            net_overrides: HashMap::new(),
            via_cost: 60.0,
            max_rip_events: 600,
            refine_passes: 2,
            snapshot_dir: None,
            snapshot_every: 1,
            pair_gap: None,
            rules: DrcRules::default(),
            layers: HashMap::new(),
            unit_mm: None,
            bridge_vias: 2,
            plane_reach_mm: 6.0,
            plane_refine: true,
            plane_via_factor: 0.25,
            write_fill: false,
            trace_routing: false,
            restart_attempts: 0,
            write_polygons: true,
            plane_block_mm: 0.6,
            allow_plane_jumps: true,
            last_resort_pad_vias: true,
            plane_jump_mm: 5.0,
            verbose: true,
            grid: None,
            pairs: true,
            allow_vias_in_pads: false,
            allow_neckdown: true,
            add_gnd_planes: true,
            protect_planes: true,
            pair_tolerance: 0.1,
            group_tolerances: HashMap::new(),
            plane_penalty: 3.0,
            plane_repair_rounds: 4,
            include_existing: false,
            power_width_max_mm: 5.0,
            pour_outlines: false,
            pour_polygons: false,
            pour_reach_mm: 4.0,
            fill_empty: true,
            fill_net: "GND".into(),
            nudge_parts: false,
            nudge_max_widths: 3.0,
            nudge_attempts: 6,
            stitch_pitch_mm: 4.0,
            keep_track_widths: false,
            cancel: None,
            on_snapshot: None,
        }
    }
}

impl Config {
    pub fn use_trace_routing(&mut self) {
        self.trace_routing = true;
        self.write_fill = false;
        self.add_gnd_planes = false;
        // With copper areas available (EasyEDA JSON) layers can be planes again; their polygons
        // come from `make_pour_polygons`, not from the plane fill.
        self.write_polygons = false;
        if !self.pour_polygons {
            for plan in self.layers.values_mut() { plan.role = LayerRole::Signal; }
        }
    }

    pub fn tolerance_for(&self, name: &str) -> f64 {
        self.net_overrides
            .get(name)
            .and_then(|o| self.group_tolerances.get(o.diff_group.trim()))
            .copied()
            .unwrap_or(self.pair_tolerance)
    }
    /// Routing grid: the user's value, else a quarter of the track pitch (width + clearance) so
    /// neighbouring tracks pack exactly, coarsened until the board fits a sane number of cells.
    pub fn grid_for(&self, board: &Board) -> f64 {
        if let Some(g) = self.grid {
            return g;
        }
        let dsn = if board.grid_wire > 0.0 { board.grid_wire } else { 0.25 };
        let unit_mm = self.unit_mm.unwrap_or_else(|| board.detect_unit_mm().0);
        let floor = |v: f64, m: f64| {
            if self.rules.enabled && unit_mm > 0.0 {
                v.max(m / unit_mm)
            } else {
                v
            }
        };
        let w = floor(board.rule_width, self.rules.min_trace_mm);
        let c = floor(board.rule_clear, self.rules.min_clearance_mm);
        if w <= 0.0 || c <= 0.0 {
            return dsn;
        }
        // The obstacle test keeps MARGIN on top of the clearance. Without it in the pitch,
        // tracks four cells apart miss by a hair and every lane costs five cells.
        let pitch = w + c + MARGIN + 1e-4;
        let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
        for &(x, y) in &board.boundary {
            lo = (lo.0.min(x), lo.1.min(y));
            hi = (hi.0.max(x), hi.1.max(y));
        }
        let area = ((hi.0 - lo.0) * (hi.1 - lo.1)).max(0.0) * board.layers.len().max(1) as f64;
        for k in [4.0, 3.0, 2.0] {
            let g = pitch / k;
            if area / (g * g) <= 4_000_000.0 || k == 2.0 {
                return g.max(dsn.min(g));
            }
        }
        dsn
    }

    pub fn validate(&self, board: &Board) -> Result<(), String> {
        if self
            .group_tolerances
            .values()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err("Group tolerances must be finite and nonnegative".into());
        }
        if !self.via_cost.is_finite()
            || self.via_cost < 0.0
            || !self.pair_tolerance.is_finite()
            || self.pair_tolerance < 0.0
        {
            return Err("Via cost and pair tolerance must be finite and nonnegative".into());
        }
        let grid = self.grid_for(board);
        if !grid.is_finite() || grid <= 0.0 {
            return Err("Grid must be positive".into());
        }
        let mut groups: HashMap<&str, usize> = HashMap::new();
        for (name, ov) in &self.net_overrides {
            if ov.width.is_some_and(|v| !v.is_finite() || v <= 0.0)
                || ov.clearance.is_some_and(|v| !v.is_finite() || v < 0.0)
            {
                return Err(format!("Invalid width or clearance for {name}"));
            }
            if !ov.diff_group.trim().is_empty() && ov.diff_group.trim() != "-" {
                if !board.nets.iter().any(|n| n.name == *name) {
                    return Err(format!("Unknown grouped net: {name}"));
                }
                *groups.entry(ov.diff_group.trim()).or_default() += 1;
            }
        }
        if let Some((name, _)) = groups.iter().find(|(_, count)| **count < 2) {
            return Err(format!("Group {name} needs at least two nets"));
        }
        for (ln, plan) in &self.layers {
            if !board.layers.iter().any(|l| &l.name == ln) {
                return Err(format!("Unknown layer in layer plan: {ln}"));
            }
            if let Some(n) = plan
                .nets
                .iter()
                .find(|n| !board.nets.iter().any(|b| &b.name == *n))
            {
                return Err(format!("Unknown net {n} assigned to layer {ln}"));
            }
        }
        let r = &self.rules;
        if [
            r.min_trace_mm,
            r.min_clearance_mm,
            r.min_via_diameter_mm,
            r.min_via_drill_mm,
            r.min_edge_clearance_mm,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err("Design-rule minimums must be finite and nonnegative".into());
        }
        let (mut lo, mut hi) = (
            (f64::INFINITY, f64::INFINITY),
            (f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for &(x, y) in &board.boundary {
            if !x.is_finite() || !y.is_finite() {
                return Err("Invalid board boundary".into());
            }
            lo = (lo.0.min(x), lo.1.min(y));
            hi = (hi.0.max(x), hi.1.max(y));
        }
        let cells =
            ((hi.0 - lo.0) / grid + 1.0) * ((hi.1 - lo.1) / grid + 1.0) * board.layers.len() as f64;
        if !cells.is_finite() || cells > 80_000_000.0 {
            return Err("Grid is too fine for this board; increase grid spacing".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct Grid {
    pub x0: f64,
    pub y0: f64,
    pub g: f64,
    pub nx: usize,
    pub ny: usize,
    pub nl: usize,
}

impl Grid {
    #[inline]
    pub fn n(&self) -> usize {
        self.nx * self.ny
    }
    #[inline]
    pub fn center(&self, x: i32, y: i32) -> P {
        (self.x0 + x as f64 * self.g, self.y0 + y as f64 * self.g)
    }
    #[inline]
    pub fn cell(&self, p: P) -> (i32, i32) {
        (
            ((p.0 - self.x0) / self.g).round() as i32,
            ((p.1 - self.y0) / self.g).round() as i32,
        )
    }
    #[inline]
    pub fn idx(&self, l: usize, x: i32, y: i32) -> usize {
        l * self.n() + y as usize * self.nx + x as usize
    }
}

/// Obstacle counts for one probe class (clearance, probe half width).
#[derive(Clone)]
pub struct Field {
    pub clr: f64,
    pub p: f64,
    /// probe class for vias (ignores trace-only blockers such as exclusive planes)
    pub via: bool,
    /// low 16 bits: fixed-copper stamp count, high 16 bits: routed (dynamic) copper count
    pub cnt: Vec<u32>,
    /// one byte per cell derived from `cnt` (bit 0 fixed, bit 1 routed): what searches read
    mask: Vec<u8>,
    /// via fields: `cnt` summed over all layers per (x, y), so a via test is one lookup
    col: Vec<u32>,
}

impl Field {
    pub(crate) fn new(grid: &Grid, clr: f64, p: f64, via: bool) -> Field {
        let n = grid.n() * grid.nl;
        Field {
            clr,
            p,
            via,
            cnt: vec![0; n],
            mask: vec![0; n],
            col: vec![0; if via { grid.n() } else { 0 }],
        }
    }
    #[inline(always)]
    pub fn fx(&self, i: usize) -> bool {
        self.mask[i] & 1 != 0
    }
    #[inline(always)]
    pub fn dy(&self, i: usize) -> bool {
        self.mask[i] & 2 != 0
    }
    /// (blocked by fixed copper, blocked by routed copper) for a through via at column `c`
    #[inline(always)]
    pub fn via_col(&self, c: usize) -> (bool, bool) {
        let v = self.col[c];
        (v & 0xFFFF != 0, v >> 16 != 0)
    }
    pub fn threshold(&self, pr: &Prim) -> f64 {
        self.clr.max(pr.clr) + self.p + MARGIN
    }
    pub(crate) fn stamp(&mut self, g: &Grid, pr: &Prim, delta: i32, dynamic: bool) {
        if pr.trace_only && self.via {
            return;
        }
        let t = self.threshold(pr);
        let x0 = (((pr.bb.0 - t - g.x0) / g.g).floor() as i32).max(0);
        let x1 = (((pr.bb.2 + t - g.x0) / g.g).ceil() as i32).min(g.nx as i32 - 1);
        let y0 = (((pr.bb.1 - t - g.y0) / g.g).floor() as i32).max(0);
        let y1 = (((pr.bb.3 + t - g.y0) / g.g).ceil() as i32).min(g.ny as i32 - 1);
        let arr = &mut self.cnt;
        let mask = &mut self.mask;
        let col = &mut self.col;
        let via = self.via;
        let unit: i64 = if dynamic { 65536 } else { 1 };
        let off = pr.layer * g.n();
        // Clip diagonal capsules to each row before testing cells. A long diagonal
        // otherwise scans its entire rectangular bounding box on every rip-up.
        let segment = match &pr.geo {
            Geo::Seg { a, b, hw } => Some((*a, *b, hw + t)),
            _ => None,
        };
        for y in y0..=y1 {
            let mut lo = x0;
            let mut hi = x1;
            if let Some((a, b, radius)) = segment {
                let dy = b.1 - a.1;
                if dy.abs() > 1e-12 {
                    let cy = g.center(0, y).1;
                    let u = (cy - radius - a.1) / dy;
                    let v = (cy + radius - a.1) / dy;
                    let start = u.min(v).clamp(0.0, 1.0);
                    let end = u.max(v).clamp(0.0, 1.0);
                    let xa = a.0 + start * (b.0 - a.0);
                    let xb = a.0 + end * (b.0 - a.0);
                    lo = lo.max(((xa.min(xb) - radius - g.x0) / g.g).floor() as i32);
                    hi = hi.min(((xa.max(xb) + radius - g.x0) / g.g).ceil() as i32);
                }
            }
            for x in lo..=hi {
                if pr.dist(g.center(x, y)) < t {
                    let i = off + y as usize * g.nx + x as usize;
                    let v = (arr[i] as i64 + delta as i64 * unit) as u32;
                    arr[i] = v;
                    mask[i] = (v & 0xFFFF != 0) as u8 | (((v >> 16 != 0) as u8) << 1);
                    if via {
                        let c = i - off;
                        col[c] = (col[c] as i64 + delta as i64 * unit) as u32;
                    }
                }
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Wire {
    pub layer: usize,
    pub width: f64,
    pub pts: Vec<P>,
    /// stroke of generated plane/pour copper (not a routed trace)
    #[serde(default)]
    pub fill: bool,
}

pub type Cell = (usize, i32, i32);

#[derive(Clone, Default)]
pub struct Routed {
    pub wires: Vec<Wire>,
    pub vias: Vec<P>,
    pub tree: Vec<Cell>,
    pub dynp: Vec<Prim>,
}

#[derive(Clone)]
pub struct NetState {
    pub name: String,
    pub pins: Vec<usize>,
    pub width: f64,
    pub clr: f64,
    pub via: String,
    pub via_r: f64,
    pub tf: usize,
    pub narrow_tf: usize,
    pub vf: usize,
    pub r: Routed,
    pub unconnected: Vec<usize>,
    pub pair: Option<usize>,
    /// index into Router::bundles for groups of three or more nets
    pub bundle: Option<usize>,
    pub rip_count: u32,
    pub coupled: bool,
    /// routed as a group follower (kept parallel to its lead) instead of a coupled swath
    pub guided: bool,
    pub ptf: usize,
    pub pvf: usize,
    /// copper that already existed in the DSN (never removed; stamped as fixed)
    pub pre: Routed,
    /// electrically separate groups of pins (after existing wiring / planes); comps[0] is the start
    pub comps: Vec<Vec<usize>>,
    pub comp_cells: Vec<Vec<Cell>>,
    pub skip: bool,
    /// index into Router::planes when this net owns a plane its pins are merged into
    pub plane: Option<usize>,
    pub plane_mul: f32,
}

/// Cells close to the already-routed lead net of a differential group are cheap.
#[derive(Clone)]
pub struct Guide {
    pub near: Vec<bool>,
    pub bonus: f32,
    pub penalty: f32,
}

#[derive(Clone)]
pub struct PinInfo {
    pub name: String,
    pub net: i32,
    pub c: P,
    pub prims: Vec<Prim>,
    pub cells: Vec<Cell>,
}

#[derive(Default, Clone)]
pub struct SearchBuf {
    g: Vec<f32>,
    par: Vec<u32>,
    touched: Vec<u32>,
    tmark: Vec<u32>,
    gen: u32,
    heap: BinaryHeap<Reverse<u64>>,
}

pub struct Router {
    pub ground_fill_length: f64,
    pub cfg: Config,
    pub grid: Grid,
    pub layer_names: Vec<String>,
    pub excl: Vec<bool>,
    pub fields: Vec<Field>,
    congestion: Vec<u16>,
    pub nets: Vec<NetState>,
    pub pins: Vec<PinInfo>,
    pub fixed: Vec<Prim>,
    pub term: HashMap<Cell, usize>,
    pub res_unit: String,
    pub res: f64,
    /// millimetres per DSN coordinate unit
    pub unit_mm: f64,
    /// global minimum trace width (DSN units); 0 when the DRC floor is disabled
    pub min_width: f64,
    /// attraction map used while routing the follower of a differential group
    pub guide: Option<Guide>,
    pub via_names: Vec<String>,
    pub board_name: String,
    buf: SearchBuf,
    pub snap_count: usize,
    pub t0: Instant,
    pub stats_searches: usize,
    pub t_astar: f32,
    /// seconds per search kind: window, full, rip-up, thin trace
    pub t_cat: [f32; 4],
    /// abort a search after this many expansions (0 = unlimited)
    pub pop_budget: usize,
    /// last-resort mode: traces may use plane / no-routing layers at a high cost
    pub jump_mode: bool,
    /// last-resort mode: the failing net may put vias inside its own pads
    pub pad_via_mode: bool,
    /// A* heuristic weight (1 = exact shortest path; larger = faster, a little longer)
    pub h_weight: f32,
    /// layer changes that could not get all the vias requested for their net
    pub via_shortfall: usize,
    /// connections completed with a jump on a plane / no-routing layer
    pub jumps_used: usize,
    /// connections that needed a via inside a pad
    pub pad_vias_used: usize,
    /// (layer, net, plane) of every copper fill that was generated
    pub fill_done: Vec<(usize, usize, Option<usize>)>,
    /// compact wide-track version of the plane copper, per net (what the SES contains)
    pub fill_export: Vec<(usize, Wire)>,
    /// exact plane / pour copper polygons (net, layer, ring with slits around obstacles)
    pub fill_polys: Vec<(usize, usize, Vec<P>)>,
    /// (pour outlines drawn, outlines dropped because they could not be drawn cleanly)
    pub outline_stats: (usize, usize),
    /// (net name, layer name) pairs that must not get a generated pour (the document has one)
    pub pour_skip: Vec<(String, String)>,
    /// copper-area problems worth telling the user about (a plane net that stays in pieces)
    pub pour_warnings: Vec<String>,
    /// parts nudged by `nudge::solve`: (part index, dx, dy) in DSN units
    pub part_moves: Vec<(usize, f64, f64)>,
    /// per search: (seconds, expansions, soft, window cells, found)
    pub prof: Vec<(f32, u32, bool, u32, bool)>,
    pub pops_total: u64,
    /// worker copies of this router used for speculative parallel routing
    workers: Vec<Router>,
    /// speculative routing: (attempted, used, not fully connected, conflicted)
    pub spec_stats: [usize; 4],
    /// seconds: wall time in speculate(), worker sync, worker routing (sum over workers)
    pub spec_time: [f32; 3],
    pub t_reach: f32,
    pub t_stamp: f32,
    pub n_reach: usize,
    pub rip_events: usize,
    pub boundary: Vec<P>,
    pub planes: Vec<crate::plane::PlaneInfo>,
    pub bundles: Vec<Vec<usize>>,
    /// per layer: 0 = no plane, else plane index + 1 (empty vec when the layer has none)
    pub plane_id: Vec<Vec<u8>>,
    pub cur_net: usize,
    pub cur_land: bool,
    /// position of every net in the initial routing order (most constrained first)
    route_rank: Vec<usize>,
    /// per (x, y): number of pads close enough that copper here narrows their escape
    crowd: Vec<u8>,
}

#[derive(Clone, Copy, PartialEq)]
enum Blk {
    Free,
    Soft,
    Hard,
}

pub(crate) fn via_radius(board: &Board, name: &str) -> f64 {
    let mut r: f64 = 0.0;
    if let Some(ps) = board.padstacks.get(name) {
        for s in &ps.shapes {
            match s {
                dsn::Shape::Circle { d, .. } => r = r.max(d / 2.0),
                dsn::Shape::Rect { x1, y1, x2, y2, .. } => {
                    r = r.max(((x2 - x1).max(y2 - y1)) / 2.0)
                }
                dsn::Shape::Polygon { pts, .. } | dsn::Shape::Path { pts, .. } => {
                    for &(x, y) in pts {
                        r = r.max((x * x + y * y).sqrt());
                    }
                }
            }
        }
    }
    if r == 0.0 {
        1.0
    } else {
        r
    }
}

impl Router {
    pub fn new(board: &Board, cfg: Config) -> Router {
        Self::new_with(board, cfg, false)
    }

    /// `light`: analyse only (pins, existing copper, planes, connectivity) without allocating
    /// the routing grid fields; used for the GUI preview.
    pub fn new_with(board: &Board, cfg: Config, light: bool) -> Router {
        let nl = board.layers.len();
        let g = cfg.grid_for(board);
        let (mut bx0, mut by0, mut bx1, mut by1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for &(x, y) in &board.boundary {
            bx0 = bx0.min(x);
            by0 = by0.min(y);
            bx1 = bx1.max(x);
            by1 = by1.max(y);
        }
        let grid = Grid {
            x0: bx0,
            y0: by0,
            g,
            nx: ((bx1 - bx0) / g).floor() as usize + 1,
            ny: ((by1 - by0) / g).floor() as usize + 1,
            nl,
        };
        let layer_names: Vec<String> = board.layers.iter().map(|l| l.name.clone()).collect();
        let excl: Vec<bool> = board
            .layers
            .iter()
            .map(|l| {
                l.power
                    || cfg.exclusive.iter().any(|e| e == &l.name)
                    || cfg
                        .layers
                        .get(&l.name)
                        .is_some_and(|p| p.role == LayerRole::Plane)
            })
            .collect();
        let unit_mm = cfg.unit_mm.unwrap_or_else(|| board.detect_unit_mm().0);
        let (min_w, min_c, min_via_r, min_edge) = if cfg.rules.enabled && unit_mm > 0.0 {
            (
                cfg.rules.min_trace_mm / unit_mm,
                cfg.rules.min_clearance_mm / unit_mm,
                cfg.rules.min_via_diameter_mm / unit_mm / 2.0,
                cfg.rules.min_edge_clearance_mm / unit_mm,
            )
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // nets
        let mut net_id: HashMap<String, usize> = HashMap::new();
        let mut nets: Vec<NetState> = vec![];
        let mut pin_net: HashMap<String, i32> = HashMap::new();
        let default_w = if board.rule_width > 0.0 {
            board.rule_width
        } else {
            g * 4.0
        }
        .max(min_w);
        let default_c = if board.rule_clear > 0.0 {
            board.rule_clear
        } else {
            g * 2.0
        }
        .max(min_c);
        for (i, n) in board.nets.iter().enumerate() {
            net_id.insert(n.name.clone(), i);
            let cl = board
                .classes
                .iter()
                .find(|c| c.nets.iter().any(|x| x == &n.name));
            let ov = cfg.net_overrides.get(&n.name);
            let width = ov
                .and_then(|o| o.width)
                .or(cl.and_then(|c| c.width))
                .unwrap_or(default_w)
                .max(min_w);
            let clr = ov
                .and_then(|o| o.clearance)
                .or(cl.and_then(|c| c.clearance))
                .unwrap_or(default_c)
                .max(min_c);
            let via = ov
                .and_then(|o| o.via.clone())
                .filter(|v| board.via_names.contains(v) || board.padstacks.contains_key(v))
                .or_else(|| cl.and_then(|c| c.vias.first().cloned()))
                .or_else(|| board.via_names.first().cloned())
                .unwrap_or_default();
            let via_r = via_radius(board, &via).max(min_via_r);
            for p in &n.pins {
                pin_net.insert(format!("{}-{}", p.comp, p.pin), i as i32);
            }
            nets.push(NetState {
                name: n.name.clone(),
                pins: vec![],
                width,
                clr,
                via,
                via_r,
                tf: 0,
                narrow_tf: 0,
                vf: 0,
                r: Routed::default(),
                unconnected: vec![],
                pair: None,
                bundle: None,
                rip_count: 0,
                coupled: false,
                guided: false,
                ptf: 0,
                pvf: 0,
                pre: Routed::default(),
                comps: vec![],
                comp_cells: vec![],
                skip: cfg.net_overrides.get(&n.name).map_or(false, |o| o.skip),
                plane: None,
                plane_mul: 1.0,
            });
        }

        // pins
        let abs = board.abs_pins();
        let mut keys: Vec<&String> = abs.keys().collect();
        keys.sort();
        let mut pins: Vec<PinInfo> = vec![];
        let mut term: HashMap<Cell, usize> = HashMap::new();
        for k in keys {
            let ap = &abs[k];
            let net = pin_net.get(k).copied().unwrap_or(-1);
            let clr = if net >= 0 {
                nets[net as usize].clr
            } else {
                default_c
            };
            let mut prims = vec![];
            let mut cells: Vec<Cell> = vec![];
            for (l, sh) in &ap.shapes {
                let (geo, inside): (Geo, Box<dyn Fn(P) -> bool>) = match sh {
                    AbsShape::Circle { cx, cy, r } => {
                        let (cx, cy, r) = (*cx, *cy, *r);
                        (
                            Geo::Circle { c: (cx, cy), r },
                            Box::new(move |p: P| dist(p, (cx, cy)) <= r),
                        )
                    }
                    AbsShape::Poly(pts) => {
                        let pts2 = pts.clone();
                        (
                            Geo::Poly(pts.clone()),
                            Box::new(move |p: P| point_in_poly(p, &pts2)),
                        )
                    }
                };
                let pr = Prim::new(*l, geo, net, clr);
                // terminal cells: cell centres inside the pad
                let x0 = (((pr.bb.0 - grid.x0) / g).floor() as i32).max(0);
                let x1 = (((pr.bb.2 - grid.x0) / g).ceil() as i32).min(grid.nx as i32 - 1);
                let y0 = (((pr.bb.1 - grid.y0) / g).floor() as i32).max(0);
                let y1 = (((pr.bb.3 - grid.y0) / g).ceil() as i32).min(grid.ny as i32 - 1);
                let mut found = false;
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        if inside(grid.center(x, y)) {
                            cells.push((*l, x, y));
                            found = true;
                        }
                    }
                }
                if !found {
                    let (x, y) = grid.cell(ap.x_y_of(&pr));
                    cells.push((
                        *l,
                        x.clamp(0, grid.nx as i32 - 1),
                        y.clamp(0, grid.ny as i32 - 1),
                    ));
                }
                prims.push(pr);
            }
            cells.sort();
            cells.dedup();
            let pi = pins.len();
            if net >= 0 {
                nets[net as usize].pins.push(pi);
                for &c in &cells {
                    term.insert(c, pi);
                }
            }
            pins.push(PinInfo {
                name: k.clone(),
                net,
                c: (ap.x, ap.y),
                prims,
                cells,
            });
        }

        // fixed obstacles: outline, keepouts
        let mut fixed: Vec<Prim> = vec![];
        for l in 0..nl {
            let b = &board.boundary;
            for i in 0..b.len() {
                let (a, c) = (b[i], b[(i + 1) % b.len()]);
                fixed.push(Prim::new(
                    l,
                    Geo::Seg { a, b: c, hw: 0.0 },
                    -2,
                    default_c.max(min_edge),
                ));
            }
        }
        for k in &board.keepouts {
            let layers: Vec<usize> = match &k.layer {
                None => (0..nl).collect(),
                Some(n) => board.layer_index(n).into_iter().collect(),
            };
            for l in layers {
                let geo = match &k.shape {
                    dsn::Shape::Circle { d, cx, cy, .. } => Geo::Circle {
                        c: (*cx, *cy),
                        r: d / 2.0,
                    },
                    dsn::Shape::Rect { x1, y1, x2, y2, .. } => {
                        Geo::Poly(vec![(*x1, *y1), (*x2, *y1), (*x2, *y2), (*x1, *y2)])
                    }
                    dsn::Shape::Polygon { pts, .. } => Geo::Poly(pts.clone()),
                    dsn::Shape::Path { pts, aperture, .. } if pts.len() >= 2 => {
                        for w in pts.windows(2) {
                            fixed.push(Prim::new(
                                l,
                                Geo::Seg {
                                    a: w[0],
                                    b: w[1],
                                    hw: aperture / 2.0,
                                },
                                -2,
                                0.0,
                            ));
                        }
                        continue;
                    }
                    _ => continue,
                };
                fixed.push(Prim::new(l, geo, -2, 0.0));
            }
        }

        // fields per distinct probe class
        let mut fields: Vec<Field> = vec![];
        let fgrid = if light {
            Grid {
                nx: 0,
                ny: 0,
                ..grid.clone()
            }
        } else {
            grid.clone()
        };
        let mut keyed: HashMap<(i64, i64, bool), usize> = HashMap::new();
        let mut get_field = |clr: f64, p: f64, via: bool, fields: &mut Vec<Field>| -> usize {
            let k = ((clr * 1e5).round() as i64, (p * 1e5).round() as i64, via);
            *keyed.entry(k).or_insert_with(|| {
                fields.push(Field::new(&fgrid, clr, p, via));
                fields.len() - 1
            })
        };
        for n in nets.iter_mut() {
            n.tf = get_field(n.clr, n.width / 2.0, false, &mut fields);
            n.narrow_tf = get_field(n.clr, n.width.min(min_w.max(grid.g * 0.05)) / 2.0, false, &mut fields);
            n.vf = get_field(n.clr, n.via_r, true, &mut fields);
        }
        let mut pair_list = dsn::find_diff_pairs(&board.nets);
        pair_list.retain(|&(a, b)| {
            [a, b].iter().all(|&i| {
                cfg.net_overrides
                    .get(&board.nets[i].name)
                    .is_none_or(|o| o.diff_group.trim() != "-")
            })
        });
        let mut groups: std::collections::BTreeMap<String, Vec<usize>> =
            std::collections::BTreeMap::new();
        for (i, n) in board.nets.iter().enumerate() {
            if let Some(o) = cfg.net_overrides.get(&n.name) {
                if !o.diff_group.trim().is_empty() && o.diff_group.trim() != "-" {
                    groups
                        .entry(o.diff_group.trim().into())
                        .or_default()
                        .push(i);
                }
            }
        }
        for members in groups.values() {
            pair_list.retain(|(a, b)| !members.contains(a) && !members.contains(b));
        }
        let mut bundles: Vec<Vec<usize>> = vec![];
        for members in groups.values() {
            if members.len() == 2 {
                pair_list.push((members[0], members[1]));
            } else if members.len() > 2 {
                bundles.push(members.clone());
            }
        }
        for &(p, q) in &pair_list {
            let (w, clr, vr) = (
                nets[p].width.max(nets[q].width),
                nets[p].clr.max(nets[q].clr),
                nets[p].via_r.max(nets[q].via_r),
            );
            let gap = cfg.pair_gap.unwrap_or(clr);
            let ps = (w + gap / 2.0) + 0.1 * w;
            let dv = 2.0 * vr + clr + 0.05;
            let pf = get_field(clr, ps, false, &mut fields);
            let pv = get_field(clr, dv / 2.0 + vr, true, &mut fields);
            for &x in &[p, q] {
                nets[x].ptf = pf;
                nets[x].pvf = pv;
            }
        }
        let mut r = Router {
            ground_fill_length: 0.0,
            cfg,
            grid,
            layer_names,
            excl,
            congestion: vec![0; if light { 0 } else { fgrid.n() * fgrid.nl }],
            fields,
            nets,
            pins,
            fixed,
            term,
            res_unit: board.res_unit.clone(),
            res: board.res,
            unit_mm,
            min_width: min_w,
            guide: None,
            via_names: board.via_names.clone(),
            board_name: board.name.clone(),
            buf: SearchBuf::default(),
            snap_count: 0,
            t0: Instant::now(),
            stats_searches: 0,
            t_astar: 0.0,
            t_cat: [0.0; 4],
            pop_budget: 0,
            jump_mode: false,
            pad_via_mode: false,
            h_weight: h_weight(),
            via_shortfall: 0,
            jumps_used: 0,
            pad_vias_used: 0,
            fill_done: vec![],
            fill_export: vec![],
            fill_polys: vec![],
            outline_stats: (0, 0),
            pour_skip: vec![],
            pour_warnings: vec![],
            part_moves: vec![],
            prof: vec![],
            pops_total: 0,
            workers: vec![],
            spec_stats: [0; 4],
            spec_time: [0.0; 3],
            t_reach: 0.0,
            t_stamp: 0.0,
            n_reach: 0,
            rip_events: 0,
            boundary: board.boundary.clone(),
            planes: vec![],
            bundles: vec![],
            plane_id: vec![vec![]; nl],
            cur_net: usize::MAX,
            cur_land: false,
            route_rank: vec![],
            crowd: vec![],
        };
        r.ingest_existing(board);
        r.build_layer_planes(board);
        // stamp all fixed copper (pads, outline, keepouts, pre-existing wiring and vias)
        let t = Instant::now();
        let mut all_fixed: Vec<Prim> = r.fixed.clone();
        for p in &r.pins {
            all_fixed.extend(p.prims.iter().cloned());
        }
        for n in &r.nets {
            all_fixed.extend(n.pre.dynp.iter().cloned());
        }
        for pr in all_fixed.iter().filter(|_| !light) {
            for f in r.fields.iter_mut() {
                f.stamp(&r.grid, pr, 1, false);
            }
        }
        if !r.cfg.allow_vias_in_pads && !light {
            // Independent fixed masks: own-net unstamping must not remove this restriction.
            for pin in &r.pins {
                for pr in &pin.prims {
                    for f in r.fields.iter_mut().filter(|f| f.via) {
                        f.stamp(&r.grid, pr, 1, false);
                    }
                }
            }
        }
        r.init_components();
        if !light && r.cfg.trace_routing {
            r.build_crowd();
        }
        if r.cfg.verbose {
            eprintln!(
                "grid {}x{}x{} (g={}), {} nets, {} pins, {} fields, stamped fixed copper in {:.2}s",
                r.grid.nx,
                r.grid.ny,
                r.grid.nl,
                r.grid.g,
                r.nets.len(),
                r.pins.len(),
                r.fields.len(),
                t.elapsed().as_secs_f32()
            );
        }
        for (p, n) in pair_list {
            r.nets[p].pair = Some(n);
            r.nets[n].pair = Some(p);
        }
        for (bi, b) in bundles.iter().enumerate() {
            for &m in b {
                r.nets[m].bundle = Some(bi);
            }
        }
        r.bundles = bundles;
        r
    }

    /// Pads per grid column within one via-plus-track distance. Copper placed there (above
    /// all vias, which block every layer) fences in fine-pitch pins that are routed later.
    fn build_crowd(&mut self) {
        let g = &self.grid;
        let via_r = self.nets.iter().map(|n| n.via_r).fold(0.0, f64::max);
        let clr = self.nets.iter().map(|n| n.clr).fold(0.0, f64::max);
        let reach = via_r + clr + self.min_width.max(g.g);
        let mut crowd = vec![0u8; g.n()];
        let mut cells: Vec<usize> = vec![];
        for pin in &self.pins {
            cells.clear();
            for pr in &pin.prims {
                let x0 = (((pr.bb.0 - reach - g.x0) / g.g).floor() as i32).max(0);
                let x1 = (((pr.bb.2 + reach - g.x0) / g.g).ceil() as i32).min(g.nx as i32 - 1);
                let y0 = (((pr.bb.1 - reach - g.y0) / g.g).floor() as i32).max(0);
                let y1 = (((pr.bb.3 + reach - g.y0) / g.g).ceil() as i32).min(g.ny as i32 - 1);
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        if pr.dist(g.center(x, y)) < reach {
                            cells.push(y as usize * g.nx + x as usize);
                        }
                    }
                }
            }
            cells.sort_unstable();
            cells.dedup();
            for &c in &cells {
                crowd[c] = crowd[c].saturating_add(1);
            }
        }
        self.crowd = crowd;
    }

    // ------------------------------------------------------------ stamping
    pub(crate) fn stamp_net(&mut self, n: usize, delta: i32) {
        let t = Instant::now();
        self.stamp_net_inner(n, delta);
        self.t_stamp += t.elapsed().as_secs_f32();
    }

    fn stamp_net_inner(&mut self, n: usize, delta: i32) {
        let grid = self.grid.clone();
        for &pi in &self.nets[n].pins {
            for pr in &self.pins[pi].prims {
                for f in self.fields.iter_mut() {
                    f.stamp(&grid, pr, delta, false);
                }
            }
        }
        for pr in &self.nets[n].pre.dynp {
            for f in self.fields.iter_mut() {
                f.stamp(&grid, pr, delta, false);
            }
        }
        for pr in &self.nets[n].r.dynp {
            for f in self.fields.iter_mut() {
                f.stamp(&grid, pr, delta, true);
            }
        }
    }

    pub(crate) fn stamp_dyn(&mut self, prims: &[Prim], delta: i32) {
        let grid = self.grid.clone();
        for pr in prims {
            for f in self.fields.iter_mut() {
                f.stamp(&grid, pr, delta, true);
            }
        }
    }

    // ------------------------------------------------------------ queries
    pub fn wire_length(&self, n: usize) -> f64 {
        let mut l = 0.0;
        for w in self.nets[n].r.wires.iter().filter(|w| !w.fill) {
            for s in w.pts.windows(2) {
                l += dist(s[0], s[1]);
            }
        }
        l
    }
    pub fn net_cost(&self, n: usize) -> f64 {
        self.wire_length(n)
            + self.nets[n].r.vias.len() as f64 * self.cfg.via_cost as f64 * self.grid.g
    }
    pub fn total_length(&self) -> f64 {
        (0..self.nets.len())
            .map(|n| self.wire_length(n))
            .sum::<f64>()
            - self.ground_fill_length
    }
    /// Sum over nets of the Euclidean minimum spanning tree of the pin centres: a (loose) lower
    /// bound on achievable wire length (a Steiner tree could be up to ~13% shorter).
    pub fn net_bound(&self, ni: usize) -> f64 {
        let n = &self.nets[ni];
        let mut total = 0.0;
        {
            let pts: Vec<P> = n.pins.iter().map(|&p| self.pins[p].c).collect();
            if pts.len() < 2 {
                return 0.0;
            }
            let mut inn = vec![false; pts.len()];
            let mut d = vec![f64::MAX; pts.len()];
            d[0] = 0.0;
            for _ in 0..pts.len() {
                let mut u = usize::MAX;
                for i in 0..pts.len() {
                    if !inn[i] && (u == usize::MAX || d[i] < d[u]) {
                        u = i;
                    }
                }
                inn[u] = true;
                total += d[u];
                for i in 0..pts.len() {
                    if !inn[i] {
                        d[i] = d[i].min(dist(pts[u], pts[i]));
                    }
                }
            }
        }
        total
    }
    pub fn length_lower_bound(&self) -> f64 {
        (0..self.nets.len()).map(|n| self.net_bound(n)).sum()
    }
    pub fn num_vias(&self) -> usize {
        self.nets.iter().map(|n| n.r.vias.len()).sum()
    }
    pub fn unrouted_count(&self) -> usize {
        self.nets.iter().map(|n| n.unconnected.len()).sum()
    }
    pub fn connection_count(&self) -> usize {
        self.nets
            .iter()
            .filter(|n| !n.skip)
            .map(|n| n.comps.len().saturating_sub(1))
            .sum()
    }

    /// representative pins of the groups that still have to be connected (comps[1..])
    pub fn reps(&self, n: usize) -> Vec<usize> {
        if self.nets[n].skip {
            return vec![];
        }
        self.nets[n].comps.iter().skip(1).map(|c| c[0]).collect()
    }

    // ------------------------------------------------------------ search
    fn blk_trace(f: &Field, excl: &[bool], grid: &Grid, l: usize, i: usize) -> Blk {
        let _ = grid;
        if excl[l] || f.fx(i) {
            Blk::Hard
        } else if f.dy(i) {
            Blk::Soft
        } else {
            Blk::Free
        }
    }

    /// A* from `seeds` to any of `targets`. Returns the cell path (seed .. target).
    #[allow(clippy::too_many_arguments)]
    /// Preferred layers still allow short pad escapes on other layers.
    pub fn routing_layer_costs(&self, n: usize) -> Vec<f32> {
        if !self.cfg.trace_routing || n >= self.nets.len() { return vec![1.0; self.grid.nl]; }
        let assigned = |l: &String| self.cfg.layers.get(l).is_some_and(|p| p.nets.contains(&self.nets[n].name));
        let has_assignment = self.layer_names.iter().any(assigned);
        self.layer_names.iter().map(|l| {
            if assigned(l) { 1.0 }
            else if has_assignment { 6.0 }
            else if self.cfg.layers.get(l).is_some_and(|p| !p.nets.is_empty()) { 4.0 }
            else { 1.0 }
        }).collect()
    }

    pub(crate) fn astar(
        &mut self,
        fi: (usize, usize),
        seeds: &[Cell],
        targets: &[Cell],
        soft: bool,
        win: (i32, i32, i32, i32),
        soft_pen: f32,
    ) -> Option<Vec<Cell>> {
        let t = Instant::now();
        let before = self.pops_total;
        let r = self.astar_inner(fi, seeds, targets, soft, win, soft_pen);
        let dt = t.elapsed().as_secs_f32();
        self.t_astar += dt;
        if std::env::var_os("AR_PROF").is_some() {
            self.prof.push((dt, (self.pops_total - before) as u32, soft, ((win.2 - win.0 + 1) as i64 * (win.3 - win.1 + 1) as i64) as u32, r.is_some()));
        }
        r
    }

    #[allow(clippy::too_many_arguments)]
    fn astar_inner(
        &mut self,
        fi: (usize, usize),
        seeds: &[Cell],
        targets: &[Cell],
        soft: bool,
        win: (i32, i32, i32, i32),
        soft_pen: f32,
    ) -> Option<Vec<Cell>> {
        self.stats_searches += 1;
        let grid = self.grid.clone();
        let nt = grid.n();
        let total = nt * grid.nl;
        if self.buf.g.len() != total {
            self.buf.g = vec![f32::INFINITY; total];
            self.buf.par = vec![u32::MAX; total];
            self.buf.tmark = vec![0; total];
        }
        let layer_cost = self.routing_layer_costs(self.cur_net);
        let buf = &mut self.buf;
        for &t in &buf.touched {
            buf.g[t as usize] = f32::INFINITY;
            buf.par[t as usize] = u32::MAX;
        }
        buf.touched.clear();
        buf.gen += 1;
        let gen = buf.gen;
        let tf = &self.fields[fi.0];
        let vf = &self.fields[fi.1];
        let real_excl = &self.excl;
        let open_all: Vec<bool> = vec![false; self.excl.len()];
        let excl: &Vec<bool> = if self.jump_mode { &open_all } else { &self.excl };
        let jump = self.jump_mode;
        let hw = self.h_weight;
        let budget = self.pop_budget;
        let mut pops = 0usize;
        let via_cost = if self.cur_land {
            self.cfg.via_cost * self.cfg.plane_via_factor
        } else {
            self.cfg.via_cost
        };
        let (wx0, wy0, wx1, wy1) = win;
        let land = self.cur_land;
        let plane_id = &self.plane_id;
        let pnets: Vec<usize> = self.planes.iter().map(|p| p.net).collect();
        let cur_net = self.cur_net;
        let crowd = &self.crowd;
        let guide = self.guide.as_ref();
        let no_guide = guide.is_none();
        let pen: f32 = if cur_net < self.nets.len() && !pnets.is_empty() {
            self.cfg.plane_penalty * self.nets[cur_net].plane_mul
        } else {
            0.0
        };

        // target set + bbox for heuristic
        let (mut tx0, mut ty0, mut tx1, mut ty1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
        let mut ntgt = 0;
        for &(l, x, y) in targets {
            let i = grid.idx(l, x, y);
            let hard = if land && excl[l] {
                tf.fx(i)
            } else {
                Self::blk_trace(tf, excl, &grid, l, i) == Blk::Hard
            };
            // A trace ending on a cell inside routed foreign clearance violates it even
            // when the cell is own copper (pad edge, via column): only rip-up may use it.
            if hard || (!soft && tf.dy(i)) {
                continue;
            }
            buf.tmark[i] = gen;
            ntgt += 1;
            tx0 = tx0.min(x);
            ty0 = ty0.min(y);
            tx1 = tx1.max(x);
            ty1 = ty1.max(y);
        }
        if ntgt == 0 {
            return None;
        }
        let h = |x: i32, y: i32| -> f32 {
            let dx = (tx0 - x).max(x - tx1).max(0) as f32;
            let dy = (ty0 - y).max(y - ty1).max(0) as f32;
            let (a, b) = if dx > dy { (dx, dy) } else { (dy, dx) };
            ((a - b) + b * SQ2) * hw
        };
        // Positive float bits sort by cost. Packing the cell into the low bits
        // preserves the original deterministic tie-break with one integer comparison.
        let key = |f: f32, i: usize| -> Reverse<u64> {
            Reverse(((f.to_bits() as u64) << 32) | i as u64)
        };
        let heap = &mut buf.heap;
        heap.clear();
        for &(l, x, y) in seeds {
            if x < wx0 || x > wx1 || y < wy0 || y > wy1 || excl[l] {
                continue;
            }
            let i = grid.idx(l, x, y);
            let b = Self::blk_trace(tf, excl, &grid, l, i);
            if b == Blk::Hard || (b == Blk::Soft && !soft) {
                continue;
            }
            if buf.g[i] > 0.0 {
                buf.g[i] = 0.0;
                buf.touched.push(i as u32);
                heap.push(key(h(x, y), i));
            }
        }
        const D8: [(i32, i32, f32); 8] = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, SQ2),
            (1, -1, SQ2),
            (-1, 1, SQ2),
            (-1, -1, SQ2),
        ];
        let nx = grid.nx as i32;
        let mut found: Option<u32> = None;
        while let Some(Reverse(entry)) = heap.pop() {
            let fk = (entry >> 32) as u32;
            let s = entry as u32 as usize;
            let l = s / nt;
            let rem = s - l * nt;
            let yq = rem / grid.nx;
            let (x, y) = ((rem - yq * grid.nx) as i32, yq as i32);
            let gs = buf.g[s];
            // stale entry check
            if f32::from_bits(fk) > gs + h(x, y) + 1e-3 {
                continue;
            }
            if buf.tmark[s] == gen {
                found = Some(s as u32);
                break;
            }
            pops += 1;
            self.pops_total += 1;
            if pops % 2048 == 0 && self.cfg.cancel.as_ref().is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed)) { return None; }
            if budget != 0 && pops > budget {
                break;
            }
            let base = l * nt;
            let lc = layer_cost[l];
            for &(dx, dy, c) in D8.iter() {
                let (xx, yy) = (x + dx, y + dy);
                if xx < wx0 || xx > wx1 || yy < wy0 || yy > wy1 {
                    continue;
                }
                let i2 = base + (yy * nx + xx) as usize;
                // Every other cost term only adds, so this bound rejects exactly the
                // neighbours the full cost would (the guide can discount: skip it then).
                if no_guide && gs + c * lc >= buf.g[i2] {
                    continue;
                }
                let b = Self::blk_trace(tf, excl, &grid, l, i2);
                if b == Blk::Hard || (b == Blk::Soft && !soft) {
                    continue;
                }
                let mut cost = c * (lc + self.congestion[i2] as f32 * 0.5) + if b == Blk::Soft { soft_pen } else { 0.0 };
                if pen > 0.0 && !plane_id[l].is_empty() {
                    let v = plane_id[l][i2 - base];
                    if v != 0 && pnets[(v - 1) as usize] != cur_net {
                        cost += c * pen;
                    }
                }
                if jump && real_excl[l] {
                    cost *= 12.0;
                }
                if let Some(gd) = guide {
                    cost *= if gd.near[i2] { gd.bonus } else { gd.penalty };
                }
                // Most neighbours already have a better route. Avoid the two
                // extra obstacle probes for diagonal corner checks in that case.
                let ng = gs + cost;
                if ng >= buf.g[i2] {
                    continue;
                }
                if dx != 0 && dy != 0 {
                    let a = Self::blk_trace(tf, excl, &grid, l, base + (y * nx + xx) as usize);
                    let bb = Self::blk_trace(tf, excl, &grid, l, base + (yy * nx + x) as usize);
                    if a == Blk::Hard
                        || bb == Blk::Hard
                        || (!soft && (a == Blk::Soft || bb == Blk::Soft))
                    {
                        continue;
                    }
                }
                if ng < buf.g[i2] {
                    if buf.g[i2].is_infinite() {
                        buf.touched.push(i2 as u32);
                    }
                    buf.g[i2] = ng;
                    buf.par[i2] = s as u32;
                    heap.push(key(ng + h(xx, yy), i2));
                }
            }
            // vias: through via to every other (non exclusive) layer
            let ci = rem;
            let vblk = match vf.via_col(ci) {
                (true, _) => Blk::Hard,
                (false, true) => Blk::Soft,
                _ => Blk::Free,
            };
            if vblk == Blk::Hard || (vblk == Blk::Soft && !soft) {
                continue;
            }
            let crowd_via = if crowd.is_empty() { 1.0 } else { 1.0 + CROWD_VIA * crowd[ci].min(4) as f32 };
            for ll in 0..grid.nl {
                let i2 = ll * nt + ci;
                if ll == l || (excl[ll] && !(land && buf.tmark[i2] == gen)) {
                    continue;
                }
                let b = if excl[ll] {
                    if tf.fx(i2) {
                        Blk::Hard
                    } else if tf.dy(i2) {
                        Blk::Soft
                    } else {
                        Blk::Free
                    }
                } else {
                    Self::blk_trace(tf, excl, &grid, ll, i2)
                };
                if b == Blk::Hard || (b == Blk::Soft && !soft) {
                    continue;
                }
                let cost = via_cost * crowd_via
                    + if vblk == Blk::Soft || b == Blk::Soft {
                        soft_pen
                    } else {
                        0.0
                    };
                let ng = gs + cost;
                if ng < buf.g[i2] {
                    if buf.g[i2].is_infinite() {
                        buf.touched.push(i2 as u32);
                    }
                    buf.g[i2] = ng;
                    buf.par[i2] = s as u32;
                    heap.push(key(ng + h(x, y), i2));
                }
            }
        }
        let end = found?;
        let mut path = vec![];
        let mut cur = end;
        loop {
            let s = cur as usize;
            let l = s / nt;
            let rem = s % nt;
            path.push((l, (rem % grid.nx) as i32, (rem / grid.nx) as i32));
            let p = buf.par[s];
            if p == u32::MAX {
                break;
            }
            cur = p;
        }
        path.reverse();
        Some(path)
    }

    pub(crate) fn reachable(
        &mut self,
        fi: (usize, usize),
        seeds: &[Cell],
        targets: &[Cell],
    ) -> bool {
        if self.cur_land || self.jump_mode {
            return true;
        }
        let t = Instant::now();
        let r = if targets.len() < seeds.len() {
            self.reachable_inner(fi, targets, seeds)
        } else { self.reachable_inner(fi, seeds, targets) };
        self.t_reach += t.elapsed().as_secs_f32();
        self.n_reach += 1;
        r
    }

    /// Cheap flood fill: can any seed reach any target through free cells at all?
    /// (Used to avoid an exhaustive heap-based A* that is bound to fail.)
    pub(crate) fn reachable_inner(
        &self,
        fi: (usize, usize),
        seeds: &[Cell],
        targets: &[Cell],
    ) -> bool {
        let grid = &self.grid;
        let nt = grid.n();
        let total = nt * grid.nl;
        let tf = &self.fields[fi.0];
        let vf = &self.fields[fi.1];
        let excl = &self.excl;
        let hard = |l: usize, i: usize| excl[l] || tf.fx(i) || tf.dy(i);
        let mut tgt = vec![0u64; total.div_ceil(64)];
        let mut has_target = false;
        for &(l, x, y) in targets {
            let i = grid.idx(l, x, y);
            if !hard(l, i) {
                tgt[i >> 6] |= 1u64 << (i & 63);
                has_target = true;
            }
        }
        if !has_target {
            return false;
        }
        let mut seen = vec![0u64; total.div_ceil(64)];
        let mut q: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
        let mark = |seen: &mut Vec<u64>, i: usize| -> bool {
            let (w, b) = (i >> 6, 1u64 << (i & 63));
            if seen[w] & b != 0 {
                false
            } else {
                seen[w] |= b;
                true
            }
        };
        for &(l, x, y) in seeds {
            let i = grid.idx(l, x, y);
            if !hard(l, i) && mark(&mut seen, i) {
                q.push_back(i as u32);
            }
        }
        let nx = grid.nx as i32;
        let ny = grid.ny as i32;
        const D8: [(i32, i32); 8] = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ];
        let mut visited = 0usize;
        while let Some(s) = q.pop_front() {
            visited += 1;
            if visited % 4096 == 0 && self.cancelled() { return false; }
            let s = s as usize;
            if tgt[s >> 6] & (1u64 << (s & 63)) != 0 {
                return true;
            }
            let l = s / nt;
            let rem = s % nt;
            let (x, y) = ((rem % grid.nx) as i32, (rem / grid.nx) as i32);
            let base = l * nt;
            for &(dx, dy) in D8.iter() {
                let (xx, yy) = (x + dx, y + dy);
                if xx < 0 || yy < 0 || xx >= nx || yy >= ny {
                    continue;
                }
                let i2 = base + (yy * nx + xx) as usize;
                if hard(l, i2) {
                    continue;
                }
                if dx != 0
                    && dy != 0
                    && (hard(l, base + (y * nx + xx) as usize)
                        || hard(l, base + (yy * nx + x) as usize))
                {
                    continue;
                }
                if mark(&mut seen, i2) {
                    q.push_back(i2 as u32);
                }
            }
            if vf.via_col(rem) != (false, false) {
                continue;
            }
            for ll in 0..grid.nl {
                if ll == l || excl[ll] {
                    continue;
                }
                let i2 = ll * nt + rem;
                if !hard(ll, i2) && mark(&mut seen, i2) {
                    q.push_back(i2 as u32);
                }
            }
        }
        false
    }

    /// Full-window hard search.
    pub(crate) fn full_search(
        &mut self,
        fi: (usize, usize),
        seeds: &[Cell],
        targets: &[Cell],
        full: (i32, i32, i32, i32),
    ) -> Option<Vec<Cell>> {
        self.astar(fi, seeds, targets, false, full, 0.0)
    }

    pub fn path_cost(&self, path: &[Cell]) -> f32 {
        let mut c = 0.0;
        for w in path.windows(2) {
            c += if w[0].0 != w[1].0 {
                self.cfg.via_cost
            } else if w[0].1 != w[1].1 && w[0].2 != w[1].2 {
                SQ2
            } else {
                1.0
            };
        }
        c
    }

    /// Benchmark/parity hook: route pin `a` -> pin `b` of net `n` on the current board state.
    /// Returns (cost, cells, seconds).
    pub fn debug_search(
        &mut self,
        n: usize,
        a: usize,
        b: usize,
    ) -> Option<(f32, usize, f32)> {
        let grid = self.grid.clone();
        let full = (0, 0, grid.nx as i32 - 1, grid.ny as i32 - 1);
        let seeds = self.pins[self.nets[n].pins[a]].cells.clone();
        let targets = self.pins[self.nets[n].pins[b]].cells.clone();
        let fi = (self.nets[n].tf, self.nets[n].vf);
        self.stamp_net(n, -1);
        let t = Instant::now();
        let r = self.full_search(fi, &seeds, &targets, full);
        let dt = t.elapsed().as_secs_f32();
        self.stamp_net(n, 1);
        r.map(|p| (self.path_cost(&p), p.len(), dt))
    }

    // ------------------------------------------------------------ geometry from a path
    pub(crate) fn path_geometry(&self, n: usize, path: &[Cell]) -> (Vec<Wire>, Vec<P>) {
        let ns = &self.nets[n];
        let mut wires: Vec<Wire> = vec![];
        let mut vias: Vec<P> = vec![];
        let mut run: Vec<P> = vec![];
        let mut run_layer = path[0].0;
        let flush = |run: &mut Vec<P>, layer: usize, wires: &mut Vec<Wire>| {
            // drop collinear points
            let mut out: Vec<P> = vec![];
            for &p in run.iter() {
                while out.len() >= 2 {
                    let (a, b) = (out[out.len() - 2], out[out.len() - 1]);
                    let cr = (b.0 - a.0) * (p.1 - b.1) - (b.1 - a.1) * (p.0 - b.0);
                    if cr.abs() < 1e-9
                        && (b.0 - a.0) * (p.0 - b.0) + (b.1 - a.1) * (p.1 - b.1) > 0.0
                    {
                        out.pop();
                    } else {
                        break;
                    }
                }
                if out.last().map_or(true, |&q| dist(q, p) > 1e-9) {
                    out.push(p);
                }
            }
            if out.len() >= 2 {
                wires.push(Wire {
                    fill: false,
                    layer,
                    width: ns.width,
                    pts: out,
                });
            }
            run.clear();
        };
        for (i, &(l, x, y)) in path.iter().enumerate() {
            let c = self.grid.center(x, y);
            if l != run_layer {
                flush(&mut run, run_layer, &mut wires);
                vias.push(c);
                run_layer = l;
            }
            run.push(c);
            let _ = i;
        }
        flush(&mut run, run_layer, &mut wires);
        // Grid endpoints already lie inside terminal copper. Do not add unchecked,
        // arbitrary-angle centre stubs (which could cross a neighbouring pad).
        // Multi-terminal nets grow from every cell of their tree. Moving that tree
        // here would leave later branches attached to copper that no longer exists.
        if self.nets[n].pins.len() == 2 {
            for w in &mut wires {
                self.smooth_wire(n, w);
            }
        }
        (wires, vias)
    }

    pub(crate) fn widened_wires(&self, n: usize, wires: &[Wire], requested_width: f64) -> Vec<Wire> {
            let mut expanded: Vec<Wire> = Vec::new();
            for w in wires {
                if w.width >= requested_width || w.fill { expanded.push(w.clone()); continue; }
                for s in w.pts.windows(2) {
                    let steps = (dist(s[0], s[1]) / self.grid.g).ceil().max(1.0) as usize;
                    for i in 0..steps {
                        let point = |i: usize| {
                            let t = i as f64 / steps as f64;
                            (
                                s[0].0 + t * (s[1].0 - s[0].0),
                                s[0].1 + t * (s[1].1 - s[0].1),
                            )
                        };
                        let (a, b) = (point(i), point(i + 1));
                        let full = Prim::new(
                            w.layer,
                            Geo::Seg {
                                a,
                                b,
                                hw: requested_width / 2.0,
                            },
                            n as i32,
                            self.nets[n].clr,
                        );
                        let width = if self.prims_conflict(&[full]).is_none() {
                            requested_width
                        } else {
                            w.width
                        };
                        if let Some(last) = expanded.last_mut().filter(|last| {
                            last.layer == w.layer
                                && last.width == width
                                && dist(*last.pts.last().unwrap(), a) < 1e-8
                        }) {
                            last.pts.push(b);
                        } else {
                            expanded.push(Wire {
                                fill: false,
                                layer: w.layer,
                                width,
                                pts: vec![a, b],
                            });
                        }
                    }
                }
            }
        expanded
    }

    pub(crate) fn commit_path(&mut self, n: usize, path: &[Cell], requested_width: f64) {
        let (mut wires, vias) = self.path_geometry(n, path);
        let (clr, via_r) = (self.nets[n].clr, self.nets[n].via_r);
        if self.nets[n].width < requested_width && !self.cfg.trace_routing {
            wires = self.widened_wires(n, &wires, requested_width);
        }
        let nl = self.grid.nl;
        let ns = &mut self.nets[n];
        for wr in &wires {
            for s in wr.pts.windows(2) {
                ns.r.dynp.push(Prim::new(
                    wr.layer,
                    Geo::Seg {
                        a: s[0],
                        b: s[1],
                        hw: wr.width / 2.0,
                    },
                    n as i32,
                    clr,
                ));
            }
        }
        for &v in &vias {
            for l in 0..nl {
                ns.r.dynp
                    .push(Prim::new(l, Geo::Circle { c: v, r: via_r }, n as i32, clr));
            }
            let (x, y) = (self.grid.cell(v).0, self.grid.cell(v).1);
            for l in 0..nl {
                ns.r.tree.push((l, x, y));
            }
        }
        ns.r.tree.extend_from_slice(path);
        ns.r.wires.extend(wires);
        if self.pad_via_mode {
            let pins = self.nets[n].pins.clone();
            let inside = |v: &P| {
                pins.iter().any(|&pi| {
                    self.pins[pi]
                        .prims
                        .iter()
                        .any(|pr| pr.dist(*v) < 0.0 || pr.dist(*v) <= 1e-9)
                })
            };
            let k = vias.iter().filter(|v| inside(v)).count();
            self.pad_vias_used += k;
        }
        self.nets[n].r.vias.extend(vias);
    }

    pub(crate) fn rip_net(&mut self, n: usize) {
        // unstamp dynamic copper and clear routing (pads stay stamped)
        let prims = std::mem::take(&mut self.nets[n].r.dynp);
        self.stamp_dyn(&prims, -1);
        let ns = &mut self.nets[n];
        ns.r = Routed::default();
        ns.unconnected = if ns.skip {
            vec![]
        } else {
            ns.comps.iter().skip(1).map(|c| c[0]).collect()
        };
        if ns.coupled || ns.guided {
            ns.coupled = false;
            ns.guided = false;
            for m in self.group_members(n) {
                if m != n && (self.nets[m].coupled || self.nets[m].guided) {
                    self.rip_net(m);
                }
            }
        }
    }

    /// Victims: nets whose dynamic copper conflicts with the path.
    fn path_victims(&self, n: usize, path: &[Cell], tf_index: usize) -> HashSet<usize> {
        let mut v = HashSet::new();
        let ns = &self.nets[n];
        let tf = &self.fields[tf_index];
        let vf = &self.fields[ns.vf];
        let max_clr = self.nets.iter().map(|x| x.clr).fold(0.0, f64::max);
        let reach = max_clr + tf.p.max(vf.p) + 0.1;
        for (i, &(l, x, y)) in path.iter().enumerate() {
            let c = self.grid.center(x, y);
            let idx = self.grid.idx(l, x, y);
            let mut probes: Vec<&Field> = vec![];
            if tf.dy(idx) {
                probes.push(tf);
            }
            let layer_change = i > 0 && path[i - 1].0 != l;
            let next_change = i + 1 < path.len() && path[i + 1].0 != l;
            let mut vias_here = false;
            if layer_change || next_change {
                vias_here = true;
            }
            for (m, o) in self.nets.iter().enumerate() {
                if m == n {
                    continue;
                }
                for pr in &o.r.dynp {
                    if c.0 < pr.bb.0 - reach
                        || c.0 > pr.bb.2 + reach
                        || c.1 < pr.bb.1 - reach
                        || c.1 > pr.bb.3 + reach
                    {
                        continue;
                    }
                    if (pr.layer == l || vias_here)
                        && ((pr.layer == l && probes.iter().any(|f| pr.dist(c) < f.threshold(pr)))
                            || (vias_here && pr.dist(c) < vf.threshold(pr)))
                    {
                        v.insert(m);
                        break;
                    }
                }
            }
        }
        v
    }

    // ------------------------------------------------------------ net routing
    /// Route a single net. Returns list of victims ripped (they need rerouting).
    /// Pre-existing wiring and plane copper count as already connected groups of pins.
    pub fn route_net_plain(&mut self, n: usize, allow_rip: bool) -> Vec<usize> {
        let mut ripped: Vec<usize> = vec![];
        if self.nets[n].skip || self.nets[n].comps.len() < 2 {
            return ripped;
        }
        // own pads + copper out of the fields
        self.stamp_net(n, -1);
        self.cur_net = n;
        let ncomp = self.nets[n].comps.len();
        let landing = if self.nets[n].plane.is_some() {
            self.plane_landing(n)
        } else {
            None
        };
        self.cur_land = landing.is_some();
        let mut tree: Vec<Cell> = self.nets[n].r.tree.clone();
        tree.extend_from_slice(&self.nets[n].comp_cells[0]);
        let mut conn_pins: Vec<usize> = self.nets[n].comps[0].clone();
        let mut remaining: Vec<usize> = (1..ncomp).collect();
        let mut failed: Vec<usize> = vec![];
        let nx = self.grid.nx as i32;
        let ny = self.grid.ny as i32;
        let full = (0, 0, nx - 1, ny - 1);
        let planned_width = self.nets[n].width;
        let staged = self.cfg.trace_routing && self.cfg.allow_neckdown;
        let fi = (if staged { self.nets[n].narrow_tf } else { self.nets[n].tf }, self.nets[n].vf);
        while !remaining.is_empty() {
            self.nets[n].width = if staged { self.fields[fi.0].p * 2.0 } else { planned_width };
            // nearest group to the connected set
            let mut best = (f64::MAX, 0usize);
            for (ri, &c) in remaining.iter().enumerate() {
                for &p in &self.nets[n].comps[c] {
                    for &q in &conn_pins {
                        let (a, b) = (self.pins[p].c, self.pins[q].c);
                        let d = (a.0 - b.0).abs() + (a.1 - b.1).abs();
                        if d < best.0 {
                            best = (d, ri);
                        }
                    }
                }
            }
            let c = remaining.remove(best.1);
            let comp_cells = self.nets[n].comp_cells[c].clone();
            let (mut bx0, mut by0, mut bx1, mut by1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
            for &(_, x, y) in &comp_cells {
                bx0 = bx0.min(x);
                by0 = by0.min(y);
                bx1 = bx1.max(x);
                by1 = by1.max(y);
            }
            let margin = (30.0 / self.grid.g) as i32;
            let (seeds, targets, win0): (Vec<Cell>, Vec<Cell>, (i32, i32, i32, i32));
            if let Some((ll, mask)) = &landing {
                // plane mode: grow from the group towards the tree or any free plane cell
                let m = margin * 2;
                let w = (
                    (bx0 - m).max(0),
                    (by0 - m).max(0),
                    (bx1 + m).min(nx - 1),
                    (by1 + m).min(ny - 1),
                );
                let mut tg = tree.clone();
                tg.extend(self.landing_cells(*ll, mask, w));
                seeds = comp_cells.clone();
                targets = tg;
                win0 = w;
            } else {
                let mut dmin = i32::MAX;
                for &(_, x, y) in &tree {
                    let dx = (bx0 - x).max(x - bx1).max(0);
                    let dy = (by0 - y).max(y - by1).max(0);
                    dmin = dmin.min(dx.max(dy));
                }
                let r = dmin.saturating_add(margin).min(nx.max(ny));
                seeds = tree.clone();
                targets = comp_cells.clone();
                win0 = (
                    (bx0 - r).max(0),
                    (by0 - r).max(0),
                    (bx1 + r).min(nx - 1),
                    (by1 + r).min(ny - 1),
                );
            }
            let tc = Instant::now();
            // a bounded first try: when it runs away, the cheap flood below decides what is next
            self.pop_budget = 60_000;
            let mut path = if win0 != full {
                self.astar(fi, &seeds, &targets, false, win0, 0.0)
            } else {
                None
            };
            self.pop_budget = 0;
            self.t_cat[0] += tc.elapsed().as_secs_f32();
            // full-board targets (plane mode needs the whole plane as target set)
            let targets_full: Vec<Cell> = match &landing {
                Some((ll, mask)) => {
                    let mut tg = tree.clone();
                    tg.extend(self.landing_cells(*ll, mask, full));
                    tg
                }
                None => targets.clone(),
            };
            let tc = Instant::now();
            if path.is_none() && self.reachable(fi, &seeds, &targets_full) {
                // long detours: the flood proved a path exists, so favour speed over exactness
                self.h_weight = full_h_weight();
                path = if landing.is_some() {
                    self.astar(fi, &seeds, &targets_full, false, full, 0.0)
                } else {
                    self.full_search(fi, &seeds, &targets_full, full)
                };
                self.h_weight = h_weight();
            }
            self.t_cat[1] += tc.elapsed().as_secs_f32();
            let tc = Instant::now();
            let requested_width = planned_width;
            let narrow_fi = (self.nets[n].narrow_tf, fi.1);
            let narrow_width = self.fields[narrow_fi.0].p * 2.0;
            if path.is_none() && self.cfg.allow_neckdown && narrow_fi.0 != fi.0 && narrow_width < requested_width {
                if self.reachable(narrow_fi, &seeds, &targets_full) {
                    path = self.astar(narrow_fi, &seeds, &targets_full, false, full, 0.0);
                    if path.is_some() { self.nets[n].width = narrow_width; }
                }
            }
            self.t_cat[3] += tc.elapsed().as_secs_f32();
            let tc = Instant::now();
            if path.is_none() && allow_rip && self.rip_events < self.cfg.max_rip_events {
                // Escape fine-pitch pads at the legal neckdown width even when
                // the escape needs to displace routed copper. Fixed pads stay hard.
                let rip_fi = if self.cfg.allow_neckdown { narrow_fi } else { fi };
                if let Some(p) = self.astar(rip_fi, &seeds, &targets_full, true, full, 25.0) {
                    let mut victims: Vec<usize> = self.path_victims(n, &p, rip_fi.0).into_iter().collect();
                    victims.sort();
                    let stuck = victims.iter().any(|&m| self.nets[m].rip_count >= 6);

                    if !stuck {
                        // Retain congestion history even if a repair is rolled back:
                        // retrying the identical corridor otherwise just cycles victims.
                        if !victims.is_empty() {
                            for &(l, x, y) in &p {
                                let i = self.grid.idx(l, x, y);
                                if self.fields[rip_fi.0].dy(i) {
                                    self.congestion[i] = self.congestion[i].saturating_add(2).min(40);
                                }
                            }
                        }
                        for &m in &victims {
                            self.rip_events += 1;
                            self.nets[m].rip_count += 1;
                            let partner = if self.nets[m].coupled {
                                self.nets[m].pair
                            } else {
                                None
                            };
                            let mates: Vec<usize> = if self.nets[m].guided {
                                self.group_members(m)
                            } else {
                                vec![]
                            };
                            self.rip_net(m);
                            ripped.push(m);
                            if let Some(q) = partner {
                                ripped.push(q);
                            }
                            for q in mates {
                                if q != m && !ripped.contains(&q) {
                                    ripped.push(q);
                                }
                            }
                        }
                        self.nets[n].width = self.fields[rip_fi.0].p * 2.0;
                        path = Some(p);
                    }
                }
            }
            self.t_cat[2] += tc.elapsed().as_secs_f32();
            if self.jump_mode {
                if let Some(p) = &path {
                    let on_plane = p.iter().filter(|c| self.excl[c.0]).count();
                    let limit = self.cfg.plane_jump_mm / self.unit_mm.max(1e-12) / self.grid.g;
                    if on_plane as f64 > limit {
                        path = None;
                    } else if on_plane > 1 {
                        self.jumps_used += 1;
                    }
                }
            }
            match path {
                Some(p) => {
                    self.commit_path(n, &p, requested_width);
                    tree.extend_from_slice(&p);
                    for &v in &self.nets[n].r.vias.clone() {
                        let (x, y) = self.grid.cell(v);
                        for l in 0..self.grid.nl {
                            tree.push((l, x, y));
                        }
                    }
                    tree.extend_from_slice(&comp_cells);
                    let add = self.nets[n].comps[c].clone();
                    conn_pins.extend(add);
                }
                None => failed.push(self.nets[n].comps[c][0]),
            }
            self.nets[n].width = requested_width;
        }
        self.nets[n].unconnected = failed;
        self.stamp_net(n, 1);
        self.cur_net = usize::MAX;
        self.cur_land = false;
        ripped
    }

    /// Still open after everything else. Stage 1: a short trace on a plane / no-routing layer.
    /// Stage 2: a via inside a pad of the failing net. Stage 3: both.
    pub fn jump_pass(&mut self) {
        let has_excl = self.excl.iter().any(|&e| e);
        let mut stages: Vec<(bool, bool)> = vec![];
        if self.cfg.allow_plane_jumps && has_excl {
            stages.push((true, false));
        }
        if self.cfg.last_resort_pad_vias && !self.cfg.allow_vias_in_pads {
            stages.push((false, true));
            if self.cfg.allow_plane_jumps && has_excl {
                stages.push((true, true));
            }
        }
        for (jump, padvia) in stages {
            let failing: Vec<usize> = (0..self.nets.len())
                .filter(|&n| !self.nets[n].unconnected.is_empty() && !self.nets[n].skip)
                .collect();
            if failing.is_empty() {
                return;
            }
            for n in failing {
                if self.cancelled() {
                    return;
                }
                self.last_resort(n, jump, padvia);
            }
        }
    }

    fn last_resort(&mut self, n: usize, jump: bool, padvia: bool) {
        let old = (self.nets[n].r.clone(), self.nets[n].unconnected.clone());
        let (c, gd) = (self.nets[n].coupled, self.nets[n].guided);
        let (used_j, used_p) = (self.jumps_used, self.pad_vias_used);
        let grid = self.grid.clone();
        // lift the "no vias in pads" mask for this net's own pads only
        let lift = padvia && !self.cfg.allow_vias_in_pads;
        if lift {
            for &pi in &self.nets[n].pins {
                for pr in &self.pins[pi].prims {
                    for f in self.fields.iter_mut().filter(|f| f.via) {
                        f.stamp(&grid, pr, -1, false);
                    }
                }
            }
        }
        self.jump_mode = jump;
        self.pad_via_mode = lift;
        self.route_missing(n);
        self.jump_mode = false;
        self.pad_via_mode = false;
        if lift {
            for &pi in &self.nets[n].pins {
                for pr in &self.pins[pi].prims {
                    for f in self.fields.iter_mut().filter(|f| f.via) {
                        f.stamp(&grid, pr, 1, false);
                    }
                }
            }
        }
        if self.cfg.verbose {
            eprintln!(
                "last resort {} jump={} padvia={}: open {} -> {}",
                self.nets[n].name,
                jump,
                padvia,
                old.1.len(),
                self.nets[n].unconnected.len()
            );
        }
        // the new copper must be clean: anything that violates clearance is thrown away
        let mut bad = false;
        if self.nets[n].unconnected.len() < old.1.len() {
            let saved = self.cfg.allow_vias_in_pads;
            self.cfg.allow_vias_in_pads = true; // via-in-pad is judged by plain clearance below
            let new = self.nets[n].r.dynp.clone();
            bad = self.prims_conflict(&new).is_some();
            self.cfg.allow_vias_in_pads = saved;
        }
        if bad || self.nets[n].unconnected.len() >= old.1.len() {
            let prims = std::mem::take(&mut self.nets[n].r.dynp);
            self.stamp_dyn(&prims, -1);
            self.nets[n].r = old.0;
            self.nets[n].unconnected = old.1;
            let prims = self.nets[n].r.dynp.clone();
            self.stamp_dyn(&prims, 1);
            self.jumps_used = used_j;
            self.pad_vias_used = used_p;
        }
        self.nets[n].coupled = c;
        self.nets[n].guided = gd;
    }

    /// Re-run the plain router for a net from scratch (used by the last-resort passes).
    fn route_missing(&mut self, n: usize) {
        let prims = std::mem::take(&mut self.nets[n].r.dynp);
        self.stamp_dyn(&prims, -1);
        self.nets[n].r = Routed::default();
        self.route_net_plain(n, false);
    }

    /// Last resort for connections that are still open: let the failing net push other nets
    /// out of its way (unlimited rip-up for this attempt), reroute the pushed nets, and keep the
    /// result only when fewer connections are open than before. Otherwise everything is put back.
    pub fn rescue(&mut self) {
        let mut explored = HashSet::new();
        let mut stale = 0;
        let threads = self.spec_threads();
        for round in 0..8 {
            let failing: Vec<usize> = (0..self.nets.len())
                .filter(|&n| !self.nets[n].unconnected.is_empty() && !self.nets[n].skip)
                .collect();
            if failing.is_empty() {
                return;
            }
            explored.insert(failing.clone());
            let mut progress = false;
            let mut idx = 0;
            while idx < failing.len() {
                if self.cancelled() {
                    return;
                }
                let before = self.unrouted_count();
                // Trials for different open nets are independent: run a wave of them in parallel
                // on copies of the current state, then apply the first one (in net order) that
                // the sequential rules would have accepted. Later results of that wave started
                // from the old state, so the following wave starts again after the accepted net.
                if threads > 1 && std::env::var("AR_RESCUE_PAR").is_ok_and(|v| v == "1") /* measured slower: off */ {
                    let mut wave: Vec<usize> = vec![];
                    let mut j = idx;
                    while j < failing.len() && wave.len() < threads {
                        if !self.nets[failing[j]].unconnected.is_empty() {
                            wave.push(failing[j]);
                        }
                        j += 1;
                    }
                    if wave.len() >= 2 {
                        self.snapshot(&format!("Repair {}: exploring alternatives for {} nets in parallel", round + 1, wave.len()));
                        let results = self.rescue_wave(&wave, round);
                        let mut accepted = None;
                        for (n, open, remaining, saved) in results {
                            if self.cfg.verbose {
                                eprintln!("rescue round {} {}: open {} -> {} {:?} (parallel)", round + 1, self.nets[n].name, before, open,
                                    remaining.iter().map(|&m| self.nets[m].name.clone()).collect::<Vec<_>>());
                            }
                            if open < before || (open == before && explored.insert(remaining)) {
                                self.restore_routes(saved);
                                progress = true;
                                accepted = failing.iter().position(|&x| x == n);
                                break;
                            }
                        }
                        idx = accepted.map_or(j, |p| p + 1);
                        continue;
                    }
                }
                let n = failing[idx];
                idx += 1;
                if self.nets[n].unconnected.is_empty() {
                    continue;
                }
                let saved = self.save_routes();
                let sig = |x: &NetState| {
                    let len: f64 = x.r.dynp.iter().map(|p| p.bb.0 + p.bb.1 * 3.0).sum();
                    (x.r.wires.len(), x.r.vias.len(), x.r.dynp.len(), (len * 100.0) as i64)
                };
                let sigs: Vec<_> = self.nets.iter().map(sig).collect();
                self.snapshot(&format!("Repair {}: exploring alternatives for {}", round + 1, self.nets[n].name));
                let (open, remaining) = self.rescue_trial(n, round);
                // A displacement can move the last open connection to a different
                // net before it can be solved. Keep novel equal-completion states;
                // rejecting every plateau makes this repair impossible.
                if self.cfg.verbose {
                    eprintln!("rescue round {} {}: open {} -> {} {:?}", round + 1, self.nets[n].name, before, open,
                        remaining.iter().map(|&m| self.nets[m].name.clone()).collect::<Vec<_>>());
                }
                if open < before || (open == before && explored.insert(remaining)) {
                    progress = true;
                    continue;
                }
                // put everything back
                let changed: Vec<usize> = (0..self.nets.len())
                    .filter(|&i| sig(&self.nets[i]) != sigs[i] || self.nets[i].unconnected != saved[i].1)
                    .collect();
                for &i in &changed {
                    let prims = std::mem::take(&mut self.nets[i].r.dynp);
                    self.stamp_dyn(&prims, -1);
                }
                for (i, s) in saved.into_iter().enumerate() {
                    if changed.contains(&i) {
                        self.nets[i].r = s.0;
                        let prims = self.nets[i].r.dynp.clone();
                        self.stamp_dyn(&prims, 1);
                    }
                    self.nets[i].unconnected = s.1;
                    self.nets[i].coupled = s.2;
                    self.nets[i].guided = s.3;
                }
            }
            if !progress && !self.cfg.trace_routing {
                return;
            }
            // Routing is deterministic: a round without progress only differs from the
            // previous one by its via cost. After every via cost was tried, stop replaying.
            stale = if progress { 0 } else { stale + 1 };
            if stale >= 3 {
                return;
            }
        }
    }

    /// One repair attempt: remove net `n` and route it again with unlimited rip-up (a few nets
    /// are pushed aside and re-routed), then give everything still open one plain attempt.
    /// Returns (open connections, nets still open). The caller decides whether to keep it.
    fn rescue_trial(&mut self, n: usize, round: usize) -> (usize, Vec<usize>) {
        let old_events = self.rip_events;
        let preferred_via_cost = self.cfg.via_cost;
        self.cfg.via_cost *= [1.0, 0.5, 1.0, 0.25][round % 4];
        self.rip_events = 0;
        for x in self.nets.iter_mut() {
            x.rip_count = 0;
        }
        self.rip_net(n);
        let mut queue: std::collections::VecDeque<usize> = [n].into_iter().collect();
        let mut guard = 0;
        while let Some(m) = queue.pop_front() {
            guard += 1;
            if guard > 40 {
                break;
            }
            for v in self.route_net(m, true) {
                if !queue.contains(&v) {
                    queue.push_back(v);
                }
            }
        }
        // anything still open gets one more plain attempt
        let open: Vec<usize> = (0..self.nets.len())
            .filter(|&m| !self.nets[m].unconnected.is_empty() && !self.nets[m].skip)
            .collect();
        for m in open {
            self.rip_net(m);
            self.route_net(m, false);
        }
        self.rip_events = old_events;
        self.cfg.via_cost = preferred_via_cost;
        let remaining: Vec<usize> = (0..self.nets.len())
            .filter(|&m| !self.nets[m].unconnected.is_empty() && !self.nets[m].skip)
            .collect();
        (self.unrouted_count(), remaining)
    }

    /// `rescue_trial` for several nets at once, each on its own copy of the current state.
    /// Returns, in the order of `nets`, (net, open, nets still open, resulting routes).
    #[allow(clippy::type_complexity)]
    fn rescue_wave(&mut self, nets: &[usize], round: usize) -> Vec<(usize, usize, Vec<usize>, Vec<(Routed, Vec<usize>, bool, bool)>)> {
        let mut workers = std::mem::take(&mut self.workers);
        while workers.len() < nets.len() {
            workers.push(self.clone());
        }
        let master: &Router = self;
        let out = std::thread::scope(|s| {
            let handles: Vec<_> = workers
                .iter_mut()
                .zip(nets.iter().copied())
                .map(|(w, n)| {
                    s.spawn(move || {
                        w.sync_from(master);
                        let (open, remaining) = w.rescue_trial(n, round);
                        (n, open, remaining, w.save_routes())
                    })
                })
                .collect();
            handles.into_iter().filter_map(|h| h.join().ok()).collect::<Vec<_>>()
        });
        self.workers = workers;
        out
    }

    /// Two terminals that keep ripping each other out (neighbouring fine-pitch escapes) cannot
    /// be solved by moving one net at a time. Clear every routed net around the open terminals,
    /// route the failing net first and the cleared nets around it, most constrained first.
    /// A result is kept only when fewer connections are open. Larger areas never paid off in
    /// testing: beyond this, `route_all` restarts the board with the failing nets first.
    pub fn regional_rescue(&mut self) {
        for round in 0..1u32 {
            let failing: Vec<usize> = (0..self.nets.len())
                .filter(|&n| !self.nets[n].unconnected.is_empty() && !self.nets[n].skip)
                .collect();
            if failing.is_empty() {
                return;
            }
            let radius = 40.0 * self.grid.g * (1u32 << round) as f64;
            for n in failing {
                if self.cancelled() {
                    return;
                }
                if self.nets[n].unconnected.is_empty() {
                    continue;
                }
                let before = self.unrouted_count();
                // terminals that are cut off; small nets: every terminal
                let mut centres: Vec<P> = vec![];
                for comp in &self.nets[n].comps {
                    if self.nets[n].pins.len() <= 4
                        || comp.iter().any(|p| self.nets[n].unconnected.contains(p))
                    {
                        centres.extend(comp.iter().map(|&p| self.pins[p].c));
                    }
                }
                let near = |pr: &Prim| {
                    centres.iter().any(|c| {
                        pr.bb.0 <= c.0 + radius
                            && pr.bb.2 >= c.0 - radius
                            && pr.bb.1 <= c.1 + radius
                            && pr.bb.3 >= c.1 - radius
                    })
                };
                let area: Vec<usize> = (0..self.nets.len())
                    .filter(|&m| m != n && !self.nets[m].skip && self.nets[m].r.dynp.iter().any(near))
                    .collect();
                if area.is_empty() {
                    continue;
                }
                let saved = self.save_routes();
                let counts: Vec<u32> = self.nets.iter().map(|x| x.rip_count).collect();
                let old_events = self.rip_events;
                self.snapshot(&format!(
                    "Repair area {}: re-routing {} nets around {}",
                    round + 1,
                    area.len(),
                    self.nets[n].name
                ));
                self.rip_events = 0;
                for x in self.nets.iter_mut() {
                    x.rip_count = 0;
                }
                self.rip_net(n);
                for &m in &area {
                    self.rip_net(m);
                }
                let rank = |r: &Router, m: usize| r.route_rank.get(m).copied().unwrap_or(usize::MAX);
                let mut cleared: Vec<usize> = (0..self.nets.len())
                    .filter(|&m| m != n && !self.nets[m].unconnected.is_empty() && !self.nets[m].skip)
                    .collect();
                cleared.sort_by_key(|&m| rank(self, m));
                let mut queue: std::collections::VecDeque<usize> =
                    std::iter::once(n).chain(cleared.iter().copied()).collect();
                let limit = queue.len() * 6 + 40;
                let mut guard = 0;
                while let Some(m) = queue.pop_front() {
                    guard += 1;
                    if guard > limit || self.cancelled() {
                        break;
                    }
                    for v in self.route_net(m, true) {
                        if !queue.contains(&v) {
                            queue.push_back(v);
                        }
                    }
                }
                let open: Vec<usize> = (0..self.nets.len())
                    .filter(|&m| !self.nets[m].unconnected.is_empty() && !self.nets[m].skip)
                    .collect();
                for m in open {
                    if self.cancelled() {
                        break;
                    }
                    self.rip_net(m);
                    self.route_net(m, false);
                }
                let open = self.unrouted_count();
                if self.cfg.verbose {
                    eprintln!(
                        "area repair {} {}: {} nets cleared, open {} -> {}",
                        round + 1,
                        self.nets[n].name,
                        area.len(),
                        before,
                        open
                    );
                }
                if open < before && !self.cancelled() {
                    self.rip_events = old_events;
                    continue;
                }
                self.restore_routes(saved);
                self.rip_events = old_events;
                for (x, c) in self.nets.iter_mut().zip(counts) {
                    x.rip_count = c;
                }
            }
        }
    }

    pub fn cancelled(&self) -> bool {
        self.cfg
            .cancel
            .as_ref()
            .map_or(false, |c| c.load(std::sync::atomic::Ordering::Relaxed))
    }

    pub(crate) fn snapshot(&mut self, label: &str) {
        if self.cfg.snapshot_dir.is_none() && self.cfg.on_snapshot.is_none() {
            return;
        }
        let snap = crate::snapshot::Snapshot::capture(self, label);
        if let Some(cb) = &self.cfg.on_snapshot {
            cb(snap.clone());
        }
        let Some(dir) = self.cfg.snapshot_dir.clone() else {
            return;
        };
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join(format!("snap_{:05}.json", self.snap_count));
        self.snap_count += 1;
        if let Ok(s) = serde_json::to_string(&snap) {
            let _ = std::fs::write(path, s);
        }
    }

    pub fn route_all(&mut self) {
        let t = Instant::now();
        let mut order: Vec<usize> = (0..self.nets.len())
            .filter(|&n| !self.nets[n].skip && self.nets[n].comps.len() >= 2)
            .collect();
        let hpwl = |r: &Router, n: usize| -> f64 {
            let (mut a, mut b, mut c, mut d) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &p in &r.nets[n].pins {
                let (x, y) = r.pins[p].c;
                a = a.min(x);
                b = b.min(y);
                c = c.max(x);
                d = d.max(y);
            }
            (c - a) + (d - b)
        };
        // Route terminals with the fewest legal escape cells first. Short-net
        // ordering alone lets easy tracks fence in fine-pitch IC pins.
        let mut escape = vec![usize::MAX; self.nets.len()];
        if self.cfg.trace_routing {
            for &n in &order {
                self.stamp_net(n, -1);
                let tf = &self.fields[self.nets[n].narrow_tf];
                let vf = &self.fields[self.nets[n].vf];
                for &pi in &self.nets[n].pins {
                    let pin = &self.pins[pi];
                    let mut exits = HashSet::new();
                    let mut can_via = false;
                    for &(l,x,y) in &pin.cells {
                        let ci = y as usize * self.grid.nx + x as usize;
                        if (0..self.grid.nl).all(|ll| !vf.fx(ll*self.grid.n()+ci)) { can_via = true; }
                        for (dx,dy) in [(1,0),(-1,0),(0,1),(0,-1)] {
                            let (xx,yy) = (x+dx,y+dy);
                            if xx<0 || yy<0 || xx>=self.grid.nx as i32 || yy>=self.grid.ny as i32 || self.excl[l] { continue; }
                            let i = self.grid.idx(l,xx,yy);
                            let p = self.grid.center(xx,yy);
                            if !tf.fx(i) && !pin.prims.iter().any(|pr| pr.layer==l && pr.dist(p)<=1e-9) { exits.insert(i); }
                        }
                    }
                    escape[n] = escape[n].min(exits.len() + if can_via { 1000 } else { 0 });
                }
                self.stamp_net(n, 1);
            }
        }
        // Signals first; nets that only need to drop into a plane come last, so their vias
        // take whatever room is left instead of blocking the escape routes of fine-pitch pads.
        order.sort_by(|&a, &b| {
            let key = |n: usize| {
                (
                    self.nets[n].plane.is_some(),
                    escape[n],
                    hpwl(self, n) + self.nets[n].pins.len() as f64,
                )
            };
            key(a).partial_cmp(&key(b)).unwrap()
        });
        self.route_rank = vec![usize::MAX; self.nets.len()];
        for (i, &n) in order.iter().enumerate() {
            self.route_rank[n] = i;
        }
        self.snapshot("start");
        self.route_pass(order.clone(), &t);
        // Connections still open: route those nets FIRST on a clean board and everything else
        // around them. Keep the best of the attempts.
        let mut best = self.save_routes();
        let mut best_open = self.unrouted_count();
        // Local repairs keep routed progress and come first (inside route_pass). When they
        // cannot close a connection, one restart is always allowed: an open net is worse
        // than a longer run, and the best attempt is the one that is kept.
        for attempt in 0..self.cfg.restart_attempts.clamp(1, 3) {
            if best_open == 0 || self.cancelled() || t.elapsed().as_secs_f32() > 150.0 {
                break;
            }
            let mut failing: Vec<usize> = (0..self.nets.len())
                .filter(|&n| !self.nets[n].unconnected.is_empty() && !self.nets[n].skip)
                .collect();
            failing.sort_by_key(|&n| std::cmp::Reverse(self.nets[n].rip_count));
            let mut front: Vec<usize> = failing.clone();
            // earlier attempts are remembered so the same order is not tried twice
            let rot = attempt % front.len().max(1);
            front.rotate_left(rot);
            let mut new_order = front.clone();
            new_order.extend(order.iter().copied().filter(|n| !front.contains(n)));
            self.snapshot(&format!("Retry {}: restarting to resolve {} open connections", attempt + 1, best_open));
            self.reset_routes();
            if self.cfg.verbose {
                eprintln!("restart {}: failing nets first {:?}", attempt + 1, front.iter().map(|&n| self.nets[n].name.clone()).collect::<Vec<_>>());
            }
            self.route_pass(new_order, &t);
            let open = self.unrouted_count();
            if open < best_open {
                best_open = open;
                best = self.save_routes();
            }
        }
        self.restore_routes(best);
        self.last_resorts();
        self.plane_repair();
        self.snapshot("initial routing done");
        if self.cfg.verbose {
            eprintln!(
                "timers: astar {:.2}s ({} searches), reachability {:.2}s ({}), stamping {:.2}s",
                self.t_astar, self.stats_searches, self.t_reach, self.n_reach, self.t_stamp
            );
            eprintln!(
                "initial: {} of {} connections routed, length {:.1}, vias {}, {:.1}s",
                self.connection_count() - self.unrouted_count(),
                self.connection_count(),
                self.total_length(),
                self.num_vias(),
                t.elapsed().as_secs_f32()
            );
        }
    }

    fn save_routes(&self) -> Vec<(Routed, Vec<usize>, bool, bool)> {
        self.nets
            .iter()
            .map(|x| (x.r.clone(), x.unconnected.clone(), x.coupled, x.guided))
            .collect()
    }

    fn reset_routes(&mut self) {
        for n in 0..self.nets.len() {
            self.rip_net(n);
            self.nets[n].rip_count = 0;
        }
        self.rip_events = 0;
    }

    fn restore_routes(&mut self, saved: Vec<(Routed, Vec<usize>, bool, bool)>) {
        self.reset_routes();
        for (i, st) in saved.into_iter().enumerate() {
            self.nets[i].r = st.0;
            let prims = self.nets[i].r.dynp.clone();
            self.stamp_dyn(&prims, 1);
            self.nets[i].unconnected = st.1;
            self.nets[i].coupled = st.2;
            self.nets[i].guided = st.3;
        }
    }

    fn last_resorts(&mut self) {
        self.jump_pass();
    }

    /// The routing loop: every net in `order`, with rip-up queue, retries and the rescue pass.
    fn route_pass(&mut self, order: Vec<usize>, t: &Instant) {
        let mut queue: std::collections::VecDeque<usize> = order.iter().copied().collect();
        let mut count = 0;
        let mut guard = 0;
        let threads = self.spec_threads();
        let mut spec: HashMap<usize, Spec> = HashMap::new();
        let mut retries: HashMap<usize, u8> = HashMap::new();
        while let Some(n) = queue.pop_front() {
            guard += 1;
            if guard > order.len() * 20 + 1000 || self.cancelled() {
                break;
            }
            // Speculative parallel routing: the next few plain nets are routed at the same time on
            // copies of the current state; each result is only used if it is still clean when its
            // turn comes (otherwise the net is routed normally). Deterministic: waves, copies and
            // commit order depend on the queue only.
            let mut done = false;
            if threads > 1 && self.speculable(n) {
                if !spec.contains_key(&n) {
                    let ahead: Vec<usize> = queue.iter().copied().take(WAVE_LOOKAHEAD).collect();
                    let disjoint = std::env::var("AR_WAVE").is_ok_and(|v| v == "1"); // consecutive waves measured faster
                    let wave = self.pick_wave(n, &ahead, threads, disjoint, |me, m| me.speculable(m) && !spec.contains_key(&m));
                    if wave.len() >= 2 {
                        for (m, sp) in self.speculate(&wave, false) {
                            spec.insert(m, sp);
                        }
                    }
                }
                if let Some(sp) = spec.remove(&n) {
                    match self.install_spec(n, &sp) {
                        SpecOutcome::Installed => done = true,
                        SpecOutcome::Conflict => {
                            // a neighbour got there first: try again in a later wave against the
                            // new state instead of routing it here on the main thread
                            let k = retries.entry(n).or_insert(0);
                            if *k < 2 && std::env::var("AR_RETRY").map_or(true, |v| v != "0") {
                                *k += 1;
                                queue.push_back(n);
                                continue;
                            }
                        }
                        SpecOutcome::Unsolved => {}
                    }
                }
            }
            let ripped = if done { vec![] } else { self.route_net(n, true) };
            for m in ripped {
                if !queue.contains(&m) {
                    queue.push_back(m);
                }
            }
            count += 1;
            if self.cfg.verbose {
                eprintln!(
                    "  [{:>3}] {:<10} pins={:<3} unrouted={} len={:.1} vias={} t={:.1}s",
                    count,
                    self.nets[n].name,
                    self.nets[n].pins.len(),
                    self.nets[n].unconnected.len(),
                    self.wire_length(n),
                    self.nets[n].r.vias.len(),
                    t.elapsed().as_secs_f32()
                );
            }
            if self.cfg.snapshot_every > 0 && count % self.cfg.snapshot_every == 0 {
                let label = format!("routed {}", self.nets[n].name);
                self.snapshot(&label);
            }
        }
        if self.cfg.verbose {
            eprintln!("phase main loop done {:.1}s, open {}", t.elapsed().as_secs_f32(), self.unrouted_count());
            eprintln!("speculate(): wall {:.2}s, worker sync {:.2}s, worker routing {:.2}s (sums over workers)", self.spec_time[0], self.spec_time[1], self.spec_time[2]);
            eprintln!("speculative routing: {} tried, {} used, {} not fully connected, {} conflicted", self.spec_stats[0], self.spec_stats[1], self.spec_stats[2], self.spec_stats[3]);
            eprintln!("phase search kinds (window, full, rip-up, thin): {:?}", self.t_cat);
        }
        // retry nets that still have unrouted connections
        for _round in 0..2 {
            if self.cancelled() {
                break;
            }
            let failing: Vec<usize> = (0..self.nets.len())
                .filter(|&n| !self.nets[n].unconnected.is_empty())
                .collect();
            if failing.is_empty() {
                break;
            }
            for n in failing {
                self.snapshot(&format!("Repair pass {}: {}", _round + 1, self.nets[n].name));
                self.rip_net(n);
                let ripped = self.route_net(n, true);
                for m in ripped {
                    self.rip_net(m);
                    self.route_net(m, false);
                }
            }
        }
        if self.cfg.verbose {
            eprintln!("phase retries done {:.1}s, open {}", t.elapsed().as_secs_f32(), self.unrouted_count());
        }
        self.snapshot("Resolving remaining connections (keeping routed progress)");
        self.rescue();
        self.regional_rescue();
        if self.cfg.verbose {
            eprintln!("phase rescue done {:.1}s, open {}", t.elapsed().as_secs_f32(), self.unrouted_count());
        }
    }

    /// Rip up and reroute each net with everything else fixed; keep when cheaper.
    pub fn refine(&mut self) {
        let t = Instant::now();
        for pass in 0..self.cfg.refine_passes {
            let before = self.total_length();
            let mut improved = 0;
            let mut order: Vec<usize> = (0..self.nets.len())
                .filter(|&n| {
                    !self.nets[n].skip
                        && self.nets[n].comps.len() >= 2
                        && self.nets[n].plane.is_none()
                })
                .collect();
            order.sort_by(|&a, &b| self.net_cost(b).partial_cmp(&self.net_cost(a)).unwrap());
            let threads = self.spec_threads();
            let mut i = 0;
            while i < order.len() {
                if self.cancelled() {
                    break;
                }
                // next wave: nets that really need a second look, routed in parallel; nets whose
                // areas overlap are left for a later wave
                let needs_look = |me: &Router, n: usize| {
                    !(me.nets[n].coupled || me.nets[n].guided)
                        && !(me.nets[n].unconnected.is_empty() && me.wire_length(n) <= 1.10 * me.net_bound(n))
                };
                let mut first = None;
                while i < order.len() {
                    let n = order[i];
                    i += 1;
                    if needs_look(self, n) {
                        first = Some(n);
                        break;
                    }
                }
                let Some(first) = first else { break };
                let ahead: Vec<usize> = order[i..].iter().copied().take(WAVE_LOOKAHEAD).collect();
                let wave: Vec<usize> = if threads > 1 {
                    self.pick_wave(first, &ahead, threads, false, |me, m| needs_look(me, m) && me.refinable(m))
                } else {
                    vec![first]
                };
                if wave.len() > 1 {
                    order.retain(|m| !wave[1..].contains(m) || false);
                    // keep the position bookkeeping simple: removed entries all lie after `i`
                    i = i.min(order.len());
                }
                let mut specs: HashMap<usize, Spec> = HashMap::new();
                if threads > 1 && wave.len() >= 2 {
                    specs = self.speculate(&wave, true).into_iter().collect();
                    self.spec_stats[0] += wave.len();
                }
                let n_wave = wave.len();
                let _ = n_wave;
                for n in wave {
                    let old_cost = self.net_cost(n);
                    let old_unc = self.nets[n].unconnected.len();
                    if let Some(sp) = specs.remove(&n) {
                        let new_cost = self.routed_cost(&sp.r);
                        let better = sp.unconnected.len() < old_unc
                            || (sp.unconnected.len() == old_unc && new_cost < old_cost - 1e-6);
                        if better && !sp.r.dynp.is_empty() && self.prims_conflict(&sp.r.dynp).is_none() {
                            self.stamp_net(n, -1);
                            self.nets[n].r = sp.r;
                            self.nets[n].unconnected = sp.unconnected;
                            self.stamp_net(n, 1);
                            improved += 1;
                            self.spec_stats[1] += 1;
                        }
                        continue;
                    }
                    let old = (self.nets[n].r.clone(), self.nets[n].unconnected.clone());
                    self.rip_net(n);
                    self.route_net(n, false);
                    let new_cost = self.net_cost(n);
                    let better = self.nets[n].unconnected.len() < old_unc
                        || (self.nets[n].unconnected.len() == old_unc && new_cost < old_cost - 1e-6);
                    if better {
                        improved += 1;
                    } else {
                        // restore
                        self.rip_net(n);
                        self.stamp_net(n, -1);
                        self.nets[n].r = old.0;
                        self.nets[n].unconnected = old.1;
                        self.stamp_net(n, 1);
                    }
                }
            }
            if self.cfg.verbose {
                eprintln!(
                    "refine pass {}: improved {} nets, length {:.1} -> {:.1} ({:.1}s) [parallel tries so far {}, used {}]",
                    pass + 1,
                    improved,
                    before,
                    self.total_length(),
                    t.elapsed().as_secs_f32(),
                    self.spec_stats[0],
                    self.spec_stats[1]
                );
            }
            self.snapshot(&format!("refine pass {}", pass + 1));
            if improved == 0 {
                break;
            }
        }
    }
}

/// How far ahead in the queue a wave looks for nets that do not get in each other's way.
const WAVE_LOOKAHEAD: usize = 48;
/// Nets closer than this (DSN units) are not routed in the same wave.
const WAVE_MARGIN: f64 = 12.0;

/// What became of a worker's result.
#[derive(PartialEq)]
pub(crate) enum SpecOutcome {
    Installed,
    /// the worker could not connect the net without rip-up
    Unsolved,
    /// the result collided with a net routed in the meantime
    Conflict,
}

/// Result of routing one net on a worker copy.
pub(crate) struct Spec {
    r: Routed,
    unconnected: Vec<usize>,
}

impl Clone for Router {
    /// A full copy for use as a worker (no GPU context, no snapshot output, no workers of its own).
    fn clone(&self) -> Router {
        let mut cfg = self.cfg.clone();
        cfg.on_snapshot = None;
        cfg.snapshot_dir = None;
        cfg.verbose = false;
        Router {
            ground_fill_length: self.ground_fill_length,
            cfg,
            grid: self.grid.clone(),
            layer_names: self.layer_names.clone(),
            excl: self.excl.clone(),
            fields: self.fields.clone(),
            congestion: self.congestion.clone(),
            nets: self.nets.clone(),
            pins: self.pins.clone(),
            fixed: self.fixed.clone(),
            term: self.term.clone(),
            res_unit: self.res_unit.clone(),
            res: self.res,
            unit_mm: self.unit_mm,
            min_width: self.min_width,
            guide: self.guide.clone(),
            via_names: self.via_names.clone(),
            board_name: self.board_name.clone(),
            buf: SearchBuf::default(),
            snap_count: 0,
            t0: self.t0,
            stats_searches: 0,
            t_astar: 0.0,
            t_cat: [0.0; 4],
            pop_budget: self.pop_budget,
            jump_mode: self.jump_mode,
            pad_via_mode: self.pad_via_mode,
            h_weight: self.h_weight,
            via_shortfall: 0,
            jumps_used: 0,
            pad_vias_used: 0,
            fill_done: vec![],
            fill_export: vec![],
            fill_polys: vec![],
            outline_stats: (0, 0),
            pour_skip: vec![],
            pour_warnings: vec![],
            part_moves: vec![],
            prof: vec![],
            pops_total: 0,
            workers: vec![],
            spec_stats: [0; 4],
            spec_time: [0.0; 3],
            t_reach: 0.0,
            t_stamp: 0.0,
            n_reach: 0,
            rip_events: self.rip_events,
            boundary: self.boundary.clone(),
            planes: self.planes.clone(),
            bundles: self.bundles.clone(),
            plane_id: self.plane_id.clone(),
            cur_net: usize::MAX,
            cur_land: false,
            route_rank: self.route_rank.clone(),
            crowd: self.crowd.clone(),
        }
    }
}

impl Router {
    /// Worker threads for speculative routing (`AR_THREADS` overrides; 1 = off).
    pub(crate) fn spec_threads(&self) -> usize {
        std::env::var("AR_THREADS")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(8) // fixed: the wave size shapes the result, so it must not depend on the machine
            .max(1)
    }

    /// Bounding box of a net's pins, grown by `WAVE_MARGIN`.
    fn net_area(&self, n: usize) -> (f64, f64, f64, f64) {
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for &p in &self.nets[n].pins {
            let (x, y) = self.pins[p].c;
            b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
        }
        (b.0 - WAVE_MARGIN, b.1 - WAVE_MARGIN, b.2 + WAVE_MARGIN, b.3 + WAVE_MARGIN)
    }

    /// `first` plus up to `max - 1` nets from `ahead` (in order) whose areas are disjoint.
    fn pick_wave(&self, first: usize, ahead: &[usize], max: usize, disjoint: bool, ok: impl Fn(&Router, usize) -> bool) -> Vec<usize> {
        let mut wave = vec![first];
        let mut boxes = vec![self.net_area(first)];
        for &m in ahead {
            if wave.len() >= max {
                break;
            }
            if m == first || wave.contains(&m) || !ok(self, m) {
                continue;
            }
            let b = self.net_area(m);
            if !disjoint || boxes.iter().all(|o| b.0 > o.2 || o.0 > b.2 || b.1 > o.3 || o.1 > b.3) {
                wave.push(m);
                boxes.push(b);
            }
        }
        wave
    }

    /// Plain nets only: no pairs, groups or planes, nothing routed yet.
    fn speculable(&self, n: usize) -> bool {
        let ns = &self.nets[n];
        !ns.skip
            && ns.comps.len() >= 2
            && ns.pair.is_none()
            && ns.bundle.is_none()
            && ns.plane.is_none()
            && !ns.coupled
            && !ns.guided
            && ns.r.dynp.is_empty()
            && !self.jump_mode
            && !self.pad_via_mode
    }

    fn refinable(&self, n: usize) -> bool {
        let ns = &self.nets[n];
        !ns.skip && ns.comps.len() >= 2 && ns.pair.is_none() && ns.bundle.is_none() && ns.plane.is_none() && !ns.coupled && !ns.guided
    }

    fn routed_cost(&self, r: &Routed) -> f64 {
        let len: f64 = r
            .wires
            .iter()
            .filter(|w| !w.fill)
            .map(|w| w.pts.windows(2).map(|s| dist(s[0], s[1])).sum::<f64>())
            .sum();
        len + r.vias.len() as f64 * self.cfg.via_cost as f64 * self.grid.g
    }

    /// Bring a worker up to the state of `m` (only what routing changes).
    fn sync_from(&mut self, m: &Router) {
        for (a, b) in self.fields.iter_mut().zip(&m.fields) {
            a.cnt.clone_from(&b.cnt);
            a.mask.clone_from(&b.mask);
            a.col.clone_from(&b.col);
        }
        self.congestion.clone_from(&m.congestion);
        self.nets.clone_from(&m.nets);
        self.rip_events = m.rip_events;
        self.cfg.via_cost = m.cfg.via_cost;
        self.cfg.allow_vias_in_pads = m.cfg.allow_vias_in_pads;
        self.h_weight = m.h_weight;
        self.jump_mode = m.jump_mode;
        self.pad_via_mode = m.pad_via_mode;
        self.pop_budget = m.pop_budget;
    }

    /// Route each of `nets` (without rip-up) on its own copy of the current state, in parallel.
    /// With `rip_first` the net's existing copper is removed first (refinement).
    pub(crate) fn speculate(&mut self, nets: &[usize], rip_first: bool) -> Vec<(usize, Spec)> {
        let t_all = Instant::now();
        let mut workers = std::mem::take(&mut self.workers);
        while workers.len() < nets.len() {
            workers.push(self.clone());
        }
        let master: &Router = self;
        let times = std::sync::Mutex::new([0.0f32; 2]);
        let times_ref = &times;
        let out: Vec<(usize, Spec)> = std::thread::scope(|s| {
            let handles: Vec<_> = workers
                .iter_mut()
                .zip(nets.iter().copied())
                .map(|(w, n)| {
                    s.spawn(move || {
                        let t0 = Instant::now();
                        w.sync_from(master);
                        let t1 = Instant::now();
                        if rip_first {
                            w.rip_net(n);
                        }
                        w.route_net_plain(n, false);
                        if let Ok(mut t) = times_ref.lock() {
                            t[0] += (t1 - t0).as_secs_f32();
                            t[1] += t1.elapsed().as_secs_f32();
                        }
                        (n, Spec { r: w.nets[n].r.clone(), unconnected: w.nets[n].unconnected.clone() })
                    })
                })
                .collect();
            handles.into_iter().filter_map(|h| h.join().ok()).collect()
        });
        self.workers = workers;
        self.spec_time[0] += t_all.elapsed().as_secs_f32();
        if let Ok(t) = times.lock() {
            self.spec_time[1] += t[0];
            self.spec_time[2] += t[1];
        }
        out
    }

    /// Use a worker's result if the net is still unrouted, fully connected and clean against
    /// everything routed meanwhile.
    pub(crate) fn install_spec(&mut self, n: usize, sp: &Spec) -> SpecOutcome {
        self.spec_stats[0] += 1;
        if !self.speculable(n) || !sp.unconnected.is_empty() || sp.r.dynp.is_empty() {
            self.spec_stats[2] += 1;
            return SpecOutcome::Unsolved;
        }
        if self.prims_conflict(&sp.r.dynp).is_some() {
            self.spec_stats[3] += 1;
            return SpecOutcome::Conflict;
        }
        self.spec_stats[1] += 1;
        self.stamp_net(n, -1);
        self.nets[n].r = sp.r.clone();
        self.nets[n].unconnected = vec![];
        self.stamp_net(n, 1);
        SpecOutcome::Installed
    }
}

trait PinXY {
    fn x_y_of(&self, pr: &Prim) -> P;
}
impl PinXY for dsn::AbsPin {
    fn x_y_of(&self, pr: &Prim) -> P {
        let bb = pr.bb;
        ((bb.0 + bb.2) / 2.0, (bb.1 + bb.3) / 2.0)
    }
}

#[cfg(test)]
mod stamp_tests {
    use super::*;

    #[test]
    fn clipped_stamps_match_exhaustive_geometry_and_unstamp() {
        let g = Grid { x0: -12.3, y0: -10.7, g: 0.37, nx: 97, ny: 89, nl: 2 };
        for a in [(-15.0, -8.0), (0.0, 0.0), (18.2, 20.1)] {
            for b in [(-14.0, 19.0), (21.0, -11.0), a, (a.0, 23.0), (23.0, a.1)] {
                for hw in [0.0, 0.3, 2.7] {
                    for dynamic in [false, true] {
                        let pr = Prim::new(1, Geo::Seg { a, b, hw }, 0, 0.43);
                        let mut f = Field::new(&g, 0.61, 0.23, false);
                        f.stamp(&g, &pr, 1, dynamic);
                        let unit = if dynamic { 65536 } else { 1 };
                        for y in 0..g.ny as i32 {
                            for x in 0..g.nx as i32 {
                                let expected = if pr.dist(g.center(x, y)) < f.threshold(&pr) { unit } else { 0 };
                                assert_eq!(f.cnt[g.idx(1, x, y)], expected, "{a:?} {b:?} {hw} at {x},{y}");
                                assert_eq!(f.cnt[g.idx(0, x, y)], 0);
                            }
                        }
                        f.stamp(&g, &pr, -1, dynamic);
                        assert!(f.cnt.iter().all(|&v| v == 0));
                    }
                }
            }
        }
    }
}
