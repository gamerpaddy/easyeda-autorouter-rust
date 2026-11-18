//! autorouter-gui: open a DSN, set routing options, preview, route with live progress, save SES.
//!   `autorouter-gui [board.dsn]`
//!   `autorouter-gui board.dsn --svg out.svg [--snapshots DIR] [--index N] [--only NET1,NET2]`  (headless export)

#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use dsn::{AbsShape, Board};
use eframe::egui::{self, Color32, Pos2, Stroke};
use router::preview::Preview;
use router::snapshot::Snapshot;
use router::{drc, ses, Config, NetOverride, Router};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};

const LAYER_COLORS: [(u8, u8, u8); 8] = [
    (220, 60, 60),
    (60, 160, 230),
    (240, 200, 50),
    (70, 200, 110),
    (200, 90, 220),
    (240, 140, 50),
    (90, 220, 220),
    (160, 160, 160),
];

fn lc(l: usize) -> (u8, u8, u8) {
    LAYER_COLORS[l % LAYER_COLORS.len()]
}
/// A polygon of any shape: closed outline plus horizontal hatching (even-odd), so concave
/// regions are filled correctly without triangulating them.
fn hatch_polygon(painter: &egui::Painter, v: &[Pos2], fill: Color32, edge: Stroke, step: f32) {
    if v.len() < 3 {
        return;
    }
    painter.add(egui::Shape::closed_line(v.to_vec(), edge));
    let (mut y0, mut y1) = (f32::MAX, f32::MIN);
    for p in v {
        y0 = y0.min(p.y);
        y1 = y1.max(p.y);
    }
    let step = step.max(2.0);
    let mut y = (y0 / step).ceil() * step + 0.01;
    let mut guard = 0;
    while y < y1 && guard < 4000 {
        guard += 1;
        let mut xs: Vec<f32> = vec![];
        for i in 0..v.len() {
            let (a, b) = (v[i], v[(i + 1) % v.len()]);
            if (a.y > y) != (b.y > y) {
                xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        }
        xs.sort_by(|a, b| a.total_cmp(b));
        for pair in xs.chunks_exact(2) {
            painter.line_segment([Pos2::new(pair[0], y), Pos2::new(pair[1], y)], Stroke::new(1.0, fill));
        }
        y += step;
    }
}

fn col(l: usize, a: u8) -> Color32 {
    let (r, g, b) = lc(l);
    Color32::from_rgba_unmultiplied(r, g, b, a)
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
enum DispUnit {
    #[default]
    Mil,
    Mm,
}

impl DispUnit {
    fn mm(self) -> f64 {
        match self {
            DispUnit::Mil => 0.0254,
            DispUnit::Mm => 1.0,
        }
    }
    fn label(self) -> &'static str {
        match self {
            DispUnit::Mil => "mil",
            DispUnit::Mm => "mm",
        }
    }
}

/// Settings that belong to one board file.
#[derive(Clone, Default, Serialize, Deserialize)]
#[serde(default)]
struct BoardSaved {
    net_overrides: HashMap<String, NetOverride>,
    group_tolerances: HashMap<String, f64>,
    layers: HashMap<String, router::LayerPlan>,
    exclusive: Vec<String>,
    exclusive_planes: Vec<String>,
    unit_mm: Option<f64>,
}

/// Everything kept between restarts (`%LOCALAPPDATA%/Autorouter/settings.json`).
#[derive(Serialize, Deserialize)]
#[serde(default)]
struct Settings {
    /// settings layout version; older files get new defaults for changed options
    version: u32,
    cfg: Config,
    unit: DispUnit,
    show_pads: bool,
    show_planes: bool,
    show_airwires: bool,
    last_path: String,
    options_path: String,
    boards: HashMap<String, BoardSaved>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            version: 2,
            cfg: Config {
                verbose: false,
                ..Config::default()
            },
            unit: DispUnit::Mil,
            show_pads: true,
            show_planes: true,
            show_airwires: true,
            last_path: String::new(),
            options_path: "route_options.json".into(),
            boards: HashMap::new(),
        }
    }
}

fn settings_path() -> PathBuf {
    std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("Autorouter")
        .join("settings.json")
}

fn layer_label(name: &str) -> String {
    match name {
        "1" => "Top (1)".into(),
        "2" => "Bottom (2)".into(),
        "21" => "Inner 1 (21)".into(),
        "22" => "Inner 2 (22)".into(),
        // EasyEDA Pro project: inner layers are named In1, In2, ...
        n if n.starts_with("In") && n[2..].parse::<u32>().is_ok() => format!("Inner {}", &n[2..]),
        _ => name.to_string(),
    }
}

/// Drag value shown in display units; `k` = display units per DSN unit.
fn dv(ui: &mut egui::Ui, v: &mut f64, k: f64, lo: f64, hi: f64, speed: f64, suffix: &str) -> bool {
    let mut d = *v * k;
    let mut w = egui::DragValue::new(&mut d)
        .speed(speed * k)
        .range(lo * k..=hi * k)
        .max_decimals(4);
    if !suffix.is_empty() {
        w = w.suffix(format!(" {}", suffix));
    }
    if ui.add(w).changed() {
        *v = d / k;
        true
    } else {
        false
    }
}

/// Keep the popup open for text editing and multiple net selections.
fn net_picker(ui: &mut egui::Ui, id: egui::Id, filter: &mut String, names: &[String], assigned: &mut Vec<String>) -> egui::InnerResponse<Option<(bool, egui::Rect)>> {
    egui::ComboBox::from_id_salt(id)
        .selected_text("+ net")
        .close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)
        .show_ui(ui, |ui| {
            let search = ui.add(egui::TextEdit::singleline(filter).hint_text("Search nets (for example GND)"));
            let query = filter.trim().to_lowercase();
            let mut changed = false;
            for n in names {
                if !n.to_lowercase().contains(&query) { continue; }
                if !assigned.contains(n) && ui.selectable_label(false, n).clicked() {
                    assigned.push(n.clone()); changed = true;
                }
            }
            (changed, search.rect)
        })
}

struct Pad {
    name: String,
    center: (f64, f64),
    layer: usize,
    net: i32,
    shape: AbsShape,
}

impl Pad {
    fn offset(&self, positions: &HashMap<&str, (f64, f64)>) -> (f64, f64) {
        positions.get(self.name.as_str()).map_or((0.0, 0.0), |p|
            (p.0 - self.center.0, p.1 - self.center.1))
    }
}

enum Msg {
    Snap(Box<Snapshot>),
    Done(Box<RunResult>),
}

/// One clickable line of the report.
struct Issue {
    text: String,
    pos: (f64, f64),
    net: Option<usize>,
    error: bool,
}

struct RunResult {
    ses: String,
    /// routed EasyEDA source JSON or Pro project (when the board was loaded from one)
    json: Option<Vec<u8>>,
    /// `json` is an EasyEDA Pro project (.epro2 / .epro), not Standard JSON text
    pro: bool,
    text: String,
    export_ok: bool,
    options: String,
    issues: Vec<Issue>,

}

/// Ear-clipping triangulation of a simple polygon (indices into `pts`).
fn triangulate(pts: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let n = pts.len();
    if n < 3 {
        return vec![];
    }
    let area: f64 = (0..n).map(|i| pts[i].0 * pts[(i + 1) % n].1 - pts[(i + 1) % n].0 * pts[i].1).sum();
    let mut idx: Vec<usize> = if area >= 0.0 { (0..n).collect() } else { (0..n).rev().collect() };
    let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    let mut out = vec![];
    let mut guard = 0;
    while idx.len() > 3 && guard < n * n {
        guard += 1;
        let m = idx.len();
        let mut clipped = false;
        for k in 0..m {
            let (a, b, c) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
            if cross(pts[a], pts[b], pts[c]) <= 1e-12 {
                continue;
            }
            let inside = idx.iter().any(|&p| {
                p != a && p != b && p != c && pts[p] != pts[a] && pts[p] != pts[b] && pts[p] != pts[c]
                    && cross(pts[a], pts[b], pts[p]) >= 0.0
                    && cross(pts[b], pts[c], pts[p]) >= 0.0
                    && cross(pts[c], pts[a], pts[p]) >= 0.0
            });
            if !inside {
                out.push([a, b, c]);
                idx.remove(k);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // degenerate (touching slits): drop one vertex and go on
            idx.remove(0);
        }
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out
}

struct App {
    path: String,
    /// the EasyEDA source document the board was loaded from, if any
    easyeda: Option<Arc<dsn::easyeda::EasyedaDoc>>,
    // Immutable board geometry is shared by the UI and routing worker.
    board: Option<Arc<Board>>,
    preview: Option<Preview>,
    pads: Vec<Pad>,
    pin_pos: HashMap<String, (f64, f64)>,
    cfg: Config,
    // run state
    rx: Option<mpsc::Receiver<Msg>>,
    cancel: Arc<AtomicBool>,
    running: bool,
    snaps: Vec<Snapshot>,
    cur: usize,
    follow: bool,
    playing: bool,
    last_tick: f64,
    result: Option<RunResult>,
    // view
    layer_vis: Vec<bool>,
    scale: f32,
    off: egui::Vec2,
    fitted: bool,
    sel_net: Option<usize>,
    filter: String,
    bulk_net_source: Option<usize>,
    bulk_net_values: [f64; 2],
    bulk_net_fields: [bool; 2],
    bulk_net_matching: bool,
    layer_net_filter: HashMap<usize, String>,
    show_pads: bool,
    show_airwires: bool,
    show_planes: bool,
    msg: String,
    options_path: String,
    unit: DispUnit,
    unit_mm: f64,
    /// None = all layers, Some(l) = only that layer's copper (pads always stay visible)
    view_layer: Option<usize>,
    multi_sel: HashSet<usize>,
    dirty_preview: bool,
    boards: HashMap<String, BoardSaved>,
    saved_json: String,
    /// spot to highlight (clicked report line)
    focus: Option<(f64, f64)>,
    canvas_rect: egui::Rect,
}

fn load_snaps(dir: &PathBuf) -> Vec<Snapshot> {
    let mut files: Vec<_> = std::fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| {
                    p.extension().map_or(false, |x| x == "json")
                        && p.file_name()
                            .map_or(false, |n| n.to_string_lossy().starts_with("snap_"))
                })
                .collect()
        })
        .unwrap_or_default();
    files.sort();
    files
        .iter()
        .filter_map(|f| serde_json::from_str::<Snapshot>(&std::fs::read_to_string(f).ok()?).ok())
        .collect()
}

fn pads_of(board: &Board) -> (Vec<Pad>, HashMap<String, (f64, f64)>) {
    let mut pin_net: HashMap<String, i32> = HashMap::new();
    for (i, n) in board.nets.iter().enumerate() {
        for p in &n.pins {
            pin_net.insert(format!("{}-{}", p.comp, p.pin), i as i32);
        }
    }
    let mut pads = vec![];
    let mut pos = HashMap::new();
    for (k, ap) in &board.abs_pins() {
        pos.insert(k.clone(), (ap.x, ap.y));
        for (l, sh) in &ap.shapes {
            pads.push(Pad {
                name: k.clone(),
                center: (ap.x, ap.y),
                layer: *l,
                net: pin_net.get(k).copied().unwrap_or(-1),
                shape: sh.clone(),
            });
        }
    }
    (pads, pos)
}

impl App {
    fn new() -> App {
        let st: Settings = std::fs::read_to_string(settings_path())
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        let mut cfg = st.cfg.clone();
        if st.version < 2 {
            cfg.plane_reach_mm = Config::default().plane_reach_mm;
            cfg.plane_jump_mm = Config::default().plane_jump_mm;
        }
        cfg.verbose = false;
        cfg.net_overrides.clear();
        cfg.layers.clear();
        cfg.group_tolerances.clear();
        cfg.exclusive.clear();
        cfg.exclusive_planes.clear();
        cfg.unit_mm = None;
        App {
            unit: st.unit,
            unit_mm: 0.0254,
            view_layer: None,
            multi_sel: HashSet::new(),
            dirty_preview: false,
            boards: st.boards,
            saved_json: String::new(),
            focus: None,
            canvas_rect: egui::Rect::NOTHING,
            path: st.last_path.clone(),
            board: None,
            easyeda: None,
            preview: None,
            pads: vec![],
            pin_pos: HashMap::new(),
            cfg,
            rx: None,
            cancel: Arc::new(AtomicBool::new(false)),
            running: false,
            snaps: vec![],
            cur: 0,
            follow: true,
            playing: false,
            last_tick: 0.0,
            result: None,
            layer_vis: vec![],
            scale: 1.0,
            off: egui::Vec2::ZERO,
            fitted: false,
            sel_net: None,
            filter: String::new(),
            bulk_net_source: None,
            bulk_net_values: [0.0; 2],
            bulk_net_fields: [true; 2],
            bulk_net_matching: false,
            layer_net_filter: HashMap::new(),
            show_pads: st.show_pads,
            show_planes: st.show_planes,
            show_airwires: st.show_airwires,
            msg: String::new(),
            options_path: st.options_path.clone(),
        }
    }

    fn load(&mut self, path: &str) {
        let abs = std::path::absolute(path)
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|_| path.to_string());
        let path = abs.as_str();
        let is_pro = path.to_lowercase().ends_with(".epro2") || path.to_lowercase().ends_with(".epro");
        let raw = match std::fs::read(path) {
            Ok(b) => b,
            Err(e) => {
                self.msg = format!("cannot read {}: {}", path, e);
                log_error(&self.msg);
                return;
            }
        };
        // an .epro2 / .epro is a ZIP archive, everything else is text
        let src = if is_pro {
            String::new()
        } else {
            match String::from_utf8(raw.clone()) {
                Ok(s) => s,
                Err(e) => {
                    self.msg = format!("cannot read {}: {}", path, e);
                    log_error(&self.msg);
                    return;
                }
            }
        };
        let is_json = is_pro || path.to_lowercase().ends_with(".json");
        let mut doc: Option<Arc<dsn::easyeda::EasyedaDoc>> = None;
        let parsed = if is_json {
            match if is_pro { dsn::easyeda_pro::load(&raw) } else { dsn::easyeda::load(&src) } {
                Ok(d) => {
                    let b = dsn::parse_dsn(&d.dsn);
                    doc = Some(Arc::new(d));
                    b
                }
                Err(e) => {
                    self.msg = format!("EasyEDA file error: {}", e);
                    log_error(&self.msg);
                    return;
                }
            }
        } else {
            dsn::parse_dsn(&src)
        };
        let board = match parsed {
            Ok(b) => b,
            Err(e) => {
                self.msg = format!("DSN error: {}", e);
                log_error(&self.msg);
                return;
            }
        };
        let (pads, pos) = pads_of(&board);
        self.bulk_net_source = None;
        self.bulk_net_values = [0.0; 2];
        // exclusive defaults: power-type layers
        self.cfg.exclusive = board
            .layers
            .iter()
            .filter(|l| l.power)
            .map(|l| l.name.clone())
            .collect();
        self.cfg.exclusive_planes.clear();
        self.cfg.net_overrides.clear();
        self.cfg.group_tolerances.clear();
        self.cfg.layers.clear();
        self.cfg.unit_mm = None;
        // detected pairs become numbered groups 1, 2, 3 ...
        for (k, (a, b)) in dsn::find_diff_pairs(&board.nets).into_iter().enumerate() {
            for i in [a, b] {
                self.cfg
                    .net_overrides
                    .entry(board.nets[i].name.clone())
                    .or_default()
                    .diff_group = (k + 1).to_string();
            }
        }
        // settings remembered for this board file
        if let Some(bs) = self.boards.get(path).cloned() {
            let known = |n: &String| board.nets.iter().any(|x| &x.name == n);
            self.cfg.net_overrides = bs
                .net_overrides
                .into_iter()
                .filter(|(n, _)| known(n))
                .collect();
            self.cfg.group_tolerances = bs.group_tolerances;
            self.cfg.layers = bs
                .layers
                .into_iter()
                .filter(|(l, _)| board.layers.iter().any(|x| &x.name == l))
                .map(|(l, mut p)| {
                    p.nets.retain(|n| known(n));
                    (l, p)
                })
                .collect();
            self.cfg.exclusive = bs.exclusive;
            self.cfg.exclusive_planes = bs.exclusive_planes;
            self.cfg.unit_mm = bs.unit_mm;
        }
        // EasyEDA JSON keeps track widths and can hold copper areas: real widths, real fills
        self.cfg.keep_track_widths = doc.is_some();
        self.cfg.pour_polygons = doc.is_some();
        if doc.is_some() {
            self.cfg.power_width_max_mm = 0.0;
        }
        self.cfg.use_trace_routing();
        self.easyeda = doc;
        self.unit_mm = self.cfg.unit_mm.unwrap_or_else(|| board.detect_unit_mm().0);
        self.multi_sel.clear();
        self.view_layer = None;
        if let Err(e) = self.cfg.validate(&board) {
            log_error(&e);
            self.msg = e;
            return;
        }
        let light = Router::new_with(
            &board,
            Config {
                verbose: false,
                ..self.cfg.clone()
            },
            true,
        );
        self.preview = Some(light.preview());
        self.layer_vis = vec![true; board.layers.len()];
        self.pads = pads;
        self.pin_pos = pos;
        self.snaps.clear();
        self.result = None;
        self.cur = 0;
        self.fitted = false;
        self.sel_net = None;
        self.path = path.to_string();
        self.msg = format!(
            "{}: {} nets, {} layers, {} existing wire path(s), {} plane(s)",
            path,
            board.nets.len(),
            board.layers.len(),
            board.wires.len(),
            board.planes.len()
        );
        if let Some(d) = &self.easyeda {
            for n in &d.notes {
                self.msg += &format!("
{}", n);
            }
        }
        self.board = Some(Arc::new(board));
    }

    /// Rebuild the pre-route analysis after layer roles, units or rules changed.
    fn refresh_preview(&mut self) {
        self.cfg.use_trace_routing();
        self.dirty_preview = false;
        let Some(board) = &self.board else { return };
        if self.cfg.validate(board).is_err() {
            return;
        }
        let light = Router::new_with(
            board,
            Config {
                verbose: false,
                ..self.cfg.clone()
            },
            true,
        );
        self.unit_mm = light.unit_mm;
        self.preview = Some(light.preview());
    }

    fn settings_json(&self) -> String {
        let mut cfg = self.cfg.clone();
        cfg.verbose = false;
        let mut boards = self.boards.clone();
        if self.board.is_some() && !self.path.is_empty() {
            boards.insert(
                self.path.clone(),
                BoardSaved {
                    net_overrides: cfg.net_overrides.clone(),
                    group_tolerances: cfg.group_tolerances.clone(),
                    layers: cfg.layers.clone(),
                    exclusive: cfg.exclusive.clone(),
                    exclusive_planes: cfg.exclusive_planes.clone(),
                    unit_mm: cfg.unit_mm,
                },
            );
        }
        cfg.net_overrides.clear();
        cfg.group_tolerances.clear();
        cfg.layers.clear();
        cfg.exclusive.clear();
        cfg.exclusive_planes.clear();
        cfg.unit_mm = None;
        serde_json::to_string_pretty(&Settings {
            version: 2,
            cfg,
            unit: self.unit,
            show_pads: self.show_pads,
            show_planes: self.show_planes,
            show_airwires: self.show_airwires,
            last_path: self.path.clone(),
            options_path: self.options_path.clone(),
            boards,
        })
        .unwrap_or_default()
    }

    /// Write the settings file whenever something changed.
    fn save_settings(&mut self) {
        let j = self.settings_json();
        if j != self.saved_json {
            let p = settings_path();
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            if std::fs::write(&p, &j).is_ok() {
                self.saved_json = j;
            }
        }
    }

    fn start(&mut self) {
        self.cfg.use_trace_routing();
        let Some(board) = self.board.clone() else {
            return;
        };
        if let Err(e) = self.cfg.validate(&board) {
            log_error(&e);
            self.msg = e;
            return;
        }
        let (tx, rx) = mpsc::channel::<Msg>();
        self.cancel = Arc::new(AtomicBool::new(false));
        let mut cfg = self.cfg.clone();
        cfg.verbose = false;
        cfg.cancel = Some(self.cancel.clone());
        let txs = Mutex::new(tx.clone());
        cfg.on_snapshot = Some(Arc::new(move |s: Snapshot| {
            if let Ok(t) = txs.lock() {
                let _ = t.send(Msg::Snap(Box::new(s)));
            }
        }));
        let name = std::path::Path::new(&self.path)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "board".into());
        self.snaps.clear();
        self.result = None;
        self.cur = 0;
        self.follow = true;
        self.running = true;
        self.rx = Some(rx);
        let doc = self.easyeda.clone();
        std::thread::spawn(move || {
            let parts = doc.as_ref().map(|d| router::nudge::specs_from_doc(d)).unwrap_or_default();
            let skip: Vec<(String, String)> = doc.as_ref().map(|d| d.areas.iter().cloned().collect()).unwrap_or_default();
            let solved = router::nudge::solve(&board, cfg, &parts, &skip);
            let nudge_log = solved.log.clone();
            let r = solved.router;
            let rep = drc::check(&r);
            let mut text = String::new();
            for l in &nudge_log {
                text += &format!("{l}
");
            }
            text += &format!(
                "routed {}/{} connections, length {:.1}, vias {}, {:.1}s\n",
                r.connection_count() - r.unrouted_count(),
                r.connection_count(),
                r.total_length(),
                r.num_vias(),
                r.t0.elapsed().as_secs_f32()
            );
            let pre: usize = r.nets.iter().map(|n| n.pre.wires.len()).sum();
            if pre > 0 {
                text += &format!("kept {} existing wire path(s) untouched\n", pre);
            }
            let lb = r.length_lower_bound();
            if lb > 0.0 {
                text += &format!("length vs MST lower bound: {:.3}x\n", r.total_length() / lb);
            }
            for p in r.plane_report() {
                text += &format!(
                    "plane {}: {} piece(s) with pins, {} dead piece(s){}\n",
                    p.name,
                    p.islands,
                    p.orphans,
                    if p.islands > 1 || p.orphans > 0 {
                        "  <-- cut by routing"
                    } else {
                        ""
                    }
                );
            }
            text += &format!(
                "DRC: {} clearance violations ({} shorts), {} open nets\n",
                rep.clearance_violations.len(),
                rep.shorts,
                rep.open_nets.len()
            );
            let mut issues: Vec<Issue> = vec![];
            for (v, pos) in rep.clearance_violations.iter().zip(&rep.violation_pos) {
                issues.push(Issue {
                    text: format!("clearance: {}", v),
                    pos: *pos,
                    net: None,
                    error: true,
                });
            }
            for (o, (net, pos)) in rep.open_nets.iter().zip(&rep.open_pos) {
                issues.push(Issue {
                    text: format!("open: {}", o),
                    pos: *pos,
                    net: Some(*net),
                    error: true,
                });
            }
            for p in r.pair_report() {
                let line = format!(
                    "{} {}{}{}: {} (skew {:.2})",
                    if p.n.is_empty() { "group" } else { "pair" },
                    p.p,
                    if p.n.is_empty() { "" } else { "/" },
                    p.n,
                    if p.matched {
                        format!("{}, length matched", p.kind)
                    } else if p.coupled {
                        format!("{}, exceeds length tolerance", p.kind)
                    } else {
                        "NOT routed together".to_string()
                    },
                    p.len_p - p.len_n
                );
                if p.matched {
                    text += &format!("{}\n", line);
                } else {
                    issues.push(Issue {
                        text: line,
                        pos: p.pos,
                        net: p.nets.first().copied(),
                        error: !p.coupled,
                    });
                }
            }
            if r.jumps_used > 0 {
                text += &format!(
                    "{} connection(s) use a short jump on a plane / no-routing layer\n",
                    r.jumps_used
                );
            }
            if r.pad_vias_used > 0 {
                text += &format!(
                    "{} connection(s) needed a via inside a pad of their own net\n",
                    r.pad_vias_used
                );
            }
            if r.via_shortfall > 0 {
                text += &format!(
                    "{} layer change(s) have fewer vias than requested (no room)\n",
                    r.via_shortfall
                );
            }
            let export_ok = !r.cfg.pairs
                || r.pair_report().iter().all(|p| p.coupled);
            if !export_ok {
                text += "SES export blocked: a differential group is not routed together. Adjust the group or disable differential routing.
";
            }
            if !rep.clearance_violations.is_empty() || !rep.open_nets.is_empty() || !export_ok {
                let details = issues.iter().map(|i| i.text.as_str()).collect::<Vec<_>>().join("\n");
                log_error(&format!("{}{}", text, details));
            }
            for w in &r.pour_warnings {
                text += &format!("WARNING copper area: {}\n", w);
            }
            let pro = doc.as_ref().is_some_and(|d| d.pro.is_some());
            let json = match doc.as_ref().map(|d| router::easyeda_out::write_output(&r, d)) {
                Some(Ok(j)) => Some(j),
                Some(Err(e)) => {
                    text += &format!("cannot write the EasyEDA file: {e}
");
                    None
                }
                None => None,
            };
            if let Some((_, st)) = &json {
                text += &format!(
                    "EasyEDA {}: {} track(s), {} via(s), {} copper area(s) added
",
                    if pro { "Pro project" } else { "JSON" },
                    st.tracks, st.vias, st.areas
                );
                if st.omitted > 0 {
                    text += &format!(
                        "{} existing track(s)/via(s) are not in the saved file (shown ghosted); nets that rely on them are open in that file. Tick 'Keep the existing tracks and vias in the saved file' to keep them.\n",
                        st.omitted
                    );
                }
            }
            let _ = tx.send(Msg::Done(Box::new(RunResult {
                json: json.map(|j| j.0),
                pro,
                ses: ses::write_ses(&r, &name),
                text,
                export_ok,
                options: serde_json::to_string_pretty(&r.cfg).unwrap_or_default(),
                issues,
            })));
        });
    }

    fn poll(&mut self) {
        let mut done = false;
        if let Some(rx) = &self.rx {
            loop {
                let m = match rx.try_recv() {
                    Ok(m) => m,
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => {
                        if !done {
                            self.msg="Routing worker failed; see %LOCALAPPDATA%/Autorouter/autorouter.log".into();
                            log_error(&self.msg);
                        }
                        done = true;
                        break;
                    }
                };
                match m {
                    Msg::Snap(s) => {
                        self.snaps.push(*s);
                        if self.follow {
                            self.cur = self.snaps.len() - 1;
                        }
                    }
                    Msg::Done(r) => {
                        self.result = Some(*r);
                        done = true;
                    }
                }
            }
        }
        if done {
            self.running = false;
            self.rx = None;
        }
    }

    fn cli_command(&self) -> String {
        format!(
            "autorouter -nogui \"{}\" -o out.ses --options {}",
            self.path, self.options_path
        )
    }

    fn bounds(&self) -> (f64, f64, f64, f64) {
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        if let Some(bd) = &self.board {
            for &(x, y) in &bd.boundary {
                b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
            }
        }
        b
    }

    /// display units per DSN unit
    fn k(&self) -> f64 {
        self.unit_mm / self.unit.mm()
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        let Some(board) = self.board.clone() else {
            ui.label("Open a .dsn, an EasyEDA source .json or an EasyEDA Pro .epro2 / .epro file to begin.");
            return;
        };
        let k = self.k();
        let ul = self.unit.label();
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::CollapsingHeader::new("Units").default_open(true).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Show lengths in");
                    ui.selectable_value(&mut self.unit, DispUnit::Mil, "mil");
                    ui.selectable_value(&mut self.unit, DispUnit::Mm, "mm");
                });
                let (auto_mm, assumed) = board.detect_unit_mm();
                let mut over = self.cfg.unit_mm.is_some();
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut over, "Size of one file unit").on_hover_text("How many millimetres one coordinate unit of the file is. Normally detected automatically; set it yourself when the file declares the wrong unit.").changed() {
                        self.cfg.unit_mm = if over { Some(auto_mm) } else { None };
                        self.dirty_preview = true;
                    }
                    if let Some(u) = &mut self.cfg.unit_mm {
                        if ui.add(egui::DragValue::new(u).speed(0.0005).range(0.00001..=100.0).suffix(" mm")).changed() {
                            self.dirty_preview = true;
                        }
                    } else {
                        ui.label(format!("{:.4} mm (detected)", auto_mm));
                    }
                });
                if assumed && self.cfg.unit_mm.is_none() {
                    ui.small(format!(
                        "The file declares '{} {}' but its clearances and board size are only plausible at {:.4} mm per unit, so that is used.",
                        board.res_unit, board.res, auto_mm
                    ));
                }
            });
            egui::CollapsingHeader::new("Design rules (global minimums)").default_open(false).show(ui, |ui| {
                ui.small("Everything below these values is raised to them: DSN widths and clearances, net overrides, necked-down traces, vias, and copper-to-edge distance.");
                let r = &mut self.cfg.rules;
                ui.checkbox(&mut r.enabled, "Enforce these minimums")
                    .on_hover_text("Off: the widths, clearances and via sizes of the file are used as they are.");
                ui.add_enabled_ui(r.enabled, |ui| {
                    let row = |ui: &mut egui::Ui, label: &str, v: &mut f64, tip: &str| {
                        ui.horizontal(|ui| {
                            ui.label(label).on_hover_text(tip);
                            let mut d = *v / self.unit.mm();
                            let (lo, hi) = (0.0, 100.0 / self.unit.mm());
                            if ui
                                .add(egui::DragValue::new(&mut d).speed(self.unit.mm().recip() * 0.005).range(lo..=hi).max_decimals(4).suffix(format!(" {}", ul)))
                                .changed()
                            {
                                *v = d * self.unit.mm();
                            }
                        });
                    };
                    row(ui, "Minimum trace width", &mut r.min_trace_mm, "Narrowest trace that may be drawn.");
                    row(ui, "Minimum clearance", &mut r.min_clearance_mm, "Smallest copper-to-copper spacing.");
                    row(ui, "Minimum via diameter", &mut r.min_via_diameter_mm, "Vias are made at least this wide (pad diameter).");
                    row(ui, "Minimum via drill", &mut r.min_via_drill_mm, "Drill size written for vias in the EasyEDA JSON. The SES has no drill size.");
                    row(ui, "Minimum copper to board edge", &mut r.min_edge_clearance_mm, "Distance kept free between copper and the board outline.");
                });
                if ui.button("Reset to JLCPCB 4-layer (economic)").clicked() {
                    self.cfg.rules = router::DrcRules::default();
                }
                ui.small(format!(
                    "In DSN units: trace {:.3}, clearance {:.3}, via dia {:.3}",
                    self.cfg.rules.min_trace_mm / self.unit_mm,
                    self.cfg.rules.min_clearance_mm / self.unit_mm,
                    self.cfg.rules.min_via_diameter_mm / self.unit_mm
                ));
            });
            egui::CollapsingHeader::new("Layers").default_open(true).show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("◀").on_hover_text("previous layer view (Shift+Tab / PageUp)").clicked() {
                        self.cycle_layer(-1);
                    }
                    ui.label(match self.view_layer {
                        None => "view: all layers".to_string(),
                        Some(l) => format!("view: {}", layer_label(&board.layers[l].name)),
                    });
                    if ui.button("▶").on_hover_text("next layer view (Tab / PageDown)").clicked() {
                        self.cycle_layer(1);
                    }
                });
                ui.small(if self.easyeda.is_some() {
                    "Per layer: pick the nets that belong on it. A traces layer routes traces and its nets prefer it and get copper areas in the free room. A plane layer carries no traces; its nets become planes that their pins reach through vias. Set rail widths in the Nets list."
                } else {
                    "Per layer: pick the nets that prefer it; other nets prefer layers with no nets listed. Short pad escapes may use other layers. Set rail widths in the Nets list. Only the routed traces and vias are exported."
                });
                let mut net_names: Vec<String> = board.nets.iter().map(|n| n.name.clone()).collect();
                net_names.sort_by_cached_key(|n| (n.to_lowercase(), n.clone()));
                let mut changed = false;
                for (i, l) in board.layers.iter().enumerate() {
                    ui.separator();
                    let (n_tr, n_via) = self
                        .preview
                        .as_ref()
                        .map(|pv| {
                            (
                                pv.pre_wires.iter().filter(|(_, w)| w.layer == i).count(),
                                pv.pre_vias.len(),
                            )
                        })
                        .unwrap_or((0, 0));
                    let imported: Vec<String> = self
                        .preview
                        .as_ref()
                        .map(|pv| {
                            pv.planes
                                .iter()
                                .filter(|p| p.layer == i && !p.synthetic)
                                .map(|p| p.name.clone())
                                .collect()
                        })
                        .unwrap_or_default();
                    ui.horizontal(|ui| {
                        let (r, g, b) = lc(i);
                        ui.colored_label(Color32::from_rgb(r, g, b), "■");
                        ui.checkbox(&mut self.layer_vis[i], layer_label(&l.name));
                        ui.small(format!("{} existing trace(s)", n_tr));
                        if !imported.is_empty() {
                            ui.small(format!("planes: {}", imported.join(", ")));
                        }
                        if i == 0 {
                            ui.small(format!("{} via(s) on board", n_via));
                        }
                    });
                    let plan = self.cfg.layers.entry(l.name.clone()).or_default();
                    let role_before = plan.role;
                    if self.easyeda.is_some() {
                        ui.horizontal(|ui| {
                            ui.label("Layer type:");
                            ui.selectable_value(&mut plan.role, router::LayerRole::Signal, "traces + copper areas")
                                .on_hover_text("Traces are routed on this layer. The nets listed below prefer it and get copper areas in the free room.");
                            ui.selectable_value(&mut plan.role, router::LayerRole::Plane, "plane")
                                .on_hover_text("No traces on this layer. The nets listed below become planes (copper areas); their pins reach them through vias.");
                        });
                        if plan.role != role_before {
                            changed = true;
                        }
                    } else {
                        plan.role = router::LayerRole::Signal;
                    }
                    ui.horizontal(|ui| {
                        let mut ex = self.cfg.exclusive.contains(&l.name) || plan.role == router::LayerRole::Plane;
                        let is_plane = plan.role == router::LayerRole::Plane;
                        if ui
                            .add_enabled(!is_plane, egui::Checkbox::new(&mut ex, "No traces on this layer"))
                            .on_hover_text("Nothing is routed on this layer. It can still hold copper areas and vias pass through it.")
                            .changed()
                        {
                            self.cfg.exclusive.retain(|n| n != &l.name);
                            if ex {
                                self.cfg.exclusive.push(l.name.clone());
                            }
                            changed = true;
                        }
                    });
                    let plan = self.cfg.layers.entry(l.name.clone()).or_default();
                    ui.horizontal_wrapped(|ui| {
                        ui.label("Nets for this layer:");
                        let mut remove = None;
                        for (j, n) in plan.nets.iter().enumerate() {
                            if ui.small_button(format!("{} ✕", n)).on_hover_text("remove").clicked() {
                                remove = Some(j);
                            }
                        }
                        if let Some(j) = remove {
                            plan.nets.remove(j);
                            changed = true;
                        }
                        let picker = net_picker(ui, egui::Id::unique(("addnet", i)), self.layer_net_filter.entry(i).or_default(), &net_names, &mut plan.nets);
                        changed |= picker.inner.is_some_and(|(changed, _)| changed);
                    });
                    if plan.nets.is_empty() && plan.role == router::LayerRole::Plane {
                        ui.small("Plane layer without nets: nothing is routed on it and no net gets copper on it.");
                    }
                }
                if changed {
                    self.dirty_preview = true;
                }

            });
            if let Some(pv) = &self.preview {
                let imported: Vec<&router::plane::PlaneInfo> = pv.planes.iter().filter(|p| !p.synthetic).collect();
                if !imported.is_empty() {
                    egui::CollapsingHeader::new("Existing copper areas (from the file)").default_open(true).show(ui, |ui| {
                        for pl in imported {
                            ui.horizontal(|ui| {
                                let mut ex = self.cfg.exclusive_planes.contains(&pl.name);
                                if ui
                                    .checkbox(&mut ex, format!("{}  ({} pins)", pl.name, pl.members.len()))
                                    .on_hover_text("Ticked: no trace is routed through this area at all. Unticked: traces may cross it, at the penalty below.")
                                    .changed()
                                {
                                    self.cfg.exclusive_planes.retain(|n| n != &pl.name);
                                    if ex {
                                        self.cfg.exclusive_planes.push(pl.name.clone());
                                    }
                                }
                            });
                        }
                        ui.add(egui::Slider::new(&mut self.cfg.plane_penalty, 0.0..=20.0).text("penalty for crossing an area"))
                            .on_hover_text("Extra cost for every step a trace spends inside another net's area. Higher: fewer and shorter cuts through areas.");
                        ui.add(egui::Slider::new(&mut self.cfg.plane_repair_rounds, 0..=10).text("repair attempts"))
                            .on_hover_text("If traces cut an area into pieces, the worst offenders are re-routed with a higher penalty, at most this many rounds, until the area is whole again.");
                    });
                }
            }
            egui::CollapsingHeader::new("Routing").default_open(true).show(ui, |ui| {
                ui.label(format!("Lengths are shown in {}", ul));
                ui.small("All settings in this section apply to the whole board and every layer. Hover a setting for details.");
                let json_mode = self.easyeda.is_some();

                ui.add_space(4.0);
                ui.strong("Traces and vias");
                ui.checkbox(&mut self.cfg.allow_vias_in_pads, "Allow vias inside pads")
                    .on_hover_text("Off: a via never touches a pad. On: vias may sit inside pads, which can cost extra at the fabricator (plugged or filled vias).");
                ui.checkbox(&mut self.cfg.allow_neckdown, "Narrow a trace where the full width does not fit")
                    .on_hover_text("Where a trace of the requested width does not fit, it is narrowed to 75%, 50% or 35%, never below the minimum trace width (Design rules). The full width is restored wherever it fits.");
                ui.horizontal(|ui| {
                    ui.label("Widen power traces up to");
                    ui.add(egui::DragValue::new(&mut self.cfg.power_width_max_mm).range(0.0..=20.0).speed(0.1).suffix(" mm"));
                })
                .response
                .on_hover_text("Nets that have a layer assigned or a width set grow into the free room around them, up to this width. 0 = keep the requested widths. Not needed when copper areas carry the current.");
                ui.horizontal(|ui| {
                    ui.label("Via cost").on_hover_text("How much the router dislikes a via, as extra trace length.");
                    ui.add(egui::DragValue::new(&mut self.cfg.via_cost).range(1.0..=500.0).speed(0.5).suffix(" grid steps"));
                });
                ui.small(format!(
                    "One via counts like {:.2} {} of extra trace. Higher: fewer vias and longer detours. Lower: more vias and shorter traces.",
                    self.cfg.via_cost as f64 * self.cfg.grid.unwrap_or(board.grid_wire.max(0.25)) * k,
                    ul
                ));
                ui.horizontal(|ui| {
                    ui.label("Bridge split planes/nets with up to");
                    ui.add(egui::DragValue::new(&mut self.cfg.bridge_vias).range(0..=8).suffix(" vias"));
                })
                .response
                .on_hover_text("When a plane or net ends up in separate pieces, the pieces are joined with a trace on any routable layer, using at most this many new vias per bridge. Pieces are joined as a tree (no loops). 0 = no bridging. Applies to all layers.");

                ui.add_space(6.0);
                ui.strong(if json_mode { "Copper areas (pours)" } else { "Copper areas (SES)" });
                if json_mode {
                    ui.checkbox(&mut self.cfg.pour_polygons, "Create copper areas for the nets assigned to a layer")
                        .on_hover_text("Every net assigned to a layer (Layers) gets copper areas on that layer, near its own copper; nets on one layer share the room by distance. Press Ctrl+B in EasyEDA afterwards to cut around pads, vias and traces.");
                    ui.horizontal(|ui| {
                        ui.label("An area reaches at most");
                        ui.add(egui::DragValue::new(&mut self.cfg.pour_reach_mm).range(0.5..=50.0).speed(0.1).suffix(" mm"));
                    })
                    .response
                    .on_hover_text("How far a net's copper area may extend from that net's own pads, vias and traces.");
                    ui.add_enabled_ui(self.cfg.pour_polygons, |ui| {
                        ui.checkbox(&mut self.cfg.fill_empty, "Fill the empty space of every layer")
                            .on_hover_text("After everything else is done, the free room on every layer (planes and signal layers alike) gets a copper area of the fill net below. Where such areas lie on top of each other they are joined by stitching vias, placed only where the via fits and is completely covered by the copper on every layer it touches.");
                        ui.add_enabled_ui(self.cfg.fill_empty, |ui| {
                            let mut names: Vec<String> = board.nets.iter().map(|n| n.name.clone()).collect();
                            names.sort_by_cached_key(|n| (n.to_lowercase(), n.clone()));
                            ui.horizontal(|ui| {
                                ui.label("Fill with net");
                                egui::ComboBox::from_id_salt("fill_net")
                                    .selected_text(self.cfg.fill_net.clone())
                                    .show_ui(ui, |ui| {
                                        for n in &names {
                                            ui.selectable_value(&mut self.cfg.fill_net, n.clone(), n);
                                        }
                                    });
                            });
                            ui.horizontal(|ui| {
                                ui.label("Stitching vias every");
                                ui.add(egui::DragValue::new(&mut self.cfg.stitch_pitch_mm).range(0.0..=50.0).speed(0.1).suffix(" mm"));
                            })
                            .response
                            .on_hover_text("Distance between the vias that tie the fill areas of different layers together. Vias avoid other nets' areas where they can and cut them only sparingly. 0 = no stitching vias (fill areas without copper of the fill net are then dropped).");
                        });
                    });
                } else {
                    ui.checkbox(&mut self.cfg.keep_track_widths, "Write real track widths")
                        .on_hover_text("Only for CADs that keep the width of imported tracks. Off: traces are written as plain centre lines with the fewest segments, and power traces are not widened.");
                    ui.checkbox(&mut self.cfg.pour_outlines, "Draw the area outlines as traces")
                        .on_hover_text("Draws the area a pour of each layer-assigned net would cover as a thin closed trace on its layer. Re-trace it as a copper area in the CAD, then delete the trace.");
                    ui.checkbox(&mut self.cfg.include_existing, "Also write the existing wiring to the SES")
                        .on_hover_text("Existing traces always stay on the board. Off: the SES only holds the new routes.");
                }
                if json_mode {
                    ui.checkbox(&mut self.cfg.include_existing, "Keep the existing tracks and vias in the saved file")
                        .on_hover_text("The tracks and vias that are already on the board are always shown and routed around. Off: the saved file holds only the new routing, the existing ones are left out (they show ghosted in the preview), so nets that rely on them are open in that file. On: they stay in the saved file.");
                }
                ui.checkbox(&mut self.cfg.protect_planes, "Do not cut imported copper areas")
                    .on_hover_text("On: traces avoid the copper areas that are already in the file (a trace may still cross one with a penalty, see Existing areas). Off: they may be cut freely.");

                ui.add_space(6.0);
                ui.strong("Search quality");
                ui.horizontal(|ui| {
                    ui.label("Clean-up passes");
                    ui.add(egui::DragValue::new(&mut self.cfg.refine_passes).range(0..=10));
                })
                .response
                .on_hover_text("After the first routing the board is re-routed net by net to shorten traces and remove vias. More passes: slightly better, slower. 0 = none.");
                let mut custom_grid = self.cfg.grid.is_some();
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut custom_grid, "Routing grid").on_hover_text("Off: chosen automatically. A finer grid finds tighter routes but is much slower.").changed() {
                        self.cfg.grid = if custom_grid { Some(board.grid_wire.max(0.25)) } else { None };
                    }
                    if let Some(g) = &mut self.cfg.grid {
                        dv(ui, g, k, 0.01, 10.0, 0.01, ul);
                    } else {
                        ui.label(format!("(automatic: {:.4} {})", self.cfg.grid_for(&board) * k, ul))
                            .on_hover_text("A quarter of trace width + clearance, so neighbouring tracks pack exactly.");
                    }
                });

                ui.add_space(6.0);
                ui.strong("Differential pairs");
                ui.checkbox(&mut self.cfg.pairs, "Route differential pairs together (+/-, _P/_N, DP/DN)")
                    .on_hover_text("Nets whose names form a pair are routed side by side. Groups you make in the Nets list are routed together too.");
                let mut gap = self.cfg.pair_gap.is_some();
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut gap, "Gap between the two traces").on_hover_text("Edge-to-edge distance between the two traces of a pair (not centre-to-centre).").changed() {
                        self.cfg.pair_gap = if gap { Some(0.6) } else { None };
                    }
                    if let Some(g) = &mut self.cfg.pair_gap {
                        dv(ui, g, k, 0.01, 20.0, 0.01, ul);
                    } else {
                        ui.label("(the nets' clearance)");
                    }
                });
                ui.small("Pairs are routed side by side; if that does not fit, the second trace hugs the first.");

                ui.add_space(6.0);
                ui.strong("Last resorts (only for connections that stay open)");
                ui.checkbox(&mut self.cfg.allow_plane_jumps, "Short jumps on plane and no-routing layers")
                    .on_hover_text("A connection that cannot be routed otherwise may use a short stretch of trace on a plane or no-routing layer. The plane is poured again around it.");
                ui.horizontal(|ui| {
                    ui.label("Longest jump");
                    ui.add(egui::DragValue::new(&mut self.cfg.plane_jump_mm).speed(0.1).range(0.5..=50.0).suffix(" mm"));
                })
                .response
                .on_hover_text("Longest stretch such a jump may have, per connection.");
                ui.checkbox(&mut self.cfg.last_resort_pad_vias, "Via inside a pad of the open net")
                    .on_hover_text("Tried after jumps: a via inside a pad of the open net itself. Can cost extra at the fabricator (plugged or filled vias).");
                if json_mode {
                    ui.checkbox(&mut self.cfg.nudge_parts, "Nudge small parts, then route again")
                        .on_hover_text("Tried last: small passives and other small SMD parts near the missing connection are moved a little. The router keeps existing routes where possible and repairs the affected nets. Moves that do not help are undone. Never ICs, through-hole parts, connectors, switches or oscillators, and never a part that already has wiring. The moved parts are saved in the EasyEDA JSON. Off by default.");
                    ui.add_enabled_ui(self.cfg.nudge_parts, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Move a part at most");
                            ui.add(egui::DragValue::new(&mut self.cfg.nudge_max_widths).range(0.5..=3.0).speed(0.05).suffix(" part sizes"));
                        })
                        .response
                        .on_hover_text("How far a part may end up from where it started, measured in its own size (its longest side). 3 is the maximum.");
                    });
                }
            });
            if let Some(pv) = &self.preview {
                egui::CollapsingHeader::new("Existing copper (kept)").default_open(false).show(ui, |ui| {
                    ui.label(format!("{} wire path(s), {} via(s), {} plane(s)/area(s)", pv.pre_wires.len(), pv.pre_vias.len(), pv.planes.iter().filter(|p| !p.synthetic).count()));
                    let holes = pv.obstacles.iter().filter(|o| o.keepout).count();
                    ui.label(format!("{} other object(s) without a net kept clear: {} hole(s), slot(s) or region(s), {} piece(s) of copper", pv.obstacles.len(), holes, pv.obstacles.len() - holes));
                    ui.label("Existing traces, vias and planes are never removed or cut.");
                });
            }
            egui::CollapsingHeader::new("Nets").default_open(true).show(ui, |ui| {
                egui::CollapsingHeader::new("Apply width / clearance to multiple nets").show(ui, |ui| {
                    // Capture every current value before applying either field. In matching mode,
                    // width and clearance each select their own set of matching nets.
                    let values: Vec<[f64; 2]> = board.nets.iter().enumerate().map(|(i, n)| {
                        let ov = self.cfg.net_overrides.get(&n.name);
                        let (w, c) = self.preview.as_ref().map_or(
                            (board.rule_width, board.rule_clear),
                            |p| (p.class_width[i], p.class_clearance[i]),
                        );
                        [ov.and_then(|o| o.width).unwrap_or(w), ov.and_then(|o| o.clearance).unwrap_or(c)]
                    }).collect();
                    if self.bulk_net_source != self.sel_net || self.bulk_net_values[0] == 0.0 {
                        self.bulk_net_source = self.sel_net;
                        self.bulk_net_values = self.sel_net.and_then(|i| values.get(i).copied())
                            .or_else(|| values.first().copied()).unwrap_or([1.0, 0.6]);
                    }
                    for (field, label) in ["Width", "Clearance"].iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.bulk_net_fields[field], *label);
                            ui.add_enabled_ui(self.bulk_net_fields[field], |ui| {
                                dv(ui, &mut self.bulk_net_values[field], k,
                                    if field == 0 { 0.0001 } else { 0.0 }, 100.0, 0.01, ul);
                            });
                        });
                    }
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.bulk_net_matching, false, "All nets");
                        ui.selectable_value(&mut self.bulk_net_matching, true, "All with same value");
                    });
                    let reference = self.sel_net.and_then(|i| values.get(i).copied());
                    if self.bulk_net_matching {
                        if let Some(i) = self.sel_net.filter(|&i| i < values.len()) {
                            ui.label(format!("Match {}: width {:.4} {}, clearance {:.4} {}",
                                board.nets[i].name, values[i][0] * k, ul, values[i][1] * k, ul));
                        } else {
                            ui.label("Click a net name below to choose the values to match.");
                        }
                        ui.small("Each checked field matches its own current value, including file defaults.");
                    }
                    ui.small("Includes nets hidden by search. Design-rule minimums still apply.");
                    let enabled = !values.is_empty() && self.bulk_net_fields.iter().any(|&v| v)
                        && (!self.bulk_net_matching || reference.is_some());
                    if ui.add_enabled(enabled, egui::Button::new("Apply to nets")).clicked() {
                        let mut changed = 0;
                        for (i, net) in board.nets.iter().enumerate() {
                            let mut touched = false;
                            for field in 0..2 {
                                let matches = !self.bulk_net_matching || reference.is_some_and(|r|
                                    (values[i][field] - r[field]).abs() <= 1e-9 * r[field].abs().max(1.0));
                                if self.bulk_net_fields[field] && matches {
                                    let ov = self.cfg.net_overrides.entry(net.name.clone()).or_default();
                                    if field == 0 { ov.width = Some(self.bulk_net_values[field]); }
                                    else { ov.clearance = Some(self.bulk_net_values[field]); }
                                    touched = true;
                                }
                            }
                            changed += usize::from(touched);
                        }
                        self.dirty_preview = true;
                        self.msg = format!("Applied net dimensions to {changed} nets");
                    }
                });
                ui.horizontal(|ui| {
                    ui.add(egui::TextEdit::singleline(&mut self.filter).hint_text("Search nets (for example GND)"));
                    if ui.small_button("all").on_hover_text("Route every net again (clears all the \"route\" unticks)").clicked() {
                        for o in self.cfg.net_overrides.values_mut() {
                            o.skip = false;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    let n_sel = self.multi_sel.len();
                    if ui
                        .add_enabled(n_sel >= 2, egui::Button::new(format!("Group {} selected", n_sel)))
                        .on_hover_text("Route the picked nets together as one differential, length-matched group (2 or more nets).")
                        .clicked()
                    {
                        let name = self.next_group_name();
                        for &i in &self.multi_sel {
                            self.cfg.net_overrides.entry(board.nets[i].name.clone()).or_default().diff_group = name.clone();
                        }
                        self.multi_sel.clear();
                    }
                    if ui.add_enabled(n_sel >= 1, egui::Button::new("Ungroup selected")).clicked() {
                        for &i in &self.multi_sel {
                            self.cfg.net_overrides.entry(board.nets[i].name.clone()).or_default().diff_group = "-".into();
                        }
                        self.multi_sel.clear();
                    }
                });
                let pv = self.preview.as_ref();
                let via_info: Vec<(String, f64)> = board
                    .via_names
                    .iter()
                    .map(|v| {
                        let d = board.padstacks.get(v).map_or(0.0, |ps| {
                            ps.shapes.iter().fold(0.0f64, |m, s| match s {
                                dsn::Shape::Circle { d, .. } => m.max(*d),
                                dsn::Shape::Rect { x1, y1, x2, y2, .. } => m.max((x2 - x1).max(y2 - y1)),
                                _ => m,
                            })
                        });
                        (v.clone(), d)
                    })
                    .collect();
                egui::Grid::new("nets").striped(true).show(ui, |ui| {
                    ui.label("pick").on_hover_text("Tick two or more nets to route them together as a group (see the buttons above).");
                    ui.label("route").on_hover_text("Untick to leave this net unrouted.");
                    ui.label("net");
                    ui.label("open").on_hover_text("Connections of this net that are still unrouted (a tick = complete).");
                    ui.label(format!("width ({})", ul)).on_hover_text("Trace width for this net. Empty: the default of the file.");
                    ui.label(format!("clearance ({})", ul)).on_hover_text("Smallest gap to other copper for this net. Empty: the default of the file.");
                    ui.label("group").on_hover_text("Nets with the same group name are routed together and their lengths matched.");
                    ui.label("via type").on_hover_text("The via this net uses when it changes layer (the via padstacks defined in the file).");
                    ui.label("vias per change").on_hover_text("How many vias this net puts side by side at every layer change, e.g. 2-4 for power nets. Where there is no room for that many, one via fewer is used, down to a single via. Vias never overlap.");
                    ui.end_row();
                    let query = self.filter.trim().to_lowercase();
                    let mut ordered: Vec<_> = board.nets.iter().enumerate().collect();
                    ordered.sort_by_cached_key(|(_, n)| (n.name.to_lowercase(), n.name.clone()));
                    for (i, n) in ordered {
                        if !n.name.to_lowercase().contains(&query) {
                            continue;
                        }
                        let ov = self.cfg.net_overrides.entry(n.name.clone()).or_insert_with(NetOverride::default);
                        let mut s = self.multi_sel.contains(&i);
                        if ui.checkbox(&mut s, "").changed() {
                            if s {
                                self.multi_sel.insert(i);
                            } else {
                                self.multi_sel.remove(&i);
                            }
                        }
                        let mut route = !ov.skip;
                        if ui.checkbox(&mut route, "").changed() {
                            ov.skip = !route;
                        }
                        let sel = self.sel_net == Some(i);
                        let grouped = !ov.diff_group.trim().is_empty() && ov.diff_group.trim() != "-";
                        let label = format!("{}{}", if grouped { "⇄ " } else { "" }, n.name);
                        if ui.selectable_label(sel, label).clicked() {
                            self.sel_net = if sel { None } else { Some(i) };
                        }
                        let open = pv.map_or(0, |p| p.groups[i].saturating_sub(1));
                        ui.label(if open == 0 { "✓".to_string() } else { open.to_string() });
                        let (dw, dc) = pv.map_or((0.0, 0.0), |p| (p.class_width[i], p.class_clearance[i]));
                        let mut w = ov.width.unwrap_or(dw);
                        if dv(ui, &mut w, k, 0.0001, 100.0, 0.01, "") {
                            ov.width = Some(w);
                        }
                        let mut c = ov.clearance.unwrap_or(dc);
                        if dv(ui, &mut c, k, 0.0, 100.0, 0.01, "") {
                            ov.clearance = Some(c);
                        }
                        ui.add(egui::TextEdit::singleline(&mut ov.diff_group).desired_width(45.0));
                        egui::ComboBox::from_id_salt(("via", i))
                            .width(60.0)
                            .selected_text(ov.via.clone().unwrap_or_else(|| "auto".into()))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut ov.via, None, "auto");
                                for v in &via_info {
                                    ui.selectable_value(&mut ov.via, Some(v.0.clone()), format!("{}  (⌀ {:.3} {})", v.0, v.1 * k, ul));
                                }
                            });
                        let mut vc = ov.via_count.unwrap_or(1);
                        if ui.add(egui::DragValue::new(&mut vc).range(1..=8)).changed() {
                            ov.via_count = if vc > 1 { Some(vc) } else { None };
                        }
                        ui.end_row();
                    }
                });
                ui.small("Groups are numbered (1, 2, 3 ...). Any number of nets (2 or more) can share a group; detected pairs are grouped on load. Type - to disable a detected pair.");
                ui.horizontal(|ui| {
                    ui.label(format!("Length mismatch tolerance ({})", ul));
                    dv(ui, &mut self.cfg.pair_tolerance, k, 0.001, 1000.0, 0.01, "");
                });
                let groups: std::collections::BTreeSet<String> = self.cfg.net_overrides.values().map(|o|o.diff_group.trim().to_owned()).filter(|g|!g.is_empty() && g!="-").collect();
                for group in groups {
                    ui.horizontal(|ui| {
                        ui.label(format!("Group {} tolerance ({})", group, ul));
                        let value=self.cfg.group_tolerances.entry(group).or_insert(self.cfg.pair_tolerance);
                        dv(ui, value, k, 0.001, 1000.0, 0.01, "");
                    });
                }
            });
            egui::CollapsingHeader::new("Same options on the command line").default_open(false).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.text_edit_singleline(&mut self.options_path);
                });
                if ui.button("Save options file").clicked() {
                    let mut c = self.cfg.clone();
                    c.net_overrides.retain(|_, o| o.skip || o.width.is_some() || o.clearance.is_some() || !o.diff_group.is_empty());
                    self.msg = match std::fs::write(&self.options_path, serde_json::to_string_pretty(&c).unwrap()) {
                        Ok(_) => format!("saved {}", self.options_path),
                        Err(e) => e.to_string(),
                    };
                }
                if ui.button("Load options file").clicked() {
                    match std::fs::read_to_string(&self.options_path).map_err(|e| e.to_string()).and_then(|t| serde_json::from_str::<Config>(&t).map_err(|e| e.to_string())) {
                        Ok(mut c) => {
                            c.use_trace_routing();
                            c.verbose = false;
                            self.cfg = c;
                            self.dirty_preview = true;
                            self.msg = format!("loaded {}", self.options_path);
                        }
                        Err(e) => self.msg = e,
                    }
                }
                let mut cmd = self.cli_command();
                ui.add(egui::TextEdit::multiline(&mut cmd).desired_rows(3).desired_width(f32::INFINITY));
            });
        });
    }

    fn cycle_layer(&mut self, dir: i32) {
        let Some(b) = &self.board else { return };
        let n = b.layers.len() as i32;
        // positions: -1 = all layers, 0..n = single layer
        let cur = self.view_layer.map_or(-1, |l| l as i32);
        let next = (cur + 1 + dir).rem_euclid(n + 1) - 1;
        self.view_layer = if next < 0 { None } else { Some(next as usize) };
    }

    fn next_group_name(&self) -> String {
        let used: std::collections::HashSet<&str> = self
            .cfg
            .net_overrides
            .values()
            .map(|o| o.diff_group.trim())
            .collect();
        (1..)
            .map(|i| i.to_string())
            .find(|s| !used.contains(s.as_str()))
            .unwrap()
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        let (resp, painter) =
            ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
        let rect = resp.rect;
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 18, 22));
        let Some(board) = &self.board else {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Open a DSN file",
                egui::FontId::proportional(20.0),
                Color32::GRAY,
            );
            return;
        };
        let b = self.bounds();
        if !self.fitted {
            let (w, h) = ((b.2 - b.0) as f32, (b.3 - b.1) as f32);
            self.scale = (rect.width() / w).min(rect.height() / h) * 0.95;
            self.off = egui::vec2(
                rect.center().x - (b.0 + b.2) as f32 / 2.0 * self.scale,
                rect.center().y + (b.1 + b.3) as f32 / 2.0 * self.scale,
            );
            self.fitted = true;
        }
        if resp.dragged() {
            self.off += resp.drag_delta();
        }
        if resp.hovered() {
            let sc = ctx.input(|i| i.smooth_scroll_delta.y);
            if sc != 0.0 {
                let f = (sc * 0.0025).exp();
                if let Some(m) = resp.hover_pos() {
                    self.off = m.to_vec2() + (self.off - m.to_vec2()) * f;
                }
                self.scale *= f;
            }
        }
        self.canvas_rect = rect;
        if resp.double_clicked() {
            self.focus = None;
        }
        let (scale, off) = (self.scale, self.off);
        let tr = |x: f64, y: f64| Pos2::new(x as f32 * scale + off.x, -(y as f32) * scale + off.y);
        let pts: Vec<Pos2> = board.boundary.iter().map(|&(x, y)| tr(x, y)).collect();
        painter.add(egui::Shape::closed_line(
            pts,
            Stroke::new(1.5, Color32::from_rgb(230, 220, 120)),
        ));
        let sel = self.sel_net;
        let dim = |net: i32| sel.map_or(false, |s| s as i32 != net);
        let skipped = |net: usize| {
            board
                .nets
                .get(net)
                .and_then(|n| self.cfg.net_overrides.get(&n.name))
                .map_or(false, |o| o.skip)
        };

        let view = self.view_layer;
        let vis = self.layer_vis.clone();
        let shown = move |l: usize| vis.get(l).copied().unwrap_or(true) && view.map_or(true, |v| v == l);
        // planes
        if self.show_planes {
            if let Some(pv) = &self.preview {
                for pl in pv.planes.iter().filter(|p| !p.synthetic) {
                    if !shown(pl.layer) {
                        continue;
                    }
                    let ex = self.cfg.exclusive_planes.contains(&pl.name);
                    let v: Vec<Pos2> = pl.poly.iter().map(|&(x, y)| tr(x, y)).collect();
                    painter.add(egui::Shape::convex_polygon(
                        v.clone(),
                        col(pl.layer, if ex { 28 } else { 16 }),
                        Stroke::new(1.0, col(pl.layer, if ex { 200 } else { 90 })),
                    ));
                    if let Some(&(x, y)) = pl.poly.first() {
                        painter.text(
                            tr(x, y) + egui::vec2(6.0, 6.0),
                            egui::Align2::LEFT_TOP,
                            format!("{}{}", pl.name, if ex { "  (no routing)" } else { "" }),
                            egui::FontId::proportional(12.0),
                            col(pl.layer, 220),
                        );
                    }
                }
            }
        }
        // EasyEDA JSON: the copper areas that will be exported, exactly as written
        if self.easyeda.is_some() {
            if let Some(s) = self.snaps.get(self.cur) {
                for (net, layer, ring) in &s.pours {
                    if !shown(*layer) {
                        continue;
                    }
                    let (r, g, b) = lc(*layer);
                    let h = (*net as u32).wrapping_mul(2654435761);
                    let tint = |v: u8, k: u32| (v as f32 * 0.28 + 10.0 + ((h >> k) & 0x3f) as f32 * 0.6) as u8;
                    let c = Color32::from_rgb(tint(r, 0), tint(g, 8), tint(b, 16));
                    let pts: Vec<Pos2> = ring.iter().map(|&(x, y)| tr(x, y)).collect();
                    let mut mesh = egui::Mesh::default();
                    for p in &pts {
                        mesh.colored_vertex(*p, c);
                    }
                    for t in triangulate(&ring.iter().map(|&(x, y)| (x, -y)).collect::<Vec<_>>()) {
                        mesh.add_triangle(t[0] as u32, t[1] as u32, t[2] as u32);
                    }
                    painter.add(egui::Shape::mesh(mesh));
                    painter.add(egui::Shape::closed_line(pts, Stroke::new(1.0, Color32::from_rgb(r / 2 + 30, g / 2 + 30, b / 2 + 30))));
                }
            }
        }
        // generated planes / pours: solid, dimmed copper below everything else
        if let Some(s) = self.snaps.get(self.cur).filter(|_| self.easyeda.is_none()) {
            for w in s.wires.iter().filter(|w| w.wire.fill) {
                if !shown(w.wire.layer) {
                    continue;
                }
                let (r, g, b) = lc(w.wire.layer);
                let c = Color32::from_rgb(
                    (r as f32 * 0.30 + 12.0) as u8,
                    (g as f32 * 0.30 + 12.0) as u8,
                    (b as f32 * 0.30 + 14.0) as u8,
                );
                let wd = (w.wire.width as f32 * scale).max(1.0);
                for seg in w.wire.pts.windows(2) {
                    painter.line_segment(
                        [tr(seg[0].0, seg[0].1), tr(seg[1].0, seg[1].1)],
                        Stroke::new(wd, c),
                    );
                }
            }
        }
        let moved_positions: HashMap<&str, (f64, f64)> = self.snaps.get(self.cur).into_iter()
            .flat_map(|s| s.pin_positions.iter().map(|(name, p)| (name.as_str(), *p))).collect();
        // Pads are never hidden: those on layers that are not shown stay as faint outlines.
        if self.show_pads {
            for p in &self.pads {
                let (dx, dy) = p.offset(&moved_positions);
                let on = shown(p.layer);
                let c = col(p.layer, if dim(p.net) { 30 } else if on { 110 } else { 45 });
                let edge = Stroke::new(1.0, col(p.layer, if on { 160 } else { 110 }));
                match &p.shape {
                    AbsShape::Circle { cx, cy, r } => {
                        painter.circle(tr(*cx + dx, *cy + dy), *r as f32 * scale, c, edge);
                    }
                    AbsShape::Poly(pts) => {
                        let v: Vec<Pos2> = pts.iter().map(|&(x, y)| tr(x + dx, y + dy)).collect();
                        painter.add(egui::Shape::convex_polygon(v, c, edge));
                    }
                }
            }
        }
        // existing copper (kept on the board and always shown; ghosted when the saved EasyEDA file
        // will not contain it)
        let ghost = self.easyeda.is_some() && !self.cfg.include_existing;
        if let Some(pv) = &self.preview {
            for (net, w) in &pv.pre_wires {
                if !shown(w.layer) {
                    continue;
                }
                let wd = (w.width as f32 * scale).max(1.0);
                let c = col(w.layer, if dim(*net as i32) { 40 } else if ghost { 95 } else { 200 });
                for seg in w.pts.windows(2) {
                    let (a, b2) = (tr(seg[0].0, seg[0].1), tr(seg[1].0, seg[1].1));
                    if ghost {
                        // outline only: this copper is not part of the saved file
                        painter.line_segment([a, b2], Stroke::new(wd + 2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 40)));
                    } else {
                        painter.line_segment(
                            [a, b2],
                            Stroke::new(wd + 2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 70)),
                        );
                    }
                    painter.line_segment([a, b2], Stroke::new(wd, c));
                    painter.circle_filled(a, wd / 2.0, c);
                    painter.circle_filled(b2, wd / 2.0, c);
                }
            }
            for (_, v, r) in &pv.pre_vias {
                let c = tr(v.0, v.1);
                if ghost {
                    painter.circle_filled(c, *r as f32 * scale, Color32::from_rgba_unmultiplied(190, 190, 190, 70));
                    painter.circle_stroke(c, *r as f32 * scale, Stroke::new(1.0, Color32::from_rgb(190, 190, 190)));
                    painter.circle_filled(c, *r as f32 * scale * 0.45, Color32::from_rgb(18, 18, 22));
                } else {
                    painter.circle_filled(c, *r as f32 * scale, Color32::from_rgb(190, 190, 190));
                    painter.circle_filled(c, *r as f32 * scale * 0.45, Color32::from_rgb(18, 18, 22));
                }
            }
            // everything else that is on the board but belongs to no net: copper without a net
            // in its layer colour, keep-outs (holes, slots, regions) hatched in red
            for o in &pv.obstacles {
                let Some(&l) = o.layers.iter().find(|&&l| shown(l)) else { continue };
                let (fill, edge) = if o.keepout {
                    (Color32::from_rgba_unmultiplied(235, 70, 70, 70), Stroke::new(1.5, Color32::from_rgb(235, 90, 90)))
                } else {
                    (col(l, 150), Stroke::new(1.0, col(l, 230)))
                };
                match &o.geo {
                    router::geom::Geo::Circle { c, r } => {
                        let p = tr(c.0, c.1);
                        painter.circle_filled(p, *r as f32 * scale, fill);
                        painter.circle_stroke(p, *r as f32 * scale, edge);
                        if o.keepout {
                            // a hole: a cross through it
                            let rr = *r as f32 * scale * 0.6;
                            painter.line_segment([p - egui::vec2(rr, rr), p + egui::vec2(rr, rr)], edge);
                            painter.line_segment([p - egui::vec2(rr, -rr), p + egui::vec2(rr, -rr)], edge);
                        } else {
                            painter.circle_filled(p, *r as f32 * scale * 0.45, Color32::from_rgb(18, 18, 22));
                        }
                    }
                    router::geom::Geo::Seg { a, b, hw } => {
                        let wd = (*hw as f32 * 2.0 * scale).max(1.0);
                        let (pa, pb) = (tr(a.0, a.1), tr(b.0, b.1));
                        painter.line_segment([pa, pb], Stroke::new(wd, fill));
                        painter.circle_filled(pa, wd / 2.0, fill);
                        painter.circle_filled(pb, wd / 2.0, fill);
                    }
                    router::geom::Geo::Poly(pts) => {
                        let v: Vec<Pos2> = pts.iter().map(|&(x, y)| tr(x, y)).collect();
                        hatch_polygon(&painter, &v, fill, edge, if o.keepout { 5.0 } else { 3.0 });
                    }
                }
            }
        }
        // routed
        let snap = self.snaps.get(self.cur);
        if let Some(s) = snap {
            for w in s.wires.iter().filter(|w| !w.wire.fill) {
                if !shown(w.wire.layer) {
                    continue;
                }
                let c = col(w.wire.layer, if dim(w.net as i32) { 40 } else { 235 });
                let wd = (w.wire.width as f32 * scale).max(1.0);
                for seg in w.wire.pts.windows(2) {
                    painter.line_segment(
                        [tr(seg[0].0, seg[0].1), tr(seg[1].0, seg[1].1)],
                        Stroke::new(wd, c),
                    );
                }
                for p in &w.wire.pts {
                    painter.circle_filled(tr(p.0, p.1), wd / 2.0, c);
                }
            }
            for v in &s.vias {
                let c = tr(v.x, v.y);
                let a = if dim(v.net as i32) { 60 } else { 255 };
                painter.circle_filled(
                    c,
                    v.r as f32 * scale,
                    Color32::from_rgba_unmultiplied(230, 230, 230, a),
                );
                painter.circle_filled(c, v.r as f32 * scale * 0.45, Color32::from_rgb(18, 18, 22));
            }
        }
        // Screen-space labels remain readable as the board is zoomed.
        let mut labels = 0;
        let mut label = |net: usize, point: (f64, f64), size: f32| {
            let pos = tr(point.0, point.1);
            if size >= 18.0 && rect.contains(pos) && labels < 150 {
                if let Some(n) = board.nets.get(net) {
                    painter.text(
                        pos,
                        egui::Align2::CENTER_CENTER,
                        &n.name,
                        egui::FontId::proportional(11.0),
                        Color32::WHITE,
                    );
                    labels += 1;
                }
            }
        };
        if self.show_pads {
            for p in &self.pads {
                if p.net < 0 || !shown(p.layer) {
                    continue;
                }
                let (center, size) = match &p.shape {
                    AbsShape::Circle { cx, cy, r } => ((*cx, *cy), 2.0 * *r),
                    AbsShape::Poly(pts) => {
                        let mut b = (
                            f64::INFINITY,
                            f64::INFINITY,
                            f64::NEG_INFINITY,
                            f64::NEG_INFINITY,
                        );
                        for &(x, y) in pts {
                            b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
                        }
                        (
                            ((b.0 + b.2) / 2.0, (b.1 + b.3) / 2.0),
                            (b.2 - b.0).min(b.3 - b.1),
                        )
                    }
                };
                let (dx, dy) = p.offset(&moved_positions);
                label(p.net as usize, (center.0 + dx, center.1 + dy), size as f32 * scale);
            }
        }
        let pre = self
            .preview
            .iter()
            .flat_map(|p| p.pre_wires.iter().map(|(n, w)| (*n, w)));
        let routed = snap
            .iter()
            .flat_map(|s| s.wires.iter().filter(|w| !w.wire.fill).map(|w| (w.net, &w.wire)));
        for (net, w) in pre.chain(routed) {
            if !shown(w.layer) || w.width as f32 * scale < 5.0 {
                continue;
            }
            if let Some(seg) = w.pts.windows(2).max_by(|a, b| {
                router::geom::dist(a[0], a[1]).total_cmp(&router::geom::dist(b[0], b[1]))
            }) {
                let len = router::geom::dist(seg[0], seg[1]) as f32 * scale;
                if len > 90.0 {
                    label(
                        net,
                        ((seg[0].0 + seg[1].0) / 2.0, (seg[0].1 + seg[1].1) / 2.0),
                        20.0,
                    );
                }
            }
        }
        // airwires: before routing from the preview, afterwards the unrouted ones of the snapshot
        if self.show_airwires {
            let white = Stroke::new(1.0, Color32::from_rgb(255, 255, 255));
            match snap {
                None => {
                    if let Some(pv) = &self.preview {
                        for (net, a, b2) in &pv.airwires {
                            if !skipped(*net) {
                                painter.line_segment([tr(a.0, a.1), tr(b2.0, b2.1)], white);
                            }
                        }
                    }
                }
                Some(s) => {
                    for (net, pin) in &s.unrouted {
                        if let (Some(a), Some(first)) = (
                            moved_positions.get(pin.as_str()).or_else(|| self.pin_pos.get(pin)),
                            board.nets[*net]
                                .pins
                                .first()
                                .and_then(|p| {
                                    let name = format!("{}-{}", p.comp, p.pin);
                                    moved_positions.get(name.as_str()).or_else(|| self.pin_pos.get(&name))
                                }),
                        ) {
                            painter.line_segment([tr(a.0, a.1), tr(first.0, first.1)], white);
                        }
                    }
                }
            }
        }
        if let Some(f) = self.focus {
            let c = tr(f.0, f.1);
            let pulse = (ctx.input(|i| i.time) * 3.0).sin() as f32 * 4.0;
            for (rad, w) in [(22.0 + pulse, 3.0), (34.0 + pulse, 1.5)] {
                painter.circle_stroke(c, rad, Stroke::new(w, Color32::from_rgb(255, 60, 200)));
            }
            painter.line_segment([c - egui::vec2(46.0, 0.0), c - egui::vec2(12.0, 0.0)], Stroke::new(1.5, Color32::from_rgb(255, 60, 200)));
            painter.line_segment([c + egui::vec2(12.0, 0.0), c + egui::vec2(46.0, 0.0)], Stroke::new(1.5, Color32::from_rgb(255, 60, 200)));
            ctx.request_repaint();
        }
        if let Some(l) = self.view_layer {
            painter.text(
                rect.right_top() + egui::vec2(-8.0, 6.0),
                egui::Align2::RIGHT_TOP,
                format!("layer view: {}   ( Tab / Shift+Tab: next / previous layer )", layer_label(&board.layers[l].name)),
                egui::FontId::proportional(13.0),
                col(l, 255),
            );
        }
        // excluded layers note
        if !self.cfg.exclusive.is_empty() {
            painter.text(
                rect.left_top() + egui::vec2(8.0, 6.0),
                egui::Align2::LEFT_TOP,
                format!("no routing on layer(s): {}", self.cfg.exclusive.join(", ")),
                egui::FontId::proportional(12.0),
                Color32::from_rgb(255, 170, 90),
            );
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll();
        if self.dirty_preview && !self.running {
            self.refresh_preview();
        }
        if ctx.memory(|m| m.focused().is_none()) {
            let (next, prev) = ctx.input_mut(|i| {
                // Tab / PageDown = next layer, Shift+Tab / PageUp = previous (layout independent)
                let prev = i.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab)
                    || i.consume_key(egui::Modifiers::NONE, egui::Key::PageUp);
                let next = i.consume_key(egui::Modifiers::NONE, egui::Key::Tab)
                    || i.consume_key(egui::Modifiers::NONE, egui::Key::PageDown);
                (next, prev)
            });
            if next {
                self.cycle_layer(1);
            }
            if prev {
                self.cycle_layer(-1);
            }
        }
        self.save_settings();
        if !self.running {
            let dropped = ctx.input(|i| {
                i.raw
                    .dropped_files
                    .iter()
                    .map(|f| f.path().to_path_buf())
                    .find(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("dsn")))
            });
            if let Some(p) = dropped {
                self.load(&p.to_string_lossy());
            }
        }
        if self.running {
            ctx.request_repaint_after(std::time::Duration::from_millis(60));
        }
        if self.playing && !self.snaps.is_empty() {
            let now = ctx.input(|i| i.time);
            if now - self.last_tick > 0.1 {
                self.last_tick = now;
                if self.cur + 1 < self.snaps.len() {
                    self.cur += 1;
                } else {
                    self.playing = false;
                }
            }
            ctx.request_repaint();
        }

        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("📂 Open DSN / EasyEDA…").clicked() {
                    if let Some(p) = rfd::FileDialog::new()
                        .add_filter("Specctra DSN, EasyEDA source or EasyEDA Pro project", &["dsn", "json", "epro2", "epro"])
                        .pick_file()
                    {
                        let p = p.to_string_lossy().to_string();
                        self.load(&p);
                    }
                }
                ui.add(egui::TextEdit::singleline(&mut self.path).desired_width(320.0));
                if ui.button("load").clicked() {
                    let p = self.path.clone();
                    self.load(&p);
                }
                ui.separator();
                let can_run = self.board.is_some() && !self.running;
                if ui
                    .add_enabled(can_run, egui::Button::new("▶ Route"))
                    .clicked()
                {
                    self.start();
                }
                if ui
                    .add_enabled(self.running, egui::Button::new("■ Stop"))
                    .clicked()
                {
                    self.cancel.store(true, Ordering::Relaxed);
                }
                let can_save = self.result.as_ref().is_some_and(|r| r.export_ok);
                if self.result.as_ref().is_some_and(|r| r.json.is_some()) {
                    let pro = self.result.as_ref().is_some_and(|r| r.pro);
                    if ui
                        .add_enabled(can_save, egui::Button::new(if pro { "💾 Save EasyEDA Pro project…" } else { "💾 Save EasyEDA JSON…" }))
                        .on_hover_text(if pro {
                            "The loaded EasyEDA Pro project plus routed tracks, vias and copper pours. Open it in EasyEDA Pro."
                        } else {
                            "The loaded EasyEDA file plus routed tracks, vias and copper areas. Open it in EasyEDA with File > Open > EasyEDA source."
                        })
                        .clicked()
                    {
                        let ext = if pro && self.path.to_lowercase().ends_with(".epro") { "epro" } else if pro { "epro2" } else { "json" };
                        let stem = std::path::Path::new(&self.path)
                            .file_stem()
                            .map(|s| s.to_string_lossy().to_string())
                            .unwrap_or("out".into());
                        if let (Some(p), Some(r)) = (
                            rfd::FileDialog::new()
                                .set_file_name(format!("{}-routed.{}", stem, ext))
                                .add_filter(if pro { "EasyEDA Pro project" } else { "EasyEDA source" }, &[ext])
                                .save_file(),
                            &self.result,
                        ) {
                            let _ = std::fs::write(p.with_extension("options.json"), &r.options);
                            self.msg = match r.json.as_ref().map(|j| std::fs::write(&p, j)) {
                                Some(Ok(_)) => format!("saved {} (settings: {})", p.display(), p.with_extension("options.json").display()),
                                Some(Err(e)) => e.to_string(),
                                None => String::new(),
                            };
                        }
                    }
                }
                if ui
                    .add_enabled(can_save, egui::Button::new("💾 Save SES…"))
                    .clicked()
                {
                    let stem = std::path::Path::new(&self.path)
                        .file_stem()
                        .map(|s| s.to_string_lossy().to_string())
                        .unwrap_or("out".into());
                    if let (Some(p), Some(r)) = (
                        rfd::FileDialog::new()
                            .set_file_name(format!("{}.ses", stem))
                            .add_filter("Specctra SES", &["ses"])
                            .save_file(),
                        &self.result,
                    ) {
                        let _ = std::fs::write(p.with_extension("options.json"), &r.options);
                        self.msg = match std::fs::write(&p, &r.ses) {
                            Ok(_) => {
                                format!("saved {} (settings: {})", p.display(), p.with_extension("options.json").display())
                            }
                            Err(e) => e.to_string(),
                        };
                    }
                }
                ui.separator();
                ui.separator();
                if ui.button("◀").on_hover_text("previous layer (Shift+Tab / PageUp)").clicked() {
                    self.cycle_layer(-1);
                }
                let lbl = match (self.view_layer, &self.board) {
                    (Some(l), Some(b)) => layer_label(&b.layers[l].name),
                    _ => "all layers".to_string(),
                };
                ui.label(format!("view: {}", lbl));
                if ui.button("▶").on_hover_text("next layer (Tab / PageDown)").clicked() {
                    self.cycle_layer(1);
                }
                ui.separator();
                ui.checkbox(&mut self.show_pads, "pads");
                ui.checkbox(&mut self.show_planes, "planes");
                ui.checkbox(&mut self.show_airwires, "airwires");
                if ui.button("fit").clicked() {
                    self.fitted = false;
                }
            });
            if !self.msg.is_empty() {
                ui.small(&self.msg);
            }
        });

        egui::Panel::left("opts")
            .default_size(340.0)
            .show(ui, |ui| {
                self.side_panel(ui);
            });

        egui::Panel::bottom("timeline").show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui.button(if self.playing { "⏸" } else { "▶" }).clicked() {
                    self.playing = !self.playing;
                    self.follow = false;
                    if self.playing && self.cur + 1 >= self.snaps.len() {
                        self.cur = 0;
                    }
                }
                if ui.button("⏮").clicked() {
                    self.cur = 0;
                    self.follow = false;
                }
                if ui.button("◀").clicked() && self.cur > 0 {
                    self.cur -= 1;
                    self.follow = false;
                }
                if ui.button("▶|").clicked() && self.cur + 1 < self.snaps.len() {
                    self.cur += 1;
                }
                if !self.snaps.is_empty() {
                    let last = self.snaps.len() - 1;
                    if ui.add(egui::Slider::new(&mut self.cur, 0..=last).text("snapshot")).changed() {
                        self.follow = self.cur == last;
                    }
                }
                ui.checkbox(&mut self.follow, "follow live");
            });
            if let Some(s) = self.snaps.get(self.cur) {
                ui.label(format!("{}   |   routed {}/{}   length {:.1}   vias {}   t={:.2}s", s.label, s.routed, s.connections, s.length, s.vias.len(), s.elapsed));
            } else if let Some(pv) = &self.preview {
                let open: usize = pv.groups.iter().map(|g| g.saturating_sub(1)).sum();
                ui.label(format!("Preview: {} open connection(s) to route. Adjust the options on the left, then press Route.", open));
            }
            if self.running {
                ui.label("routing…");
            }
            let mut jump: Option<((f64, f64), Option<usize>)> = None;
            if let Some(r) = &self.result {
                ui.separator();
                if ui.button("Copy log").clicked() {
                    let mut log = format!("Board: {}\n{}", self.path, r.text);
                    for issue in &r.issues { log.push_str(&format!("{}\n", issue.text)); }
                    log.push_str("\nRouting progress:\n");
                    for snap in &self.snaps { log.push_str(&format!("{:.1}s: {} ({}/{})\n", snap.elapsed, snap.label, snap.routed, snap.connections)); }
                    log.push_str(&format!("\nOptions used for this run:\n{}", r.options));
                    ui.ctx().copy_text(log);
                }
                ui.monospace(&r.text);
                if !r.issues.is_empty() {
                    ui.label("Click a line to show it on the board:");
                    egui::ScrollArea::vertical().max_height(130.0).show(ui, |ui| {
                        for is in &r.issues {
                            let c = if is.error {
                                Color32::from_rgb(255, 110, 110)
                            } else {
                                Color32::from_rgb(255, 200, 90)
                            };
                            let txt = egui::RichText::new(format!(
                                "{} {}",
                                if is.error { "⛔" } else { "⚠" },
                                is.text
                            ))
                            .color(c)
                            .monospace();
                            if ui.add(egui::Label::new(txt).sense(egui::Sense::click())).on_hover_text("show on board").clicked() {
                                jump = Some((is.pos, is.net));
                            }
                        }
                    });
                }
            }
            if let Some((pos, net)) = jump {
                self.focus = Some(pos);
                self.sel_net = net;
                self.view_layer = None;
                let rect = self.canvas_rect;
                if rect.is_positive() {
                    let b = self.bounds();
                    let span = ((b.2 - b.0).max(b.3 - b.1) as f32).max(1.0);
                    // zoom so about a fifth of the board is visible
                    self.scale = self.scale.max(rect.width().min(rect.height()) / (span / 5.0));
                    self.off = egui::vec2(
                        rect.center().x - pos.0 as f32 * self.scale,
                        rect.center().y + pos.1 as f32 * self.scale,
                    );
                }
            }
        });

        egui::CentralPanel::default().show(ui, |ui| {
            self.canvas(ui);
        });
    }
}

// ----------------------------------------------------------------------------- headless svg
fn to_svg(board: &Board, pads: &[Pad], pv: &Preview, s: Option<&Snapshot>) -> String {
    use std::fmt::Write;
    let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for &(x, y) in &board.boundary {
        b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
    }
    let m = 5.0;
    let (w, h) = (b.2 - b.0 + 2.0 * m, b.3 - b.1 + 2.0 * m);
    let mut o = String::new();
    let _ = writeln!(
        o,
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='{} {} {} {}' width='1400'><rect x='{}' y='{}' width='{}' height='{}' fill='#121216'/>",
        b.0 - m, -b.3 - m, w, h, b.0 - m, -b.3 - m, w, h
    );
    let hex = |l: usize| {
        let (r, g, b) = lc(l);
        format!("#{:02x}{:02x}{:02x}", r, g, b)
    };
    let pts: Vec<String> = board
        .boundary
        .iter()
        .map(|&(x, y)| format!("{},{}", x, -y))
        .collect();
    let _ = writeln!(
        o,
        "<polygon points='{}' fill='none' stroke='#e6dc78' stroke-width='0.5'/>",
        pts.join(" ")
    );
    for pl in &pv.planes {
        let pts: Vec<String> = pl
            .poly
            .iter()
            .map(|&(x, y)| format!("{},{}", x, -y))
            .collect();
        let _ = writeln!(o, "<polygon points='{}' fill='{}' fill-opacity='0.08' stroke='{}' stroke-opacity='0.4' stroke-width='0.3'/>", pts.join(" "), hex(pl.layer), hex(pl.layer));
    }
    let moved_positions: HashMap<&str, (f64, f64)> = s.into_iter()
        .flat_map(|s| s.pin_positions.iter().map(|(name, p)| (name.as_str(), *p))).collect();
    for p in pads {
        let (dx, dy) = p.offset(&moved_positions);
        match &p.shape {
            AbsShape::Circle { cx, cy, r } => {
                let _ = writeln!(
                    o,
                    "<circle cx='{}' cy='{}' r='{}' fill='{}' fill-opacity='0.4'/>",
                    cx + dx,
                    -(cy + dy),
                    r,
                    hex(p.layer)
                );
            }
            AbsShape::Poly(v) => {
                let pts: Vec<String> = v.iter().map(|&(x, y)| format!("{},{}", x + dx, -(y + dy))).collect();
                let _ = writeln!(
                    o,
                    "<polygon points='{}' fill='{}' fill-opacity='0.4'/>",
                    pts.join(" "),
                    hex(p.layer)
                );
            }
        }
    }
    let line = |o: &mut String, pts: &[(f64, f64)], layer: usize, width: f64, op: f32| {
        let p: Vec<String> = pts.iter().map(|&(x, y)| format!("{},{}", x, -y)).collect();
        let _ = writeln!(o, "<polyline points='{}' fill='none' stroke='{}' stroke-width='{}' stroke-linecap='round' stroke-linejoin='round' stroke-opacity='{}'/>", p.join(" "), hex(layer), width, op);
    };
    for (_, w) in &pv.pre_wires {
        line(&mut o, &w.pts, w.layer, w.width, 0.8);
    }
    if let Some(s) = s {
        for w in &s.wires {
            line(&mut o, &w.wire.pts, w.wire.layer, w.wire.width, 0.9);
        }
        for v in &s.vias {
            let _ = writeln!(o, "<circle cx='{}' cy='{}' r='{}' fill='#ddd'/><circle cx='{}' cy='{}' r='{}' fill='#121216'/>", v.x, -v.y, v.r, v.x, -v.y, v.r * 0.45);
        }
    } else {
        for (_, a, b2) in &pv.airwires {
            let _ = writeln!(
                o,
                "<line x1='{}' y1='{}' x2='{}' y2='{}' stroke='#fff' stroke-width='0.25'/>",
                a.0, -a.1, b2.0, -b2.1
            );
        }
    }
    let _ = writeln!(o, "</svg>");
    o
}

fn main() -> eframe::Result {
    std::panic::set_hook(Box::new(|info| {
        log_error(&format!(
            "{}\n{}",
            info,
            std::backtrace::Backtrace::force_capture()
        ))
    }));
    if std::env::args().any(|a| a == "-nogui" || a == "--nogui") {
        #[cfg(windows)]
        unsafe {
            unsafe extern "system" {
                fn AttachConsole(process: u32) -> i32;
            }
            AttachConsole(u32::MAX);
        }
        cli::run();
        return Ok(());
    }
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut app = App::new();
    if let Some(a) = args.first().filter(|a| !a.starts_with("--")) {
        app.load(a);
    } else if !app.path.is_empty() && std::path::Path::new(&app.path).exists() {
        let last = app.path.clone();
        app.load(&last);
    }
    if args.iter().any(|a| a == "--selftest") {
        // run the GUI's background routing path without a window
        app.start();
        while app.running {
            std::thread::sleep(std::time::Duration::from_millis(50));
            app.poll();
        }
        println!("{} live snapshots received", app.snaps.len());
        if let Some(r) = &app.result {
            print!("{}", r.text);
            println!("SES bytes: {}", r.ses.len());
        }
        return Ok(());
    }
    if let Some(i) = args.iter().position(|a| a == "--svg") {
        let out = &args[i + 1];
        if let Some(j) = args.iter().position(|a| a == "--snapshots") {
            app.snaps = load_snaps(&PathBuf::from(&args[j + 1]));
        }
        let idx = args
            .iter()
            .position(|a| a == "--index")
            .and_then(|j| args[j + 1].parse().ok())
            .unwrap_or(app.snaps.len().saturating_sub(1));
        let mut snap = app.snaps.get(idx).cloned();
        if let (Some(s), Some(j)) = (snap.as_mut(), args.iter().position(|a| a == "--only")) {
            let board = app.board.as_ref().expect("no board");
            let names: Vec<&str> = args[j + 1].split(',').collect();
            let keep: Vec<usize> = board
                .nets
                .iter()
                .enumerate()
                .filter(|(_, n)| names.contains(&n.name.as_str()))
                .map(|(i, _)| i)
                .collect();
            s.wires.retain(|w| keep.contains(&w.net));
            s.vias.retain(|v| keep.contains(&v.net));
        }
        let (Some(board), Some(pv)) = (&app.board, &app.preview) else {
            eprintln!("usage: autorouter-gui board.dsn --svg out.svg [--snapshots DIR] [--index N] [--only NETS]");
            std::process::exit(2);
        };
        std::fs::write(out, to_svg(board, &app.pads, pv, snap.as_ref())).unwrap();
        println!("wrote {}", out);
        return Ok(());
    }
    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1500.0, 920.0])
            .with_title(concat!("PCB Autorouter | Built ", env!("AUTOROUTER_BUILD_DATE"))),
        ..Default::default()
    };
    let result = eframe::run_native("autorouter", opts, Box::new(|_cc| Ok(Box::new(app))));
    if let Err(e) = &result {
        log_error(&e.to_string());
    }
    result
}

fn log_error(message: &str) {
    use std::io::Write;
    let dir = std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("Autorouter");
    let _ = std::fs::create_dir_all(&dir);
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(dir.join("autorouter.log"))
    {
        let _ = writeln!(f, "{:?}: {}", std::time::SystemTime::now(), message);
    }
}

#[cfg(test)]
mod ui_tests {
    use super::*;

    #[test]
    fn net_search_stays_open_when_clicked_and_accepts_text() {
        let ctx = egui::Context::default();
        let mut filter = String::new();
        let mut assigned = vec![];
        let names = vec!["AGND".into(), "GND".into(), "VCC".into()];
        let mut frame = |events| {
            let mut rect = egui::Rect::NOTHING;
            let mut search = None;
            let output = ctx.run_ui(egui::RawInput { screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(640.0,480.0))), events, ..Default::default() }, |ui| {
                    let response = net_picker(ui, egui::Id::unique("test nets"), &mut filter, &names, &mut assigned);
                    rect = response.response.rect;
                    search = response.inner.map(|(_, rect)| rect);
            });
            output.drop_without_applying_deltas();
            (rect, search)
        };
        let click = |pos| vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
            egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
        ];
        let (button, _) = frame(vec![]);
        frame(click(button.center()));
        let (_, search) = frame(vec![]);
        let search = search.expect("menu opened");
        frame(click(search.center()));
        assert!(frame(vec![]).1.is_some(), "clicking search must not dismiss popup");
        assert!(frame(vec![egui::Event::Text("gnd".into())]).1.is_some());
        drop(frame);
        assert_eq!(filter, "gnd");
    }
}
