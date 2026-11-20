//! The placement tab: optional automatic placement of the parts and manual moving of parts
//! (also after routing, when connections stayed open). The result goes back to the router as
//! a new file ("Use this placement for routing").

use eframe::egui::{self, Color32, Pos2, Stroke};
use place::draw::{scene, Prim, SceneOpts, RED};
use place::engine::*;
use place::model::*;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

struct Shared {
    pl: Vec<Pl>,
    phase: String,
    frac: f32,
    result: Option<Result<Outcome, String>>,
}

pub struct PlaceState {
    doc: Option<place::Doc>,
    board: Option<Arc<Board>>,
    /// name of the file, for the default name of a saved board
    title: String,
    s: Settings,
    /// 0..=3 presets, 4 = fixed size
    size_mode: usize,
    max_w: f32,
    max_h: f32,
    room_idx: usize,
    clear_routing: bool,
    write_outline: bool,
    before: Vec<Pl>,
    after: Option<Outcome>,
    errs: Vec<String>,
    show_before: bool,
    shared: Arc<Mutex<Shared>>,
    running: bool,
    cancel: Arc<AtomicBool>,
    scale: f32,
    off: egui::Vec2,
    fitted: bool,
    rats: bool,
    gnd: bool,
    labels: bool,
    pin_nets: bool,
    show_caps: bool,
    select: Option<usize>,
    pub status: String,
    /// positions set by hand (and the original ones for the rest)
    manual: Vec<Pl>,
    /// lock state per part (file locks plus the user's)
    lock: Vec<bool>,
    drag: Option<(usize, (f64, f64))>,
    work: Option<Arc<Board>>,
    /// settings text as last written to disk
    saved: String,
}

const RED_C: Color32 = Color32::from_rgb(RED[0], RED[1], RED[2]);

impl PlaceState {
    pub fn new() -> PlaceState {
        let mut p = PlaceState {
            doc: None,
            board: None,
            title: String::new(),
            s: Settings::default(),
            size_mode: 2,
            max_w: 80.0,
            max_h: 60.0,
            room_idx: 2,
            clear_routing: true,
            write_outline: false,
            before: vec![],
            after: None,
            errs: vec![],
            show_before: false,
            shared: Arc::new(Mutex::new(Shared { pl: vec![], phase: String::new(), frac: 0.0, result: None })),
            running: false,
            cancel: Arc::new(AtomicBool::new(false)),
            scale: 1.0,
            off: egui::vec2(0.0, 0.0),
            fitted: false,
            rats: true,
            gnd: false,
            labels: true,
            pin_nets: true,
            show_caps: true,
            select: None,
            manual: vec![],
            lock: vec![],
            drag: None,
            work: None,
            saved: String::new(),
            status: String::new(),
        };
        p.load_cfg();
        p
    }

    pub fn running(&self) -> bool {
        self.running
    }

    fn cfg_path() -> PathBuf {
        let base = std::env::var_os("APPDATA").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
        base.join("autoplacer").join("settings.txt")
    }

    fn cfg_text(&self) -> String {
        let sides = match self.s.sides {
            Sides::Front => "front",
            Sides::Back => "back",
            Sides::Both => "both",
        };
        format!(
            "sides={}\nsize_mode={}\nmax_w={}\nmax_h={}\nroom_idx={}\nclearance={}\nedge_margin={}\nbig_pins={}\neffort={}\nseed={}\nbypass={}\nallow_rotate={}\nclear_routing={}\nwrite_outline={}\nrats={}\ngnd={}\nlabels={}\npin_nets={}\nshow_caps={}\n",
            sides, self.size_mode, self.max_w, self.max_h, self.room_idx, self.s.clearance, self.s.edge_margin, self.s.big_pins, self.s.effort, self.s.seed, self.s.bypass, self.s.allow_rotate, self.clear_routing, self.write_outline, self.rats, self.gnd, self.labels, self.pin_nets, self.show_caps
        )
    }

    /// restore the settings of the last session (all projects share them)
    fn load_cfg(&mut self) {
        let Ok(text) = std::fs::read_to_string(Self::cfg_path()) else { return };
        for l in text.lines() {
            let Some((k, v)) = l.split_once('=') else { continue };
            let b = || v == "true";
            match k {
                "sides" => self.s.sides = match v { "back" => Sides::Back, "both" => Sides::Both, _ => Sides::Front },
                "size_mode" => self.size_mode = v.parse().unwrap_or(2).min(4),
                "max_w" => self.max_w = v.parse().unwrap_or(self.max_w),
                "max_h" => self.max_h = v.parse().unwrap_or(self.max_h),
                "room_idx" => self.room_idx = v.parse().unwrap_or(2).min(3),
                "clearance" => self.s.clearance = v.parse().unwrap_or(self.s.clearance),
                "edge_margin" => self.s.edge_margin = v.parse().unwrap_or(self.s.edge_margin),
                "big_pins" => self.s.big_pins = v.parse().unwrap_or(self.s.big_pins),
                "effort" => self.s.effort = v.parse().unwrap_or(self.s.effort),
                "seed" => self.s.seed = v.parse().unwrap_or(self.s.seed),
                "bypass" => self.s.bypass = b(),
                "allow_rotate" => self.s.allow_rotate = b(),
                "clear_routing" => self.clear_routing = b(),
                "write_outline" => self.write_outline = b(),
                "rats" => self.rats = b(),
                "gnd" => self.gnd = b(),
                "labels" => self.labels = b(),
                "pin_nets" => self.pin_nets = b(),
                "show_caps" => self.show_caps = b(),
                _ => {}
            }
        }
        self.saved = self.cfg_text();
    }

    pub fn save_cfg(&mut self) {
        let t = self.cfg_text();
        if t == self.saved {
            return;
        }
        let p = Self::cfg_path();
        if let Some(d) = p.parent() {
            let _ = std::fs::create_dir_all(d);
        }
        if std::fs::write(&p, &t).is_ok() {
            self.saved = t;
        }
    }

    /// The file the router has loaded (also a placed one): the parts are shown and edited here.
    pub fn set_source(&mut self, raw: &[u8], title: &str) {
        if self.running {
            self.cancel.store(true, Ordering::Relaxed);
        }
        match place::load(raw) {
            Ok((doc, board)) => {
                self.before = board.parts.iter().map(Pl::of).collect();
                self.manual = self.before.clone();
                self.lock = board.parts.iter().map(|p| p.fixed()).collect();
                self.drag = None;
                self.status = format!(
                    "{} parts, {} nets{}",
                    board.parts.len(),
                    board.nets.len(),
                    if board.outline.is_some() { ", board outline found" } else { ", no board outline" }
                );
                self.doc = Some(doc);
                self.board = Some(Arc::new(board));
                self.after = None;
                self.running = false;
                self.errs.clear();
                self.select = None;
                self.fitted = false;
                self.show_before = false;
                self.title = title.to_string();
            }
            Err(e) => {
                self.doc = None;
                self.board = None;
                self.status = format!("the placer cannot read this file: {e}");
            }
        }
    }

    fn settings(&self) -> Settings {
        let mut s = self.s.clone();
        s.size = match self.size_mode {
            0 => Size::NoRoom,
            1 => Size::Tight,
            3 => Size::Relaxed,
            _ => Size::Normal,
        };
        s.max_size = if self.size_mode == 4 { Some((self.max_w as f64, self.max_h as f64)) } else { None };
        s.room = [0.0, 0.5, 1.0, 2.0][self.room_idx];
        s
    }

    fn run(&mut self) {
        let Some(orig) = self.board.clone() else { return };
        let mut wb = (*orig).clone();
        for i in 0..wb.parts.len() {
            if !orig.parts[i].movable {
                continue;
            }
            if self.lock[i] {
                let m = self.manual[i];
                wb.parts[i] = if m != Pl::of(&orig.parts[i]) {
                    orig.parts[i].baked(&m)
                } else {
                    let mut p = orig.parts[i].clone();
                    p.locked = true;
                    p
                };
            } else {
                wb.parts[i].locked = false;
            }
        }
        let board = Arc::new(wb);
        self.work = Some(board.clone());
        let s = self.settings();
        self.cancel.store(false, Ordering::Relaxed);
        {
            let mut g = self.shared.lock().unwrap();
            g.result = None;
            g.pl = self.manual.clone();
            g.phase = "starting".into();
            g.frac = 0.0;
        }
        self.running = true;
        self.after = None;
        self.errs.clear();
        let shared = self.shared.clone();
        let cancel = self.cancel.clone();
        std::thread::spawn(move || {
            let r = place(&board, &s, &mut |pl, phase, f| {
                if let Ok(mut g) = shared.lock() {
                    g.pl = pl.to_vec();
                    g.phase = phase.to_string();
                    g.frac = f;
                }
                !cancel.load(Ordering::Relaxed)
            });
            if let Ok(mut g) = shared.lock() {
                g.result = Some(r);
            }
        });
    }

    pub fn poll(&mut self) {
        if !self.running {
            return;
        }
        let done = self.shared.lock().unwrap().result.take();
        if let Some(r) = done {
            self.running = false;
            match r {
                Ok(o) => {
                    let b = self.work.as_ref().unwrap();
                    self.errs = verify(b, &self.settings(), &o);
                    self.status = format!("placed: wire length {:.0} -> {:.0} mm", o.hpwl_before, o.hpwl_after);
                    self.after = Some(o);
                    self.show_before = false;
                }
                Err(e) => self.status = format!("placement failed: {e}"),
            }
        }
    }

    /// placements of the result, with the hand-placed (locked) parts where the user put them
    fn final_pl(&self, o: &[Pl]) -> Vec<Pl> {
        (0..o.len()).map(|i| if self.lock.get(i).copied().unwrap_or(false) { self.manual[i] } else { o[i] }).collect()
    }

    /// the placement that "use for routing" and "save" take: the automatic result with the
    /// hand-placed parts, or only the hand-placed parts
    fn result_pl(&self) -> Vec<Pl> {
        match &self.after {
            Some(o) => self.final_pl(&o.pl),
            None => self.manual.clone(),
        }
    }

    /// The file with the placement applied (None: nothing moved, or an error in `status`).
    fn build_file(&mut self) -> Option<Vec<u8>> {
        let pls = self.result_pl();
        let (Some(doc), Some(board)) = (self.doc.as_ref(), self.board.as_ref()) else { return None };
        let outline = match &self.after {
            Some(o) if self.write_outline && board.outline.is_none() && doc.can_write_outline() => {
                let m = self.s.edge_margin + 0.2;
                Some([o.used[0] - m, o.used[1] - m, o.used[2] + m, o.used[3] + m])
            }
            _ => None,
        };
        let mut sb = (**board).clone();
        for p in &mut sb.parts {
            if p.movable {
                p.locked = false;
            }
        }
        let res = doc.save(&sb, &pls, self.clear_routing, outline);
        match res {
            Ok(d) => Some(d),
            Err(e) => {
                self.status = format!("cannot write the placement: {e}");
                None
            }
        }
    }

    fn moved_count(&self) -> usize {
        let Some(b) = &self.board else { return 0 };
        self.result_pl().iter().zip(&b.parts).filter(|(p, q)| **p != Pl::of(q)).count()
    }

    fn current_pl(&self) -> Vec<Pl> {
        if self.running {
            return self.final_pl(&self.shared.lock().unwrap().pl);
        }
        match (&self.after, self.show_before) {
            (Some(o), false) => self.final_pl(&o.pl),
            (Some(_), true) => self.before.clone(),
            _ => self.manual.clone(),
        }
    }

    /// Start editing: a shown result becomes the hand-editable state.
    fn begin_edit(&mut self) {
        if self.after.is_some() {
            self.manual = self.current_pl();
            self.after = None;
            self.show_before = false;
        }
    }

    fn rotate_sel(&mut self, steps: u8) {
        let (Some(i), Some(b)) = (self.select, self.board.clone()) else { return };
        let p = &b.parts[i];
        if !p.movable || !p.rot_ok {
            self.status = format!("{} cannot be turned", p.refdes);
            return;
        }
        self.begin_edit();
        self.turn(i, &b, |pl| pl.d = (pl.d + steps) & 3);
    }

    fn flip_sel(&mut self) {
        let (Some(i), Some(b)) = (self.select, self.board.clone()) else { return };
        let p = &b.parts[i];
        if !p.movable || !p.flip_ok {
            self.status = format!("{} cannot be put on the other side{}", p.refdes, if b.fmt == Fmt::Std { " (Standard files do not support it)" } else { "" });
            return;
        }
        self.begin_edit();
        self.turn(i, &b, |pl| pl.flip = !pl.flip);
    }

    /// change the orientation about the centre of the part and lock it
    fn turn(&mut self, i: usize, b: &Board, f: impl Fn(&mut Pl)) {
        let p = &b.parts[i];
        let mut pl = self.manual[i];
        let r = p.rect(&pl);
        f(&mut pl);
        let r2 = p.rect(&pl);
        pl.x += (r[0] + r[2]) / 2.0 - (r2[0] + r2[2]) / 2.0;
        pl.y += (r[1] + r[3]) / 2.0 - (r2[1] + r2[3]) / 2.0;
        self.manual[i] = pl;
        self.lock[i] = true;
    }

    fn toggle_lock_sel(&mut self) {
        let (Some(i), Some(b)) = (self.select, self.board.clone()) else { return };
        if !b.parts[i].movable {
            self.status = format!("{} is fixed and cannot be unlocked", b.parts[i].refdes);
            return;
        }
        self.begin_edit();
        self.lock[i] = !self.lock[i];
    }

    fn reset_sel(&mut self) {
        let (Some(i), Some(b)) = (self.select, self.board.clone()) else { return };
        self.begin_edit();
        self.manual[i] = Pl::of(&b.parts[i]);
        self.lock[i] = b.parts[i].fixed();
    }

    fn reset_all(&mut self) {
        let Some(b) = self.board.clone() else { return };
        self.after = None;
        self.show_before = false;
        self.manual = b.parts.iter().map(Pl::of).collect();
        self.lock = b.parts.iter().map(|p| p.fixed()).collect();
        self.errs.clear();
    }

    /// Indices (into the placer's nets) of the nets that are still open after routing.
    fn open_net_ids(&self, open: &[String]) -> Vec<usize> {
        let Some(b) = &self.board else { return vec![] };
        open.iter().filter_map(|n| b.nets.iter().position(|x| &x.name == n)).collect()
    }

    /// Side panel. Returns the new file when the user takes the placement over for routing.
    pub fn side_panel(&mut self, ui: &mut egui::Ui, open: &[String]) -> Option<Vec<u8>> {
        let mut take: Option<Vec<u8>> = None;
        let Some(board) = self.board.clone() else {
            ui.label("Open an EasyEDA Standard (.json) or Pro (.epro2) board first: the placement works on EasyEDA files.");
            if !self.status.is_empty() {
                ui.label(&self.status);
            }
            return None;
        };
        ui.label(&self.status);
        ui.separator();
        ui.heading("Place or move parts");
        ui.small("Optional. Moving parts by hand works without the automatic placement, also after routing: when connections stay open, move the parts in the way (the open nets are drawn in red), then use the placement for routing and route again.");
        if !open.is_empty() {
            ui.colored_label(Color32::from_rgb(255, 120, 120), format!("{} open net(s) after the last routing: {}", open.len(), open.iter().take(6).cloned().collect::<Vec<_>>().join(", ") + if open.len() > 6 { " ..." } else { "" }));
        }
        ui.separator();
        ui.strong("Automatic placement");
        ui.horizontal(|ui| {
            ui.label("Sides:");
            ui.radio_value(&mut self.s.sides, Sides::Front, "Front only").on_hover_text("Cheapest to assemble: every movable part goes to the front.");
            ui.radio_value(&mut self.s.sides, Sides::Back, "Back only");
            ui.radio_value(&mut self.s.sides, Sides::Both, "Front + back").on_hover_text("Parts may be flipped when that shortens the wiring (Pro files only).");
        });
        if let Some(o) = &board.outline {
            let bb = place::geom::bbox_of(o);
            ui.label(format!("Board outline found ({:.1} x {:.1} mm): every part stays inside.", bb[2] - bb[0], bb[3] - bb[1]));
        } else {
            ui.label("No board outline in the file – how tight may it get?");
            egui::ComboBox::from_id_salt("size").selected_text(["No room to breathe", "Tight", "Normal", "Relaxed", "Fixed size…"][self.size_mode]).show_ui(ui, |ui| {
                for (i, n) in ["No room to breathe (densest)", "Tight", "Normal", "Relaxed", "Fixed size (max PCB size)…"].iter().enumerate() {
                    ui.selectable_value(&mut self.size_mode, i, *n);
                }
            });
            if self.size_mode == 4 {
                ui.horizontal(|ui| {
                    ui.label("max W x H mm");
                    ui.add(egui::DragValue::new(&mut self.max_w).range(5.0..=1000.0).speed(0.5));
                    ui.add(egui::DragValue::new(&mut self.max_h).range(5.0..=1000.0).speed(0.5));
                });
            }
            ui.checkbox(&mut self.write_outline, "Add a board outline around the result (Standard files)");
        }
        egui::Grid::new("place_grid").num_columns(2).show(ui, |ui| {
            ui.label("Gap between parts");
            ui.add(egui::DragValue::new(&mut self.s.clearance).range(0.05..=3.0).speed(0.01).suffix(" mm"));
            ui.end_row();
            ui.label("Distance to edge");
            ui.add(egui::DragValue::new(&mut self.s.edge_margin).range(0.0..=5.0).speed(0.01).suffix(" mm"));
            ui.end_row();
            ui.label("Room around big chips");
            egui::ComboBox::from_id_salt("room").selected_text(["Off", "Low", "Normal", "Generous"][self.room_idx]).show_ui(ui, |ui| {
                for (i, n) in ["Off", "Low", "Normal", "Generous"].iter().enumerate() {
                    ui.selectable_value(&mut self.room_idx, i, *n);
                }
            });
            ui.end_row();
            ui.label("Big chip from");
            ui.add(egui::DragValue::new(&mut self.s.big_pins).range(4..=500).suffix(" pins"));
            ui.end_row();
            ui.label("Effort");
            ui.add(egui::Slider::new(&mut self.s.effort, 1..=10));
            ui.end_row();
            ui.label("Seed");
            ui.add(egui::DragValue::new(&mut self.s.seed).range(1..=999999));
            ui.end_row();
        });
        ui.checkbox(&mut self.s.bypass, "Decoupling caps right at the chip pins");
        ui.checkbox(&mut self.s.allow_rotate, "Allow turning parts");
        ui.horizontal(|ui| {
            if self.running {
                if ui.button("⏹ Stop").clicked() {
                    self.cancel.store(true, Ordering::Relaxed);
                }
                let g = self.shared.lock().unwrap();
                ui.add(egui::ProgressBar::new(g.frac).text(g.phase.clone()).desired_width(180.0));
            } else if ui.button("▶ Place parts").on_hover_text("Moves every part that is not locked (red). Parts you moved by hand are locked and stay.").clicked() {
                self.run();
            }
        });
        if let Some(o) = self.after.clone() {
            ui.separator();
            ui.strong("Result");
            ui.label(format!("Wire length: {:.0} → {:.0} mm ({:+.0}%)", o.hpwl_before, o.hpwl_after, (o.hpwl_after / o.hpwl_before.max(1e-9) - 1.0) * 100.0));
            if !o.caps.is_empty() {
                let avg = o.caps.iter().map(|c| c.2).sum::<f64>() / o.caps.len() as f64;
                let max = o.caps.iter().map(|c| c.2).fold(0.0, f64::max);
                ui.label(format!("{} decoupling caps: pad to pin avg {:.1} mm, worst {:.1} mm", o.caps.len(), avg, max));
            }
            ui.label(format!("Used area: {:.1} x {:.1} mm", o.used[2] - o.used[0], o.used[3] - o.used[1]));
            for n in &o.notes {
                ui.colored_label(Color32::from_rgb(255, 190, 90), n);
            }
            if self.errs.is_empty() {
                ui.colored_label(Color32::from_rgb(120, 220, 140), "Check passed: no overlaps, inside the board, locked parts unmoved");
            } else {
                for e in self.errs.iter().take(8) {
                    ui.colored_label(RED_C, e);
                }
            }
            ui.checkbox(&mut self.show_before, "Show original");
        }
        ui.separator();
        ui.strong("Use the placement");
        ui.checkbox(&mut self.clear_routing, "Remove existing tracks/vias of the file").on_hover_text("Moved parts make them wrong, the router starts from a clean board. Untick to keep the tracks that are in the file (those at moved parts then no longer fit).");
        let moved = self.moved_count();
        ui.label(format!("{moved} part(s) moved or turned"));
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(!self.running && moved > 0, egui::Button::new("➡ Use this placement for routing"))
                .on_hover_text("The router loads the board with the parts where they are now. Previous routing results are dropped; press Route again.")
                .clicked()
            {
                if let Some(d) = self.build_file() {
                    take = Some(d);
                }
            }
            if ui.add_enabled(!self.running && moved > 0, egui::Button::new("💾 Save placed board…")).clicked() {
                if let Some(d) = self.build_file() {
                    let stem = std::path::Path::new(&self.title).file_stem().and_then(|s| s.to_str()).unwrap_or("board").to_string();
                    let ext = self.doc.as_ref().map_or("json", |d| d.ext());
                    if let Some(out) = rfd::FileDialog::new().set_file_name(format!("{stem}-placed.{ext}")).add_filter("EasyEDA", &[ext]).save_file() {
                        self.status = match std::fs::write(&out, d) {
                            Ok(()) => format!("saved {}", out.display()),
                            Err(e) => format!("save failed: {e}"),
                        };
                    }
                }
            }
            if ui.add_enabled(!self.running && moved > 0, egui::Button::new("↺ Undo all moves")).clicked() {
                self.reset_all();
            }
        });
        ui.separator();
        ui.strong("View");
        ui.checkbox(&mut self.rats, "Ratsnest");
        ui.checkbox(&mut self.gnd, "…including ground nets");
        ui.checkbox(&mut self.labels, "Designators");
        ui.checkbox(&mut self.pin_nets, "Net names on pins (zoom in)");
        ui.checkbox(&mut self.show_caps, "Decoupling links");
        let locked: Vec<usize> = (0..board.parts.len()).filter(|&i| self.lock[i] || !board.parts[i].movable).collect();
        ui.separator();
        ui.strong("Manual placement");
        ui.small("Click a part to select it, drag to move. R / Shift+R turn, F flip, L lock or unlock. Moved or turned parts lock themselves: the automatic placement works around them.");
        if let Some(i) = self.select {
            let p = &board.parts[i];
            ui.label(format!("Selected: {} {} ({} pads)", p.refdes, p.package, p.pads.len()));
            ui.horizontal_wrapped(|ui| {
                let mut l = self.lock[i];
                if ui.add_enabled(p.movable && !self.running, egui::Checkbox::new(&mut l, "🔒 Locked")).changed() {
                    self.toggle_lock_sel();
                }
                if ui.add_enabled(!self.running, egui::Button::new("⟲")).on_hover_text("turn left (R)").clicked() {
                    self.rotate_sel(1);
                }
                if ui.add_enabled(!self.running, egui::Button::new("⟳")).on_hover_text("turn right (Shift+R)").clicked() {
                    self.rotate_sel(3);
                }
                if ui.add_enabled(!self.running, egui::Button::new("⇅ Flip")).on_hover_text("other board side (F)").clicked() {
                    self.flip_sel();
                }
                if ui.add_enabled(!self.running, egui::Button::new("Reset")).on_hover_text("back to the position in the file").clicked() {
                    self.reset_sel();
                }
            });
        } else {
            ui.small("(no part selected)");
        }
        ui.separator();
        ui.colored_label(RED_C, format!("🔒 Locked parts: {} (never moved by the automatic placement)", locked.len()));
        egui::ScrollArea::vertical().id_salt("locked_list").max_height(160.0).show(ui, |ui| {
            for i in locked {
                let p = &board.parts[i];
                let t = format!("{} {}{}", p.refdes, p.package, if !p.movable { " (fixed)" } else if p.locked { " (from file)" } else { "" });
                if ui.selectable_label(self.select == Some(i), egui::RichText::new(t).color(RED_C)).clicked() {
                    self.select = Some(i);
                }
            }
        });
        for n in board.notes.iter().take(6) {
            ui.small(n);
        }
        take
    }

    pub fn canvas(&mut self, ui: &mut egui::Ui, open: &[String]) {
        let ctx = ui.ctx().clone();
        let (resp, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
        let rect = resp.rect;
        painter.rect_filled(rect, 0.0, Color32::from_rgb(18, 18, 22));
        let Some(board) = self.board.clone() else {
            painter.text(rect.center(), egui::Align2::CENTER_CENTER, "Open an EasyEDA board to place or move its parts", egui::FontId::proportional(20.0), Color32::GRAY);
            return;
        };
        let pl = self.current_pl();
        let region: Vec<P> = match (&self.after, self.show_before) {
            (Some(o), false) if board.outline.is_none() => o.region.clone(),
            _ => vec![],
        };
        if !self.fitted {
            let b = place::draw::bounds(&board, &pl, &region);
            let (w, h) = (((b[2] - b[0]) as f32).max(1.0), ((b[3] - b[1]) as f32).max(1.0));
            self.scale = (rect.width() / w).min(rect.height() / h) * 0.92;
            self.off = egui::vec2(rect.center().x - (b[0] + b[2]) as f32 / 2.0 * self.scale, rect.center().y + (b[1] + b[3]) as f32 / 2.0 * self.scale);
            self.fitted = true;
        }
        let (sc0, off0) = (self.scale, self.off);
        let to_world = move |m: Pos2| (((m.x - off0.x) / sc0) as f64, (-(m.y - off0.y) / sc0) as f64);
        let hit = |w: (f64, f64), pl: &[Pl]| {
            (0..board.parts.len()).rev().find(|&i| {
                let r = board.parts[i].rect(&pl[i]);
                w.0 >= r[0] && w.0 <= r[2] && w.1 >= r[1] && w.1 <= r[3]
            })
        };
        if !self.running && resp.drag_started() {
            if let Some(m) = resp.interact_pointer_pos() {
                let w = to_world(m);
                if let Some(i) = hit(w, &pl) {
                    if board.parts[i].movable {
                        self.begin_edit();
                        let cur = self.manual[i];
                        self.drag = Some((i, (w.0 - cur.x, w.1 - cur.y)));
                        self.select = Some(i);
                    }
                }
            }
        }
        if let Some((i, o)) = self.drag {
            if resp.dragged() {
                if let Some(m) = resp.interact_pointer_pos() {
                    let w = to_world(m);
                    let mut p = self.manual[i];
                    p.x = ((w.0 - o.0) / 0.05).round() * 0.05;
                    p.y = ((w.1 - o.1) / 0.05).round() * 0.05;
                    self.manual[i] = p;
                    self.lock[i] = true;
                }
            } else {
                self.drag = None;
            }
        } else if resp.dragged() {
            self.off += resp.drag_delta();
        }
        if !ctx.egui_wants_keyboard_input() && !self.running {
            let (r, shift, f, l) = ctx.input(|s| (s.key_pressed(egui::Key::R), s.modifiers.shift, s.key_pressed(egui::Key::F), s.key_pressed(egui::Key::L)));
            if r {
                self.rotate_sel(if shift { 3 } else { 1 });
            }
            if f {
                self.flip_sel();
            }
            if l {
                self.toggle_lock_sel();
            }
        }
        let pl = self.current_pl();
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
        let (scale, off) = (self.scale, self.off);
        let tr = |p: P| Pos2::new(p.0 as f32 * scale + off.x, -p.1 as f32 * scale + off.y);
        if resp.clicked() {
            if let Some(m) = resp.interact_pointer_pos() {
                let w = (((m.x - off.x) / scale) as f64, (-(m.y - off.y) / scale) as f64);
                self.select = (0..board.parts.len()).rev().find(|&i| {
                    let r = board.parts[i].rect(&pl[i]);
                    w.0 >= r[0] && w.0 <= r[2] && w.1 >= r[1] && w.1 <= r[3]
                });
            }
        }
        let caps: Vec<(usize, usize, f64)> = if self.show_caps && !self.running && !self.show_before { self.after.as_ref().map(|o| o.caps.clone()).unwrap_or_default() } else { vec![] };
        let opts = SceneOpts { rats: self.rats, gnd: self.gnd, labels: self.labels, select: self.select, caps: &caps, region: &region, locked: &self.lock, pin_nets: self.pin_nets };
        let c32 = |c: [u8; 4]| Color32::from_rgba_unmultiplied(c[0], c[1], c[2], c[3]);
        let clip = painter.with_clip_rect(rect);
        for prim in scene(&board, &pl, &opts) {
            match prim {
                Prim::Poly { pts, fill, stroke } => {
                    let v: Vec<Pos2> = pts.iter().map(|&p| tr(p)).collect();
                    let st = stroke.map_or(Stroke::NONE, |(c, w)| Stroke::new((w as f32 * scale).max(1.0), c32(c)));
                    match fill {
                        Some(f) if v.len() == 4 => {
                            clip.add(egui::Shape::convex_polygon(v, c32(f), st));
                        }
                        _ => {
                            clip.add(egui::Shape::closed_line(v, st));
                        }
                    }
                }
                Prim::Line { a, b, col, w } => {
                    clip.line_segment([tr(a), tr(b)], Stroke::new((w as f32 * scale).max(1.0), c32(col)));
                }
                Prim::Text { at, s, size, col } => {
                    let px = size as f32 * scale;
                    if px > 6.0 {
                        clip.text(tr(at), egui::Align2::CENTER_CENTER, s, egui::FontId::proportional(px.min(40.0)), c32(col));
                    }
                }
            }
        }
        // the nets that stayed open after routing: every pin joined to the nearest one before it
        for ni in self.open_net_ids(open) {
            let net = &board.nets[ni];
            let pts: Vec<P> = net.pins.iter().map(|&(p, k)| board.parts[p].pad_xy(k, &pl[p])).collect();
            for k in 1..pts.len() {
                let near = (0..k).min_by(|&a, &b| {
                    let d = |q: P| (q.0 - pts[k].0).powi(2) + (q.1 - pts[k].1).powi(2);
                    d(pts[a]).total_cmp(&d(pts[b]))
                });
                if let Some(a) = near {
                    clip.line_segment([tr(pts[a]), tr(pts[k])], Stroke::new(2.5, Color32::from_rgb(255, 70, 70)));
                }
            }
        }
        if let Some(i) = self.select {
            if let Some(p) = board.parts.get(i) {
                painter.text(rect.left_top() + egui::vec2(10.0, 10.0), egui::Align2::LEFT_TOP, format!("{}  {}  {} pads{}", p.refdes, p.package, p.pads.len(), if p.fixed() { "  🔒 locked" } else { "" }), egui::FontId::proportional(15.0), Color32::WHITE);
            }
        }
    }
}
