//! Format independent board model. Units: mm, y up. A part is described by its *current*
//! orientation: pad/body offsets are relative to the part origin as the part sits in the file.
//! A placement is an origin plus a quarter-turn count (counter-clockwise) and a flip (mirror x
//! first, then rotate) applied on top of that.

pub type P = (f64, f64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fmt {
    Std,
    Pro,
}

#[derive(Clone, Debug)]
pub struct PadG {
    pub num: String,
    /// net index or -1
    pub net: i32,
    /// pad centre relative to the part origin
    pub dx: f64,
    pub dy: f64,
    /// half extents of the pad's bounding box
    pub hx: f64,
    pub hy: f64,
    pub tht: bool,
}

#[derive(Clone, Debug)]
pub struct Part {
    pub refdes: String,
    pub package: String,
    pub x: f64,
    pub y: f64,
    pub bottom: bool,
    pub locked: bool,
    /// the writer can move this part (false: kept fixed, treated like a locked one)
    pub movable: bool,
    /// the writer can rotate it
    pub rot_ok: bool,
    /// the writer can put it on the other side
    pub flip_ok: bool,
    pub tht: bool,
    pub pads: Vec<PadG>,
    /// body (pads + component outline) relative to the origin: minx, miny, maxx, maxy
    pub body: [f64; 4],
    /// index into the source document (None: loose pad / hole of the board)
    pub src: Option<usize>,
}

impl Part {
    /// true when the placer must not touch it
    pub fn fixed(&self) -> bool {
        self.locked || !self.movable
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetKind {
    Gnd,
    Power,
    Signal,
}

#[derive(Clone, Debug)]
pub struct Net {
    pub name: String,
    /// (part, pad)
    pub pins: Vec<(usize, usize)>,
    pub kind: NetKind,
}

#[derive(Clone, Debug)]
pub struct Board {
    pub fmt: Fmt,
    pub title: String,
    pub parts: Vec<Part>,
    pub nets: Vec<Net>,
    /// board outline, if the file has one
    pub outline: Option<Vec<P>>,
    pub cutouts: Vec<Vec<P>>,
    pub notes: Vec<String>,
    /// number of tracks / vias that are on the board (they become invalid when parts move)
    pub routing: usize,
}

/// Where a part is placed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pl {
    pub x: f64,
    pub y: f64,
    /// quarter turns, counter-clockwise
    pub d: u8,
    pub flip: bool,
}

impl Pl {
    pub fn of(p: &Part) -> Pl {
        Pl { x: p.x, y: p.y, d: 0, flip: false }
    }
}

pub fn classify_net(name: &str) -> NetKind {
    let n = name.trim().to_uppercase();
    if n.is_empty() {
        return NetKind::Signal;
    }
    if n.contains("GND") || n.contains("GROUND") || n.contains("VSS") || n == "0V" || n == "COM" || n == "VGND" {
        return NetKind::Gnd;
    }
    // +3V3, -5V, 12V, 3.3V, +1V8_CORE ...
    let t = n.trim_start_matches(|c| c == '+' || c == '-' || c == '_');
    let digits = t.chars().take_while(|c| c.is_ascii_digit() || *c == '.').count();
    if digits > 0 && t[digits..].starts_with('V') {
        return NetKind::Power;
    }
    const HAS: [&str; 9] = ["VCC", "VDD", "VEE", "VBUS", "VBAT", "VCORE", "VSYS", "VPWR", "VMOT"];
    const START: [&str; 6] = ["VIN", "VOUT", "VREF", "VPP", "VAA", "VIO"];
    if HAS.iter().any(|k| n.contains(k)) || START.iter().any(|k| n.starts_with(k)) || n == "V+" || n == "V-" || n.starts_with("PWR") || n.ends_with("_PWR") {
        return NetKind::Power;
    }
    NetKind::Signal
}

/// offset under (quarter turns, flip)
pub fn xf_off(dx: f64, dy: f64, d: u8, flip: bool) -> P {
    let dx = if flip { -dx } else { dx };
    match d & 3 {
        0 => (dx, dy),
        1 => (-dy, dx),
        2 => (-dx, -dy),
        _ => (dy, -dx),
    }
}

impl Part {
    /// The part as it sits under `pl`, with that orientation made its own and locked.
    pub fn baked(&self, pl: &Pl) -> Part {
        let mut p = self.clone();
        for q in &mut p.pads {
            let o = xf_off(q.dx, q.dy, pl.d, pl.flip);
            q.dx = o.0;
            q.dy = o.1;
            if pl.d & 1 == 1 {
                std::mem::swap(&mut q.hx, &mut q.hy);
            }
        }
        let r = self.rect(pl);
        p.body = [r[0] - pl.x, r[1] - pl.y, r[2] - pl.x, r[3] - pl.y];
        p.x = pl.x;
        p.y = pl.y;
        p.bottom ^= pl.flip;
        p.locked = true;
        p
    }

    /// world centre of pad `i` under placement `pl`
    pub fn pad_xy(&self, i: usize, pl: &Pl) -> P {
        let p = &self.pads[i];
        let o = xf_off(p.dx, p.dy, pl.d, pl.flip);
        (pl.x + o.0, pl.y + o.1)
    }

    /// world bounding box of pad `i`
    pub fn pad_box(&self, i: usize, pl: &Pl) -> [f64; 4] {
        let p = &self.pads[i];
        let c = self.pad_xy(i, pl);
        let (hx, hy) = if pl.d & 1 == 1 { (p.hy, p.hx) } else { (p.hx, p.hy) };
        [c.0 - hx, c.1 - hy, c.0 + hx, c.1 + hy]
    }

    /// world bounding box of the body
    pub fn rect(&self, pl: &Pl) -> [f64; 4] {
        let b = self.body;
        let mut r = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for c in [(b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3])] {
            let o = xf_off(c.0, c.1, pl.d, pl.flip);
            let w = (pl.x + o.0, pl.y + o.1);
            r[0] = r[0].min(w.0);
            r[1] = r[1].min(w.1);
            r[2] = r[2].max(w.0);
            r[3] = r[3].max(w.1);
        }
        r
    }
}
