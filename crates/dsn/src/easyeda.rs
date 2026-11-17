//! EasyEDA Standard PCB source JSON ("File > Export > EasyEDA source") as router input.
//!
//! The JSON is turned into the same Specctra DSN text EasyEDA itself would export (all pads in
//! one image `u1`, coordinates in EasyEDA units = 10 mil = 0.254 mm, x right and y up from the
//! board's top-left corner), so everything downstream works unchanged. The original document
//! is kept so the result can be written back into it.

use serde_json::Value;
use std::collections::HashSet;
use std::fmt::Write;

/// EasyEDA length unit in millimetres.
pub const PX_MM: f64 = 0.254;

/// Added to every clearance. EasyEDA's own DSN export lists 0.6034 as the default clearance where
/// the rule says 0.5984, and its DRC flags gaps in between; routing 0.01 wider is never flagged.
pub(crate) const CLEARANCE_MARGIN: f64 = 0.01;

pub struct EasyedaDoc {
    pub root: Value,
    /// every footprint of the document, in file order
    pub parts: Vec<PartInfo>,
    /// DSN text generated from the document
    pub dsn: String,
    /// JSON x = DSN x + ox, JSON y = oy - DSN y
    pub ox: f64,
    pub oy: f64,
    pub notes: Vec<String>,
    /// (net, layer id) of every copper area already in the document
    pub areas: HashSet<(String, String)>,
    /// every object id in the document (new ids must not collide)
    pub ids: HashSet<String>,
    /// set when the document is an EasyEDA Pro project (`.epro2` / `.epro`); `root` is empty then
    pub pro: Option<crate::easyeda_pro::ProDoc>,
    /// The tracks and vias that were already on the board, as the writer finds them: indices into
    /// the `shape` list (Standard JSON) or line numbers of the PCB (Pro). Footprint contents are
    /// not listed.
    pub existing: Vec<usize>,
}

impl EasyedaDoc {
    /// Move footprints in `root`: (part index, dx, dy) in DSN coordinates (y up from the board's
    /// top-left corner, like everything the router sees). Silkscreen, text and 3D outline move
    /// with the pads.
    pub fn apply_part_moves(&self, root: &mut Value, moves: &[(usize, f64, f64)]) {
        let Some(arr) = root.get_mut("shape").and_then(|s| s.as_array_mut()) else {
            return;
        };
        for &(pi, dx, dy) in moves {
            let Some(p) = self.parts.get(pi) else { continue };
            if let Some(Value::String(s)) = arr.get_mut(p.shape) {
                if let Some(moved) = shift_lib(s, dx, -dy) {
                    *s = moved;
                }
            }
        }
    }

    /// Router coordinates to document coordinates. For a Pro project this is still the Standard
    /// space (10 mil units, y down); `ProDoc::from_std` turns it into mil, y up.
    pub fn to_json_xy(&self, p: (f64, f64)) -> (f64, f64) {
        (p.0 + self.ox, self.oy - p.1)
    }
}


/// One footprint (LIB shape) of the document.
#[derive(Debug, Clone)]
pub struct PartInfo {
    /// index into the document's `shape` array
    pub shape: usize,
    /// footprint name (the `package` attribute)
    pub package: String,
    /// reference designator (C1, R12, ...), empty when unknown
    pub designator: String,
    /// pin numbers (the `N` of `u1-N`) of its pads
    pub pads: Vec<usize>,
    /// every pad is a surface-mount pad and the part has no holes
    pub smd: bool,
    /// all its objects can be moved (known shape kinds only)
    pub movable: bool,
}

fn shift_num(v: f64, d: f64) -> String {
    num(v + d)
}

/// "x y x y ..." (or "x,y") shifted by (dx, dy).
fn shift_pairs(s: &str, dx: f64, dy: f64) -> Option<String> {
    let comma = s.contains(',') && !s.contains(' ');
    let v: Vec<f64> = s
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
        .map(|t| t.parse().ok())
        .collect::<Option<Vec<f64>>>()?;
    if v.len() % 2 != 0 {
        return None;
    }
    let sep = if comma { "," } else { " " };
    Some(
        v.chunks_exact(2)
            .map(|c| format!("{}{}{}", shift_num(c[0], dx), sep, shift_num(c[1], dy)))
            .collect::<Vec<_>>()
            .join(if comma { " " } else { " " }),
    )
}

/// SVG-like path (M, L, A, Z) shifted by (dx, dy); anything else is refused.
fn shift_path(s: &str, dx: f64, dy: f64) -> Option<String> {
    let toks: Vec<&str> = s.split_whitespace().collect();
    let mut out: Vec<String> = vec![];
    let mut cmd = ' ';
    let mut k = 0; // numbers seen since the command letter
    for t in toks {
        if t.len() == 1 && t.chars().all(|c| c.is_ascii_alphabetic()) {
            if !matches!(t, "M" | "L" | "A" | "Z" | "C" | "Q") {
                return None;
            }
            cmd = t.chars().next()?;
            k = 0;
            out.push(t.to_string());
            continue;
        }
        let v: f64 = t.parse().ok()?;
        let shifted = match cmd {
            'M' | 'L' | 'C' | 'Q' => {
                if k % 2 == 0 { shift_num(v, dx) } else { shift_num(v, dy) }
            }
            'A' => match k % 7 {
                5 => shift_num(v, dx),
                6 => shift_num(v, dy),
                _ => t.to_string(),
            },
            _ => return None,
        };
        k += 1;
        out.push(shifted);
    }
    Some(out.join(" "))
}

/// Shift the string value of `"key":"..."` occurrences inside a JSON text.
fn shift_json_attr(text: &str, key: &str, dx: f64, dy: f64) -> Option<String> {
    let pat = format!("\"{key}\":\"");
    let mut out = String::new();
    let mut rest = text;
    while let Some(i) = rest.find(&pat) {
        out.push_str(&rest[..i + pat.len()]);
        rest = &rest[i + pat.len()..];
        let end = rest.find('"')?;
        out.push_str(&shift_pairs(&rest[..end], dx, dy)?);
        rest = &rest[end..];
    }
    out.push_str(rest);
    Some(out)
}

/// A whole footprint shape (`LIB~x~y~...#@$child#@$child...`) moved by (dx, dy) in document
/// coordinates. None when it holds something this does not know how to move.
pub fn shift_lib(shape: &str, dx: f64, dy: f64) -> Option<String> {
    let mut out: Vec<String> = vec![];
    for (k, part) in shape.split("#@$").enumerate() {
        let mut t: Vec<String> = part.split('~').map(|x| x.to_string()).collect();
        let setnum = |t: &mut Vec<String>, i: usize, d: f64| -> Option<()> {
            let v: f64 = t.get(i)?.parse().ok()?;
            t[i] = shift_num(v, d);
            Some(())
        };
        let tag = t[0].clone();
        if k == 0 {
            if tag != "LIB" {
                return None;
            }
            setnum(&mut t, 1, dx)?;
            setnum(&mut t, 2, dy)?;
        } else {
            match tag.as_str() {
                "TEXT" => {
                    setnum(&mut t, 2, dx)?;
                    setnum(&mut t, 3, dy)?;
                    let p = t.get(11)?.clone();
                    t[11] = shift_path(&p, dx, dy)?;
                }
                "CIRCLE" | "HOLE" | "VIA" | "RECT" => {
                    setnum(&mut t, 1, dx)?;
                    setnum(&mut t, 2, dy)?;
                }
                "SOLIDREGION" => {
                    let p = t.get(3)?.clone();
                    t[3] = shift_path(&p, dx, dy)?;
                }
                "ARC" => {
                    let p = t.get(4)?.clone();
                    t[4] = shift_path(&p, dx, dy)?;
                }
                "TRACK" => {
                    let p = t.get(4)?.clone();
                    t[4] = shift_pairs(&p, dx, dy)?;
                }
                "PAD" => {
                    setnum(&mut t, 2, dx)?;
                    setnum(&mut t, 3, dy)?;
                    if !t.get(10)?.is_empty() {
                        let p = t[10].clone();
                        t[10] = shift_pairs(&p, dx, dy)?;
                    }
                    let last = t.len() - 1;
                    if t[last].contains(',') && t[last].parse::<f64>().is_err() {
                        let p = t[last].clone();
                        t[last] = shift_pairs(&p, dx, dy)?;
                    }
                }
                "SVGNODE" => {
                    // JSON after the first `~`; only coordinates are touched
                    let body = part.splitn(2, '~').nth(1)?;
                    let a = shift_json_attr(body, "c_origin", dx, dy)?;
                    let b = shift_json_attr(&a, "points", dx, dy)?;
                    out.push(format!("SVGNODE~{b}"));
                    continue;
                }
                _ => return None,
            }
        }
        out.push(t.join("~"));
    }
    Some(out.join("#@$"))
}


// ---------------------------------------------------------------- copper objects of any shape
#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Cmd(char),
    Num(f64),
}

/// "M 1 2 L 3 4 A r r 0 0 1 x y Z", also glued ("L4096.5,3563") and comma separated.
fn tokens(s: &str) -> Vec<Tok> {
    let b: Vec<char> = s.chars().collect();
    let mut out = vec![];
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_alphabetic() && c != 'e' && c != 'E' {
            out.push(Tok::Cmd(c));
            i += 1;
        } else if c.is_ascii_digit() || c == '-' || c == '+' || c == '.' {
            let st = i;
            i += 1;
            while i < b.len() {
                let d = b[i];
                let exp = (d == '-' || d == '+') && matches!(b[i - 1], 'e' | 'E');
                if d.is_ascii_digit() || d == '.' || d == 'e' || d == 'E' || exp {
                    i += 1;
                } else {
                    break;
                }
            }
            if let Ok(v) = b[st..i].iter().collect::<String>().parse::<f64>() {
                out.push(Tok::Num(v));
            }
        } else {
            i += 1;
        }
    }
    out
}

fn arc_points(p0: (f64, f64), rx: f64, ry: f64, rot: f64, large: bool, sweep: bool, p1: (f64, f64)) -> Vec<(f64, f64)> {
    let (mut rx, mut ry) = (rx.abs(), ry.abs());
    if rx < 1e-9 || ry < 1e-9 || (p0.0 - p1.0).abs() + (p0.1 - p1.1).abs() < 1e-12 {
        return vec![p1];
    }
    let phi = rot.to_radians();
    let (c, s) = (phi.cos(), phi.sin());
    let (dx, dy) = ((p0.0 - p1.0) / 2.0, (p0.1 - p1.1) / 2.0);
    let (x1, y1) = (c * dx + s * dy, -s * dx + c * dy);
    let lam = (x1 * x1) / (rx * rx) + (y1 * y1) / (ry * ry);
    if lam > 1.0 {
        let k = lam.sqrt();
        rx *= k;
        ry *= k;
    }
    let num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
    let den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
    let mut co = if den.abs() < 1e-12 { 0.0 } else { (num / den).max(0.0).sqrt() };
    if large == sweep {
        co = -co;
    }
    let (cxp, cyp) = (co * rx * y1 / ry, -co * ry * x1 / rx);
    let (cx, cy) = (c * cxp - s * cyp + (p0.0 + p1.0) / 2.0, s * cxp + c * cyp + (p0.1 + p1.1) / 2.0);
    let ang = |ux: f64, uy: f64, vx: f64, vy: f64| -> f64 {
        let a = (ux * vy - uy * vx).atan2(ux * vx + uy * vy);
        a
    };
    let th1 = ang(1.0, 0.0, (x1 - cxp) / rx, (y1 - cyp) / ry);
    let mut dth = ang((x1 - cxp) / rx, (y1 - cyp) / ry, (-x1 - cxp) / rx, (-y1 - cyp) / ry);
    if !sweep && dth > 0.0 {
        dth -= std::f64::consts::TAU;
    } else if sweep && dth < 0.0 {
        dth += std::f64::consts::TAU;
    }
    let n = ((dth.abs() / (std::f64::consts::PI / 18.0)).ceil() as usize).max(2);
    (1..=n)
        .map(|k| {
            let t = th1 + dth * k as f64 / n as f64;
            let (ex, ey) = (rx * t.cos(), ry * t.sin());
            (c * ex - s * ey + cx, s * ex + c * ey + cy)
        })
        .collect()
}

/// Sub-paths (open or closed) of an SVG-like path made of M, L, H, V, A, C, Q, Z.
fn path_polylines(s: &str) -> Vec<Vec<(f64, f64)>> {
    let tk = tokens(s);
    let mut out: Vec<Vec<(f64, f64)>> = vec![];
    let mut cur: Vec<(f64, f64)> = vec![];
    let mut pos = (0.0, 0.0);
    let mut start = (0.0, 0.0);
    let mut i = 0;
    let num = |i: usize| -> Option<f64> {
        match tk.get(i) {
            Some(Tok::Num(v)) => Some(*v),
            _ => None,
        }
    };
    let mut cmd = 'M';
    while i < tk.len() {
        if let Tok::Cmd(c) = tk[i] {
            cmd = c.to_ascii_uppercase();
            i += 1;
            if cmd == 'Z' {
                if !cur.is_empty() {
                    cur.push(start);
                    out.push(std::mem::take(&mut cur));
                }
                pos = start;
            }
            continue;
        }
        match cmd {
            'M' | 'L' => {
                let (Some(x), Some(y)) = (num(i), num(i + 1)) else { break };
                i += 2;
                if cmd == 'M' {
                    if cur.len() >= 2 {
                        out.push(std::mem::take(&mut cur));
                    } else {
                        cur.clear();
                    }
                    start = (x, y);
                    cur.push((x, y));
                    cmd = 'L';
                } else {
                    if cur.is_empty() {
                        cur.push(pos);
                    }
                    cur.push((x, y));
                }
                pos = (x, y);
            }
            'H' | 'V' => {
                let Some(v) = num(i) else { break };
                i += 1;
                let p = if cmd == 'H' { (v, pos.1) } else { (pos.0, v) };
                if cur.is_empty() {
                    cur.push(pos);
                }
                cur.push(p);
                pos = p;
            }
            'A' => {
                let v: Vec<Option<f64>> = (0..7).map(|k| num(i + k)).collect();
                if v.iter().any(|x| x.is_none()) {
                    break;
                }
                let v: Vec<f64> = v.into_iter().flatten().collect();
                i += 7;
                if cur.is_empty() {
                    cur.push(pos);
                }
                let p1 = (v[5], v[6]);
                cur.extend(arc_points(pos, v[0], v[1], v[2], v[3] != 0.0, v[4] != 0.0, p1));
                pos = p1;
            }
            'C' | 'Q' => {
                let k = if cmd == 'C' { 6 } else { 4 };
                let v: Vec<Option<f64>> = (0..k).map(|q| num(i + q)).collect();
                if v.iter().any(|x| x.is_none()) {
                    break;
                }
                let v: Vec<f64> = v.into_iter().flatten().collect();
                i += k;
                if cur.is_empty() {
                    cur.push(pos);
                }
                for step in 1..=10 {
                    let t = step as f64 / 10.0;
                    let u = 1.0 - t;
                    let p = if cmd == 'C' {
                        (
                            u * u * u * pos.0 + 3.0 * u * u * t * v[0] + 3.0 * u * t * t * v[2] + t * t * t * v[4],
                            u * u * u * pos.1 + 3.0 * u * u * t * v[1] + 3.0 * u * t * t * v[3] + t * t * t * v[5],
                        )
                    } else {
                        (
                            u * u * pos.0 + 2.0 * u * t * v[0] + t * t * v[2],
                            u * u * pos.1 + 2.0 * u * t * v[1] + t * t * v[3],
                        )
                    };
                    cur.push(p);
                }
                pos = (v[k - 2], v[k - 1]);
            }
            _ => {
                i += 1;
            }
        }
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

/// A copper layer id of the document (top, bottom, enabled inner layers) or `None`.
/// "11" (multi-layer) is every copper layer.
fn copper_layers(id: &str, layers: &[String]) -> Vec<String> {
    match id {
        "11" => layers.to_vec(),
        _ if layers.iter().any(|l| l == id) => vec![id.to_string()],
        _ => vec![],
    }
}

/// A copper object that is not a plain track or pad.
pub(crate) struct Stroke {
    pub(crate) width: f64,
    pub(crate) layer: String,
    pub(crate) net: String,
    pub(crate) pts: Vec<(f64, f64)>,
}

pub(crate) struct Region {
    pub(crate) layer: String,
    pub(crate) net: String,
    pub(crate) pts: Vec<(f64, f64)>,
    /// a copper area or a solid region that is copper (otherwise: keep everything out)
    pub(crate) copper: bool,
}

pub(crate) struct Pad {
    pub(crate) shape: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) w: f64,
    pub(crate) h: f64,
    pub(crate) layer: String,
    pub(crate) net: String,
    pub(crate) pts: Vec<(f64, f64)>,
    pub(crate) rot: f64,
}

fn num(v: f64) -> String {
    let s = format!("{:.4}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" || s.is_empty() {
        "0".into()
    } else {
        s
    }
}

fn pts_of(s: &str) -> Vec<(f64, f64)> {
    let v: Vec<f64> = s
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
        .filter_map(|t| t.parse().ok())
        .collect();
    v.chunks_exact(2).map(|c| (c[0], c[1])).collect()
}

fn f(s: Option<&&str>) -> f64 {
    s.and_then(|t| t.parse().ok()).unwrap_or(0.0)
}

/// "1px", "0.254mm", "10mil", "1" (px) -> EasyEDA units
fn length_px(s: &str) -> Option<f64> {
    let t = s.trim();
    let split = t.find(|c: char| c.is_alphabetic()).unwrap_or(t.len());
    let v: f64 = t[..split].trim().parse().ok()?;
    Some(match t[split..].trim().to_lowercase().as_str() {
        "mm" => v / PX_MM,
        "mil" => v / 10.0,
        "inch" | "in" => v * 100.0,
        _ => v,
    })
}

fn quote(s: &str) -> String {
    if s.chars().any(|c| c.is_whitespace() || "()\"".contains(c)) {
        format!("\"{}\"", s.replace('"', ""))
    } else {
        s.to_string()
    }
}

/// Stadium (slot) around the segment a-b with radius r, relative to `c`.
pub(crate) fn stadium(a: (f64, f64), b: (f64, f64), r: f64, c: (f64, f64)) -> Vec<(f64, f64)> {
    let ang = (b.1 - a.1).atan2(b.0 - a.0);
    let mut out = vec![];
    let n = 12;
    for k in 0..=n {
        let t = ang - std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / n as f64;
        out.push((b.0 + r * t.cos() - c.0, b.1 + r * t.sin() - c.1));
    }
    for k in 0..=n {
        let t = ang + std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * k as f64 / n as f64;
        out.push((a.0 + r * t.cos() - c.0, a.1 + r * t.sin() - c.1));
    }
    out
}

/// Pad copper outline relative to the pad centre, y flipped to DSN orientation by the caller.
enum PadShape {
    Circle(f64),
    Poly(Vec<(f64, f64)>),
}

fn pad_shape(p: &Pad) -> PadShape {
    let c = (p.x, p.y);
    match p.shape.as_str() {
        "ELLIPSE" if (p.w - p.h).abs() < 1e-3 => PadShape::Circle(p.w),
        "OVAL" | "ELLIPSE" => {
            let r = p.w.min(p.h) / 2.0;
            if p.pts.len() == 2 {
                return PadShape::Poly(stadium(p.pts[0], p.pts[1], r, c));
            }
            // no end points: long axis along x (w > h) or y, turned by the pad rotation
            let half = (p.w.max(p.h) / 2.0 - r).max(0.0);
            let (dx, dy) = if p.w >= p.h { (half, 0.0) } else { (0.0, half) };
            let a = p.rot.to_radians();
            let (dx, dy) = (dx * a.cos() - dy * a.sin(), dx * a.sin() + dy * a.cos());
            PadShape::Poly(stadium((c.0 - dx, c.1 - dy), (c.0 + dx, c.1 + dy), r, c))
        }
        _ => {
            if p.pts.len() >= 3 {
                return PadShape::Poly(p.pts.iter().map(|q| (q.0 - c.0, q.1 - c.1)).collect());
            }
            let (hw, hh) = (p.w / 2.0, p.h / 2.0);
            let a = p.rot.to_radians();
            PadShape::Poly(
                [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
                    .iter()
                    .map(|&(x, y)| (x * a.cos() - y * a.sin(), x * a.sin() + y * a.cos()))
                    .collect(),
            )
        }
    }
}

pub fn load(text: &str) -> Result<EasyedaDoc, String> {
    let root: Value = serde_json::from_str(text).map_err(|e| format!("not valid JSON: {e}"))?;
    let shape_list: Vec<(usize, String)> = root
        .get("shape")
        .and_then(|s| s.as_array())
        .ok_or("not an EasyEDA PCB source file (no \"shape\" list); export it with File > Export > EasyEDA source from the PCB editor")?
        .iter()
        .enumerate()
        .filter_map(|(i, s)| s.as_str().map(|t| (i, t.to_string())))
        .collect();
    let shapes: Vec<String> = shape_list.iter().map(|(_, s)| s.clone()).collect();
    let mut parts_info: Vec<PartInfo> = vec![];
    if root.get("head").and_then(|h| h.get("docType")).and_then(|d| d.as_str()) != Some("3") {
        return Err("this is not an EasyEDA PCB document (docType 3)".into());
    }
    let notes: Vec<String> = vec![];
    let mut pads: Vec<Pad> = vec![];
    let mut holes: Vec<(f64, f64, f64)> = vec![];
    let mut outline: Vec<Vec<(f64, f64)>> = vec![];
    let mut tracks: Vec<(f64, String, String, Vec<(f64, f64)>)> = vec![];
    let mut vias: Vec<(f64, f64, f64, String)> = vec![];
    let mut strokes: Vec<Stroke> = vec![];
    let mut regions: Vec<Region> = vec![];
    let mut counts = [0usize; 6]; // vias in footprints, solid regions/rects, arcs/circles, texts, copper areas, tracks in footprints
    // copper layers of the document (top, bottom, enabled inner layers)
    let mut copper: Vec<String> = vec!["1".into(), "2".into()];
    if let Some(ls) = root.get("layers").and_then(|l| l.as_array()) {
        for l in ls.iter().filter_map(|l| l.as_str()) {
            let t: Vec<&str> = l.split('~').collect();
            if t[0].parse::<u32>().is_ok_and(|n| (21..=52).contains(&n)) && t.get(5) == Some(&"true") {
                copper.push(t[0].to_string());
            }
        }
    }
    let mut areas: HashSet<(String, String)> = HashSet::new();
    let mut ids: HashSet<String> = HashSet::new();
    let mut other_outline = 0;
    let mut existing: Vec<usize> = vec![];

    for (si, s) in shapes.iter().enumerate() {
        let pads_before = pads.len();
        let parts: Vec<&str> = s.split("#@$").collect();
        for (k, part) in parts.iter().enumerate() {
            let t: Vec<&str> = part.split('~').collect();
            let top = k == 0 && t[0] != "LIB";
            let in_lib = k > 0 && parts[0].starts_with("LIB~");
            let loose = top || in_lib;
            let tag = t[0];
            // object id is a field of every kind; collect them all to avoid collisions
            for field in &t {
                if field.starts_with("gge") {
                    ids.insert(field.to_string());
                }
            }
            match tag {
                "PAD" if t.len() > 12 => pads.push(Pad {
                    shape: t[1].to_string(),
                    x: f(t.get(2)),
                    y: f(t.get(3)),
                    w: f(t.get(4)),
                    h: f(t.get(5)),
                    layer: t[6].to_string(),
                    net: t[7].to_string(),
                    pts: pts_of(t[10]),
                    rot: f(t.get(11)),
                }),
                "HOLE" => holes.push((f(t.get(1)), f(t.get(2)), f(t.get(3)))),
                "TRACK" if loose && t.len() > 5 => {
                    if top && t[2] == "10" {
                        outline.push(pts_of(t[4]));
                    } else if !copper_layers(t[2], &copper).is_empty() {
                        for l in copper_layers(t[2], &copper) {
                            tracks.push((f(t.get(1)), l, t[3].to_string(), pts_of(t[4])));
                        }
                        if in_lib {
                            counts[5] += 1;
                        }
                        if top {
                            existing.push(shape_list[si].0);
                        }
                    }
                }
                "VIA" if loose => {
                    if top {
                        existing.push(shape_list[si].0);
                    }
                    vias.push((f(t.get(1)), f(t.get(2)), f(t.get(3)), t.get(4).unwrap_or(&"").to_string()));
                    if in_lib {
                        counts[0] += 1;
                    }
                }
                "COPPERAREA" if top && t.len() > 4 => {
                    areas.insert((t[3].to_string(), t[2].to_string()));
                    for l in copper_layers(t[2], &copper) {
                        for poly in path_polylines(t[4]) {
                            if poly.len() >= 4 {
                                regions.push(Region { layer: l.clone(), net: t[3].to_string(), pts: poly, copper: true });
                            }
                        }
                        counts[4] += 1;
                    }
                }
                "SOLIDREGION" if loose && t.len() > 4 => {
                    if top && t[1] == "10" {
                        // a solid region on the board outline layer is a cut-out of the board
                        for poly in path_polylines(t[3]) {
                            if poly.len() >= 4 {
                                for l in &copper {
                                    regions.push(Region { layer: l.clone(), net: String::new(), pts: poly.clone(), copper: false });
                                }
                                counts[1] += 1;
                            }
                        }
                    } else {
                        // solid = copper, npth = a hole in the board; a cutout only clears pours
                        let kind = t.get(4).copied().unwrap_or("solid");
                        if kind != "cutout" {
                            for l in copper_layers(t[1], &copper) {
                                for poly in path_polylines(t[3]) {
                                    if poly.len() >= 4 {
                                        regions.push(Region { layer: l.clone(), net: t[2].to_string(), pts: poly, copper: kind == "solid" });
                                    }
                                }
                                counts[1] += 1;
                            }
                        }
                    }
                }
                "RECT" if loose && t.len() > 5 => {
                    if top && t[5] == "10" {
                        other_outline += 1;
                    } else {
                        let (x, y, w, h) = (f(t.get(1)), f(t.get(2)), f(t.get(3)), f(t.get(4)));
                        for l in copper_layers(t[5], &copper) {
                            regions.push(Region {
                                layer: l,
                                net: String::new(),
                                pts: vec![(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)],
                                copper: true,
                            });
                            counts[1] += 1;
                        }
                    }
                }
                "ARC" if loose && t.len() > 4 => {
                    if top && t[2] == "10" {
                        other_outline += 1;
                    } else {
                        for l in copper_layers(t[2], &copper) {
                            for poly in path_polylines(t[4]) {
                                strokes.push(Stroke { width: f(t.get(1)), layer: l.clone(), net: t[3].to_string(), pts: poly });
                            }
                            counts[2] += 1;
                        }
                    }
                }
                "CIRCLE" if loose && t.len() > 5 => {
                    if top && t[5] == "10" {
                        other_outline += 1;
                    } else {
                        let (cx, cy, r) = (f(t.get(1)), f(t.get(2)), f(t.get(3)));
                        let n = 48;
                        // the polygon around the circle is never smaller than the circle
                        let rr = r / (std::f64::consts::PI / n as f64).cos();
                        let ring: Vec<(f64, f64)> = (0..=n)
                            .map(|k| {
                                let a = std::f64::consts::TAU * k as f64 / n as f64;
                                (cx + rr * a.cos(), cy + rr * a.sin())
                            })
                            .collect();
                        for l in copper_layers(t[5], &copper) {
                            strokes.push(Stroke { width: f(t.get(4)), layer: l, net: String::new(), pts: ring.clone() });
                            counts[2] += 1;
                        }
                    }
                }
                "TEXT" if loose && t.len() > 11 => {
                    for l in copper_layers(t[7], &copper) {
                        for poly in path_polylines(t[11]) {
                            strokes.push(Stroke { width: f(t.get(4)), layer: l.clone(), net: String::new(), pts: poly });
                        }
                        counts[3] += 1;
                    }
                }
                _ => {}
            }
        }
        if s.starts_with("LIB~") {
            let header: Vec<&str> = parts[0].split('~').collect();
            let package = header.get(3).and_then(|a| a.split('`').nth(1)).unwrap_or("").to_string();
            let pad_layers: Vec<&str> = parts
                .iter()
                .skip(1)
                .filter(|p| p.starts_with("PAD~"))
                .map(|p| p.split('~').nth(6).unwrap_or(""))
                .collect();
            let has_hole = parts.iter().skip(1).any(|p| p.starts_with("HOLE~") || p.starts_with("VIA~"));
            let designator = parts
                .iter()
                .skip(1)
                .find(|p| p.starts_with("TEXT~P~"))
                .and_then(|p| p.split('~').nth(10))
                .unwrap_or("")
                .to_string();
            parts_info.push(PartInfo {
                shape: shape_list[si].0,
                package,
                designator,
                pads: (pads_before + 1..=pads.len()).collect(),
                smd: !pad_layers.is_empty() && pad_layers.iter().all(|l| *l == "1" || *l == "2") && !has_hole,
                movable: shift_lib(s, 0.0, 0.0).is_some(),
            });
        }
    }
    // ---- design rules
    let drc = root.get("DRCRULE").and_then(|d| d.get("Default"));
    let getf = |k: &str, d: f64| drc.and_then(|r| r.get(k)).and_then(|v| v.as_f64()).unwrap_or(d);
    let (track_w, clear, via_d) = (getf("trackWidth", 1.0), getf("clearance", 0.6) + CLEARANCE_MARGIN, getf("viaHoleDiameter", 2.4));
    let mut special: Vec<(String, f64, f64)> = vec![];
    if let Some(sn) = root.get("routerRule").and_then(|r| r.get("specialNets")).and_then(|s| s.as_array()) {
        for e in sn {
            let name = e.get("net").and_then(|n| n.as_str()).unwrap_or("");
            let w = e.get("width").and_then(|v| v.as_str()).and_then(length_px).unwrap_or(track_w);
            let c = e.get("clearance").and_then(|v| v.as_str()).and_then(length_px).map_or(clear, |c| c + CLEARANCE_MARGIN);
            special.push((name.to_string(), w, c));
        }
    }
    finish(
        Collected {
            notes, pads, holes, outline, tracks, vias, strokes, regions, counts, copper, areas, ids, other_outline,
            track_w, clear, via_d, special, existing,
        },
        root,
        parts_info,
        None,
    )
}

/// Everything a source format has to tell the DSN writer, in EasyEDA Standard conventions: units
/// of 10 mil, y pointing down.
pub(crate) struct Collected {
    pub notes: Vec<String>,
    pub pads: Vec<Pad>,
    /// mounting holes: x, y, radius
    pub holes: Vec<(f64, f64, f64)>,
    /// board outline pieces (chained into one polygon)
    pub outline: Vec<Vec<(f64, f64)>>,
    /// width, layer, net, points
    pub tracks: Vec<(f64, String, String, Vec<(f64, f64)>)>,
    /// x, y, diameter, net
    pub vias: Vec<(f64, f64, f64, String)>,
    pub strokes: Vec<Stroke>,
    pub regions: Vec<Region>,
    /// vias in footprints, solid regions/rects, arcs/circles, texts, copper areas, tracks in footprints
    pub counts: [usize; 6],
    /// copper layer names: top, bottom, then the inner layers
    pub copper: Vec<String>,
    /// (net, layer) of every copper area already in the document
    pub areas: HashSet<(String, String)>,
    pub ids: HashSet<String>,
    pub other_outline: usize,
    pub track_w: f64,
    /// clearance including the safety margin
    pub clear: f64,
    pub via_d: f64,
    /// net class overrides: net, width, clearance (margin included)
    pub special: Vec<(String, f64, f64)>,
    /// see `EasyedaDoc::existing`
    pub existing: Vec<usize>,
}

pub(crate) fn finish(
    c: Collected,
    root: Value,
    parts_info: Vec<PartInfo>,
    pro: Option<crate::easyeda_pro::ProDoc>,
) -> Result<EasyedaDoc, String> {
    let Collected {
        mut notes, pads, holes, outline, tracks, vias, strokes, regions, counts, copper, areas, ids, other_outline,
        track_w, clear, via_d, special, existing,
    } = c;

    if other_outline > 0 {
        notes.push(format!("{other_outline} arc/circle/region object(s) on the board-outline layer were ignored"));
    }
    if pads.is_empty() {
        return Err("the document has no pads".into());
    }

    // ---- board outline: chain the layer-10 polylines; fall back to the bounding box
    let mut boundary: Vec<(f64, f64)> = vec![];
    let mut segs: Vec<Vec<(f64, f64)>> = outline.into_iter().filter(|p| p.len() >= 2).collect();
    if !segs.is_empty() {
        let mut chain = segs.remove(0);
        let near = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < 1e-2 && (a.1 - b.1).abs() < 1e-2;
        let mut progress = true;
        while progress && !segs.is_empty() {
            progress = false;
            for i in 0..segs.len() {
                let (head, tail) = (chain[0], *chain.last().unwrap());
                let (s0, s1) = (segs[i][0], *segs[i].last().unwrap());
                let joined = if near(tail, s0) {
                    chain.extend_from_slice(&segs[i][1..]);
                    true
                } else if near(tail, s1) {
                    let mut r = segs[i].clone();
                    r.reverse();
                    chain.extend_from_slice(&r[1..]);
                    true
                } else if near(head, s1) {
                    let mut p = segs[i].clone();
                    p.extend_from_slice(&chain[1..]);
                    chain = p;
                    true
                } else {
                    false
                };
                if joined {
                    segs.remove(i);
                    progress = true;
                    break;
                }
            }
        }
        let closed = chain.len() >= 4 && near(chain[0], *chain.last().unwrap());
        if closed && segs.is_empty() {
            chain.pop();
            boundary = chain;
        } else {
            notes.push("the board outline is not one closed polyline; the bounding box of the outline objects is used".into());
            let all: Vec<(f64, f64)> = chain.iter().chain(segs.iter().flatten()).copied().collect();
            let (x0, x1) = all.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.0), a.1.max(p.0)));
            let (y0, y1) = all.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.1), a.1.max(p.1)));
            boundary = vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
        }
    } else {
        notes.push("no board outline (layer BoardOutLine) found; the pad bounding box plus 10 units is used".into());
        let (x0, x1) = pads.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.x), a.1.max(p.x)));
        let (y0, y1) = pads.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.y), a.1.max(p.y)));
        boundary = vec![(x0 - 10.0, y0 - 10.0), (x1 + 10.0, y0 - 10.0), (x1 + 10.0, y1 + 10.0), (x0 - 10.0, y1 + 10.0)];
    }
    let ox = boundary.iter().fold(f64::MAX, |a, p| a.min(p.0));
    let oy = boundary.iter().fold(f64::MAX, |a, p| a.min(p.1));
    let dx = |x: f64| x - ox;
    let dy = |y: f64| -(y - oy);

    // ---- copper layers
    let layers: Vec<String> = copper.clone();

    // ---- DSN text
    let mut o = String::new();
    let _ = writeln!(o, "(PCB ''\n  (resolution mil 1000)\n  (structure");
    let _ = write!(o, "    (boundary(path signal 0");
    for p in boundary.iter().chain(std::iter::once(&boundary[0])) {
        let _ = write!(o, " {} {}", num(dx(p.0)), num(dy(p.1)));
    }
    let _ = writeln!(o, "))");
    let _ = writeln!(o, "    (via via0)\n    (grid via 0.25)\n    (grid wire 0.25)\n    (grid place 0.25)");
    let _ = writeln!(o, "    (rule(clear {}))\n    (rule(width {}))", num(clear), num(track_w));
    for l in &layers {
        let _ = writeln!(o, "    (layer {l}\n      (type signal)\n    )");
    }
    for (x, y, r) in &holes {
        // HOLE objects store the radius
        let _ = writeln!(o, "    (keepout \"\" (circle pcb {} {} {}))", num(2.0 * r), num(dx(*x)), num(dy(*y)));
    }
    // Copper areas and solid regions: a region of a net of the board is a plane of that net, any
    // other copper (and board slots) is simply kept clear.
    let known: HashSet<&str> = pads.iter().filter(|p| !p.net.is_empty()).map(|p| p.net.as_str()).collect();
    for r in &regions {
        let mut pts = r.pts.clone();
        if pts.len() >= 2 && pts.first() == pts.last() {
            pts.pop();
        }
        if pts.len() < 3 {
            continue;
        }
        let mut body = String::new();
        for q in &pts {
            let _ = write!(body, " {} {}", num(dx(q.0)), num(dy(q.1)));
        }
        if r.copper && known.contains(r.net.as_str()) {
            let _ = writeln!(o, "    (plane {} (polygon {} 0{}))", quote(&r.net), r.layer, body);
        } else {
            let _ = writeln!(o, "    (keepout \"\" (polygon {} 0{}))", r.layer, body);
        }
    }
    let _ = writeln!(o, "  )\n  (placement\n    (component u1\n      (place u1 0 0 front 0\n      )\n    )\n  )");
    let _ = writeln!(o, "  (library\n    (image u1");

    // padstacks: one per distinct (shape, layers)
    let mut stacks: Vec<String> = vec![];
    let mut stack_text: Vec<String> = vec![];
    let mut pin_net: Vec<(String, String)> = vec![];
    let mut pin_lines = String::new();
    for (i, p) in pads.iter().enumerate() {
        let on: Vec<&String> = match p.layer.as_str() {
            "1" => vec![&layers[0]],
            "2" => vec![&layers[1]],
            _ => layers.iter().collect(),
        };
        let mut body = String::new();
        match pad_shape(p) {
            PadShape::Circle(d) => {
                for l in &on {
                    let _ = writeln!(body, "      (shape(circle {l} {} 0 0))", num(d));
                }
            }
            PadShape::Poly(pts) => {
                for l in &on {
                    let _ = write!(body, "      (shape(polygon {l} 0.01");
                    for q in &pts {
                        let _ = write!(body, " {} {}", num(q.0), num(-q.1));
                    }
                    let _ = writeln!(body, "))");
                }
            }
        }
        let k = match stacks.iter().position(|s| *s == body) {
            Some(k) => k,
            None => {
                stacks.push(body.clone());
                stack_text.push(body);
                stacks.len() - 1
            }
        };
        let _ = writeln!(pin_lines, "      (pin ps{k} {} {} {})", i + 1, num(dx(p.x)), num(dy(p.y)));
        if !p.net.is_empty() {
            pin_net.push((p.net.clone(), format!("u1-{}", i + 1)));
        }
    }
    o.push_str(&pin_lines);
    let _ = writeln!(o, "    )");
    let _ = writeln!(o, "    (padstack via0");
    for l in &layers {
        let _ = writeln!(o, "      (shape(circle {l} {}))", num(via_d));
    }
    let _ = writeln!(o, "    )");
    // vias that are already on the board keep their own diameter
    let via_name = |d: f64| -> String {
        if d <= 0.0 || (d - via_d).abs() < 1e-6 {
            "via0".to_string()
        } else {
            format!("via_d{}", num(d).replace('.', "_"))
        }
    };
    let mut via_sizes: Vec<f64> = vec![];
    for v in &vias {
        if via_name(v.2) != "via0" && !via_sizes.iter().any(|s| (s - v.2).abs() < 1e-6) {
            via_sizes.push(v.2);
        }
    }
    for d in &via_sizes {
        let _ = writeln!(o, "    (padstack {}", via_name(*d));
        for l in &layers {
            let _ = writeln!(o, "      (shape(circle {l} {}))", num(*d));
        }
        let _ = writeln!(o, "    )");
    }
    for (k, b) in stack_text.iter().enumerate() {
        let _ = writeln!(o, "    (padstack ps{k}\n{b}    )");
    }
    let _ = writeln!(o, "  )");

    // ---- network: nets in first-appearance order, then per-net rules
    let mut order: Vec<String> = vec![];
    for (n, _) in &pin_net {
        if !order.contains(n) {
            order.push(n.clone());
        }
    }
    let _ = writeln!(o, "  (network");
    for n in &order {
        let pins: Vec<&str> = pin_net.iter().filter(|(m, _)| m == n).map(|(_, p)| p.as_str()).collect();
        let _ = writeln!(o, "    (net {}\n      (pins {})\n    )", quote(n), pins.join(" "));
    }
    for (name, w, c) in &special {
        if name.is_empty() || !order.iter().any(|n| n == name) {
            continue;
        }
        let _ = writeln!(
            o,
            "    (class {} {}
      (circuit
        (use_via via0)
      )
      (rule
        (width {})
        (clearance {})
      )
    )",
            quote(name),
            quote(name),
            num(*w),
            num(*c)
        );
    }
    let _ = writeln!(o, "  )");

    // ---- existing wiring stays as it is
    let _ = writeln!(o, "  (wiring");
    for (w, l, net, pts) in &tracks {
        if pts.len() < 2 {
            continue;
        }
        let _ = write!(o, "    (wire (path {l} {}", num(*w));
        for q in pts {
            let _ = write!(o, " {} {}", num(dx(q.0)), num(dy(q.1)));
        }
        let _ = writeln!(o, "){}(type protect))", if net.is_empty() { String::new() } else { format!("(net {})", quote(net)) });
    }
    for s in &strokes {
        if s.pts.len() < 2 {
            continue;
        }
        let _ = write!(o, "    (wire (path {} {}", s.layer, num(s.width.max(0.01)));
        for q in &s.pts {
            let _ = write!(o, " {} {}", num(dx(q.0)), num(dy(q.1)));
        }
        let _ = writeln!(o, "){}(type protect))", if s.net.is_empty() { String::new() } else { format!("(net {})", quote(&s.net)) });
    }
    for (x, y, d, net) in &vias {
        let _ = writeln!(
            o,
            "    (via {} {} {}{})",
            via_name(*d),
            num(dx(*x)),
            num(dy(*y)),
            if net.is_empty() { String::new() } else { format!(" (net {})", quote(net)) }
        );
    }
    let _ = writeln!(o, "  )\n)");

    if counts.iter().any(|&c| c > 0) || !strokes.is_empty() || !regions.is_empty() {
        notes.push(format!(
            "copper objects loaded as obstacles: {} via(s) inside footprints, {} solid region(s)/rectangle(s), {} arc/circle(s), {} text(s) on copper, {} copper area(s), {} footprint track(s)",
            counts[0], counts[1], counts[2], counts[3], counts[4], counts[5]
        ));
    }
    if !tracks.is_empty() || !vias.is_empty() {
        notes.push(format!(
            "{} existing track(s) and {} via(s) are kept as they are and routed around",
            tracks.len(),
            vias.len()
        ));
    }
    if !areas.is_empty() {
        notes.push(format!("{} existing copper area(s) are kept; nets and layers that already have one get no new fill", areas.len()));
    }
    Ok(EasyedaDoc { root, parts: parts_info, dsn: o, ox, oy, notes, areas, ids, pro, existing })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn footprints_move_as_a_whole() {
        let lib = "LIB~100~200~package`C0402`x`y~180~~gge1~1~u~1~0~~yes~~#@$TEXT~N~100~190~0.6~180~0~3~~4.5~C1~M 100 190 L 101 191#@$SOLIDREGION~100~~M 98 198 L 98 202 A 0.5 0.5 0 0 1 99 203 Z~solid~gge2~~~~0#@$TRACK~0.6~3~~98 198 102 198~gge3~0#@$CIRCLE~99~201~0.1~0.2~101~gge4~0~~#@$PAD~RECT~98~200~3~2~1~A~1~0~97 199 99 199 99 201~180~gge5~0~~Y~0~0~0.2~98,200#@$SVGNODE~{\"attrs\":{\"c_origin\":\"100,200\"},\"childNodes\":[{\"attrs\":{\"points\":\"98 198 102 202\"}}]}";
        let moved = shift_lib(lib, 5.0, -3.0).unwrap();
        assert!(moved.starts_with("LIB~105~197~"));
        assert!(moved.contains("PAD~RECT~103~197~3~2~1~A~1~0~102 196 104 196 104 198~180~gge5~0~~Y~0~0~0.2~103,197"));
        assert!(moved.contains("M 103 195 A 0.5 0.5 0 0 1 104 200 Z") || moved.contains("A 0.5 0.5 0 0 1 104 200"));
        assert!(moved.contains("TRACK~0.6~3~~103 195 107 195"));
        assert!(moved.contains("CIRCLE~104~198~"));
        assert!(moved.contains("\"c_origin\":\"105,197\"") && moved.contains("\"points\":\"103 195 107 199\""));
        // and back again
        assert_eq!(shift_lib(&moved, -5.0, 3.0).unwrap(), lib);
        // unknown objects are refused instead of half moved
        assert!(shift_lib("LIB~1~1~a~0#@$WEIRD~1", 1.0, 1.0).is_none());
    }

    #[test]
    fn every_footprint_of_the_adau_board_can_be_moved() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let Ok(j) = std::fs::read_to_string(dir.join("PCB_PCB_adau1452-dsp-v2-copy_2026-11-16.json")) else {
            return;
        };
        let doc = load(&j).unwrap();
        assert_eq!(doc.parts.len(), 85);
        assert!(doc.parts.iter().all(|p| p.movable), "{:?}", doc.parts.iter().find(|p| !p.movable));
        assert_eq!(doc.parts.iter().map(|p| p.pads.len()).sum::<usize>(), 323);
        assert!(doc.parts.iter().filter(|p| p.smd).count() > 60);
    }

    #[test]
    fn paths_with_glued_and_comma_forms_and_arcs_are_read() {
        let p = path_polylines("M 4091 3564 L 4097.5 3570.5 L4096.5,3563 L4093.5,3563 Z");
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].len(), 5);
        assert_eq!(p[0][2], (4096.5, 3563.0));
        assert_eq!(p[0][0], *p[0].last().unwrap());
        // a half circle of radius 1.75: the points stay on the circle
        let a = path_polylines("M4110,3623 A1.75,1.75 0 0 0 4113.5,3623");
        assert_eq!(a.len(), 1);
        assert!(a[0].len() > 5);
        for q in &a[0] {
            let d = ((q.0 - 4111.75).powi(2) + (q.1 - 3623.0).powi(2)).sqrt();
            assert!((d - 1.75).abs() < 1e-6, "{q:?} {d}");
        }
    }

    #[test]
    fn every_kind_of_copper_object_becomes_an_obstacle() {
        let lib = "LIB~100~100~package`QFN`x`y~0~~gge20~1~u~1~0~~yes~~            #@$PAD~RECT~100~100~6~6~1~GND~1~0~~0~gge21~0~~Y~0~0~0.2~100,100            #@$VIA~104~100~1.5748~GND~0.59~gge22~0            #@$VIA~96~100~1.5748~GND~0.59~gge23~0            #@$TRACK~1~1~~90 90 90 95~gge24~0            #@$TRACK~1~3~~90 80 95 80~gge25~0";
        let json = format!(
            r#"{{"head":{{"docType":"3"}},"layers":[],
          "DRCRULE":{{"Default":{{"trackWidth":1,"clearance":0.6,"viaHoleDiameter":2.4}}}},
          "shape":["TRACK~1~10~~0 0 200 0 200 120 0 120 0 0~gge1~0",
            "PAD~ELLIPSE~60~60~5~5~11~X~1~1.2~~0~gge2~0~~Y~0~0~0.2~60,60",
            "PAD~ELLIPSE~140~60~5~5~11~X~2~1.2~~0~gge3~0~~Y~0~0~0.2~140,60",
            "VIA~20~20~2.441~~0.61~gge4~0",
            "SOLIDREGION~1~~M 10 10 L 20 10 L20,15 L 10 15 Z~solid~gge5~~~~0",
            "SOLIDREGION~1~GND~M 30 10 L 40 10 L 40 15 L 30 15 Z~solid~gge6~~~~0",
            "SOLIDREGION~1~~M 50 10 L 60 10 L 60 15 Z~cutout~gge7~~~~0",
            "RECT~70~10~5~5~1~gge8~0~0~~~~",
            "CIRCLE~90~20~4~1~1~gge9~0~~",
            "ARC~1~2~~M 100 20 A 5 5 0 0 1 110 20~~gge10~0",
            "TEXT~N~120~20~0.6~0~0~1~~4.5~AB~M 120 20 L 125 25~none~gge11~~0~",
            "COPPERAREA~1~1~GND~M 150 80 L 190 80 L 190 110 L 150 110 Z~1~solid~gge12~spoke~none~[[]]~0~gge12~2~1~2~1~yes~0",
            "{lib}"]}}"#
        );
        let doc = load(&json).unwrap();
        let b = crate::parse_dsn(&doc.dsn).unwrap();
        // footprint vias + the loose one, each with its own diameter
        assert_eq!(b.vias.len(), 3, "{:?}", b.vias);
        assert!(b.vias.iter().any(|v| v.net.is_empty() && v.name != "via0"));
        assert!(b.vias.iter().filter(|v| v.net == "GND").count() == 2);
        // copper tracks of a footprint count, silkscreen tracks do not
        assert_eq!(b.wires.len(), 1 + 1 + 1 + 1 + 0, "{:?}", b.wires.iter().map(|w| (&w.layer, w.pts.len())).collect::<Vec<_>>());
        // netless solid region, rectangle: kept clear; GND solid region and the copper area: planes of GND
        assert_eq!(b.keepouts.len(), 2);
        assert_eq!(b.planes.len(), 2);
        assert!(b.planes.iter().all(|p| p.net == "GND"));
        // the cutout region is not copper and is skipped
        assert!(doc.notes.iter().any(|n| n.contains("2 via(s) inside footprints")), "{:?}", doc.notes);
    }

    #[test]
    fn units_parse() {
        assert_eq!(length_px("1px"), Some(1.0));
        assert!((length_px("0.254mm").unwrap() - 1.0).abs() < 1e-9);
        assert!((length_px("10mil").unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn small_board_round_trip() {
        let json = r#"{"head":{"docType":"3"},"layers":["21~Inner1~#999966~true~false~true~0~Signal"],
          "DRCRULE":{"Default":{"trackWidth":1,"clearance":0.6,"viaHoleDiameter":2.4}},
          "shape":["TRACK~1~10~~100 100 200 100 200 150 100 150 100 100~gge1~0",
                   "PAD~ELLIPSE~110~110~4~4~11~A~1~1~~0~gge2~0~~Y~0~0~0.2~110,110",
                   "PAD~RECT~190~140~3~2~1~A~2~0~188.5 139 191.5 139 191.5 141 188.5 141~0~gge3~0~~Y~0~0~0.2~190,140"]}"#;
        let d = load(json).unwrap();
        assert_eq!((d.ox, d.oy), (100.0, 100.0));
        let b = crate::parse_dsn(&d.dsn).unwrap();
        assert_eq!(b.layers.len(), 3);
        assert_eq!(b.nets.len(), 1);
        assert_eq!(b.nets[0].pins.len(), 2);
        let pins = b.abs_pins();
        let p = &pins["u1-1"];
        assert!((p.x - 10.0).abs() < 1e-9 && (p.y + 10.0).abs() < 1e-9);
        assert_eq!(d.to_json_xy((10.0, -10.0)), (110.0, 110.0));
    }
}

#[cfg(test)]
mod board_tests {
    use super::*;

    /// The ADAU1452 board: the JSON import must describe the same board as EasyEDA's own DSN.
    #[test]
    fn matches_easyeda_dsn_export() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let (Ok(j), Ok(d)) = (
            std::fs::read_to_string(dir.join("adau-unrouted.json")),
            std::fs::read_to_string(dir.join("Autorouter_PCB_adau1452-dsp-v2_2026-11-15.dsn")),
        ) else {
            return;
        };
        let doc = load(&j).unwrap();
        let a = crate::parse_dsn(&doc.dsn).unwrap();
        let b = crate::parse_dsn(&d).unwrap();
        assert_eq!((doc.ox, doc.oy), (4020.0, 3562.5));
        assert_eq!(a.layers.iter().map(|l| &l.name).collect::<Vec<_>>(), b.layers.iter().map(|l| &l.name).collect::<Vec<_>>());
        let sorted = |bd: &crate::Board| {
            let mut v: Vec<(String, usize)> = bd.nets.iter().map(|n| (n.name.clone(), n.pins.len())).collect();
            v.sort();
            v
        };
        assert_eq!(sorted(&a), sorted(&b));
        // pad centres and net membership
        let centres = |bd: &crate::Board| {
            let nets: std::collections::HashMap<String, String> = bd
                .nets
                .iter()
                .flat_map(|n| n.pins.iter().map(move |p| (format!("{}-{}", p.comp, p.pin), n.name.clone())))
                .collect();
            let mut v: Vec<(i64, i64, String, usize)> = bd
                .abs_pins()
                .iter()
                .map(|(k, p)| ((p.x * 100.0).round() as i64, (p.y * 100.0).round() as i64, nets.get(k).cloned().unwrap_or_default(), p.shapes.len()))
                .collect();
            v.sort();
            v
        };
        // EasyEDA exports every mounting hole as a netless pin; here they are keepouts
        let (ca, cb) = (centres(&a), centres(&b));
        let (na, nb) = (ca.iter().filter(|c| c.2.is_empty()).count(), cb.iter().filter(|c| c.2.is_empty()).count());
        assert_eq!(na + a.keepouts.len(), nb, "netless pads + holes");
        assert!(a.keepouts.len() >= 1);
        let (ca, cb): (Vec<_>, Vec<_>) = (ca.into_iter().filter(|c| !c.2.is_empty()).collect(), cb.into_iter().filter(|c| !c.2.is_empty()).collect());
        assert_eq!(ca.len(), cb.len());
        let diff: Vec<_> = ca.iter().zip(&cb).filter(|(x, y)| x != y).take(5).collect();
        assert!(diff.is_empty(), "{diff:?}");
        // pad copper extents agree (bounding boxes per pad, all layers)
        let ext = |bd: &crate::Board| {
            let mut v: Vec<(i64, i64, i64, i64)> = vec![];
            let netted: std::collections::HashSet<String> = bd.nets.iter().flat_map(|n| n.pins.iter().map(|p| format!("{}-{}", p.comp, p.pin))).collect();
            for (k, p) in bd.abs_pins().iter() {
                if !netted.contains(k) {
                    continue;
                }
                for (_, s) in &p.shapes {
                    let (x0, y0, x1, y1) = match s {
                        crate::AbsShape::Circle { cx, cy, r } => (cx - r, cy - r, cx + r, cy + r),
                        crate::AbsShape::Poly(q) => q.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |a, p| (a.0.min(p.0), a.1.min(p.1), a.2.max(p.0), a.3.max(p.1))),
                    };
                    v.push(((x0 * 20.0).round() as i64, (y0 * 20.0).round() as i64, (x1 * 20.0).round() as i64, (y1 * 20.0).round() as i64));
                }
            }
            v.sort();
            v
        };
        let (ea, eb) = (ext(&a), ext(&b));
        assert_eq!(ea.len(), eb.len(), "pad shape count");
        let bad: Vec<_> = ea.iter().zip(&eb).filter(|(x, y)| (x.0 - y.0).abs() > 2 || (x.1 - y.1).abs() > 2 || (x.2 - y.2).abs() > 2 || (x.3 - y.3).abs() > 2).take(5).collect();
        assert!(bad.is_empty(), "{bad:?}");
        let bb = |bd: &crate::Board| bd.boundary.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.0), a.1.max(p.0)));
        assert!((bb(&a).1 - bb(&b).1).abs() < 0.5);
        eprintln!("notes: {:?}; keepouts {} vs {}", doc.notes, a.keepouts.len(), b.keepouts.len());
    }
}
