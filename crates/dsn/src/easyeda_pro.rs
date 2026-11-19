//! EasyEDA Pro projects (`.epro2`) as router input and output.
//!
//! An `.epro2` is a ZIP archive. Its `.epru` entry holds every document of the project (symbols,
//! footprints, schematic pages, the PCB) as lines `{head}||{body}|`. The PCB document only lists
//! its components by position, the pads come from the footprint documents they refer to.
//! Coordinates are mil with y pointing up.
//!
//! The router works in EasyEDA Standard conventions (10 mil units, y pointing down), so everything
//! is converted on the way in, and the result is converted back when new tracks, vias and copper
//! pours are appended to the PCB document. Everything that is already in the project, and all
//! other documents and files of the archive, are written back unchanged.

use crate::easyeda::{finish, stadium, Collected, EasyedaDoc, Pad, PartInfo, Region, Stroke, CLEARANCE_MARGIN};
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::io::{Cursor, Read, Write};

/// EasyEDA Standard units per mil.
const PX: f64 = 0.1;
/// Record types that describe copper, holes or regions and are read (anything else in this list
/// is reported, everything not in it is bookkeeping).
const GEOMETRY: [&str; 12] = ["LINE", "ARC", "CIRCLE", "VIA", "FILL", "POLY", "POUR", "RECT", "ELLIPSE", "TEXT", "STRING", "REGION"];
/// Largest distance between an arc and its polyline, in mil.
const ARC_TOL: f64 = 0.05;

type Pt = (f64, f64);

/// Pro (mil, y up) to the router's input space (10 mil, y down).
fn st(p: Pt) -> Pt {
    (p.0 * PX, -p.1 * PX)
}

/// The Pro document kept for writing the result back.
#[derive(Debug, Clone)]
pub struct ProDoc {
    /// the project file as it was read (a ZIP archive)
    bytes: Vec<u8>,
    /// the archive entry that holds the PCB: the `.epru` of an `.epro2`, `PCB/<uuid>.epcb` of an `.epro`
    epru_name: String,
    /// every line of that entry
    lines: Vec<String>,
    /// new objects are inserted at this line index (end of the PCB document)
    insert_at: usize,
    max_ticket: u64,
    /// the older `.epro` layout (positional records, ids `e<number>`, no tickets)
    legacy: bool,
    next_eid: u64,
    ids: HashSet<String>,
    pour_n: usize,
    pour_order: i64,
    /// per part, in the order of `EasyedaDoc::parts`: its COMPONENT line and the lines of its labels
    parts: Vec<PartLines>,
    /// hole diameter of the default via rule in mil
    pub via_hole_mil: f64,
    pub title: String,
}

#[derive(Debug, Clone)]
struct PartLines {
    comp: usize,
    attrs: Vec<usize>,
}

/// An object the router adds. Coordinates are the router's document space (10 mil units, y down,
/// as given by `EasyedaDoc::to_json_xy`).
#[derive(Debug, Clone)]
pub enum NewObject {
    Track { net: String, layer: String, width: f64, pts: Vec<Pt> },
    Via { net: String, at: Pt, dia: f64, hole: f64 },
    Pour { net: String, layer: String, ring: Vec<Pt> },
}

// ------------------------------------------------------------------ text format

/// `{head}||{body}|` -> (head, body). The head always ends in `}`.
fn split_line(l: &str) -> Option<(&str, &str)> {
    let l = l.trim_end_matches(['\r', '\n']);
    let i = l.find("}||")?;
    let rest = &l[i + 3..];
    Some((&l[..=i], rest.strip_suffix('|').unwrap_or(rest)))
}

struct Rec {
    ty: String,
    id: String,
    line: usize,
    body: Value,
}

struct DocSpan {
    start: usize,
    end: usize,
    doc_type: String,
    uuid: String,
}

fn parse_records(lines: &[String], d: &DocSpan) -> Vec<Rec> {
    let mut v = vec![];
    for i in d.start + 1..d.end {
        let Some((h, b)) = split_line(&lines[i]) else { continue };
        let Ok(head) = serde_json::from_str::<Value>(h) else { continue };
        let ty = head.get("type").and_then(|t| t.as_str()).unwrap_or("").to_string();
        let id = head.get("id").and_then(|t| t.as_str()).unwrap_or("").to_string();
        let body = serde_json::from_str::<Value>(b).unwrap_or(Value::Null);
        v.push(Rec { ty, id, line: i, body });
    }
    v
}

fn f(b: &Value, k: &str) -> f64 {
    b.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0)
}

fn s<'a>(b: &'a Value, k: &str) -> &'a str {
    b.get(k).and_then(|v| v.as_str()).unwrap_or("")
}

fn int(b: &Value, k: &str) -> i64 {
    b.get(k).and_then(|v| v.as_i64().or_else(|| v.as_f64().map(|x| x as i64))).unwrap_or(0)
}

fn present<'a>(b: &'a Value, k: &str) -> Option<&'a Value> {
    b.get(k).filter(|v| !v.is_null())
}

// ------------------------------------------------------------------ geometry

/// Placement of a footprint: rotation (counter-clockwise, y up) about its origin, after mirroring
/// for the bottom side.
#[derive(Clone, Copy)]
struct Xf {
    x: f64,
    y: f64,
    angle: f64,
    bottom: bool,
}

impl Xf {
    const ID: Xf = Xf { x: 0.0, y: 0.0, angle: 0.0, bottom: false };

    fn pt(&self, p: Pt) -> Pt {
        let px = if self.bottom { -p.0 } else { p.0 };
        let a = self.angle.to_radians();
        (self.x + px * a.cos() - p.1 * a.sin(), self.y + px * a.sin() + p.1 * a.cos())
    }

    /// rotation of an object of the footprint
    fn rot(&self, a: f64) -> f64 {
        (if self.bottom { -a } else { a }) + self.angle
    }

    fn layer(&self, id: i64) -> i64 {
        match (self.bottom, id) {
            (true, 1) => 2,
            (true, 2) => 1,
            _ => id,
        }
    }
}

fn rotate(p: Pt, a_deg: f64) -> Pt {
    let a = a_deg.to_radians();
    (p.0 * a.cos() - p.1 * a.sin(), p.0 * a.sin() + p.1 * a.cos())
}

/// Points of the arc from `a` to `b` that turns by `sweep_deg` (positive: counter-clockwise, y
/// up), without `a`, with `b`.
fn arc_points(a: Pt, b: Pt, sweep_deg: f64) -> Vec<Pt> {
    let sweep = sweep_deg.to_radians();
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let l = (dx * dx + dy * dy).sqrt();
    if l < 1e-9 || sweep.abs() < 1e-6 || sweep.abs() > std::f64::consts::TAU - 1e-6 {
        return vec![b];
    }
    let r = l / (2.0 * (sweep.abs() / 2.0).sin());
    let nl = (-dy / l, dx / l); // left of the direction a -> b
    let k = r * (sweep / 2.0).cos() * sweep.signum();
    let c = ((a.0 + b.0) / 2.0 + nl.0 * k, (a.1 + b.1) / 2.0 + nl.1 * k);
    let a0 = (a.1 - c.1).atan2(a.0 - c.0);
    let step = if r <= ARC_TOL { std::f64::consts::PI } else { 2.0 * (1.0 - ARC_TOL / r).acos() };
    let n = ((sweep.abs() / step).ceil() as usize).clamp(1, 720);
    let mut v: Vec<Pt> = (1..n)
        .map(|i| {
            let t = a0 + sweep * i as f64 / n as f64;
            (c.0 + r * t.cos(), c.1 + r * t.sin())
        })
        .collect();
    v.push(b);
    v
}

/// Rectangle with its top-left corner at (x, y), corner radius and rotation about its centre.
fn rect_poly(x: f64, y: f64, w: f64, h: f64, rot: f64, radius: f64) -> Vec<Pt> {
    let r = radius.clamp(0.0, w.abs().min(h.abs()) / 2.0);
    let mut v: Vec<Pt> = vec![];
    if r < 1e-6 {
        v.extend([(x, y), (x, y - h), (x + w, y - h), (x + w, y)]);
    } else {
        // counter-clockwise from the top-left corner
        for (c, a0) in [((x + r, y - r), 90.0), ((x + r, y - h + r), 180.0), ((x + w - r, y - h + r), 270.0), ((x + w - r, y - r), 0.0)] {
            for k in 0..=6 {
                let t = (a0 + 90.0 * k as f64 / 6.0f64).to_radians();
                v.push((c.0 + r * t.cos(), c.1 + r * t.sin()));
            }
        }
    }
    let c = (x + w / 2.0, y - h / 2.0);
    if rot.abs() > 1e-9 {
        v = v.into_iter().map(|p| { let q = rotate((p.0 - c.0, p.1 - c.1), rot); (q.0 + c.0, q.1 + c.1) }).collect();
    }
    v
}

fn circle_poly(c: Pt, r: f64) -> Vec<Pt> {
    let n = 48;
    // outside the circle, never smaller than it
    let rr = r / (std::f64::consts::PI / n as f64).cos();
    (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            (c.0 + rr * a.cos(), c.1 + rr * a.sin())
        })
        .collect()
}

fn close(mut v: Vec<Pt>) -> Vec<Pt> {
    if v.len() >= 2 && v.first() != v.last() {
        v.push(v[0]);
    }
    v
}

/// One contour of a Pro path: `["R", x, y, w, h, rot, radius]`, `["CIRCLE", cx, cy, r]` or
/// `[x, y, "L", x, y, ..., "ARC", sweep, x, y, ...]`.
fn contour(t: &[Value], warn: &mut Vec<String>) -> Option<Vec<Pt>> {
    let num = |i: usize| t.get(i).and_then(|v| v.as_f64());
    match t.first()? {
        Value::String(k) if k == "R" => {
            Some(rect_poly(num(1)?, num(2)?, num(3)?, num(4)?, num(5).unwrap_or(0.0), num(6).unwrap_or(0.0)))
        }
        Value::String(k) if k == "CIRCLE" => Some(circle_poly((num(1)?, num(2)?), num(3)?)),
        _ => {
            let mut pts: Vec<Pt> = vec![(num(0)?, num(1)?)];
            let (mut i, mut cmd) = (2, "L");
            while i < t.len() {
                match &t[i] {
                    Value::String(c) => {
                        cmd = c.as_str();
                        i += 1;
                    }
                    _ => match cmd {
                        "L" => {
                            pts.push((num(i)?, num(i + 1)?));
                            i += 2;
                        }
                        "ARC" | "CARC" => {
                            let (sweep, e) = (num(i)?, (num(i + 1)?, num(i + 2)?));
                            if cmd == "CARC" {
                                warn.push("a centre-point arc (CARC) in a path was replaced by its chord".into());
                                pts.push(e);
                            } else {
                                let a = *pts.last()?;
                                pts.extend(arc_points(a, e, sweep));
                            }
                            i += 3;
                        }
                        "C" => {
                            let a = *pts.last()?;
                            let (p1, p2, e) = ((num(i)?, num(i + 1)?), (num(i + 2)?, num(i + 3)?), (num(i + 4)?, num(i + 5)?));
                            for k in 1..=10 {
                                let t = k as f64 / 10.0;
                                let u = 1.0 - t;
                                pts.push((
                                    u * u * u * a.0 + 3.0 * u * u * t * p1.0 + 3.0 * u * t * t * p2.0 + t * t * t * e.0,
                                    u * u * u * a.1 + 3.0 * u * u * t * p1.1 + 3.0 * u * t * t * p2.1 + t * t * t * e.1,
                                ));
                            }
                            i += 6;
                        }
                        other => {
                            warn.push(format!("path command {other} is not supported, the object is partly ignored"));
                            return Some(pts);
                        }
                    },
                }
            }
            Some(pts)
        }
    }
}

/// All contours of a `path` value: an array of contours, or one contour.
fn contours(path: &Value, warn: &mut Vec<String>) -> Vec<Vec<Pt>> {
    let Some(a) = path.as_array() else { return vec![] };
    if a.first().is_some_and(|v| v.is_array()) {
        a.iter().filter_map(|c| c.as_array()).filter_map(|c| contour(c, warn)).collect()
    } else {
        contour(a, warn).into_iter().collect()
    }
}

fn bbox_area(v: &[Pt]) -> f64 {
    let (x0, x1) = v.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.0), a.1.max(p.0)));
    let (y0, y1) = v.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p.1), a.1.max(p.1)));
    (x1 - x0) * (y1 - y0)
}

// ------------------------------------------------------------------ reading

/// Name and text of the `.epru` entry of an `.epro2` archive.
fn read_epru(bytes: &[u8]) -> Result<(String, String), String> {
    let mut z = zip::ZipArchive::new(Cursor::new(bytes))
        .map_err(|e| format!("not an EasyEDA Pro project (.epro2 and .epro are ZIP archives): {e}"))?;
    let names: Vec<String> = z.file_names().filter_map(|n| n.ok()).map(|n| n.to_string()).collect();
    let name = names
        .iter()
        .find(|n| n.to_lowercase().ends_with(".epru"))
        .ok_or("the project archive has no .epru entry")?
        .clone();
    Ok((name.clone(), read_entry(&mut z, &name)?))
}

fn read_entry(z: &mut zip::ZipArchive<Cursor<&[u8]>>, name: &str) -> Result<String, String> {
    let mut text = String::new();
    z.by_name(name)
        .map_err(|e| e.to_string())?
        .read_to_string(&mut text)
        .map_err(|e| format!("cannot read {name}: {e}"))?;
    Ok(text)
}

/// A project as the loader needs it, whatever the file layout.
struct Project {
    /// records of the PCB document
    recs: Vec<Rec>,
    /// records of every footprint, by uuid
    fps: HashMap<String, Vec<Rec>>,
    /// archive entry that holds the PCB, and its lines
    entry: String,
    lines: Vec<String>,
    /// new objects go in at this line
    insert_at: usize,
    /// highest ticket (.epro2) or object number (.epro)
    seed: u64,
    legacy: bool,
    notes: Vec<String>,
}

/// Load an EasyEDA Pro project: `.epro2` (one `.epru` text with all documents) or the older
/// `.epro` (one file per document, every record a positional JSON array).
pub fn load(bytes: &[u8]) -> Result<EasyedaDoc, String> {
    let mut z = zip::ZipArchive::new(Cursor::new(bytes))
        .map_err(|e| format!("not an EasyEDA Pro project (.epro2 and .epro are ZIP archives): {e}"))?;
    let names: Vec<String> = z.file_names().filter_map(|n| n.ok()).map(|n| n.to_string()).collect();
    let has = |ext: &str| names.iter().any(|n| n.to_lowercase().ends_with(ext));
    let json_field = |z: &mut zip::ZipArchive<Cursor<&[u8]>>, entry: &str, f: &dyn Fn(&Value) -> Option<String>| {
        read_entry(z, entry).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()).and_then(|v| f(&v)).unwrap_or_default()
    };
    if has(".epru") {
        let (name, text) = read_epru(bytes)?;
        let title = json_field(&mut z, "project2.json", &|v| v.get("title")?.as_str().map(|s| s.to_string()));
        build(bytes.to_vec(), project_epro2(name, &text)?, title)
    } else if has(".epcb") {
        let title = json_field(&mut z, "project.json", &|v| v.pointer("/config/title")?.as_str().map(|s| s.to_string()));
        let p = project_epro1(&mut z, &names)?;
        build(bytes.to_vec(), p, title)
    } else {
        Err("not an EasyEDA Pro project: the archive has neither a .epru nor a .epcb entry".into())
    }
}

fn project_epro2(entry: String, text: &str) -> Result<Project, String> {
    let lines: Vec<String> = text.split('\n').map(|l| l.to_string()).collect();
    // ---- split into documents
    let mut spans: Vec<DocSpan> = vec![];
    let mut max_ticket = 0u64;
    for (i, l) in lines.iter().enumerate() {
        let Some((h, b)) = split_line(l) else { continue };
        let Ok(head) = serde_json::from_str::<Value>(h) else { continue };
        for k in ["ticket", "firstTicket"] {
            max_ticket = max_ticket.max(head.get(k).and_then(|v| v.as_u64()).unwrap_or(0));
        }
        if head.get("type").and_then(|t| t.as_str()) == Some("DOCHEAD") {
            if let Some(last) = spans.last_mut() {
                last.end = i;
            }
            let body = serde_json::from_str::<Value>(b).unwrap_or(Value::Null);
            spans.push(DocSpan { start: i, end: lines.len(), doc_type: s(&body, "docType").into(), uuid: s(&body, "uuid").into() });
        }
    }
    if spans.is_empty() {
        return Err("not an EasyEDA Pro project: the .epru entry has no documents".into());
    }
    // ---- the PCB: the one with the most components
    let pcbs: Vec<usize> = (0..spans.len()).filter(|&i| spans[i].doc_type == "PCB").collect();
    if pcbs.is_empty() {
        return Err("the project has no PCB document".into());
    }
    let mut best = (0usize, 0usize);
    for &i in &pcbs {
        let n = lines[spans[i].start..spans[i].end].iter().filter(|l| l.starts_with("{\"type\":\"COMPONENT\"")).count();
        if n >= best.1 {
            best = (i, n);
        }
    }
    let pcb = &spans[best.0];
    let recs = parse_records(&lines, pcb);
    let mut notes: Vec<String> = vec![];
    if pcbs.len() > 1 {
        let name = recs.iter().find(|r| r.ty == "META").map(|r| s(&r.body, "title").to_string()).unwrap_or_default();
        notes.push(format!("the project has {} PCBs; \"{name}\" (the one with the most parts) is used", pcbs.len()));
    }
    let fps = spans.iter().filter(|d| d.doc_type == "FOOTPRINT").map(|d| (d.uuid.clone(), parse_records(&lines, d))).collect();
    let mut insert_at = pcb.end;
    while insert_at > pcb.start + 1 && lines[insert_at - 1].trim().is_empty() {
        insert_at -= 1;
    }
    Ok(Project { recs, fps, entry, lines, insert_at, seed: max_ticket, legacy: false, notes })
}

fn legacy_lines(text: &str) -> Vec<String> {
    text.split('\n').map(|l| l.to_string()).collect()
}

fn legacy_records(lines: &[String]) -> Vec<Rec> {
    lines
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            let a: Vec<Value> = serde_json::from_str(l.trim()).ok()?;
            legacy_rec(&a, i)
        })
        .collect()
}

fn project_epro1(z: &mut zip::ZipArchive<Cursor<&[u8]>>, names: &[String]) -> Result<Project, String> {
    let ends = |n: &str, e: &str| n.to_lowercase().ends_with(e);
    // ---- the PCB: the one with the most components
    let mut best: Option<(String, Vec<String>, usize)> = None;
    let pcb_names: Vec<&String> = names.iter().filter(|n| ends(n, ".epcb")).collect();
    for n in &pcb_names {
        let lines = legacy_lines(&read_entry(z, n)?);
        let c = lines.iter().filter(|l| l.starts_with("[\"COMPONENT\"")).count();
        if best.as_ref().is_none_or(|b| c > b.2) {
            best = Some(((*n).clone(), lines, c));
        }
    }
    let (entry, lines, _) = best.ok_or("the project has no PCB document")?;
    let recs = legacy_records(&lines);
    let mut notes = vec![];
    if pcb_names.len() > 1 {
        notes.push(format!("the project has {} PCBs; the one with the most parts is used", pcb_names.len()));
    }
    // ---- footprints: FOOTPRINT/<uuid>.efoo
    let mut fps: HashMap<String, Vec<Rec>> = HashMap::new();
    for n in names.iter().filter(|n| ends(n, ".efoo")) {
        let uuid = n.rsplit('/').next().unwrap_or(n).trim_end_matches(".efoo").to_string();
        fps.insert(uuid, legacy_records(&legacy_lines(&read_entry(z, n)?)));
    }
    let mut insert_at = lines.len();
    while insert_at > 1 && lines[insert_at - 1].trim().is_empty() {
        insert_at -= 1;
    }
    Ok(Project { recs, fps, entry, lines, insert_at, seed: 0, legacy: true, notes })
}

/// A record of the older `.epro` layout (`["TYPE", id, ...]`, fields by position) as the body
/// the newer layout uses (named fields), so both are read by the same code.
fn legacy_rec(a: &[Value], line: usize) -> Option<Rec> {
    let ty = a.first()?.as_str()?;
    let g = |i: usize| a.get(i).cloned().unwrap_or(Value::Null);
    let id = a.get(1).and_then(|v| v.as_str()).unwrap_or("").to_string();
    let flag = |i: usize| a.get(i).is_some_and(|v| v.as_bool().unwrap_or_else(|| v.as_f64().unwrap_or(0.0) != 0.0));
    let mk = |ty: &str, id: String, body: Value| Some(Rec { ty: ty.into(), id, line, body });
    match ty {
        "LINE" => mk(ty, id, json!({"netName":g(3),"layerId":g(4),"startX":g(5),"startY":g(6),"endX":g(7),"endY":g(8),"width":g(9)})),
        "VIA" => mk(ty, id, json!({"netName":g(3),"centerX":g(5),"centerY":g(6),"holeDiameter":g(7),"viaDiameter":g(8)})),
        "PAD" => {
            // "no hole" is a round hole of size 0
            let hole = a
                .get(9)
                .and_then(|h| h.as_array())
                .filter(|h| h.len() >= 3 && (h[1].as_f64().unwrap_or(0.0) > 0.0 || h[2].as_f64().unwrap_or(0.0) > 0.0))
                .map(|h| json!({"holeType":h[0],"width":h[1],"height":h[2]}));
            let pad = a
                .get(10)
                .and_then(|p| p.as_array())
                .filter(|p| p.len() >= 3)
                .map(|p| json!({"padType":p[0],"width":p[1],"height":p[2],"radius":p.get(3).cloned().unwrap_or(json!(0))}));
            mk(
                ty,
                id,
                json!({"netName":g(3),"layerId":g(4),"num":g(5),"centerX":g(6),"centerY":g(7),"padAngle":g(8),"hole":hole,
                    "defaultPad":pad,"padOffsetX":g(12),"padOffsetY":g(13),"relativeAngle":g(14),"plated":flag(15)}),
            )
        }
        "FILL" => mk(ty, id, json!({"netName":g(3),"layerId":g(4),"width":g(5),"path":g(7)})),
        "POLY" => mk(ty, id, json!({"netName":g(3),"layerId":g(4),"width":g(5),"path":g(6)})),
        "POUR" => mk(ty, id, json!({"netName":g(3),"layerId":g(4),"name":g(6),"order":g(7),"path":g(8)})),
        "COMPONENT" => mk(ty, id, json!({"layerId":g(3),"x":g(4),"y":g(5),"angle":g(6)})),
        "ATTR" => mk(ty, id, json!({"parentId":g(3),"x":g(5),"y":g(6),"key":g(7),"value":g(8)})),
        // [PAD_NET, component, pad number, net, pad id, length]
        "PAD_NET" => mk(ty, json!(["PAD_NET", g(1), g(2), g(4)]).to_string(), json!({"padNet":g(3)})),
        "LAYER" => mk(ty, json!(["LAYER", g(1)]).to_string(), json!({"use":flag(4)})),
        "RULE" => {
            let (kind, ctx) = match a.get(1).and_then(|k| k.as_str())? {
                "1" => ("SAFE", json!({"safeSpacing":[{"content":a.get(4).and_then(|c| c.get(1)).cloned().unwrap_or(Value::Null)}]})),
                "3" => ("TRACK", json!({"track":{"content":[{"stroDef":a.get(4).and_then(|c| c.get(2)).cloned().unwrap_or(Value::Null)}]}})),
                "5" => ("RADIUS", g(4)),
                _ => return None,
            };
            let state = if flag(3) { json!("DEFAULT") } else { json!(0) };
            mk(ty, json!(["RULE", kind, g(2)]).to_string(), json!({"ruleState":state,"ruleContext":ctx}))
        }
        "HEAD" => mk("META", String::new(), json!({"title":a.get(1).and_then(|h| h.get("title")).cloned().unwrap_or(Value::Null)})),
        // geometry this reader does not know the layout of: reported, not guessed
        t if GEOMETRY.contains(&t) => mk("UNKNOWN", id, json!({"type":t})),
        _ => None,
    }
}

#[derive(Default)]
struct Acc {
    pads: Vec<Pad>,
    holes: Vec<(f64, f64, f64)>,
    tracks: Vec<(f64, String, String, Vec<Pt>)>,
    vias: Vec<(f64, f64, f64, String)>,
    strokes: Vec<Stroke>,
    regions: Vec<Region>,
    counts: [usize; 6],
    areas: HashSet<(String, String)>,
    /// copper layers: (Pro layer id, router layer name)
    copper: Vec<(i64, String)>,
    outlines: Vec<Vec<Pt>>,
    warn: Vec<String>,
    unknown: HashMap<String, usize>,
}

/// Router layer name of a copper layer id: top "1", bottom "2", inner layer n "In<n>".
pub fn layer_name(id: i64) -> String {
    match id {
        1 => "1".into(),
        2 => "2".into(),
        n => format!("In{}", n - 14),
    }
}

/// Pro layer id of a router layer name.
pub fn layer_id(name: &str) -> Option<i64> {
    match name {
        "1" => Some(1),
        "2" => Some(2),
        n => n.strip_prefix("In")?.parse::<i64>().ok().map(|k| k + 14),
    }
}

impl Acc {
    fn layer(&self, id: i64) -> Option<String> {
        self.copper.iter().find(|(i, _)| *i == id).map(|(_, n)| n.clone())
    }

    fn keepout_all(&mut self, pts: Vec<Pt>) {
        let pts = close(pts.into_iter().map(st).collect());
        if pts.len() < 4 {
            return;
        }
        for (_, l) in self.copper.clone() {
            self.regions.push(Region { layer: l, net: String::new(), pts: pts.clone(), copper: false });
        }
    }

    /// pad of a footprint (or a loose pad of the board); true when it has copper and became a pin
    fn pad(&mut self, b: &Value, xf: &Xf, net: &str) -> bool {
        let layer = match xf.layer(int(b, "layerId")) {
            1 => "1",
            2 => "2",
            12 => "11",
            _ => return false,
        };
        let plated = b.get("plated").and_then(|v| v.as_bool()).unwrap_or(true);
        let c = xf.pt((f(b, "centerX"), f(b, "centerY")));
        let rot = xf.rot(f(b, "padAngle"));
        let hole = present(b, "hole").map(|h| (s(h, "holeType").to_string(), f(h, "width"), f(h, "height")));
        let dp = present(b, "defaultPad").map(|d| (s(d, "padType").to_string(), f(d, "width"), f(d, "height"), f(d, "radius")));

        let mut copper = true;
        if let Some((kind, hw, hh)) = &hole {
            // the hole is displaced from the pad centre by the pad offset and turned by its own angle
            let off = rotate((f(b, "padOffsetX"), f(b, "padOffsetY")), rot);
            let hc = (c.0 + off.0, c.1 + off.1);
            if !plated {
                let big = dp.as_ref().is_some_and(|d| d.1 > hw + 1.0 || d.2 > hh + 1.0);
                copper = big;
                if kind == "SLOT" && (hw - hh).abs() > 1e-3 {
                    let a = rot + f(b, "relativeAngle");
                    let (long, short) = (hw.max(*hh), hw.min(*hh));
                    let half = long / 2.0 - short / 2.0;
                    let d = if hw >= hh { rotate((half, 0.0), a) } else { rotate((0.0, half), a) };
                    let poly = stadium((hc.0 - d.0, hc.1 - d.1), (hc.0 + d.0, hc.1 + d.1), short / 2.0, (0.0, 0.0));
                    self.keepout_all(poly);
                } else {
                    let h = st(hc);
                    self.holes.push((h.0, h.1, hw.max(*hh) / 2.0 * PX));
                }
                self.counts[1] += 1;
            }
        }
        if !copper {
            return false;
        }
        let (shape, w, h, _radius) = match (&dp, &hole) {
            (Some(d), _) => (d.0.clone(), d.1, d.2, d.3),
            (None, Some((_, hw, hh, ..))) => ("ELLIPSE".to_string(), *hw, *hh, 0.0),
            _ => return false,
        };
        if w <= 0.0 || h <= 0.0 {
            return false;
        }
        let cs = st(c);
        let mut pad = Pad {
            shape: String::new(),
            x: cs.0,
            y: cs.1,
            w: w * PX,
            h: h * PX,
            layer: layer.into(),
            net: net.into(),
            pts: vec![],
            rot: 0.0,
        };
        match shape.as_str() {
            "OVAL" | "ELLIPSE" if (w - h).abs() < 1e-3 => pad.shape = "ELLIPSE".into(),
            "OVAL" | "ELLIPSE" => {
                // a stadium along the longer axis (an ellipse is covered by it)
                let half = w.max(h) / 2.0 - w.min(h) / 2.0;
                let d = if w >= h { rotate((half, 0.0), rot) } else { rotate((0.0, half), rot) };
                pad.shape = "OVAL".into();
                pad.pts = vec![st((c.0 - d.0, c.1 - d.1)), st((c.0 + d.0, c.1 + d.1))];
            }
            other => {
                if other != "RECT" {
                    self.warn.push(format!("pad shape {other} is replaced by its bounding rectangle"));
                }
                // rounded corners are left sharp: never smaller than the real copper
                pad.shape = "RECT".into();
                pad.pts = rect_poly(c.0 - w / 2.0, c.1 + h / 2.0, w, h, 0.0, 0.0)
                    .into_iter()
                    .map(|p| {
                        let q = rotate((p.0 - c.0, p.1 - c.1), rot);
                        st((q.0 + c.0, q.1 + c.1))
                    })
                    .collect();
            }
        }
        self.pads.push(pad);
        true
    }

    /// Any non-pad object of the board or of a footprint.
    fn object(&mut self, ty: &str, b: &Value, xf: &Xf, in_fp: bool) {
        let net = s(b, "netName").to_string();
        let layer_id = xf.layer(int(b, "layerId"));
        match ty {
            "LINE" => {
                if let Some(l) = self.layer(layer_id) {
                    let a = st(xf.pt((f(b, "startX"), f(b, "startY"))));
                    let e = st(xf.pt((f(b, "endX"), f(b, "endY"))));
                    self.tracks.push((f(b, "width") * PX, l, net, vec![a, e]));
                    if in_fp {
                        self.counts[5] += 1;
                    }
                }
            }
            "ARC" => {
                if let Some(l) = self.layer(layer_id) {
                    let (a, e) = ((f(b, "startX"), f(b, "startY")), (f(b, "endX"), f(b, "endY")));
                    if a == e && f(b, "arcAngle") == 0.0 {
                        *self.unknown.entry("ARC".into()).or_default() += 1;
                        return;
                    }
                    let mut pts = vec![a];
                    pts.extend(arc_points(a, e, f(b, "arcAngle")));
                    self.strokes.push(Stroke {
                        width: f(b, "width") * PX,
                        layer: l,
                        net,
                        pts: pts.into_iter().map(|p| st(xf.pt(p))).collect(),
                    });
                    self.counts[2] += 1;
                }
            }
            "CIRCLE" => {
                if let Some(l) = self.layer(layer_id) {
                    let (c, r) = ((f(b, "centerX"), f(b, "centerY")), f(b, "radius"));
                    if r <= 0.0 {
                        *self.unknown.entry("CIRCLE".into()).or_default() += 1;
                        return;
                    }
                    let ring = close(circle_poly(c, r).into_iter().map(|p| st(xf.pt(p))).collect());
                    self.strokes.push(Stroke { width: f(b, "width") * PX, layer: l, net, pts: ring });
                    self.counts[2] += 1;
                }
            }
            "VIA" => {
                let c = st(xf.pt((f(b, "centerX"), f(b, "centerY"))));
                self.vias.push((c.0, c.1, f(b, "viaDiameter") * PX, net));
                if in_fp {
                    self.counts[0] += 1;
                }
            }
            "FILL" | "POLY" | "POUR" => {
                let Some(path) = b.get("path") else { return };
                let polys: Vec<Vec<Pt>> = {
                    let mut w = std::mem::take(&mut self.warn);
                    let v = contours(path, &mut w);
                    self.warn = w;
                    v
                };
                let poly_type = s(b, "polyType").to_string();
                if ty == "POLY" && layer_id == 11 {
                    // board outline (the biggest) and cut-outs
                    self.outlines.extend(polys);
                    return;
                }
                for p in polys {
                    let pts: Vec<Pt> = p.into_iter().map(|q| xf.pt(q)).collect();
                    if pts.len() < 3 {
                        continue;
                    }
                    match (ty, layer_id) {
                        // slot region / board cut-out: nothing may be left in it, on any layer
                        (_, 11) | (_, 12) => {
                            self.keepout_all(pts);
                            self.counts[1] += 1;
                        }
                        ("POLY", l) if poly_type == "NORMAL" || poly_type.is_empty() => {
                            // a polyline on copper
                            if let Some(name) = self.layer(l) {
                                self.strokes.push(Stroke {
                                    width: f(b, "width") * PX,
                                    layer: name,
                                    net: net.clone(),
                                    pts: pts.into_iter().map(st).collect(),
                                });
                                self.counts[2] += 1;
                            }
                        }
                        (_, l) => {
                            let Some(name) = self.layer(l) else { continue };
                            let pts = close(pts.into_iter().map(st).collect());
                            // a region of a type we do not know is kept clear, never used as copper
                            let known = ty != "POLY";
                            if !known {
                                self.warn.push(format!("region type {poly_type} is treated as a keep-out"));
                            }
                            if ty == "POUR" {
                                self.areas.insert((net.clone(), name.clone()));
                                self.counts[4] += 1;
                            } else {
                                self.counts[1] += 1;
                            }
                            self.regions.push(Region { layer: name, net: net.clone(), pts, copper: known });
                        }
                    }
                }
            }
            "PAD" | "POURED" | "ATTR" | "COMPONENT" | "PAD_NET" => {}
            other => {
                *self.unknown.entry(other.to_string()).or_default() += 1;
            }
        }
    }
}

/// Active design rules of the PCB document: track width, clearance, via diameter and hole, mil.
fn rules(recs: &[Rec]) -> (f64, f64, f64, f64) {
    let (mut track, mut clear, mut via, mut hole) = (10.0, 6.0, 24.0, 12.0);
    for r in recs.iter().filter(|r| r.ty == "RULE") {
        if r.body.get("ruleState").and_then(|v| v.as_str()) != Some("DEFAULT") {
            continue;
        }
        let ctx = r.body.get("ruleContext").unwrap_or(&Value::Null);
        let kind = serde_json::from_str::<Value>(&r.id).ok().and_then(|v| v.get(1).and_then(|k| k.as_str()).map(|k| k.to_string())).unwrap_or_default();
        match kind.as_str() {
            "TRACK" => {
                if let Some(d) = ctx.pointer("/track/content/0/stroDef").and_then(|v| v.as_f64()) {
                    track = d;
                }
            }
            "RADIUS" => {
                if let Some(d) = ctx.get("defRadius").and_then(|v| v.as_f64()) {
                    via = 2.0 * d;
                }
                if let Some(d) = ctx.get("defInner").and_then(|v| v.as_f64()) {
                    hole = 2.0 * d;
                }
            }
            "SAFE" => {
                // lower-triangular matrix, track / SMD pad / through-hole pad / via first
                if let Some(rows) = ctx.pointer("/safeSpacing/0/content").and_then(|v| v.as_array()) {
                    let m = rows.iter().take(4).filter_map(|r| r.as_array()).flatten().filter_map(|v| v.as_f64()).fold(0.0, f64::max);
                    if m > 0.0 {
                        clear = m;
                    }
                }
            }
            _ => {}
        }
    }
    (track, clear, via, hole)
}

fn build(bytes: Vec<u8>, p: Project, title: String) -> Result<EasyedaDoc, String> {
    let Project { recs, fps, lines, entry, insert_at, seed, legacy, mut notes } = p;
    let mut acc = Acc::default();
    // ---- copper layers: top, bottom, then the enabled inner layers
    acc.copper = vec![(1, "1".into()), (2, "2".into())];
    let mut inner: Vec<i64> = vec![];
    for r in recs.iter().filter(|r| r.ty == "LAYER") {
        let id = serde_json::from_str::<Value>(&r.id).ok().and_then(|v| v.get(1).and_then(|k| k.as_i64())).unwrap_or(0);
        if (15..=46).contains(&id) && r.body.get("use").and_then(|u| u.as_bool()) == Some(true) {
            inner.push(id);
        }
    }
    inner.sort();
    for id in inner {
        acc.copper.push((id, layer_name(id)));
    }

    // pad nets: (component id, pad element id) -> net
    let mut pad_net: HashMap<(String, String), String> = HashMap::new();
    for r in recs.iter().filter(|r| r.ty == "PAD_NET") {
        if let Some(v) = serde_json::from_str::<Value>(&r.id).ok() {
            if let (Some(c), Some(p)) = (v.get(1).and_then(|x| x.as_str()), v.get(3).and_then(|x| x.as_str())) {
                pad_net.insert((c.to_string(), p.to_string()), s(&r.body, "padNet").to_string());
            }
        }
    }
    // attributes per component
    let mut attr: HashMap<(&str, &str), (&Value, usize)> = HashMap::new();
    for r in recs.iter().filter(|r| r.ty == "ATTR") {
        attr.insert((s(&r.body, "parentId"), s(&r.body, "key")), (&r.body, r.line));
    }

    // ---- components
    let mut parts: Vec<PartInfo> = vec![];
    let mut part_lines: Vec<PartLines> = vec![];
    let mut bottom = 0;
    let mut missing: Vec<String> = vec![];
    for comp in recs.iter().filter(|r| r.ty == "COMPONENT") {
        let uuid = attr.get(&(comp.id.as_str(), "Footprint")).map(|a| s(a.0, "value")).unwrap_or("");
        let des = attr.get(&(comp.id.as_str(), "Designator")).map(|a| s(a.0, "value").to_string()).unwrap_or_default();
        let Some(frecs) = fps.get(uuid) else {
            if !uuid.is_empty() || !des.is_empty() {
                missing.push(if des.is_empty() { comp.id.clone() } else { des });
            }
            continue;
        };
        let xf = Xf { x: f(&comp.body, "x"), y: f(&comp.body, "y"), angle: f(&comp.body, "angle"), bottom: int(&comp.body, "layerId") == 2 };
        if xf.bottom {
            bottom += 1;
        }
        let before = acc.pads.len();
        let mut smd = true;
        for r in frecs.iter() {
            if r.ty == "PAD" {
                let net = pad_net.get(&(comp.id.clone(), r.id.clone())).cloned().unwrap_or_default();
                if present(&r.body, "hole").is_some() || int(&r.body, "layerId") == 12 {
                    smd = false;
                }
                acc.pad(&r.body, &xf, &net);
            } else {
                if GEOMETRY.contains(&r.ty.as_str()) {
                    acc.object(&r.ty, &r.body, &xf, true);
                }
            }
        }
        let package = frecs.iter().find(|r| r.ty == "META").map(|r| s(&r.body, "title").to_string()).unwrap_or_default();
        let mut attrs = vec![];
        for r in recs.iter().filter(|r| r.ty == "ATTR" && s(&r.body, "parentId") == comp.id) {
            if present(&r.body, "x").is_some() && present(&r.body, "y").is_some() {
                attrs.push(r.line);
            }
        }
        parts.push(PartInfo {
            shape: comp.line,
            package,
            designator: des,
            pads: (before + 1..=acc.pads.len()).collect(),
            smd,
            movable: true,
        });
        part_lines.push(PartLines { comp: comp.line, attrs });
    }
    if !missing.is_empty() {
        notes.push(format!("{} part(s) without a footprint in the project are ignored: {}", missing.len(), missing.join(", ")));
    }
    if bottom > 0 {
        notes.push(format!("{bottom} part(s) on the bottom side: footprints are mirrored about the vertical axis (not verified against EasyEDA Pro)"));
    }

    // ---- loose objects of the board
    let mut existing: Vec<usize> = vec![];
    for r in &recs {
        // tracks and vias that are on the board already
        if match r.ty.as_str() {
            "VIA" => true,
            "LINE" | "ARC" => acc.layer(int(&r.body, "layerId")).is_some(),
            _ => false,
        } {
            existing.push(r.line);
        }
        match r.ty.as_str() {
            "PAD" => {
                acc.pad(&r.body, &Xf::ID, s(&r.body, "netName"));
            }
            t if GEOMETRY.contains(&t) => acc.object(&r.ty, &r.body, &Xf::ID, false),
            _ => {}
        }
    }

    for r in recs.iter().chain(fps.values().flatten()).filter(|r| r.ty == "UNKNOWN") {
        *acc.unknown.entry(s(&r.body, "type").to_string()).or_default() += 1;
    }

    // ---- board outline: the biggest closed contour; the others are cut-outs
    let mut outs = std::mem::take(&mut acc.outlines);
    outs.sort_by(|a, b| bbox_area(b).total_cmp(&bbox_area(a)));
    let mut outline: Vec<Vec<Pt>> = vec![];
    for (k, o) in outs.into_iter().enumerate() {
        if o.len() < 3 {
            continue;
        }
        if k == 0 {
            outline.push(close(o.into_iter().map(st).collect()));
        } else {
            acc.keepout_all(o);
            acc.counts[1] += 1;
        }
    }

    // ---- rules
    let (track, clear, via, hole) = rules(&recs);
    notes.push(format!(
        "design rules (mil): track {track}, clearance {clear} (largest of track/pad/via), via {via} / hole {hole}"
    ));
    for (k, n) in &acc.unknown {
        notes.push(format!("{n} {k} object(s) were not recognised and are ignored"));
    }
    acc.warn.sort();
    acc.warn.dedup();
    notes.extend(acc.warn.iter().cloned());

    // ---- bookkeeping for writing back
    let ids: HashSet<String> = recs.iter().map(|r| r.id.clone()).collect();
    let mut pour_n = 0usize;
    let mut pour_order = -1i64;
    for r in recs.iter().filter(|r| r.ty == "POUR") {
        pour_n = pour_n.max(s(&r.body, "name").trim_start_matches("POUR").parse().unwrap_or(0));
        pour_order = pour_order.max(int(&r.body, "order"));
    }
    let next_eid = ids.iter().filter_map(|i| i.strip_prefix('e')?.parse::<u64>().ok()).max().unwrap_or(0) + 1;
    let pro = ProDoc {
        bytes,
        epru_name: entry,
        lines,
        insert_at,
        max_ticket: seed,
        legacy,
        next_eid,
        ids: ids.clone(),
        pour_n,
        pour_order,
        parts: part_lines,
        via_hole_mil: hole,
        title,
    };
    notes.insert(
        0,
        format!(
            "EasyEDA Pro {} PCB with {} copper layer(s), {} part(s), {} pad(s)",
            if legacy { "(.epro)" } else { "(.epro2)" },
            acc.copper.len(),
            parts.len(),
            acc.pads.len()
        ),
    );
    let copper: Vec<String> = acc.copper.iter().map(|(_, n)| n.clone()).collect();
    finish(
        Collected {
            notes,
            pads: acc.pads,
            holes: acc.holes,
            outline,
            tracks: acc.tracks,
            vias: acc.vias,
            strokes: acc.strokes,
            regions: acc.regions,
            counts: acc.counts,
            copper,
            areas: acc.areas,
            ids,
            other_outline: 0,
            track_w: track * PX,
            clear: clear * PX + CLEARANCE_MARGIN,
            via_d: via * PX,
            special: vec![],
            existing,
            rules_mm: Some([track * 0.0254, clear * 0.0254, via * 0.0254, hole * 0.0254]),
        },
        Value::Null,
        parts,
        Some(pro),
    )
}

// ------------------------------------------------------------------ writing

fn r4(v: f64) -> f64 {
    let r = (v * 1e4).round() / 1e4;
    if r == 0.0 { 0.0 } else { r }
}

/// SplitMix64, to make new object ids that look like the ones EasyEDA writes.
fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Move the x/y of a record by (dx, dy) mil.
fn shift_record(line: &str, dx: f64, dy: f64) -> Option<String> {
    let (h, b) = split_line(line)?;
    let mut body: Value = serde_json::from_str(b).ok()?;
    for (k, d) in [("x", dx), ("y", dy)] {
        let v = body.get(k)?.as_f64()?;
        body[k] = json!(r4(v + d));
    }
    let cr = if line.ends_with('\r') { "\r" } else { "" };
    Some(format!("{h}||{}|{cr}", serde_json::to_string(&body).ok()?))
}

/// Move the x/y of a record of the older `.epro` layout by (dx, dy) mil: components keep them at
/// fields 4 and 5, attributes (labels) at 5 and 6.
fn shift_legacy(line: &str, dx: f64, dy: f64) -> Option<String> {
    let mut a: Vec<Value> = serde_json::from_str(line.trim()).ok()?;
    let (ix, iy) = match a.first()?.as_str()? {
        "COMPONENT" => (4, 5),
        "ATTR" => (5, 6),
        _ => return None,
    };
    let (x, y) = (a.get(ix)?.as_f64()?, a.get(iy)?.as_f64()?);
    a[ix] = json!(r4(x + dx));
    a[iy] = json!(r4(y + dy));
    let cr = if line.ends_with('\r') { "\r" } else { "" };
    Some(format!("{}{cr}", serde_json::to_string(&a).ok()?))
}

/// New records, in the layout of the project they go into.
struct Writer<'a> {
    doc: &'a ProDoc,
    ticket: u64,
    eid: u64,
    used: HashSet<String>,
    counter: u64,
    lines: Vec<String>,
}

impl Writer<'_> {
    fn id(&mut self) -> String {
        if self.doc.legacy {
            self.eid += 1;
            return format!("e{}", self.eid - 1);
        }
        loop {
            self.counter += 1;
            let id = format!("{:016x}", mix(self.doc.max_ticket.wrapping_mul(0x1_0000_0001) ^ self.counter));
            if self.used.insert(id.clone()) {
                return id;
            }
        }
    }

    /// `body` is the object of the `.epro2` layout, `fields` the positional fields that follow
    /// the id in the `.epro` layout.
    fn push(&mut self, ty: &str, body: Value, fields: Vec<Value>) {
        let id = self.id();
        if self.doc.legacy {
            let mut a = vec![json!(ty), json!(id)];
            a.extend(fields);
            self.lines.push(Value::Array(a).to_string());
        } else {
            self.ticket += 1;
            self.lines.push(format!("{{\"type\":\"{ty}\",\"ticket\":{},\"id\":\"{id}\"}}||{body}|", self.ticket));
        }
    }
}

impl ProDoc {
    /// The project with `add` appended to the PCB and the parts moved by `moves` (part index,
    /// dx, dy) in router units, y up. The records at the line numbers `omit` (see
    /// `EasyedaDoc::existing`) are left out. Returns the new project file and the number of
    /// objects added.
    pub fn save(&self, add: &[NewObject], moves: &[(usize, f64, f64)], omit: &[usize]) -> Result<(Vec<u8>, usize), String> {
        let mut lines = self.lines.clone();
        for &(pi, dx, dy) in moves {
            let Some(p) = self.parts.get(pi) else { continue };
            for &li in std::iter::once(&p.comp).chain(p.attrs.iter()) {
                let moved = if self.legacy { shift_legacy(&lines[li], dx / PX, dy / PX) } else { shift_record(&lines[li], dx / PX, dy / PX) };
                if let Some(n) = moved {
                    lines[li] = n;
                }
            }
        }
        let mut w = Writer { doc: self, ticket: self.max_ticket, eid: self.next_eid, used: self.ids.clone(), counter: 0, lines: vec![] };
        let (mut pour_n, mut order) = (self.pour_n, self.pour_order);
        let mil = |p: Pt| (r4(p.0 / PX), r4(-p.1 / PX));
        for o in add {
            match o {
                NewObject::Track { net, layer, width, pts } => {
                    let Some(lid) = layer_id(layer) else { continue };
                    for seg in pts.windows(2) {
                        if (seg[0].0 - seg[1].0).abs() < 1e-9 && (seg[0].1 - seg[1].1).abs() < 1e-9 {
                            continue;
                        }
                        let (a, e) = (mil(seg[0]), mil(seg[1]));
                        let wd = r4(width / PX);
                        let body = json!({"partitionId":"","groupId":0,"netName":net,"layerId":lid,
                            "startX":a.0,"startY":a.1,"endX":e.0,"endY":e.1,"width":wd,"locked":false,"zIndex":-1});
                        w.push("LINE", body, vec![json!(0), json!(net), json!(lid), json!(a.0), json!(a.1), json!(e.0), json!(e.1), json!(wd), json!(0)]);
                    }
                }
                NewObject::Via { net, at, dia, hole } => {
                    let c = mil(*at);
                    let (hd, vd) = (r4(hole / PX), r4(dia / PX));
                    let body = json!({"partitionId":"","groupId":0,"netName":net,"ruleName":"","centerX":c.0,"centerY":c.1,
                        "holeDiameter":hd,"viaDiameter":vd,"viaType":"NORMAL","topSolderExpansion":null,
                        "bottomSolderExpansion":null,"locked":false,"unusedInnerLayers":[]});
                    w.push(
                        "VIA",
                        body,
                        vec![json!(0), json!(net), json!(""), json!(c.0), json!(c.1), json!(hd), json!(vd), json!(0), Value::Null, Value::Null, json!(0), json!([])],
                    );
                }
                NewObject::Pour { net, layer, ring } => {
                    let Some(lid) = layer_id(layer) else { continue };
                    let ring = close(ring.clone());
                    if ring.len() < 4 {
                        continue;
                    }
                    let mut path: Vec<Value> = vec![];
                    for (i, p) in ring.iter().enumerate() {
                        let m = mil(*p);
                        if i == 1 {
                            path.push(json!("L"));
                        }
                        path.push(json!(m.0));
                        path.push(json!(m.1));
                    }
                    pour_n += 1;
                    order += 1;
                    let name = format!("POUR{pour_n}");
                    let body = json!({"partitionId":"","groupId":0,"netName":net,"layerId":lid,"width":0.2,
                        "name":name,"order":order,"path":[path],"pourType":{"pourType":"SOLID","fineness":8},
                        "keepIsland":false,"locked":false,"zIndex":-1});
                    w.push(
                        "POUR",
                        body,
                        vec![json!(0), json!(net), json!(lid), json!(0.2), json!(name), json!(order), json!([path]), json!(["SOLID", 8]), json!(0), json!(0)],
                    );
                }
            }
        }
        let out = w.lines;
        let n = out.len();
        let omit: HashSet<usize> = omit.iter().copied().collect();
        let mut kept: Vec<String> = Vec::with_capacity(lines.len() + n);
        for (i, l) in lines.into_iter().enumerate() {
            if i == self.insert_at {
                kept.extend(out.iter().cloned());
            }
            if !omit.contains(&i) {
                kept.push(l);
            }
        }
        if self.insert_at >= self.lines.len() {
            kept.extend(out);
        }
        let text = kept.join("\n");
        Ok((self.pack(&text)?, n))
    }

    /// The archive with the PCB entry replaced; every other entry is copied as it is.
    fn pack(&self, text: &str) -> Result<Vec<u8>, String> {
        let mut zin = zip::ZipArchive::new(Cursor::new(&self.bytes[..])).map_err(|e| e.to_string())?;
        let mut out = Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut out);
            for i in 0..zin.len() {
                let file = zin.by_index_raw(i).map_err(|e| e.to_string())?;
                if file.name().is_ok_and(|n| n == self.epru_name.as_str()) {
                    drop(file);
                    let opts = zip::write::SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
                    zw.start_file(self.epru_name.as_str(), opts).map_err(|e| e.to_string())?;
                    zw.write_all(text.as_bytes()).map_err(|e| e.to_string())?;
                } else {
                    zw.raw_copy_file(file).map_err(|e| e.to_string())?;
                }
            }
            zw.finish().map_err(|e| e.to_string())?;
        }
        Ok(out.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Option<Vec<u8>> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../ProPrj_stm32f767_2026-11-17.epro2");
        std::fs::read(p).ok()
    }

    fn near(a: Pt, b: Pt) -> bool {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
    }

    #[test]
    fn lines_split_into_head_and_body() {
        let (h, b) = split_line("{\"type\":\"VIA\",\"ticket\":1,\"id\":\"a\"}||{\"netName\":\"A||B\"}|\r").unwrap();
        assert_eq!(h, "{\"type\":\"VIA\",\"ticket\":1,\"id\":\"a\"}");
        assert_eq!(b, "{\"netName\":\"A||B\"}");
        assert!(split_line("no separator").is_none());
    }

    #[test]
    fn arcs_turn_the_way_pro_writes_them() {
        // from EasyEDA Pro's own pour data: a -90 degree corner of radius 0.59 around (-62.5, 92.5)
        let pts = arc_points((-63.09, 92.5), (-62.5, 93.09), -90.0);
        assert!(near(*pts.last().unwrap(), (-62.5, 93.09)));
        for p in &pts {
            let d = ((p.0 + 62.5).powi(2) + (p.1 - 92.5).powi(2)).sqrt();
            assert!((d - 0.59).abs() < 1e-6, "{p:?} {d}");
        }
        // positive sweeps go counter-clockwise, about the same centre when the chord is reversed
        let ccw = arc_points((-62.5, 93.09), (-63.09, 92.5), 90.0);
        assert!(ccw.iter().all(|p| (((p.0 + 62.5).powi(2) + (p.1 - 92.5).powi(2)).sqrt() - 0.59).abs() < 1e-6));
        // a clockwise half circle from (0,0) to (2,0) bulges upwards
        let half = arc_points((0.0, 0.0), (2.0, 0.0), -180.0);
        assert!(half.iter().all(|p| p.1 > -1e-9 && (((p.0 - 1.0).powi(2) + p.1.powi(2)).sqrt() - 1.0).abs() < 1e-6));
    }

    #[test]
    fn paths_of_every_shape_are_read() {
        let mut w = vec![];
        let v: Value = serde_json::from_str(r#"[["R",-260,260,525,525,0,0]]"#).unwrap();
        let c = contours(&v, &mut w);
        assert_eq!(c.len(), 1);
        // top-left corner (x, y), extends down
        assert!(c[0].contains(&(-260.0, 260.0)) && c[0].contains(&(265.0, -265.0)));
        let v: Value = serde_json::from_str(r#"[-105,-860,"L",-20,-860,-20,-975,"ARC",-90,-30,-985,"L",-105,-860]"#).unwrap();
        let c = contours(&v, &mut w);
        assert_eq!(c.len(), 1);
        assert!(c[0].len() > 5 && near(*c[0].last().unwrap(), (-105.0, -860.0)));
        // rounded rectangle: stays inside its box
        let r = rect_poly(0.0, 10.0, 20.0, 10.0, 0.0, 3.0);
        assert!(r.iter().all(|p| p.0 >= -1e-9 && p.0 <= 20.0 + 1e-9 && p.1 <= 10.0 + 1e-9 && p.1 >= -1e-9));
        assert!(w.is_empty());
    }

    #[test]
    fn footprints_rotate_counter_clockwise() {
        let xf = Xf { x: 100.0, y: 50.0, angle: 90.0, bottom: false };
        assert!(near(xf.pt((10.0, 0.0)), (100.0, 60.0)));
        let bot = Xf { x: 0.0, y: 0.0, angle: 0.0, bottom: true };
        assert!(near(bot.pt((10.0, 5.0)), (-10.0, 5.0)));
        assert_eq!((bot.layer(1), bot.layer(2), bot.layer(12)), (2, 1, 12));
    }

    #[test]
    fn layer_names_round_trip() {
        for id in [1, 2, 15, 16, 46] {
            assert_eq!(layer_id(&layer_name(id)), Some(id));
        }
        assert_eq!(layer_name(15), "In1");
    }

    #[test]
    fn sample_project_loads_completely() {
        let Some(bytes) = sample() else { return };
        let doc = load(&bytes).unwrap();
        let b = crate::parse_dsn(&doc.dsn).unwrap();
        assert_eq!(b.layers.iter().map(|l| l.name.as_str()).collect::<Vec<_>>(), ["1", "2", "In1", "In2"]);
        assert_eq!(doc.parts.len(), 22);
        // 290 footprint pads and the loose through-hole pad
        assert_eq!(b.abs_pins().len(), 291);
        // the two routed nets: six track segments, six vias
        assert_eq!(b.wires.len(), 6);
        assert_eq!(b.vias.len(), 6);
        // board outline 1299.2 x 2125 mil = 129.92 x 212.5 units
        let xs = b.boundary.iter().map(|p| p.0);
        let ys = b.boundary.iter().map(|p| p.1);
        let (x0, x1) = (xs.clone().fold(f64::MAX, f64::min), xs.fold(f64::MIN, f64::max));
        let (y0, y1) = (ys.clone().fold(f64::MAX, f64::min), ys.fold(f64::MIN, f64::max));
        assert!((x1 - x0 - 129.92126).abs() < 1e-3 && (y1 - y0 - 212.5).abs() < 1e-3, "{x0} {x1} {y0} {y1}");
        // two GND fills and the VDD pour are copper of their net; the slot region and the two
        // slot regions of footprints are keep-outs on all four layers
        assert_eq!(b.planes.len(), 3);
        assert!(b.keepouts.len() >= 12);
        assert!(doc.areas.contains(&("VDD".to_string(), "1".to_string())));
        assert!(doc.pro.is_some());
    }

    /// An existing track starts and ends on the centre of a pad of its own net: proves the
    /// footprint placement of the importer without going through the router.
    #[test]
    fn existing_tracks_land_on_pads() {
        let Some(bytes) = sample() else { return };
        let doc = load(&bytes).unwrap();
        let b = crate::parse_dsn(&doc.dsn).unwrap();
        let nets: HashMap<String, String> = b
            .nets
            .iter()
            .flat_map(|n| n.pins.iter().map(move |p| (format!("{}-{}", p.comp, p.pin), n.name.clone())))
            .collect();
        let pins = b.abs_pins();
        let mut on_pad = 0;
        for w in &b.wires {
            for e in [w.pts[0], *w.pts.last().unwrap()] {
                if b.vias.iter().any(|v| (v.x - e.0).abs() < 0.01 && (v.y - e.1).abs() < 0.01) {
                    continue;
                }
                if pins.iter().any(|(k, p)| nets.get(k) == Some(&w.net) && (p.x - e.0).abs() < 0.01 && (p.y - e.1).abs() < 0.01) {
                    on_pad += 1;
                }
            }
        }
        // the pad-ended ends: C1.2, C13.2, U3.3, C2.1, C2.2, U3.1 (the rest are joints of a chain)
        assert!(on_pad >= 6, "only {on_pad} track ends are on a pad of their net");
    }

    #[test]
    fn saving_adds_objects_and_nothing_else() {
        let Some(bytes) = sample() else { return };
        let doc = load(&bytes).unwrap();
        let pro = doc.pro.as_ref().unwrap();
        let add = vec![
            NewObject::Track { net: "GND".into(), layer: "2".into(), width: 1.0, pts: vec![(0.0, 0.0), (10.0, 0.0), (10.0, -5.0)] },
            NewObject::Via { net: "GND".into(), at: (10.0, -5.0), dia: 2.4, hole: 1.2 },
            NewObject::Pour { net: "GND".into(), layer: "In1".into(), ring: vec![(0.0, 0.0), (5.0, 0.0), (5.0, -5.0), (0.0, -5.0)] },
        ];
        let (out, n) = pro.save(&add, &[], &[]).unwrap();
        assert_eq!(n, 4); // two track segments, a via and a pour
        let (_, text) = read_epru(&out).unwrap();
        let (_, old) = read_epru(&bytes).unwrap();
        // every original line is still there, in order; the new ones sit at the end of the PCB
        let (lo, ln) = (old.split('\n').collect::<Vec<_>>(), text.split('\n').collect::<Vec<_>>());
        assert_eq!(ln.len(), lo.len() + 4);
        let at = pro.insert_at;
        assert_eq!(&ln[..at], &lo[..at]);
        assert_eq!(&ln[at + 4..], &lo[at..]);
        assert!(ln[at].contains("\"type\":\"LINE\"") && ln[at].contains("\"layerId\":2"));
        assert!(ln[at + 3].contains("\"type\":\"POUR\"") && ln[at + 3].contains("\"layerId\":15") && ln[at + 3].contains("POUR2"));
        // document space (10 mil units, y down) back to mil, y up: (10, -5) is (100, 50) mil
        let via: Value = serde_json::from_str(split_line(ln[at + 2]).unwrap().1).unwrap();
        assert!((f(&via, "centerX") - 100.0).abs() < 1e-3 && (f(&via, "centerY") - 50.0).abs() < 1e-3, "{via}");
        assert!((f(&via, "viaDiameter") - 24.0).abs() < 1e-3 && (f(&via, "holeDiameter") - 12.0).abs() < 1e-3);
        // and the result loads again with the new copper in it
        let again = load(&out).unwrap();
        let b = crate::parse_dsn(&again.dsn).unwrap();
        assert_eq!(b.wires.len(), 6 + 2);
        assert_eq!(b.vias.len(), 6 + 1);
        assert!(again.areas.contains(&("GND".to_string(), "In1".to_string())));
        // other entries of the archive are copied as they are
        let (mut za, mut zb) = (zip::ZipArchive::new(Cursor::new(&bytes)).unwrap(), zip::ZipArchive::new(Cursor::new(&out)).unwrap());
        for name in ["project2.json", "IMAGE/38faafe4e1834ff1b73513f80374a7fa.webp"] {
            let (mut a, mut c) = (vec![], vec![]);
            za.by_name(name).unwrap().read_to_end(&mut a).unwrap();
            zb.by_name(name).unwrap().read_to_end(&mut c).unwrap();
            assert_eq!(a, c, "{name}");
        }
    }

    #[test]
    fn moving_a_part_moves_its_labels_too() {
        let Some(bytes) = sample() else { return };
        let doc = load(&bytes).unwrap();
        let pro = doc.pro.as_ref().unwrap();
        let (out, _) = pro.save(&[], &[(0, 1.0, 2.0)], &[]).unwrap(); // 10 mil right, 20 mil up
        let (_, old) = read_epru(&bytes).unwrap();
        let (_, new) = read_epru(&out).unwrap();
        let (lo, ln): (Vec<_>, Vec<_>) = (old.split('\n').collect(), new.split('\n').collect());
        let changed: Vec<usize> = (0..lo.len()).filter(|&i| lo[i] != ln[i]).collect();
        assert!(changed.contains(&pro.parts[0].comp) && changed.len() == 1 + pro.parts[0].attrs.len(), "{changed:?}");
        let body = |l: &str| serde_json::from_str::<Value>(split_line(l).unwrap().1).unwrap();
        let (a, b) = (body(lo[pro.parts[0].comp]), body(ln[pro.parts[0].comp]));
        assert!((f(&b, "x") - f(&a, "x") - 10.0).abs() < 1e-6 && (f(&b, "y") - f(&a, "y") - 20.0).abs() < 1e-6);
    }

    fn sample_legacy() -> Option<Vec<u8>> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../ProPrj_stm32f767_2026-11-17.epro");
        std::fs::read(p).ok()
    }

    /// The same project saved in the older `.epro` layout describes exactly the same board.
    #[test]
    fn legacy_project_matches_the_new_layout() {
        let (Some(a), Some(b)) = (sample(), sample_legacy()) else { return };
        let (new, old) = (load(&a).unwrap(), load(&b).unwrap());
        assert!(old.pro.as_ref().unwrap().legacy && !new.pro.as_ref().unwrap().legacy);
        assert_eq!(new.dsn, old.dsn);
        assert_eq!(new.parts.len(), old.parts.len());
        assert_eq!(new.areas, old.areas);
        assert!(old.notes[0].contains(".epro)"), "{:?}", old.notes);
    }

    #[test]
    fn saving_into_a_legacy_project_adds_records_in_its_layout() {
        let Some(bytes) = sample_legacy() else { return };
        let doc = load(&bytes).unwrap();
        let pro = doc.pro.as_ref().unwrap();
        let add = vec![
            NewObject::Track { net: "GND".into(), layer: "2".into(), width: 1.0, pts: vec![(0.0, 0.0), (10.0, 0.0), (10.0, -5.0)] },
            NewObject::Via { net: "GND".into(), at: (10.0, -5.0), dia: 2.4, hole: 1.2 },
            NewObject::Pour { net: "GND".into(), layer: "In1".into(), ring: vec![(0.0, 0.0), (5.0, 0.0), (5.0, -5.0), (0.0, -5.0)] },
        ];
        let (out, n) = pro.save(&add, &[], &[]).unwrap();
        assert_eq!(n, 4);
        let mut z = zip::ZipArchive::new(Cursor::new(&out[..])).unwrap();
        let text = read_entry(&mut z, &pro.epru_name).unwrap();
        let mut zo = zip::ZipArchive::new(Cursor::new(&bytes[..])).unwrap();
        let old = read_entry(&mut zo, &pro.epru_name).unwrap();
        let (lo, ln): (Vec<_>, Vec<_>) = (old.split('\n').collect(), text.split('\n').collect());
        assert_eq!(ln.len(), lo.len() + 4);
        let at = pro.insert_at;
        assert_eq!(&ln[..at], &lo[..at]);
        assert_eq!(&ln[at + 4..], &lo[at..]);
        let rec = |i: usize| serde_json::from_str::<Vec<Value>>(ln[at + i]).unwrap();
        // ["LINE", id, group, net, layer, x1, y1, x2, y2, width, locked]
        let l = rec(0);
        assert_eq!((l[0].as_str(), l[3].as_str(), l[4].as_i64(), l[9].as_f64()), (Some("LINE"), Some("GND"), Some(2), Some(10.0)));
        assert_eq!((l[5].as_f64(), l[6].as_f64(), l[7].as_f64(), l[8].as_f64()), (Some(0.0), Some(0.0), Some(100.0), Some(0.0)));
        // ["VIA", id, group, net, rule, x, y, hole, diameter, ...]
        let v = rec(2);
        assert_eq!((v[0].as_str(), v[5].as_f64(), v[6].as_f64(), v[7].as_f64(), v[8].as_f64()), (Some("VIA"), Some(100.0), Some(50.0), Some(12.0), Some(24.0)));
        // ["POUR", id, group, net, layer, width, name, order, path, ...]
        let p = rec(3);
        assert_eq!((p[0].as_str(), p[4].as_i64(), p[6].as_str()), (Some("POUR"), Some(15), Some("POUR2")));
        // fresh, unused object ids
        let ids: Vec<String> = (0..4).map(|i| rec(i)[1].as_str().unwrap().to_string()).collect();
        assert_eq!(ids.iter().collect::<HashSet<_>>().len(), 4);
        assert!(ids.iter().all(|i| !pro.ids.contains(i)), "{ids:?}");
        // loads again with the new copper in it
        let again = load(&out).unwrap();
        let b = crate::parse_dsn(&again.dsn).unwrap();
        assert_eq!((b.wires.len(), b.vias.len()), (6 + 2, 6 + 1));
        assert!(again.areas.contains(&("GND".to_string(), "In1".to_string())));
    }

    #[test]
    fn moving_a_part_of_a_legacy_project_moves_its_labels_too() {
        let Some(bytes) = sample_legacy() else { return };
        let doc = load(&bytes).unwrap();
        let pro = doc.pro.as_ref().unwrap();
        let (out, _) = pro.save(&[], &[(0, 1.0, 2.0)], &[]).unwrap();
        let mut z = zip::ZipArchive::new(Cursor::new(&out[..])).unwrap();
        let new = read_entry(&mut z, &pro.epru_name).unwrap();
        let ln: Vec<&str> = new.split('\n').collect();
        let changed: Vec<usize> = (0..pro.lines.len()).filter(|&i| pro.lines[i] != ln[i]).collect();
        assert!(changed.contains(&pro.parts[0].comp) && changed.len() == 1 + pro.parts[0].attrs.len(), "{changed:?}");
        let (a, b) = (
            serde_json::from_str::<Vec<Value>>(&pro.lines[pro.parts[0].comp]).unwrap(),
            serde_json::from_str::<Vec<Value>>(ln[pro.parts[0].comp]).unwrap(),
        );
        assert!((b[4].as_f64().unwrap() - a[4].as_f64().unwrap() - 10.0).abs() < 1e-6);
        assert!((b[5].as_f64().unwrap() - a[5].as_f64().unwrap() - 20.0).abs() < 1e-6);
    }

    /// The tracks and vias that were on the board can be left out of the saved project, in both
    /// file layouts, and nothing else goes with them.
    #[test]
    fn existing_tracks_and_vias_can_be_left_out() {
        for bytes in [sample(), sample_legacy()].into_iter().flatten() {
            let doc = load(&bytes).unwrap();
            let pro = doc.pro.as_ref().unwrap();
            // six track segments and six vias
            assert_eq!(doc.existing.len(), 12);
            let (out, _) = pro.save(&[], &[], &doc.existing).unwrap();
            let again = load(&out).unwrap();
            let (b0, b1) = (crate::parse_dsn(&doc.dsn).unwrap(), crate::parse_dsn(&again.dsn).unwrap());
            assert_eq!((b1.wires.len(), b1.vias.len()), (0, 0));
            assert_eq!(b1.abs_pins().len(), b0.abs_pins().len());
            assert_eq!((b1.planes.len(), b1.keepouts.len()), (b0.planes.len(), b0.keepouts.len()));
            // exactly those lines are gone
            let mut z = zip::ZipArchive::new(Cursor::new(&out[..])).unwrap();
            let new = read_entry(&mut z, &pro.epru_name).unwrap();
            assert_eq!(new.split('\n').count(), pro.lines.len() - 12);
            // omitting and adding at once keeps the new records at the end of the PCB
            let add = vec![NewObject::Via { net: "GND".into(), at: (1.0, -1.0), dia: 2.4, hole: 1.2 }];
            let (out, n) = pro.save(&add, &[], &doc.existing).unwrap();
            assert_eq!(n, 1);
            let b2 = crate::parse_dsn(&load(&out).unwrap().dsn).unwrap();
            assert_eq!((b2.wires.len(), b2.vias.len()), (0, 1));
        }
    }
}
