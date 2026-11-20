//! EasyEDA Pro `.epro2`: a ZIP whose `.epru` entry holds line-delimited records
//! `{head}||{body}|`. Documents start at a DOCHEAD line. Units: mil, y up. The PCB lists its
//! components by position; the pads come from the FOOTPRINT documents they refer to.
use crate::geom::*;
use crate::model::*;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::io::{Cursor, Read, Write};

/// mil -> mm
pub const U: f64 = 0.0254;

pub struct ProDoc {
    bytes: Vec<u8>,
    epru_name: String,
    lines: Vec<String>,
    /// part index -> label (ATTR) lines that have a position
    labels: Vec<Vec<usize>>,
    /// lines of tracks, vias and computed pours
    routing: Vec<usize>,
}

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

fn parse_range(lines: &[String], a: usize, b: usize) -> Vec<Rec> {
    let mut v = vec![];
    for i in a + 1..b {
        let Some((h, bd)) = split_line(&lines[i]) else { continue };
        let Ok(head) = serde_json::from_str::<Value>(h) else { continue };
        let ty = head.get("type").and_then(|t| t.as_str()).unwrap_or("").to_string();
        let id = head.get("id").and_then(|t| t.as_str()).unwrap_or("").to_string();
        let body = serde_json::from_str::<Value>(bd).unwrap_or(Value::Null);
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

/// footprint offset (mil) -> offset in the file's orientation (mil)
fn off(p: P, angle: f64, bottom: bool) -> P {
    let px = if bottom { -p.0 } else { p.0 };
    let a = angle.to_radians();
    (px * a.cos() - p.1 * a.sin(), px * a.sin() + p.1 * a.cos())
}

pub fn load(bytes: &[u8]) -> Result<(ProDoc, Board), String> {
    let mut z = zip::ZipArchive::new(Cursor::new(bytes)).map_err(|e| format!("not an EasyEDA Pro project (.epro2 is a ZIP archive): {e}"))?;
    let names: Vec<String> = z.file_names().filter_map(|n| n.ok()).map(|n| n.to_string()).collect();
    let epru = names.iter().find(|n| n.to_lowercase().ends_with(".epru")).cloned().ok_or("this looks like the older .epro layout, which is not supported; open the project in EasyEDA Pro and save it as .epro2")?;
    let mut text = String::new();
    z.by_name(&epru).map_err(|e| e.to_string())?.read_to_string(&mut text).map_err(|e| e.to_string())?;
    let lines: Vec<String> = text.split('\n').map(|l| l.to_string()).collect();

    // documents
    struct Span {
        start: usize,
        end: usize,
        ty: String,
        uuid: String,
    }
    let mut spans: Vec<Span> = vec![];
    for (i, l) in lines.iter().enumerate() {
        if !l.starts_with("{\"type\":\"DOCHEAD\"") {
            continue;
        }
        if let Some(last) = spans.last_mut() {
            last.end = i;
        }
        let body = split_line(l).and_then(|(_, b)| serde_json::from_str::<Value>(b).ok()).unwrap_or(Value::Null);
        spans.push(Span { start: i, end: lines.len(), ty: s(&body, "docType").into(), uuid: s(&body, "uuid").into() });
    }
    let mut best: Option<(usize, usize)> = None;
    for (k, sp) in spans.iter().enumerate() {
        if sp.ty == "PCB" {
            let n = lines[sp.start..sp.end].iter().filter(|l| l.starts_with("{\"type\":\"COMPONENT\"")).count();
            if best.map_or(true, |b| n >= b.1) {
                best = Some((k, n));
            }
        }
    }
    let (bk, _) = best.ok_or("the project has no PCB document")?;
    let pcb = &spans[bk];
    let recs = parse_range(&lines, pcb.start, pcb.end);
    let fps: HashMap<String, Vec<Rec>> = spans.iter().filter(|d| d.ty == "FOOTPRINT").map(|d| (d.uuid.clone(), parse_range(&lines, d.start, d.end))).collect();

    let mut notes: Vec<String> = vec![];
    let npcb = spans.iter().filter(|d| d.ty == "PCB").count();
    if npcb > 1 {
        notes.push(format!("the project has {npcb} PCBs; the one with the most parts is used"));
    }

    // nets
    let mut net_ids: HashMap<String, usize> = HashMap::new();
    let mut nets: Vec<Net> = vec![];
    let mut net = |name: &str| -> i32 {
        if name.is_empty() {
            return -1;
        }
        if let Some(&i) = net_ids.get(name) {
            return i as i32;
        }
        net_ids.insert(name.to_string(), nets.len());
        nets.push(Net { name: name.to_string(), pins: vec![], kind: classify_net(name) });
        (nets.len() - 1) as i32
    };
    let mut pad_net: HashMap<(String, String), String> = HashMap::new();
    for r in recs.iter().filter(|r| r.ty == "PAD_NET") {
        if let Ok(v) = serde_json::from_str::<Value>(&r.id) {
            if let (Some(c), Some(p)) = (v.get(1).and_then(|x| x.as_str()), v.get(3).and_then(|x| x.as_str())) {
                pad_net.insert((c.to_string(), p.to_string()), s(&r.body, "padNet").to_string());
            }
        }
    }
    let mut attr: HashMap<(&str, &str), &Rec> = HashMap::new();
    for r in recs.iter().filter(|r| r.ty == "ATTR") {
        attr.insert((s(&r.body, "parentId"), s(&r.body, "key")), r);
    }

    let mut parts: Vec<Part> = vec![];
    let mut labels: Vec<Vec<usize>> = vec![];
    let mut missing: Vec<String> = vec![];
    for comp in recs.iter().filter(|r| r.ty == "COMPONENT") {
        let uuid = attr.get(&(comp.id.as_str(), "Footprint")).map(|a| s(&a.body, "value")).unwrap_or("");
        let des = attr.get(&(comp.id.as_str(), "Designator")).map(|a| s(&a.body, "value").to_string()).unwrap_or_default();
        let Some(frecs) = fps.get(uuid) else {
            missing.push(if des.is_empty() { comp.id.clone() } else { des });
            continue;
        };
        let angle = f(&comp.body, "angle");
        let bottom = int(&comp.body, "layerId") == 2;
        let mut pads: Vec<PadG> = vec![];
        let mut body_pts: Vec<P> = vec![];
        let mut tht = false;
        for r in frecs {
            match r.ty.as_str() {
                "PAD" => {
                    let layer = int(&r.body, "layerId");
                    let hole = present(&r.body, "hole");
                    let dp = present(&r.body, "defaultPad");
                    let (w, h) = match (dp, hole) {
                        (Some(d), _) => (f(d, "width"), f(d, "height")),
                        (None, Some(hl)) => (f(hl, "width"), f(hl, "height")),
                        _ => continue,
                    };
                    if w <= 0.0 || h <= 0.0 {
                        continue;
                    }
                    if layer != 1 && layer != 2 && layer != 12 {
                        continue;
                    }
                    let c = off((f(&r.body, "centerX"), f(&r.body, "centerY")), angle, bottom);
                    let rot = (if bottom { -f(&r.body, "padAngle") } else { f(&r.body, "padAngle") }) + angle;
                    let (co, si) = (rot.to_radians().cos().abs(), rot.to_radians().sin().abs());
                    let is_tht = hole.is_some() || layer == 12;
                    tht |= is_tht;
                    let netname = pad_net.get(&(comp.id.clone(), r.id.clone())).cloned().unwrap_or_default();
                    pads.push(PadG { num: s(&r.body, "num").to_string(), net: net(&netname), dx: c.0 * U, dy: c.1 * U, hx: (w * co + h * si) / 2.0 * U, hy: (w * si + h * co) / 2.0 * U, tht: is_tht });
                }
                "FILL" | "POLY" if int(&r.body, "layerId") == 48 => {
                    if let Some(p) = r.body.get("path") {
                        for c in pro_path_contours(p) {
                            for q in c {
                                let o = off(q, angle, bottom);
                                body_pts.push((o.0 * U, o.1 * U));
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        for p in &pads {
            body_pts.push((p.dx - p.hx, p.dy - p.hy));
            body_pts.push((p.dx + p.hx, p.dy + p.hy));
        }
        if pads.is_empty() {
            missing.push(des);
            continue;
        }
        let package = frecs.iter().find(|r| r.ty == "META").map(|r| s(&r.body, "title").to_string()).unwrap_or_default();
        let lab: Vec<usize> = recs.iter().filter(|r| r.ty == "ATTR" && s(&r.body, "parentId") == comp.id && present(&r.body, "x").is_some() && present(&r.body, "y").is_some()).map(|r| r.line).collect();
        parts.push(Part {
            refdes: if des.is_empty() { comp.id.clone() } else { des },
            package,
            x: f(&comp.body, "x") * U,
            y: f(&comp.body, "y") * U,
            bottom,
            locked: comp.body.get("locked").and_then(|v| v.as_bool()).unwrap_or(false),
            movable: true,
            rot_ok: true,
            flip_ok: !tht,
            tht,
            pads,
            body: bbox_of(&body_pts),
            src: Some(comp.line),
        });
        labels.push(lab);
    }
    if !missing.is_empty() {
        notes.push(format!("{} part(s) without a footprint in the project are ignored: {}", missing.len(), missing.join(", ")));
    }

    // loose pads and outline, routing
    let mut loops: Vec<Vec<P>> = vec![];
    let mut segs: Vec<(P, P)> = vec![];
    let mut routing: Vec<usize> = vec![];
    let mut n_routing = 0;
    let mut loose = 0;
    for r in &recs {
        match r.ty.as_str() {
            "POLY" if int(&r.body, "layerId") == 11 => {
                if let Some(p) = r.body.get("path") {
                    for c in pro_path_contours(p) {
                        loops.push(c.into_iter().map(|q| (q.0 * U, q.1 * U)).collect());
                    }
                }
            }
            "LINE" if int(&r.body, "layerId") == 11 => {
                segs.push(((f(&r.body, "startX") * U, f(&r.body, "startY") * U), (f(&r.body, "endX") * U, f(&r.body, "endY") * U)));
            }
            "LINE" | "ARC" if matches!(int(&r.body, "layerId"), 1 | 2 | 15..=46) => {
                routing.push(r.line);
                n_routing += 1;
            }
            "VIA" => {
                routing.push(r.line);
                n_routing += 1;
            }
            "POURED" => routing.push(r.line),
            "PAD" => {
                let hole = present(&r.body, "hole");
                let dp = present(&r.body, "defaultPad");
                let (w, h) = match (dp, hole) {
                    (Some(d), _) => (f(d, "width"), f(d, "height")),
                    (None, Some(hl)) => (f(hl, "width"), f(hl, "height")),
                    _ => continue,
                };
                let (c, rot) = ((f(&r.body, "centerX") * U, f(&r.body, "centerY") * U), f(&r.body, "padAngle"));
                let (co, si) = (rot.to_radians().cos().abs(), rot.to_radians().sin().abs());
                let (hx, hy) = ((w * co + h * si) / 2.0 * U, (w * si + h * co) / 2.0 * U);
                let pad = PadG { num: s(&r.body, "num").to_string(), net: net(s(&r.body, "netName")), dx: 0.0, dy: 0.0, hx, hy, tht: true };
                parts.push(Part {
                    refdes: format!("TP{loose}"),
                    package: "loose pad".into(),
                    x: c.0,
                    y: c.1,
                    bottom: false,
                    locked: true,
                    movable: false,
                    rot_ok: false,
                    flip_ok: false,
                    tht: true,
                    pads: vec![pad],
                    body: [-hx, -hy, hx, hy],
                    src: None,
                });
                labels.push(vec![]);
                loose += 1;
            }
            _ => {}
        }
    }
    loops.extend(chain_segments(&segs));
    let (outline, cutouts) = split_outline(loops);
    for (pi, p) in parts.iter().enumerate() {
        for (k, pad) in p.pads.iter().enumerate() {
            if pad.net >= 0 {
                nets[pad.net as usize].pins.push((pi, k));
            }
        }
    }
    if parts.is_empty() {
        return Err("no components found in this PCB".into());
    }
    notes.push("EasyEDA Pro: parts can be moved, turned in 90 degree steps and put on the other side (mirroring convention taken from the autorouter's loader, check one flipped part in EasyEDA)".into());
    let title = recs.iter().find(|r| r.ty == "META").map(|r| s(&r.body, "title").to_string()).unwrap_or_default();
    let board = Board { fmt: Fmt::Pro, title, parts, nets, outline, cutouts, notes, routing: n_routing };
    Ok((ProDoc { bytes: bytes.to_vec(), epru_name: epru, lines, labels, routing }, board))
}

fn rewrite(line: &str, edit: &dyn Fn(&mut Value)) -> Option<String> {
    let (h, b) = split_line(line)?;
    let mut v: Value = serde_json::from_str(b).ok()?;
    edit(&mut v);
    Some(format!("{}||{}|", h, serde_json::to_string(&v).ok()?))
}

impl ProDoc {
    pub fn save(&self, board: &Board, pls: &[Pl], clear_routing: bool) -> Result<Vec<u8>, String> {
        let mut lines = self.lines.clone();
        for (pi, part) in board.parts.iter().enumerate() {
            let Some(li) = part.src else { continue };
            let pl = pls[pi];
            if pl == Pl::of(part) || part.fixed() {
                continue;
            }
            if pl.d != 0 && !part.rot_ok {
                return Err(format!("{}: cannot be rotated", part.refdes));
            }
            if pl.flip && !part.flip_ok {
                return Err(format!("{}: cannot be put on the other side", part.refdes));
            }
            let (ox, oy) = (part.x / U, part.y / U);
            let (nx, ny) = (pl.x / U, pl.y / U);
            let dq = pl.d as f64 * 90.0;
            let (d, flip) = (pl.d, pl.flip);
            // the same transform for the part and its labels, in mil
            let tf = move |x: f64, y: f64| -> P {
                let o = xf_off(x - ox, y - oy, d, flip);
                (nx + o.0, ny + o.1)
            };
            let old_line = lines[li].clone();
            let new_line = rewrite(&old_line, &|v| {
                let a = f(v, "angle");
                let na = if flip { dq - a } else { a + dq }.rem_euclid(360.0);
                v["x"] = json!(round4(nx));
                v["y"] = json!(round4(ny));
                v["angle"] = json!(round4(na));
                if flip {
                    let l = int(v, "layerId");
                    v["layerId"] = json!(3 - l);
                }
            })
            .ok_or_else(|| format!("{}: cannot rewrite its COMPONENT line", part.refdes))?;
            lines[li] = new_line;
            for &ai in &self.labels[pi] {
                if let Some(nl) = rewrite(&lines[ai].clone(), &|v| {
                    let q = tf(f(v, "x"), f(v, "y"));
                    v["x"] = json!(round4(q.0));
                    v["y"] = json!(round4(q.1));
                    let a = f(v, "angle");
                    v["angle"] = json!(round4((if flip { -a } else { a } + dq).rem_euclid(360.0)));
                    if flip {
                        let l = int(v, "layerId");
                        if l == 3 || l == 4 {
                            v["layerId"] = json!(7 - l);
                        }
                    }
                }) {
                    lines[ai] = nl;
                }
            }
        }
        let drop: HashSet<usize> = if clear_routing { self.routing.iter().cloned().collect() } else { HashSet::new() };
        let kept: Vec<&str> = lines.iter().enumerate().filter(|(i, _)| !drop.contains(i)).map(|(_, l)| l.as_str()).collect();
        self.pack(&kept.join("\n"))
    }

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

fn round4(v: f64) -> f64 {
    (v * 10000.0).round() / 10000.0
}
