//! EasyEDA Standard (`.json`): a `shape` array of `~` separated strings. A footprint is a `LIB`
//! shape: head + children joined by `#@$`, every child in absolute document coordinates (10 mil
//! units, y down).
use crate::geom::*;
use crate::model::*;
use serde_json::Value;
use std::collections::HashMap;

/// document units -> mm
pub const U: f64 = 0.254;

pub struct StdDoc {
    root: Value,
    /// indices of top-level tracks / vias on copper
    routing: Vec<usize>,
}

fn pf(s: &str) -> f64 {
    s.trim().parse().unwrap_or(0.0)
}

fn fmt(v: f64) -> String {
    let mut s = format!("{:.4}", v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}

/// "x y x y" / "x,y x,y" -> points
fn pairs(s: &str) -> Vec<P> {
    let v: Vec<f64> = s.split(|c: char| c.is_whitespace() || c == ',').filter(|t| !t.is_empty()).filter_map(|t| t.parse().ok()).collect();
    v.chunks_exact(2).map(|c| (c[0], c[1])).collect()
}

fn map_pairs(s: &str, f: &dyn Fn(f64, f64) -> P) -> String {
    let comma = s.contains(',') && !s.contains(' ');
    let pts = pairs(s);
    if comma {
        // "x,y" single pair or "x,y,x,y"
        return pts.iter().map(|p| { let q = f(p.0, p.1); format!("{},{}", fmt(q.0), fmt(q.1)) }).collect::<Vec<_>>().join(",");
    }
    pts.iter().map(|p| { let q = f(p.0, p.1); format!("{} {}", fmt(q.0), fmt(q.1)) }).collect::<Vec<_>>().join(" ")
}

/// Points of an SVG-like path (M L A Z C Q), arcs by their end points.
fn path_points(s: &str) -> Vec<P> {
    let mut out = vec![];
    for (cmd, nums) in path_tokens(s) {
        match cmd {
            'M' | 'L' => out.extend(nums.chunks_exact(2).map(|c| (c[0], c[1]))),
            'A' => {
                for c in nums.chunks_exact(7) {
                    out.push((c[5], c[6]));
                }
            }
            _ => {}
        }
    }
    out
}

fn path_tokens(s: &str) -> Vec<(char, Vec<f64>)> {
    let mut out: Vec<(char, Vec<f64>)> = vec![];
    let mut cur = String::new();
    let mut cmd = ' ';
    let mut nums: Vec<f64> = vec![];
    let flush = |cur: &mut String, nums: &mut Vec<f64>| {
        if !cur.is_empty() {
            if let Ok(v) = cur.parse::<f64>() {
                nums.push(v);
            }
            cur.clear();
        }
    };
    for ch in s.chars() {
        if ch.is_ascii_alphabetic() && ch != 'e' && ch != 'E' {
            flush(&mut cur, &mut nums);
            if cmd != ' ' {
                out.push((cmd, std::mem::take(&mut nums)));
            }
            cmd = ch.to_ascii_uppercase();
        } else if ch == ',' || ch.is_whitespace() {
            flush(&mut cur, &mut nums);
        } else if ch == '-' && !cur.is_empty() && !cur.ends_with('e') && !cur.ends_with('E') {
            flush(&mut cur, &mut nums);
            cur.push(ch);
        } else {
            cur.push(ch);
        }
    }
    flush(&mut cur, &mut nums);
    if cmd != ' ' {
        out.push((cmd, nums));
    }
    out
}

/// Re-write a path with its points mapped; `rot` is added to the x axis rotation of arcs.
fn map_path(s: &str, f: &dyn Fn(f64, f64) -> P, rot: f64) -> Option<String> {
    let comma = s.contains(',');
    let mut toks: Vec<String> = vec![];
    for (cmd, n) in path_tokens(s) {
        match cmd {
            'M' | 'L' => {
                for (k, c) in n.chunks_exact(2).enumerate() {
                    let q = f(c[0], c[1]);
                    let l = if cmd == 'M' && k == 0 { 'M' } else { 'L' };
                    toks.push(if comma { format!("{}{},{}", l, fmt(q.0), fmt(q.1)) } else { format!("{} {} {}", l, fmt(q.0), fmt(q.1)) });
                }
            }
            'A' => {
                for c in n.chunks_exact(7) {
                    let q = f(c[5], c[6]);
                    toks.push(format!("A {} {} {} {} {} {} {}", fmt(c[0]), fmt(c[1]), fmt((c[2] + rot).rem_euclid(360.0)), fmt(c[3]), fmt(c[4]), fmt(q.0), fmt(q.1)));
                }
            }
            'C' | 'Q' => {
                let k = if cmd == 'C' { 6 } else { 4 };
                for c in n.chunks_exact(k) {
                    let mut v = vec![];
                    for pr in c.chunks_exact(2) {
                        let q = f(pr[0], pr[1]);
                        v.push(format!("{} {}", fmt(q.0), fmt(q.1)));
                    }
                    toks.push(format!("{} {}", cmd, v.join(" ")));
                }
            }
            'Z' => toks.push("Z".into()),
            _ => return None,
        }
    }
    Some(toks.join(if comma { "" } else { " " }))
}

fn attr(attrs: &str, key: &str) -> String {
    let t: Vec<&str> = attrs.split('`').collect();
    let mut i = 0;
    while i + 1 < t.len() {
        if t[i] == key {
            return t[i + 1].to_string();
        }
        i += 2;
    }
    String::new()
}

struct NetMap {
    ids: HashMap<String, usize>,
    nets: Vec<Net>,
}

impl NetMap {
    fn get(&mut self, name: &str) -> i32 {
        if name.is_empty() {
            return -1;
        }
        if let Some(&i) = self.ids.get(name) {
            return i as i32;
        }
        let i = self.nets.len();
        self.ids.insert(name.to_string(), i);
        self.nets.push(Net { name: name.to_string(), pins: vec![], kind: classify_net(name) });
        i as i32
    }
}

fn pad_of(f: &[&str], ox: f64, oy: f64, nm: &mut NetMap) -> Option<PadG> {
    if f.len() < 12 {
        return None;
    }
    let (x, y, w, h) = (pf(f[2]), pf(f[3]), pf(f[4]), pf(f[5]));
    let pts = pairs(f[10]);
    let (cx, cy, hx, hy) = if pts.len() >= 3 {
        let b = bbox_of(&pts);
        ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, (b[2] - b[0]) / 2.0, (b[3] - b[1]) / 2.0)
    } else {
        let r = pf(f[11]).to_radians();
        let (c, s) = (r.cos().abs(), r.sin().abs());
        (x, y, (w * c + h * s) / 2.0, (w * s + h * c) / 2.0)
    };
    let layer = f[6];
    let tht = pf(f[9]) > 0.0 || layer == "11";
    Some(PadG { num: f[8].to_string(), net: nm.get(f[7]), dx: (cx - ox) * U, dy: -(cy - oy) * U, hx: hx * U, hy: hy * U, tht })
}

pub fn load(bytes: &[u8]) -> Result<(StdDoc, Board), String> {
    let text = std::str::from_utf8(bytes).map_err(|e| format!("not UTF-8: {e}"))?.trim_start_matches('\u{feff}');
    let root: Value = serde_json::from_str(text).map_err(|e| format!("not an EasyEDA Standard file: {e}"))?;
    let shapes = root.get("shape").and_then(|s| s.as_array()).ok_or("not an EasyEDA Standard PCB: no `shape` array")?;
    let mut nm = NetMap { ids: HashMap::new(), nets: vec![] };
    let mut parts: Vec<Part> = vec![];
    let mut notes: Vec<String> = vec![];
    let mut segs: Vec<(P, P)> = vec![];
    let mut loops: Vec<Vec<P>> = vec![];
    let mut routing: Vec<usize> = vec![];
    let mut loose = 0;
    for (si, sv) in shapes.iter().enumerate() {
        let Some(s) = sv.as_str() else { continue };
        let tag = s.split('~').next().unwrap_or("");
        match tag {
            "LIB" => {
                let mut it = s.split("#@$");
                let head: Vec<&str> = it.next().unwrap_or("").split('~').collect();
                if head.len() < 11 {
                    continue;
                }
                let (ox, oy) = (pf(head[1]), pf(head[2]));
                let mut part = Part {
                    refdes: String::new(),
                    package: attr(head[3], "package"),
                    x: ox * U,
                    y: -oy * U,
                    bottom: head[7] == "2",
                    locked: head[10] == "1",
                    movable: true,
                    rot_ok: true,
                    flip_ok: false,
                    tht: false,
                    pads: vec![],
                    body: [0.0; 4],
                    src: Some(si),
                };
                let mut body: Vec<P> = vec![];
                let mut bad: Vec<String> = vec![];
                for c in it {
                    let f: Vec<&str> = c.split('~').collect();
                    match f[0] {
                        "PAD" => {
                            if let Some(p) = pad_of(&f, ox, oy, &mut nm) {
                                part.tht |= p.tht;
                                body.push((p.dx - p.hx, p.dy - p.hy));
                                body.push((p.dx + p.hx, p.dy + p.hy));
                                part.pads.push(p);
                            }
                        }
                        "TRACK" => {
                            if f.len() > 4 && (f[2] == "99" || f[2] == "100") {
                                for q in pairs(f[4]) {
                                    body.push(((q.0 - ox) * U, -(q.1 - oy) * U));
                                }
                            }
                        }
                        "CIRCLE" => {
                            if f.len() > 5 && (f[5] == "99" || f[5] == "100") {
                                let (cx, cy, r) = (pf(f[1]), pf(f[2]), pf(f[3]));
                                for q in [(cx - r, cy - r), (cx + r, cy + r)] {
                                    body.push(((q.0 - ox) * U, -(q.1 - oy) * U));
                                }
                            }
                        }
                        "TEXT" => {
                            if f.len() > 10 && f[1] == "P" {
                                part.refdes = f[10].to_string();
                            }
                        }
                        "HOLE" => part.tht = true,
                        "ARC" => {}
                        "RECT" => part.rot_ok = false,
                        "SOLIDREGION" | "SVGNODE" => {}
                        other => bad.push(other.to_string()),
                    }
                }
                if !bad.is_empty() {
                    part.movable = false;
                    notes.push(format!("{}: unknown shape kinds ({}), part is kept in place", part.refdes, bad.join(",")));
                }
                if body.is_empty() {
                    body.push((-0.5, -0.5));
                    body.push((0.5, 0.5));
                }
                let b = bbox_of(&body);
                part.body = b;
                if part.refdes.is_empty() {
                    part.refdes = format!("?{}", parts.len());
                }
                parts.push(part);
            }
            "TRACK" => {
                let f: Vec<&str> = s.split('~').collect();
                if f.len() > 4 {
                    if f[2] == "10" {
                        let p = pairs(f[4]);
                        for w in p.windows(2) {
                            segs.push(((w[0].0 * U, -w[0].1 * U), (w[1].0 * U, -w[1].1 * U)));
                        }
                    } else if matches!(f[2], "1" | "2") || f[2].parse::<i32>().map(|v| (21..=52).contains(&v)).unwrap_or(false) {
                        routing.push(si);
                    }
                }
            }
            "VIA" => routing.push(si),
            "RECT" | "CIRCLE" | "ARC" | "SOLIDREGION" => {
                let f: Vec<&str> = s.split('~').collect();
                match tag {
                    "SOLIDREGION" if f.len() > 3 && f[1] == "10" => {
                        let p: Vec<P> = path_points(f[3]).into_iter().map(|q| (q.0 * U, -q.1 * U)).collect();
                        loops.push(p);
                    }
                    "CIRCLE" if f.len() > 5 && f[5] == "10" => {
                        let (cx, cy, r) = (pf(f[1]) * U, -pf(f[2]) * U, pf(f[3]) * U);
                        loops.push((0..32).map(|k| { let t = k as f64 * std::f64::consts::TAU / 32.0; (cx + r * t.cos(), cy + r * t.sin()) }).collect());
                    }
                    "RECT" if f.len() > 5 && f[5] == "10" => {
                        let (x, y, w, h) = (pf(f[1]) * U, -pf(f[2]) * U, pf(f[3]) * U, pf(f[4]) * U);
                        loops.push(vec![(x, y), (x + w, y), (x + w, y - h), (x, y - h)]);
                    }
                    _ => {}
                }
            }
            "PAD" | "HOLE" => {
                let f: Vec<&str> = s.split('~').collect();
                let mut part = Part {
                    refdes: format!("{}{}", if tag == "HOLE" { "H" } else { "TP" }, loose),
                    package: tag.to_string(),
                    x: 0.0,
                    y: 0.0,
                    bottom: false,
                    locked: true,
                    movable: false,
                    rot_ok: false,
                    flip_ok: false,
                    tht: true,
                    pads: vec![],
                    body: [0.0; 4],
                    src: None,
                };
                if tag == "HOLE" && f.len() > 3 {
                    let r = pf(f[3]) * U / 2.0;
                    part.x = pf(f[1]) * U;
                    part.y = -pf(f[2]) * U;
                    part.body = [-r, -r, r, r];
                } else if let Some(p) = pad_of(&f, 0.0, 0.0, &mut nm) {
                    part.x = 0.0;
                    part.y = 0.0;
                    part.body = [p.dx - p.hx, p.dy - p.hy, p.dx + p.hx, p.dy + p.hy];
                    part.pads.push(p);
                } else {
                    continue;
                }
                loose += 1;
                parts.push(part);
            }
            _ => {}
        }
    }
    loops.extend(chain_segments(&segs));
    let (outline, cutouts) = split_outline(loops);
    let mut nets = nm.nets;
    for (pi, p) in parts.iter().enumerate() {
        for (k, pad) in p.pads.iter().enumerate() {
            if pad.net >= 0 {
                nets[pad.net as usize].pins.push((pi, k));
            }
        }
    }
    let title = root.pointer("/head/title").and_then(|v| v.as_str()).unwrap_or("").to_string();
    let n_routing = routing.len();
    let mut board = Board { fmt: Fmt::Std, title, parts, nets, outline, cutouts, notes, routing: n_routing };
    if board.parts.is_empty() {
        return Err("no components found in this file".into());
    }
    board.notes.push("EasyEDA Standard: parts can be moved and turned in 90 degree steps; moving to the back side is not supported in this format".into());
    Ok((StdDoc { root, routing }, board))
}

struct T {
    ox: f64,
    oy: f64,
    nx: f64,
    ny: f64,
    th: f64,
}

impl T {
    fn p(&self, x: f64, y: f64) -> P {
        let (q0, q1) = (x - self.ox, y - self.oy);
        let a = self.th.to_radians();
        let (c, s) = (a.cos(), a.sin());
        // keep exact values for quarter turns
        let (c, s) = (if c.abs() < 1e-9 { 0.0 } else { c }, if s.abs() < 1e-9 { 0.0 } else { s });
        (self.nx + q0 * c - q1 * s, self.ny + q0 * s + q1 * c)
    }
}

fn set(f: &mut Vec<String>, i: usize, v: String) {
    if i < f.len() {
        f[i] = v;
    }
}

fn move_child(c: &str, t: &T) -> Result<String, String> {
    let mut f: Vec<String> = c.split('~').map(|s| s.to_string()).collect();
    let tag = f[0].clone();
    let xy = |f: &mut Vec<String>, ix: usize, iy: usize| {
        if iy < f.len() {
            let q = t.p(pf(&f[ix]), pf(&f[iy]));
            f[ix] = fmt(q.0);
            f[iy] = fmt(q.1);
        }
    };
    let pts = |f: &mut Vec<String>, i: usize| {
        if i < f.len() && !f[i].trim().is_empty() {
            f[i] = map_pairs(&f[i], &|x, y| t.p(x, y));
        }
    };
    let rot = |f: &mut Vec<String>, i: usize| {
        if i < f.len() {
            f[i] = fmt((pf(&f[i]) + t.th).rem_euclid(360.0));
        }
    };
    match tag.as_str() {
        "PAD" => {
            xy(&mut f, 2, 3);
            pts(&mut f, 10);
            rot(&mut f, 11);
            pts(&mut f, 14);
            if f.len() > 19 && f[19].contains(',') {
                f[19] = map_pairs(&f[19], &|x, y| t.p(x, y));
            }
        }
        "TRACK" => pts(&mut f, 4),
        "CIRCLE" | "HOLE" => xy(&mut f, 1, 2),
        "RECT" => {
            if t.th.abs() > 1e-9 {
                return Err("RECT cannot turn".into());
            }
            xy(&mut f, 1, 2)
        }
        "ARC" => {
            if f.len() > 4 {
                f[4] = map_path(&f[4], &|x, y| t.p(x, y), t.th).ok_or("unsupported arc path")?;
            }
            pts(&mut f, 5);
        }
        "TEXT" => {
            xy(&mut f, 2, 3);
            rot(&mut f, 5);
            if f.len() > 11 && f[11].starts_with('M') {
                f[11] = map_path(&f[11], &|x, y| t.p(x, y), 0.0).ok_or("unsupported text path")?;
            }
        }
        "SOLIDREGION" => {
            if f.len() > 3 {
                f[3] = map_path(&f[3], &|x, y| t.p(x, y), t.th).ok_or("unsupported region path")?;
            }
        }
        "SVGNODE" => {
            let body = c[tag.len() + 1..].to_string();
            let mut v: Value = serde_json::from_str(&body).map_err(|e| e.to_string())?;
            if let Some(a) = v.get_mut("attrs") {
                if let Some(o) = a.get("c_origin").and_then(|o| o.as_str()).map(|s| s.to_string()) {
                    let p = pairs(&o);
                    if let Some(p0) = p.first() {
                        let q = t.p(p0.0, p0.1);
                        a["c_origin"] = Value::String(format!("{},{}", fmt(q.0), fmt(q.1)));
                    }
                }
                if let Some(r) = a.get("c_rotation").and_then(|o| o.as_str()).map(|s| s.to_string()) {
                    let mut n: Vec<f64> = r.split(',').map(pf).collect();
                    if n.len() == 3 {
                        n[2] = (n[2] + t.th).rem_euclid(360.0);
                        a["c_rotation"] = Value::String(n.iter().map(|v| fmt(*v)).collect::<Vec<_>>().join(","));
                    }
                }
            }
            return Ok(format!("SVGNODE~{}", serde_json::to_string(&v).map_err(|e| e.to_string())?));
        }
        other => return Err(format!("cannot move {other}")),
    }
    Ok(f.join("~"))
}

impl StdDoc {
    pub fn save(&self, board: &Board, pls: &[Pl], clear_routing: bool) -> Result<Vec<u8>, String> {
        let mut root = self.root.clone();
        let shapes = root.get_mut("shape").and_then(|s| s.as_array_mut()).ok_or("no shape array")?;
        for (pi, part) in board.parts.iter().enumerate() {
            let Some(si) = part.src else { continue };
            let pl = pls[pi];
            if pl == Pl::of(part) || part.fixed() {
                continue;
            }
            if pl.flip {
                return Err(format!("{}: flipping is not supported in the Standard format", part.refdes));
            }
            if pl.d != 0 && !part.rot_ok {
                return Err(format!("{}: cannot be rotated", part.refdes));
            }
            let s = shapes[si].as_str().ok_or("bad shape")?.to_string();
            let mut it = s.split("#@$");
            let mut head: Vec<String> = it.next().unwrap().split('~').map(|s| s.to_string()).collect();
            let (ox, oy) = (pf(&head[1]), pf(&head[2]));
            let (nx, ny) = (pl.x / U, -pl.y / U);
            let t = T { ox, oy, nx, ny, th: -(pl.d as f64) * 90.0 };
            head[1] = fmt(nx);
            head[2] = fmt(ny);
            head[4] = fmt((pf(&head[4]) + t.th).rem_euclid(360.0));
            let mut out = vec![head.join("~")];
            for c in it {
                out.push(move_child(c, &t).map_err(|e| format!("{}: {e}", part.refdes))?);
            }
            shapes[si] = Value::String(out.join("#@$"));
        }
        if clear_routing {
            let drop: std::collections::HashSet<usize> = self.routing.iter().cloned().collect();
            let kept: Vec<Value> = shapes.iter().enumerate().filter(|(i, _)| !drop.contains(i)).map(|(_, v)| v.clone()).collect();
            *shapes = kept;
        }
        serde_json::to_vec(&root).map_err(|e| e.to_string())
    }

    /// Set the board outline to a rectangle (mm, y up) on layer 10.
    pub fn with_outline(&self, rect: [f64; 4]) -> StdDoc {
        let mut root = self.root.clone();
        if let Some(shapes) = root.get_mut("shape").and_then(|s| s.as_array_mut()) {
            let (x0, y0, x1, y1) = (rect[0] / U, -rect[3] / U, rect[2] / U, -rect[1] / U);
            let pts = format!("{} {} {} {} {} {} {} {} {} {}", fmt(x0), fmt(y0), fmt(x1), fmt(y0), fmt(x1), fmt(y1), fmt(x0), fmt(y1), fmt(x0), fmt(y0));
            shapes.push(Value::String(format!("TRACK~1~10~~{}~gge99001~0", pts)));
        }
        StdDoc { root, routing: self.routing.clone() }
    }
}
