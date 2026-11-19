//! Write a routed board back into the EasyEDA document it was loaded from: the Standard source
//! JSON or an EasyEDA Pro project (`.epro2`). Tracks with their real widths, vias and copper areas
//! are appended to the document; everything else in it is left exactly as it was.

use crate::engine::Router;
use crate::geom::{dist, P};
use dsn::easyeda::{EasyedaDoc, PX_MM};
use dsn::easyeda_pro::{NewObject, ProDoc};
use serde_json::Value;

#[derive(Default, Debug, Clone, Copy)]
pub struct JsonStats {
    pub tracks: usize,
    pub vias: usize,
    pub areas: usize,
    /// 0 ohm jumper resistors on the board (single sided mode)
    pub jumpers: usize,
    /// tracks and vias that were on the board before routing and are left out of the saved file
    /// (they are kept with `include_existing`)
    pub omitted: usize,
}

fn n4(v: f64) -> String {
    let s = format!("{:.4}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" || s.is_empty() {
        "0".into()
    } else {
        s
    }
}

fn path(doc: &EasyedaDoc, ring: &[P]) -> String {
    let mut s = String::new();
    for (i, p) in ring.iter().enumerate() {
        let (x, y) = doc.to_json_xy(*p);
        if i == 0 {
            s += &format!("M {} {} L", n4(x), n4(y));
        } else {
            s += &format!(" {} {}", n4(x), n4(y));
        }
    }
    s + " Z "
}

pub fn write_json(r: &Router, doc: &EasyedaDoc) -> (String, JsonStats) {
    let mut root: Value = doc.root.clone();
    doc.apply_part_moves(&mut root, &r.part_moves);
    let mut used = doc.ids.clone();
    let mut next = 900_000u64;
    let mut new_id = move || loop {
        next += 1;
        let id = format!("gge{next}");
        if used.insert(id.clone()) {
            return id;
        }
    };
    let mut add: Vec<String> = vec![];
    let mut stats = JsonStats::default();
    let unit = if r.unit_mm > 0.0 { r.unit_mm } else { PX_MM };
    for n in &r.nets {
        for w in n.r.wires.iter().filter(|w| !w.fill) {
            let mut pts: Vec<P> = vec![];
            for &p in &w.pts {
                if pts.last().is_none_or(|&q| dist(q, p) > 1e-6) {
                    pts.push(p);
                }
            }
            if pts.len() < 2 {
                continue;
            }
            let coords: Vec<String> = pts
                .iter()
                .map(|&p| {
                    let (x, y) = doc.to_json_xy(p);
                    format!("{} {}", n4(x), n4(y))
                })
                .collect();
            add.push(format!(
                "TRACK~{}~{}~{}~{}~{}~0",
                n4(w.width),
                r.layer_names[w.layer],
                n.name,
                coords.join(" "),
                new_id()
            ));
            stats.tracks += 1;
        }
        for &v in &n.r.vias {
            let (x, y) = doc.to_json_xy(v);
            let dia = 2.0 * n.via_r;
            let mut drill = (r.cfg.rules.min_via_drill_mm / unit).max(0.0);
            // the ring around the hole stays at least 0.1 mm (JLCPCB: diameter >= hole + 0.1)
            if drill > dia - 0.1 / unit {
                drill = dia - 0.1 / unit;
            }
            if drill <= 0.0 {
                drill = dia * 0.5;
            }
            add.push(format!("VIA~{}~{}~{}~{}~{}~{}~0", n4(x), n4(y), n4(dia), n.name, n4(drill / 2.0), new_id()));
            stats.vias += 1;
        }
    }
    // 0 ohm jumper resistors (single sided boards): a footprint with its two pads on the net
    for (k, (ni, j)) in r.jumper_list().into_iter().enumerate() {
        let net = &r.nets[ni].name;
        let layer = &r.layer_names[j.layer];
        let (cx, cy) = doc.to_json_xy(j.centre());
        let vertical = (j.p[0].0 - j.p[1].0).abs() < (j.p[0].1 - j.p[1].1).abs();
        let rot = if vertical { 90 } else { 0 };
        let pads = j.pads(ni as i32, 0.0);
        let (lcsc, mpn) = crate::engine::jumper_part(&j.size).unwrap_or(("", ""));
        let uid = format!("{:032x}", 0x6a756d7065720000u128 + k as u128);
        let mut s = format!(
            "LIB~{}~{}~package`R{}`Value`0Ω`Supplier`LCSC`Supplier Part`{}`Manufacturer Part`{}`JLCPCB Part Class`Basic Part`Contributor`lcsc`~~~{}~1~{}~1~0~~yes~~",
            n4(cx),
            n4(cy),
            j.size,
            lcsc,
            mpn,
            new_id(),
            uid
        );
        for (num, (pad, c)) in pads.iter().zip(j.p.iter()).enumerate() {
            let (px, py) = doc.to_json_xy(*c);
            let pts: Vec<String> = match &pad.geo {
                crate::geom::Geo::Poly(v) => v
                    .iter()
                    .map(|&q| {
                        let (x, y) = doc.to_json_xy(q);
                        format!("{} {}", n4(x), n4(y))
                    })
                    .collect(),
                _ => vec![],
            };
            s += &format!(
                "#@$PAD~RECT~{}~{}~{}~{}~{}~{}~{}~0~{}~{}~{}~0~~Y~0~-393.7008~0.2000~{},{}",
                n4(px),
                n4(py),
                n4(j.along),
                n4(j.across),
                layer,
                net,
                num + 1,
                pts.join(" "),
                rot,
                new_id(),
                n4(px),
                n4(py)
            );
        }
        let _ = k;
        add.push(s);
        stats.jumpers += 1;
    }
    for (net, layer, ring) in &r.fill_polys {
        let name = &r.nets[*net].name;
        let lname = &r.layer_names[*layer];
        // a net that already has an area on the layer gets no new one, except the fill of the
        // empty space (same net, so it merges with the existing area)
        let is_fill = r.fill_net_index() == Some(*net) && r.fill_applies(*layer, *net);
        if doc.areas.contains(&(name.clone(), lname.clone())) && !is_fill {
            continue;
        }
        let outline = path(doc, ring);
        let id = new_id();
        add.push(format!(
            "COPPERAREA~1~{}~{}~{}~{}~solid~{}~spoke~none~[[\"{}\"]]~0~{}~2~1~2~1~yes~0",
            lname,
            name,
            outline,
            n4(r.nets[*net].clr.max(1.0)),
            id,
            outline,
            id
        ));
        stats.areas += 1;
    }
    if let Some(shapes) = root.get_mut("shape").and_then(|s| s.as_array_mut()) {
        if !r.cfg.include_existing && !doc.existing.is_empty() {
            let omit: std::collections::HashSet<usize> = doc.existing.iter().copied().collect();
            let mut i = 0;
            shapes.retain(|_| {
                i += 1;
                !omit.contains(&(i - 1))
            });
            stats.omitted = omit.len();
        }
        shapes.extend(add.into_iter().map(Value::String));
    }
    (serde_json::to_string(&root).unwrap_or_default(), stats)
}

/// The routed board in the format it was loaded from: the Standard JSON text, or the Pro project
/// archive with the new objects added to its PCB.
pub fn write_output(r: &Router, doc: &EasyedaDoc) -> Result<(Vec<u8>, JsonStats), String> {
    match &doc.pro {
        Some(p) => write_epro2(r, doc, p),
        None => {
            let (text, st) = write_json(r, doc);
            Ok((text.into_bytes(), st))
        }
    }
}

/// New tracks, vias and copper pours as objects of an EasyEDA Pro PCB.
pub fn write_epro2(r: &Router, doc: &EasyedaDoc, pro: &ProDoc) -> Result<(Vec<u8>, JsonStats), String> {
    let unit = if r.unit_mm > 0.0 { r.unit_mm } else { PX_MM };
    let mut add: Vec<NewObject> = vec![];
    let mut stats = JsonStats::default();
    for n in &r.nets {
        for w in n.r.wires.iter().filter(|w| !w.fill) {
            let mut pts: Vec<P> = vec![];
            for &p in &w.pts {
                if pts.last().is_none_or(|&q| dist(q, p) > 1e-6) {
                    pts.push(p);
                }
            }
            if pts.len() < 2 {
                continue;
            }
            stats.tracks += pts.len() - 1;
            add.push(NewObject::Track {
                net: n.name.clone(),
                layer: r.layer_names[w.layer].clone(),
                width: w.width,
                pts: pts.iter().map(|&p| doc.to_json_xy(p)).collect(),
            });
        }
        for &v in &n.r.vias {
            let dia = 2.0 * n.via_r;
            // at least the hole of the project's via rule
            let mut drill = (r.cfg.rules.min_via_drill_mm / unit).max(pro.via_hole_mil * 0.1);
            if drill > dia - 0.1 / unit {
                drill = dia - 0.1 / unit;
            }
            if drill <= 0.0 {
                drill = dia * 0.5;
            }
            add.push(NewObject::Via { net: n.name.clone(), at: doc.to_json_xy(v), dia, hole: drill });
            stats.vias += 1;
        }
    }
    for (net, layer, ring) in &r.fill_polys {
        let name = &r.nets[*net].name;
        let lname = &r.layer_names[*layer];
        let is_fill = r.fill_net_index() == Some(*net) && r.fill_applies(*layer, *net);
        if doc.areas.contains(&(name.clone(), lname.clone())) && !is_fill {
            continue;
        }
        add.push(NewObject::Pour { net: name.clone(), layer: lname.clone(), ring: ring.iter().map(|&p| doc.to_json_xy(p)).collect() });
        stats.areas += 1;
    }
    // Jumper resistors are parts, which this writer cannot add to a Pro project: the tracks run
    // to their pad places and the parts come from the placement list (`jumper_list_json`).
    stats.jumpers = r.jumper_list().len();
    let omit: &[usize] = if r.cfg.include_existing { &[] } else { &doc.existing };
    stats.omitted = omit.len();
    let (bytes, _) = pro.save(&add, &r.part_moves, omit)?;
    Ok((bytes, stats))
}

/// Placement list of the jumper resistors for the CAD: one entry per part with the net, the size
/// (0603, 0805, 1206), the centre in mm and mil (y up, as EasyEDA Pro shows it), the angle
/// (0 = pads left and right, 90 = pads above and below), the layer and the pad size in mm.
pub fn jumper_list_json(r: &Router, doc: &EasyedaDoc) -> Option<String> {
    let list = r.jumper_list();
    if list.is_empty() {
        return None;
    }
    let unit = if r.unit_mm > 0.0 { r.unit_mm } else { PX_MM };
    let items: Vec<Value> = list
        .iter()
        .enumerate()
        .map(|(k, (ni, j))| {
            let (jx, jy) = doc.to_json_xy(j.centre());
            let (x_mm, y_mm) = (jx * PX_MM, -jy * PX_MM);
            let vertical = (j.p[0].0 - j.p[1].0).abs() < (j.p[0].1 - j.p[1].1).abs();
            let (lcsc, mpn) = crate::engine::jumper_part(&j.size).unwrap_or(("", ""));
            serde_json::json!({
                "designator": format!("RJ{}", k + 1),
                "net": r.nets[*ni].name,
                "size": j.size,
                "footprint": format!("R{}", j.size),
                "value": "0R",
                "lcsc": lcsc,
                "manufacturer_part": mpn,
                "jlcpcb_class": "Basic Part",
                "layer": r.layer_names[j.layer],
                "x_mm": (x_mm * 1e4).round() / 1e4,
                "y_mm": (y_mm * 1e4).round() / 1e4,
                "x_mil": (x_mm / 0.0254 * 1e3).round() / 1e3,
                "y_mil": (y_mm / 0.0254 * 1e3).round() / 1e3,
                "angle": if vertical { 90 } else { 0 },
                "pad_length_mm": (j.along * unit * 1e3).round() / 1e3,
                "pad_width_mm": (j.across * unit * 1e3).round() / 1e3,
                "pad_pitch_mm": (dist(j.p[0], j.p[1]) * unit * 1e3).round() / 1e3,
            })
        })
        .collect();
    serde_json::to_string_pretty(&serde_json::json!({ "jumpers": items })).ok()
}
