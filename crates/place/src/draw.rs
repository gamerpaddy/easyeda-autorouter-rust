//! Drawing primitives shared by the GUI and the SVG export. World units: mm, y up.
use crate::engine::net_weight;
use crate::model::*;

pub type Rgba = [u8; 4];

pub enum Prim {
    Poly { pts: Vec<P>, fill: Option<Rgba>, stroke: Option<(Rgba, f64)> },
    Line { a: P, b: P, col: Rgba, w: f64 },
    Text { at: P, s: String, size: f64, col: Rgba },
}

pub struct SceneOpts<'a> {
    pub rats: bool,
    pub gnd: bool,
    pub labels: bool,
    pub select: Option<usize>,
    pub caps: &'a [(usize, usize, f64)],
    pub region: &'a [P],
    /// lock state per part (empty: as in the file)
    pub locked: &'a [bool],
    /// net names on the pads
    pub pin_nets: bool,
}

pub const RED: Rgba = [255, 45, 45, 255];
pub const FRONT: Rgba = [222, 170, 60, 235];
pub const BACK: Rgba = [80, 140, 240, 235];
pub const OUTLINE: Rgba = [255, 214, 10, 255];

fn rect_poly(r: [f64; 4]) -> Vec<P> {
    vec![(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
}

pub fn scene(b: &Board, pl: &[Pl], o: &SceneOpts) -> Vec<Prim> {
    let mut v: Vec<Prim> = vec![];
    if !o.region.is_empty() {
        v.push(Prim::Poly { pts: o.region.to_vec(), fill: None, stroke: Some(([120, 120, 130, 255], 0.12)) });
    }
    if let Some(out) = &b.outline {
        v.push(Prim::Poly { pts: out.clone(), fill: None, stroke: Some((OUTLINE, 0.2)) });
    }
    for c in &b.cutouts {
        v.push(Prim::Poly { pts: c.clone(), fill: Some([255, 214, 10, 30]), stroke: Some((OUTLINE, 0.15)) });
    }
    // parts: back side first so the front stays on top
    let mut order: Vec<usize> = (0..b.parts.len()).collect();
    let is_fixed = |i: usize| if o.locked.is_empty() { b.parts[i].fixed() } else { o.locked[i] || !b.parts[i].movable };
    order.sort_by_key(|&i| (!(b.parts[i].bottom ^ pl[i].flip), is_fixed(i)));
    for &i in &order {
        let p = &b.parts[i];
        let l = pl[i];
        let back = p.bottom ^ l.flip;
        let fixed = is_fixed(i);
        let r = p.rect(&l);
        let sel = o.select == Some(i);
        let (fill, stroke, w) = if fixed {
            (Some([255, 40, 40, 70]), RED, 0.2)
        } else if back {
            (Some([80, 140, 240, 28]), [110, 160, 255, 255], 0.1)
        } else {
            (Some([90, 200, 140, 24]), [120, 210, 160, 255], 0.1)
        };
        let stroke = if sel { [255, 255, 255, 255] } else { stroke };
        v.push(Prim::Poly { pts: rect_poly(r), fill, stroke: Some((stroke, if sel { 0.25 } else { w })) });
        for k in 0..p.pads.len() {
            let pb = p.pad_box(k, &l);
            let col = if fixed { [255, 110, 110, 235] } else if p.pads[k].tht { [190, 200, 190, 235] } else if back { BACK } else { FRONT };
            v.push(Prim::Poly { pts: rect_poly(pb), fill: Some(col), stroke: None });
            if o.pin_nets && p.pads[k].net >= 0 {
                let (w, h) = (pb[2] - pb[0], pb[3] - pb[1]);
                let name = &b.nets[p.pads[k].net as usize].name;
                let size = (w.min(h) * 0.75).min(w.max(h) / (name.chars().count().max(1) as f64 * 0.6)).max(w.min(h) * 0.3);
                v.push(Prim::Text { at: ((pb[0] + pb[2]) / 2.0, (pb[1] + pb[3]) / 2.0), s: name.clone(), size, col: [10, 10, 14, 255] });
            }
        }
        if o.labels {
            let size = ((r[2] - r[0]).min((r[3] - r[1]) * 2.5) * 0.35).clamp(0.25, 1.6);
            v.push(Prim::Text { at: ((r[0] + r[2]) / 2.0, (r[1] + r[3]) / 2.0), s: p.refdes.clone(), size, col: if fixed { [255, 200, 200, 255] } else { [235, 235, 235, 255] } });
        }
    }
    if o.rats {
        for (ni, net) in b.nets.iter().enumerate() {
            let w = net_weight(net);
            if (w == 0.0 && !o.gnd) || net.pins.len() < 2 {
                continue;
            }
            let hot = o.select.map_or(false, |s| net.pins.iter().any(|q| q.0 == s));
            let pts: Vec<P> = net.pins.iter().map(|&(p, k)| b.parts[p].pad_xy(k, &pl[p])).collect();
            let col = if hot { [255, 255, 255, 230] } else if net.kind == NetKind::Power { [255, 130, 0, 110] } else if net.kind == NetKind::Gnd { [120, 120, 120, 60] } else { [0, 200, 255, 100] };
            let wd = if hot { 0.1 } else { 0.05 };
            let _ = ni;
            if pts.len() <= 40 {
                // minimum spanning tree
                let n = pts.len();
                let mut inn = vec![false; n];
                let mut dist = vec![f64::MAX; n];
                let mut from = vec![0usize; n];
                dist[0] = 0.0;
                for _ in 0..n {
                    let mut bi = usize::MAX;
                    for k in 0..n {
                        if !inn[k] && (bi == usize::MAX || dist[k] < dist[bi]) {
                            bi = k;
                        }
                    }
                    inn[bi] = true;
                    if bi != 0 || dist[bi] != 0.0 {
                        if dist[bi] != 0.0 || bi != 0 {
                            v.push(Prim::Line { a: pts[from[bi]], b: pts[bi], col, w: wd });
                        }
                    }
                    for k in 0..n {
                        if !inn[k] {
                            let d = (pts[k].0 - pts[bi].0).powi(2) + (pts[k].1 - pts[bi].1).powi(2);
                            if d < dist[k] {
                                dist[k] = d;
                                from[k] = bi;
                            }
                        }
                    }
                }
            } else {
                let c = (pts.iter().map(|p| p.0).sum::<f64>() / pts.len() as f64, pts.iter().map(|p| p.1).sum::<f64>() / pts.len() as f64);
                for p in pts {
                    v.push(Prim::Line { a: p, b: c, col, w: wd });
                }
            }
        }
    }
    for &(cap, host, _) in o.caps {
        let (cp, hp) = (&b.parts[cap], &b.parts[host]);
        // nearest pad pair on the shared power net
        let mut best: Option<(f64, P, P)> = None;
        for i in 0..cp.pads.len() {
            for k in 0..hp.pads.len() {
                if cp.pads[i].net >= 0 && cp.pads[i].net == hp.pads[k].net && b.nets[cp.pads[i].net as usize].kind == NetKind::Power {
                    let (a, c) = (cp.pad_xy(i, &pl[cap]), hp.pad_xy(k, &pl[host]));
                    let d = (a.0 - c.0).powi(2) + (a.1 - c.1).powi(2);
                    if best.map_or(true, |x| d < x.0) {
                        best = Some((d, a, c));
                    }
                }
            }
        }
        if let Some((_, a, c)) = best {
            v.push(Prim::Line { a, b: c, col: [255, 0, 210, 230], w: 0.12 });
        }
    }
    v
}

pub fn bounds(b: &Board, pl: &[Pl], region: &[P]) -> [f64; 4] {
    let mut pts: Vec<P> = vec![];
    for (i, p) in b.parts.iter().enumerate() {
        let r = p.rect(&pl[i]);
        pts.push((r[0], r[1]));
        pts.push((r[2], r[3]));
    }
    if let Some(o) = &b.outline {
        pts.extend(o.iter().cloned());
    }
    pts.extend(region.iter().cloned());
    crate::geom::bbox_of(&pts)
}

pub fn to_svg(b: &Board, pl: &[Pl], o: &SceneOpts, title: &str) -> String {
    let bb = bounds(b, pl, o.region);
    let m = 2.0;
    let (x0, y0, w, h) = (bb[0] - m, -bb[3] - m, bb[2] - bb[0] + 2.0 * m, bb[3] - bb[1] + 2.0 * m);
    let mut s = format!("<svg xmlns='http://www.w3.org/2000/svg' style='width:100vw;height:100vh;background:#121216' preserveAspectRatio='xMidYMid meet' viewBox='{:.3} {:.3} {:.3} {:.3}'>
<rect x='{:.3}' y='{:.3}' width='{:.3}' height='{:.3}' fill='#121216'/>
", x0, y0, w, h, x0, y0, w, h);
    let col = |c: Rgba| format!("rgb({},{},{})", c[0], c[1], c[2]);
    let op = |c: Rgba| format!("{:.2}", c[3] as f64 / 255.0);
    for p in scene(b, pl, o) {
        match p {
            Prim::Poly { pts, fill, stroke } => {
                let d: Vec<String> = pts.iter().map(|q| format!("{:.3},{:.3}", q.0, -q.1)).collect();
                let f = fill.map_or("fill='none'".to_string(), |c| format!("fill='{}' fill-opacity='{}'", col(c), op(c)));
                let st = stroke.map_or(String::new(), |(c, w)| format!("stroke='{}' stroke-opacity='{}' stroke-width='{}'", col(c), op(c), w));
                s += &format!("<polygon points='{}' {} {}/>\n", d.join(" "), f, st);
            }
            Prim::Line { a, b, col: c, w } => {
                s += &format!("<line x1='{:.3}' y1='{:.3}' x2='{:.3}' y2='{:.3}' stroke='{}' stroke-opacity='{}' stroke-width='{}'/>\n", a.0, -a.1, b.0, -b.1, col(c), op(c), w);
            }
            Prim::Text { at, s: t, size, col: c } => {
                s += &format!("<text x='{:.3}' y='{:.3}' font-size='{:.3}' font-family='sans-serif' text-anchor='middle' dominant-baseline='middle' fill='{}'>{}</text>\n", at.0, -at.1, size, col(c), t.replace('&', "&amp;").replace('<', "&lt;"));
            }
        }
    }
    s += &format!("<text x='{:.3}' y='{:.3}' font-size='1.4' fill='#ccc' font-family='sans-serif'>{}</text>\n</svg>\n", x0 + 0.5, y0 + 1.6, title.replace('&', "&amp;").replace('<', "&lt;"));
    s
}
