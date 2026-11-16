//! Specctra SES writer.

use crate::engine::{Router, Wire};

use std::fmt::Write;

fn q(s: &str) -> String {
    if s.chars()
        .any(|c| c.is_whitespace() || c == '(' || c == ')' || c == '"')
    {
        format!("\"{}\"", s)
    } else {
        s.to_string()
    }
}

fn num(v: f64) -> String {
    let s = format!("{:.4}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    if s == "-0" {
        "0".into()
    } else {
        s
    }
}

pub fn write_ses(r: &Router, dsn_name: &str) -> String {
    let mut o = String::new();
    let coord = |v: f64| num(v * r.res);
    let _ = writeln!(o, "(session {}", q(dsn_name));
    let _ = writeln!(o, "  (base_design {})", q(dsn_name));
    let _ = writeln!(o, "  (routes");
    let _ = writeln!(o, "    (resolution {} {})", r.res_unit, num(r.res));
    let _ = writeln!(o, "    (parser");
    let _ = writeln!(o, "      (host_cad \"autorouter\")");
    let _ = writeln!(o, "    )");
    let _ = writeln!(o, "    (network_out");
    for (ni, n) in r.nets.iter().enumerate() {
        let pre_w: &[crate::engine::Wire] = if r.cfg.include_existing {
            &n.pre.wires
        } else {
            &[]
        };
        let pre_v: &[(f64, f64)] = if r.cfg.include_existing {
            &n.pre.vias
        } else {
            &[]
        };
        let fill: Vec<&crate::engine::Wire> = if r.cfg.write_fill {
            r.fill_export.iter().filter(|(k, _)| *k == ni).map(|(_, w)| w).collect()
        } else {
            vec![]
        };
        let polys: Vec<&(usize, usize, Vec<(f64, f64)>)> = if r.cfg.write_polygons && !r.cfg.pour_polygons {
            r.fill_polys.iter().filter(|p| p.0 == ni).collect()
        } else {
            vec![]
        };
        let keep = |w: &&crate::engine::Wire| !w.fill;
        if !n.r.wires.iter().any(|w| keep(&w))
            && fill.is_empty()
            && polys.is_empty()
            && n.r.vias.is_empty()
            && pre_w.is_empty()
            && pre_v.is_empty()
        {
            continue;
        }
        let _ = writeln!(o, "      (net {}", q(&n.name));
        // Without `keep_track_widths` the CAD applies its own width: the centre lines are
        // written as they are, with as few segments as possible.
        let split: Vec<Wire> = n.r.wires.iter().filter(keep).cloned().collect();
        for w in pre_w.iter().chain(split.iter()).chain(fill.iter().copied()) {
            let _ = write!(
                o,
                "        (wire (path {} {}",
                q(&r.layer_names[w.layer]),
                coord(w.width)
            );
            for p in &w.pts {
                let _ = write!(o, " {} {}", coord(p.0), coord(p.1));
            }
            let _ = writeln!(o, "))");
        }
        for (_, l, pts) in &polys {
            let _ = write!(o, "        (wire (polygon {} 0", q(&r.layer_names[*l]));
            for p in pts {
                let _ = write!(o, " {} {}", coord(p.0), coord(p.1));
            }
            let _ = writeln!(o, "))");
        }
        for v in pre_v.iter().chain(n.r.vias.iter()) {
            let _ = writeln!(
                o,
                "        (via {} {} {})",
                q(&n.via),
                coord(v.0),
                coord(v.1)
            );
        }
        let _ = writeln!(o, "      )");
    }
    let _ = writeln!(o, "    )");
    let _ = writeln!(o, "  )");
    let _ = writeln!(o, ")");
    o
}
