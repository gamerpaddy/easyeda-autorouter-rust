//! Diagnostic: route (no refine/finish) and dump the copper around every open connection.
//! usage: dump <dsn> <options.json> <out.json> [radius]
use router::geom::Geo;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let b = dsn::parse_dsn(&std::fs::read_to_string(&a[1]).unwrap()).unwrap();
    let mut cfg: router::Config = serde_json::from_str(&std::fs::read_to_string(&a[2]).unwrap()).unwrap();
    cfg.use_trace_routing();
    cfg.verbose = false;
    let radius: f64 = a.get(4).map_or(14.0, |s| s.parse().unwrap());
    let mut r = router::Router::new(&b, cfg);
    r.route_all();
    let mut out = vec![];
    for n in 0..r.nets.len() {
        for &pin in &r.nets[n].unconnected {
            let c = r.pins[pin].c;
            let mut prims = vec![];
            let near = |p: &router::geom::Prim| p.bb.0 < c.0 + radius && p.bb.2 > c.0 - radius && p.bb.1 < c.1 + radius && p.bb.3 > c.1 - radius;
            let mut add = |p: &router::geom::Prim, kind: &str| {
                if !near(p) { return; }
                let name = if p.net >= 0 { r.nets[p.net as usize].name.clone() } else { String::new() };
                let geo = match &p.geo {
                    Geo::Circle { c, r } => serde_json::json!({"c": [c.0, c.1], "r": r}),
                    Geo::Seg { a, b, hw } => serde_json::json!({"a": [a.0, a.1], "b": [b.0, b.1], "hw": hw}),
                    Geo::Poly(p) => serde_json::json!({"poly": p.iter().map(|q| [q.0, q.1]).collect::<Vec<_>>()}),
                };
                prims.push(serde_json::json!({"layer": p.layer, "net": name, "kind": kind, "geo": geo}));
            };
            for p in &r.pins { for pr in &p.prims { add(pr, "pad"); } }
            for f in &r.fixed { add(f, "fixed"); }
            for ns in &r.nets { for pr in ns.r.dynp.iter().chain(ns.pre.dynp.iter()) { add(pr, "route"); } }
            out.push(serde_json::json!({"net": r.nets[n].name, "pin": r.pins[pin].name, "c": [c.0, c.1], "radius": radius, "g": r.grid.g, "prims": prims}));
        }
    }
    eprintln!("open: {}", out.len());
    std::fs::write(&a[3], serde_json::to_string(&out).unwrap()).unwrap();
}
