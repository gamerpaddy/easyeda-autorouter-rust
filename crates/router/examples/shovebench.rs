//! Replay dumped shove problems (AR_DUMP_SHOVE=dir) with different solver settings.
//! usage: shovebench <dir>
use router::shove::*;
fn main() {
    let dir = std::env::args().nth(1).unwrap();
    let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).collect();
    files.sort();
    let variants: Vec<(&str, Params)> = vec![
        ("base", Params { max_shove: 0.0, sweeps: 80, margin: 0.02, relax: 1.0, cap_mult: 3.0 }),
        ("relax .7 sweeps 300", Params { max_shove: 0.0, sweeps: 300, margin: 0.02, relax: 0.7, cap_mult: 3.0 }),
        ("relax .5 sweeps 400", Params { max_shove: 0.0, sweeps: 400, margin: 0.02, relax: 0.5, cap_mult: 3.0 }),
        ("relax 1 cap 1.5 sweeps 300", Params { max_shove: 0.0, sweeps: 300, margin: 0.02, relax: 1.0, cap_mult: 1.5 }),
        ("relax .7 cap 1.5 sweeps 400", Params { max_shove: 0.0, sweeps: 400, margin: 0.02, relax: 0.7, cap_mult: 1.5 }),
        ("relax .6 cap 2 sweeps 400 margin .05", Params { max_shove: 0.0, sweeps: 400, margin: 0.05, relax: 0.6, cap_mult: 2.0 }),
    ];
    for f in files {
        let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&f).unwrap()).unwrap();
        let nodes: Vec<Node> = serde_json::from_value(doc["nodes"].clone()).unwrap();
        let items: Vec<Item> = serde_json::from_value(doc["items"].clone()).unwrap();
        let obst: Vec<Obst> = serde_json::from_value(doc["obst"].clone()).unwrap();
        let max_shove = doc["max_shove"].as_f64().unwrap();
        println!("{} net {} original: {}", f.display(), doc["net"], doc["why"].as_str().unwrap_or(""));
        for (name, p) in &variants {
            let mut n = nodes.clone();
            let mut p = *p;
            p.max_shove = max_shove;
            let t = std::time::Instant::now();
            let r = solve_with(&mut n, &items, &obst, &p);
            let far = n.iter().map(|x| ((x.pos.0 - x.orig.0).powi(2) + (x.pos.1 - x.orig.1).powi(2)).sqrt()).fold(0.0, f64::max);
            println!("   {:40} {:?} ({:.2}s, farthest node {:.2})", name, r.as_ref().map(|_| "OK").unwrap_or_else(|e| e.as_str()), t.elapsed().as_secs_f32(), far);
        }
    }
}
