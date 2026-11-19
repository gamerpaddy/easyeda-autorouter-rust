//! Diagnostic: solve an EasyEDA Standard JSON board with an options file, then print the map of
//! blocked cells around the given pins (see `Router::debug_pin_map`).
//! usage: pinmap <board.json> <options.json> <rx> <ry> <pin> [pin ...]
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let src = std::fs::read_to_string(&a[1]).unwrap();
    let doc = dsn::easyeda::load(&src).unwrap();
    let board = dsn::parse_dsn(&doc.dsn).unwrap();
    let mut cfg: router::Config = serde_json::from_str(&std::fs::read_to_string(&a[2]).unwrap()).unwrap();
    cfg.verbose = false;
    cfg.grid = None;
    // as the command line sets it for an EasyEDA file
    cfg.include_existing = true;
    cfg.keep_track_widths = true;
    cfg.pour_polygons = true;
    cfg.power_width_max_mm = 0.0;
    cfg.use_trace_routing();
    let (rx, ry): (i32, i32) = (a[3].parse().unwrap(), a[4].parse().unwrap());
    let parts = router::nudge::specs_from_doc(&doc);
    let skip: Vec<(String, String)> = doc.areas.iter().cloned().collect();
    let mut r = router::nudge::solve(&board, cfg, &parts, &skip).router;
    let rep = router::drc::check(&r);
    println!("DRC: {} violations, open: {:?}; unrouted_count {}", rep.clearance_violations.len(), rep.open_nets, r.unrouted_count());
    println!("escape cells: {}", r.escape.len());
    for pin in &a[5..] {
        if let Some(pi) = r.pins.iter().position(|p| p.name.eq_ignore_ascii_case(pin)) {
            let c = r.pins[pi].c;
            let n = r.pins[pi].net as usize;
            for w in r.nets[n].r.wires.iter().filter(|w| w.pts.iter().any(|q| (q.0 - c.0).abs() < 4.0 && (q.1 - c.1).abs() < 4.0)) {
                println!("{} wire near pad {}: layer {} width {:.3} {:?}", r.nets[n].name, pin, w.layer, w.width, w.pts);
            }
        }
        println!("{}", r.debug_pin_map(pin, rx, ry));
    }
}
