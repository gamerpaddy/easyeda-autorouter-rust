use router::{drc, ses, Config, NetOverride};
use std::path::PathBuf;

fn usage() -> ! {
    eprintln!(
        "usage: autorouter -nogui <in.dsn | in.json | in.epro2 | in.epro> -o <out.ses | out.json | out.epro2 | out.epro> [options]

  An EasyEDA Standard source file (File > Export > EasyEDA source) as input is routed and
  written back as EasyEDA JSON (-o *.json, default <name>-routed.json) with real track
  widths, vias and copper areas for the layer-assigned nets. -o *.ses writes a session file.
  An EasyEDA Pro project (*.epro2, or the older *.epro) is routed the same way and written back
  as a Pro project in the same format (-o *.epro2 / *.epro, default <name>-routed.<ext>): the PCB gets the new tracks, vias and copper pours,
  everything else in the project stays as it was.
  --no-fills             EasyEDA JSON: do not create copper areas
  --pour-reach-mm X      EasyEDA JSON: copper areas reach at most X mm from copper of their own
                         net (default 4)
  --fill-net NET         EasyEDA JSON: net of the copper areas that fill the empty space of every
                         layer (default GND), tied together with stitching vias
  --no-fill-empty        EasyEDA JSON: leave the empty space unfilled
  --stitch-pitch-mm X    EasyEDA JSON: distance of the stitching vias (default 4, 0 = none)
  --nudge-parts          EasyEDA JSON: when connections stay open, move small passives and other
                         small SMD parts a little and route again (default off; never ICs,
                         through-hole parts, connectors, switches or oscillators)
  --nudge-widths X       how far such a part may move from its place, in part sizes (default 3, max 3)
  --nudge-attempts N     how many moved layouts are routed at most (default 6)

  --options FILE         load options from a JSON file (the GUI writes the same format)
  --save-options FILE    write the effective options to a JSON file and continue
  --exclusive L1,L2      layers on which no traces are routed
  --exclusive-plane N@L  plane (NET@LAYER) in which no traces are routed (repeatable)
  --plane-penalty X      extra cost factor for cutting a foreign plane (default 3)
  --skip NET1,NET2       leave these nets unrouted
  --net-width NET=W      override a net's trace width (repeatable)
  --net-clearance NET=C  override a net's clearance (repeatable)
  --include-existing     DSN input: also write the existing wiring into the SES (an EasyEDA JSON /
                         Pro file always keeps what is on the board and adds the result to it)
  --allow-vias-in-pads   allow new vias to overlap pads (default off)
  --tighten              after routing pull every trace tight: the shortest path around what is in
                         the way, with 0/45/90 degree pieces
  --tighten-free         the same at any angle (not only 45 degrees)
  --no-suggest           do not test other settings when connections stay open
  --no-neckdown          never narrow a trace (default: a trace first keeps its full width and
                         goes around obstacles; only when that fails it is narrowed to 75%,
                         50%, then the minimum width, and widened again wherever room allows)
  --max-detour X         longest detour kept at full width, as a multiple of the straight
                         distance (+10 mm) (default 3, 0 = any detour before narrowing)
  --single-sided         one copper layer only: no vias, no via-in-pad, no jumps, no stitching,
                         copper areas on that layer only. The layer is the one with the most SMD
                         pads (bottom on a tie); pads only on the other layer stay open
  --single-layer NAME    like --single-sided, on the named layer
  --no-jumpers           single sided: no 0 ohm resistors (default: a connection with no path
                         at all hops over other traces with the smallest 0402/0603/0805/1206 that
                         fits, as a last resort; the parts are in the output / a .jumpers.json)
  --jumper-sizes LIST    sizes the router may use, e.g. 0805,1206 (default 0402,0603,0805,1206)
  --allow-plane-cuts     disable protection of imported planes
  --layer NAME=signal:NET,NET  preferred routing layer for assigned nets; short escapes
                         may use other layers, e.g. --layer 21=signal:GND
                         EasyEDA JSON: NAME=plane:NET,... makes the layer a plane (no traces,
                         pins reach it through vias); signal layers get pours for their nets
  --max-power-width-mm X with --keep-track-widths: widen layer-assigned /
                         width-override nets into the free room, up to X mm (default 5, 0 = off)
  --pour-outlines        also draw each assigned net's pour area as a thin closed trace on its
                         layer, to re-trace as a copper area in the CAD
  --keep-track-widths    the CAD keeps imported track widths: write real widths and widen
                         power traces (default off: EasyEDA drops widths, so plain centre
                         lines with the fewest segments are written)
  --unit-mm X            millimetres per DSN unit (default: detected)
  (the design rules of an imported EasyEDA file are used unless one of the rule switches is given)
  --rules NAME           design-rule preset: cheapest (JLCPCB 0.127, via 0.45/0.30), economic (default:
                         0.09, via 0.45/0.20), minimum (0.09, via 0.30/0.20, edge 0.2), relaxed;
                         single switches below override it
  --project-via          use the via of the project (the minimum via diameter only raises it);
                         default: vias are made exactly the minimum via diameter
  --min-via-drill-mm X   drill written for vias
  --stitch-near-mm X     stitching vias only within X mm of a pad, trace or via (default 3, 0 = anywhere)
  --min-trace-mm X --min-clearance-mm X --min-via-mm X --min-edge-mm X   global DRC minimums
  --no-drc-minimums      disable the global minimums (default: JLCPCB 4-layer economic)
  --net-vias NET=N       vias side by side at every layer change of a net (repeatable)
  --net-via NET=NAME     via padstack for a net (repeatable)
  --bridge-vias N        max new vias when joining a split plane/net (default 2, 0 = off)
  --no-plane-jumps       never use plane / no-routing layers, even for nets that stay open
  --jump-mm X            longest last-resort jump on such a layer (default 5)
  --snapshots DIR        write routing snapshots (viewable in autorouter)
  --every N   --via-cost C   --refine N   --no-pairs   --quiet"
    );
    let help = std::env::args().any(|a| a == "--help" || a == "-h");
    if !help {
        log_diagnostic("Invalid or incomplete CLI arguments; use -nogui --help");
    }
    std::process::exit(if help { 0 } else { 2 })
}

pub fn run() {
    let args: Vec<String> = std::env::args()
        .skip(1)
        .filter(|a| a != "-nogui" && a != "--nogui")
        .collect();
    if args.is_empty() {
        usage();
    }
    if args.iter().any(|a| a == "--help" || a == "-h") {
        usage();
    }
    for (i, a) in args.iter().enumerate() {
        if [
            "--options",
            "--save-options",
            "-o",
            "--exclusive",
            "--exclusive-plane",
            "--plane-penalty",
            "--skip",
            "--net-width",
            "--net-clearance",
            "--snapshots",
            "--every",
            "--via-cost",
            "--refine",
            "--grid",
            "--layer",
            "--net-vias",
            "--net-via",
            "--bridge-vias",
            "--jump-mm",
            "--unit-mm",
            "--min-trace-mm",
            "--min-clearance-mm",
            "--min-via-mm",
            "--min-via-drill-mm",
            "--stitch-near-mm",
            "--rules",
            "--min-edge-mm",
            "--max-power-width-mm",
            "--pour-reach-mm",
            "--single-layer",
            "--jumper-sizes",
            "--max-detour",
        ]
        .contains(&a.as_str())
            && i + 1 == args.len()
        {
            usage();
        }
    }
    // first pass: options file
    let mut cfg = Config::default();
    if let Some(i) = args.iter().position(|a| a == "--options") {
        let txt = std::fs::read_to_string(&args[i + 1]).expect("cannot read options file");
        cfg = serde_json::from_str(&txt).expect("bad options file");
        cfg.verbose = true;
        // the grid is adaptive and the pair gap is the clearance: old option files may name them
        cfg.grid = None;
        cfg.pair_gap = None;
    }
    if let Some(i) = args.iter().position(|a| a == "--rules") {
        match router::DrcRules::preset(&args[i + 1]) {
            Some(r) => cfg.rules = r,
            None => {
                eprintln!("unknown rule preset '{}' (cheapest, economic, minimum, relaxed)", args[i + 1]);
                std::process::exit(2)
            }
        }
    }
    // rules chosen on the command line or in an options file win over the rules of the file
    let rules_chosen = args.iter().any(|a| {
        ["--options", "--rules", "--min-trace-mm", "--min-clearance-mm", "--min-via-mm", "--min-via-drill-mm", "--no-drc-minimums"].contains(&a.as_str())
    });
    let mut input = None;
    let mut out = None;
    let mut save_opts = None;
    let mut no_fills = false;
    let mut no_suggest = false;
    let mut power_width_set = false;
    let mut i = 0;
    let kv = |s: &str| -> (String, f64) {
        let (k, v) = s.rsplit_once('=').unwrap_or_else(|| usage());
        (k.to_string(), v.parse().unwrap_or_else(|_| usage()))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--options" => i += 1,
            "--save-options" => {
                i += 1;
                save_opts = Some(args[i].clone());
            }
            "-o" => {
                i += 1;
                out = Some(PathBuf::from(&args[i]));
            }
            "--exclusive" => {
                i += 1;
                cfg.exclusive = args[i].split(',').map(|s| s.to_string()).collect();
            }
            "--exclusive-plane" => {
                i += 1;
                cfg.exclusive_planes.push(args[i].clone());
            }
            "--plane-penalty" => {
                i += 1;
                cfg.plane_penalty = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--skip" => {
                i += 1;
                for n in args[i].split(',') {
                    cfg.net_overrides
                        .entry(n.to_string())
                        .or_insert_with(NetOverride::default)
                        .skip = true;
                }
            }
            "--net-width" => {
                i += 1;
                let (n, v) = kv(&args[i]);
                cfg.net_overrides
                    .entry(n)
                    .or_insert_with(NetOverride::default)
                    .width = Some(v);
            }
            "--net-clearance" => {
                i += 1;
                let (n, v) = kv(&args[i]);
                cfg.net_overrides
                    .entry(n)
                    .or_insert_with(NetOverride::default)
                    .clearance = Some(v);
            }
            "--layer" => {
                i += 1;
                let (name, rest) = args[i].split_once('=').unwrap_or_else(|| usage());
                let (role, nets) = rest.split_once(':').unwrap_or((rest, ""));
                let plan = cfg.layers.entry(name.to_string()).or_default();
                plan.role = match role {
                    "plane" => router::LayerRole::Plane,
                    "signal" => router::LayerRole::Signal,
                    _ => usage(),
                };
                plan.nets = nets
                    .split(',')
                    .filter(|n| !n.is_empty())
                    .map(|n| n.to_string())
                    .collect();
            }
            "--unit-mm" => {
                i += 1;
                cfg.unit_mm = Some(args[i].parse().unwrap_or_else(|_| usage()));
            }
            "--min-trace-mm" => {
                i += 1;
                cfg.rules.min_trace_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--min-clearance-mm" => {
                i += 1;
                cfg.rules.min_clearance_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--min-via-mm" => {
                i += 1;
                cfg.rules.min_via_diameter_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--min-edge-mm" => {
                i += 1;
                cfg.rules.min_edge_clearance_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--rules" => i += 1,
            "--project-via" => cfg.rules.use_min_via = false,
            "--min-via-drill-mm" => {
                i += 1;
                cfg.rules.min_via_drill_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--stitch-near-mm" => {
                i += 1;
                cfg.stitch_near_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--no-drc-minimums" => cfg.rules.enabled = false,
            "--net-vias" => {
                i += 1;
                let (n, v) = kv(&args[i]);
                cfg.net_overrides.entry(n).or_default().via_count = Some(v as usize);
            }
            "--net-via" => {
                i += 1;
                let (n, v) = args[i].rsplit_once('=').unwrap_or_else(|| usage());
                cfg.net_overrides.entry(n.to_string()).or_default().via = Some(v.to_string());
            }
            "--bridge-vias" => {
                i += 1;
                cfg.bridge_vias = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--jump-mm" => {
                i += 1;
                cfg.plane_jump_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--no-plane-jumps" => cfg.allow_plane_jumps = false,
            "--max-power-width-mm" => {
                power_width_set = true;
                i += 1;
                cfg.power_width_max_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--pour-outlines" => cfg.pour_outlines = true,
            "--keep-track-widths" => cfg.keep_track_widths = true,
            "--no-fills" => no_fills = true,
            "--no-suggest" => no_suggest = true,
            "--tighten" => cfg.tighten = 1,
            "--tighten-free" => cfg.tighten = 2,
            "--pour-reach-mm" => {
                i += 1;
                cfg.pour_reach_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--fill-net" => {
                i += 1;
                cfg.fill_net = args[i].clone();
            }
            "--no-fill-empty" => cfg.fill_empty = false,
            "--nudge-parts" => cfg.nudge_parts = true,
            "--nudge-widths" => {
                i += 1;
                cfg.nudge_max_widths = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--nudge-attempts" => {
                i += 1;
                cfg.nudge_attempts = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--stitch-pitch-mm" => {
                i += 1;
                cfg.stitch_pitch_mm = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--allow-vias-in-pads" => cfg.allow_vias_in_pads = true,
            "--no-neckdown" => cfg.allow_neckdown = false,
            "--single-sided" => cfg.single_sided = true,
            "--single-layer" => {
                i += 1;
                cfg.single_sided = true;
                cfg.single_layer = Some(args[i].clone());
            }
            "--no-jumpers" => cfg.jumpers = false,
            "--jumper-sizes" => {
                i += 1;
                cfg.jumper_sizes = args[i].split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect();
            }
            "--max-detour" => {
                i += 1;
                cfg.max_detour = args[i].parse().unwrap_or_else(|_| usage());
            }
            "--allow-plane-cuts" => cfg.protect_planes = false,
            "--include-existing" => cfg.include_existing = true,
            "--snapshots" => {
                i += 1;
                cfg.snapshot_dir = Some(PathBuf::from(&args[i]));
            }
            "--every" => {
                i += 1;
                cfg.snapshot_every = args[i].parse().unwrap();
            }
            "--via-cost" => {
                i += 1;
                cfg.via_cost = args[i].parse().unwrap();
            }
            "--refine" => {
                i += 1;
                cfg.refine_passes = args[i].parse().unwrap();
            }
            "--grid" => {
                i += 1;
                cfg.grid = Some(args[i].parse().unwrap());
            }
            "--no-pairs" => cfg.pairs = false,
            "--quiet" => cfg.verbose = false,
            s if !s.starts_with('-') => input = Some(PathBuf::from(s)),
            _ => usage(),
        }
        i += 1;
    }
    let input = input.unwrap_or_else(|| usage());
    let is_pro = input.extension().is_some_and(|e| e.eq_ignore_ascii_case("epro2") || e.eq_ignore_ascii_case("epro"));
    let is_json = is_pro || input.extension().is_some_and(|e| e.eq_ignore_ascii_case("json"));
    if is_json {
        // an EasyEDA file keeps everything that is on the board; the result adds to it
        cfg.include_existing = true;
        // EasyEDA keeps track widths and has copper areas
        cfg.keep_track_widths = true;
        cfg.pour_polygons = !no_fills;
        // copper areas carry the current; many widened pieces only slow the import down
        if !power_width_set && !no_fills {
            cfg.power_width_max_mm = 0.0;
        }
    }
    cfg.use_trace_routing();
    if let Some(f) = save_opts {
        std::fs::write(&f, serde_json::to_string_pretty(&cfg).unwrap())
            .expect("cannot write options");
        eprintln!("options saved to {}", f);
    }
    let out = out.unwrap_or_else(|| {
        if is_json {
            let stem = input.file_stem().unwrap_or_default().to_string_lossy().to_string();
            input.with_file_name(format!("{stem}-routed.{}", if is_pro { input.extension().map_or("epro2".to_string(), |e| e.to_string_lossy().to_lowercase()) } else { "json".to_string() }))
        } else {
            input.with_extension("ses")
        }
    });
    // an .epro2 / .epro is a ZIP archive, everything else is text
    let raw = std::fs::read(&input).expect("cannot read input file");
    let src = if is_pro { String::new() } else { String::from_utf8(raw.clone()).expect("input file is not valid UTF-8") };
    let doc = if is_json {
        let d = (if is_pro { dsn::easyeda_pro::load(&raw) } else { dsn::easyeda::load(&src) }).unwrap_or_else(|e| {
            eprintln!("EasyEDA file error: {e}");
            std::process::exit(2)
        });
        for n in &d.notes {
            println!("note: {n}");
        }
        if let (Some(r), false) = (d.rules_mm, rules_chosen) {
            cfg.apply_file_rules(r);
            println!(
                "design rules of the file: trace {:.3} (default width; narrowing floor {:.3}), clearance {:.3}, via {:.3} / hole {:.3} mm",
                r[0], cfg.rules.min_trace_mm, r[1], r[2], r[3]
            );
        }
        Some(d)
    } else {
        None
    };
    let board = dsn::parse_dsn(doc.as_ref().map_or(src.as_str(), |d| d.dsn.as_str())).expect("dsn parse error");
    cfg.validate(&board).expect("invalid routing options");
    let parts = doc.as_ref().map(router::nudge::specs_from_doc).unwrap_or_default();
    let skip: Vec<(String, String)> = doc.as_ref().map(|d| d.areas.iter().cloned().collect()).unwrap_or_default();
    let base_cfg = cfg.clone();
    let solved = router::nudge::solve(&board, cfg, &parts, &skip);
    for l in &solved.log {
        println!("{l}");
    }
    let r = solved.router;
    if std::env::var_os("AR_PROF").is_some() {
        let mut v = r.prof.clone();
        let total: f32 = v.iter().map(|x| x.0).sum();
        v.sort_by(|a, b| b.0.total_cmp(&a.0));
        let n = v.len();
        let top = |k: usize| v.iter().take(k).map(|x| x.0).sum::<f32>();
        eprintln!("PROF {} searches, {:.2}s total, {:.1} M expansions ({:.0} ns/expansion)", n, total, r.pops_total as f64 / 1e6, total as f64 * 1e9 / r.pops_total.max(1) as f64);
        eprintln!("PROF top 10 = {:.2}s, top 50 = {:.2}s, top 200 = {:.2}s", top(10), top(50), top(200));
        for (name, f) in [("hard", (|x: &&(f32, u32, bool, u32, bool)| !x.2) as fn(&&(f32, u32, bool, u32, bool)) -> bool), ("soft(rip-up)", |x| x.2)] {
            let sel: Vec<_> = v.iter().filter(f).collect();
            eprintln!("PROF {:<13} {} searches {:.2}s, found {} ", name, sel.len(), sel.iter().map(|x| x.0).sum::<f32>(), sel.iter().filter(|x| x.4).count());
        }
        let fails: Vec<_> = v.iter().filter(|x| !x.4).collect();
        eprintln!("PROF failed searches {} taking {:.2}s", fails.len(), fails.iter().map(|x| x.0).sum::<f32>());
        let big = v.iter().filter(|x| x.3 as f64 > 0.8 * (r.grid.nx * r.grid.ny) as f64).collect::<Vec<_>>();
        eprintln!("PROF full-board-window searches {} taking {:.2}s (found {})", big.len(), big.iter().map(|x| x.0).sum::<f32>(), big.iter().filter(|x| x.4).count());
        for x in v.iter().take(6) { eprintln!("PROF   slow: {:.3}s {} expansions soft={} window={} found={}", x.0, x.1, x.2, x.3, x.4); }
    }
    let report = drc::check(&r);
    let name = input.file_stem().unwrap().to_string_lossy().to_string();
    println!(
        "routed {}/{} connections, length {:.1}, vias {}, time {:.2}s",
        r.connection_count() - r.unrouted_count(),
        r.connection_count(),
        r.total_length(),
        r.num_vias(),
        r.t0.elapsed().as_secs_f32()
    );
    let pre_w: usize = r.nets.iter().map(|n| n.pre.wires.len()).sum();
    if pre_w > 0 {
        println!("kept {} existing wire path(s) untouched", pre_w);
    }
    println!(
        "length vs MST lower bound: {:.1} / {:.1} = {:.3}x",
        r.total_length(),
        r.length_lower_bound(),
        r.total_length() / r.length_lower_bound()
    );
    for p in r.plane_report() {
        println!(
            "plane {}: {} piece(s) with pins, {} dead piece(s){}",
            p.name,
            p.islands,
            p.orphans,
            if p.islands > 1 || p.orphans > 0 {
                "  <-- cut by routing"
            } else {
                ""
            }
        );
    }
    println!(
        "DRC: {} clearance violations ({} shorts), {} open nets",
        report.clearance_violations.len(),
        report.shorts,
        report.open_nets.len()
    );
    for v in report.clearance_violations.iter().take(10) {
        println!("  {}", v);
    }
    for p in r.pair_report() {
        println!(
            "{} {}{}{}: {}  len {:.1} / {:.1} (skew {:.2})",
            if p.n.is_empty() { "group" } else { "pair" },
            p.p,
            if p.n.is_empty() { "" } else { "/" },
            p.n,
            if p.matched {
                format!("{}, length matched", p.kind)
            } else if p.coupled {
                format!("{}, UNMET LENGTH TOLERANCE", p.kind)
            } else {
                "NOT coupled (independent fallback)".to_string()
            },
            p.len_p,
            p.len_n,
            (p.len_p - p.len_n).abs()
        );
    }
    if r.jumps_used > 0 {
        println!("{} connection(s) use a short jump on a plane / no-routing layer", r.jumps_used);
    }
    if r.pad_vias_used > 0 {
        println!("{} connection(s) needed a via inside a pad of their own net", r.pad_vias_used);
    }
    if r.via_shortfall > 0 {
        println!("{} layer change(s) have fewer vias than requested (no room)", r.via_shortfall);
    }
    for o in &report.open_nets {
        println!("  open: {}", o);
    }
    if (r.unrouted_count() > 0 || !report.open_nets.is_empty()) && !no_suggest {
        let mut open_nets: Vec<String> = report.open_nets.iter().map(|o| o.split(" (").next().unwrap_or(o).to_string()).collect();
        open_nets.extend(r.nets.iter().filter(|n| !n.unconnected.is_empty() && !n.skip).map(|n| n.name.clone()));
        open_nets.sort();
        open_nets.dedup();
        let open_before = r.unrouted_count().max(report.open_nets.len());
        println!("connections stay open: testing other settings ...");
        let found = router::suggest::suggest(&board, &base_cfg, &parts, &skip, open_before, &open_nets, 4);
        if found.is_empty() {
            println!("no other setting that was tried routes more; try a different layer plan, wider spacing between parts or more layers");
        }
        for s in &found {
            println!("suggestion: {}", s.text());
        }
    }
    if !report.clearance_violations.is_empty() || !report.open_nets.is_empty() {
        log_diagnostic(&format!(
            "{}: {} clearance violations, open nets: {:?}",
            input.display(),
            report.clearance_violations.len(),
            report.open_nets
        ));
    }
    if r.cfg.pairs && r.pair_report().iter().any(|p| p.coupled && !p.matched) {
        eprintln!("warning: some differential groups exceed their length tolerance");
    }
    assert!(!r.cfg.pairs || r.pair_report().iter().all(|p| p.coupled), "SES export blocked: a differential group is not routed together. Adjust options or explicitly disable pairs.");
    let json_out = doc.is_some() && out.extension().is_some_and(|e| e.eq_ignore_ascii_case("json") || e.eq_ignore_ascii_case("epro2") || e.eq_ignore_ascii_case("epro"));
    if let (true, Some(d)) = (json_out, &doc) {
        let (bytes, st) = router::easyeda_out::write_output(&r, d).unwrap_or_else(|e| {
            eprintln!("cannot write the EasyEDA file: {e}");
            std::process::exit(2)
        });
        std::fs::write(&out, bytes).expect("cannot write EasyEDA file");
        println!(
            "EasyEDA {}: {} track(s), {} via(s), {} copper area(s) added{}",
            if d.pro.is_some() { "Pro project" } else { "JSON" },
            st.tracks,
            st.vias,
            st.areas,
            if r.outline_stats.1 > 0 { format!(" ({} pour piece(s) skipped)", r.outline_stats.1) } else { String::new() }
        );
        if st.jumpers > 0 {
            println!("{} jumper resistor(s) (0 ohm) carry nets over other traces:", st.jumpers);
            for (ni, j) in r.jumper_list() {
                let c = j.centre();
                println!("  {} on net {} at ({:.2}, {:.2}) mm", j.size, r.nets[ni].name, c.0 * r.unit_mm, c.1 * r.unit_mm);
            }
            if let Some(list) = router::easyeda_out::jumper_list_json(&r, d) {
                let f = out.with_extension("jumpers.json");
                std::fs::write(&f, list).expect("cannot write jumper list");
                println!("jumper placement list: {}", f.display());
            }
            if d.pro.is_some() {
                println!("note: EasyEDA Pro: the tracks run to the pad places of the jumpers; place the 0 ohm resistors from the list, they close the nets");
            }
        }
        if st.omitted > 0 {
            println!(
                "note: {} existing track(s)/via(s) are not in the saved file (they are shown in the preview only); nets that rely on them are open in that file. Use --include-existing to keep them",
                st.omitted
            );
        }
        for w in &r.pour_warnings {
            println!("WARNING copper area: {w}");
        }
        if !r.fill_polys.is_empty() {
            println!("note: copper areas are outlines only; rebuild them in EasyEDA (Ctrl+B in EasyEDA Standard) to cut around pads and traces");
        }
    } else {
        std::fs::write(&out, ses::write_ses(&r, &name)).expect("cannot write ses");
    }
    // the effective settings of this run (layers, widths, rules) next to the result
    let _ = std::fs::write(out.with_extension("options.json"), serde_json::to_string_pretty(&r.cfg).unwrap_or_default());
    if !json_out && !r.fill_polys.is_empty() {
        println!(
            "plane copper in SES: {} polygon(s), {} vertices",
            r.fill_polys.len(),
            r.fill_polys.iter().map(|p| p.2.len()).sum::<usize>()
        );
        println!("polygon check: {} cell(s) too close to foreign copper", r.check_fill_polygons());
    }
    if !r.fill_export.is_empty() {
        println!(
            "plane copper in SES: {} wide tracks, {} clearance conflicts",
            r.fill_export.len(),
            r.check_fill_export()
        );
    }
    if r.cfg.pour_outlines {
        println!(
            "pour outlines: {} drawn, {} skipped (no clean outline)",
            r.outline_stats.0, r.outline_stats.1
        );
    }
    println!("wrote {}", out.display());
}

fn log_diagnostic(message: &str) {
    use std::io::Write;
    let dir = std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("Autorouter");
    let _ = std::fs::create_dir_all(&dir);
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(dir.join("autorouter.log"))
    {
        let _ = writeln!(f, "{:?}: {}", std::time::SystemTime::now(), message);
    }
}
