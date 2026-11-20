use place::engine::*;
use place::model::*;
use std::time::Instant;

const USAGE: &str = "autoplacer-cli <board.json|board.epro2> [options]
  -o FILE            output (default: <name>-placed.<ext>)
  --info             only describe the board
  --sides S          front (default) | back | both
  --size S           noroom | tight | normal (default) | relaxed   (only without a board outline)
  --max WxH          fixed board size in mm (only without a board outline)
  --room R           room around big chips: off | low | normal (default) | generous
  --big-pins N       pin count of a big chip (default 16)
  --clearance MM     gap between parts (default 0.25)
  --edge MM          distance to the board edge (default 0.3)
  --effort N         1..10 (default 3)
  --seed N
  --no-rotate        keep part orientations
  --no-bypass        no special handling of decoupling capacitors
  --keep-routing     keep existing tracks and vias (default: removed, they no longer fit)
  --svg FILE         also write a picture of the result (SVG);  --svg-before FILE: of the input
  --write-outline    when the file has no outline, add a rectangle around the placed parts (Standard format)";

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.is_empty() || a.iter().any(|x| x == "-h" || x == "--help") {
        println!("{USAGE}");
        return;
    }
    if let Err(e) = run(&a) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn run(a: &[String]) -> Result<(), String> {
    let mut input = None;
    let mut out = None;
    let mut s = Settings::default();
    let (mut info, mut clear, mut wo) = (false, true, false);
    let (mut svg, mut svg_before): (Option<String>, Option<String>) = (None, None);
    let mut i = 0;
    let val = |i: &mut usize| -> Result<String, String> {
        *i += 1;
        a.get(*i).cloned().ok_or_else(|| format!("{} needs a value", a[*i - 1]))
    };
    while i < a.len() {
        match a[i].as_str() {
            "-o" => out = Some(val(&mut i)?),
            "--info" => info = true,
            "--sides" => {
                s.sides = match val(&mut i)?.as_str() {
                    "front" => Sides::Front,
                    "back" => Sides::Back,
                    "both" => Sides::Both,
                    o => return Err(format!("bad --sides {o}")),
                }
            }
            "--size" => {
                s.size = match val(&mut i)?.as_str() {
                    "noroom" => Size::NoRoom,
                    "tight" => Size::Tight,
                    "normal" => Size::Normal,
                    "relaxed" => Size::Relaxed,
                    o => return Err(format!("bad --size {o}")),
                }
            }
            "--max" => {
                let v = val(&mut i)?;
                let (w, h) = v.split_once(['x', 'X']).ok_or("--max needs WxH")?;
                s.max_size = Some((w.parse().map_err(|_| "bad width")?, h.parse().map_err(|_| "bad height")?));
            }
            "--room" => {
                s.room = match val(&mut i)?.as_str() {
                    "off" => 0.0,
                    "low" => 0.5,
                    "normal" => 1.0,
                    "generous" => 2.0,
                    o => return Err(format!("bad --room {o}")),
                }
            }
            "--big-pins" => s.big_pins = val(&mut i)?.parse().map_err(|_| "bad number")?,
            "--clearance" => s.clearance = val(&mut i)?.parse().map_err(|_| "bad number")?,
            "--edge" => s.edge_margin = val(&mut i)?.parse().map_err(|_| "bad number")?,
            "--effort" => s.effort = val(&mut i)?.parse().map_err(|_| "bad number")?,
            "--seed" => s.seed = val(&mut i)?.parse().map_err(|_| "bad number")?,
            "--no-rotate" => s.allow_rotate = false,
            "--no-bypass" => s.bypass = false,
            "--keep-routing" => clear = false,
            "--write-outline" => wo = true,
            "--svg" => svg = Some(val(&mut i)?),
            "--svg-before" => svg_before = Some(val(&mut i)?),
            x if x.starts_with('-') => return Err(format!("unknown option {x}")),
            x => input = Some(x.to_string()),
        }
        i += 1;
    }
    let input = input.ok_or("no input file")?;
    let bytes = std::fs::read(&input).map_err(|e| format!("{input}: {e}"))?;
    let (doc, board) = place::load(&bytes)?;
    let locked = board.parts.iter().filter(|p| p.fixed()).count();
    println!("{}: {} parts ({} locked/fixed), {} nets, outline: {}, {} tracks/vias", board.title, board.parts.len(), locked, board.nets.len(), if board.outline.is_some() { "yes" } else { "none" }, board.routing);
    for n in &board.notes {
        println!("note: {n}");
    }
    if let Some(f) = &svg_before {
        let pl: Vec<Pl> = board.parts.iter().map(Pl::of).collect();
        let o = place::draw::SceneOpts { rats: true, gnd: false, labels: true, select: None, caps: &[], region: &[], locked: &[], pin_nets: false };
        std::fs::write(f, place::draw::to_svg(&board, &pl, &o, "before")).map_err(|e| e.to_string())?;
    }
    if info {
        for p in &board.parts {
            println!("  {:8} {:20} {:3} pads {}{}{} at ({:.2},{:.2})", p.refdes, p.package, p.pads.len(), if p.locked { "LOCKED " } else { "" }, if p.bottom { "back " } else { "" }, if !p.movable { "fixed" } else { "" }, p.x, p.y);
        }
        return Ok(());
    }
    let t = Instant::now();
    let mut last = String::new();
    let o = place(&board, &s, &mut |_, phase, f| {
        let l = format!("{phase} {}0%", (f * 10.0) as i32);
        if l != last {
            eprint!("\r{l}   ");
            last = l;
        }
        true
    })?;
    eprintln!();
    println!("wire length {:.1} mm -> {:.1} mm ({:+.1}%), {:.1}s", o.hpwl_before, o.hpwl_after, (o.hpwl_after / o.hpwl_before.max(1e-9) - 1.0) * 100.0, t.elapsed().as_secs_f64());
    if !o.caps.is_empty() {
        let avg = o.caps.iter().map(|c| c.2).sum::<f64>() / o.caps.len() as f64;
        let max = o.caps.iter().map(|c| c.2).fold(0.0, f64::max);
        println!("{} bypass capacitors: power pad to chip pin avg {:.2} mm, max {:.2} mm", o.caps.len(), avg, max);
        let seg = |c: usize, h: usize| -> Option<(P, P)> {
            let (cp, hp) = (&board.parts[c], &board.parts[h]);
            let mut best: Option<(f64, P, P)> = None;
            for i in 0..cp.pads.len() {
                for k in 0..hp.pads.len() {
                    if cp.pads[i].net >= 0 && cp.pads[i].net == hp.pads[k].net && board.nets[cp.pads[i].net as usize].kind == NetKind::Power {
                        let (a, b) = (cp.pad_xy(i, &o.pl[c]), hp.pad_xy(k, &o.pl[h]));
                        let d = (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2);
                        if best.map_or(true, |x| d < x.0) {
                            best = Some((d, a, b));
                        }
                    }
                }
            }
            best.map(|b| (b.1, b.2))
        };
        let cross = |a: P, b: P, c: P, d: P| {
            let close = |p: P, q: P| (p.0 - q.0).abs() < 1e-6 && (p.1 - q.1).abs() < 1e-6;
            if close(a, c) || close(a, d) || close(b, c) || close(b, d) { return false; }
            let f = |p: P, q: P, r: P| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
            f(a, b, c) * f(a, b, d) < 0.0 && f(c, d, a) * f(c, d, b) < 0.0
        };
        let mut crossings = 0;
        for x in 0..o.caps.len() {
            for y in x + 1..o.caps.len() {
                if o.caps[x].1 == o.caps[y].1 {
                    if let (Some(s1), Some(s2)) = (seg(o.caps[x].0, o.caps[x].1), seg(o.caps[y].0, o.caps[y].1)) {
                        if cross(s1.0, s1.1, s2.0, s2.1) { crossings += 1; }
                    }
                }
            }
        }
        println!("   crossing capacitor links: {crossings}");
        let mut w = o.caps.clone();
        w.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
        for c in w.iter().take(6) {
            println!("   {} -> {}: {:.2} mm", board.parts[c.0].refdes, board.parts[c.1].refdes, c.2);
        }
    }
    println!("used area {:.1} x {:.1} mm", o.used[2] - o.used[0], o.used[3] - o.used[1]);
    for n in &o.notes {
        println!("note: {n}");
    }
    let errs = verify(&board, &s, &o);
    if errs.is_empty() {
        println!("check: no overlaps, everything inside the board, locked parts unmoved");
    } else {
        println!("CHECK FAILED ({} problems):", errs.len());
        for e in errs.iter().take(20) {
            println!("  {e}");
        }
    }
    if let Some(f) = &svg {
        let o2 = place::draw::SceneOpts { rats: true, gnd: false, labels: true, select: None, caps: &o.caps, region: if board.outline.is_none() { &o.region } else { &[] }, locked: &[], pin_nets: false };
        std::fs::write(f, place::draw::to_svg(&board, &o.pl, &o2, "after")).map_err(|e| e.to_string())?;
    }
    let moved = o.pl.iter().zip(&board.parts).filter(|(p, q)| **p != Pl::of(q)).count();
    println!("{moved} parts moved");
    let outline = if wo && board.outline.is_none() && doc.can_write_outline() {
        let m = s.edge_margin + 0.2;
        Some([o.used[0] - m, o.used[1] - m, o.used[2] + m, o.used[3] + m])
    } else {
        None
    };
    let data = doc.save(&board, &o.pl, clear, outline)?;
    let out = out.unwrap_or_else(|| {
        let p = std::path::Path::new(&input);
        let stem = p.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
        p.with_file_name(format!("{stem}-placed.{}", doc.ext())).to_string_lossy().to_string()
    });
    std::fs::write(&out, data).map_err(|e| format!("{out}: {e}"))?;
    println!("wrote {out}");
    Ok(())
}
