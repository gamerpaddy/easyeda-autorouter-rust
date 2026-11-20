use place::engine::*;
use place::model::*;

fn sample(name: &str) -> Option<Vec<u8>> {
    std::fs::read(format!("D:/Downloads/{name}")).ok()
}

/// place, save, reload: every pad must sit where the placer put it
fn roundtrip(name: &str, sides: Sides) {
    let Some(bytes) = sample(name) else { return };
    let (doc, b) = place::load(&bytes).unwrap();
    let mut s = Settings::default();
    s.sides = sides;
    s.effort = 1;
    let o = place(&b, &s, &mut |_, _, _| true).unwrap();
    let errs = verify(&b, &s, &o);
    assert!(errs.is_empty(), "{name}: {:?}", &errs[..errs.len().min(5)]);
    let out = doc.save(&b, &o.pl, true, None).unwrap();
    let (_d2, b2) = place::load(&out).unwrap();
    assert_eq!(b.parts.len(), b2.parts.len());
    let mut moved = 0;
    let mut turned = 0;
    let mut flipped = 0;
    for (i, p) in b.parts.iter().enumerate() {
        let q = &b2.parts[i];
        assert_eq!(p.refdes, q.refdes);
        assert_eq!(p.pads.len(), q.pads.len(), "{}", p.refdes);
        if p.fixed() {
            assert_eq!((p.x, p.y), (q.x, q.y), "{} is locked", p.refdes);
        }
        if o.pl[i] != Pl::of(p) {
            moved += 1;
        }
        if o.pl[i].d != 0 {
            turned += 1;
        }
        if o.pl[i].flip {
            flipped += 1;
        }
        let pl = if o.unplaced.contains(&i) { Pl::of(p) } else { o.pl[i] };
        for k in 0..p.pads.len() {
            let want = p.pad_box(k, &pl);
            let got = q.pad_box(k, &Pl::of(q));
            for c in 0..4 {
                assert!((want[c] - got[c]).abs() < 0.03, "{name}: {} pad {} box {:?} vs {:?} (pl {:?})", p.refdes, p.pads[k].num, want, got, o.pl[i]);
            }
            let nn = |bd: &Board, pad: &PadG| if pad.net >= 0 { bd.nets[pad.net as usize].name.clone() } else { String::new() };
            assert_eq!(nn(&b, &p.pads[k]), nn(&b2, &q.pads[k]));
        }
    }
    println!("{name}: {moved} moved, {turned} turned, {flipped} flipped, all pads where expected");
    assert!(moved > 0);
}

#[test]
fn std_front() {
    roundtrip("PCB_PCB_ESP32-S31-Ethernet_board_2026-11-17.json", Sides::Front);
}

#[test]
fn std_big() {
    roundtrip("PCB_PCB_adau1452-dsp-v2_2026-11-18.json", Sides::Front);
}

#[test]
fn pro_front() {
    roundtrip("ProPrj_stm32f767_2026-11-17.epro2", Sides::Front);
}

#[test]
fn pro_both() {
    roundtrip("ProPrj_stm32f767_2026-11-17.epro2", Sides::Both);
}

#[test]
fn pro_curvetracer_both() {
    roundtrip("ProPrj_curvetracer1_2026-11-18.epro2", Sides::Both);
}

/// a part turned and flipped by hand, then baked, keeps its pads in place
#[test]
fn baked_matches_placement() {
    let Some(bytes) = sample("ProPrj_stm32f767_2026-11-17.epro2") else { return };
    let (_d, b) = place::load(&bytes).unwrap();
    for (i, p) in b.parts.iter().enumerate().filter(|(_, p)| p.pads.len() > 1) {
        for (d, flip) in [(1u8, false), (2, true), (3, true)] {
            let pl = Pl { x: 10.0 + i as f64, y: -5.0, d, flip };
            let q = p.baked(&pl);
            for k in 0..p.pads.len() {
                let (a, c) = (p.pad_box(k, &pl), q.pad_box(k, &Pl::of(&q)));
                for n in 0..4 {
                    assert!((a[n] - c[n]).abs() < 1e-9);
                }
            }
            let (r1, r2) = (p.rect(&pl), q.rect(&Pl::of(&q)));
            for n in 0..4 {
                assert!((r1[n] - r2[n]).abs() < 1e-9);
            }
        }
    }
}
