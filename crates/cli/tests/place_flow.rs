//! A part moved in the placement tab reaches the router: the saved file, loaded by the router,
//! has exactly the pads of that part moved, by the distance it was moved.

#[test]
fn moved_part_is_where_the_router_sees_it() {
    let Ok(bytes) = std::fs::read("D:/Downloads/ProPrj_PCB_PCB_usb3clframegrabberwithfx3_2026-11-20_2026-11-20.epro2") else {
        return;
    };
    let (doc, board) = place::load(&bytes).unwrap();
    let i = board.parts.iter().position(|p| p.movable && !p.locked && p.pads.len() >= 2 && p.pads.len() <= 4).expect("a small part");
    let mut pls: Vec<place::model::Pl> = board.parts.iter().map(place::model::Pl::of).collect();
    pls[i].x += 5.0;
    pls[i].y -= 2.0;
    let out = doc.save(&board, &pls, false, None).unwrap();
    let load = |b: &[u8]| {
        let d = dsn::easyeda_pro::load(b).unwrap();
        let brd = dsn::parse_dsn(&d.dsn).unwrap();
        let unit = brd.detect_unit_mm().0;
        brd.abs_pins().into_iter().map(|(k, p)| (k, (p.x * unit, p.y * unit))).collect::<std::collections::HashMap<_, _>>()
    };
    let (a, b) = (load(&bytes), load(&out));
    assert_eq!(a.len(), b.len());
    let moved: Vec<(f64, f64)> = a
        .iter()
        .filter_map(|(k, p)| {
            let q = b.get(k)?;
            ((q.0 - p.0).abs() > 1e-6 || (q.1 - p.1).abs() > 1e-6).then_some((q.0 - p.0, q.1 - p.1))
        })
        .collect();
    assert_eq!(moved.len(), board.parts[i].pads.len(), "only the pads of the moved part change");
    for m in moved {
        assert!((m.0.abs() - 5.0).abs() < 0.05 && (m.1.abs() - 2.0).abs() < 0.05, "moved by {m:?}");
    }
}
