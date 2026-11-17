use router::{drc, ses, Config, Router};

fn board() -> dsn::Board {
    dsn::parse_dsn(
        r#"(pcb test (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 30 0 30 20 0 20))
       (layer top (type signal)) (layer bottom (type signal))
       (grid wire 0.5) (via via0) (rule (width 1) (clearance 0.5)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin pad 1 5 5) (pin pad 2 25 5)
                           (pin pad 3 5 15) (pin pad 4 25 15))
       (padstack pad (shape (circle top 2)) (shape (circle bottom 2)))
       (padstack via0 (shape (circle top 1)) (shape (circle bottom 1))))
      (network (net GND (pins U-1 U-2)) (net DATA (pins U-3 U-4))))"#,
    )
    .unwrap()
}

#[test]
fn finish_preserves_connectivity_and_creates_safe_ground_on_each_layer() {
    let b = board();
    let mut r = Router::new(
        &b,
        Config {
            verbose: false,
            ..Config::default()
        },
    );
    r.route_all();
    r.refine();
    r.finish();
    let rep = drc::check(&r);
    assert!(
        rep.clearance_violations.is_empty(),
        "{:?}",
        rep.clearance_violations
    );
    assert!(rep.open_nets.is_empty(), "{:?}", rep.open_nets);
    let gnd = &r.nets[0];
    for layer in 0..2 {
        assert!(gnd.r.wires.iter().any(|w| w.layer == layer));
    }
    for n in &r.nets {
        for w in &n.r.wires {
            for s in w.pts.windows(2) {
                let dx = (s[1].0 - s[0].0).abs();
                let dy = (s[1].1 - s[0].1).abs();
                assert!(dx < 1e-8 || dy < 1e-8 || (dx - dy).abs() < 1e-8);
            }
        }
    }
}

#[test]
fn ses_coordinates_and_width_are_resolution_scaled() {
    let b = board();
    let mut r = Router::new(
        &b,
        Config {
            verbose: false,
            keep_track_widths: true,
            ..Config::default()
        },
    );
    r.nets[0].r.wires.push(router::Wire {
        fill: false,
        layer: 0,
        width: 1.25,
        pts: vec![(5.0, 5.0), (10.0, 5.0)],
    });
    let s = ses::write_ses(&r, "test");
    assert!(s.contains("(resolution mil 1000)"));
    assert!(s.contains("(path top 1250 5000 5000 10000 5000)"));
}

#[test]
fn via_mask_survives_own_pad_exemption() {
    let b = board();
    for allow in [false, true] {
        let r = Router::new(
            &b,
            Config {
                verbose: false,
                allow_vias_in_pads: allow,
                ..Config::default()
            },
        );
        let (x, y) = r.grid.cell((5.0, 5.0));
        let idx = r.grid.idx(0, x, y);
        // When disabled, own pad contributes an additional independent mask.
        let blocked = r.fields[r.nets[0].vf].cnt[idx];
        assert_eq!(blocked, if allow { 1 } else { 2 });
    }
}

#[test]
fn invalid_groups_and_grid_are_rejected() {
    let b = board();
    let mut c = Config::default();
    c.grid = Some(0.0);
    assert!(c.validate(&b).is_err());
    c.grid = None;
    c.net_overrides.entry("DATA".into()).or_default().diff_group = "USB".into();
    assert!(c.validate(&b).is_err());
    c.net_overrides.entry("GND".into()).or_default().diff_group = "USB".into();
    assert!(c.validate(&b).is_ok());
}

#[test]
fn detects_data_suffixes_without_matching_supply_polarities() {
    let nets = [
        "DP", "DN", "USB_DP", "USB_DN", "CLK_P", "CLK_N", "+12V", "-12V",
    ]
    .map(|name| dsn::Net {
        name: name.into(),
        pins: vec![],
    });
    assert_eq!(dsn::find_diff_pairs(&nets), vec![(0, 1), (2, 3), (4, 5)]);
}

#[test]
fn narrow_connection_keeps_full_width_outside_bottleneck() {
    let mut b = board();
    b.nets.remove(0);
    for (y1, y2) in [(0.0, 14.1), (15.9, 20.0)] {
        b.keepouts.push(dsn::Keepout {
            layer: None,
            shape: dsn::Shape::Rect {
                layer: "top".into(),
                x1: 14.0,
                y1,
                x2: 16.0,
                y2,
            },
            via_only: false,
            wire_only: false,
        });
    }
    for allow in [false, true] {
        let mut r = Router::new(
            &b,
            Config {
                verbose: false,
                allow_neckdown: allow,
                grid: Some(0.25),
                ..Config::default()
            },
        );
        r.route_all();
        if allow {
            assert_eq!(r.unrouted_count(), 0);
            assert!(r.nets[0].r.wires.iter().any(|w| w.width < 1.0 - 1e-9));
            assert!(r.nets[0].r.wires.iter().any(|w| w.width == 1.0));
            assert!(drc::check(&r).clearance_violations.is_empty());
        } else {
            assert!(r.unrouted_count() > 0);
        }
    }
}

#[test]
fn three_terminal_pair_routes_as_coupled_branches() {
    let b = dsn::parse_dsn(
        r#"(pcb pair (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 40 0 40 25 0 25))
       (layer top (type signal)) (grid wire 0.25) (rule (width 0.5) (clearance 0.25)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin pad 1 5 5) (pin pad 2 20 5) (pin pad 3 35 5)
                           (pin pad 4 5 8) (pin pad 5 20 8) (pin pad 6 35 8))
       (padstack pad (shape (circle top 1))))
      (network (net DP (pins U-1 U-2 U-3)) (net DN (pins U-4 U-5 U-6))))"#,
    )
    .unwrap();
    let mut r = Router::new(
        &b,
        Config {
            verbose: false,
            ..Config::default()
        },
    );
    r.route_all();
    r.finish();
    assert!(r.pair_report()[0].coupled);
    assert!(r.pair_report()[0].matched);
    let report = drc::check(&r);
    assert!(
        report.clearance_violations.is_empty(),
        "{:?}",
        report.clearance_violations
    );
    assert!(report.open_nets.is_empty(), "{:?}", report.open_nets);
}

fn smd_board() -> dsn::Board {
    dsn::parse_dsn(
        r#"(pcb smd (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 40 0 40 30 0 30))
       (layer top (type signal)) (layer bottom (type signal))
       (grid wire 0.5) (via via0) (rule (width 1) (clearance 0.5)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin smd 1 5 5) (pin smd 2 35 25) (pin smd 3 5 25) (pin smd 4 35 5)
                           (pin smd 5 20 15) (pin smd 6 20 5))
       (padstack smd (shape (circle top 2)))
       (padstack via0 (shape (circle top 1.5)) (shape (circle bottom 1.5))))
      (network (net GND (pins U-1 U-2 U-5)) (net SIG (pins U-3 U-4)) (net VCC (pins U-6))))"#,
    )
    .unwrap()
}

#[test]
fn plane_layer_connects_through_vias_without_drawing_ground_traces() {
    let b = smd_board();
    let mut cfg = Config {
        verbose: false,
        ..Config::default()
    };
    cfg.layers.insert(
        "bottom".into(),
        router::LayerPlan {
            role: router::LayerRole::Plane,
            nets: vec!["GND".into()],
        },
    );
    assert!(cfg.validate(&b).is_ok());
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    let rep = drc::check(&r);
    assert!(rep.open_nets.is_empty(), "{:?}", rep.open_nets);
    assert!(rep.clearance_violations.is_empty(), "{:?}", rep.clearance_violations);
    let gnd = &r.nets[0];
    // every ground pad reaches the plane by a via, no routed ground trace on the plane layer
    assert!(!gnd.r.vias.is_empty());
    assert!(gnd.r.wires.iter().all(|w| w.layer != 1 || w.fill));
    assert!(gnd.r.wires.iter().any(|w| w.layer == 1 && w.fill));
    // no other net is routed on the plane layer
    for n in &r.nets[1..] {
        assert!(n.r.wires.iter().all(|w| w.layer != 1));
    }
}

#[test]
fn groups_of_three_are_accepted_and_routed_together() {
    let b = dsn::parse_dsn(
        r#"(pcb bus (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 60 0 60 30 0 30))
       (layer top (type signal)) (layer bottom (type signal))
       (grid wire 0.5) (via via0) (rule (width 0.5) (clearance 0.5)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin smd 1 5 6) (pin smd 2 5 12) (pin smd 3 5 18)
                           (pin smd 4 55 6) (pin smd 5 55 12) (pin smd 6 55 18))
       (padstack smd (shape (circle top 1.5)))
       (padstack via0 (shape (circle top 1)) (shape (circle bottom 1))))
      (network (net A (pins U-1 U-4)) (net B (pins U-2 U-5)) (net C (pins U-3 U-6))))"#,
    )
    .unwrap();
    let mut cfg = Config {
        verbose: false,
        ..Config::default()
    };
    for n in ["A", "B", "C"] {
        cfg.net_overrides.entry(n.into()).or_default().diff_group = "1".into();
    }
    assert!(cfg.validate(&b).is_ok());
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    assert_eq!(r.unrouted_count(), 0);
    let rep = drc::check(&r);
    assert!(rep.clearance_violations.is_empty() && rep.open_nets.is_empty());
    let pr = r.pair_report();
    assert_eq!(pr.len(), 1);
    assert_eq!(pr[0].kind, "guided");
}

#[test]
fn global_minimums_raise_widths_and_detect_ten_mil_units() {
    let b = board();
    let mut cfg = Config {
        verbose: false,
        unit_mm: Some(1.0),
        ..Config::default()
    };
    cfg.rules.min_trace_mm = 2.5;
    cfg.rules.min_clearance_mm = 0.75;
    let r = Router::new(&b, cfg);
    assert!(r.nets.iter().all(|n| n.width >= 2.5 - 1e-9 && n.clr >= 0.75 - 1e-9));
    // a DSN that declares `mil 1000` but uses 10 mil coordinates is recognised
    let ten = dsn::parse_dsn(
        r#"(pcb t (resolution mil 1000) (structure (boundary (path signal 0 0 0 393.7 0 393.7 393.7 0 393.7))
        (layer 1 (type signal)) (rule (clear 0.6034)) (rule (width 0.8711))))"#,
    )
    .unwrap();
    let (u, assumed) = ten.detect_unit_mm();
    assert!(assumed && (u - 0.254).abs() < 1e-9, "{u}");
}

#[test]
fn traces_end_in_the_middle_of_their_pads() {
    let b = smd_board();
    let mut r = Router::new(
        &b,
        Config {
            verbose: false,
            add_gnd_planes: false,
            ..Config::default()
        },
    );
    r.route_all();
    r.refine();
    r.finish();
    assert_eq!(r.unrouted_count(), 0);
    let centers = [(5.0, 5.0), (35.0, 25.0), (5.0, 25.0), (35.0, 5.0), (20.0, 15.0), (20.0, 5.0)];
    let mut ends = 0;
    for n in &r.nets {
        for w in n.r.wires.iter().filter(|w| !w.fill) {
            for e in [w.pts[0], *w.pts.last().unwrap()] {
                let near = centers.iter().any(|c| ((c.0 - e.0).powi(2) + (c.1 - e.1).powi(2)).sqrt() < 1.0);
                if near {
                    ends += 1;
                    assert!(
                        centers.iter().any(|c| ((c.0 - e.0).powi(2) + (c.1 - e.1).powi(2)).sqrt() < 1e-6),
                        "end {:?} is off-centre",
                        e
                    );
                }
            }
        }
    }
    assert!(ends >= 4);
    assert!(drc::check(&r).clearance_violations.is_empty());
}

#[test]
fn ses_omits_fill_by_default_and_power_nets_get_extra_vias() {
    let b = smd_board();
    let mut cfg = Config {
        verbose: false,
        write_fill: true,
        ..Config::default()
    };
    cfg.layers.insert(
        "bottom".into(),
        router::LayerPlan {
            role: router::LayerRole::Plane,
            nets: vec!["GND".into()],
        },
    );
    cfg.net_overrides.entry("GND".into()).or_default().via_count = Some(2);
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    let rep = drc::check(&r);
    assert!(rep.open_nets.is_empty() && rep.clearance_violations.is_empty());
    // three ground pads, two vias each
    assert_eq!(r.via_shortfall, 0);
    assert!(r.nets[0].r.vias.len() >= 6, "{}", r.nets[0].r.vias.len());
    let fill = r.nets[0].r.wires.iter().filter(|w| w.fill).count();
    assert!(fill > 0);
    let big = ses::write_ses(&r, "t");
    r.cfg.write_fill = false;
    let small = ses::write_ses(&r, "t");
    assert!(fill > 0);
    assert!(!r.fill_export.is_empty());
    assert_eq!(r.check_fill_export(), 0);
    assert_eq!(
        small.matches("(wire").count() + r.fill_export.len(),
        big.matches("(wire").count()
    );
}

#[test]
fn plane_copper_is_exported_as_clearance_safe_polygons() {
    let b = smd_board();
    let mut cfg = Config {
        verbose: false,
        ..Config::default()
    };
    cfg.layers.insert(
        "bottom".into(),
        router::LayerPlan {
            role: router::LayerRole::Plane,
            nets: vec!["GND".into()],
        },
    );
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    assert!(!r.fill_polys.is_empty());
    assert_eq!(r.check_fill_polygons(), 0);
    let text = ses::write_ses(&r, "t");
    assert!(text.contains("(polygon bottom 0"));
    // no thousands of strokes: the polygon replaces them
    assert!(text.matches("(wire").count() < 40);
}

#[test]
fn wide_tracks_export_without_polygons() {
    let b = smd_board();
    let mut cfg = Config { verbose: false, write_fill: true, write_polygons: false, ..Config::default() };
    cfg.layers.insert("bottom".into(), router::LayerPlan {
        role: router::LayerRole::Plane, nets: vec!["GND".into()],
    });
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    assert!(!r.fill_export.is_empty());
    assert!(r.fill_polys.is_empty());
    assert_eq!(r.check_fill_export(), 0);
    let report = drc::check(&r);
    assert!(report.open_nets.is_empty());
    assert!(report.clearance_violations.is_empty());
    let text = ses::write_ses(&r, "tracks");
    assert!(!text.contains("(polygon"));
    assert!(text.contains("(path bottom"));
}

#[test]
fn trace_routing_migrates_old_fill_settings_and_routes_real_copper() {
    let b = smd_board();
    let mut cfg = Config { verbose: false, write_fill: true, write_polygons: true, add_gnd_planes: true, ..Config::default() };
    cfg.layers.insert("bottom".into(), router::LayerPlan {
        role: router::LayerRole::Plane, nets: vec!["GND".into()],
    });
    cfg.use_trace_routing();
    assert_eq!(cfg.restart_attempts, 0);
    assert!(!cfg.write_fill && !cfg.write_polygons && !cfg.add_gnd_planes);
    assert_eq!(cfg.layers["bottom"].role, router::LayerRole::Signal);
    let mut r = Router::new(&b, cfg);
    let gnd = r.nets.iter().position(|n| n.name == "GND").unwrap();
    assert_eq!(r.routing_layer_costs(gnd), vec![6.0, 1.0]);
    assert!(!r.excl[1]);
    r.route_all(); r.refine(); r.finish();
    assert!(r.fill_export.is_empty() && r.fill_polys.is_empty());
    assert!(r.nets.iter().all(|n| n.r.wires.iter().all(|w| !w.fill)));
    assert!(r.nets[gnd].r.wires.iter().any(|w| w.layer == 1));
    let report = drc::check(&r);
    assert!(report.clearance_violations.is_empty(), "{:?}", report.clearance_violations);
    assert!(report.open_nets.is_empty(), "{:?}", report.open_nets);
    assert!(!ses::write_ses(&r, "t").contains("(polygon"));
}

#[test]
fn easyeda_json_round_trip_lands_on_the_pads() {
    let json = r#"{"head":{"docType":"3"},"layers":[],
      "DRCRULE":{"Default":{"trackWidth":1,"clearance":0.6,"viaHoleDiameter":2.4}},
      "shape":["TRACK~1~10~~100 100 220 100 220 160 100 160 100 100~gge1~0",
               "PAD~ELLIPSE~110~110~4~4~11~A~1~1~~0~gge2~0~~Y~0~0~0.2~110,110",
               "PAD~ELLIPSE~205~150~4~4~11~A~2~1~~0~gge3~0~~Y~0~0~0.2~205,150",
               "PAD~ELLIPSE~150~110~4~4~11~B~1~1~~0~gge4~0~~Y~0~0~0.2~150,110",
               "PAD~ELLIPSE~150~150~4~4~11~B~2~1~~0~gge5~0~~Y~0~0~0.2~150,150"]}"#;
    let doc = dsn::easyeda::load(json).unwrap();
    let board = dsn::parse_dsn(&doc.dsn).unwrap();
    let mut cfg = Config { verbose: false, unit_mm: Some(0.254), keep_track_widths: true, ..Config::default() };
    cfg.use_trace_routing();
    cfg.rules.min_trace_mm = 0.254;
    let mut r = Router::new(&board, cfg);
    r.route_all();
    r.finish();
    let (out, st) = router::easyeda_out::write_json(&r, &doc);
    assert!(st.tracks >= 2, "{st:?}");
    let v: serde_json::Value = serde_json::from_str(&out).unwrap();
    let shapes: Vec<&str> = v["shape"].as_array().unwrap().iter().map(|s| s.as_str().unwrap()).collect();
    assert_eq!(&shapes[..5], &serde_json::from_str::<serde_json::Value>(json).unwrap()["shape"].as_array().unwrap().iter().map(|s| s.as_str().unwrap()).collect::<Vec<_>>()[..]);
    // every net's tracks start and end on one of that net's pads (JSON coordinates)
    for net in ["A", "B"] {
        let pads: Vec<(f64, f64)> = if net == "A" { vec![(110.0, 110.0), (205.0, 150.0)] } else { vec![(150.0, 110.0), (150.0, 150.0)] };
        let ends: Vec<(f64, f64)> = shapes[5..]
            .iter()
            .filter(|s| s.starts_with("TRACK") && s.split('~').nth(3) == Some(net))
            .flat_map(|s| {
                let n: Vec<f64> = s.split('~').nth(4).unwrap().split(' ').map(|x| x.parse().unwrap()).collect();
                [(n[0], n[1]), (n[n.len() - 2], n[n.len() - 1])]
            })
            .collect();
        assert!(!ends.is_empty(), "no track for {net}");
        for e in &ends {
            let on_pad = pads.iter().any(|p| ((e.0 - p.0).powi(2) + (e.1 - p.1).powi(2)).sqrt() <= 2.0);
            let on_via = shapes.iter().any(|s| {
                let t: Vec<&str> = s.split('~').collect();
                t[0] == "VIA" && ((e.0 - t[1].parse::<f64>().unwrap()).powi(2) + (e.1 - t[2].parse::<f64>().unwrap()).powi(2)).sqrt() < 0.5
            });
            let joins_own = ends.iter().filter(|o| ((o.0 - e.0).powi(2) + (o.1 - e.1).powi(2)).sqrt() < 0.5).count() > 1;
            assert!(on_pad || on_via || joins_own, "{net}: track end {e:?} is not on a pad");
        }
    }
}

/// Two power nets share one plane layer and their vias sit closer together than the ~1 mm block
/// grid the outlines are traced on: every via must still lie completely inside its own area.
#[test]
fn plane_polygons_cover_every_via_of_their_net() {
    let b = dsn::parse_dsn(
        r#"(pcb pw (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 60 0 60 40 0 40))
       (layer top (type signal)) (layer bottom (type signal))
       (grid wire 0.5) (via via0) (rule (width 1) (clearance 0.5)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin smd 1 10 10) (pin smd 2 14 10) (pin smd 3 18 10) (pin smd 4 22 10)
                           (pin smd 5 10 30) (pin smd 6 50 30) (pin smd 7 30 20))
       (padstack smd (shape (circle top 2)))
       (padstack via0 (shape (circle top 1.5)) (shape (circle bottom 1.5))))
      (network (net P1 (pins U-1 U-3 U-5)) (net P2 (pins U-2 U-4 U-6)) (net GND (pins U-7))))"#,
    )
    .unwrap();
    let mut cfg = Config { verbose: false, unit_mm: Some(0.254), pour_polygons: true, ..Config::default() };
    cfg.layers.insert("bottom".into(), router::LayerPlan {
        role: router::LayerRole::Plane, nets: vec!["P1".into(), "P2".into(), "GND".into()],
    });
    cfg.use_trace_routing();
    cfg.rules.min_trace_mm = 0.254;
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    assert!(r.pour_warnings.is_empty(), "{:?}", r.pour_warnings);
    let bottom = r.layer_names.iter().position(|n| n == "bottom").unwrap();
    let mut checked = 0;
    for (ni, n) in r.nets.iter().enumerate() {
        let polys: Vec<&Vec<(f64, f64)>> = r.fill_polys.iter().filter(|p| p.0 == ni && p.1 == bottom).map(|p| &p.2).collect();
        if polys.is_empty() {
            continue;
        }
        for &(x, y) in &n.r.vias {
            // only vias that the area actually reaches are meant to connect to it
            if !polys.iter().any(|p| router::geom::point_in_poly((x, y), p)) {
                continue;
            }
            checked += 1;
            for k in 0..8 {
                let t = k as f64 * std::f64::consts::FRAC_PI_4;
                let q = (x + n.via_r * t.cos(), y + n.via_r * t.sin());
                assert!(
                    polys.iter().any(|p| router::geom::point_in_poly(q, p)),
                    "{}: via at ({x:.2},{y:.2}) sticks out of its copper area at {q:?}",
                    n.name
                );
            }
        }
    }
    assert!(checked > 0, "no via ended up inside a copper area");
}

/// The empty space of every layer is filled with ground; the fill areas on top of each other are
/// stitched, every via lies completely inside the area on each layer it touches, and no two vias
/// of one net overlap.
#[test]
fn empty_space_is_filled_and_stitched_with_covered_vias() {
    let b = dsn::parse_dsn(
        r#"(pcb pw (resolution mil 1000)
      (structure (boundary (path pcb 0 0 0 80 0 80 50 0 50))
       (layer top (type signal)) (layer bottom (type signal))
       (grid wire 0.5) (via via0) (rule (width 1) (clearance 0.5)))
      (placement (component img (place U 0 0 front 0)))
      (library (image img (pin smd 1 10 10) (pin smd 2 14 10) (pin smd 3 70 40) (pin smd 4 66 40)
                           (pin smd 5 10 40) (pin smd 6 40 25) (pin smd 7 70 10))
       (padstack smd (shape (circle top 2)))
       (padstack via0 (shape (circle top 1.5)) (shape (circle bottom 1.5))))
      (network (net P1 (pins U-1 U-2)) (net P2 (pins U-3 U-4)) (net GND (pins U-5 U-6 U-7))))"#,
    )
    .unwrap();
    let mut cfg = Config { verbose: false, unit_mm: Some(0.254), pour_polygons: true, ..Config::default() };
    cfg.use_trace_routing();
    cfg.rules.min_trace_mm = 0.254;
    cfg.stitch_pitch_mm = 3.0;
    let mut r = Router::new(&b, cfg);
    r.route_all();
    r.refine();
    r.finish();
    let gnd = r.nets.iter().position(|n| n.name == "GND").unwrap();
    let on = |l: usize| r.fill_polys.iter().filter(|p| p.0 == gnd && p.1 == l).count();
    assert!(on(0) > 0, "no ground fill on top");
    assert!(on(1) > 0, "no ground fill on bottom");
    let polys: Vec<Vec<&Vec<(f64, f64)>>> = (0..2)
        .map(|l| r.fill_polys.iter().filter(|p| p.0 == gnd && p.1 == l).map(|p| &p.2).collect())
        .collect();
    let vr = r.nets[gnd].via_r;
    let mut stitched = 0;
    for &(x, y) in &r.nets[gnd].r.vias {
        for l in 0..2 {
            let pts: Vec<(f64, f64)> = std::iter::once((x, y))
                .chain((0..8).map(|k| {
                    let t = k as f64 * std::f64::consts::FRAC_PI_4;
                    (x + vr * t.cos(), y + vr * t.sin())
                }))
                .collect();
            let inside = pts.iter().filter(|&&q| polys[l].iter().any(|p| router::geom::point_in_poly(q, p))).count();
            assert!(inside == 0 || inside == pts.len(), "via ({x:.2},{y:.2}) is only partly covered on layer {l}");
        }
        stitched += 1;
    }
    assert!(stitched >= 4, "only {stitched} vias");
    for n in &r.nets {
        for i in 0..n.r.vias.len() {
            for j in i + 1..n.r.vias.len() {
                let (a, c) = (n.r.vias[i], n.r.vias[j]);
                assert!(((a.0 - c.0).powi(2) + (a.1 - c.1).powi(2)).sqrt() >= 2.0 * n.via_r - 1e-6, "{}: vias overlap", n.name);
            }
        }
    }
    let report = drc::check(&r);
    assert!(report.clearance_violations.is_empty(), "{:?}", report.clearance_violations);
    assert!(report.open_nets.is_empty(), "{:?}", report.open_nets);
}

/// A wall with a narrow gap, and a small part sitting in the gap: the connection through the gap
/// can only be made when the part is nudged out of the way.
fn gap_board() -> String {
    let lib = "LIB~100~60~package`C0603`x`y~0~~gge20~1~u~1~0~~yes~~\
        #@$TEXT~P~100~66~0.6~0~0~3~~4.5~C1~M 100 66 L 101 67\
        #@$PAD~RECT~96~60~4~4~1~P~1~0~~0~gge21~0~~Y~0~0~0.2~96,60\
        #@$PAD~RECT~104~60~4~4~1~Q~2~0~~0~gge22~0~~Y~0~0~0.2~104,60";
    let wall = |layer: &str, id: u32, pts: &str| format!("\"TRACK~1~{layer}~WALL~{pts}~gge{id}~0\"");
    format!(
        r#"{{"head":{{"docType":"3"}},"layers":[],
      "DRCRULE":{{"Default":{{"trackWidth":1,"clearance":0.6,"viaHoleDiameter":2.4}}}},
      "shape":["TRACK~1~10~~0 0 200 0 200 120 0 120 0 0~gge1~0",
        "PAD~ELLIPSE~60~60~5~5~11~X~1~1.2~~0~gge2~0~~Y~0~0~0.2~60,60",
        "PAD~ELLIPSE~140~60~5~5~11~X~2~1.2~~0~gge3~0~~Y~0~0~0.2~140,60",
        {w1},{w2},{w3},"{lib}"]}}"#,
        w1 = wall("1", 10, "100 0 100 57"),
        w2 = wall("1", 11, "100 63 100 120"),
        w3 = wall("2", 12, "100 0 100 120"),
        lib = lib
    )
}

#[test]
fn small_parts_are_nudged_only_when_asked_and_only_when_a_route_is_missing() {
    let doc = dsn::easyeda::load(&gap_board()).unwrap();
    assert_eq!(doc.parts.len(), 1);
    assert!(doc.parts[0].smd && doc.parts[0].movable);
    let board = dsn::parse_dsn(&doc.dsn).unwrap();
    let make = |nudge: bool| {
        let mut cfg = Config { verbose: false, unit_mm: Some(0.254), keep_track_widths: true, nudge_parts: nudge, ..Config::default() };
        cfg.use_trace_routing();
        cfg.rules.min_trace_mm = 0.254;
        cfg
    };
    let parts = router::nudge::specs_from_doc(&doc);
    // off: the connection stays open and nothing moves
    let off = router::nudge::solve(&board, make(false), &parts, &[]);
    assert!(off.moves.is_empty() && off.log.is_empty());
    let rep = drc::check(&off.router);
    assert!(!rep.open_nets.is_empty() || off.router.unrouted_count() > 0, "the gap must be blocked without nudging");
    // on: the part is moved out of the gap (at most 3 part sizes) and the board is complete
    let on = router::nudge::solve(&board, make(true), &parts, &[]);
    let rep = drc::check(&on.router);
    assert!(rep.open_nets.is_empty() && on.router.unrouted_count() == 0, "{:?}", on.log);
    assert!(rep.clearance_violations.is_empty(), "{:?}", rep.clearance_violations);
    assert_eq!(on.moves.len(), 1, "{:?}", on.log);
    let (_, dx, dy) = on.moves[0];
    let size = 12.0; // two 4-unit pads 4 units apart
    assert!((dx * dx + dy * dy).sqrt() <= 3.0 * size + 1e-6);
    // the moved part is written back to the document: pads, text and header moved together
    let (json, _) = router::easyeda_out::write_json(&on.router, &doc);
    let v: serde_json::Value = serde_json::from_str(&json).unwrap();
    let lib = v["shape"].as_array().unwrap().iter().filter_map(|s| s.as_str()).find(|s| s.starts_with("LIB~")).unwrap();
    assert!(!lib.starts_with("LIB~100~60~"), "{lib}");
    assert!(lib.contains("TEXT~P~") && lib.contains("PAD~RECT~"));
    // never a part with wiring on it, never through-hole: the standalone pads did not move
    assert!(json.contains("PAD~ELLIPSE~60~60~") && json.contains("PAD~ELLIPSE~140~60~"));
}

/// Everything on the board that has no net of its own is kept clear and shown in the preview.
#[test]
fn objects_without_a_net_are_obstacles_and_in_the_preview() {
    let json = r#"{"head":{"docType":"3"},"layers":[],
      "DRCRULE":{"Default":{"trackWidth":1,"clearance":0.6,"viaHoleDiameter":2.4}},
      "shape":["TRACK~1~10~~0 0 200 0 200 120 0 120 0 0~gge1~0",
        "PAD~ELLIPSE~20~60~5~5~11~X~1~1.2~~0~gge2~0~~Y~0~0~0.2~20,60",
        "PAD~ELLIPSE~180~60~5~5~11~X~2~1.2~~0~gge3~0~~Y~0~0~0.2~180,60",
        "VIA~100~20~2.441~~0.61~gge4~0",
        "ARC~1~1~S$9678~M 100 40 A 5 5 0 0 1 110 40~~gge5~0",
        "RECT~100~60~6~6~1~gge6~0~0~~~~",
        "SOLIDREGION~1~~M 100 80 L 108 80 L 108 88 L 100 88 Z~solid~gge7~~~~0",
        "SOLIDREGION~11~~M 100 95 L 108 95 L 108 103 L 100 103 Z~npth~gge8~~~~0",
        "SOLIDREGION~10~~M 100 105 L 108 105 L 108 113 L 100 113 Z~cutout~gge9~~~~0",
        "HOLE~100~10~4~gge10~0"]}"#;
    let doc = dsn::easyeda::load(json).unwrap();
    let board = dsn::parse_dsn(&doc.dsn).unwrap();
    let mut cfg = Config { verbose: false, unit_mm: Some(0.254), keep_track_widths: true, ..Config::default() };
    cfg.use_trace_routing();
    let r = Router::new(&board, cfg);
    let pv = r.preview();
    let keep = pv.obstacles.iter().filter(|o| o.keepout).count();
    let copper = pv.obstacles.len() - keep;
    // kept clear: hole, npth region, board cut-out, and the copper rectangle / solid region without a net;
    // copper of no net: the via and the arc
    // (the arc is a chain of short pieces)
    assert!(copper >= 2, "{:?}", pv.obstacles.iter().map(|o| (&o.geo, &o.layers)).collect::<Vec<_>>());
    assert!(pv.obstacles.iter().any(|o| !o.keepout && matches!(o.geo, router::geom::Geo::Seg { .. })), "the arc is missing");
    assert_eq!(keep, 5);
    // and the router cannot go through any of them
    let mut r = r;
    r.route_all();
    r.finish();
    let blockers: Vec<(f64, f64)> = vec![(100.0, 20.0), (105.0, 40.0), (103.0, 63.0), (104.0, 84.0), (104.0, 99.0), (104.0, 109.0), (100.0, 10.0)];
    for n in &r.nets {
        for w in &n.r.wires {
            for s in w.pts.windows(2) {
                for b in &blockers {
                    let d = router::geom::dist_pt_seg(*b, s[0], s[1]);
                    assert!(d > 2.0, "a trace runs through the object at {b:?} (distance {d})");
                }
            }
        }
    }
    assert_eq!(router::drc::check(&r).open_nets.len(), 0);
}

/// An EasyEDA Pro project (STM32F767 board with a slot, regions, a pour, two routed nets): route
/// it, write it back as .epro2, load the result again.
#[test]
fn easyeda_pro_project_routes_and_round_trips() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../ProPrj_stm32f767_2026-11-17.epro2");
    let Ok(bytes) = std::fs::read(path) else { return };
    let doc = dsn::easyeda_pro::load(&bytes).unwrap();
    let board = dsn::parse_dsn(&doc.dsn).unwrap();
    let mut cfg = Config { verbose: false, unit_mm: Some(0.254), keep_track_widths: true, pour_polygons: true, ..Config::default() };
    cfg.use_trace_routing();
    let mut r = Router::new(&board, cfg);
    r.route_all();
    r.finish();
    let report = drc::check(&r);
    assert!(report.clearance_violations.is_empty(), "{:?}", report.clearance_violations);
    assert!(report.open_nets.is_empty(), "{:?}", report.open_nets);

    let (out, st) = router::easyeda_out::write_output(&r, &doc).unwrap();
    assert!(st.tracks > 100 && st.vias > 10, "{st:?}");
    let again = dsn::easyeda_pro::load(&out).unwrap();
    let b2 = dsn::parse_dsn(&again.dsn).unwrap();
    // everything of the original project is still there, plus the new tracks and vias
    assert_eq!(b2.wires.len(), board.wires.len() + st.tracks);
    assert_eq!(b2.vias.len(), board.vias.len() + st.vias);
    assert_eq!(b2.abs_pins().len(), board.abs_pins().len());
    assert_eq!(b2.nets.len(), board.nets.len());

    // the slot region (x 420..560 mil, y 715..860 mil) stays free of new copper
    let zip_text = {
        use std::io::Read;
        let mut z = zip::ZipArchive::new(std::io::Cursor::new(&out)).unwrap();
        let mut t = String::new();
        z.by_name("stm32f767.epru").unwrap().read_to_string(&mut t).unwrap();
        t
    };
    let old_text = {
        use std::io::Read;
        let mut z = zip::ZipArchive::new(std::io::Cursor::new(&bytes)).unwrap();
        let mut t = String::new();
        z.by_name("stm32f767.epru").unwrap().read_to_string(&mut t).unwrap();
        t
    };
    let old: std::collections::HashSet<&str> = old_text.split('\n').collect();
    let new_lines: Vec<&str> = zip_text.split('\n').filter(|l| !old.contains(l) && l.contains("}||")).collect();
    assert_eq!(zip_text.split('\n').filter(|l| old.contains(l)).count() + new_lines.len(), zip_text.split('\n').count());
    assert_eq!(new_lines.iter().filter(|l| l.contains("\"type\":\"LINE\"")).count(), st.tracks);
    assert_eq!(new_lines.iter().filter(|l| l.contains("\"type\":\"VIA\"")).count(), st.vias);
    let in_slot = |x: f64, y: f64| (420.0..=560.0).contains(&x) && (715.0..=860.0).contains(&y);
    for l in new_lines.iter().filter(|l| l.contains("\"type\":\"LINE\"") || l.contains("\"type\":\"VIA\"")) {
        let v: serde_json::Value = serde_json::from_str(l.split("}||").nth(1).unwrap().trim_end_matches('|')).unwrap();
        let g = |k: &str| v[k].as_f64();
        let pts = [(g("startX"), g("startY")), (g("endX"), g("endY")), (g("centerX"), g("centerY"))];
        for (x, y) in pts.into_iter().filter_map(|(x, y)| Some((x?, y?))) {
            assert!(!in_slot(x, y), "new copper inside the slot at {x},{y}: {l}");
        }
    }
}
