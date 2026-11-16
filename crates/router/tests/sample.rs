use router::{drc, Config, Router};
use std::path::Path;

fn load(name: &str) -> Option<dsn::Board> {
    let p = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(name);
    let s = std::fs::read_to_string(p).ok()?;
    Some(dsn::parse_dsn(&s).unwrap())
}

fn run(b: &dsn::Board, cfg: Config) -> Router {
    let mut r = Router::new(
        b,
        Config {
            verbose: false,
            refine_passes: 1,
            ..cfg
        },
    );
    r.route_all();
    r.refine();
    r
}

#[test]
fn sample_fully_routed_and_drc_clean() {
    let Some(b) = load("Autorouter_PCB_Curvetracer1_2026-11-15.dsn") else {
        return;
    };
    let r = run(&b, Config::default());
    assert_eq!(r.unrouted_count(), 0);
    let rep = drc::check(&r);
    assert!(
        rep.clearance_violations.is_empty(),
        "{:?}",
        rep.clearance_violations
    );
    assert!(rep.open_nets.is_empty(), "{:?}", rep.open_nets);
}

#[test]
fn exclusive_layers_get_no_traces() {
    let Some(b) = load("Autorouter_PCB_Curvetracer1_2026-11-15.dsn") else {
        return;
    };
    let r = run(
        &b,
        Config {
            exclusive: vec!["2".into(), "21".into()],
            ..Config::default()
        },
    );
    assert_eq!(r.unrouted_count(), 0);
    for n in &r.nets {
        for w in &n.r.wires {
            assert!(
                r.layer_names[w.layer] == "1" || r.layer_names[w.layer] == "22",
                "wire on layer {}",
                r.layer_names[w.layer]
            );
        }
    }
    assert!(drc::check(&r).clearance_violations.is_empty());
}

#[test]
fn differential_pair_is_coupled() {
    let Some(b) = load("tests/pair_test.dsn") else {
        return;
    };
    let r = run(&b, Config::default());
    assert_eq!(r.unrouted_count(), 0);
    let pr = r.pair_report();
    assert_eq!(pr.len(), 1);
    assert!(pr[0].coupled, "pair fell back to independent routing");
    assert!(
        (pr[0].len_p - pr[0].len_n).abs() < 1.0,
        "skew {}",
        (pr[0].len_p - pr[0].len_n).abs()
    );
    assert!(drc::check(&r).clearance_violations.is_empty());
}

#[test]
fn prefix_polarity_nets_are_not_pairs() {
    let Some(b) = load("Autorouter_PCB_Curvetracer1_2026-11-15.dsn") else {
        return;
    };
    assert!(dsn::find_diff_pairs(&b.nets).is_empty());
}

#[test]
fn existing_wiring_is_kept_and_respected() {
    let Some(b) = load("tests/prewired_test.dsn") else {
        return;
    };
    let r = run(&b, Config::default());
    assert_eq!(r.unrouted_count(), 0);
    // every pre-existing wire is still there, untouched
    let kept: usize = r.nets.iter().map(|n| n.pre.wires.len()).sum();
    assert_eq!(kept, b.wires.len());
    for w in &b.wires {
        let n = r.nets.iter().find(|n| n.name == w.net).unwrap();
        assert!(n
            .pre
            .wires
            .iter()
            .any(|x| x.pts == w.pts && (x.width - w.width).abs() < 1e-9));
    }
    // the existing traces already solved some connections
    assert!(r.connection_count() < 146);
    let rep = drc::check(&r);
    assert!(
        rep.clearance_violations.is_empty(),
        "{:?}",
        rep.clearance_violations
    );
    assert!(rep.open_nets.is_empty(), "{:?}", rep.open_nets);
}

#[test]
fn plane_pins_are_connected_and_plane_stays_whole() {
    let Some(b) = load("tests/plane_test.dsn") else {
        return;
    };
    let r = run(&b, Config::default());
    assert_eq!(r.unrouted_count(), 0);
    assert!(
        r.planes[0].members.len() > 20,
        "THT GND pins should sit on the plane"
    );
    for p in r.plane_report() {
        assert_eq!(p.islands, 1, "plane cut into pieces");
        assert_eq!(p.orphans, 0, "plane has dead islands");
    }
    let rep = drc::check(&r);
    assert!(rep.clearance_violations.is_empty() && rep.open_nets.is_empty());
}

#[test]
fn exclusive_plane_gets_no_traces() {
    let Some(b) = load("tests/plane_test.dsn") else {
        return;
    };
    let r = run(
        &b,
        Config {
            exclusive_planes: vec!["GND@22".into()],
            ..Config::default()
        },
    );
    assert_eq!(r.unrouted_count(), 0);
    for n in &r.nets {
        for w in &n.r.wires {
            assert_ne!(
                r.layer_names[w.layer], "22",
                "trace routed inside an exclusive plane ({})",
                n.name
            );
        }
    }
    assert!(drc::check(&r).clearance_violations.is_empty());
}

#[test]
fn skipped_nets_stay_unrouted() {
    let Some(b) = load("Autorouter_PCB_Curvetracer1_2026-11-15.dsn") else {
        return;
    };
    let mut cfg = Config::default();
    cfg.net_overrides.insert(
        "GND".into(),
        router::NetOverride {
            skip: true,
            ..Default::default()
        },
    );
    let r = run(&b, cfg);
    let gnd = r.nets.iter().find(|n| n.name == "GND").unwrap();
    assert!(gnd.r.wires.is_empty());
    assert_eq!(r.unrouted_count(), 0);
}
