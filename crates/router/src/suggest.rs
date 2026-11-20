//! When connections stay open: other settings that were tried on the board and what they gave.
//!
//! Every suggestion has been run: it is only offered when it leaves fewer connections open than the
//! settings that were used, and the result says by how much.

use crate::engine::{Config, DrcRules, LayerRole};
use crate::nudge::{solve, PartSpec};
use crate::drc;
use dsn::Board;

/// One setting that can be changed.
#[derive(Clone, Debug)]
pub enum Change {
    /// A net leaves a plane layer: it is routed with traces (and may get a copper area on a
    /// signal layer) instead of sharing the plane layer.
    RemoveFromPlane { net: String, layer: String },
    /// Vias may sit half over the edge of a pad.
    ViasAtPadEdge,
    /// Vias may sit inside pads.
    AllowViasInPads,
    /// Replace the design rules by a preset (`DrcRules::preset`).
    RulesPreset(String),
    /// Small parts may be moved a little to make room.
    NudgeParts,
}

impl Change {
    pub fn apply(&self, cfg: &mut Config) {
        match self {
            Change::RemoveFromPlane { net, layer } => {
                if let Some(p) = cfg.layers.get_mut(layer) {
                    p.nets.retain(|n| n != net);
                }
            }
            Change::ViasAtPadEdge => cfg.vias_at_pad_edge = true,
            Change::AllowViasInPads => cfg.allow_vias_in_pads = true,
            Change::RulesPreset(name) => {
                if let Some(r) = DrcRules::preset(name) {
                    cfg.rules = r;
                }
            }
            Change::NudgeParts => cfg.nudge_parts = true,
        }
    }

    pub fn describe(&self) -> String {
        match self {
            Change::RemoveFromPlane { net, layer } => {
                format!("take {net} off the plane on layer {layer} (Layers: remove it from the plane's nets), it is then routed with traces")
            }
            Change::ViasAtPadEdge => "set 'Vias on pads' to 'half over the edge'".to_string(),
            Change::AllowViasInPads => "set 'Vias on pads' to 'inside the pad' (may cost extra at the fabricator)".to_string(),
            Change::RulesPreset(name) => format!("use the '{name}' design rule preset"),
            Change::NudgeParts => "tick 'move small passives a little when connections stay open' (Last resorts)".to_string(),
        }
    }
}

/// A tested set of changes.
#[derive(Clone, Debug)]
pub struct Suggestion {
    pub changes: Vec<Change>,
    /// open connections with the used settings / with this suggestion
    pub open_before: usize,
    pub open_after: usize,
    /// clearance violations of the new result
    pub violations: usize,
    pub seconds: f32,
}

impl Suggestion {
    pub fn text(&self) -> String {
        let what = self.changes.iter().map(|c| c.describe()).collect::<Vec<_>>().join("; ");
        let outcome = if self.open_after == 0 {
            "routes everything".to_string()
        } else {
            format!("{} connection(s) open instead of {}", self.open_after, self.open_before)
        };
        format!("{what}: tested, {outcome}, {} clearance violation(s), {:.0}s", self.violations, self.seconds)
    }
}

/// Settings worth trying for the nets named in `open_nets` (nets with an open connection), tested
/// in order; the first that routes everything ends the search. At most `max_runs` runs are made.
/// Suggestions that do not beat `open_before` are left out.
pub fn suggest(
    board: &Board,
    cfg: &Config,
    parts: &[PartSpec],
    skip: &[(String, String)],
    open_before: usize,
    open_nets: &[String],
    max_runs: usize,
) -> Vec<Suggestion> {
    let mut candidates: Vec<Vec<Change>> = vec![];
    // 1. open nets that share a plane layer: route them with traces instead
    let mut off: Vec<Change> = vec![];
    let mut layers: Vec<(&String, &crate::engine::LayerPlan)> = cfg.layers.iter().collect();
    layers.sort_by(|a, b| a.0.cmp(b.0));
    for (name, plan) in layers {
        if plan.role != LayerRole::Plane {
            continue;
        }
        for n in &plan.nets {
            if open_nets.iter().any(|o| o == n) {
                off.push(Change::RemoveFromPlane { net: n.clone(), layer: name.clone() });
            }
        }
    }
    if !off.is_empty() {
        // all together first, then one by one when there are several
        if off.len() > 1 {
            candidates.push(off.clone());
        }
        for c in off {
            candidates.push(vec![c]);
        }
    }
    // then, ordered by what they cost the user: nudging small parts, vias inside pads, looser rules
    if !cfg.nudge_parts && !parts.is_empty() {
        candidates.push(vec![Change::NudgeParts]);
    }
    if !cfg.allow_vias_in_pads && !cfg.vias_at_pad_edge {
        candidates.push(vec![Change::ViasAtPadEdge]);
    }
    if !cfg.allow_vias_in_pads {
        candidates.push(vec![Change::AllowViasInPads]);
    }
    if let Some(m) = DrcRules::preset("minimum") {
        let r = &cfg.rules;
        if r.min_trace_mm > m.min_trace_mm + 1e-9
            || r.min_clearance_mm > m.min_clearance_mm + 1e-9
            || r.min_via_diameter_mm > m.min_via_diameter_mm + 1e-9
        {
            candidates.push(vec![Change::RulesPreset("minimum".into())]);
        }
    }
    let mut out: Vec<Suggestion> = vec![];
    let started = std::time::Instant::now();
    for changes in candidates.into_iter().take(max_runs) {
        if cfg.cancel.as_ref().is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed)) {
            break;
        }
        // no new test after two minutes
        if started.elapsed().as_secs() > 120 {
            break;
        }
        let mut c2 = cfg.clone();
        c2.on_snapshot = None;
        c2.snapshot_dir = None;
        c2.verbose = false;
        // The tests run without moving parts (that is what takes long); a suggestion that helps
        // without nudging helps with it too. Only the suggestion to nudge turns it on.
        c2.nudge_parts = false;
        for ch in &changes {
            ch.apply(&mut c2);
        }
        let t = std::time::Instant::now();
        let solved = solve(board, c2, parts, skip);
        let r = &solved.router;
        let rep = drc::check(r);
        let open_after = r.unrouted_count().max(rep.open_nets.len());
        if open_after < open_before {
            out.push(Suggestion {
                changes,
                open_before,
                open_after,
                violations: rep.clearance_violations.len(),
                seconds: t.elapsed().as_secs_f32(),
            });
            if open_after == 0 {
                break;
            }
        }
    }
    out.sort_by_key(|s| (s.open_after, s.violations));
    out
}
