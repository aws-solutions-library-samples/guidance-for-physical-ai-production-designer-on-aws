//! Random bt2plc programs (series-parallel and WaitForRobot trees as in
//! ../bt2plc/tests/proptest_trees.rs, with random joint trajectories): plan -> program.json -> bt2krl.
//!
//! Properties: generation succeeds (so the strict validator and the IR round trip passed); steps
//! are 1..N in order; REQUEST/GRANT masks equal the program; every grant wait comes before the
//! op's first motion and the release after its last motion and hold; the last point of every op
//! is an exact stop; all joint values are within the model limits; the output is deterministic.
//! Decimation keeps the endpoints and stays within tolerance (separate property).

mod common;
use bt2krl::ir::Instr;
use bt2krl::motion;
use bt2krl::program::Waypoint;
use bt2krl::{Options, model};
use common::Rng;
use common::*;
use proptest::prelude::*;
use serde_json::Value;

fn sp(t: &mut Tree, rng: &mut Rng, robots: &[usize], depth: u32, ctr: &mut usize) -> Value {
    let leaf = depth == 0 || rng.below(4) == 0;
    if leaf {
        let r = robots[rng.below(robots.len() as u64) as usize];
        *ctr += 1;
        return t.exec(
            &format!("r{r}"),
            &format!("O{ctr}"),
            0.2 + rng.below(20) as f64 / 10.0,
        );
    }
    if robots.len() >= 2 && rng.below(2) == 0 {
        let mut parts: Vec<Vec<usize>> = Vec::new();
        for &r in robots {
            let k = rng.below(parts.len() as u64 + 1) as usize;
            if k == parts.len() {
                parts.push(vec![r]);
            } else {
                parts[k].push(r);
            }
        }
        if parts.len() >= 2 {
            return par(parts
                .iter()
                .map(|p| sp(t, rng, p, depth - 1, ctr))
                .collect());
        }
    }
    let n = 1 + rng.below(3);
    seq((0..n).map(|_| sp(t, rng, robots, depth - 1, ctr)).collect())
}

fn coordinated(t: &mut Tree, rng: &mut Rng, nr: usize, blind: bool) -> Value {
    let n_items = 4 + rng.below(20) as usize;
    let items: Vec<usize> = (0..n_items)
        .map(|_| rng.below(nr as u64) as usize)
        .collect();
    let mut per_robot: Vec<Vec<Value>> = vec![Vec::new(); nr];
    for (pos, &r) in items.iter().enumerate() {
        let mut deps = Vec::new();
        for (q, &rq) in items.iter().enumerate().take(pos) {
            if rq != r && rng.below(5) == 0 {
                deps.push((format!("r{rq}"), format!("S{q}")));
            }
        }
        if !deps.is_empty() {
            let d: Vec<(&str, &str)> = deps.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
            per_robot[r].push(t.wait(&format!("r{r}"), &d));
        }
        per_robot[r].push(t.exec(
            &format!("r{r}"),
            &format!("S{pos}"),
            0.2 + rng.below(10) as f64 / 10.0,
        ));
    }
    let branches: Vec<Value> = per_robot
        .into_iter()
        .filter(|b| !b.is_empty())
        .map(seq)
        .collect();
    let homes: Vec<Value> = (0..nr)
        .map(|r| {
            if blind {
                t.blind_home(&format!("r{r}"))
            } else {
                t.home(&format!("r{r}"))
            }
        })
        .collect();
    seq(vec![par(branches), par(homes)])
}

fn check(text: &str) {
    let mut plc = bt2plc::Config::default();
    // Small cap so that some trees overflow into the permit bit.
    plc.protocol.max_zones = 3;
    let programs = programs(text, &plc);
    let ns: Vec<String> = programs.iter().map(|p| p.robot.ns.clone()).collect();
    let cfg = cfg_for(&ns.iter().map(String::as_str).collect::<Vec<_>>());
    let g = bt2krl::generate(&programs, &cfg, &Options::default())
        .unwrap_or_else(|e| panic!("bt2krl rejects a valid program: {e}"));
    let g2 = bt2krl::generate(&programs, &cfg, &Options::default()).unwrap();
    assert_eq!(g.output.files(), g2.output.files(), "not deterministic");
    let kr240 = model::lookup("KR240R2900_ULTRA").unwrap();
    for (r, p) in g.robots.iter().zip(&programs) {
        let f = &r.lowered.prod;
        assert_eq!(f.ops.len(), p.ops.len());
        for pt in &f.points {
            assert_eq!(kr240.violation(&pt.q), None, "{} out of limits", pt.name);
        }
        for (k, (o, po)) in f.ops.iter().zip(&p.ops).enumerate() {
            assert_eq!(o.index as usize, k + 1);
            assert_eq!(o.mask, po.mask(), "mask of op {}", k + 1);
            let pos = |pred: &dyn Fn(&Instr) -> bool| o.instrs.iter().position(pred);
            let first_move = pos(&|i| matches!(i, Instr::Move { .. })).expect("every op moves");
            let last_move = o
                .instrs
                .iter()
                .rposition(|i| matches!(i, Instr::Move { .. }))
                .unwrap();
            assert!(
                matches!(o.instrs[last_move], Instr::Move { approx: false, .. }),
                "exact stop"
            );
            assert_eq!(o.instrs[1], Instr::SetStep(o.index));
            if o.mask != 0 {
                let req = pos(&|i| *i == Instr::Request(o.mask)).expect("request");
                let grant = pos(&|i| *i == Instr::WaitGrant(o.mask)).expect("grant");
                let rel = pos(&|i| *i == Instr::Release(o.mask)).expect("release");
                let wrel = pos(&|i| *i == Instr::WaitReleased(o.mask)).expect("4-phase");
                assert!(req < grant && grant < first_move && last_move < rel && rel < wrel);
                if let Some(h) = pos(&|i| matches!(i, Instr::Hold(_))) {
                    assert!(last_move < h && h < rel);
                }
            } else {
                assert!(
                    !o.instrs
                        .iter()
                        .any(|i| matches!(i, Instr::Request(_) | Instr::WaitGrant(_)))
                );
            }
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn series_parallel_programs(seed in any::<u64>(), nr in 2usize..=4, depth in 1u32..=4) {
        let mut rng = Rng(seed);
        let mut t = Tree { seed, ..Default::default() };
        let robots: Vec<usize> = (0..nr).collect();
        let mut ctr = 0;
        let root = sp(&mut t, &mut rng, &robots, depth, &mut ctr);
        check(&t.envelope(seq(vec![root])));
    }

    #[test]
    fn coordinated_programs(seed in any::<u64>(), nr in 2usize..=4, blind in any::<bool>()) {
        let mut rng = Rng(seed);
        let mut t = Tree { seed, ..Default::default() };
        let root = coordinated(&mut t, &mut rng, nr, blind);
        check(&t.envelope(root));
    }

    #[test]
    fn decimation_keeps_endpoints_and_tolerance(
        seed in any::<u64>(), n in 2usize..60, tol in 0.0f64..3.0,
    ) {
        let mut rng = Rng(seed);
        let mut q = [0.0f64; 6];
        let w: Vec<Waypoint> = (0..n).map(|i| {
            for v in &mut q { *v += rng.unit() * 2.0; }
            Waypoint { t_s: i as f64 * 0.1, q_deg: q, phase: if rng.below(8) == 0 { "approach".into() } else { "transit".into() } }
        }).collect();
        let keep = motion::decimate(&w, tol);
        prop_assert_eq!(keep[0], 0);
        prop_assert_eq!(*keep.last().unwrap(), n - 1);
        prop_assert!(keep.windows(2).all(|p| p[0] < p[1]));
        for i in 1..n {
            if w[i].phase != w[i - 1].phase {
                prop_assert!(keep.contains(&i) && keep.contains(&(i - 1)));
            }
        }
        for s in keep.windows(2) {
            for i in s[0] + 1..s[1] {
                prop_assert!(motion::chord_dev(&w, s[0], s[1], i) <= tol + 1e-9);
            }
        }
    }
}
