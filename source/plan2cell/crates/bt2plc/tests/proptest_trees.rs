//! Random series-parallel and coordinated (WaitForRobot) trees, 2-4 robots.
//!
//! Properties (all on the IR; `ir::validate` additionally runs inside `plan`):
//! - the pipeline accepts every well-formed tree;
//! - closure(program order ∪ zone tickets ∪ permits) == BT happens-before (exact equivalence);
//! - BT-concurrent ops never share a zone bit;
//! - at most 31 zones; overflow goes to permits;
//! - deterministic output.

mod common;
use bt2plc::Config;
use bt2plc::ir::PERMIT_BIT;
use common::*;
use proptest::prelude::*;
use serde_json::Value;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
}

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
        // Parallel over a random partition of the robots.
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

fn coordinated(t: &mut Tree, rng: &mut Rng, nr: usize) -> Value {
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
    let homes: Vec<Value> = (0..nr).map(|r| t.home(&format!("r{r}"))).collect();
    seq(vec![par(branches), par(homes)])
}

fn check(text: &str) {
    let cfg = Config::default();
    let (p, _) = plan_json(text, &cfg).expect("pipeline accepts a well-formed tree");
    assert_eq!(
        bt2plc::ir::validate::enforced_closure(&p),
        p.hb.anc,
        "zone protocol order != BT happens-before"
    );
    assert!(p.zones.len() <= 31);
    for a in 0..p.ops.len() {
        for b in a + 1..p.ops.len() {
            if p.hb.concurrent(a, b) {
                assert_eq!(
                    p.ops[a].zone_mask & p.ops[b].zone_mask & !PERMIT_BIT,
                    0,
                    "{} and {} share a zone",
                    p.op_ref(a),
                    p.op_ref(b)
                );
            }
        }
    }
    let (p2, _) = plan_json(text, &cfg).unwrap();
    let d = bt2plc::Diagnostics::default();
    assert_eq!(
        bt2plc::backend::common::cell_plan_json::render(&p, &cfg, &d),
        bt2plc::backend::common::cell_plan_json::render(&p2, &cfg, &d)
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn series_parallel_trees(seed in any::<u64>(), nr in 2usize..=4, depth in 1u32..=4) {
        let mut rng = Rng(seed);
        let mut t = Tree::default();
        let robots: Vec<usize> = (0..nr).collect();
        let mut ctr = 0;
        let root = sp(&mut t, &mut rng, &robots, depth, &mut ctr);
        check(&t.envelope(seq(vec![root])));
    }

    #[test]
    fn coordinated_trees(seed in any::<u64>(), nr in 2usize..=4) {
        let mut rng = Rng(seed);
        let mut t = Tree::default();
        let root = coordinated(&mut t, &mut rng, nr);
        check(&t.envelope(root));
    }
}
