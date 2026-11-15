//! Frame-level checks of the vw-ks24 executor (no PLC): the signal map on the wire, the VW job
//! handshake, and a Rust twin of the head's interlock arbitration. These run in milliseconds and
//! are the fast counterpart of the trust co-simulation in `tests/cosim.rs`.
#![allow(clippy::needless_range_loop)] // zone / robot indices are used across several arrays
mod common;
use common::*;

fn exec() -> (bt2krl::exec::Executor, bt2krl::program::Program) {
    exec_of(0)
}

fn exec_of(ri: usize) -> (bt2krl::exec::Executor, bt2krl::program::Program) {
    let programs = bt2krl::load_bt2plc_dir(&plc_dir("welding_cell_2_small")).unwrap();
    let p = programs[ri].clone();
    let cfg = vw_cfg();
    let s = bt2krl::lower::RobotSetup::resolve(&p, &cfg, true).unwrap();
    let l = bt2krl::lower::lower(&p, &s, &cfg, bt2krl::Profile::VwKs24).unwrap();
    (bt2krl::exec::Executor::new(&p, &s, &cfg, &l).unwrap(), p)
}

fn bit(f: &[u8; 32], i: u16) -> bool {
    let i = usize::from(i) - 1;
    f[i / 8] & (1 << (i % 8)) != 0
}

#[test]
fn idle_frame_has_the_vw_signal_map() {
    let (mut e, _p) = exec();
    let (f, _s) = e.scan(20, &[0; 32], &[0x01, 0, 0, 0, 0, 0, 0, 0]);
    // At home, releases set, no request, no echo, no step, no Folge end.
    assert!(bit(&f, 15), "PF0 must be set at home");
    assert!(bit(&f, 41) && bit(&f, 51), "interlock releases must be set");
    assert!(!bit(&f, 81) && !bit(&f, 91), "no request while idle");
    assert!(!bit(&f, 14), "no Folge end while idle");
    assert_eq!(f[0], 0, "PRG_NR_REFL must be 0");
    let step = u16::from(f[12]) | (u16::from(f[13]) << 8);
    assert_eq!(
        step, 0,
        "op index must be 0 (bytes 12-13 = $OUT[97..112]); frame {f:?}"
    );
    assert_eq!(f[28], 0, "error code must be 0");
    // Nothing outside the signals the profile uses.
    for (i, b) in f.iter().enumerate() {
        let allowed = matches!(i, 0 | 1 | 5 | 6 | 10 | 11 | 12 | 13 | 16..=27 | 28);
        assert!(
            allowed || *b == 0,
            "byte {i} = {b:#04x} is not part of the profile"
        );
    }
}

#[test]
fn drives_and_ready_are_separate_signals() {
    let (mut e, _p) = exec();
    let mut cmd = [0u8; 32];
    cmd[1] |= 1 << 1; // E10 drives on
    let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
    for _ in 0..200 {
        e.scan(20, &cmd, &safe);
    }
    let (f, _) = e.scan(20, &cmd, &safe);
    assert!(
        bit(&f, 10),
        "A10 (RK100, safety circuit) must be set with the drives on"
    );
    assert!(bit(&f, 9), "A9 (ready) must be set while idle at home");
    assert!(bit(&f, 12), "A12 (auto) must be set");
}

/// A minimal VW-side "head" for fast iteration: PRG_NR / SRB / echo / Folge end, grants and
/// partner releases. It is NOT the generated PLC (that is tests/cosim.rs); it only exercises the
/// executor's half of the handshake.
struct MiniHead {
    plan_id: u16,
    job: u16,
    srb: bool,
    grant: u32,
    partner: u32,
    drives: bool,
}

impl MiniHead {
    fn cmd(&self) -> [u8; 32] {
        let mut c = [0u8; 32];
        let mut set = |i: u16| {
            let i = usize::from(i) - 1;
            c[i / 8] |= 1 << (i % 8);
        };
        if self.drives {
            set(10); // E10 drives
        }
        if self.srb {
            set(9); // E9 SRB
        }
        for n in 1..=16u16 {
            if self.grant & (1 << n) != 0 {
                set(80 + n);
            }
            if self.partner & (1 << n) != 0 {
                set(40 + n);
            }
        }
        c[0] = self.job as u8;
        // PLAN_ID on the free byte pair 12-13 (the test config checks it)
        c[12] = self.plan_id as u8;
        c[13] = (self.plan_id >> 8) as u8;
        c
    }
}

#[test]
fn a_full_vw_job_handshake_runs_and_needs_the_grants() {
    let (mut e, p) = exec();
    let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
    let mut h = MiniHead {
        plan_id: p.plan_id(),
        job: 0,
        srb: false,
        grant: 0,
        partner: u32::MAX, // partners always clear
        drives: true,
    };
    // bring-up: A10 at once, A12 after the drive start-up, A9 ready at home
    let mut f = [0u8; 32];
    for _ in 0..200 {
        f = e.scan(20, &h.cmd(), &safe).0;
    }
    assert!(
        bit(&f, 10) && bit(&f, 12) && bit(&f, 9) && bit(&f, 15),
        "bring-up: {f:?}"
    );

    // select the production Folge: the robot mirrors it on PRG_NR_REFL and waits for SRB
    h.job = p.folgen.production;
    for _ in 0..20 {
        f = e.scan(20, &h.cmd(), &safe).0;
    }
    assert_eq!(
        f[0], p.folgen.production as u8,
        "PRG_NR_REFL must mirror PRG_NR"
    );
    assert!(bit(&f, 15), "the robot must still be at home before SRB");
    let step = |f: &[u8; 32]| u16::from(f[12]) | (u16::from(f[13]) << 8);
    assert_eq!(step(&f), 0);

    // SRB: the cycle starts (PF0 off), op 1 is reported and its interlock requested
    h.srb = true;
    for _ in 0..10 {
        f = e.scan(20, &h.cmd(), &safe).0;
    }
    assert!(
        !bit(&f, 15),
        "PF0 must drop at the cycle start (at {:?}, error {:#06x})",
        e.current(),
        e.error_code()
    );
    assert_eq!(step(&f), 1, "op 1 must be reported");
    assert!(bit(&f, 81), "op 1 requests interlock V1 (zone IL01)");
    // Without the grant the robot must not move.
    let pose = e.pose();
    for _ in 0..100 {
        e.scan(20, &h.cmd(), &safe);
    }
    assert_eq!(e.pose(), pose, "no motion without the PLC grant");
    assert_eq!(e.step(), 1);

    // Grant V1 and V11 (IL01, IL02): the cycle runs to the Folge end.
    h.grant = (1 << 1) | (1 << 11);
    let mut ended = false;
    for _ in 0..4000 {
        f = e.scan(20, &h.cmd(), &safe).0;
        if bit(&f, 14) {
            ended = true;
            break;
        }
    }
    assert!(
        ended,
        "the Folge must reach its end (A14); step {} ",
        e.step()
    );
    assert_eq!(step(&f), 0, "the op index must be 0 at the Folge end");
    assert!(
        e.pose()
            .iter()
            .zip(p.home_pose_deg)
            .all(|(a, b)| (a - b).abs() < 0.5),
        "the last op must end at home: {:?}",
        e.pose()
    );

    // Clearing PRG_NR ends the job: A14 falls and the robot reports home again.
    h.job = 0;
    h.srb = false;
    for _ in 0..20 {
        f = e.scan(20, &h.cmd(), &safe).0;
    }
    assert!(!bit(&f, 14) && f[0] == 0, "Folge end and echo must clear");
    assert!(
        bit(&f, 15) && bit(&f, 9),
        "the robot is at home and ready again"
    );

    // A second cycle must run as well.
    h.job = p.folgen.production;
    h.srb = true;
    let mut ended2 = false;
    for _ in 0..4000 {
        f = e.scan(20, &h.cmd(), &safe).0;
        if bit(&f, 14) {
            ended2 = true;
            break;
        }
    }
    assert!(ended2, "a second cycle must run");
}

#[test]
fn a_withdrawn_partner_release_stops_the_robot_on_its_path() {
    let (mut e, p) = exec();
    let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
    let mut h = MiniHead {
        plan_id: p.plan_id(),
        job: 0,
        srb: false,
        grant: (1 << 1) | (1 << 11),
        partner: u32::MAX,
        drives: true,
    };
    for _ in 0..200 {
        e.scan(20, &h.cmd(), &safe);
    }
    h.job = p.folgen.production;
    h.srb = true;
    for _ in 0..200 {
        e.scan(20, &h.cmd(), &safe);
    }
    assert!(e.step() >= 1);
    // Withdraw the partner release of the interlock the robot holds: it must stop moving.
    h.partner = 0;
    for _ in 0..5 {
        e.scan(20, &h.cmd(), &safe);
    }
    let pose = e.pose();
    for _ in 0..100 {
        e.scan(20, &h.cmd(), &safe);
    }
    assert_eq!(
        e.pose(),
        pose,
        "the online driving condition must stop the robot"
    );
    // Restoring it continues the cycle.
    h.partner = u32::MAX;
    for _ in 0..50 {
        e.scan(20, &h.cmd(), &safe);
    }
    assert_ne!(
        e.pose(),
        pose,
        "the robot must continue once the release is back"
    );
}

#[test]
fn every_robot_requests_the_interlock_of_its_first_op() {
    for ri in 0..2 {
        let (mut e, p) = exec_of(ri);
        let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
        let mut h = MiniHead {
            plan_id: p.plan_id(),
            job: 0,
            srb: false,
            grant: 0,
            partner: u32::MAX,
            drives: true,
        };
        for _ in 0..200 {
            e.scan(20, &h.cmd(), &safe);
        }
        h.job = p.folgen.production;
        h.srb = true;
        let mut f = [0u8; 32];
        for _ in 0..20 {
            f = e.scan(20, &h.cmd(), &safe).0;
        }
        // The map comes from program.json when bt2plc ran in the vw profile, else from
        // bt2krl.toml (these examples were generated in the sim profile).
        let cfg = vw_cfg();
        let rc = cfg.robot(&p.robot.ns).unwrap();
        let op1 = &p.ops[0];
        let want: Vec<u8> = op1
            .zones
            .iter()
            .map(|z| {
                p.zones
                    .iter()
                    .find(|x| &x.name == z)
                    .and_then(|x| x.interlock)
                    .or_else(|| rc.interlocks.get(z).copied())
                    .unwrap_or_else(|| panic!("no interlock for {z}"))
            })
            .collect();
        assert!(!want.is_empty(), "robot {ri}: op 1 must use a zone");
        assert_eq!(e.step(), 1, "robot {ri}: op 1 must be reported");
        for n in &want {
            assert!(
                bit(&f, 80 + u16::from(*n)),
                "robot {} ({}): op 1 {} must request V{n} ($OUT[{}]); frame {:?}",
                ri,
                p.robot.ns,
                op1.operation_id,
                80 + u16::from(*n),
                &f[..16]
            );
        }
    }
}

/// program.json from a bt2plc run in profile `vw-ks24` (no ST, no trust): the interlock map
/// then comes from the plan, which is the path the co-simulation uses.
fn vw_programs(interlocks: &[(&str, u8)]) -> Vec<bt2krl::program::Program> {
    let mut cfg = bt2plc::Config::default();
    cfg.robot_sim.enabled = false;
    cfg.protocol.profile = bt2plc::config::Profile::VwKs24;
    cfg.protocol.max_zones = 16;
    cfg.protocol.plan_id_byte = 12;
    cfg.cell.job_number = bt2plc::config::JobNumber::Byte;
    let map: std::collections::BTreeMap<String, u8> = interlocks
        .iter()
        .map(|(z, n)| ((*z).to_string(), *n))
        .collect();
    for ns in ["robot1", "robot2"] {
        cfg.robots.push(bt2plc::config::RobotCfg {
            ns: ns.into(),
            interlocks: map.clone(),
            ..Default::default()
        });
    }
    let text = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../test_data/bt_1790934549_welding_cell_2_small.json"),
    )
    .unwrap();
    programs(&text, &cfg)
}

#[test]
fn the_interlock_map_from_program_json_reaches_the_wire() {
    let programs = vw_programs(&[("IL01", 1), ("IL02", 11)]);
    // bt2krl config without an own map: it must use the one from program.json.
    let mut s = String::from("[vw_ks24]\nplan_id_byte = 12\n");
    for p in &programs {
        s.push_str(&format!(
            "[[robot]]\nns = \"{}\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n",
            p.robot.ns
        ));
    }
    let cfg = bt2krl::Config::parse(&s).unwrap();
    let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
    for p in &programs {
        assert_eq!(p.zones[0].interlock, Some(1), "{}", p.robot.ns);
        assert_eq!(p.zones[1].interlock, Some(11), "{}", p.robot.ns);
        let s = bt2krl::lower::RobotSetup::resolve(p, &cfg, true).unwrap();
        let l = bt2krl::lower::lower(p, &s, &cfg, bt2krl::Profile::VwKs24).unwrap();
        let mut e = bt2krl::exec::Executor::new(p, &s, &cfg, &l).unwrap();
        let mut h = MiniHead {
            plan_id: p.plan_id(),
            job: 0,
            srb: false,
            grant: 0,
            partner: u32::MAX,
            drives: true,
        };
        for _ in 0..200 {
            e.scan(20, &h.cmd(), &safe);
        }
        h.job = p.folgen.production;
        h.srb = true;
        let mut f = [0u8; 32];
        for _ in 0..20 {
            f = e.scan(20, &h.cmd(), &safe).0;
        }
        let z = &p.ops[0].zones[0];
        let n = p
            .zones
            .iter()
            .find(|x| &x.name == z)
            .unwrap()
            .interlock
            .unwrap();
        assert_eq!(e.step(), 1, "{}: op 1 must be reported", p.robot.ns);
        assert!(
            bit(&f, 80 + u16::from(n)),
            "{}: op 1 ({z}) must request V{n}; bytes 8-13 {:?}",
            p.robot.ns,
            &f[8..14]
        );
    }
}

/// Both robots of the welding sample against a Rust twin of the head's interlock logic: the
/// arbiter maps the VW signals onto one REQUEST bit per zone (`A(80+n) OR NOT A(40+n)`),
/// grants a zone to one robot at a time in ticket order, and routes the partner releases.
/// This is the fast counterpart of the trust co-simulation in tests/cosim.rs.
#[test]
fn two_robots_run_a_cycle_through_the_vw_interlocks() {
    let programs = vw_programs(&[("IL01", 1), ("IL02", 11)]);
    let mut s = String::from("[vw_ks24]\nplan_id_byte = 12\n");
    for p in &programs {
        s.push_str(&format!(
            "[[robot]]\nns = \"{}\"\nmodel = \"KR240R2900_ULTRA\"\ne_axes = \"none\"\n",
            p.robot.ns
        ));
    }
    let cfg = bt2krl::Config::parse(&s).unwrap();
    let n_zones = programs[0].zones.len();
    let il: Vec<Vec<u8>> = programs
        .iter()
        .map(|p| p.zones.iter().map(|z| z.interlock.unwrap_or(0)).collect())
        .collect();
    let mut ex: Vec<bt2krl::exec::Executor> = programs
        .iter()
        .map(|p| {
            let s = bt2krl::lower::RobotSetup::resolve(p, &cfg, true).unwrap();
            let l = bt2krl::lower::lower(p, &s, &cfg, bt2krl::Profile::VwKs24).unwrap();
            bt2krl::exec::Executor::new(p, &s, &cfg, &l).unwrap()
        })
        .collect();
    // ticket order per zone, from the plan's waits_for chain is not available here: use FCFS,
    // which is enough for the mutual-exclusion property this test checks.
    let mut holder: Vec<Option<usize>> = vec![None; n_zones];
    let mut frames: Vec<[u8; 32]> = vec![[0; 32]; programs.len()];
    let safe = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
    let mut done = vec![false; programs.len()];
    let mut overlap = 0u32;
    let mut grant_step: Vec<Option<u16>> = vec![None; programs.len()];
    let mut bad_handshake: Vec<String> = Vec::new();
    for t in 0..6000u32 {
        // ---- head: decode, arbitrate, encode ----
        let bit = |f: &[u8; 32], i: u16| {
            let i = usize::from(i) - 1;
            f[i / 8] & (1 << (i % 8)) != 0
        };
        let mut req = vec![0u32; programs.len()];
        for (r, f) in frames.iter().enumerate() {
            for z in 0..n_zones {
                let n = il[r][z];
                if n == 0 {
                    continue;
                }
                // "occupied" counts only while the robot is in a job (echo != 0), as the head does
                if bit(f, 80 + u16::from(n)) || (f[0] != 0 && !bit(f, 40 + u16::from(n))) {
                    req[r] |= 1 << z;
                }
            }
        }
        for z in 0..n_zones {
            if let Some(h) = holder[z]
                && req[h] & (1 << z) == 0
            {
                holder[z] = None;
            }
        }
        let mut grant = vec![0u32; programs.len()];
        for r in 0..programs.len() {
            let want = req[r];
            if want != 0
                && (0..n_zones).all(|z| want & (1 << z) == 0 || holder[z].is_none_or(|h| h == r))
            {
                for z in 0..n_zones {
                    if want & (1 << z) != 0 {
                        holder[z] = Some(r);
                    }
                }
                grant[r] = want;
            }
        }
        // Mutual exclusion is about OCCUPANCY (A(40+n) FALSE while in a job), not about the
        // request: two robots may request the same interlock at the same time.
        for z in 0..n_zones {
            let inside: Vec<usize> = (0..programs.len())
                .filter(|&r| {
                    let n = il[r][z];
                    n != 0 && frames[r][0] != 0 && !bit(&frames[r], 40 + u16::from(n))
                })
                .collect();
            if inside.len() > 1 {
                panic!("scan {t}: zone {z} occupied by {inside:?}");
            }
        }
        // The head grants per step: a robot must not change its step while it still holds a
        // grant for the previous one (the head raises ERR_HANDSHAKE for that). The VW scheme
        // has no 4-phase wait, so the release level must last long enough for the head to see
        // it and drop the grant.
        for r in 0..programs.len() {
            let st = ex[r].step();
            if grant[r] != 0 && st != 0 {
                if let Some(g) = grant_step[r]
                    && g != st
                {
                    bad_handshake.push(format!(
                        "scan {t}: R{} is at step {st} while still granted for step {g}",
                        r + 1
                    ));
                }
                grant_step[r] = Some(st);
            }
            if grant[r] == 0 {
                grant_step[r] = None;
            }
        }
        let busy = (0..programs.len()).filter(|&r| ex[r].step() > 0).count();
        if busy == programs.len() {
            overlap += 1;
        }
        // ---- robots ----
        for r in 0..programs.len() {
            let mut c = [0u8; 32];
            let mut set = |i: u16| {
                let i = usize::from(i) - 1;
                c[i / 8] |= 1 << (i % 8);
            };
            set(10); // drives
            if t > 120 {
                set(9); // SRB
            }
            for z in 0..n_zones {
                let n = il[r][z];
                if n == 0 {
                    continue;
                }
                if grant[r] & (1 << z) != 0 {
                    set(80 + u16::from(n));
                }
                // partner release: every other member's A(40+n)
                let ok = (0..programs.len())
                    .all(|q| q == r || il[q][z] == 0 || bit(&frames[q], 40 + u16::from(il[q][z])));
                if ok {
                    set(40 + u16::from(n));
                }
            }
            if t > 100 {
                c[0] = programs[r].folgen.production as u8;
            }
            c[12] = programs[r].plan_id() as u8;
            c[13] = (programs[r].plan_id() >> 8) as u8;
            let (f, _) = ex[r].scan(20, &c, &safe);
            if bit(&f, 14) {
                done[r] = true;
            }
            frames[r] = f;
        }
        if done.iter().all(|&d| d) {
            break;
        }
    }
    assert!(
        done.iter().all(|&d| d),
        "both robots must reach the Folge end: {done:?}"
    );
    for (r, p) in programs.iter().enumerate() {
        assert!(
            ex[r]
                .pose()
                .iter()
                .zip(p.home_pose_deg)
                .all(|(a, b)| (a - b).abs() < 0.5),
            "{}: must end at home",
            p.robot.ns
        );
    }
    assert!(
        bad_handshake.is_empty(),
        "{} handshake violations, first: {}",
        bad_handshake.len(),
        bad_handshake[0]
    );
    assert!(
        overlap > 50,
        "the robots must work in parallel (overlap {overlap} scans)"
    );
}
