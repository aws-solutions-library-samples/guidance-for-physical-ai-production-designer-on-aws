//! Reads a rendered Folge (.src + .dat) back into the IR (TASK 8.3). It accepts exactly the
//! subset [`crate::render`] writes: compound instructions (titled folds) are rebuilt from their
//! title and checked line by line against a fresh rendering, everything else is decoded line by
//! line. Anything else is an error.

use crate::ir::{Folge, FolgeKind, Instr, OpBlock, Point};
use crate::krl::Text;
use crate::render::{self, Ctx};

type R<T> = std::result::Result<T, String>;

fn strip_comment(l: &str) -> &str {
    match l.find(';') {
        Some(i) => l[..i].trim_end(),
        None => l,
    }
}

fn num_list(body: &str) -> R<Vec<(String, f64)>> {
    body.split(',')
        .map(|kv| {
            let mut it = kv.split_whitespace();
            let k = it.next().ok_or("empty component")?.to_string();
            let v: f64 = it
                .next()
                .ok_or("component without value")?
                .parse()
                .map_err(|e| format!("bad number in {kv}: {e}"))?;
            Ok((k, v))
        })
        .collect()
}

/// Points and PLAN_ID of a Folge data list.
pub fn parse_dat(dat: &str) -> R<(Vec<Point>, Option<u16>)> {
    let mut pts = Vec::new();
    let mut plan = None;
    for raw in dat.lines() {
        let l = raw.trim();
        if let Some(v) = l.strip_prefix("DECL INT PLAN_ID=") {
            plan = Some(v.parse().map_err(|e| format!("PLAN_ID {v}: {e}"))?);
            continue;
        }
        let (e6, rest) = if let Some(r) = l.strip_prefix("DECL AXIS ") {
            (false, r)
        } else if let Some(r) = l.strip_prefix("DECL E6AXIS ") {
            (true, r)
        } else {
            continue;
        };
        let (name, lit) = rest.split_once('=').ok_or(format!("bad point {l}"))?;
        let body = lit
            .strip_prefix('{')
            .and_then(|b| b.strip_suffix('}'))
            .ok_or(format!("bad point literal {l}"))?;
        let comps = num_list(body)?;
        let want: Vec<String> = (1..=6)
            .map(|i| format!("A{i}"))
            .chain((1..=6).filter(|_| e6).map(|i| format!("E{i}")))
            .collect();
        if comps.iter().map(|c| &c.0).ne(want.iter()) {
            return Err(format!("unexpected components in {l}"));
        }
        let q = std::array::from_fn(|i| comps[i].1);
        let e = e6.then(|| std::array::from_fn(|i| comps[6 + i].1));
        pts.push(Point {
            name: name.to_string(),
            q,
            e,
        });
    }
    Ok((pts, plan))
}

/// Re-render one compound instruction and compare it with the original block.
fn same_as_rendered(
    ctx: &Ctx,
    f: &Folge,
    ins: &Instr,
    block: &[&str],
    vel: &mut Option<u8>,
) -> R<()> {
    let mut t = Text::default();
    render::instr(ctx, f, &mut t, ins, vel);
    let le = ctx.cfg.output.line_ending;
    let text = t.finish(le);
    let want: Vec<&str> = text.lines().map(str::trim).collect();
    let got: Vec<&str> = block.iter().map(|l| l.trim()).collect();
    if want != got {
        return Err(format!(
            "block {:?} differs from the rendering of {ins:?}",
            got.first()
        ));
    }
    Ok(())
}

fn indices(cond: &str, prefix: &str) -> R<Vec<u16>> {
    let mut out = Vec::new();
    let mut rest = cond;
    while let Some(i) = rest.find(prefix) {
        rest = &rest[i + prefix.len()..];
        let j = rest.find(']').ok_or("unterminated index")?;
        out.push(rest[..j].parse().map_err(|e| format!("index: {e}"))?);
        rest = &rest[j..];
    }
    Ok(out)
}

fn mask_of(ixs: &[u16], base: u16) -> R<u32> {
    let mut m = 0u32;
    for &n in ixs {
        let b = n
            .checked_sub(base)
            .filter(|&b| b < 32)
            .ok_or(format!("signal {n} is not in the REQUEST/GRANT field"))?;
        m |= 1 << b;
    }
    Ok(m)
}

fn hex32(s: &str) -> R<u32> {
    crate::program::parse_hex(s)
        .filter(|&m| m <= u64::from(u32::MAX))
        .map(|m| m as u32)
        .ok_or(format!("bad mask {s}"))
}

fn ms_of(s: &str) -> R<u64> {
    let v: f64 = s.parse().map_err(|e| format!("seconds {s}: {e}"))?;
    Ok((v * 1000.0).round() as u64)
}

pub fn parse_folge(ctx: &Ctx, src: &str, dat: &str) -> R<Folge> {
    let (points, _) = parse_dat(dat)?;
    let lines: Vec<&str> = src.lines().map(|l| l.trim_end_matches('\r')).collect();
    let def = lines
        .iter()
        .position(|l| l.starts_with("DEF FOLGE"))
        .ok_or("no DEF FOLGE<n>()")?;
    let number: u16 = lines[def]
        .trim_start_matches("DEF FOLGE")
        .trim_end_matches("()")
        .parse()
        .map_err(|e| format!("Folge number: {e}"))?;
    let end = lines.iter().rposition(|l| *l == "END").ok_or("no END")?;
    let kind = if number == ctx.p.folgen.production {
        FolgeKind::Production
    } else {
        FolgeKind::Home
    };
    let mut f = Folge {
        number,
        kind,
        points,
        prologue: Vec::new(),
        ops: Vec::new(),
        epilogue: Vec::new(),
    };
    let point_ix = |f: &Folge, name: &str| {
        f.points
            .iter()
            .position(|p| p.name == name)
            .ok_or(format!("undeclared point {name}"))
    };
    let step_prefix = match ctx.profile {
        crate::Profile::VwKs24 => format!("{}=", crate::vw::step_signal(&ctx.cfg.vw_ks24).1),
        crate::Profile::Sim => "BT_STEP=".to_string(),
    };
    let mut vel: Option<u8> = None;
    let mut cur: Option<OpBlock> = None;
    let mut seen_op = false;
    let body: Vec<&str> = lines[def + 1..end].iter().map(|l| l.trim()).collect();
    let mut i = 0;
    let push = |f: &mut Folge, cur: &mut Option<OpBlock>, seen: bool, ins: Instr| match cur {
        Some(o) => o.instrs.push(ins),
        None if seen => f.epilogue.push(ins),
        None => f.prologue.push(ins),
    };
    while i < body.len() {
        let l = body[i];
        if l.is_empty()
            || l.starts_with("DECL ")
            || (l.starts_with(';') && !l.starts_with(";FOLD") && !l.starts_with(";ENDFOLD"))
        {
            i += 1;
            continue;
        }
        if let Some(t) = l.strip_prefix(";FOLD ") {
            if let Some(rest) = t.strip_prefix("OP ") {
                if cur.is_some() {
                    return Err("nested OP fold".into());
                }
                let parts: Vec<&str> = rest.split_whitespace().collect();
                if parts.len() != 4 || parts[2] != "MASK" {
                    return Err(format!("bad OP fold title {l}"));
                }
                cur = Some(OpBlock {
                    index: parts[0].parse().map_err(|e| format!("op index: {e}"))?,
                    operation_id: parts[1].to_string(),
                    mask: crate::program::parse_hex(parts[3])
                        .ok_or(format!("bad mask {}", parts[3]))? as u32,
                    instrs: Vec::new(),
                });
                seen_op = true;
                i += 1;
                continue;
            }
            // Compound instruction: the whole fold.
            let mut depth = 0;
            let mut j = i;
            loop {
                if j >= body.len() {
                    return Err(format!("unterminated fold {l}"));
                }
                if body[j].starts_with(";FOLD") {
                    depth += 1;
                } else if body[j].starts_with(";ENDFOLD") {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                j += 1;
            }
            let block = &body[i..=j];
            let words: Vec<&str> = t.split_whitespace().collect();
            let ins = match words.as_slice() {
                ["INI"] => {
                    let mut approx_pct = None;
                    let (mut tool, mut base) = (0, 0);
                    for b in block {
                        if let Some(v) = b.strip_prefix("$APO.CPTP=") {
                            approx_pct = v.parse().ok();
                        } else if let Some(v) = b.strip_prefix("BAS(#TOOL,") {
                            tool = v.trim_end_matches(')').parse().map_err(|_| "tool")?;
                        } else if let Some(v) = b.strip_prefix("BAS(#BASE,") {
                            base = v.trim_end_matches(')').parse().map_err(|_| "base")?;
                        }
                    }
                    Instr::Init {
                        approx_pct: approx_pct.ok_or("INI without $APO.CPTP")?,
                        tool,
                        base,
                    }
                }
                ["PLAN", "ID", id] => Instr::CheckPlan(
                    crate::program::parse_hex(id).ok_or(format!("plan id {id}"))? as u16,
                ),
                ["START", "DEVIATION", pt, tol] => Instr::CheckStart {
                    point: point_ix(&f, pt)?,
                    tol_mdeg: ms_of(tol)? as u32,
                },
                ["HOME", "MOVE", pt, s, "S"] => Instr::HomeMove {
                    point: point_ix(&f, pt)?,
                    time_ms: ms_of(s)?,
                },
                ["FINISH"] => Instr::Finish,
                ["DONE"] => Instr::Done,
                // profile vw-ks24
                ["AUE", "INIT"] => Instr::VwAueInit,
                ["VWP0", pt, "10", "PCT"] => Instr::VwP0 {
                    point: point_ix(&f, pt)?,
                },
                ["VWAUEWAIT"] => Instr::VwAueWait,
                ["VWSTART", m] => Instr::VwCycleStart {
                    mask: hex32(m)?,
                    makro50: block.iter().any(|b| b.contains("SELECT(#MAKRO,50")),
                },
                ["VWLOCK", m] => Instr::VwLock(hex32(m)?),
                ["VWUNLOCK", m] => Instr::VwUnlock(hex32(m)?),
                ["VWFB", m] => Instr::VwOnlineStop(hex32(m)?),
                ["VWMAKRO", _] => Instr::VwMakro15,
                ["VWTECH"] => Instr::VwTechCall(ctx.cfg.vw_ks24.weld_call.clone()),
                ["AUE", "END"] => Instr::VwAueEnd,
                _ => return Err(format!("unknown fold {l}")),
            };
            same_as_rendered(ctx, &f, &ins, block, &mut vel)?;
            if matches!(
                ins,
                Instr::HomeMove { .. } | Instr::Finish | Instr::Done | Instr::VwMakro15
            ) {
                seen_op = true;
            }
            push(&mut f, &mut cur, seen_op, ins);
            i = j + 1;
            continue;
        }
        if l == ";ENDFOLD" {
            match cur.take() {
                Some(o) => f.ops.push(o),
                None => return Err("stray ;ENDFOLD".into()),
            }
            i += 1;
            continue;
        }
        if l == "IF BT_ABRT THEN" {
            if body.get(i + 1) != Some(&"RETURN") || body.get(i + 2) != Some(&"ENDIF") {
                return Err("bad abort guard".into());
            }
            push(&mut f, &mut cur, seen_op, Instr::AbortGuard);
            i += 3;
            continue;
        }
        let c = strip_comment(l);
        let comment = l.split_once(';').map(|x| x.1.trim()).unwrap_or("");
        // REQUEST / RELEASE: consecutive $OUT[...] writes.
        if c.starts_with("$OUT[") {
            let on = c.ends_with("=TRUE");
            let mut ixs = Vec::new();
            while i < body.len() {
                let c2 = strip_comment(body[i]);
                if !c2.starts_with("$OUT[") || c2.ends_with("=TRUE") != on {
                    break;
                }
                if !(c2.ends_with("=TRUE") || c2.ends_with("=FALSE")) {
                    return Err(format!("unexpected output write {c2}"));
                }
                ixs.extend(indices(c2, "$OUT[")?);
                i += 1;
            }
            let m = mask_of(&ixs, ctx.io.request)?;
            let ins = if on {
                Instr::Request(m)
            } else {
                Instr::Release(m)
            };
            push(&mut f, &mut cur, seen_op, ins);
            continue;
        }
        let ins = if c == "BT_RUN=TRUE" {
            Instr::Running
        } else if let Some(k) = c.strip_prefix(step_prefix.as_str()) {
            Instr::SetStep(k.parse().map_err(|e| format!("step {k}: {e}"))?)
        } else if let Some(cond) = c.strip_prefix("WAIT FOR ") {
            let m = mask_of(&indices(cond, "$IN[")?, ctx.io.grant)?;
            if cond.starts_with("NOT ") {
                Instr::WaitReleased(m)
            } else {
                Instr::WaitGrant(m)
            }
        } else if let Some(s) = c.strip_prefix("WAIT SEC ") {
            let v = ms_of(s)?;
            match comment {
                "start delay" => Instr::Delay(v),
                "hold" => Instr::Hold(v),
                "step dwell" => Instr::Dwell(v),
                other => return Err(format!("WAIT SEC with unknown role '{other}'")),
            }
        } else if let Some(v) = c.strip_prefix("BAS(#VEL_PTP,") {
            vel = Some(
                v.trim_end_matches(')')
                    .parse()
                    .map_err(|e| format!("velocity {v}: {e}"))?,
            );
            i += 1;
            continue;
        } else if let Some(rest) = c.strip_prefix("PTP ") {
            let (name, approx) = match rest.strip_suffix(" C_PTP") {
                Some(n) => (n, true),
                None => (rest, false),
            };
            Instr::Move {
                point: point_ix(&f, name)?,
                vel_pct: vel.ok_or(format!("PTP {name} before any BAS(#VEL_PTP)"))?,
                approx,
            }
        } else {
            return Err(format!("unexpected line: {l}"));
        };
        push(&mut f, &mut cur, seen_op, ins);
        i += 1;
    }
    if cur.is_some() {
        return Err("unterminated OP fold".into());
    }
    Ok(f)
}
