//! Rendering of the `vw-ks24` instructions (TASK 7.3): the VW tech package does the Folge
//! handshake, so the generated Folge only follows its template and drives the standard robot
//! interlocks A41-56 / A81-96 inline (MAKRO20 semantics).

use crate::ir::{Instr, Point};
use crate::krl::{Text, ascii};
use crate::render::Ctx;
use crate::vw;

/// `$LOOP_MSG[]` is cleared with the VW standard 128-space literal.
const CLEAR_MSG: &str = "                                                                                                                                ";

/// The VW wait pattern: a loop message, then the condition with the operator escape that only
/// works outside EXT mode (`makro20.src`), so automatic operation can never leave a wait early.
fn vw_wait(t: &mut Text, msg: &str, cond: &str) {
    t.line(format!("$LOOP_MSG[]=\"{msg}\""));
    t.line("$LOOP_CONT=TRUE");
    t.line(format!(
        "WAIT FOR ({cond}) OR (NOT $LOOP_CONT AND ($MODE_OP<>#EX))"
    ));
    t.line(format!("$LOOP_MSG[]=\"{CLEAR_MSG}\""));
}

fn makro(t: &mut Text, n: u16) {
    t.line(format!("SEL_RES=SELECT(#MAKRO,{n},TRUE,,,,,FALSE)"));
}

/// Interlock names of a mask, for the fold comments.
fn names(ctx: &Ctx, mask: u32) -> Vec<String> {
    let il = ctx.il.as_ref();
    crate::io::bits(mask)
        .iter()
        .map(|&b| {
            let z = ctx
                .p
                .zones
                .iter()
                .find(|z| u32::from(z.bit) == b)
                .map(|z| ascii(&z.name))
                .unwrap_or_else(|| format!("bit{b}"));
            let n = il
                .and_then(|i| i.indices(ctx.p, 1 << b).ok())
                .and_then(|v| v.first().copied());
            match n {
                Some(n) => format!("{z}=V{n}"),
                None => z,
            }
        })
        .collect()
}

fn indices(ctx: &Ctx, mask: u32) -> Vec<u8> {
    ctx.il
        .as_ref()
        .and_then(|i| i.indices(ctx.p, mask).ok())
        .unwrap_or_default()
}

/// Online driving condition: the partner releases of every interlock currently held.
fn online_stop(ctx: &Ctx, t: &mut Text, held: u32) {
    let mut terms = vec![format!("$CYCFLAG[{}]", vw::FB_CYCFLAG_SAFE)];
    for n in indices(ctx, held) {
        terms.push(format!("$IN[{}]", vw::partner_in(n)));
    }
    t.line("VALCYCFL30=$CYCFLAG[254]");
    t.line(format!(
        "$CYCFLAG[{}]=({}) AND $CYCFLAG[{}] OR ($TIMER[{}]<0)",
        vw::FB_CYCFLAG,
        terms.join(" AND "),
        vw::FB_CYCFLAG_KEEP,
        vw::FB_TIMER
    ));
    t.line("VW(#FB_CHECK,TRUE)");
}

pub fn instr(ctx: &Ctx, points: &[Point], t: &mut Text, ins: &Instr, vel: &mut Option<u8>) {
    let v = &ctx.cfg.vw_ks24;
    match ins {
        Instr::VwAueInit => {
            t.open(";FOLD AUE INIT");
            t.line("VW(#VW_AUE_INIT,TRUE)");
            t.line("PENTER:");
            t.line(format!(
                "INTERRUPT DECL {} WHEN $CYCFLAG[{}]==FALSE DO VW(#FB_STOP,TRUE)",
                vw::FB_INTERRUPT,
                vw::FB_CYCFLAG
            ));
            t.line(format!("INTERRUPT ON {}", vw::FB_INTERRUPT));
            t.line("INT10=TRUE");
            t.close(";ENDFOLD");
            *vel = None;
        }
        Instr::VwP0 { point } => {
            let p = &points[*point];
            t.open(format!(";FOLD VWP0 {} 10 PCT", p.name));
            t.comment("P0 = home: VW_INIT runs BAS(#INITMOV); AUE_WAIT later sets $H_POS here");
            t.line("VW(#VW_INIT,TRUE)");
            t.line("VW_USR_R (#USR_INIT)");
            t.line("BAS(#VEL_PTP,10)");
            t.line(format!("$APO.CPTP={}", ctx.cfg.motion.approx_ptp_pct));
            t.line(format!("PTP {}", p.name));
            t.line(format!("$OUT[{}]=TRUE ; PF0 Grundstellung", vw::PF0_OUT));
            if v.makro0 {
                t.comment("MAKRO0 RUECKSETZEN_EA: resets A33-A96 (also the tool releases)");
                makro(t, 0);
            }
            t.close(";ENDFOLD");
            *vel = Some(10);
        }
        Instr::VwAueWait => {
            t.open(";FOLD VWAUEWAIT");
            t.line("VW(#VW_AUE_WAIT,TRUE )");
            t.open("IF N_VW==TRUE THEN");
            t.line("GOTO PEXIT");
            t.close("ENDIF");
            t.close(";ENDFOLD");
        }
        Instr::VwCycleStart { mask, makro50 } => {
            t.open(format!(";FOLD VWSTART 16#{mask:08X}"));
            let n = names(ctx, *mask);
            if !n.is_empty() {
                t.comment(format!(
                    "release every interlock this robot uses: {}",
                    n.join(" ")
                ));
            }
            t.line(format!("$OUT[{}]=FALSE ; PF0 off", vw::PF0_OUT));
            if *makro50 {
                t.comment("MAKRO50 Ini Merker: the cyclic flags of the driving conditions");
                makro(t, 50);
            }
            for i in indices(ctx, *mask) {
                t.line(format!("$OUT[{}]=TRUE ; release V{i}", vw::release_out(i)));
            }
            t.close(";ENDFOLD");
        }
        Instr::VwLock(mask) => {
            t.open(format!(";FOLD VWLOCK 16#{mask:08X}"));
            t.comment(format!("enter {}", names(ctx, *mask).join(" ")));
            let ix = indices(ctx, *mask);
            for i in &ix {
                t.line(format!("$OUT[{}]=TRUE ; request V{i}", vw::request_out(*i)));
            }
            if v.use_makro20 {
                t.comment("MAKRO20 ROBOTER VERRIEGELUNGEN (cell-specific macro number)");
                makro(t, 20);
            } else {
                t.comment("MAKRO20 semantics, inline for these interlocks only");
                for i in &ix {
                    let (r, q) = (vw::release_out(*i), vw::request_out(*i));
                    t.line(format!("$OUT[{r}]=$OUT[{r}] OR $OUT[{q}]"));
                }
                let grants: Vec<String> = ix
                    .iter()
                    .map(|i| format!("$IN[{}]", vw::grant_in(*i)))
                    .collect();
                let gmsg: Vec<String> = ix
                    .iter()
                    .map(|i| format!("E{}", vw::grant_in(*i)))
                    .collect();
                t.comment("PLC grant of the request");
                vw_wait(t, &gmsg.join(" & "), &grants.join(" AND "));
                let partners: Vec<String> = ix
                    .iter()
                    .map(|i| format!("$IN[{}]", vw::partner_in(*i)))
                    .collect();
                let pmsg: Vec<String> = ix
                    .iter()
                    .map(|i| format!("E{}", vw::partner_in(*i)))
                    .collect();
                t.comment("partner release");
                vw_wait(t, &pmsg.join(" & "), &partners.join(" AND "));
                for i in &ix {
                    t.line(format!("$OUT[{}]=FALSE ; occupied", vw::release_out(*i)));
                }
                for i in &ix {
                    t.line(format!("$OUT[{}]=FALSE", vw::request_out(*i)));
                }
            }
            t.close(";ENDFOLD");
        }
        Instr::VwUnlock(mask) => {
            t.open(format!(";FOLD VWUNLOCK 16#{mask:08X}"));
            t.comment(format!("leave {}", names(ctx, *mask).join(" ")));
            for i in indices(ctx, *mask) {
                t.line(format!("$OUT[{}]=TRUE ; release V{i}", vw::release_out(i)));
            }
            t.close(";ENDFOLD");
        }
        Instr::VwOnlineStop(held) => {
            t.open(format!(";FOLD VWFB 16#{held:08X}"));
            online_stop(ctx, t, *held);
            t.close(";ENDFOLD");
        }
        Instr::VwMakro15 => {
            t.open(format!(";FOLD VWMAKRO {}", v.makro15));
            t.comment("Gesamtfertigmeldung");
            makro(t, v.makro15);
            t.close(";ENDFOLD");
        }
        Instr::VwTechCall(call) => {
            t.open(";FOLD VWTECH");
            t.line(ascii(call));
            t.close(";ENDFOLD");
        }
        Instr::VwAueEnd => {
            t.open(";FOLD AUE END");
            t.line("VW(#VW_AUE_END,TRUE)");
            t.line("GOTO PENTER");
            t.line("PEXIT:");
            t.close(";ENDFOLD");
        }
        other => unreachable!("not a vw-ks24 instruction: {other:?}"),
    }
}

/// The step write of the `vw-ks24` profile (`BT_PKTNR`, declared by `bt_config.dat`).
pub fn step_line(ctx: &Ctx, k: u16) -> String {
    let (_, name) = vw::step_signal(&ctx.cfg.vw_ks24);
    format!("{name}={k}")
}

/// PLAN_ID check when a free telegram byte is configured for it.
pub fn plan_check(ctx: &Ctx, t: &mut Text, id: u16) {
    t.open(format!(";FOLD PLAN ID 16#{id:04X}"));
    t.open("IF BT_PLANID<>PLAN_ID THEN");
    t.line("$LOOP_MSG[]=\"BT2KRL PLAN_ID mismatch - regenerate KRL and PLC\"");
    t.line("HALT");
    t.close("ENDIF");
    t.close(";ENDFOLD");
    let _ = ctx;
}

// ───────────────────────────── $config.dat and cell.src ─────────────────────────────

/// The declarations to merge into `KRC/R1/System/$config.dat` (fold `USER GLOBALS`).
/// The VW standard already declares everything else this profile uses.
pub fn config_dat(ctx: &Ctx) -> String {
    let v = &ctx.cfg.vw_ks24;
    let mut t = Text::default();
    t.comment(format!(
        "bt2krl vw-ks24: merge into KRC/R1/System/$config.dat (fold USER GLOBALS), robot {} plan {}",
        ctx.p.robot.ns,
        ctx.plan_hex()
    ));
    t.comment(
        "The VW KS24 standard already declares PRG_NR, PRG_NR_REFL, SEL_RES, INT10, VALCYCFL30,",
    );
    t.comment(
        "N_VW and the interlock signals A41-56 / A81-96; only what bt2krl adds is listed here.",
    );
    t.open(format!(
        ";FOLD BT2KRL {} {}",
        crate::krl::token(&ctx.p.robot.ns),
        ctx.plan_hex()
    ));
    let (base, name) = vw::step_signal(v);
    let place = if base == vw::STEP_OUT_VW {
        format!(
            "the outputs of the VW {}, which is local to vw_usr_r",
            vw::STEP_SIGNAL_VW
        )
    } else {
        "configured step_signal".to_string()
    };
    t.line(format!(
        "SIGNAL {name} $OUT[{base}] TO $OUT[{}] ; op index ({place})",
        base + 15
    ));
    if v.plan_id_byte != 0 {
        let a = 8 * u16::from(v.plan_id_byte) + 1;
        t.line(format!(
            "SIGNAL BT_PLANID $IN[{a}] TO $IN[{}] ; PLAN_ID (free telegram byte {}-{})",
            a + 15,
            v.plan_id_byte,
            v.plan_id_byte + 1
        ));
        t.line(format!("DECL INT PLAN_ID={}", ctx.p.plan_id()));
    } else {
        t.comment("no PLAN_ID check: the VW standard has no signal for it (see INSTALL.md)");
    }
    t.close(";ENDFOLD");
    t.finish(ctx.cfg.output.line_ending)
}

/// Optional patch file: a `$config.dat` fragment with a header that says what to do with it.
pub fn config_dat_full(ctx: &Ctx) -> String {
    let mut t = Text::default();
    t.comment("OPTIONAL PATCH, do not copy over the controller's $config.dat as a whole:");
    t.comment("the controller's file carries cell-specific VW content that bt2krl does not know.");
    t.comment("Paste the fold below into the USER GLOBALS fold of KRC/R1/System/$config.dat.");
    let body = config_dat(ctx);
    let sep = match ctx.cfg.output.line_ending {
        crate::config::LineEnding::Crlf => "\r\n",
        crate::config::LineEnding::Lf => "\n",
    };
    format!("{}{sep}{body}", t.finish(ctx.cfg.output.line_ending))
}

/// Optional patch file: the `cell.src` CASE entry for the production Folge.
pub fn cell_src(ctx: &Ctx, l: &crate::lower::Lowered) -> String {
    let n = l.prod.number;
    let mut t = Text::default();
    t.comment("OPTIONAL PATCH, do not copy over the controller's cell.src:");
    t.comment("its CASE list is cell-specific. Add the two lines below to the SWITCH PGNO block");
    t.comment("of KRC/R1/Folgen/cell.src (CASE 0 of SWITCH P_TYPE), keeping the existing cases.");
    t.line("");
    t.open("SWITCH PGNO");
    t.open(format!("CASE {n}"));
    t.line(format!("FOLGE{n}()"));
    t.dedent();
    t.close("ENDSWITCH");
    t.line("");
    t.comment(format!(
        "Folge {n} must be 1..255: PRG_NR is 8 bits (E1-E8) on these robots."
    ));
    t.finish(ctx.cfg.output.line_ending)
}
