"""Dense verification of an N-robot joint timeline against a scene (scenes/SCHEMA.md).

    rep = verify_timeline(model, tl, check_hz=100)      # tl: player.timeline.Timeline (any source)

Works on any Timeline (the generator's, umarell plans, the expert, policies): the timeline is evaluated the way
umarell executes it (linear interpolation of its samples, ``Timeline.sample``) on a uniform grid at ``check_hz``.

Checks (each with its worst value and a pass flag):
  arm_arm_spheres   corrected-sphere clearance > gate, every robot pair (and, for parity, umarell's own 35-sphere
                    model on the scene's robot geometry)
  arm_arm_hull      robot-robot MuJoCo hull distance >= collision.arm_arm.hull_min_distance_m at every sample
  box_hull          URDF collision hulls vs obstacle boxes (MuJoCo) >= obstacles.box_hull.min_distance_m
  self_hull         no MuJoCo hull self-contact; coal hull self-distance (SRDF pairs removed) >= scene min
  floor_spheres     corrected sphere rule (tool included); floor_hull: hull vertices z >= floor
  boxes             box rule incl. tool spheres (cells with boxes)
  rule_W            max wedge depth of every TCP over all welds <= tol, on the check grid and again on a dense
                    TCP-only grid (``rule_w_hz``, default 1 kHz)
  electrode_off_weld  (only when the scene enables obstacles.box_hull.electrode_off_weld; docs/n2_contract.md 3.1)
                    while the TCP is farther than the rule's radius from every weld: electrode hull(s) - workpiece box
                    penetration (min_distance - MuJoCo signed distance) <= score_tol at every check-grid sample; the
                    continuous certificate of the same rule (threshold min_distance - score_tol) is part of
                    ``continuous``
                    no-dip rule (round 3, trajgen.nodip, ``model.nodip``): the same check name, now time-aware and
                    on the dense grid (``dense_hz``, default trajgen.nodip.SCORE_HZ = 400 Hz): any electrode -
                    workpiece penetration fails except contacts inside the robot's contact window of its own weld
                    (``nodip_check``); ``box_hull`` then also runs on the dense grid (``box_hull.dense``)
  arm_arm_umarell_spheres  umarell's own sphere gate (parity: umarell's validators use it)
  welds             per weld event: position / tool-axis / rotation errors at arrival, entry direction over the
                    last entry_span, TCP drift during the dwell; every task weld exactly once
  strokes           straight on-axis descent and lift-off: how far up the weld's approach line the TCP stays
                    inside the tube (scene descent/liftoff tube_m) before arrival and after the dwell; both must
                    reach min_offset_m (a joint-space drag out of the seam leaves the tube within millimetres)
  dynamics          plan_dynamics_n (execute.plan_dynamics for R robots: 200 Hz grid, h = 0.05 s, x URDF limits)
  home              every arm ends at HOME
  arm_arm_hull_skew robot-robot hull distance >= hull_min_distance_m with one robot's program shifted by every
                    grid step within +-skew_window_s (the planners' dispatch-skew window)
  skew_tolerance    the shift scan (timing_tolerance): sphere rows at +-0.05 / 0.1 s, the window, and per robot the
                    largest constant lag / lead that keeps the sphere gate, hull contact-freedom and the hull buffer
"""
from __future__ import annotations

import numpy as np



def _r(x, n=4):
    return None if x is None or not np.isfinite(x) else round(float(x), n)


def plan_dynamics_n(sample, T, v_max, a_max, rids, *, hz=200.0, h=0.05, t0=0.0):
    """execute.plan_dynamics for any number of robots: joint velocity (central difference over 1/hz), acceleration
    and jerk (central differences with h = 0.05 s) of ``sample(ts) -> (n, R, 6)`` against the URDF limits."""
    ts = np.arange(t0, T + 1e-9, 1.0 / hz)
    d = 0.5 / hz
    q = lambda x: sample(np.clip(x, t0, T))
    v = (q(ts + d) - q(ts - d)) / (2 * d)
    a = (q(ts + h) - 2 * q(ts) + q(ts - h)) / h ** 2
    j = (q(ts + 1.5 * h) - 3 * q(ts + 0.5 * h) + 3 * q(ts - 0.5 * h) - q(ts - 1.5 * h)) / h ** 3
    rv, ra, rj = np.abs(v) / v_max, np.abs(a) / a_max, np.abs(j) / a_max
    out = {"grid_hz": hz, "fd_h_s": h, "samples": int(len(ts)), "T_s": _r(T - t0, 3)}
    R = len(rids)
    for name, r in (("vel_frac_vmax", rv), ("acc_frac_amax", ra), ("jerk_amax_per_s", rj)):
        m = r.max(axis=(1, 2))
        k = int(m.argmax())
        rr, jj = np.unravel_index(int(r[k].argmax()), (R, 6))
        out[name] = {"max": _r(m.max(), 3), "t_max_s": _r(ts[k], 3), "at": f"{rids[rr]} a{jj + 1}",
                     "p99": _r(np.percentile(m, 99), 3),
                     "per_joint_max": {rid: [_r(x, 3) for x in r[:, i].max(0)] for i, rid in enumerate(rids)}}
    ma = ra.max(axis=(1, 2))
    out["acc_frac_samples_over"] = {str(x): _r((ma > x).mean(), 4) for x in (0.49, 0.7, 1.0)}
    return out


def _pose_errors(p, R, w, axis_local):
    """Position, tool-axis (R @ tool.axis vs R_weld @ tool.axis) and geodesic rotation errors."""
    a = R @ axis_local
    e = w.R @ axis_local
    axis = float(np.degrees(np.arccos(np.clip(a @ e / (np.linalg.norm(a) * np.linalg.norm(e)), -1.0, 1.0))))
    Rrel = R.T @ w.R
    rot = float(np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1.0) / 2.0, -1.0, 1.0))))
    return {"pos_m": float(np.linalg.norm(p - w.position)), "tool_axis_deg": axis, "rot_deg": rot}


def verify_timeline(model, tl, *, check_hz: float = 100.0, hull_dist_every: int = 5, self_dist_every: int = 1,
                    umarell_parity: bool = True, t_end: float | None = None, dynamics: bool = True,
                    skew_s=(0.05, 0.1), rule_w_hz: float = 1000.0, skew_window_s: float = 0.1,
                    timing_max_s: float = 0.3, continuous: bool = True, dense_hz: float | None = None) -> dict:
    """Dense verification of an N-robot timeline (trajgen.ntimeline.NTimeline, or a two-robot
    player.timeline.Timeline for scenes whose robots are robot1/robot2).

    ``continuous`` (default on, phase 3 TGCERT): also certify every geometric rule between the samples
    (``trajgen.certify.ContinuousCertifier``: sphere gates, hull buffer, self hulls, floor, boxes, box hulls, rule W
    over the continuous piecewise-linear joint paths) and gate it as the check ``continuous``; with a skew window the
    check also requires every knot of the timeline on the check grid (then the grid-based skew-window hull
    certificate covers the exact path).

    ``dense_hz`` (round 3): grid of the box-hull and electrode checks when above ``check_hz``; None = the no-dip
    rule's ``SCORE_HZ`` (400 Hz) when the scene's rule is no-dip (``model.nodip``), else off (unchanged)."""
    from trajgen.ntimeline import as_ntimeline

    sc, task = model.scene, model.task
    rids = model.rids
    tl = as_ntimeline(tl)
    try:
        col = [list(tl.robot_ids).index(r) for r in rids]
    except ValueError:
        raise ValueError(f"timeline robots {tl.robot_ids} do not match the scene's {rids}") from None
    T = float(tl.t[-1]) if t_end is None else float(t_end)
    ts = np.arange(0.0, T + 1e-9, 1.0 / check_hz)
    if ts[-1] < T - 1e-9:
        ts = np.append(ts, T)
    Q = tl.sample(ts)[:, col]                                    # (N, R, 6) in the scene's robot order
    N = len(ts)
    rep = {"grid_hz": check_hz, "samples": int(N), "t_end_s": _r(T, 3), "robots": list(rids)}
    chk = {}
    pairs = [(i, j) for i in range(len(rids)) for j in range(i + 1, len(rids))]
    # ---------------------------------------------------------------- arm vs arm (every pair)
    gate = sc.collision.arm_arm_gate
    pr = {}
    cmin_all, nbad = np.inf, 0
    um = model.umarell if umarell_parity else None
    umin_all, ubad = np.inf, 0
    for i, j in pairs:
        ra, rb = rids[i], rids[j]
        c = model.pair_clearance(ra, rb, Q[:, i], Q[:, j])
        k = int(np.argmin(c))
        row = {"min_clearance_m": _r(c.min()), "t_min_s": _r(ts[k], 3), "samples_le_gate": int((c <= gate).sum())}
        cmin_all, nbad = min(cmin_all, float(c.min())), nbad + int((c <= gate).sum())
        if um is not None:
            cu = model.umarell_pair_clearance(ra, rb, Q[:, i], Q[:, j])
            row["umarell_35_spheres_min_m"] = _r(cu.min())
            row["umarell_samples_le_gate"] = int((cu <= um.warning_distance).sum())
            umin_all, ubad = min(umin_all, float(cu.min())), ubad + int((cu <= um.warning_distance).sum())
        pr[f"{ra}|{rb}"] = row
    rep["arm_arm_spheres"] = {"min_clearance_m": _r(cmin_all) if pairs else None, "gate_m": gate,
                              "samples_le_gate": nbad, "pairs": pr}
    chk["arm_arm_spheres"] = bool(nbad == 0)
    if um is not None and pairs:
        rep["arm_arm_spheres"]["umarell_35_spheres_min_m"] = _r(umin_all)
        rep["arm_arm_spheres"]["umarell_samples_le_gate"] = ubad
        rep["arm_arm_spheres"]["umarell_parity_model"] = model.parity_info
        chk["arm_arm_umarell_spheres"] = bool(ubad == 0)
    elif umarell_parity:
        rep["arm_arm_spheres"]["umarell_35_spheres_min_m"] = f"n/a ({model.parity_info.get('reason')})"
    # ---------------------------------------------------------------- MuJoCo hull distance (every pair, every sample)
    hull_pairs, dmin, hull_dd = {}, np.inf, {}
    n_aa, n_lt = 0, 0
    hmin = model.hull_min
    for i, j in pairs:
        ra, rb = rids[i], rids[j]
        dd = model.mj_pair_min_distance(ra, rb, Q[:, i], Q[:, j])
        hull_dd[(i, j)] = dd
        hit = dd < 0
        first = []
        for k in np.flatnonzero(hit)[:3]:
            first.append((float(ts[k]), model.mjw.pair_contacts_detail(ra, rb, Q[k, i], Q[k, j])[:1]))
        k = int(np.argmin(dd))
        hull_pairs[f"{ra}|{rb}"] = {"contact_samples": int(hit.sum()), "samples_lt_min": int((dd < hmin).sum()),
                                    "first": first, "min_signed_distance_m": _r(dd[k]), "t_min_s": _r(ts[k], 3)}
        n_aa += int(hit.sum())
        n_lt += int((dd < hmin).sum())
        dmin = min(dmin, float(dd[k]))
    # continuous-time certificate between the samples (trajgen.certify; round-4 finding): every sample pair whose
    # distance - (e_a + e_b) / 2 is below hull_min goes through the branch and bound
    from trajgen.certify import certify_rects, interval_disp
    cert_lb, cert_bad, cert_ev = np.inf, 0, 0
    hmin_nom = dmin                                # sampled minimum over all pairs
    # report the continuous minimum (branch and bound to 0.25 mm) only where the sampled one is within the detection
    # band (hull_min + 5 cm, minus 1 cm); else certify >= hull_min only (cheap)
    rep_min = bool(dmin < model.mjw.pair_band - 0.01)
    ref_nom = (min(hmin_nom, model.mjw.pair_band) + 0.002) if rep_min else hmin
    Dint = {rid: interval_disp(model, rid, Q[:, i]) for i, rid in enumerate(rids)}
    emax = {rid: model.step_margin(rid, Q[:, i]) for i, rid in enumerate(rids)}
    for i, j in pairs:
        ra, rb = rids[i], rids[j]
        dd = hull_dd[(i, j)]
        lip = 0.5 * (emax[ra] + emax[rb])
        b0 = dd - lip
        unc = np.flatnonzero(b0 < max(hmin, ref_nom))       # below hull_min, or near the reported minimum
        ok_part = np.ones(len(dd), bool)
        ok_part[unc] = False
        if ok_part.any():
            cert_lb = min(cert_lb, float(b0[ok_part].min()))
        if len(unc):
            cr = certify_rects(model, ra, rb, Q[:, i], Q[:, j], unc.astype(float), unc.astype(float), hmin,
                               DA=Dint[ra], DB=Dint[rb], d0=dd[unc], minimize=rep_min, prune_above=hmin_nom, diag=True)
            cert_lb = min(cert_lb, cr["lower_bound"])
            cert_bad += int(not cr["ok"])
            cert_ev += cr["n_evals"]
            hull_pairs[f"{ra}|{rb}"]["certified_lower_bound_m"] = _r(cr["lower_bound"])
    rep["arm_arm_hull"] = {"contact_samples": n_aa, "samples_lt_min": n_lt, "hull_min_distance_m": hmin,
                           "certified_continuous_min_m": (_r(min(cert_lb, model.mjw.pair_band)) if rep_min else None)
                           if pairs else None,
                           "certified_ok": bool(cert_bad == 0), "certify_evals": cert_ev,
                           "min_signed_distance_m": _r(dmin) if pairs else None,
                           "distance_cap_m": round(model.mjw.pair_band, 4), "distance_grid_hz": check_hz,
                           "pairs": hull_pairs, "enforced": bool(sc.collision.hull_backstop)}
    chk["arm_arm_hull"] = (n_aa == 0 and n_lt == 0 and cert_bad == 0) or not sc.collision.hull_backstop
    kinds = {"self": [], "floor": [], "box": []}
    for i, rid in enumerate(rids):
        for k in range(N):
            con = model.mj_single_contacts(rid, Q[k, i])
            for kk in kinds:
                if con[kk]:
                    kinds[kk].append((rid, float(ts[k]), con[kk][0]))
    sd = {}
    for i, rid in enumerate(rids):
        d = model.self_distance(rid, Q[::self_dist_every, i])
        sd[rid] = {"min_m": _r(d.min()), "t_min_s": _r(ts[::self_dist_every][int(d.argmin())], 3),
                   "samples_lt_min": int((d < sc.collision.self_min_distance).sum())}
    rep["self_hull"] = {"contact_samples": len(kinds["self"]), "first": kinds["self"][:3], "coal_distance": sd}
    chk["self_hull"] = not kinds["self"] and all(v["samples_lt_min"] == 0 for v in sd.values())
    # ---------------------------------------------------------------- single-arm static rules
    st = {}
    wdep = np.full(N, -np.inf)
    wexc = np.full(N, -np.inf)
    for i, rid in enumerate(rids):
        m = model.static_margins(rid, Q[:, i])
        wdep = np.maximum(wdep, m["wedge_depth"])
        wexc = np.maximum(wexc, m["wedge_excess"])
        st[rid] = {k2: _r(v.min()) for k2, v in m.items() if k2 not in ("wedge_depth", "wedge_excess")}
    rep["static"] = st
    rep["floor_mujoco_contact_samples"] = len(kinds["floor"])
    rep["box_hull"] = {"enabled": bool(model.box_hull_on), "min_distance_m": model.box_hull_min,
                       "workpiece_boxes": bool(getattr(model, "box_hull_wp", False)),
                       "violation_samples": len(kinds["box"]), "first": kinds["box"][:3]}
    fl = sc.floor
    chk["floor_spheres"] = (not fl.sphere_rule) or all(st[r]["floor_sphere"] >= 0 for r in rids)
    chk["floor_hull"] = ((not fl.hull_rule) or all(st[r]["floor_hull"] >= 0 for r in rids)) and not kinds["floor"]
    chk["boxes"] = all(st[r]["box"] is None or st[r]["box"] >= 0 for r in rids)
    chk["box_hull"] = (not model.box_hull_on) or not kinds["box"]
    if dense_hz is None:
        from trajgen.nodip import SCORE_HZ
        dense_hz = SCORE_HZ if getattr(model, "nodip", False) else 0.0
    dense_hz = float(dense_hz or 0.0)
    if dense_hz > check_hz and model.box_hull_on:
        bd = box_hull_dense(model, tl, col, T, dense_hz)
        rep["box_hull"]["dense"] = bd
        chk["box_hull"] = bool(chk["box_hull"] and bd["violation_samples"] == 0)
    chk["joint_limits"] = all(st[r]["joint"] >= -1e-9 for r in rids)
    bad = np.flatnonzero(wexc > 0)
    rep["rule_W"] = {"rule": sc.workpiece.rule, "per_weld_overrides": int(sum(w.rule_w is not None for w in task.welds)),
                     "max_depth_m": _r(wdep.max()) if np.isfinite(wdep.max()) else None,
                     "max_excess_over_tol_m": _r(wexc.max()) if np.isfinite(wexc.max()) else None,
                     "tol_m": sc.workpiece.tol, "violation_samples": int(len(bad)),
                     "first_violation_s": _r(ts[bad[0]], 3) if len(bad) else None}
    # rule W again on a dense TCP-only grid (the wedge is 20 mm; at 100 Hz a fast TCP moves ~9 mm per sample)
    if model.wedge_on:
        w_hz = max(float(check_hz), float(rule_w_hz))
        tw = np.arange(0.0, T + 1e-9, 1.0 / w_hz)
        nb, wmax, t_first = 0, -np.inf, None
        for s0 in range(0, len(tw), 20000):
            tc = tw[s0:s0 + 20000]
            Qw = tl.sample(tc)[:, col]
            ex = np.full(len(tc), -np.inf)
            for i, rid in enumerate(rids):
                ex = np.maximum(ex, model.wedge_excess(model.tcp(rid, Qw[:, i])[0]))
            kb = np.flatnonzero(ex > 0)
            nb += len(kb)
            wmax = max(wmax, float(ex.max()))
            if len(kb) and t_first is None:
                t_first = float(tc[kb[0]])
        rep["rule_W"]["dense_tcp"] = {"grid_hz": w_hz, "violation_samples": int(nb),
                                      "max_excess_over_tol_m": _r(wmax) if np.isfinite(wmax) else None,
                                      "first_violation_s": _r(t_first, 4)}
        bad = np.concatenate([bad, np.zeros(nb, int)])
    chk["rule_W"] = len(bad) == 0
    # ---------------------------------------------------------------- electrode off-weld rule (round 2, opt-in)
    if getattr(model, "electrode_enabled", False):
        if getattr(model, "nodip", False):
            rep["electrode_off_weld"] = nodip_check(model, tl, col, T, max(float(check_hz), dense_hz))
        else:
            rep["electrode_off_weld"] = electrode_check(model, Q, ts)
        chk["electrode_off_weld"] = bool(rep["electrode_off_weld"]["pass"])
    # ---------------------------------------------------------------- welds + strokes
    P = {rid: model.tcp(rid, Q[:, i]) for i, rid in enumerate(rids)}
    rows = []
    tolr = sc.tolerances
    pl = sc.planning
    seen = {}
    ax_local = sc.tool_axis_local
    for (rid, wid, t0, t1) in sorted(tl.weld_events, key=lambda e: e[2]):
        if wid not in model.W_ids or rid not in rids:
            rows.append({"robot": rid, "weld": wid, "error": "not a task weld" if rid in rids else "unknown robot"})
            continue
        w = task.weld(wid)
        seen[wid] = seen.get(wid, 0) + 1
        x, R = P[rid]
        ka = int(np.searchsorted(ts, t0 - 1e-9))
        kb = max(ka, int(np.searchsorted(ts, t1 + 1e-9)) - 1)
        ka = ka + int(np.argmin(np.linalg.norm(x[ka:kb + 1] - w.position, axis=1)))
        e = _pose_errors(x[ka], R[ka], w, ax_local)
        j = ka
        travel = 0.0
        while j > 0 and travel < tolr.entry_span:
            travel += np.linalg.norm(x[j] - x[j - 1])
            j -= 1
        mv = x[ka] - x[j]
        entry = float(np.degrees(np.arccos(np.clip(mv @ w.approach / (np.linalg.norm(mv) + 1e-12), -1, 1)))) \
            if np.linalg.norm(mv) > 1e-6 else None
        drift = float(np.linalg.norm(x[ka:kb + 1] - x[ka], axis=1).max()) if kb >= ka else 0.0
        d_in, lat_in, lat_out, on_axis_out = _stroke(x, ka, kb, w, pl)
        rows.append({"robot": rid, "weld": wid, "t_arrive_s": _r(t0, 3), "t_end_dwell_s": _r(t1, 3),
                     "pos_err_mm": _r(e["pos_m"] * 1000, 3), "tool_axis_err_deg": _r(e["tool_axis_deg"], 3),
                     "rot_err_deg": _r(e["rot_deg"], 2), "entry_dir_err_deg": _r(entry, 3),
                     "dwell_drift_mm": _r(drift * 1000, 3), "descent_straight_m": _r(d_in, 4),
                     "descent_lateral_mm": _r(lat_in * 1000, 3),
                     "liftoff_lateral_mm": _r(lat_out * 1000, 3), "liftoff_on_axis_m": _r(on_axis_out, 4)})
        r_ = rows[-1]
        ok = bool(e["pos_m"] <= tolr.position and e["tool_axis_deg"] <= tolr.tool_axis_deg
                  and (entry is not None and entry <= tolr.entry_direction_deg))
        if not tolr.spin_free:
            ok = ok and e["rot_deg"] <= tolr.spin_tol_deg + tolr.tool_axis_deg
        if w.allowed_robots is not None and rid not in w.allowed_robots:
            ok = False
            r_["error"] = "robot not in allowed_robots"
        r_["ok"] = ok
        r_["stroke_ok"] = bool(d_in >= pl.min_offset - 1e-6 and on_axis_out >= pl.min_offset - 1e-6)
    missing = [w.id for w in task.welds if w.id not in seen]
    good = [r for r in rows if "pos_err_mm" in r]
    rep["welds"] = {"rows": rows, "n_ok": sum(bool(r.get("ok")) for r in rows), "n_task": len(task.welds),
                    "missing": missing, "duplicates": [k2 for k2, v in seen.items() if v > 1],
                    "worst_pos_err_mm": _r(max((r["pos_err_mm"] for r in good), default=None), 3),
                    "worst_tool_axis_err_deg": _r(max((r["tool_axis_err_deg"] for r in good), default=None), 3),
                    "worst_spin_rot_err_deg": _r(max((r["rot_err_deg"] for r in good), default=None), 2),
                    "spin_free": bool(tolr.spin_free),
                    "worst_entry_dir_err_deg": _r(max((r["entry_dir_err_deg"] or 0 for r in good), default=None), 3),
                    "min_descent_straight_m": _r(min((r["descent_straight_m"] for r in good), default=None), 4),
                    "min_liftoff_on_axis_m": _r(min((r["liftoff_on_axis_m"] for r in good), default=None), 4)}
    chk["welds"] = (rep["welds"]["n_ok"] == len(task.welds) and not missing and not rep["welds"]["duplicates"])
    chk["straight_strokes"] = all(r.get("stroke_ok", False) for r in good)
    # ---------------------------------------------------------------- end + dynamics + skew tolerance
    home = np.stack([model.robots[r].home for r in rids])
    end_err = float(np.abs(tl.sample(np.array([T]))[0][col] - home).max())
    rep["end_at_home_max_rad"] = _r(end_err, 5)
    chk["ends_at_home"] = end_err < 1e-3
    if dynamics:
        v_max = np.stack([model.robots[r].v_max for r in rids])
        a_max = np.stack([model.robots[r].a_max for r in rids])
        rep["dynamics"] = plan_dynamics_n(lambda x: tl.sample(x)[:, col], T, v_max, a_max, list(rids))
    if pairs and (skew_s or skew_window_s):
        tt = timing_tolerance(model, Q, ts, skew_window_s=skew_window_s, rows_s=tuple(skew_s or ()),
                              max_s=timing_max_s)
        rep["skew_tolerance"] = tt
        if sc.collision.hull_backstop and skew_window_s:
            chk["arm_arm_hull_skew"] = bool(tt["window"]["hull_ok"])
    if continuous:
        from trajgen.certify import knots_on_grid
        cc = certifier(model).certify_timeline(tl, t_end=T, umarell_parity=um is not None)
        grid_ok = knots_on_grid(tl, check_hz, t_end=T)
        cc["knots_on_check_grid"] = grid_ok
        skew_used = bool(pairs and skew_window_s and sc.collision.hull_backstop)
        cc["skew_window_exact"] = (grid_ok if skew_used else None)
        rep["continuous"] = cc
        chk["continuous"] = bool(cc["all_ok"] and (grid_ok or not skew_used))
    rep["checks"] = chk
    rep["all_pass"] = bool(all(chk.values()))
    return rep


def electrode_check(model, Q, ts) -> dict:
    """The electrode off-weld rule on sampled configurations ``Q`` (N, R, 6) in the scene's robot order at times
    ``ts``: per sample and robot whose TCP is farther than the rule's radius from every weld, the penetration
    min_distance - (electrode hull(s) - workpiece box signed distance, MuJoCo). Fails where it exceeds score_tol.
    Report: max_penetration_m (>= 0), t / robot / box of it, n_samples (samples above score_tol), n_samples_pen
    (samples with any penetration), n_off_weld_samples (robot samples off every weld)."""
    er = model.electrode_rule
    rids = model.rids
    best, at = 0.0, (None, None, None)
    n_bad = n_pen = n_off = 0
    per = {}
    for i, rid in enumerate(rids):
        mg, bx = model.electrode_margin(rid, Q[:, i], return_box=True)
        off = np.isfinite(mg)
        n_off += int(off.sum())
        pen = -mg
        nb, npn = int((pen > er.score_tol).sum()), int((pen > 0).sum())
        n_bad += nb
        n_pen += npn
        k = int(np.argmax(pen)) if len(pen) else 0
        pk = float(max(pen[k], 0.0)) if len(pen) else 0.0
        per[rid] = {"max_penetration_m": _r(pk, 5), "n_samples": nb, "n_samples_pen": npn}
        if pk > best:
            b = int(bx[k])
            best, at = pk, (float(ts[k]), rid, model.scene.boxes[b].id if b >= 0 else None)
    return {"enabled": True, "radius_m": er.radius, "min_distance_m": er.min_distance, "score_tol_m": er.score_tol,
            "max_penetration_m": _r(best, 5), "t": _r(at[0], 4), "robot": at[1], "box": at[2], "n_samples": n_bad,
            "n_samples_pen": n_pen, "n_off_weld_samples": n_off, "grid_hz": _r(1.0 / float(ts[1] - ts[0]), 3)
            if len(ts) > 1 else None, "per_robot": per, "pass": bool(n_bad == 0),
            "note": "penetration = min_distance - electrode/workpiece signed distance while the TCP is > radius from "
                    "every weld; sampled on the check grid (the continuous certificate is in 'continuous')"}


def dense_grid(T: float, hz: float) -> np.ndarray:
    """Uniform grid 0, 1/hz, ... with T appended (the check grid's construction)."""
    ts = np.arange(0.0, T + 1e-9, 1.0 / hz)
    if ts[-1] < T - 1e-9:
        ts = np.append(ts, T)
    return ts


def _contact_windows(model, rid, ts, P, events) -> np.ndarray:
    """(N,) index into the task's welds of the weld whose contact window holds each sample (-1: none): the dwell of
    one of the robot's weld events [t_arrive, t_end_dwell], extended over the contiguous samples on both sides with
    the TCP inside the contact ball (trajgen.nodip); only samples with the TCP inside the ball count."""
    er = model.electrode_rule
    N = len(ts)
    ex = np.full(N, -1, int)
    for (r, wid, t0, t1) in events:
        if r != rid or wid not in model.W_ids:
            continue
        wi = model.W_ids.index(wid)
        inball = np.linalg.norm(P - model.W_P[wi], axis=1) <= er.radius
        ka = int(np.searchsorted(ts, t0 - 1e-9))
        kb = int(np.searchsorted(ts, t1 + 1e-9)) - 1
        core = np.flatnonzero(inball[ka:kb + 1]) + ka
        if not len(core):
            continue
        lo, hi = int(core[0]), int(core[-1])
        while lo > 0 and inball[lo - 1]:
            lo -= 1
        while hi < N - 1 and inball[hi + 1]:
            hi += 1
        seg = np.arange(lo, hi + 1)
        seg = seg[inball[seg]]
        ex[seg] = wi
    return ex


def _nodip_between(model, tl, ci, rid, ts, Q, mg, ex, min_dt: float = 1e-6) -> dict:
    """Between-sample certificate of the no-dip rule for robot ``rid`` (round-3 verification: a 0.45 ms, 0.07 mm
    electrode dip of a transit fell between two 400 Hz samples, 94 mm from any weld).

    On every grid interval whose two ends are outside the contact windows (no exemption there) the margin m is the
    electrode - workpiece signed distance, 1-Lipschitz in the electrode's point displacements, and a joint step dq
    moves any electrode point by at most sum_j |dq_j| R_j (the electrode chain radii of trajgen.certify). The path is
    piecewise linear between the timeline's knots, so the knots inside an interval are nodes too. An interval is
    certified when 0.5 (m_a + m_b - D) >= 0; otherwise it is bisected (``trajgen.certify._bisect``, down to
    ``min_dt``) and every new node is evaluated exactly. Intervals touching a contact window (contact make / break
    at the weld) are left to the dense check: their exemption depends on contact points, which are not Lipschitz.
    Returns the _bisect report (ok, violations, certified_lower_bound, nodes, uncertified) + intervals refined."""
    from trajgen.certify import _bisect, _knots
    cert = certifier(model)
    R = cert.R_elec.get(rid)
    out = {"intervals": int(max(len(ts) - 1, 0)), "refined": 0, "ok": True, "violations": [], "uncertified": 0,
           "nodes": 0, "certified_lower_bound": None}
    if R is None or len(ts) < 2:
        return out
    Re = np.asarray(R, float)[None]
    off = (ex[:-1] < 0) & (ex[1:] < 0) & np.isfinite(mg[:-1]) & np.isfinite(mg[1:]) & (mg[:-1] >= 0) & (mg[1:] >= 0)
    D = np.abs(np.diff(Q, axis=0)) @ Re[0]
    lb = 0.5 * (mg[:-1] + mg[1:] - D)
    cand = np.flatnonzero(off & (lb < 0.0))
    out["refined"] = int(len(cand))
    if not len(cand):
        fin = lb[off]
        out["certified_lower_bound"] = float(np.floor(fin.min() * 1e6) / 1e6) if len(fin) else None
        return out
    K = _knots(tl, {rid}, float(ts[-1]))
    ta, tb = [], []
    for i in cand:
        a, b = float(ts[i]), float(ts[i + 1])
        inner = K[(K > a + 1e-12) & (K < b - 1e-12)]
        nodes = np.concatenate([[a], inner, [b]])
        ta.append(nodes[:-1])
        tb.append(nodes[1:])
    ta, tb = np.concatenate(ta), np.concatenate(tb)

    def ends(ta_, tb_):
        Qx = tl.sample(np.concatenate([ta_, tb_]))[:, ci]
        n = len(ta_)
        return Qx[:n], Qx[n:]

    def lb_e(ta_, tb_):
        Qa, Qb = ends(ta_, tb_)
        va = model.electrode_margin(rid, Qa, welds=(), min_distance=0.0)
        vb = model.electrode_margin(rid, Qb, welds=(), min_distance=0.0)
        b = 0.5 * (va + vb - np.abs(Qb - Qa) @ Re[0])
        return b, np.minimum(va, vb), np.full(len(b), rid)
    r = _bisect(lb_e, ta, tb, 0.0, strict=False, minimize=False, tol=1e-4, cap=0.02, min_dt=min_dt)
    r["intervals"], r["refined"] = out["intervals"], out["refined"]
    return r


def nodip_check(model, tl, col, T: float, hz: float, certify: bool = True) -> dict:
    """The no-dip rule (trajgen.nodip) on a uniform grid at ``hz`` (>= SCORE_HZ): per robot and sample the margin
    ``SceneModel.electrode_margin(..., min_distance=0)``, the exempting weld being the one whose contact window holds
    the sample (``_contact_windows``; none elsewhere). Fails where the margin < -SCORE_TOL_M (any penetration outside
    the exemption, or a contact inside it deeper than contact_depth / outside the contact point radius).
    ``certify`` (default, round-3 fix): also between the samples outside the contact windows (``_nodip_between``):
    a penetration found there fails too (``between``), so a dip shorter than the grid step cannot pass.
    Report: max_penetration_m (max(0, -margin), the violation measure), t / robot / box of it, n_samples (failing
    samples), n_samples_pen, contact {n_window_samples, max_depth_m (electrode penetration inside the windows),
    n_welds}, per_robot, between {ok, violations, refined, nodes, uncertified, per_robot}."""
    from trajgen import nodip as N
    er = model.electrode_rule
    rids = model.rids
    ts = dense_grid(T, hz)
    events = list(tl.weld_events)
    best, at, at_wd = 0.0, (None, None, None), None
    n_bad = n_pen = n_win = 0
    cmax = 0.0
    per = {}
    btw = {"certified": True, "ok": True, "violations": [], "refined": 0, "nodes": 0, "uncertified": 0,
           "min_dt_s": 1e-6, "per_robot": {}}
    for i, rid in enumerate(rids):
        Q = tl.sample(ts)[:, col[i]]
        P = model.tcp(rid, Q)[0]
        ex = _contact_windows(model, rid, ts, P, events)
        mg = np.full(len(ts), np.inf)
        bx = np.full(len(ts), -1, int)
        off = np.flatnonzero(ex < 0)
        if len(off):
            mg[off], bx[off] = model.electrode_margin(rid, Q[off], return_box=True, welds=(), min_distance=0.0)
        for wi in np.unique(ex[ex >= 0]):
            k = np.flatnonzero(ex == wi)
            mg[k], bx[k] = model.electrode_margin(rid, Q[k], return_box=True, welds=[int(wi)], min_distance=0.0)
            d, _ = model.electrode_distance(rid, Q[k], exact_below=0.0)
            cmax = max(cmax, float(np.maximum(-d, 0.0).max()))
        n_win += int((ex >= 0).sum())
        pen = np.maximum(-mg, 0.0)
        nb, npn = int((-mg > N.SCORE_TOL_M).sum()), int((pen > 0).sum())
        n_bad += nb
        n_pen += npn
        k = int(np.argmax(pen))
        pk = float(pen[k])
        wd = float(model.weld_distance(P[k:k + 1])[0]) if len(model.W_P) else None
        per[rid] = {"max_penetration_m": _r(pk, 5), "n_samples": nb, "n_samples_pen": npn,
                    "n_window_samples": int((ex >= 0).sum()), "weld_dist_m": _r(wd, 4) if pk > 0 else None}
        if pk > best:
            b = int(bx[k])
            best, at = pk, (float(ts[k]), rid, model.scene.boxes[b].id if b >= 0 else None)
            at_wd = wd
        if certify:
            bt = _nodip_between(model, tl, col[i], rid, ts, Q, mg, ex)
            btw["per_robot"][rid] = {k: bt.get(k) for k in ("ok", "refined", "nodes", "uncertified",
                                                            "certified_lower_bound")}
            btw["refined"] += int(bt.get("refined") or 0)
            btw["nodes"] += int(bt.get("nodes") or 0)
            btw["uncertified"] += int(bt.get("uncertified") or 0)
            for v in bt.get("violations") or []:
                pen_b = max(0.0, -float(v["value"]))
                btw["violations"].append({"t_s": v["t_s"], "robot": rid, "penetration_m": _r(pen_b, 6)})
                if pen_b > best:
                    tq = tl.sample(np.array([float(v["t_s"])]))[:, col[i]]
                    _m, bq = model.electrode_margin(rid, tq, return_box=True, welds=(), min_distance=0.0)
                    best = pen_b
                    at = (float(v["t_s"]), rid, model.scene.boxes[int(bq[0])].id if int(bq[0]) >= 0 else None)
                    at_wd = float(model.weld_distance(model.tcp(rid, tq)[0])[0]) if len(model.W_P) else None
    if certify:
        btw["ok"] = not btw["violations"]
        btw["violations"] = btw["violations"][:10]
    return {"enabled": True, "rule": N.RULE_ID, "version": int(er.version), "radius_m": er.radius,
            "contact_point_radius_m": er.contact_point_radius, "contact_depth_m": er.contact_depth,
            "min_distance_m": 0.0, "plan_min_distance_m": er.min_distance, "score_tol_m": N.SCORE_TOL_M,
            "max_penetration_m": _r(best, 5), "t": _r(at[0], 4), "robot": at[1], "box": at[2],
            "weld_dist_m": _r(at_wd, 4) if at_wd is not None else None, "n_samples": n_bad,
            "n_samples_pen": n_pen, "grid_hz": _r(hz, 3), "samples": int(len(ts)),
            "contact": {"n_window_samples": n_win, "max_depth_m": _r(cmax, 5), "n_welds": len(events)},
            "per_robot": per, "between": btw if certify else {"certified": False},
            "pass": bool(n_bad == 0 and (not certify or btw["ok"])),
            "note": "no-dip rule: penetration = -(electrode/workpiece signed distance) outside the contact exemption "
                    "(robot's own weld, dwell + contact make/break, TCP within radius_m, contact point within "
                    "contact_point_radius_m, depth <= contact_depth_m); dense grid + a between-sample certificate "
                    "outside the contact windows ('between'); the continuous certificate of the static form is in "
                    "'continuous'"}


def box_hull_dense(model, tl, col, T: float, hz: float) -> dict:
    """Hull-vs-box rule (MuJoCo world model: every hull vs the obstacle boxes, every hull but the electrode vs the
    workpiece boxes when they are hull geoms) on a uniform grid at ``hz``: violation where the signed distance <
    obstacles.box_hull.min_distance_m. Exact screen: the hull cover spheres (SceneModel.hull_cover, which contain
    every hull) against the box distance field (a lower bound of the distance to the union of the boxes)."""
    ts = dense_grid(T, hz)
    bmin = model.box_hull_min
    nb, dmin, first = 0, np.inf, []
    F, lo, step, pad = model._box_field()
    slack = 0.5 * step * np.sqrt(3.0)
    for i, rid in enumerate(model.rids):
        Q = tl.sample(ts)[:, col[i]]
        K, C, Rr = model.hull_cover(rid)
        Rs, ps = model.rm[rid].joint_poses(Q)
        X = np.einsum("nhij,hj->nhi", Rs[:, K], C) + ps[:, K]                  # (N, H, 3)
        ii = np.rint((X.reshape(-1, 3) - lo) / step).astype(int)
        inside = np.all((ii >= 0) & (ii < np.array(F.shape)), axis=1)
        f = np.full(len(ii), pad)
        jj = ii[inside]
        f[inside] = F[jj[:, 0], jj[:, 1], jj[:, 2]]
        lb = (f.reshape(len(Q), -1) - slack - Rr[None]).min(1)
        need = np.flatnonzero(lb < bmin + 1e-4)
        if not len(need):
            continue
        d = model.mjw.box_distance(rid, Q[need])
        bad = np.flatnonzero(d < bmin)
        nb += len(bad)
        dmin = min(dmin, float(d.min()))
        for k in bad[:3 - len(first)] if len(first) < 3 else []:
            con = model.mj_single_contacts(rid, Q[need[k]])["box"][:1]
            first.append((rid, float(ts[need[k]]), con[0] if con else None))
    return {"grid_hz": _r(hz, 3), "samples": int(len(ts)), "violation_samples": int(nb),
            "min_distance_m": bmin, "min_signed_distance_m": _r(dmin) if np.isfinite(dmin) else None,
            "first": first}


def certifier(model):
    """The model's cached ``trajgen.certify.ContinuousCertifier``."""
    c = getattr(model, "_cont_certifier", None)
    if c is None:
        from trajgen.certify import ContinuousCertifier
        c = ContinuousCertifier(model)
        model._cont_certifier = c
    return c


def _ordered_min(lb, exact, cap: float = np.inf, chunk: int = 256) -> float:
    """min(cap, min over samples of exact(idx)) given per-sample lower bounds lb <= exact: samples are evaluated in
    order of increasing bound until the bound reaches the best value found."""
    order = np.argsort(lb, kind="stable")
    best = float(cap)
    for a in range(0, len(order), chunk):
        idx = order[a:a + chunk]
        if lb[idx[0]] >= best:
            break
        idx = idx[lb[idx] < best]
        if len(idx):
            best = min(best, float(np.min(exact(idx))))
    return best


def timing_tolerance(model, Q, ts, *, skew_window_s: float = 0.1, rows_s=(0.05, 0.1), max_s: float = 0.3) -> dict:
    """Arm-arm clearance with one robot's whole program shifted against the others (dispatch skew, or a constant
    motion lag of that robot behind its dispatches with the completion events unchanged), on the uniform grid ``ts``
    (shifts in whole grid samples, 1 / check_hz):
      * ``window``: every robot pair at every shift |delta| <= skew_window_s (the planner's window): corrected-sphere
        minimum (informational) and the robot-robot hull distance, gated >= hull_min_distance_m;
      * ``rows``: corrected-sphere (and umarell-sphere) minimum per robot at +-rows_s (the round-2 skew rows);
      * ``per_robot``: the largest constant lag / lead (s, scanned up to max_s) of one robot for which the sphere
        gate, hull contact-freedom (distance >= 0) and the hull buffer (>= hull_min) still hold at every smaller shift.
    Exact minima: sphere clearances and MuJoCo hull distances (capped at hull_min + 5 cm) are evaluated in order of
    a per-sample lower bound (bounding spheres; SceneModel.hull_cover) until the bound passes the best value."""
    rids = model.rids
    R = len(rids)
    gate = model.scene.collision.arm_arm_gate
    pr = model.scene.collision.pair_radius
    hmin = model.hull_min
    cap = model.mjw.pair_band
    dt = float(ts[1] - ts[0]) if len(ts) > 1 else 0.01
    N = len(ts)
    kmax = int(round(max_s / dt))
    kwin = int(round(skew_window_s / dt))
    S = [model.pair_spheres(r, Q[:, i]) for i, r in enumerate(rids)]
    U = [model.umarell_spheres(r, Q[:, i]) for i, r in enumerate(rids)] if model.umarell is not None else None
    # per-sample bounding spheres of each robot's pair spheres and of its hull cover spheres (lower bounds that
    # order / prune the exact evaluations; the minima below are exact)
    def groups(x, G=10):
        """Spheres clustered by their positions along the timeline (farthest-point seeds in the space of 24 sampled
        poses, nearest-seed assignment): (N, G, 3) group centres and (N, G) bounding radii per sample."""
        n = x.shape[1]
        F = x[np.linspace(0, len(x) - 1, min(24, len(x))).astype(int)].transpose(1, 0, 2).reshape(n, -1)
        seeds = [0]
        dmin = np.linalg.norm(F - F[0], axis=1)
        while len(seeds) < min(G, n):
            k = int(np.argmax(dmin))
            seeds.append(k)
            dmin = np.minimum(dmin, np.linalg.norm(F - F[k], axis=1))
        lab = np.argmin(np.linalg.norm(F[:, None] - F[seeds][None], axis=-1), axis=1)
        cs, rs = [], []
        for g in np.unique(lab):
            y = x[:, lab == g]
            c = y.mean(1)
            cs.append(c)
            rs.append(np.linalg.norm(y - c[:, None], axis=-1).max(1))
        return np.stack(cs, 1), np.stack(rs, 1)

    GS = [groups(x) for x in S]
    GS = [(c.astype(np.float32), r.astype(np.float32)) for c, r in GS]
    GH = []                       # hull cover spheres grouped by joint frame (rigid): (N, F, 3) centres, (N, F) radii
    for i, r in enumerate(rids):
        K, C, Rr = model.hull_cover(r)
        Rs, ps = model.rm[r].joint_poses(Q[:, i])
        cs, rs = [], []
        for k in np.unique(K):
            Ck, Rk = C[K == k], Rr[K == k]
            c0 = Ck.mean(0)
            cs.append(np.einsum("nij,j->ni", Rs[:, k], c0) + ps[:, k])
            rs.append(np.full(len(Q), float((np.linalg.norm(Ck - c0, axis=1) + Rk).max())))
        GH.append((np.stack(cs, 1).astype(np.float32), np.stack(rs, 1).astype(np.float32)))

    def glb(G1, G2, ii, jj):
        """(len) min over group pairs of centre distance - radii (robot 1 at samples ii, robot 2 at jj)."""
        (c1, r1), (c2, r2) = G1, G2
        d = c1[ii][:, :, None] - c2[jj][:, None]
        return (np.sqrt(np.einsum("nabk,nabk->nab", d, d)) - r1[ii][:, :, None] - r2[jj][:, None]).reshape(len(ii), -1).min(1)

    shifts = np.arange(-kmax, kmax + 1)
    ar = np.arange(N)
    # sph[(i, j)][m], hul[(i, j)][m]: robot i shifted by shifts[m] samples (q_i(t - delta)) vs robot j nominal
    sph, hul = {}, {}
    for i in range(R):
        for j in range(i + 1, R):
            cs, hs = np.empty(len(shifts)), np.full(len(shifts), cap)
            for m, k in enumerate(shifts):
                ii = np.clip(ar - k, 0, N - 1)
                lb = glb(GS[i], GS[j], ii, ar).astype(float) - 2 * pr
                cs[m] = _ordered_min(lb, lambda idx: model.pair_clearance_spheres(S[i][ii[idx]], S[j][idx], pr))
                lbh = glb(GH[i], GH[j], ii, ar).astype(float) - 1e-4
                hs[m] = _ordered_min(lbh, lambda idx: model.mjw.pair_min_distance(rids[i], rids[j], Q[ii[idx], i],
                                                                                Q[idx, j]), cap=cap)
            sph[(i, j)], hul[(i, j)] = cs, hs
    mid = kmax
    win = slice(mid - min(kwin, kmax), mid + min(kwin, kmax) + 1)
    wrows = {}
    hw, sw = np.inf, np.inf
    for (i, j), cs in sph.items():
        h = float(hul[(i, j)][win].min())
        c = float(cs[win].min())
        wrows[f"{rids[i]}|{rids[j]}"] = {"sphere_min_m": _r(c), "hull_min_m": _r(h),
                                         "hull_at_shift_s": None if h >= cap - 1e-12 else
                                         _r(float(shifts[win][int(np.argmin(hul[(i, j)][win]))] * dt), 3)}
        hw, sw = min(hw, h), min(sw, c)
    # continuous certificate over the window (time between samples and shifts between grid shifts, trajgen.certify)
    from trajgen.certify import certify_rects, interval_disp
    Dint = [interval_disp(model, r, Q[:, i]) for i, r in enumerate(rids)]
    emax = [model.step_margin(r, Q[:, i]) for i, r in enumerate(rids)]
    c_lb, c_bad, c_ev = cap, 0, 0
    rep_w = bool(hw < cap - 0.01)                  # report the continuous minimum (else certify >= hull_min only)
    ref_w = (hw + 0.002) if rep_w else hmin
    for i in range(R):
        for j in range(i + 1, R):
            for k in shifts[win]:
                ii = np.clip(ar - k, 0, N - 1)
                lip = 0.5 * (emax[i][ii] + emax[j])
                lbh = glb(GH[i], GH[j], ii, ar).astype(float) - 1e-4
                cand = np.flatnonzero(lbh - lip < max(hmin, ref_w))
                if not len(cand):
                    continue
                dd = model.mjw.pair_min_distance(rids[i], rids[j], Q[ii[cand], i], Q[cand, j])
                b0 = dd - lip[cand]
                okb = b0 >= max(hmin, ref_w)
                if okb.any():
                    c_lb = min(c_lb, float(b0[okb].min()))
                unc = np.flatnonzero(~okb)                  # below hull_min or near the minimum
                if len(unc):
                    cr = certify_rects(model, rids[i], rids[j], Q[:, i], Q[:, j], ii[cand[unc]].astype(float),
                                       cand[unc].astype(float), hmin, DA=Dint[i], DB=Dint[j], d0=dd[unc],
                                       minimize=rep_w, prune_above=hw, band=float(min(kwin, kmax)))
                    c_lb = min(c_lb, cr["lower_bound"])
                    c_bad += int(not cr["ok"])
                    c_ev += cr["n_evals"]
    out = {"gate_m": gate, "hull_min_distance_m": hmin, "hull_distance_cap_m": round(cap, 4), "grid_s": round(dt, 4),
           "window": {"skew_s": round(min(kwin, kmax) * dt, 4), "sphere_min_m": _r(sw), "hull_min_m": _r(hw),
                      "certified_hull_min_m": _r(c_lb) if rep_w else None, "certified_ok": bool(c_bad == 0),
                      "certify_evals": c_ev, "certified_note": "certified lower bound of the continuous minimum over the "
                      "window (time between samples and shifts between the grid shifts; branch and bound, within "
                      "~0.5 mm of the true minimum in the tested cases); None: the sampled minimum is at the "
                      "detection cap, certified >= hull_min only",
                      "hull_ok": bool(hw >= hmin - 1e-9 and c_bad == 0), "pairs": wrows}}

    def per_robot_series(i):
        """(sphere, hull) min over the other robots, for robot i shifted by shifts (late > 0)."""
        cs, hs = np.full(len(shifts), np.inf), np.full(len(shifts), np.inf)
        for j in range(R):
            if j == i:
                continue
            if i < j:
                c, h = sph[(i, j)], hul[(i, j)]
            else:                                         # robot i late by d == robot j early by d
                c, h = sph[(j, i)][::-1], hul[(j, i)][::-1]
            cs, hs = np.minimum(cs, c), np.minimum(hs, h)
        return cs, hs

    def reach(ok, sign):
        """largest |shift| (s) such that ok holds at every shift between 0 and it (in direction sign)."""
        idx = mid + sign * np.arange(0, kmax + 1)
        bad = np.flatnonzero(~ok[idx])
        n = (bad[0] - 1) if len(bad) else kmax
        return None if n < 0 else round(float(n * dt), 4), bool(len(bad) == 0)

    rows, urows, prow = {}, {}, {}
    for i, r in enumerate(rids):
        cs, hs = per_robot_series(i)
        rr = {}
        for name, ok in (("sphere_gate", cs > gate), ("hull_contact_free", hs >= 0), ("hull_min", hs >= hmin - 1e-9)):
            for sign, lab in ((1, "late"), (-1, "early")):
                v, full = reach(ok, sign)
                rr[f"{name}_{lab}_s"] = v if not full else f">= {v}"
        prow[r] = rr
        for dlt in rows_s:
            for sgn in (-1, 1):
                k = mid + sgn * int(round(dlt / dt))
                if 0 <= k < len(shifts):
                    rows[f"{r} {'+' if sgn > 0 else '-'}{dlt:g}s"] = _r(cs[k])
        if U is not None:
            for dlt in rows_s:
                for sgn in (-1, 1):
                    kk = sgn * int(round(dlt / dt))
                    ii = np.clip(np.arange(N) - kk, 0, N - 1)
                    cu = min(float(model.pair_clearance_spheres(U[i][ii], U[j], model.umarell.pair_radius).min())
                             for j in range(R) if j != i)
                    urows[f"{r} {'+' if sgn > 0 else '-'}{dlt:g}s"] = _r(cu)
    out["rows"] = rows
    vals = [v for v in rows.values() if v is not None]
    out["min_clearance_m"] = min(vals) if vals else None
    out["all_above_gate"] = bool(vals and min(vals) > gate)
    if urows:
        out["umarell_rows"] = urows
        out["umarell_min_clearance_m"] = min(urows.values())
        out["umarell_all_above_gate"] = bool(min(urows.values()) > model.umarell.warning_distance)
    out["per_robot"] = prow
    out["scan_max_s"] = round(kmax * dt, 4)
    out["note"] = ("per_robot: largest constant lag (late) / lead (early) of one robot against all others that keeps "
                   "each criterion at every smaller shift; '>= x' = holds over the whole scan")
    return out


def skew_tolerance(model, tl, col, ts, deltas=(0.05, 0.1)) -> dict:
    """Back-compatible wrapper: timing_tolerance on the timeline sampled at ``ts``."""
    Q = tl.sample(ts)[:, col]
    return timing_tolerance(model, Q, ts, rows_s=tuple(deltas), max_s=max(deltas), skew_window_s=max(deltas))


def lag_tolerance(model, plan, tl, *, dt: float, margin: float, check_hz: float = 100.0, per_leg_s: float = 0.05,
                  n_mc: int = 10, seed: int = 0) -> dict:
    """Informational: robustness to the executor's completion lateness, which accumulates along each robot's
    program (umarell's bt_executor polls at 20 Hz, so every leg can complete up to 50 ms late; WaitForRobot gates
    re-synchronise only where they bind). The plan is replayed (trajgen.btplan.replay, gates honoured) with
      * one robot late by ``per_leg_s`` on every leg (k x per_leg_s after its k-th leg), for each robot;
      * every leg of every robot late by U(0, per_leg_s), ``n_mc`` seeds;
    and the corrected-sphere arm-arm clearance of every pair is evaluated at ``check_hz``."""
    from trajgen.btplan import replay
    rids = model.rids
    home = np.stack([model.robots[r].home for r in rids])
    gate = model.scene.collision.arm_arm_gate
    pairs = [(i, j) for i in range(len(rids)) for j in range(i + 1, len(rids))]

    pr_r = model.scene.collision.pair_radius
    cap = model.mjw.pair_band

    def clearance(tlx):
        """(sphere min, pair, t, hull min) of a replayed timeline."""
        T = float(tlx.t[-1])
        ts = np.arange(0.0, T + 1e-9, 1.0 / check_hz)
        col = [list(tlx.robot_ids).index(r) for r in rids]
        Q = tlx.sample(ts)[:, col]
        best = (np.inf, None, None)
        hbest = cap
        for i, j in pairs:
            c = model.pair_clearance(rids[i], rids[j], Q[:, i], Q[:, j])
            k = int(np.argmin(c))
            if c[k] < best[0]:
                best = (float(c[k]), f"{rids[i]}|{rids[j]}", float(ts[k]))
            near = np.flatnonzero(c < model.hull_extent(rids[i]) + model.hull_extent(rids[j]) - 2 * pr_r + cap)
            if len(near):
                hbest = min(hbest, float(model.mj_pair_min_distance(rids[i], rids[j], Q[near, i], Q[near, j]).min()))
        return best + (hbest,)

    hmin = model.hull_min
    rows = {}
    for r in rids:
        c, pr, t, h = clearance(replay(plan, rids, home, dt=dt, margin=margin,
                                       lag=lambda rid, k, r=r: per_leg_s if rid == r else 0.0))
        rows[f"{r} +{per_leg_s:g} s/leg"] = {"min_clearance_m": _r(c), "pair": pr, "t_s": _r(t, 2), "hull_min_m": _r(h)}
    rng = np.random.default_rng(seed)
    mc, mch = [], []
    for s in range(n_mc):
        draws = {}
        f = lambda rid, k: draws.setdefault((rid, k), float(rng.uniform(0.0, per_leg_s)))
        c, _, _, h = clearance(replay(plan, rids, home, dt=dt, margin=margin, lag=f))
        mc.append(c)
        mch.append(h)
    worst_one = min(v["min_clearance_m"] for v in rows.values()) if rows else None
    worst_h = min(v["hull_min_m"] for v in rows.values()) if rows else None
    return {"gate_m": gate, "hull_min_distance_m": hmin, "hull_distance_cap_m": round(cap, 4), "per_leg_s": per_leg_s,
            "one_robot_late": rows, "one_robot_late_min_m": worst_one,
            "one_robot_late_above_gate": bool(worst_one is not None and worst_one > gate),
            "one_robot_late_hull_min_m": worst_h,
            "one_robot_late_hull_ok": bool(worst_h is not None and worst_h >= hmin - 1e-9),
            "random_all_late_min_m": _r(min(mc)) if mc else None, "random_n": n_mc,
            "random_all_late_above_gate": bool(mc and min(mc) > gate),
            "random_all_late_hull_min_m": _r(min(mch)) if mch else None,
            "random_all_late_hull_ok": bool(mch and min(mch) >= hmin - 1e-9),
            "note": "informational; accumulated completion lateness (k x per_leg_s after a robot's k-th leg). The "
                    "planner holds the gate and the hull buffer for a +-skew constant shift of any one robot "
                    "(skew_tolerance.window, gated) and every leg start is held relative to the other robots' "
                    "completed operations; the constant-lag tolerance per robot is skew_tolerance.per_robot"}


def _active_leg(legs_by_robot, rid, t):
    """op_id of robot rid's leg active at time t (dispatch .. completion; a gated wait belongs to the next leg)."""
    L = legs_by_robot.get(rid, [])
    for l in L:
        if t <= float(getattr(l, "t_done", l.t1)) + 1e-9:
            return l.op_id
    return "after_last"


def lateness_replay_check(model, plan, *, dt: float, margin: float, lag, check_hz: float = 100.0,
                          n_viol: int = 12, continuous: bool = False) -> dict:
    """Replay ``plan`` (trajgen.btplan.replay, gates honoured) with the completion lateness ``lag(rid, k)`` and
    check every robot pair on the replay: corrected-sphere clearance > gate and robot-robot hull distance >=
    hull_min (exact MuJoCo where the hull-cover lower bound is below hull_min). Returns minima, pass and up to
    ``n_viol`` distinct violating (pair, active leg of each robot) entries. ``continuous``: also certify both
    between the samples (``trajgen.certify``, arm-arm part); failing nodes are added to the violations."""
    from trajgen.btplan import replay
    rids = model.rids
    home = np.stack([model.robots[r].home for r in rids])
    gate = model.scene.collision.arm_arm_gate
    hmin = model.hull_min
    tl = replay(plan, rids, home, dt=dt, margin=margin, lag=lag)
    T = float(tl.t[-1])
    ts = np.arange(0.0, T + 1e-9, 1.0 / check_hz)
    col = [list(tl.robot_ids).index(r) for r in rids]
    Q = tl.sample(ts)[:, col]
    byr = {r: sorted([l for l in tl.legs if l.robot == r], key=lambda l: l.t0) for r in rids}
    smin, hmin_seen, viol = np.inf, np.inf, {}
    worst = None
    for i in range(len(rids)):
        for j in range(i + 1, len(rids)):
            ra, rb = rids[i], rids[j]
            c = model.pair_clearance(ra, rb, Q[:, i], Q[:, j])
            bad = c <= gate
            k = int(np.argmin(c))
            if c[k] < smin:
                smin = float(c[k])
                worst = {"pair": f"{ra}|{rb}", "t_s": _r(ts[k], 2), "kind": "spheres"}
            if model.scene.collision.hull_backstop or hmin > 0:
                lb = model.hull_lower_bound(ra, rb, Q[:, i], Q[:, j])
                near = np.flatnonzero(lb < max(hmin, 0.0) + 0.02)
                if len(near):
                    d = model.mj_pair_min_distance(ra, rb, Q[near, i], Q[near, j])
                    hmin_seen = min(hmin_seen, float(d.min()))
                    bad[near[d < hmin - 1e-9]] = True
            for k in np.flatnonzero(bad):
                key = (f"{ra}|{rb}", _active_leg(byr, ra, ts[k]), _active_leg(byr, rb, ts[k]))
                if key not in viol and len(viol) < n_viol:
                    viol[key] = {"pair": key[0], "legs": {ra: key[1], rb: key[2]}, "t_s": _r(ts[k], 2),
                                 "sphere_m": _r(c[k])}
    ok = bool(smin > gate and (hmin_seen >= hmin - 1e-9))
    out = {"pass": ok, "sphere_min_m": _r(smin) if np.isfinite(smin) else None,
           "hull_min_m": _r(hmin_seen) if np.isfinite(hmin_seen) else ">= cover bound",
           "worst": worst, "violations": list(viol.values()),
           "executor_end_s": _r(float(tl.meta.get("executor_end_s", T)), 3)}
    if continuous and len(rids) > 1:
        cc = certifier(model).certify_timeline(tl, t_end=T, umarell_parity=False, minimize=False, single=False)
        rows_c = {"arm_arm_spheres": cc["rows"]["arm_arm_spheres"]["pairs"]}
        if model.scene.collision.hull_backstop:
            rows_c["arm_arm_hull"] = cc["rows"]["arm_arm_hull"]["pairs"]
        cont_ok = True
        for kind, prs in rows_c.items():
            for key, v in prs.items():
                if v["ok"]:
                    continue
                cont_ok = False
                ra, rb = key.split("|")
                for t in [x["t_s"] for x in v["violations"]] or [v["t_lower_bound_s"] or 0.0]:
                    k2 = (key, _active_leg(byr, ra, t), _active_leg(byr, rb, t))
                    if k2 not in viol and len(viol) < n_viol:
                        viol[k2] = {"pair": key, "legs": {ra: k2[1], rb: k2[2]}, "t_s": _r(t, 3),
                                    "continuous": kind}
        out["continuous"] = {"pass": bool(cont_ok),
                             "sphere_certified_ok": all(v["ok"] for v in cc["rows"]["arm_arm_spheres"]["pairs"].values()),
                             "hull_certified_ok": all(v["ok"] for v in cc["rows"]["arm_arm_hull"]["pairs"].values()),
                             "note": "certified > gate / >= hull_min between the samples (no minimisation)"}
        out["pass"] = bool(ok and cont_ok)
        out["violations"] = list(viol.values())
    return out


def lateness_check(model, plan, *, dt: float, margin: float, delta_s: float = 0.2, check_hz: float = 100.0,
                   robots=None, continuous: bool = True) -> dict:
    """Per-leg completion lateness ``delta_s`` (every leg of a robot completes delta_s late, so k x delta_s after
    its k-th leg where no gate re-synchronises): (a) every robot late (uniform; e.g. an executor completion margin
    delta_s above the planned one), (b) each robot alone late. The gated round-4 check (docs/trajgen.md 7)."""
    rids = list(model.rids if robots is None else robots)
    pats = {"all": lambda rid, k: delta_s}
    for r in rids:
        pats[f"only {r}"] = (lambda rid, k, r=r: delta_s if rid == r else 0.0)
    rows = {name: lateness_replay_check(model, plan, dt=dt, margin=margin, lag=f, check_hz=check_hz,
                                        continuous=continuous)
            for name, f in pats.items()}
    return {"delta_s": delta_s, "pass": bool(all(v["pass"] for v in rows.values())), "patterns": rows}


def lateness_tolerance(model, plan, *, dt: float, margin: float, check_hz: float = 100.0,
                       deltas=(0.05, 0.1, 0.2, 0.3, 0.5, 1.0)) -> dict:
    """Plan property: the largest per-leg completion lateness (from ``deltas``) for which the sphere gate and the
    hull buffer hold at it and every smaller value, (a) on all robots uniformly, (b) on any one robot alone. A
    plan run by an executor whose completion margin differs from planning.executor_margin_s by x is case (a) with
    delta = x (umarell's ExecConfig.completion_margin_s default is 1.0)."""
    rids = list(model.rids)
    pats = {"all": None, **{r: r for r in rids}}
    out = {}
    for name, r in pats.items():
        tol, first_fail = 0.0, None
        for d in deltas:
            f = (lambda rid, k, d=d: d) if r is None else (lambda rid, k, d=d, r=r: d if rid == r else 0.0)
            res = lateness_replay_check(model, plan, dt=dt, margin=margin, lag=f, check_hz=check_hz, n_viol=3)
            if not res["pass"]:
                first_fail = {"delta_s": d, "sphere_min_m": res["sphere_min_m"], "hull_min_m": res["hull_min_m"],
                              "violations": res["violations"][:3]}
                break
            tol = d
        out["all_robots" if r is None else f"only {r}"] = {"tolerated_s": tol if first_fail else f">= {tol}",
                                                          "first_fail": first_fail}
    vals = [v["tolerated_s"] for v in out.values()]
    num = [float(str(v).replace(">= ", "")) for v in vals]
    return {"deltas_s": list(deltas), "per_pattern": out, "min_tolerated_s": min(num) if num else None,
            "note": "largest per-leg completion lateness (scanned values) keeping the sphere gate and the hull buffer "
                    "on the replay; 'all_robots' also covers an executor completion margin that is larger than the "
                    "planned one by that amount"}


def _stroke(x, ka, kb, w, pl):
    """Straightness of the strokes, on the dense TCP path x (N, 3):
    descent: walking back in time from the arrival sample ka, how far back up the approach line (m) the TCP stays
    within the descent tube, and the max lateral distance inside the first min_offset of that window;
    lift-off: from the end of the dwell kb on, how far along -approach the TCP gets before it leaves the lift-off tube.
    A joint-space departure out of the seam (umarell's plans) leaves the tube after a few mm."""
    p, a = w.position, w.approach
    rel = x - p
    along = -(rel @ a)                                          # distance back up the line
    lat = np.linalg.norm(rel + along[:, None] * a, axis=1)

    def walk(k0, step, tube):
        k, best, worst_lat = k0, 0.0, 0.0
        while 0 <= k < len(x) and lat[k] <= tube + 1e-6:
            best = max(best, along[k])
            if along[k] <= pl.min_offset:
                worst_lat = max(worst_lat, lat[k])
            k += step
        return best, worst_lat, k

    d_in, lat_in, _ = walk(ka, -1, pl.descent_tube)
    d_out, lat_out, k_end = walk(kb, +1, pl.liftoff_tube)
    if k_end >= len(x):                                          # timeline ends on the line (never left)
        d_out = max(d_out, pl.retract_offset)
    return d_in, lat_in, lat_out, d_out


def phase_caps_check(legs, speeds: dict, dt: float) -> dict:
    """Per-phase joint speed / acceleration vs the caps the legs were planned with (generator legs only)."""
    worst_v, worst_a = 0.0, 0.0
    at_v = at_a = None
    for L in legs:
        sp = speeds[L.robot]
        # every leg starts and ends at rest and the executor plays that rest (dwell, wait, margin): pad two rest
        # samples on both sides so the acceleration out of / into the rest is checked too (round-3 finding)
        Qs = np.vstack([L.positions[:1], L.positions[:1], L.positions, L.positions[-1:], L.positions[-1:]])
        ph = np.concatenate([L.phase[:1], L.phase[:1], L.phase, L.phase[-1:], L.phase[-1:]])
        tt = np.concatenate([L.times[:1] - 2 * dt, L.times[:1] - dt, L.times, L.times[-1:] + dt, L.times[-1:] + 2 * dt])
        if len(L.positions) < 2:
            continue
        v = np.diff(Qs, axis=0) / dt
        for k in range(len(v)):
            p = ph[k + 1] if ph[k + 1] in sp.v else "transit"
            r = float((np.abs(v[k]) / sp.v[p]).max())
            if r > worst_v:
                worst_v, at_v = r, (L.robot, L.op_id, p, round(float(tt[k + 1]), 3))
        a = np.diff(v, axis=0) / dt
        for k in range(len(a)):
            p1, p2 = ph[k + 1], ph[k + 2]
            cap = np.maximum(sp.a.get(p1, sp.a["transit"]) if p1 in sp.a else sp.a["transit"],
                             sp.a.get(p2, sp.a["transit"]) if p2 in sp.a else sp.a["transit"])
            r = float((np.abs(a[k]) / cap).max())
            if r > worst_a:
                worst_a, at_a = r, (L.robot, L.op_id, f"{p1}/{p2}", round(float(tt[k + 1]), 3))
    return {"max_vel_over_phase_cap": _r(worst_v, 4), "at_v": at_v, "max_acc_over_phase_cap": _r(worst_a, 4),
            "at_a": at_a, "note": "20 Hz sample differences, each leg padded with its rest samples at both ends; "
                                  "'wait' samples use the transit caps"}
