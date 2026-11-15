"""MuJoCo hull models of a scene, for any number of robots (no dependency on env/scene.py's robot ids).

Every robot is loaded from its own URDF (convex <collision> hulls only, visuals dropped) at its scene base pose
(full rotation). Three kinds of models are built lazily:

  * ``env[rid]``   one robot + floor plane + obstacle boxes; only robot-vs-world contacts (planning: hull-vs-box);
  * ``single[rid]`` one robot + floor + boxes; self contacts (disabled pairs excluded) and robot-vs-world contacts
    (verification);
  * ``pair[(a, b)]`` two robots, only contacts between the two arms (planning and verification).

Workpiece boxes (``is_workpiece: true``) are not hull geoms by default: the electrode must touch the workpiece, and
the sphere box rule (TCP and electrode spheres exempt) covers them. With ``obstacles.box_hull.workpiece: true``
(docs/n_contract.md 1.5, default off) they are hull geoms too, in their own contact bit, which every robot hull has
except the electrode hull(s) (``tool.electrode_hulls``, default ``electrode_hull_names``: the tip-link collision hull
that contains the TCP), so every other hull keeps the box-hull distance to the workpiece. Contact filtering uses contype/conaffinity bits, so no work is spent on pairs a model does not check. Box geoms carry
``margin = detect`` so hull-box distances below ``detect`` are reported with their signed distance.

Electrode off-weld rule (``obstacles.box_hull.electrode_off_weld``, docs/n2_contract.md 3.1, default off): a fourth
kind of model, ``elec[rid]``, holds only the robot's electrode hull(s) and the workpiece boxes (contact bit
ELEC_BIT); ``electrode_distance`` gives their signed distance within the band ``elec_band`` (min_distance + 5 cm).
The other models are unchanged by the rule.
"""
from __future__ import annotations

import hashlib
import os
import re
import warnings
from pathlib import Path

import numpy as np

from trajgen.cachedir import CACHE_DIR
from trajgen.robot import parse_chain
from trajgen.scene import R_to_quat, Scene

ROOT = Path(__file__).resolve().parents[1]
GEN = CACHE_DIR / "urdf"
WP_BIT = 4      # contact bit of workpiece boxes (obstacles.box_hull.workpiece); robots use 1 and 2
ELEC_BIT = 8    # contact bit of the electrode-only models (electrode off-weld rule)


def patched_urdf(urdf: Path) -> Path:
    """Copy of the URDF with absolute mesh paths and MuJoCo's compiler tag (cached by content hash)."""
    src = Path(urdf).resolve()
    txt = src.read_text()
    h = hashlib.sha256((str(src) + txt).encode()).hexdigest()[:12]
    out = GEN / f"{src.stem}__{h}.urdf"
    if out.exists():
        return out

    def absmesh(m):
        fn = m.group(1)
        if fn.startswith(("package://", "file://")):
            raise ValueError(f"{src}: mesh URI {fn} not supported (use paths relative to the URDF)")
        p = Path(fn)
        return f'filename="{p if p.is_absolute() else (src.parent / p).resolve()}"'

    txt = re.sub(r'filename="([^"]+)"', absmesh, txt)
    tag = ('<mujoco><compiler strippath="false" discardvisual="true" fusestatic="false" '
           'balanceinertia="true"/></mujoco>')
    txt, n = re.subn(r"(<robot\b[^>]*>)", r"\1" + tag, txt, count=1)
    if n != 1:
        raise ValueError(f"{src}: no <robot> tag")
    GEN.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(f".{os.getpid()}.tmp")
    tmp.write_text(txt)
    os.replace(tmp, out)
    return out


def electrode_hull_names(robot) -> tuple:
    """Collision-mesh names (file stems, = MuJoCo mesh names) of the robot's electrode hull(s): ``tool.electrode_hulls``
    if given, else the collision mesh of ``tool.tip_link`` that contains the TCP (the nearest one within 2 cm when
    the TCP lies just outside every hull, as the stick tool's tip at 0.2 mm). Empty if none qualifies."""
    if getattr(robot.tool, "electrode_hulls", None):
        return tuple(robot.tool.electrode_hulls)
    key = (str(Path(robot.urdf).resolve()), robot.tool.tip_link, tuple(np.round(robot.tool.tcp_offset, 9)))
    if key in _ELECTRODE_CACHE:
        return _ELECTRODE_CACHE[key]
    import xml.etree.ElementTree as ET

    import trimesh
    from scipy.spatial import Delaunay

    from trajgen.robot import rpy_matrix
    urdf = Path(robot.urdf)
    root = ET.parse(urdf).getroot()
    tcp = np.asarray(robot.tool.tcp_offset, float)
    cands = []
    for link in root.findall("link"):
        if link.get("name") != robot.tool.tip_link:
            continue
        for c in link.findall("collision"):
            g = c.find("geometry")
            mesh = g.find("mesh") if g is not None else None
            if mesh is None:
                continue
            o = c.find("origin")
            xyz = np.array([float(x) for x in ((o.get("xyz") if o is not None else None) or "0 0 0").split()])
            rpy = np.array([float(x) for x in ((o.get("rpy") if o is not None else None) or "0 0 0").split()])
            fn = mesh.get("filename")
            m = trimesh.load(str(urdf.parent / fn), force="mesh", process=False)
            V = np.asarray(m.vertices, float) * np.array([float(x) for x in (mesh.get("scale") or "1 1 1").split()])
            V = V @ rpy_matrix(*rpy).T + xyz
            try:
                inside = bool(Delaunay(V).find_simplex(tcp[None])[0] >= 0)
            except Exception:  # noqa: BLE001  (degenerate hull)
                inside = False
            cands.append((Path(fn).stem, inside, float(np.linalg.norm(V - tcp, axis=1).min())))
    ins = tuple(n for n, i, _ in cands if i)
    if not ins and cands:
        n, _, d = min(cands, key=lambda x: x[2])
        ins = (n,) if d < 0.02 else ()
    _ELECTRODE_CACHE[key] = ins
    return ins


_ELECTRODE_CACHE: dict = {}


def electrode_hull_vertices(robot) -> list:
    """[(link name, mesh name, (n, 3) vertices in the link frame)] of the robot's electrode hull(s)
    (``electrode_hull_names``), from its URDF <collision> meshes (origin and scale applied)."""
    import xml.etree.ElementTree as ET

    import trimesh

    from trajgen.robot import rpy_matrix
    names = set(electrode_hull_names(robot))
    urdf = Path(robot.urdf)
    out = []
    for link in ET.parse(urdf).getroot().findall("link"):
        for c in link.findall("collision"):
            g = c.find("geometry")
            mesh = g.find("mesh") if g is not None else None
            if mesh is None or Path(mesh.get("filename")).stem not in names:
                continue
            o = c.find("origin")
            xyz = np.array([float(x) for x in ((o.get("xyz") if o is not None else None) or "0 0 0").split()])
            rpy = np.array([float(x) for x in ((o.get("rpy") if o is not None else None) or "0 0 0").split()])
            m = trimesh.load(str(urdf.parent / mesh.get("filename")), force="mesh", process=False)
            V = np.asarray(m.vertices, float) * np.array([float(x) for x in (mesh.get("scale") or "1 1 1").split()])
            out.append((link.get("name"), Path(mesh.get("filename")).stem, V @ rpy_matrix(*rpy).T + xyz))
    return out


class MjWorld:
    def __init__(self, scene: Scene, detect: float = 0.0):
        import mujoco

        self.mujoco = mujoco
        self.scene = scene
        self.rids = scene.robot_ids
        self.detect = float(max(detect, scene.box_hull.min_distance if scene.box_hull.enabled else 0.0, 0.0)) + 1e-4
        # pair models: contacts are reported up to ``pair_band`` of separation (signed distance in contact.dist), so
        # one mj_collision gives the robot-robot hull distance wherever it is below the band
        self.hull_min = float(max(scene.collision.hull_min_distance, 0.0))
        self.pair_band = self.hull_min + 0.05
        er = getattr(scene.box_hull, "electrode", None)
        self.elec_band = float(max(er.min_distance, 0.0) if er is not None else 0.0) + 0.05
        self._env, self._single, self._pair, self._elec = {}, {}, {}, {}
        self.fromto = np.zeros(6)

    # ------------------------------------------------------------------ building
    def _spec(self, robots, mode: str, with_world: bool):
        """mode: 'env' (robot vs world only), 'single' (self + world), 'pair' (arm vs arm only)."""
        mujoco = self.mujoco
        sc = self.scene
        spec = mujoco.MjSpec()
        spec.modelname = f"trajgen_{mode}"
        spec.compiler.degree = False
        spec.option.gravity = [0, 0, 0]
        wb = spec.worldbody
        ct_world, ca_world = 0, 1
        wp_hull = bool(with_world and sc.box_hull.workpiece)   # workpiece boxes as hull geoms (bit WP_BIT)
        if with_world:
            if sc.floor.enabled:
                wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[50, 50, 0.1], pos=[0, 0, sc.floor.z],
                            contype=ct_world, conaffinity=ca_world)
            for k, b in enumerate(sc.boxes):
                if b.is_workpiece and not wp_hull:   # workpiece boxes: sphere rule only (TCP / electrode exempt)
                    continue
                q = [1, 0, 0, 0] if b.R is None else np.roll(R_to_quat(b.R), 1).tolist()
                wb.add_geom(name=f"box{k}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=b.center.tolist(), size=b.half_extents.tolist(),
                            quat=q, contype=ct_world, conaffinity=WP_BIT if b.is_workpiece else ca_world,
                            margin=self.detect)
        for i, r in enumerate(robots):
            ch = parse_chain(str(r.urdf), r.tool.tip_link)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                child = mujoco.MjSpec.from_file(str(patched_urdf(r.urdf)))
            if mode == "env":
                ct, ca = 1, 0
            elif mode == "single":
                ct, ca = 1, 1
            else:
                ct, ca = (1, 2) if i == 0 else (2, 1)
            elec = set(electrode_hull_names(r)) if wp_hull else set()
            if wp_hull:
                known = {g.meshname for g in child.geoms}
                if not elec <= known:
                    raise ValueError(f"{r.id}: tool.electrode_hulls {sorted(elec - known)} not among the URDF collision "
                                     f"meshes {sorted(x for x in known if x)}")
            for g in list(child.geoms):
                if g.contype == 0 and g.conaffinity == 0:
                    child.delete(g)
                    continue
                g.contype, g.conaffinity = ct, ca
                if wp_hull and g.meshname not in elec:
                    g.contype = ct | WP_BIT          # every hull except the electrode touches workpiece boxes
                g.margin = self.pair_band if mode == "pair" else 0.0
            for m in list(child.meshes):
                if not any(g.meshname == m.name for g in child.geoms):
                    child.delete(m)
            fr = wb.add_frame(pos=np.asarray(r.base_xyz, float).tolist(), quat=np.roll(R_to_quat(r.base_R), 1).tolist())
            fr.attach_body(child.body(ch["base_link"]), r.id + "/", "")
            if mode == "single":
                links = ch["links"]
                for a, b in zip(links[:-1], links[1:]):
                    spec.add_exclude(bodyname1=f"{r.id}/{a}", bodyname2=f"{r.id}/{b}")
                for a, b in sc.disabled_pairs_of(r.id):
                    try:
                        spec.add_exclude(bodyname1=f"{r.id}/{a}", bodyname2=f"{r.id}/{b}")
                    except Exception:  # noqa: BLE001  (fixed links fused into a body)
                        pass
        return spec

    def _spec_electrode(self, r):
        """One robot's electrode hull(s) (contype ELEC_BIT) and the workpiece boxes (conaffinity ELEC_BIT, margin
        ``elec_band``); every other geom of the robot is removed."""
        mujoco = self.mujoco
        sc = self.scene
        spec = mujoco.MjSpec()
        spec.modelname = "trajgen_electrode"
        spec.compiler.degree = False
        spec.option.gravity = [0, 0, 0]
        wb = spec.worldbody
        for k, b in enumerate(sc.boxes):
            if not b.is_workpiece:
                continue
            q = [1, 0, 0, 0] if b.R is None else np.roll(R_to_quat(b.R), 1).tolist()
            wb.add_geom(name=f"box{k}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=b.center.tolist(),
                        size=b.half_extents.tolist(), quat=q, contype=0, conaffinity=ELEC_BIT, margin=self.elec_band)
        ch = parse_chain(str(r.urdf), r.tool.tip_link)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            child = mujoco.MjSpec.from_file(str(patched_urdf(r.urdf)))
        elec = set(electrode_hull_names(r))
        known = {g.meshname for g in child.geoms}
        if not elec or not elec <= known:
            raise ValueError(f"{r.id}: electrode off-weld rule: electrode hull(s) {sorted(elec)} not among the URDF "
                             f"collision meshes {sorted(x for x in known if x)} (set tool.electrode_hulls)")
        for g in list(child.geoms):
            if g.meshname not in elec or (g.contype == 0 and g.conaffinity == 0):
                child.delete(g)
                continue
            g.contype, g.conaffinity, g.margin = ELEC_BIT, 0, 0.0
        for m in list(child.meshes):
            if not any(g.meshname == m.name for g in child.geoms):
                child.delete(m)
        fr = wb.add_frame(pos=np.asarray(r.base_xyz, float).tolist(), quat=np.roll(R_to_quat(r.base_R), 1).tolist())
        fr.attach_body(child.body(ch["base_link"]), r.id + "/", "")
        return spec

    def _compile(self, robots, mode, with_world, spec=None):
        mujoco = self.mujoco
        m = (self._spec(robots, mode, with_world) if spec is None else spec).compile()
        d = mujoco.MjData(m)
        qadr = []
        for r in robots:
            ch = parse_chain(str(r.urdf), r.tool.tip_link)
            qadr.append(np.array([m.jnt_qposadr[m.joint(f"{r.id}/{j['name']}").id] for j in ch["joints"]]))
        owner = np.full(m.ngeom, -1)
        link = [""] * m.ngeom
        for gi in range(m.ngeom):
            nm = m.geom(gi).name
            b = m.body(m.geom_bodyid[gi]).name
            for i, r in enumerate(robots):
                if b.startswith(r.id + "/"):
                    owner[gi] = i
                    link[gi] = b[len(r.id) + 1:]
            if nm == "floor":
                owner[gi] = -2
            elif nm.startswith("box"):
                owner[gi] = -3
        arm_g = [np.flatnonzero(owner == i) for i in range(len(robots))]
        return {"m": m, "d": d, "qadr": qadr, "owner": owner, "link": link, "arm_g": arm_g}

    def env(self, rid):
        if rid not in self._env:
            self._env[rid] = self._compile([self.scene.robot(rid)], "env", True)
        return self._env[rid]

    def single(self, rid):
        if rid not in self._single:
            self._single[rid] = self._compile([self.scene.robot(rid)], "single", True)
        return self._single[rid]

    def elec(self, rid):
        if rid not in self._elec:
            r = self.scene.robot(rid)
            self._elec[rid] = self._compile([r], "electrode", True, spec=self._spec_electrode(r))
        return self._elec[rid]

    def pair(self, a, b):
        k = (a, b) if self.rids.index(a) < self.rids.index(b) else (b, a)
        if k not in self._pair:
            self._pair[k] = self._compile([self.scene.robot(k[0]), self.scene.robot(k[1])], "pair", False)
        return self._pair[k], k != (a, b)

    # ------------------------------------------------------------------ queries
    def _run(self, M, qs):
        m, d = M["m"], M["d"]
        for adr, q in zip(M["qadr"], qs):
            d.qpos[adr] = q
        self.mujoco.mj_kinematics(m, d)
        self.mujoco.mj_collision(m, d)
        return d.ncon

    def world_distance(self, rid, Q) -> np.ndarray:
        """(N,) min signed hull distance robot vs boxes (capped at ``detect``) and robot vs floor plane; +detect if
        nothing is within ``detect``. Only contacts against boxes / floor are considered."""
        M = self.env(rid)
        Q = np.asarray(Q, float).reshape(-1, 6)
        out = np.full(len(Q), self.detect)
        d = M["d"]
        for i in range(len(Q)):
            n = self._run(M, [Q[i]])
            if n:
                out[i] = min(self.detect, float(d.contact.dist[:n].min()))
        return out

    def box_distance(self, rid, Q) -> np.ndarray:
        """(N,) min signed hull-vs-box distance (capped at ``detect``)."""
        M = self.env(rid)
        Q = np.asarray(Q, float).reshape(-1, 6)
        out = np.full(len(Q), self.detect)
        d, own = M["d"], M["owner"]
        if not self.scene.boxes:
            return np.full(len(Q), np.inf)
        for i in range(len(Q)):
            n = self._run(M, [Q[i]])
            if n:
                g = d.contact.geom[:n]
                isbox = (own[g[:, 0]] == -3) | (own[g[:, 1]] == -3)
                if isbox.any():
                    out[i] = min(self.detect, float(d.contact.dist[:n][isbox].min()))
        return out

    def electrode_distance(self, rid, Q):
        """(d (N,), box (N,) int) min signed distance between the robot's electrode hull(s) and the workpiece boxes,
        capped at ``elec_band``, and the index (into scene.boxes) of the box at that distance (-1: none within the
        band). The electrode off-weld rule's exact test (SceneModel.electrode_margin screens with bounding spheres)."""
        Q = np.asarray(Q, float).reshape(-1, 6)
        out = np.full(len(Q), self.elec_band)
        box = np.full(len(Q), -1, int)
        if not any(b.is_workpiece for b in self.scene.boxes):
            return out, box
        M = self.elec(rid)
        m, d = M["m"], M["d"]
        bidx = {}
        for i in range(len(Q)):
            n = self._run(M, [Q[i]])
            if not n:
                continue
            dist = d.contact.dist[:n]
            k = int(np.argmin(dist))
            if dist[k] < out[i]:
                out[i] = float(dist[k])
                g1, g2 = int(d.contact.geom[k, 0]), int(d.contact.geom[k, 1])
                g = g1 if M["owner"][g1] == -3 else g2
                if g not in bidx:
                    bidx[g] = int(m.geom(g).name[3:])
                box[i] = bidx[g]
        return out, box

    def electrode_contacts(self, rid, q):
        """Every contact of the robot's electrode hull(s) with the workpiece boxes at one configuration (within
        ``elec_band``): (dist (n,), pos (n, 3) world contact positions, box (n,) indices into scene.boxes). The
        no-dip rule's exemption test (trajgen.nodip: contact position and depth per contact)."""
        if not any(b.is_workpiece for b in self.scene.boxes):
            return np.zeros(0), np.zeros((0, 3)), np.zeros(0, int)
        M = self.elec(rid)
        m, d = M["m"], M["d"]
        n = self._run(M, [np.asarray(q, float).reshape(6)])
        if not n:
            return np.zeros(0), np.zeros((0, 3)), np.zeros(0, int)
        dist = np.array(d.contact.dist[:n], float)
        pos = np.array(d.contact.pos[:n], float)
        g = d.contact.geom[:n]
        gb = np.where(M["owner"][g[:, 0]] == -3, g[:, 0], g[:, 1])
        box = np.array([int(m.geom(int(x)).name[3:]) for x in gb], int)
        return dist, pos, box

    def single_contacts(self, rid, q) -> dict:
        """Penetrating contacts of one robot: {"self": [...], "floor": [...], "box": [...]} as (geom1 body, geom2
        body, dist)."""
        M = self.single(rid)
        n = self._run(M, [q])
        out = {"self": [], "floor": [], "box": []}
        if not n:
            return out
        m, d, own, link = M["m"], M["d"], M["owner"], M["link"]
        for k in range(n):
            con = d.contact[k]
            a, b = own[con.geom1], own[con.geom2]
            nm = (link[con.geom1] or self._gname(m, con.geom1), link[con.geom2] or self._gname(m, con.geom2), float(con.dist))
            if a >= 0 and b >= 0:
                if con.dist < 0:
                    out["self"].append(nm)
            elif -2 in (a, b):
                if con.dist < 0:
                    out["floor"].append(nm)
            elif -3 in (a, b):
                if con.dist < self.scene.box_hull.min_distance:
                    out["box"].append(nm)
        return out

    def _gname(self, m, g):
        nm = m.geom(g).name
        if nm.startswith("box"):
            return self.scene.boxes[int(nm[3:])].id
        return nm

    def pair_min_distance(self, a, b, Qa, Qb) -> np.ndarray:
        """(N,) min signed hull distance between robots a and b, capped at ``pair_band`` (exact below the band)."""
        M, swap = self.pair(a, b)
        Qa, Qb = np.asarray(Qa, float).reshape(-1, 6), np.asarray(Qb, float).reshape(-1, 6)
        if swap:
            Qa, Qb = Qb, Qa
        out = np.full(len(Qa), self.pair_band)
        d = M["d"]
        for i in range(len(Qa)):
            n = self._run(M, [Qa[i], Qb[i]])
            if n:
                out[i] = min(self.pair_band, float(d.contact.dist[:n].min()))
        return out

    def pair_contact(self, a, b, Qa, Qb, thr: float = 0.0) -> np.ndarray:
        """(N,) bool: hull distance between robots a and b below ``thr`` (0: penetrating contact; thr <= pair_band)."""
        if thr > self.pair_band:
            raise ValueError(f"pair_contact threshold {thr} above the detection band {self.pair_band}")
        return self.pair_min_distance(a, b, Qa, Qb) < thr

    def pair_contacts_detail(self, a, b, qa, qb) -> list:
        M, swap = self.pair(a, b)
        qs = [qb, qa] if swap else [qa, qb]
        n = self._run(M, qs)
        d, link = M["d"], M["link"]
        return [(link[d.contact[k].geom1], link[d.contact[k].geom2], float(d.contact[k].dist)) for k in range(n)
                if d.contact[k].dist < 0]

    def pair_distance(self, a, b, qa, qb, distmax: float = 0.5) -> float:
        """Signed min hull distance between robots a and b (capped at distmax)."""
        M, swap = self.pair(a, b)
        qs = [qb, qa] if swap else [qa, qb]
        m, d = M["m"], M["d"]
        for adr, q in zip(M["qadr"], qs):
            d.qpos[adr] = q
        self.mujoco.mj_kinematics(m, d)
        best = distmax
        for ga in M["arm_g"][0]:
            for gb in M["arm_g"][1]:
                best = min(best, self.mujoco.mj_geomDistance(m, d, int(ga), int(gb), distmax, self.fromto))
        return float(best)
