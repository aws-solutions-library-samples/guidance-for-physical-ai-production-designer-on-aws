"""MjSpec scene of the umarell two-robot KUKA spot-welding cell.

Two KR240 R2900 + stick-tool arms (from the cell's dressed URDF) at the cell's base poses,
a floor plane, a TCP site per arm, a site per weld and an optional fixture.

Geom groups:
  0  floor
  2  visual meshes (contype/conaffinity 0, never collide)
  3  convex collision hulls of the arms (contype/conaffinity 1)
  4  fixture boxes (contact-free by default; the env checks them with the sphere model)
  5  fixture visual mesh
Names: joints ``robot1/joint_a1..a6``, bodies ``robot1/link_1..link_6`` (prefix + URDF name
``like_kr240r2900_link_k``, see ``body_name``), sites ``robot1/tcp`` and ``weld/W01..``.
"""
from __future__ import annotations

import os
import re
import warnings
from pathlib import Path

import mujoco
import numpy as np

from .cell import FIXTURE_GLB, FIXTURE_POS, ROBOT_IDS, Cell, load_cell

ASSET_DIR = Path(__file__).resolve().parent / "assets"
GEN_DIR = ASSET_DIR / "generated"

LINK_PREFIX = "like_kr240r2900_"
LINKS = ["base_link"] + [f"link_{k}" for k in range(1, 7)]
JOINTS = [f"joint_a{k}" for k in range(1, 7)]

G_FLOOR, G_VISUAL, G_HULL, G_FIXBOX, G_FIXMESH = 0, 2, 3, 4, 5

# Visual style (videos).
KUKA_ORANGE = [0.96, 0.42, 0.04, 1.0]
ROBOT_RGBA = {"robot1": KUKA_ORANGE, "robot2": [0.98, 0.55, 0.10, 1.0]}
BASE_RGBA = [0.18, 0.18, 0.2, 1.0]
TOOL_RGBA = [0.62, 0.64, 0.68, 1.0]
HULL_RGBA = [0.2, 0.6, 1.0, 0.35]


def body_name(rid: str, link: str) -> str:
    """Body name in the scene, e.g. body_name('robot1', 'link_6')."""
    return f"{rid}/{LINK_PREFIX}{link}"


def quat_xyzw_to_wxyz(q) -> np.ndarray:
    q = np.asarray(q, float)
    return np.concatenate([q[..., 3:], q[..., :3]], -1)


def patched_urdf(cell: Cell, rid: str = "robot1") -> Path:
    """Copy of the robot URDF with absolute mesh paths and the MuJoCo compiler tag.

    Written to ``env/assets/generated/`` (generated, machine specific, not for git).
    """
    src = Path(cell.robots[rid].urdf).resolve()
    out = GEN_DIR / f"{cell.cell_id}__{src.stem}.urdf"
    try:
        src.relative_to(Path(cell.root).resolve())
    except ValueError:   # a URDF from outside the cell (robot override, docs/p3_urdf.md): one file per source path
        import hashlib
        out = GEN_DIR / f"{cell.cell_id}__{src.stem}__{hashlib.sha1(str(src).encode()).hexdigest()[:10]}.urdf"
    if out.exists() and out.stat().st_mtime >= src.stat().st_mtime and str(src.parent) in out.read_text():
        return out
    txt = src.read_text()
    txt = txt.replace('filename="./', f'filename="{src.parent}/')
    # other relative mesh paths (not "./", not absolute, not package:// / file://): make them absolute as well
    txt = re.sub(r'filename="(?!/|\./|package:|file:)([^"]+)"', lambda m_: f'filename="{src.parent}/{m_.group(1)}"', txt)
    tag = ('<mujoco><compiler strippath="false" discardvisual="false" fusestatic="false" '
           'balanceinertia="true"/></mujoco>')
    txt, n = re.subn(r"(<robot\b[^>]*>)", r"\1" + tag, txt, count=1)
    assert n == 1, "no <robot> tag in URDF"
    GEN_DIR.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(f".{os.getpid()}.tmp")  # per-process name: parallel workers may race here
    tmp.write_text(txt)
    os.replace(tmp, out)
    return out


def _canonical_names(child, rc) -> None:
    """Rename the arm chain's bodies / joints of a URDF spec to the scene's canonical names
    (``like_kr240r2900_{base_link, link_1..6}``, ``joint_a1..a6``) when the URDF uses other names, so env/ can drive
    any 6-axis URDF (docs/p3_urdf.md). No-op for the KR240 URDFs."""
    from robotmodel.urdf_robot import URDFRobot
    m = URDFRobot(rc.urdf, tip_link=None)
    for b_old, b_new in zip(m.link_names, [LINK_PREFIX + n for n in LINKS]):
        if b_old != b_new:
            b = child.body(b_old)
            if b is not None:
                b.name = b_new
    for j_old, j_new in zip(m.joint_names, JOINTS):
        if j_old != j_new:
            j = child.joint(j_old)
            if j is not None:
                j.name = j_new


def fixture_obj() -> Path:
    """Fixture GLB (low-res) converted once to a single OBJ via trimesh (visual only)."""
    out = GEN_DIR / "fixture_op1000_lowres.obj"
    if out.exists():
        return out
    import trimesh

    scene = trimesh.load(str(FIXTURE_GLB), force="scene")
    mesh = scene.dump(concatenate=True) if hasattr(scene, "dump") else scene
    GEN_DIR.mkdir(parents=True, exist_ok=True)
    tmp = GEN_DIR / f"fixture_op1000_lowres.{os.getpid()}.tmp.obj"  # per-process name (parallel render workers)
    mesh.export(str(tmp))
    os.replace(tmp, out)
    return out


def camera_pose(lookat, distance, azimuth_deg, elevation_deg):
    """pos + xyaxes for a fixed camera equivalent to MjvCamera(lookat, distance, az, el)."""
    az, el = np.deg2rad(azimuth_deg), np.deg2rad(elevation_deg)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    pos = np.asarray(lookat, float) - distance * fwd
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return pos, np.concatenate([right, up])


# Named cameras: lookat, distance, azimuth, elevation (MjvCamera convention).
CAMERAS = {
    "overview": ([-0.6, -1.2, 1.0], 8.0, 60.0, -28.0),
    "front": ([-0.6, -1.2, 1.1], 7.5, 0.0, -20.0),
    "side": ([-0.6, -1.2, 1.1], 7.5, 90.0, -20.0),
    "top": ([-0.6, -1.2, 0.5], 9.0, 90.0, -88.0),
    "close": ([-0.98, -1.24, 1.35], 3.5, 45.0, -35.0),
}


def build_spec(cell: Cell | str | None = None, *, visual: bool = True, fixture: str = "none",
               weld_sites: bool = True, fixture_contacts: bool = False,
               offwidth: int = 1280, offheight: int = 720) -> mujoco.MjSpec:
    """Build the two-robot welding cell.

    visual:   keep the high-res visual meshes (group 2). False = hulls only (faster compile, no
              looks; the hulls are then shown in group 3).
    fixture:  "none" | "boxes" (all obstacle boxes of ``cell``, group 4) | "mesh" (GLB visual, group 5)
              | "both".
    fixture_contacts: give the fixture boxes contype/conaffinity 1 (default off: the env scores
              the fixture with umarell's sphere model; 700+ boxes in MuJoCo's broadphase are slow).
    """
    if cell is None or isinstance(cell, str):
        cell = load_cell(cell or "welding_cell_2_small")
    assert fixture in ("none", "boxes", "mesh", "both"), fixture

    spec = mujoco.MjSpec()
    spec.modelname = f"kuka_weld_{cell.cell_id}"
    spec.compiler.degree = False
    spec.option.timestep = 0.05
    spec.option.gravity = [0, 0, 0]
    spec.visual.global_.offwidth = offwidth
    spec.visual.global_.offheight = offheight
    spec.visual.quality.shadowsize = 4096
    spec.visual.map.znear = 0.01
    spec.visual.headlight.ambient = [0.25, 0.25, 0.25]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.stat.center = [-0.6, -1.2, 1.0]
    spec.stat.extent = 5.0

    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.78, 0.84, 0.9], rgb2=[0.22, 0.25, 0.3], width=512, height=512)
    tex = spec.add_texture(name="floor", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.42, 0.44, 0.46], rgb2=[0.36, 0.38, 0.40], mark=mujoco.mjtMark.mjMARK_EDGE,
                           markrgb=[0.5, 0.52, 0.55], width=512, height=512)
    mat = spec.add_material(name="floor", texrepeat=[12, 12], reflectance=0.0)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name

    wb = spec.worldbody
    # key light is a spot: directional-light shadow maps over this 10 m cell give floor acne
    wb.add_light(name="key", pos=[1.0, -4.0, 10.0], dir=[-0.15, 0.25, -1.0], type=mujoco.mjtLightType.mjLIGHT_SPOT,
                 cutoff=45, exponent=1, castshadow=True, diffuse=[0.65, 0.63, 0.6], specular=[0.2, 0.2, 0.2])
    wb.add_light(name="fill", pos=[-5.0, 2.0, 6.0], dir=[0.5, -0.3, -0.8], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                 castshadow=False, diffuse=[0.3, 0.32, 0.36], specular=[0, 0, 0])
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[8, 8, 0.1], pos=[-0.6, -1.2, 0],
                material="floor", group=G_FLOOR, contype=1, conaffinity=1)

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for rid in ROBOT_IDS:
            rc = cell.robots[rid]
            urdf = patched_urdf(cell, rid)          # per robot (identical file when the robots share a URDF)
            child = mujoco.MjSpec.from_file(str(urdf))
            _canonical_names(child, rc)
            # Materials live in the child spec: attach_body prefixes them ("robot1/link").
            child.add_material(name="link", rgba=ROBOT_RGBA[rid], specular=0.5, shininess=0.4)
            child.add_material(name="base", rgba=BASE_RGBA, specular=0.3, shininess=0.3)
            child.add_material(name="tool", rgba=TOOL_RGBA, specular=0.8, shininess=0.7, reflectance=0.1)
            for g in child.geoms:
                is_vis = g.contype == 0 and g.conaffinity == 0
                link = g.parent.name.replace(LINK_PREFIX, "")
                if is_vis:
                    g.group, g.contype, g.conaffinity = G_VISUAL, 0, 0
                    g.material = "tool" if g.meshname.startswith("tool_") else ("base" if link == "base_link" else "link")
                else:
                    g.group, g.contype, g.conaffinity = G_HULL, 1, 1
                    g.rgba = HULL_RGBA
                    g.margin = 0.0
            if not visual:
                for g in [g for g in child.geoms if g.group == G_VISUAL]:
                    child.delete(g)
            for m in list(child.meshes):  # drop meshes no geom references (visuals when visual=False)
                if not any(g.meshname == m.name for g in child.geoms):
                    child.delete(m)
            yaw = rc.yaw
            fr = wb.add_frame(pos=rc.base.tolist(), quat=[np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
            fr.attach_body(child.body(LINK_PREFIX + "base_link"), rid + "/", "")
            # SRDF: adjacent links never collide; base_link is welded to the world so MuJoCo's
            # parent filter does not cover base_link <-> link_1.
            for a, b in zip(LINKS[:-1], LINKS[1:]):
                spec.add_exclude(bodyname1=body_name(rid, a), bodyname2=body_name(rid, b))
            l6 = spec.body(body_name(rid, "link_6"))
            l6.add_site(name=f"{rid}/tcp", pos=rc.tcp_offset.tolist(), size=[0.015, 0, 0],
                        rgba=[0.1, 0.9, 0.2, 0.9], group=1)

    if weld_sites:
        for w in cell.welds:
            wb.add_site(name=f"weld/{w.wid}", pos=w.pos.tolist(), quat=quat_xyzw_to_wxyz(w.quat_xyzw).tolist(),
                        size=[0.022, 0, 0], rgba=[0.95, 0.1, 0.1, 0.9], group=1)

    if fixture in ("boxes", "both"):
        c, h = cell.boxes
        ct = 1 if fixture_contacts else 0
        for k, (ck, hk) in enumerate(zip(c, h)):
            wb.add_geom(name=f"fixbox{k}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=ck.tolist(), size=hk.tolist(),
                        rgba=[0.55, 0.65, 0.75, 0.35], group=G_FIXBOX, contype=ct, conaffinity=ct)
    if fixture in ("mesh", "both"):
        spec.add_mesh(name="fixture", file=str(fixture_obj()))
        wb.add_geom(name="fixture", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="fixture", pos=FIXTURE_POS.tolist(),
                    rgba=[0.62, 0.64, 0.7, 1.0], group=G_FIXMESH, contype=0, conaffinity=0)

    for name, (lookat, dist, az, el) in CAMERAS.items():
        pos, xy = camera_pose(lookat, dist, az, el)
        wb.add_camera(name=name, pos=pos.tolist(), xyaxes=xy.tolist(), fovy=45)
    return spec


def build_model(cell: Cell | str | None = None, **kw) -> mujoco.MjModel:
    return build_spec(cell, **kw).compile()


def scene_option(show_hulls: bool = False, show_fixture_boxes: bool = True, show_sites: bool = True) -> mujoco.MjvOption:
    """Visual option: which geom groups to draw. Default: floor, visuals, fixture mesh/boxes, sites."""
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    for g in (G_FLOOR, G_VISUAL, G_FIXMESH):
        opt.geomgroup[g] = 1
    opt.geomgroup[G_HULL] = int(show_hulls)
    opt.geomgroup[G_FIXBOX] = int(show_fixture_boxes)
    opt.sitegroup[:] = 0
    opt.sitegroup[1] = int(show_sites)
    return opt


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", default="welding_cell_2_small")
    ap.add_argument("--fixture", default="none")
    ap.add_argument("--out", default=str(GEN_DIR / "scene.xml"))
    a = ap.parse_args()
    s = build_spec(a.cell, fixture=a.fixture)
    m = s.compile()
    Path(a.out).write_text(s.to_xml())
    print("wrote", a.out, "ngeom", m.ngeom, "nq", m.nq)
