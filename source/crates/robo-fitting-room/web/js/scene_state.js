// Single in-memory scene mirror. The fields the editor works with mirror
// the Rust SceneYaml types in src/scene.rs. Every other key of the
// manifest (a robot's `pedestal` or `payload`, the floor plan's
// `footprint_layers`, keys added later) rides along untouched, after the
// editor's own fields, so save writes back everything load read.

/** The keys the editor works with, per manifest mapping. */
export const SCENE_KEYS = ["name", "description", "scene_origin", "robots", "fixtures", "floor_plan"];
export const ROBOT_KEYS = ["robot_id", "dressed_urdf", "position", "orientation", "home_pose_deg"];
export const FIXTURE_KEYS = ["id", "urdf", "position", "orientation"];
export const FLOOR_PLAN_KEYS = ["geometry_file", "offset", "cell_origin_dxf", "scale"];

/** The entries of `obj` (a parsed YAML mapping) whose keys are not in
 *  `known`, in their order: the keys the editor carries along. */
export function extraKeys(obj, known) {
  const out = {};
  if (obj && typeof obj === "object" && !Array.isArray(obj)) {
    for (const [k, v] of Object.entries(obj)) {
      if (!known.includes(k)) out[k] = v;
    }
  }
  return out;
}

export function blankScene() {
  return {
    name: "Untitled",
    description: "",
    scene_origin: null,           // {x,y,z} | null
    robots: [],                    // {robot_id, dressed_urdf, position, orientation, home_pose_deg?, …other keys}
    fixtures: [],                  // {id, urdf, position, orientation, …other keys}
    floor_plan: null,              // {geometry_file, offset?, cell_origin_dxf?, scale, …other keys}
  };
}

export function defaultPosition() {
  return { x: 0, y: 0, z: 0 };
}
export function defaultOrientation() {
  return { a: 0, b: 0, c: 0 };
}

/** The home pose compose falls back to for `jointCount` actuated joints:
 *  the KUKA stowed default by index (degrees) for the first 6, 0 beyond;
 *  0 m for every joint whose `kinds[i]` is "prismatic" (the angles turn
 *  revolute and continuous joints only), as `drives::actuated_home_values`
 *  does. */
export function defaultHomePoseDeg(jointCount, kinds = []) {
  const stowed = [0, -90, 90, 0, 0, 0];
  const out = new Array(jointCount).fill(0);
  for (let i = 0; i < Math.min(jointCount, stowed.length); i++) {
    out[i] = kinds[i] === "prismatic" ? 0 : stowed[i];
  }
  return out;
}

/** `pose` fitted to `jointCount` joints: its own values where it has
 *  them, the default (`defaultHomePoseDeg`, by `kinds`) for the joints it
 *  lacks, extra entries dropped. */
export function fitHomePoseDeg(pose, jointCount, kinds = []) {
  const fallback = defaultHomePoseDeg(jointCount, kinds);
  const own = Array.isArray(pose) ? pose : [];
  return fallback.map((d, i) => (Number.isFinite(own[i]) ? own[i] : d));
}

/** Deep clone. `structuredClone` keeps what JSON can't (`.inf`, `.nan`
 *  in a key the editor carries along); JSON is the fallback. */
export function cloneScene(s) {
  if (typeof structuredClone === "function") return structuredClone(s);
  return JSON.parse(JSON.stringify(s));
}

/** Serialise the in-memory scene back to a YAML-shaped object for save.
 *  - Keys the editor doesn't work with are emitted as they were loaded.
 *  - Paths are emitted as authored (no `..` collapsing).
 *  - Position / orientation / scene_origin / floor_plan are emitted
 *    as plain object literals; the dump call uses flowLevel to render
 *    them on a single line per the project's manifest convention.
 *  - A robot without a recorded home_pose_deg gets the default compose
 *    would use, sized to its URDF's actuated joints: `jointsOf(robot)`
 *    returns the joints the inspector fetched for it (`[{kind}, …]`), or
 *    nothing when they aren't known — then the key is left out, so compose
 *    falls back to the same default itself (never a pose of the wrong
 *    length, which validation would reject). */
export function toYamlObject(s, jointsOf = () => undefined) {
  const obj = cloneScene(s);
  (obj.robots || []).forEach((r, i) => {
    if (!Array.isArray(r.home_pose_deg) || r.home_pose_deg.length === 0) {
      const joints = jointsOf((s.robots || [])[i]);
      if (Array.isArray(joints) && joints.length > 0) {
        r.home_pose_deg = defaultHomePoseDeg(joints.length, joints.map((j) => j.kind));
      } else {
        delete r.home_pose_deg;
      }
    }
    r.dressed_urdf = manifestPath(r.dressed_urdf);
  });
  for (const f of obj.fixtures || []) {
    f.urdf = manifestPath(f.urdf);
  }
  if (!obj.scene_origin) delete obj.scene_origin;
  if (!obj.floor_plan) delete obj.floor_plan;
  if (obj.floor_plan) {
    obj.floor_plan.geometry_file = manifestPath(obj.floor_plan.geometry_file);
    if (!obj.floor_plan.offset) delete obj.floor_plan.offset;
    if (!obj.floor_plan.cell_origin_dxf) delete obj.floor_plan.cell_origin_dxf;
  }
  return obj;
}

/** Convert an as-authored manifest path into a path the server's
 *  asset_root-rooted REST API can consume. The manifest sits one level
 *  below asset_root (`assets/fitted-robots/scene.yaml`) and references
 *  siblings via `../foo`; collapsing the leading `..` makes the path
 *  asset-root-relative without touching state. State keeps the
 *  original verbatim so save round-trips losslessly.
 */
export function apiPath(rel) {
  if (!rel || typeof rel !== "string") return rel;
  if (rel.startsWith("/")) return rel;
  const parts = rel.split("/").filter((p) => p !== "" && p !== ".");
  const out = [];
  for (const p of parts) {
    if (p === "..") {
      if (out.length) out.pop();
    } else {
      out.push(p);
    }
  }
  return out.join("/");
}

/** Hydrate a parsed YAML object into the editor's canonical shape: the
 *  editor's fields first, then every other key of each mapping as it was
 *  (see `extraKeys`). The input is not modified. */
export function fromYamlObject(parsed) {
  const obj = parsed && typeof parsed === "object" && !Array.isArray(parsed) ? cloneScene(parsed) : {};
  const s = { ...blankScene(), ...extraKeys(obj, SCENE_KEYS) };
  s.name = obj.name || "Untitled";
  s.description = obj.description || "";
  s.scene_origin = obj.scene_origin || null;

  // The editor renders robots, fixtures, and the floor plan at their
  // RAW manifest world coordinates — no scene_origin subtraction.
  // The manifest's `scene_origin` is preserved in state and emitted
  // verbatim on save; compose still subtracts it on its own.
  // This keeps the layout aligned with the floor plan as authored
  // without forcing anything to (0,0).
  // Paths in state are stored asset-root-relative (matching the
  // palette convention). On save we add the `../` prefix back so the
  // saved YAML — which lives at `fitted-robots/<name>.yaml` — keeps
  // working with compose's `parse_scene`, which resolves relative to
  // the manifest's own directory.
  s.robots = (obj.robots || []).map((r) => ({
    robot_id: r.robot_id,
    dressed_urdf: apiPath(r.dressed_urdf),
    position: { ...defaultPosition(), ...(r.position || {}) },
    orientation: { ...defaultOrientation(), ...(r.orientation || {}) },
    home_pose_deg: Array.isArray(r.home_pose_deg) ? r.home_pose_deg.slice() : null,
    ...extraKeys(r, ROBOT_KEYS),
  }));
  s.fixtures = (obj.fixtures || []).map((f) => ({
    id: f.id,
    urdf: apiPath(f.urdf),
    position: { ...defaultPosition(), ...(f.position || {}) },
    orientation: { ...defaultOrientation(), ...(f.orientation || {}) },
    ...extraKeys(f, FIXTURE_KEYS),
  }));
  if (obj.floor_plan) {
    s.floor_plan = {
      geometry_file: apiPath(obj.floor_plan.geometry_file),
      offset: obj.floor_plan.offset || null,
      cell_origin_dxf: obj.floor_plan.cell_origin_dxf || null,
      scale: obj.floor_plan.scale || 1.0,
      ...extraKeys(obj.floor_plan, FLOOR_PLAN_KEYS),
    };
  }
  return s;
}

/** Convert an asset-root-relative path back to manifest-relative form
 *  (`../foo`) for serialisation. The manifest lives at
 *  `<asset_root>/fitted-robots/<name>.yaml`, so any asset-root-relative
 *  path needs `../` prepended to be reachable from the manifest dir.
 *  Absolute paths and already-prefixed paths are left alone. */
export function manifestPath(rel) {
  if (!rel || typeof rel !== "string") return rel;
  if (rel.startsWith("/") || rel.startsWith("../")) return rel;
  return "../" + rel;
}
