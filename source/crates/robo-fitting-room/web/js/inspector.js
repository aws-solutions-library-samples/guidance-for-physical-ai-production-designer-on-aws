// Form-based inspector. No sliders — every numeric field is a precise
// `<input type="number">` with explicit step + units. Editing any field
// mutates the in-memory scene and triggers a canvas re-render.

import { urdfJoints } from "./api.js";
import { defaultHomePoseDeg, fitHomePoseDeg, apiPath } from "./scene_state.js";

export class Inspector {
  constructor(rootEl, onChange) {
    this.root = rootEl;
    this.empty = rootEl.querySelector("#inspector-empty");
    this.form = rootEl.querySelector("#inspector-form");
    this.onChange = onChange;
    this.jointCache = new Map(); // urdf path -> joints[]
  }

  showEmpty(message) {
    this.form.hidden = true;
    this.empty.hidden = false;
    if (message) this.empty.textContent = message;
  }

  async showRobot(state, index) {
    const r = state.robots[index];
    if (!r) { this.showEmpty(); return; }
    this.empty.hidden = true;
    this.form.hidden = false;

    const jointInfo = await this._fetchJoints(r.dressed_urdf).catch((e) => {
      console.warn("urdf joints failed:", e.message);
      return { joints: [] };
    });
    const jointCount = jointInfo.joints.length;
    // A prismatic joint's home value is metres, the others' degrees.
    const kinds = jointInfo.joints.map((j) => j.kind);

    // A robot without a recorded pose gets the default the inspector
    // renders, so the exported YAML matches what's on screen even if the
    // user never edits a field. A recorded pose is never replaced here,
    // not even when its length differs from the URDF's joints (or they
    // didn't load): the form says so, validation reports it, and editing
    // a joint fits the pose to the URDF.
    const recorded = Array.isArray(r.home_pose_deg) && r.home_pose_deg.length > 0;
    if (!recorded && jointCount > 0) {
      r.home_pose_deg = defaultHomePoseDeg(jointCount, kinds);
    }
    const mismatch = recorded && jointCount > 0 && r.home_pose_deg.length !== jointCount;
    // What the form shows, and what editing a joint starts from.
    const homePose = fitHomePoseDeg(r.home_pose_deg, jointCount, kinds);

    const html = `
      <div class="form-group">
        <h4>Robot</h4>
        <div class="row"><label>id</label><input type="text" data-field="robot_id" value="${attr(r.robot_id)}"/></div>
        <div class="row"><label>urdf</label><input type="text" data-field="dressed_urdf" value="${attr(r.dressed_urdf)}"/></div>
      </div>
      <div class="form-group">
        <h4>Position (m)</h4>
        ${posRows(r.position)}
      </div>
      <div class="form-group">
        <h4>Orientation (°)</h4>
        ${orientRows(r.orientation)}
      </div>
      <div class="form-group">
        <h4>Home pose (° for turning joints, m for sliding ones)</h4>
        ${
          mismatch
            ? `<div class="muted">home_pose_deg has ${r.home_pose_deg.length} entries, the URDF ${jointCount} actuated joints: kept as it is until a joint is edited or reset</div>`
            : ""
        }
        ${
          jointCount === 0
            ? `<div class="muted">URDF has no actuated joints (or failed to parse)${recorded ? `; home_pose_deg kept: [${r.home_pose_deg.map(fmtAngle).join(", ")}]` : ""}</div>`
            : jointInfo.joints
                .map(
                  (j, i) => `
            <div class="row">
              <label>${attr(j.name)} (${j.kind === "prismatic" ? "m" : "°"})</label>
              <input type="number" inputmode="decimal" lang="en" step="${j.kind === "prismatic" ? "0.001" : "1"}" data-field="home_pose_deg" data-idx="${i}" value="${fmtAngle(homePose[i])}"/>
            </div>`
                )
                .join("")
        }
        <div class="btn-row">
          <button data-action="reset-home">Reset to default</button>
          <button data-action="clear-home">Use default (drop override)</button>
        </div>
      </div>
      <div class="form-group">
        <div class="btn-row">
          <button data-action="delete" class="btn-danger">Remove robot</button>
        </div>
      </div>
    `;
    this.form.innerHTML = html;
    this._wireRobotEvents(state, index, jointCount, kinds);
  }

  showFixture(state, index) {
    const f = state.fixtures[index];
    if (!f) { this.showEmpty(); return; }
    this.empty.hidden = true;
    this.form.hidden = false;
    this.form.innerHTML = `
      <div class="form-group">
        <h4>Fixture</h4>
        <div class="row"><label>id</label><input type="text" data-field="id" value="${attr(f.id)}"/></div>
        <div class="row"><label>urdf</label><input type="text" data-field="urdf" value="${attr(f.urdf)}"/></div>
      </div>
      <div class="form-group">
        <h4>Position (m)</h4>
        ${posRows(f.position)}
      </div>
      <div class="form-group">
        <h4>Orientation (°)</h4>
        ${orientRows(f.orientation)}
      </div>
      <div class="form-group">
        <div class="btn-row">
          <button data-action="delete" class="btn-danger">Remove fixture</button>
        </div>
      </div>
    `;
    this._wireFixtureEvents(state, index);
  }

  _wireRobotEvents(state, index, jointCount, kinds) {
    const r = state.robots[index];
    this.form.querySelectorAll("input").forEach((el) => {
      el.addEventListener("input", () => {
        const f = el.dataset.field;
        const v = el.type === "number" ? parseNum(el.value) : el.value;
        if (f === "robot_id" || f === "dressed_urdf") {
          r[f] = v;
        } else if (f.startsWith("position.")) {
          r.position[f.split(".")[1]] = v;
        } else if (f.startsWith("orientation.")) {
          r.orientation[f.split(".")[1]] = v;
        } else if (f === "home_pose_deg") {
          if (!Array.isArray(r.home_pose_deg) || r.home_pose_deg.length !== jointCount) {
            r.home_pose_deg = fitHomePoseDeg(r.home_pose_deg, jointCount, kinds);
          }
          r.home_pose_deg[parseInt(el.dataset.idx, 10)] = v;
        }
        this.onChange && this.onChange();
      });
    });
    this.form.querySelector('[data-action="reset-home"]')?.addEventListener("click", () => {
      r.home_pose_deg = defaultHomePoseDeg(jointCount, kinds);
      this.showRobot(state, index);
      this.onChange && this.onChange();
    });
    this.form.querySelector('[data-action="clear-home"]')?.addEventListener("click", () => {
      r.home_pose_deg = null;
      this.showRobot(state, index);
      this.onChange && this.onChange();
    });
    this.form.querySelector('[data-action="delete"]')?.addEventListener("click", () => {
      state.robots.splice(index, 1);
      this.showEmpty();
      this.onChange && this.onChange();
    });
  }

  _wireFixtureEvents(state, index) {
    const f = state.fixtures[index];
    this.form.querySelectorAll("input").forEach((el) => {
      el.addEventListener("input", () => {
        const field = el.dataset.field;
        const v = el.type === "number" ? parseNum(el.value) : el.value;
        if (field === "id" || field === "urdf") {
          f[field] = v;
        } else if (field.startsWith("position.")) {
          f.position[field.split(".")[1]] = v;
        } else if (field.startsWith("orientation.")) {
          f.orientation[field.split(".")[1]] = v;
        }
        this.onChange && this.onChange();
      });
    });
    this.form.querySelector('[data-action="delete"]')?.addEventListener("click", () => {
      state.fixtures.splice(index, 1);
      this.showEmpty();
      this.onChange && this.onChange();
    });
  }

  async _fetchJoints(urdfPath) {
    const key = apiPath(urdfPath);
    if (this.jointCache.has(key)) return this.jointCache.get(key);
    const data = await urdfJoints(key);
    this.jointCache.set(key, data);
    return data;
  }

  /** The actuated joints of `robot`'s URDF as fetched (`[{name, kind, …}]`),
   *  or undefined when they haven't been. */
  cachedJoints(robot) {
    return robot ? this.jointCache.get(apiPath(robot.dressed_urdf))?.joints : undefined;
  }

  /** Fetch (once) the joints of every robot in `robots` that has no
   *  recorded home pose, so a save writes the default sized to its URDF; a
   *  failed fetch leaves the robot's joints unknown. */
  async fetchJointsOfUnposed(robots) {
    const unposed = (robots || []).filter(
      (r) => !Array.isArray(r.home_pose_deg) || r.home_pose_deg.length === 0
    );
    await Promise.all(
      unposed.map((r) =>
        this._fetchJoints(r.dressed_urdf).catch((e) => console.warn("urdf joints failed:", e.message))
      )
    );
  }
}

/** Format a position coordinate (metres) — always 3 decimals,
 *  `.` separator. `0` → `"0.000"`, `1.388561` → `"1.389"`. */
function fmtPos(v) {
  const n = typeof v === "number" ? v : parseFloat(v);
  if (!Number.isFinite(n)) return "0.000";
  return n.toFixed(3);
}

/** Format an angle (degrees) for display — at most 6 decimals, no
 *  trailing zeros: `12.75` → `"12.75"`, `-90` → `"-90"`, `0` → `"0"`.
 *  Display only: the scene keeps the full value, and only an edit of
 *  the field replaces it (with what was typed, unrounded). */
function fmtAngle(v) {
  const n = typeof v === "number" ? v : parseFloat(v);
  if (!Number.isFinite(n)) return "0";
  return String(Number(n.toFixed(6)));
}

/** Parse a number from an input value, accepting both `.` and `,` decimal
 *  separators (a stray comma typed by accident shouldn't silently become NaN).
 *  Empty / invalid → 0. */
function parseNum(s) {
  if (s == null || s === "") return 0;
  const cleaned = String(s).replace(",", ".");
  const n = parseFloat(cleaned);
  return Number.isFinite(n) ? n : 0;
}

function posRows(p) {
  return ["x", "y", "z"]
    .map(
      (k) => `<div class="row"><label>${k}</label>
                <input type="number" inputmode="decimal" lang="en" step="0.001" data-field="position.${k}" value="${fmtPos(p[k])}"/></div>`
    )
    .join("");
}
function orientRows(o) {
  return ["a", "b", "c"]
    .map(
      (k) => `<div class="row"><label>${k}</label>
                <input type="number" inputmode="decimal" lang="en" step="1" data-field="orientation.${k}" value="${fmtAngle(o[k])}"/></div>`
    )
    .join("");
}

function attr(s) {
  return String(s ?? "").replace(/&/g, "&amp;").replace(/"/g, "&quot;").replace(/</g, "&lt;");
}
