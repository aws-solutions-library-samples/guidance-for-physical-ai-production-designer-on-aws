"""Install small, reviewable integration hooks in an isolated Umarell v8 copy."""
from pathlib import Path
import difflib
import hashlib
import json


def install(root: Path, patch_file: Path | None = None):
    if not (root / "UF_APPLIED.json").is_file():
        raise ValueError("install only into a private copy created by umarell_fast.apply")
    stamp = root / "WELD_INTEGRATION.json"
    if stamp.exists():
        prior = json.loads(stamp.read_text())
        if prior.get("installer") == hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
            if all(hashlib.sha256((root / rel).read_bytes()).hexdigest() == value
                   for rel, value in prior["files"].items()):
                return 0
    changes = []
    touched = set()

    def edit(rel, old, new):
        touched.add(rel)
        p = root / rel
        before = p.read_text()
        if new in before:
            return
        if before.count(old) != 1:
            raise ValueError(f"integration anchor changed: {rel}")
        after = before.replace(old, new)
        p.write_text(after)
        changes.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                            fromfile="a/" + rel, tofile="b/" + rel))

    edit("agents/trajectory/agent.py",
         "from langgraph.prebuilt import create_react_agent\n",
         "# LangGraph is imported when the LLM planning path is selected.\n")
    edit("agents/trajectory/agent.py",
         "    return create_react_agent(\n",
         "    from langgraph.prebuilt import create_react_agent\n"
         "    return create_react_agent(\n")
    edit("agents/trajectory/agent.py",
         '    """The planning run itself. See :func:`plan`, which binds the model first."""\n',
         '    """The planning run itself. See :func:`plan`, which binds the model first."""\n'
         '    if (constraints or {}).get("weld_request") is not None:\n'
         '        from umarell_jam.service import native_entry\n'
         '        return native_entry(cell_id, operations, constraints, tree_id)\n')
    edit("agents/trajectory/tools/btcpp.py",
         '    "ExecuteTrajectory": {\n',
         '    "WeldPhase": {"_kind": "Action", "robot_ns": None, "weld_id": None,\n'
         '                  "phase": None, "duration_s": None, "settings": ""},\n'
         '    "ExecuteTrajectory": {\n')
    edit("agents/trajectory/tools/btcpp.py",
         '        "hold_duration_s":      0.5,\n',
         '        "hold_duration_s":      0.5,\n'
         '        "controller_exact":     False,\n')
    edit("agents/trajectory/tools/btcpp.py",
         '        "controller_exact":     False,\n',
         '        "controller_exact":     False,\n'
         '        "independent_clocks":   False,\n'
         '        "initial_velocity":     None,\n')
    edit("agents/trajectory/tools/btcpp.py",
         '    "WeldPhase": {"_kind": "Action", "robot_ns": None, "weld_id": None,\n',
         '    "WeldBoundary": {"_kind": "Action", "boundary_id": None},\n'
         '    "WeldPhase": {"_kind": "Action", "robot_ns": None, "weld_id": None,\n')
    edit("agents/trajectory/tools/behaviour_tree.py",
         '    "RequestInterlock", "ReleaseInterlock",\n',
         '    "RequestInterlock", "ReleaseInterlock", "WeldPhase",\n')
    edit("agents/trajectory/tools/behaviour_tree.py",
         '    "RequestInterlock", "ReleaseInterlock", "WeldPhase",\n',
         '    "RequestInterlock", "ReleaseInterlock", "WeldPhase", "WeldBoundary",\n')
    edit("agents/cell/bt_executor.py",
         '            "ExecuteTrajectory": self._exec_execute,\n',
         '            "ExecuteTrajectory": self._exec_execute,\n'
         '            "WeldPhase": self._exec_weld_phase,\n')
    edit("agents/cell/bt_executor.py",
         '            "WeldPhase": self._exec_weld_phase,\n',
         '            "WeldPhase": self._exec_weld_phase,\n'
         '            "WeldBoundary": self._exec_weld_boundary,\n')
    edit("agents/cell/bt_executor.py",
         '    def _exec_sequence(self, node: dict)',
         '    def _exec_weld_phase(self, node):\n'
         '        from umarell_jam.execution import execute_phase\n'
         '        return execute_phase(self, node)\n\n'
         '    def _exec_sequence(self, node: dict)')
    edit("agents/cell/bt_executor.py",
         '    def _exec_weld_phase(self, node):\n',
         '    def _exec_weld_boundary(self, node):\n'
         '        from umarell_jam.execution import execute_boundary\n'
         '        return execute_boundary(self, node)\n\n'
         '    def _exec_weld_phase(self, node):\n')
    edit("agents/cell/bt_executor.py",
         '            "trajectory": traj, "hold_duration_s": hold_s, "velocity_scale": vscale,\n',
         '            "trajectory": traj, "hold_duration_s": hold_s, "velocity_scale": vscale,\n'
         '            "controller_exact": bool(bt_port(node, "controller_exact", False)),\n')
    edit("agents/cell/bt_executor.py",
         '            if err <= tol:\n                return True, None\n',
         '            if err <= tol:\n'
         '                if not bt_port(node, "controller_exact", False):\n'
         '                    return True, None\n'
         '                status = self._rt.latest_status(robot_id) or {}\n'
         '                velocities = status.get("joint_velocities")\n'
         '                if velocities is not None and len(velocities) == len(target) and all(\n'
         '                        abs(float(v)) <= 1e-8 for v in velocities):\n'
         '                    return True, None\n')
    edit("agents/cell/bt_executor.py",
         '                if velocities is not None and len(velocities) == len(target) and all(\n'
         '                        abs(float(v)) <= 1e-8 for v in velocities):\n',
         '                from umarell_jam.execution import initial_velocity\n'
         '                expected_v = ([0.0] * len(target) if reason_code != "start_pose_mismatch"\n'
         '                              else initial_velocity(self, node, len(target)))\n'
         '                if velocities is not None and len(velocities) == len(target) and all(\n'
         '                        abs(float(v) - float(w)) <= 1e-8 for v, w in zip(velocities, expected_v)):\n')
    edit("agents/cell/bt_executor.py",
         '            actual = self._joints(robot_id)\n'
         '            if actual is None:\n'
         '                return True, None\n',
         '            actual = self._joints(robot_id)\n'
         '            if actual is None:\n'
         '                if bt_port(node, "controller_exact", False):\n'
         '                    return False, {"reason_code": "joint_feedback_missing", "robot_ns": robot_id}\n'
         '                return True, None\n')
    edit("agents/cell/bt_executor.py",
         '        deadline = self._rt.now() + timeout_s\n'
         '        err = None\n',
         '        if bt_port(node, "controller_exact", False):\n'
         '            tol = min(tol, 1e-8)\n'
         '        deadline = self._rt.now() + timeout_s\n'
         '        err = None\n')
    edit("agents/cell/bt_executor.py",
         '                err = _max_joint_error(expected, actual)\n'
         '                if err > self._cfg.schedule_deviation_rad:\n',
         '                err = _max_joint_error(expected, actual)\n'
         '                if bt_port(node, "independent_clocks", False):\n'
         '                    from umarell_jam.tracking import path_error\n'
         '                    err = path_error(traj, actual, status)\n'
         '                    if err > 1e-7:\n'
         '                        deviating_since = now - self._cfg.schedule_deviation_grace_s\n'
         '                        err = float("inf")\n'
         '                if err > self._cfg.schedule_deviation_rad:\n')
    edit("agents/robot/robot_agent.py",
         '    return bool(task.get("vass")) and bool(traj) and all(\n',
         '    return bool(task.get("vass") or task.get("controller_exact")) and bool(traj) and all(\n')
    if patch_file is not None and changes:
        patch_file.parent.mkdir(parents=True, exist_ok=True)
        patch_file.write_text("".join(changes))
    stamp.write_text(json.dumps({"installer": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                 "files": {rel: hashlib.sha256((root / rel).read_bytes()).hexdigest()
                                           for rel in sorted(touched)}}, indent=2))
    return len(changes)
