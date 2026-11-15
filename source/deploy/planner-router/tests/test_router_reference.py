"""The planner router on reference cells (cells reference their Robo Fitter scene, never copy it):

    python3 -I -m unittest discover -s deploy/planner-router/tests

A reference cell's folder holds its outputs only (agentic_planner/: the prepared model, task programs, plans) and
factoryos-cell.json {"format": 2, "kind": "reference", "fitted_scene": "jam2"}; its scene is the folder
<assets>/scenes/jam2/scene.yaml (a legacy manifest's "jam2.yaml" names it too), its robots the catalogue's urdf-dressed-robots/, its products the products folder.
The backend lays them out in the cell's view .view/ (symlinks + a view-relative scene.yaml).
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_router import RouterBase, R  # noqa: E402

SCENE = ("name: jam2\nrobots:\n- robot_id: r1\n  dressed_urdf: ../../urdf-dressed-robots/r/r.urdf\n"
         "  position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\nfixtures:\n- id: bug_cube\n  urdf: {products}/bug_cube/bug_cube.urdf\n"
         "  position:\n    x: 1.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    a: 0.0\n    b: 0.0\n    c: 0.0\n")
VIEW_SCENE = ("name: jam2\nrobots:\n- robot_id: r1\n  dressed_urdf: robots/r/r.urdf\n"
              "  position:\n    x: 0.0\n    y: 0.0\n    z: 0.0\nfixtures:\n- id: bug_cube\n  urdf: fixtures/bug_cube/bug_cube.urdf\n"
              "  position:\n    x: 1.0\n    y: 0.0\n    z: 0.0\n  orientation:\n    a: 0.0\n    b: 0.0\n    c: 0.0\n")
PRODUCT = {"name": "bug_cube", "tasks": [{"name": "spot weld 1", "process": "spotWeld", "operations": [
    {"id": "W01", "target": {"kind": "point", "position": [0.0, 0.0, 1.0], "normal": [0.0, 0.0, 1.0]}}]}]}
INPUTS = {"scene": "s" * 64, "urdfs": {"r1": "u" * 64}, "fixtures": {"bug_cube": "f" * 64}, "usda": None,
          "combined": "c" * 64}


class ReferenceCellTest(RouterBase):
    def setUp(self):
        super().setUp()
        t = self.tmp
        self.assets, self.products = t / "assets", t / "products"
        robot = self.assets / "urdf-dressed-robots" / "r"
        (robot / "meshes").mkdir(parents=True)
        (robot / "r.urdf").write_text("<robot name='r'/>")
        (robot / "meshes" / "link1.obj").write_text("o link1")
        (self.products / "bug_cube").mkdir(parents=True)
        (self.products / "bug_cube" / "bug_cube.urdf").write_text("<robot name='p'/>")
        (self.products / "bug_cube" / "product.json").write_text(json.dumps(PRODUCT))
        for name in ("jam2", "new-scene", "jam-3"):
            (self.assets / "scenes" / name).mkdir(parents=True)
        self.scene = self.assets / "scenes" / "jam2" / "scene.yaml"
        self.scene.write_text(SCENE.replace("{products}", str(self.products)))
        (self.assets / "scenes" / "new-scene" / "scene.yaml").write_text("name: new-scene\nrobots: []\n")
        (self.assets / "scenes" / "jam-3" / "scene.yaml").write_text("name: jam-3\nrobots: []\n")
        cfg = json.loads(self.config.read_text())
        cfg.update(assetRoot=str(self.assets), productsDir=str(self.products))
        self.config.write_text(json.dumps(cfg))
        # the reference cell, prepared: its outputs only
        self.ref = t / "cells" / "jam2"
        for rel, text in (("factoryos-cell.json", json.dumps({"format": 2, "kind": "reference",
                                                               "fitted_scene": "jam2", "cell_id": "jam2"})),
                          (R.PREPARED_MARKER, "name: jam2\n"),
                          ("agentic_planner/config/robots.yaml", "robots: 3\n"),
                          (R.PREPARED_INPUTS, json.dumps(INPUTS)),
                          (f"{R.TASKS_DIR}/welds.json", '{"operations": [{"id": "W01"}]}')):
            (self.ref / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.ref / rel).write_text(text)
        # a cell the old Create Cell copied from new-scene.yaml (format 1: legacy, its own scene.yaml)
        self.copy = t / "cells" / "cell_new_scene"
        self.copy.mkdir()
        (self.copy / "factoryos-cell.json").write_text('{"fitted_scene": "new-scene.yaml"}')
        (self.copy / "scene.yaml").write_text("name: n\nrobots: []\n")

    def make_view(self):
        """The backend's view of the reference cell (cell_resolve::materialise_view)."""
        view = self.ref / R.VIEW_DIR
        (view / "robots").mkdir(parents=True)
        (view / "fixtures").mkdir()
        os.symlink(self.assets / "urdf-dressed-robots" / "r", view / "robots" / "r")
        os.symlink(self.products / "bug_cube", view / "fixtures" / "bug_cube")
        os.symlink("../agentic_planner", view / "agentic_planner")
        (view / "scene.yaml").write_text(VIEW_SCENE)
        (view / ".inputs").write_text(INPUTS["combined"])   # built for the inputs it was prepared from
        return view

    # ---------------------------------------------------------------- where its files are
    def test_scene_and_robots_of_both_kinds_of_cell(self):
        self.assertTrue(R.is_reference_cell(self.ref))
        self.assertFalse(R.is_reference_cell(self.cell), "welding_cell_2_small: its own scene.yaml")
        self.assertFalse(R.is_reference_cell(self.copy), "a copy: legacy")
        self.assertEqual(self.router.scene_path(self.cell), self.cell / "scene.yaml", "legacy paths unchanged")
        self.assertEqual(self.router.robots_dir(self.cell), self.cell / "robots")
        self.assertEqual(self.router.scene_path(self.ref), self.scene, "no view yet: the Robo Fitter scene")
        self.assertIsNone(self.router.robots_dir(self.ref))
        view = self.make_view()
        self.assertEqual(self.router.scene_path(self.ref), view / "scene.yaml")
        self.assertEqual(self.router.robots_dir(self.ref), view / "robots")
        # the backend's active cellDir of a reference cell is its view: the cell is still the folder
        self.assertEqual(R.cell_root(str(view)), self.ref)

    def test_cell_view_of_a_reference_cell(self):
        view = self.make_view()
        v = self.router.build_view("jam2", self.ref) / "cells" / "jam2"
        self.assertEqual((v / "scene.yaml").resolve(), (view / "scene.yaml").resolve())
        self.assertTrue((v / "robots" / "r" / "meshes").is_dir(), "the catalogue's meshes")
        self.assertEqual((v / "robots" / "r").resolve(), (self.assets / "urdf-dressed-robots" / "r").resolve())
        self.assertTrue((v / "fixtures" / "bug_cube" / "product.json").is_file())
        self.assertTrue((v / "agentic_planner" / "config" / "robots.yaml").is_file())
        self.assertTrue((v / "agentic_planner" / "tasks" / f"{self.router.canonical_task}.json").is_file())
        self.assertFalse((v / R.VIEW_DIR).exists(), "the view itself is not linked in")
        self.assertTrue((v / "factoryos-cell.json").is_file())

    def test_cell_view_without_the_backends_view(self):
        v = self.router.build_view("jam2", self.ref) / "cells" / "jam2"
        self.assertEqual((v / "scene.yaml").resolve(), self.scene.resolve(), "the scene, by its real path")
        self.assertFalse((v / "robots").exists())

    def test_the_legacy_reference_cell_view_is_unchanged(self):
        v = self.router.build_view(self.CID, self.cell) / "cells" / self.CID
        self.assertEqual((v / "robots").resolve(), (self.cell / "robots").resolve())
        self.assertEqual(self.router.cell_sources(self.CID, self.cell)["scene.yaml"], self.cell / "scene.yaml")
        self.assertTrue(self.router.training_cell_unchanged(self.CID, self.cell))

    def test_a_migrated_training_cell_is_still_the_policies_training_cell(self):
        """fos-migrate-cells imported welding_cell_2_small: its robot as r@welding_cell_2_small in the catalogue (the
        catalogue's r is another), its scene as scenes/welding_cell_2_small/scene.yaml with that path rewritten; the
        manifest records the scene's sha256 before and as written."""
        trained = self.cell / "scene.yaml"
        before = R.sha256_file(trained)
        robot = self.assets / "urdf-dressed-robots" / "r@welding_cell_2_small"
        (robot / "meshes").mkdir(parents=True)
        (robot / "r.urdf").write_bytes((self.cell / "robots/r/r.urdf").read_bytes())
        fitted = self.assets / "scenes" / "welding_cell_2_small" / "scene.yaml"
        fitted.parent.mkdir()
        fitted.write_text(trained.read_text() + "# paths rewritten\n")
        (self.cell / "scene.yaml").unlink()
        shutil.rmtree(self.cell / "robots")
        (self.cell / R.MANIFEST).write_text(json.dumps({
            "format": 2, "kind": "reference", "fitted_scene": "welding_cell_2_small", "cell_id": self.CID,
            "imported": {"scene_sha256": before, "fitted_sha256": R.sha256_file(fitted)}}))
        # the backend's view: the variant is `r` again
        view = self.cell / R.VIEW_DIR
        (view / "robots").mkdir(parents=True)
        os.symlink(robot, view / "robots" / "r")
        (view / "scene.yaml").write_text("scene: view-relative\n")
        self.assertTrue(R.is_reference_cell(self.cell))
        self.assertEqual(self.router.cell_sources(self.CID, self.cell)["scene.yaml"], fitted)
        self.assertTrue(self.router.training_cell_unchanged(self.CID, self.cell))
        self.assertFalse(self.router.u3_own_cell(self.CID, self.cell), "U3 plans umarell's copy, as before")
        policy = next(p for p in self.cat["planners"] if p["kind"] == "policy")
        ok, why, warnings = self.router.gate(self.router.planners[policy["name"]], self.CID, self.cell)
        self.assertTrue(ok, why)
        self.assertFalse([w for w in warnings if "differs from the cell the policies were trained on" in w], warnings)
        # the scene saved since: no longer the training cell
        fitted.write_text(fitted.read_text() + "# moved a robot\n")
        self.assertFalse(self.router.training_cell_unchanged(self.CID, self.cell))
        self.assertTrue(self.router.u3_own_cell(self.CID, self.cell))

    def test_a_policy_plans_a_changed_reference_training_cell_in_its_own_view(self):
        """The release's welding_cell_2_small: a reference cell whose fitted scene (with its fixture) is not the
        policies' training scene. A policy without a task plans the cell's own program against its own prepared model,
        never umarell's training copy (its obstacles lack the fixture)."""
        fitted = self.assets / "scenes" / self.CID / "scene.yaml"
        fitted.parent.mkdir()
        fitted.write_text("name: welding_cell_2_small\nfixtures:\n- id: car_door_fixture\n")
        (self.cell / "scene.yaml").unlink()
        shutil.rmtree(self.cell / "robots")
        (self.cell / R.MANIFEST).write_text(json.dumps({"format": 2, "kind": "reference",
                                                        "fitted_scene": self.CID, "cell_id": self.CID}))
        for rel, text in ((R.PREPARED_MARKER, "name: x\n"), (R.PREPARED_INPUTS, json.dumps(INPUTS)),
                          ("agentic_planner/config/robots.yaml", "robots: own\n"),
                          ("agentic_planner/config/obstacles.yaml", "obstacles: [car_door_fixture]\n"),
                          (f"{R.TASKS_DIR}/car_door_spot_welding.json", '{"operations": [{"id": "W01", "own": 1}]}')):
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text(text)
        view = self.cell / R.VIEW_DIR
        (view / "robots").mkdir(parents=True)
        os.symlink(self.assets / "urdf-dressed-robots" / "r", view / "robots" / "r")
        (view / "scene.yaml").write_text("scene: view-relative\n")
        (view / ".inputs").write_text(INPUTS["combined"])
        self.assertFalse(self.router.training_cell_unchanged(self.CID, self.cell))
        ok, why, warnings = self.router.gate(self.router.planners["hier_la_fu"], self.CID, self.cell)
        self.assertTrue(ok, why)
        self.assertIn("they plan the cell's own prepared model", " ".join(warnings))
        self.assertNotIn("the policies use the training copy", " ".join(warnings))
        own = self.cell / R.TASKS_DIR / "car_door_spot_welding.json"
        for task in (None, "(none yet)"):
            self.set_spec("hier_la_fu", "fake:a,sleep=0.5")
            r = self.router.plan("hier_la_fu", task=task)
            job = self.router.jobs[r["job_id"]]
            self.assertEqual(job.task_file, own, task)
            deadline = time.time() + 10
            while job.view is None and time.time() < deadline:
                time.sleep(0.02)
            v = job.view / "cells" / self.CID
            self.assertEqual((v / "agentic_planner" / "config").resolve(),
                             (self.cell / "agentic_planner" / "config").resolve(), "the cell's own prepared model")
            self.assertEqual((v / "agentic_planner" / "tasks" / "car_door_spot_welding.json").resolve(), own.resolve())
            self.assertEqual((v / "scene.yaml").resolve(), (view / "scene.yaml").resolve())
            st = self.wait(r["job_id"])
            self.assertEqual(st["status"], "done", st)
        # without a program of its own: umarell's canonical program, still in the cell's own view
        own.unlink()
        r = self.router.plan("hier_la_fu")
        canon = self.tmp / "umarell" / "cells" / self.CID / R.TASKS_DIR / "car_door_spot_welding.json"
        self.assertEqual(self.router.jobs[r["job_id"]].task_file, canon)
        self.assertEqual(self.wait(r["job_id"])["status"], "done")

    # ---------------------------------------------------------------- its products and scene
    def test_products_and_describe_cell_read_the_referenced_scene(self):
        for view in (False, True):
            if view:
                self.make_view()
            products = self.router.cell_products(self.ref)
            self.assertEqual([p["product"] for p in products], ["bug_cube"], view)
            self.assertEqual(Path(products[0]["product_file"]).resolve(),
                             (self.products / "bug_cube" / "product.json").resolve())
            self.assertEqual(products[0]["welds"][0]["position"], [1.0, 0.0, 1.0])
        d = self.router.describe_cell("jam2")
        self.assertEqual((d["kind"], d["product_welds"]), ("reference", 1))
        self.assertIn("robot_id: r1", d["scene_yaml"])
        self.assertEqual(self.router.describe_cell(self.CID)["kind"], "legacy")

    # ---------------------------------------------------------------- implicit creation
    def test_planning_a_scene_without_a_cell_makes_its_cell(self):
        self.assertEqual(self.router.resolve_cell("jam2"), ("jam2", self.ref), "an existing cell")
        self.assertEqual(self.router.resolve_cell("new-scene.yaml")[0], "cell_new_scene", "the copy made from it")
        name, root = self.router.resolve_cell("jam-3.yaml")
        self.assertEqual(name, "jam_3")
        m = json.loads((root / R.MANIFEST).read_text())
        self.assertEqual(m, {"format": 2, "kind": "reference", "fitted_scene": "jam-3", "cell_id": "jam_3"}, "by name")
        self.assertEqual(sorted(p.name for p in root.iterdir()), [R.MANIFEST], "only the manifest is written")
        self.assertEqual(self.router.resolve_cell("jam-3")[0], "jam_3", "by the scene's stem, again: the same cell")
        with self.assertRaises(R.ToolError):
            self.router.resolve_cell("nothing")

    # ---------------------------------------------------------------- plans record their inputs
    def test_a_plan_records_the_inputs_it_was_made_from(self):
        self.make_view()
        self.set_spec("hier_la_fu", "fake:a")
        job = self.router.plan("hier_la_fu", cell="jam2")
        st = self.wait(job["job_id"])
        self.assertEqual(st["status"], "done", st)
        plan = json.loads((self.ref / st["plan_path"]).read_text())
        self.assertEqual(plan["planner_meta"]["inputs"], INPUTS)
        self.assertEqual(plan["cell_id"], "jam2")

    def test_umarell_plans_get_the_inputs_too(self):
        plans = self.ref / R.PLANS_DIR
        plans.mkdir(parents=True)
        (plans / "bt_1_jam2.json").write_text('{"behaviour_tree": {}, "planner_meta": {"model": "x"}}')
        self.router.stamp_inputs(self.ref, f"{R.PLANS_DIR}/bt_1_jam2.json")
        plan = json.loads((plans / "bt_1_jam2.json").read_text())
        self.assertEqual(plan["planner_meta"], {"model": "x", "inputs": INPUTS})
        # recorded once: a plan keeps the inputs it was made from
        (self.ref / R.PREPARED_INPUTS).write_text(json.dumps({**INPUTS, "combined": "d" * 64}))
        self.router.stamp_inputs(self.ref, f"{R.PLANS_DIR}/bt_1_jam2.json")
        self.assertEqual(json.loads((plans / "bt_1_jam2.json").read_text())["planner_meta"]["inputs"], INPUTS)

    def test_a_stale_prepared_model_is_prepared_again(self):
        self.assertIsNone(self.router.prepared_stale("jam2"), "the backend can't tell: not stale")
        calls = []
        self.router._prepare = lambda name, root: calls.append(name)
        # no view (e.g. a restored backup leaves views out): preparing builds it
        self.router._ensure_prepared("jam2", self.ref)
        self.assertEqual(calls, ["jam2"])
        view = self.make_view()
        (view / ".inputs").write_text("d" * 64)
        self.router._ensure_prepared("jam2", self.ref)
        self.assertEqual(calls, ["jam2", "jam2"], "a view of other inputs")
        (view / ".inputs").write_text(INPUTS["combined"])
        self.router._ensure_prepared("jam2", self.ref)
        self.assertEqual(calls, ["jam2", "jam2"], "a current view")
        calls.clear()
        self.router.prepared_stale = lambda name: "the scene jam2.yaml changed"
        self.router._prepare = lambda name, root: calls.append(name)
        self.router._ensure_prepared("jam2", self.ref)
        self.assertEqual(calls, ["jam2"])
        self.router.prepared_stale = lambda name: None
        self.router._ensure_prepared("jam2", self.ref)
        self.assertEqual(calls, ["jam2"], "prepared and current: nothing to do")

    def test_u3_stages_a_reference_cell_with_its_view(self):
        sys.path.insert(0, str(Path(R.__file__).parent))
        import umarell_u3 as U
        view = self.make_view()
        wc = self.tmp / "wc"
        (wc / "cells").mkdir(parents=True)
        task = U.stage_cell(wc, self.tmp / "umarell", "jam2", self.ref, True, self.ref / R.TASKS_DIR / "welds.json")
        c = wc / "cells" / "jam2"
        self.assertTrue((c / "agentic_planner" / "tasks" / task).is_file())
        self.assertEqual((c / "scene.yaml").resolve(), (view / "scene.yaml").resolve())
        self.assertTrue((c / "robots" / "r" / "meshes").is_dir(), "the catalogue's meshes")
        self.assertFalse((c / R.VIEW_DIR).exists())
