"""umarell_jam: the router runs umarell_jam.py, which runs a fake `python -m umarell_jam` (its CLI contract) and, on a
miss, the fake U3 of test_router.py through umarell_u3.py (standard library only; no S3: the shared cache is off or a
stub ``aws`` on a local folder).

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import time
import unittest
from argparse import Namespace
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import server as R  # noqa: E402
import test_router as T  # noqa: E402 - its fixtures (not its tests: a module attribute, not collected here)
import catalogue_cache as CC  # noqa: E402
import umarell_jam as J  # noqa: E402
import umarell_u3 as U  # noqa: E402

# Stands in for factoryos-cell-planner's umarell_jam (python -m umarell_jam): what it does comes from <root>/fake.json
# {"mode": ok | ok_misses | miss | invalid | setup | crash | hang | slow_progress, "bundle_kind", "bundle_fail"}; it
# records its plan-cell command line in <root>/last_call.json. `import` logs into <root>/imported.log, `bundle` writes
# a bundle with a MANIFEST.json.
FAKE_JAM = r'''
import argparse, json, os, sys, time
from pathlib import Path
ap = argparse.ArgumentParser(prog="umarell_jam")
ap.add_argument("--root", required=True)
ap.add_argument("--workspace")
ap.add_argument("--umarell")
sub = ap.add_subparsers(dest="cmd", required=True)
im = sub.add_parser("import")
im.add_argument("bundles", nargs="+")
bu = sub.add_parser("bundle")
bu.add_argument("--out", required=True)
bu.add_argument("--cells", nargs="+", required=True)
pc = sub.add_parser("plan-cell")
pc.add_argument("--cell-id", required=True)
pc.add_argument("--task-file")
pc.add_argument("--welds", nargs="+")
pc.add_argument("--robots", nargs="+")
pc.add_argument("--speed", choices=("max", "normal"))
for o in ("--timeout", "--repair-budget", "--prepare-budget"):
    pc.add_argument(o, type=float)
pc.add_argument("--job")
pc.add_argument("--out", required=True)
a = ap.parse_args()
root = Path(a.root)
root.mkdir(parents=True, exist_ok=True)
fake = json.loads((root / "fake.json").read_text()) if (root / "fake.json").is_file() else {}
if a.cmd == "import":                  # the bundles' names into <root>/imported.log; a "bad*" one fails
    import tarfile
    for b in a.bundles:
        kind = None
        if tarfile.is_tarfile(b):
            with tarfile.open(b) as tf:
                kind = json.loads(tf.extractfile("MANIFEST.json").read()).get("kind")
        if Path(b).name.startswith("bad") or kind not in (None, "umarell-jam-components"):
            print(f"{b}: bad manifest or kind {kind!r}", file=sys.stderr); sys.exit(1)
        for log in [root / "imported.log"] + ([Path(os.environ["FAKE_IMPORT_LOG"])] if os.environ.get("FAKE_IMPORT_LOG") else []):
            with open(log, "a") as f:
                f.write(Path(b).name + "\n")
    sys.exit(0)
if a.cmd == "bundle":                  # --out with a MANIFEST.json (fake.json: bundle_kind, bundle_fail)
    import io, tarfile
    if fake.get("bundle_fail"):
        print("nothing cached for " + ",".join(a.cells), file=sys.stderr); sys.exit(1)
    man = json.dumps({"kind": fake.get("bundle_kind", "umarell-jam-components"), "cells": a.cells,
                      "planner": "0123456789abcdef", "impl": "fedcba9876543210"}).encode()  # pragma: allowlist secret
    dest = Path(a.out)                 # a directory: the bundle's own name, as umarell_jam's bundle picks it
    if dest.is_dir() or dest.suffix != ".tgz":
        dest = dest / ("+".join(a.cells) + "__01234567-fedcba98.tgz")
    with tarfile.open(dest, "w:gz") as tf:
        info = tarfile.TarInfo("MANIFEST.json"); info.size = len(man)
        tf.addfile(info, io.BytesIO(man))
    sys.exit(0)
mode = fake.get("mode", "ok")
import fcntl                          # umarell_jam takes R/.setup.lock itself: nobody may hold it for it
with open(root / ".setup.lock", "w") as _lock:
    try:
        fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock_free = True
    except OSError:
        lock_free = False
(root / "last_call.json").write_text(json.dumps({"argv": sys.argv[1:], "cwd": os.getcwd(),
    "UMARELL_BASE": os.environ.get("UMARELL_BASE"), "job": a.job or os.environ.get("UMARELL_JAM_JOB"),
    "python": sys.executable, "setup_lock_free": lock_free, "hppfcl": __import__("importlib").util.find_spec("hppfcl") is not None,
    "cell_exists": (Path(a.workspace) / "cells" / a.cell_id).is_dir()}))
run = root / "runs" / (a.job or os.environ.get("UMARELL_JAM_JOB") or "x")
run.mkdir(parents=True, exist_ok=True)
def progress(phase):
    (run / "progress.json").write_text(json.dumps({"phase": phase, "t": time.time()}))
progress("start")
out = Path(a.out)
welds = a.welds or ["W01"]
own = Path(a.workspace) / "cells" / a.cell_id / "agentic_planner" / "tasks"
if a.task_file and Path(a.task_file).resolve().parent != own.resolve():   # the contract: one of the cell's own
    out.with_name(out.name + ".result.json").write_text(json.dumps({"status": "invalid_request",
        "reason": "--task-file is not one of the cell's own programs"}))
    sys.exit(2)
def result(status, reason="", hits=1, misses=0):
    out.with_name(out.name + ".result.json").write_text(json.dumps({"status": status, "reason": reason,
        "timings": {"total_s": 0.4}, "cache": {"hits": hits, "misses": misses}, "weld_ids": welds,
        "robots": a.robots or ["r"]}))
print(f"[jam] plan-cell {a.cell_id} mode {mode}", flush=True)
if mode == "invalid":
    result("invalid_request", "unknown weld W99"); sys.exit(2)
if mode == "setup":
    result("setup_failed", "no patched umarell copy"); sys.exit(4)
if mode == "crash":
    raise RuntimeError("boom")
if mode == "miss":
    result("cache_miss", "2 weld primitive(s) not in the cache, repair budget spent", hits=1, misses=2); sys.exit(3)
if mode == "miss_home":   # every weld refused: the robot model is in self-collision at HOME (a bad dressed robot)
    err = ("self-collision check for {r} is required but unavailable (ValueError: like_kr240r2900_link_4 vs "
           "like_kr240r2900_link_6 is -0.0612 m at HOME (margin 0.0 m))")
    receipt = run / "receipt.json"
    receipt.write_text(json.dumps({"status": "timeout_or_unknown", "components": [
        {"robot": r, "weld": w, "status": "timeout_or_unknown", "mode": "native_fallback",
         "failures": [{"native_failure": {"failures": [{"native": {"legs": [{"failures": [{"error": err.format(r=r)}]}]}}]}}]}
        for w in ("W01", "W02") for r in ("robot1", "robot2")]}))
    out.with_name(out.name + ".result.json").write_text(json.dumps({"status": "timeout_or_unknown",
        "reason": "missing_certified_primitives", "timings": {"total_s": 0.4}, "cache": {"hits": 48, "misses": 400},
        "weld_ids": ["W01", "W02"], "missing_weld_ids": ["W01", "W02"], "robots": ["robot1", "robot2"],
        "prepare": {"status": "timeout_or_unknown", "receipt": str(receipt)}}))
    sys.exit(3)
if mode == "hang":
    time.sleep(60); sys.exit(0)
if mode == "slow_progress":           # no output, but progress.json keeps changing: alive
    for i in range(8):
        time.sleep(0.3); progress(f"step {i}")
out.write_text(json.dumps({"tree_id": "bt_jam", "cell_id": a.cell_id, "status": "ok"}))
result("ok", hits=len(welds), misses=2 if mode == "ok_misses" else 0)   # ok_misses: it filled 2 misses
sys.exit(0)
'''


def checkout(path: Path, layout: str) -> Path:
    """A umarell tree with the layout umarell_u3.base_of tells apart: 76ea13a, 08d5a39 or the public 8a02db9."""
    (path / "agents" / "trajectory" / "backends").mkdir(parents=True)
    if layout in ("08d5a39", "8a02db9"):
        (path / "utils").mkdir()
        (path / "utils" / "weld_surface.py").write_text("")
        (path / "agents" / "trajectory" / "plan_jobs.py").write_text("")
        tools = path / "agents" / "trajectory" / "tools"
        tools.mkdir()
        (tools / "fastgeom.py").write_text("")
        (tools / "plan_memo.py").write_text("")
    if layout == "8a02db9":
        (path / "agents" / "trajectory" / "context_compaction.py").write_text("def compacting_prompt(system):\n")
        (path / "agents" / "cell_defaults.py").write_text('DEFAULT_CELL_ID = "welding_cell_2_small"\n')
    return path


AWS_STUB = """#!/bin/sh
# aws s3 sync <uri> <dir> ...: the fake bucket's *.tgz into <dir>; aws s3 cp <file> <uri>: into the fake bucket's
# pushed/ (a local folder: nothing leaves this host); every call logged
echo "$@" >> "$FAKE_LOG"
case "$1 $2" in
  "s3 sync") cp "$FAKE_BUCKET"/*.tgz "$4"/ 2>/dev/null; exit 0 ;;
  "s3 cp") mkdir -p "$FAKE_BUCKET/pushed" && cp "$3" "$FAKE_BUCKET/pushed/$(basename "$4")"; exit 0 ;;
esac
exit 2
"""


# ---------------------------------------------------------------- umarell_jam.py on its own
class JamUnitTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)
        env = mock.patch.dict(os.environ)
        env.start()
        self.addCleanup(env.stop)
        for k in ("FACTORY_OS_JAM_UMARELL", "FACTORY_OS_JAM_CACHE_S3", "FACTORY_OS_JAM_CACHE_PUSH"):
            os.environ.pop(k, None)
        self.wsenv = self.tmp / "workspace.env"
        self.wsenv.write_text("FACTORY_OS_STORAGE=s3\nFACTORY_OS_S3_BUCKET=factoryos-data-1-eu-central-1\n"  # pragma: allowlist secret
                              "FACTORY_OS_S3_REGION=eu-central-1\n")  # pragma: allowlist secret
        for mod in (U, CC):
            p = mock.patch.object(mod, "WORKSPACE_ENV", self.wsenv)
            p.start()
            self.addCleanup(p.stop)

    def test_the_jam_bases_are_8a02db9_then_76ea13a(self):
        """A public install has <umarell>-8a02db9 (deploy/umarell-jam.pin), the internal servers <umarell>-76ea13a
        (deploy/umarell-jam-base.txt); with both, the public commit."""
        umarell = checkout(self.tmp / "umarell", "8a02db9")
        self.assertEqual(J.jam_source(umarell), umarell.resolve(), "the public checkout itself is a jam base")
        shutil.rmtree(umarell)
        umarell = checkout(self.tmp / "umarell", "08d5a39")
        old = checkout(self.tmp / "umarell-76ea13a", "76ea13a")
        self.assertEqual(J.jam_source(umarell), old.resolve())
        self.assertEqual(J.jam_base(J.jam_source(umarell)), "76ea13a")
        pub = checkout(self.tmp / "umarell-8a02db9", "8a02db9")
        self.assertEqual(J.jam_source(umarell), pub.resolve())
        self.assertEqual(J.jam_base(pub), "8a02db9")
        shutil.rmtree(pub)
        checkout(pub, "08d5a39")                          # a dir named for the public commit but holding 08d5a39
        self.assertEqual(J.jam_source(umarell), old.resolve())

    def test_jam_base_without_a_source_is_the_patched_copys(self):
        root = self.tmp / "root"
        self.assertEqual(J.jam_base(None, root), J.JAM_BASE)
        (root / "umarell").mkdir(parents=True)
        (root / "umarell" / "UF_APPLIED.json").write_text(json.dumps({"base": "76ea13a"}))
        self.assertEqual((J.copy_base(root), J.jam_base(None, root)), ("76ea13a", "76ea13a"))
        (root / "umarell" / "UF_APPLIED.json").write_text(json.dumps({"base": "08d5a39"}))   # no jam base
        self.assertEqual((J.copy_base(root), J.jam_base(None, root)), (None, J.JAM_BASE))

    def test_the_76ea13a_layout_is_a_jam_base(self):
        umarell = checkout(self.tmp / "umarell", "08d5a39")
        checkout(self.tmp / "umarell-base", "08d5a39")
        self.assertIsNone(J.jam_source(umarell), "08d5a39 is umarell_u3's base, not umarell_jam's")
        jam = checkout(self.tmp / "umarell-76ea13a", "76ea13a")
        self.assertEqual(J.jam_source(umarell), jam.resolve())
        shutil.rmtree(jam)
        shutil.rmtree(self.tmp / "umarell-base")
        base = checkout(self.tmp / "umarell-base", "76ea13a")       # umarell-base still at 76ea13a: that one
        self.assertEqual(J.jam_source(umarell), base.resolve())
        other = checkout(self.tmp / "elsewhere", "76ea13a")
        with mock.patch.dict(os.environ, {"FACTORY_OS_JAM_UMARELL": str(other)}):
            self.assertEqual(J.jam_source(umarell), other.resolve())

    def args(self, **kw) -> Namespace:
        a = dict(python="/venv/bin/python", root=str(self.tmp / "root"), workspace=str(self.tmp / "ws"),
                 cell_id="body_weld_3", task_file=None, welds=None, robots=None, speed="max", timeout=None,
                 repair_budget=None, prepare_budget=None)
        a.update(kw)
        return Namespace(**a)

    def test_plan_cell_command_line_is_the_contract(self):
        out = self.tmp / "job" / "jam" / "plan.json"
        cmd = J.plan_cell_cmd(self.args(), None, out, "bt_1_body_weld_3")
        self.assertEqual(cmd, ["/venv/bin/python", "-m", "umarell_jam", "--root", str((self.tmp / "root").resolve()),
                               "--workspace", str((self.tmp / "ws").resolve()), "plan-cell", "--cell-id",
                               "body_weld_3", "--speed", "max", "--job", "bt_1_body_weld_3", "--out", str(out)])
        task = self.tmp / "t.json"
        cmd = J.plan_cell_cmd(self.args(task_file=str(task), welds=["W01", "W02"], robots=["r1"], speed="normal",
                                        timeout=600.0, repair_budget=300.0, prepare_budget=0.0),
                              self.tmp / "u76", out)
        i = cmd.index("plan-cell")
        self.assertEqual(cmd[i - 2:i], ["--umarell", str(self.tmp / "u76")])
        self.assertEqual(cmd[i + 1:], ["--cell-id", "body_weld_3", "--task-file", str(task.resolve()),
                                       "--welds", "W01", "W02", "--robots", "r1", "--speed", "normal",
                                       "--timeout", "600", "--repair-budget", "300", "--prepare-budget", "0",
                                       "--out", str(out)])

    def test_shared_cache_uri_per_base(self):
        self.assertEqual(J.cache_uri(env_file=self.wsenv, base="76ea13a"),
                         "s3://factoryos-data-1-eu-central-1/catalogue/umarell-jam-cache/")
        self.assertEqual(J.cache_uri(env_file=self.wsenv, base="8a02db9"),
                         "s3://factoryos-data-1-eu-central-1/catalogue/umarell-jam-cache/8a02db9/")
        with mock.patch.dict(os.environ, {"FACTORY_OS_JAM_CACHE_S3": "s3://other/jam"}):   # as is, any base
            self.assertEqual(J.cache_uri(env_file=self.wsenv, base="8a02db9"), "s3://other/jam/")

    def test_shared_cache_uri(self):
        self.assertEqual(J.cache_uri(env_file=self.wsenv),
                         "s3://factoryos-data-1-eu-central-1/catalogue/umarell-jam-cache/")
        local = self.tmp / "local.env"
        local.write_text("FACTORY_OS_STORAGE=local\n")
        self.assertIsNone(J.cache_uri(env_file=local))
        with mock.patch.dict(os.environ, {"FACTORY_OS_JAM_CACHE_S3": "s3://other/jam"}):
            self.assertEqual(J.cache_uri(env_file=self.wsenv), "s3://other/jam/")
        with mock.patch.dict(os.environ, {"FACTORY_OS_JAM_CACHE_S3": "0"}):
            self.assertIsNone(J.cache_uri(env_file=self.wsenv))
            self.assertIn("no shared cache location", J.pull_shared_cache(self.cli()))

    def stub_aws(self) -> tuple[str, Path, Path]:
        (self.tmp / "bin").mkdir(exist_ok=True)
        aws = self.tmp / "bin" / "aws"
        aws.write_text(AWS_STUB)
        aws.chmod(0o755)
        bucket = self.tmp / "bucket"
        bucket.mkdir(exist_ok=True)
        os.environ.update(FAKE_LOG=str(self.tmp / "aws.log"), FAKE_BUCKET=str(bucket))
        return str(aws), bucket, self.tmp / "aws.log"

    def cli(self) -> J.JamCli:
        src = self.tmp / "src"
        (src / "umarell_jam").mkdir(parents=True, exist_ok=True)
        (src / "umarell_jam" / "__init__.py").write_text("")
        (src / "umarell_jam" / "__main__.py").write_text(FAKE_JAM)
        return J.JamCli(sys.executable, src, self.tmp / "root", self.tmp / "ws")

    def imported(self) -> list:
        f = self.tmp / "root" / "imported.log"
        return f.read_text().split() if f.is_file() else []

    def test_pull_imports_new_bundles_once(self):
        aws, bucket, calls = self.stub_aws()
        cli = self.cli()
        (bucket / "body_weld_3.tgz").write_bytes(b"x" * 10)
        note = J.pull_shared_cache(cli, uri="s3://b/catalogue/umarell-jam-cache/", aws=aws)
        self.assertIn("imported body_weld_3.tgz", note)
        self.assertEqual(self.imported(), ["body_weld_3.tgz"])
        self.assertIn("s3 sync s3://b/catalogue/umarell-jam-cache/", calls.read_text())
        self.assertIn("--region eu-central-1", calls.read_text())
        # again: nothing new -> no import; a new bundle -> only that one; a changed one again
        self.assertIn("none new", J.pull_shared_cache(cli, uri="s3://b/x/", aws=aws))
        (bucket / "body_weld_4x8.tgz").write_bytes(b"y" * 5)
        self.assertIn("imported body_weld_4x8.tgz", J.pull_shared_cache(cli, uri="s3://b/x/", aws=aws))
        self.assertEqual(self.imported(), ["body_weld_3.tgz", "body_weld_4x8.tgz"])
        done = json.loads((self.tmp / "root" / f"catalogue-pull-{J.JAM_BASE}" / ".imported.json").read_text())
        self.assertEqual(sorted(done), ["body_weld_3.tgz", "body_weld_4x8.tgz"])

    def test_pull_never_raises(self):
        aws, bucket, _ = self.stub_aws()
        cli = self.cli()
        (bucket / "bad.tgz").write_bytes(b"z")
        self.assertIn("FAILED bad.tgz", J.pull_shared_cache(cli, uri="s3://b/x/", aws=aws))
        self.assertNotIn("bad.tgz", json.loads((self.tmp / "root" / f"catalogue-pull-{J.JAM_BASE}" / ".imported.json")
                                            .read_text()))
        (self.tmp / "bin" / "aws").write_text("#!/bin/sh\necho denied >&2\nexit 1\n")
        self.assertIn("failed: denied", J.pull_shared_cache(cli, uri="s3://b/x/", aws=aws))


# ---------------------------------------------------------------- the router's umarell_jam jobs
class JamDiagnosisTest(unittest.TestCase):
    def receipt(self, comps):
        d = Path(tempfile.mkdtemp()); self.addCleanup(shutil.rmtree, d)
        f = d / "receipt.json"; f.write_text(json.dumps({"components": comps})); return str(f)

    def test_no_receipt_no_causes(self):
        d = J.diagnose({"status": "timeout_or_unknown", "missing_weld_ids": ["W03"]})
        self.assertEqual((d["failed_components"], d["causes"], d["missing_weld_ids"]), (0, [], ["W03"]))
        self.assertNotIn("blocker", d)
        self.assertEqual(J.diagnosis_text(d), "; missing 1 weld(s): W03")

    def test_mixed_causes_have_no_blocker_and_the_most_frequent_first(self):
        fail = lambda e: {"status": "timeout_or_unknown", "failures": [{"x": {"error": e}}]}  # noqa: E731
        rc = self.receipt([fail("IK: no solution for W01 at 3.65 m"), fail("IK: no solution for W07 at 3.70 m"),
                           fail("descent blocked at step 1/6"), {"status": "certified"}])
        d = J.diagnose({"attempts": [{"repair": {"receipt": rc}}, {"repair": {"receipt": rc}}]})
        self.assertEqual((d["failed_components"], d["certified_components"]), (3, 1), "a receipt read once")
        self.assertEqual(d["causes"][0]["count"], 2)
        self.assertNotIn("blocker", d)
        self.assertIn("most frequent failure (2 of 3): IK: no solution", J.diagnosis_text(d))

    def test_an_unreadable_receipt_is_skipped(self):
        d = J.diagnose({"prepare": {"receipt": "/nonexistent/receipt.json"}})
        self.assertEqual(d["failed_components"], 0)


class JamRouterTest(T.RouterBase):
    """umarell_jam through the router: a fake umarell_jam package, and the fake U3 for the fallback."""

    def setUp(self):
        super().setUp()
        env = mock.patch.dict(os.environ, {"FACTORY_OS_JAM_CACHE_S3": "0", "FACTORY_OS_U3_CACHE_S3": "0"})
        env.start()
        self.addCleanup(env.stop)
        for k in ("FACTORY_OS_JAM_ROOT", "FACTORY_OS_JAM_UMARELL", "FACTORY_OS_JAM_CACHE_PUSH",
                  "FACTORY_OS_JAM_WORKSPACE", "FACTORY_OS_U3_WORKERS"):
            os.environ.pop(k, None)
        src = self.tmp / "src"
        (src / "umarell_jam").mkdir()
        (src / "umarell_jam" / "__init__.py").write_text("")
        (src / "umarell_jam" / "__main__.py").write_text(FAKE_JAM)
        # the fake U3 (the fallback), as test_router's U3Test sets it up
        u3 = src / "umarell_fast" / "u3"
        u3.mkdir(parents=True)
        (src / "umarell_fast" / "__init__.py").write_text("")
        (src / "umarell_fast" / "apply.py").write_text("")
        (u3 / "__init__.py").write_text("")
        (u3 / "__main__.py").write_text(T.FAKE_U3)
        (src / "docs" / "upstream" / "patches").mkdir(parents=True)
        (src / "docs" / "upstream" / "patches" / "01_x.diff").write_text("")
        self.router.s.umarell_python = Path(sys.executable)
        # umarell_jam's own venv next to the cell planner (provision-cell-planner.sh's jam-venv): this python with
        # stand-ins for its numerical runtime (the gate imports hppfcl and toppra in it)
        mods = self.tmp / "jam-mods"
        mods.mkdir()
        for m in ("hppfcl", "toppra"):
            (mods / f"{m}.py").write_text("")
        self.jam_py = self.tmp / "jam-venv" / "bin" / "python"
        self.jam_py.parent.mkdir(parents=True)
        self.jam_py.write_text(f'#!/bin/sh\nPYTHONPATH="{mods}${{PYTHONPATH:+:$PYTHONPATH}}" exec "{sys.executable}" "$@"\n')
        self.jam_py.chmod(0o755)
        # the cell's own task program (umarell_jam plans no other) and Prepare Cell's config (the job's view is the
        # cell's own)
        for rel, text in ((f"{R.TASKS_DIR}/car_door_spot_welding.json", '{"operations": [{"id": "W01"}]}'),
                          ("agentic_planner/config/robots.yaml", "robots: 1\n"),
                          ("agentic_planner/config/obstacles.yaml", "obstacles: []\n")):
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text(text)
        self.root = self.tmp / "work" / "umarell_jam"
        self.u3_root = self.tmp / "work" / "umarell_u3"
        self.which = {"patch": "/usr/bin/patch", "rsync": "/usr/bin/rsync"}
        real_which = R.shutil.which
        R.shutil.which = lambda cmd, *a, **k: self.which.get(cmd) if cmd in ("patch", "rsync") else real_which(cmd, *a, **k)
        self.addCleanup(setattr, R.shutil, "which", real_which)

    def fake(self, mode: str, **kw) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "fake.json").write_text(json.dumps({"mode": mode, **kw}))

    def last_call(self) -> dict:
        return json.loads((self.root / "last_call.json").read_text())

    def jam_cmd(self, job_id: str) -> tuple[list, list]:
        cmd = self.router.jobs[job_id].cmd
        i = cmd.index("--")
        return cmd[:i], cmd[i + 1:]

    def default_timeout(self) -> float:
        """umarell_jam's own timeout for a job without options: planners.json's plan_timeout_s (600), at least its
        budgets + 120 s (the public export sets prepare_budget_s 1200 and plan_timeout_s 2100: 2 x 300 + 1200 + setup)."""
        p = self.router.planners["umarell_jam"]
        return max(float(p.get("plan_timeout_s") or 600),
                   float(p.get("repair_budget_s") or 0) + float(p.get("prepare_budget_s") or 0) + 120.0)

    def published(self) -> list:
        plans = self.cell / R.PLANS_DIR
        return sorted(plans.iterdir()) if plans.is_dir() else []

    # ------------------------------------------------------------ the catalogue entry and the gate
    def test_gate(self):
        p = self.router.planners["umarell_jam"]
        self.assertEqual((p["kind"], p["robots"], p["fixture"], p["from"]), ("rules", "1-4", True, "plan:{jam_plan}"))
        ok, why, warnings = self.router.gate(p, self.CID, self.cell)
        self.assertTrue(ok, why)
        self.assertTrue(any("no plain umarell" in w for w in warnings), warnings)
        self.assertTrue(any("deploy/umarell-jam.pin" in w and "--umarell-jam-tarball" in w for w in warnings), warnings)
        self.assertTrue(any("component cache" in w for w in warnings), warnings)
        checkout(self.tmp / "umarell-76ea13a", "76ea13a")
        (self.root / "components" / "aa11").mkdir(parents=True)
        self.assertEqual(self.router.gate(p, self.CID, self.cell)[2], [])
        (self.tmp / "jam-mods" / "toppra.py").write_text("raise ImportError('no toppra here')")   # a venv without it
        self.jam_py.touch()        # (the runtime check is cached per interpreter and mtime)
        os.utime(self.jam_py, ns=(1, 1))
        warnings = self.router.gate(p, self.CID, self.cell)[2]
        self.assertTrue(any("numerical runtime is incomplete" in w and "no toppra here" in w for w in warnings),
                        warnings)
        shutil.rmtree(self.cell / R.TASKS_DIR)            # umarell's copy of the program does not count
        ok, why, _ = self.router.gate(p, self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("no task program of its own", why)
        self.jam_py.unlink()
        with mock.patch.object(self.router.s, "umarell_python", self.tmp / "none" / "python"):
            ok, why, _ = self.router.gate(p, self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("umarell_jam's python", why)
        shutil.rmtree(self.tmp / "src" / "umarell_jam")
        ok, why, _ = self.router.gate(p, self.CID, self.cell)
        self.assertFalse(ok)
        self.assertIn("has no umarell_jam", why)

    def test_not_recommended_unless_asked_for(self):
        (self.cell / "scene.yaml").write_text("scene: moved\n")        # outside the policies' training: U3
        self.assertEqual(self.router.recommend_planner()["recommended"], "umarell_u3")
        r = self.router.recommend_planner(request="plan it with umarell_jam")
        self.assertEqual((r["recommended"], r["ambiguous"]), ("umarell_jam", False))
        self.assertEqual(self.router.describe_planner("umarell_jam")["default_profile"], "umarell")

    # ------------------------------------------------------------ success and the option mapping
    def test_warm_plan_is_exported_and_published(self):
        jam_base = checkout(self.tmp / "umarell-76ea13a", "76ea13a")
        self.fake("ok")
        r = self.router.plan("umarell_jam")
        self.assertEqual((r["profile"], r["speed"]), ("umarell", "max"))       # planned at the cell limits
        self.assertEqual(r["fallback"]["enabled"], True)
        jam, export = self.jam_cmd(r["job_id"])
        self.assertEqual(export[export.index("--profile") + 1], "umarell")
        self.assertEqual(jam[jam.index("--python") + 1], str(self.jam_py))
        self.assertEqual(jam[jam.index("--u3-python") + 1], sys.executable)
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertTrue(st["export"]["passed"])
        jam = st["export"]["jam"]
        self.assertEqual((jam["status"], jam["cache"], jam["fallback"]["used"]), ("ok", {"hits": 1, "misses": 0}, False))
        call = self.last_call()
        argv = call["argv"]
        i = argv.index("plan-cell")
        self.assertEqual(argv[:i], ["--root", str(self.root.resolve()), "--workspace", str(self.tmp.resolve()),
                                    "--umarell", str(jam_base.resolve())])
        self.assertEqual(argv[i + 1:i + 3], ["--cell-id", self.CID])
        self.assertEqual(argv[argv.index("--speed") + 1], "max")
        self.assertEqual(argv[argv.index("--timeout") + 1], f"{self.default_timeout():g}")
        self.assertEqual(argv[argv.index("--repair-budget") + 1], "300")
        prepare = self.router.planners["umarell_jam"].get("prepare_budget_s")
        if prepare is None:                 # the internal planners.json: cache misses are only repaired
            self.assertNotIn("--prepare-budget", argv)
        else:                               # the public export's (scripts/export-public.py PUBLIC_PLANNER_DEFAULTS)
            self.assertEqual(argv[argv.index("--prepare-budget") + 1], f"{prepare:g}")
        self.assertEqual(Path(argv[argv.index("--task-file") + 1]),
                         (self.cell / R.TASKS_DIR / "car_door_spot_welding.json").resolve())
        self.assertEqual(argv[argv.index("--job") + 1], r["job_id"])
        self.assertEqual(Path(call["python"]), Path(sys.executable))   # through the jam-venv wrapper
        self.assertTrue(call["hppfcl"])
        self.assertTrue(call["setup_lock_free"], "the router holds no lock on R/.setup.lock")
        self.assertEqual((call["UMARELL_BASE"], call["job"], call["cell_exists"]), ("76ea13a", r["job_id"], True))
        self.assertEqual(Path(call["cwd"]).resolve(), (self.tmp / "src").resolve())
        plan = json.loads((self.cell / st["plan_path"]).read_text())
        self.assertEqual(plan["planner"], "umarell_jam")
        self.assertFalse((self.u3_root / "last_run.json").exists(), "no fallback on success")

    def test_a_public_install_plans_on_8a02db9(self):
        """deploy/umarell-jam.pin's public commit at <umarell>-8a02db9 (fetch-umarell.sh --jam) is umarell_jam's
        --umarell, also when an older 76ea13a tree is still installed; the job records the base."""
        checkout(self.tmp / "umarell-76ea13a", "76ea13a")
        pub = checkout(self.tmp / "umarell-8a02db9", "8a02db9")
        self.fake("ok")
        r = self.router.plan("umarell_jam")
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        call = self.last_call()
        argv = call["argv"]
        self.assertEqual(argv[argv.index("--umarell") + 1], str(pub.resolve()))
        self.assertEqual(call["UMARELL_BASE"], "8a02db9")
        self.assertEqual(st["export"]["jam"]["umarell_base"], "8a02db9")

    def test_options_are_mapped(self):
        self.fake("ok")
        r = self.router.plan("umarell_jam", options={"speed": "normal", "welds": ["W01", "W02"], "robots": "r1, r2",
                                                     "timeout_s": 120, "repair_budget_s": 30, "prepare_budget_s": 0})
        self.assertEqual((r["profile"], r["speed"]), ("umarell", "normal"))
        self.assertFalse(r["fallback"]["enabled"])
        self.assertIn("subset", r["fallback"]["why"])
        jam, export = self.jam_cmd(r["job_id"])
        self.assertEqual(export[export.index("--profile") + 1], "umarell")
        self.assertEqual(jam[jam.index("--welds") + 1], "W01,W02")
        self.assertIn("--allow-partial", export, "a requested weld subset is complete by request")
        self.assertNotIn("--fallback-u3", jam)
        self.assertEqual(r["timeout_s"], 120 + R.JAM_MARGIN_S)
        st = self.wait(r["job_id"])
        self.assertEqual(st["status"], "done", st)
        argv = self.last_call()["argv"]
        i = argv.index("plan-cell")
        self.assertEqual(argv[i + 3:], ["--task-file", argv[argv.index("--task-file") + 1], "--welds", "W01", "W02",
                                        "--robots", "r1", "r2", "--speed", "normal", "--timeout", "120",
                                        "--repair-budget", "30", "--prepare-budget", "0",
                                        "--job", r["job_id"], "--out", argv[-1]])
        self.assertEqual(st["export"]["jam"]["weld_ids"], ["W01", "W02"])

    def test_options_are_checked(self):
        for opts, msg in (({"welds": []}, "non-empty list"), ({"welds": ["W 1"]}, "non-empty list"),
                          ({"timeout_s": 0}, "number of seconds"), ({"repair_budget_s": True}, "number of seconds"),
                          ({"fallback": "yes"}, "true or false"), ({"remote": True}, "umarell_u3's"),
                          ({"speed": "ludicrous"}, "unknown options.speed")):
            with self.assertRaises(R.ToolError, msg=opts) as e:
                self.router.plan("umarell_jam", options=opts)
            self.assertIn(msg, str(e.exception), opts)
        with self.assertRaises(R.ToolError) as e:        # a subset U3 can't plan, with the fallback asked for
            self.router.plan("umarell_jam", options={"welds": ["W01"], "fallback": True})
        self.assertIn("options.fallback true, but umarell_u3 plans the whole task program", str(e.exception))
        with self.assertRaises(R.ToolError) as e:        # btexport's full profile refuses umarell_jam's trees
            self.router.plan("umarell_jam", profile="full")
        self.assertIn("umarell profile only", str(e.exception))
        r = self.router.plan("umarell_jam", profile="umarell", options={"speed": "max"})
        self.assertEqual((r["profile"], r["speed"]), ("umarell", "max"))
        self.wait(r["job_id"])
        with self.assertRaises(R.ToolError) as e:
            self.router.plan("hier_la_fu", options={"repair_budget_s": 10})
        self.assertIn("umarell_jam's", str(e.exception))

    def test_timeout_includes_the_fallback(self):
        with mock.patch.dict(os.environ, {"FACTORY_OS_U3_WORKERS": "2"}):
            r = self.router.plan("umarell_jam", options={"fallback": True})
            self.assertEqual(r["u3_workers"], 2)
            self.assertEqual(r["timeout_s"], self.default_timeout() + R.JAM_MARGIN_S + R.u3_timeout(4800, 2))
            self.wait(r["job_id"])
            r = self.router.plan("umarell_jam", options={"fallback": False})
            self.assertEqual(r["timeout_s"], self.default_timeout() + R.JAM_MARGIN_S)
            self.assertNotIn("u3_workers", r)
            self.wait(r["job_id"])

    # ------------------------------------------------------------ no plan: exit 3 / 4 / 2, with and without fallback
    def test_miss_without_fallback_publishes_nothing(self):
        self.fake("miss")
        st = self.wait(self.router.plan("umarell_jam", options={"fallback": False})["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertEqual(st["export"]["exit_code"], 3)
        self.assertIn("no complete plan", st["error"])
        self.assertIn("cache_miss", st["error"])
        self.assertIn("2 miss(es)", st["hint"])
        self.assertIn("fallback off: options.fallback false", st["hint"])
        self.assertEqual(st["export"]["jam"]["cache"], {"hits": 1, "misses": 2})
        self.assertEqual(self.published(), [])
        self.assertFalse((self.u3_root / "last_run.json").exists())

    def test_the_whole_task_is_not_exported_as_partial(self):
        self.fake("ok")
        r = self.router.plan("umarell_jam", options={"fallback": False})
        jam, export = self.jam_cmd(r["job_id"])
        self.assertNotIn("--allow-partial", export)
        self.assertEqual(self.wait(r["job_id"])["status"], "done")

    def test_miss_names_the_cause_shared_by_every_failure(self):
        """A robot model in self-collision at HOME read as '400 cache misses' with a hint to build the cache."""
        self.fake("miss_home")
        st = self.wait(self.router.plan("umarell_jam", options={"fallback": False})["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertIn("missing 2 weld(s): W01, W02", st["error"])
        self.assertIn("every failed attempt: self-collision check", st["error"])
        self.assertIn("at HOME", st["error"])
        self.assertIn("collides with itself at HOME", st["hint"])
        self.assertNotIn("prepare umarell_jam's cache", st["hint"])
        d = st["export"]["jam"]["diagnosis"]
        self.assertEqual((d["failed_components"], d["missing_weld_ids"]), (4, ["W01", "W02"]))
        self.assertEqual(d["causes"][0]["count"], 4, "robot1 / robot2 group together: numbers are blanked")

    def test_miss_with_fallback_plans_with_u3(self):
        self.fake("miss")
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertTrue(st["fallback"]["used"], st)
        self.assertEqual(st["export"]["u3"]["welds"], 1)
        jam = st["export"]["jam"]
        self.assertEqual((jam["status"], jam["fallback"]["used"], jam["fallback"]["exit_code"]), ("cache_miss", True, 0))
        run = json.loads((self.u3_root / "last_run.json").read_text())
        self.assertEqual((run["speed"], run["ops"]), ("max", ["W01"]))
        log = (self.router.jobs[st["job_id"]].dir / "btexport.log").read_text()
        self.assertIn(R.JAM_FALLBACK_MARK, log)
        self.assertIn("profile full", log, "U3's max-speed plan is exported in the full profile, as a U3 job's")
        self.assertEqual(len(self.published()), 1)

    def test_fallback_that_finds_no_plan_either(self):
        self.fake("miss")
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True, exist_ok=True)
        (tasks / "bad.json").write_text('{"operations": [{"id": "FAIL"}]}')
        st = self.wait(self.router.plan("umarell_jam", task="bad")["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertIn("fallback found none either", st["error"])
        self.assertIn("umarell_u3 fallback found none", st["hint"])
        self.assertEqual(self.published(), [])

    def test_setup_failure_with_and_without_fallback(self):
        self.fake("setup")
        st = self.wait(self.router.plan("umarell_jam", options={"fallback": False})["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertEqual(st["export"]["exit_code"], 4)
        self.assertIn("umarell_jam setup failed", st["error"])
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertTrue(st["fallback"]["used"])
        self.fake("crash")                               # a crash is a setup failure too
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertIn("exit 1", st["export"]["jam"]["fallback"]["why"])

    def test_invalid_request_never_falls_back(self):
        self.fake("invalid")
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "failed", st)
        self.assertEqual(st["export"]["exit_code"], 2)
        self.assertIn("umarell_jam refused the request", st["error"])
        self.assertIn("unknown weld W99", st["error"])
        self.assertFalse(st["fallback"]["used"])
        self.assertFalse((self.u3_root / "last_run.json").exists())

    # ------------------------------------------------------------ progress / hang detection
    def test_a_hang_is_killed(self):
        self.router.planners["umarell_jam"]["progress_timeout_s"] = 1
        self.fake("hang")
        t0 = time.time()
        st = self.wait(self.router.plan("umarell_jam", options={"fallback": False})["job_id"])
        self.assertLess(time.time() - t0, 20, "killed after progress_timeout_s, not after the fake's 60 s")
        self.assertEqual(st["status"], "failed", st)
        self.assertEqual(st["export"]["jam"]["status"], "hung")
        self.assertIn("no progress for", st["error"])
        self.fake("hang")
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertIn("hung", st["export"]["jam"]["fallback"]["why"])

    def test_progress_json_keeps_a_quiet_run_alive(self):
        self.router.planners["umarell_jam"]["progress_timeout_s"] = 1
        self.fake("slow_progress")
        st = self.wait(self.router.plan("umarell_jam", options={"fallback": False})["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertEqual(st["export"]["jam"]["progress"]["phase"], "step 7")

    # ------------------------------------------------------------ turns on one cell
    def test_turns_with_u3_only_while_the_fallback_is_on(self):
        tasks = self.cell / R.TASKS_DIR
        tasks.mkdir(parents=True, exist_ok=True)
        (tasks / "slow.json").write_text('{"operations": [{"id": "SLOW1"}]}')
        for rel in ("agentic_planner/config/robots.yaml", "agentic_planner/config/obstacles.yaml"):   # Prepare Cell's
            (self.cell / rel).parent.mkdir(parents=True, exist_ok=True)
            (self.cell / rel).write_text("x: 1\n")
        self.fake("ok")
        u3 = self.router.plan("umarell_u3", task="slow")
        self.assertEqual(u3["status"], "planning")
        free = self.router.plan("umarell_jam", task="slow", options={"fallback": False})
        turn = self.router.plan("umarell_jam", task="slow")
        self.assertEqual(free["status"], "planning")
        self.assertEqual(turn["status"], "queued")
        st = self.router.plan_status(turn["job_id"])
        if st["status"] == "queued":
            self.assertIn("umarell_jam", st.get("waiting_for", ""))
        for r in (u3, free, turn):
            st = self.wait(r["job_id"])
            self.assertEqual(st["status"], "done", st)

    def test_no_s3_by_default(self):
        """No shared cache location here (FACTORY_OS_JAM_CACHE_S3=0) and the save is opt-in: nothing is pulled or
        saved (test_catalogue_cache.py: the save itself)."""
        self.fake("ok")
        st = self.wait(self.router.plan("umarell_jam")["job_id"])
        self.assertEqual(st["status"], "done", st)
        self.assertNotIn("catalogue_push", st["export"]["jam"])
        self.assertIn("no shared cache location", st["export"]["jam"]["cache_pull"])


if __name__ == "__main__":
    unittest.main()
