"""umarell_u3: the catalogue's shared U3 plan caches are pulled before planning (standard library only; a stub
``aws`` and a stub ``umarell_fast.plan_cache``).

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import umarell_u3 as U  # noqa: E402

AWS_STUB = """#!/bin/sh
# aws s3 sync <uri> <dir> ...: copy the fake bucket's *.tgz into <dir>; log the call
echo "$@" >> "$FAKE_LOG"
[ "$1 $2" = "s3 sync" ] || exit 2
cp "$FAKE_BUCKET"/*.tgz "$4"/ 2>/dev/null
exit 0
"""
PLAN_CACHE_STUB = """import os, sys
with open(os.environ["U3_CACHE_DIR"] + "/imported.log", "a") as f:
    f.write(sys.argv[2] + "\\n")
"""


class CachePull(unittest.TestCase):
    def setUp(self):
        self.t = Path(tempfile.mkdtemp())
        (self.t / "bin").mkdir()
        aws = self.t / "bin" / "aws"
        aws.write_text(AWS_STUB)
        aws.chmod(0o755)
        self.bucket = self.t / "bucket"
        self.bucket.mkdir()
        (self.bucket / "body_weld_3.tgz").write_bytes(b"x" * 10)
        self.src = self.t / "src"
        (self.src / "umarell_fast").mkdir(parents=True)
        (self.src / "umarell_fast" / "__init__.py").write_text("")
        (self.src / "umarell_fast" / "plan_cache.py").write_text(PLAN_CACHE_STUB)
        self.cache = self.t / "cache" / "u3"
        self.cache.mkdir(parents=True)
        self.env = self.t / "workspace.env"
        self.env.write_text("FACTORY_OS_STORAGE=s3\nFACTORY_OS_S3_BUCKET=factoryos-data-1-eu-central-1\n"
                            "FACTORY_OS_S3_REGION=eu-central-1\n")
        self.patch = mock.patch.dict(os.environ, {"PATH": f"{self.t / 'bin'}:{os.environ['PATH']}",
                                                  "FAKE_LOG": str(self.t / "aws.log"),
                                                  "FAKE_BUCKET": str(self.bucket)})
        self.patch.start()
        os.environ.pop("FACTORY_OS_U3_CACHE_S3", None)
        self.envp = mock.patch.object(U, "WORKSPACE_ENV", self.env)
        self.envp.start()

    def tearDown(self):
        self.patch.stop()
        self.envp.stop()

    def test_uri_from_the_workspace_env_and_override(self):
        self.assertEqual(U.catalogue_cache_uri(self.env), "s3://factoryos-data-1-eu-central-1/catalogue/u3-plan-cache/")
        local = self.t / "local.env"
        local.write_text("FACTORY_OS_STORAGE=local\n")
        self.assertIsNone(U.catalogue_cache_uri(local))
        with mock.patch.dict(os.environ, {"FACTORY_OS_U3_CACHE_S3": "s3://other/prefix"}):
            self.assertEqual(U.catalogue_cache_uri(self.env), "s3://other/prefix/")
        with mock.patch.dict(os.environ, {"FACTORY_OS_U3_CACHE_S3": "0"}):
            self.assertIsNone(U.catalogue_cache_uri(self.env))

    def test_pull_imports_new_bundles_once(self):
        note = U.pull_catalogue_caches(self.cache, self.src)
        self.assertIn("imported body_weld_3.tgz", note)
        log = (self.cache / "imported.log").read_text().split()
        self.assertEqual([Path(p).name for p in log], ["body_weld_3.tgz"])
        call = (self.t / "aws.log").read_text()
        self.assertIn("s3 sync s3://factoryos-data-1-eu-central-1/catalogue/u3-plan-cache/", call)
        self.assertIn("--region eu-central-1", call)
        # again: nothing new -> no import; a new bundle -> only that one
        self.assertIn("none new", U.pull_catalogue_caches(self.cache, self.src))
        (self.bucket / "body_weld_4x8.tgz").write_bytes(b"y" * 5)
        self.assertIn("imported body_weld_4x8.tgz", U.pull_catalogue_caches(self.cache, self.src))
        self.assertEqual(len((self.cache / "imported.log").read_text().split()), 2)
        done = json.loads((self.cache.parent / "catalogue-pull" / ".imported.json").read_text())
        self.assertEqual(sorted(done), ["body_weld_3.tgz", "body_weld_4x8.tgz"])

    def test_a_failed_sync_never_raises(self):
        (self.t / "bin" / "aws").write_text("#!/bin/sh\necho denied >&2\nexit 1\n")
        self.assertIn("failed", U.pull_catalogue_caches(self.cache, self.src))


class ForgetFailedReplay(unittest.TestCase):
    def test_journal_dropped_only_when_the_export_failed(self):
        d = Path(tempfile.mkdtemp())
        (d / "journal.json").write_text("{}")
        self.assertFalse(U.forget_failed_replay(0, {"dir": str(d)}))
        self.assertTrue((d / "journal.json").exists())
        self.assertTrue(U.forget_failed_replay(1, {"dir": str(d)}))
        self.assertFalse((d / "journal.json").exists())
        self.assertFalse(U.forget_failed_replay(1, None))


if __name__ == "__main__":
    unittest.main()
