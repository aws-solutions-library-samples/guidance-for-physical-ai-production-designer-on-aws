"""umarell_u3: which plain umarell commit U3 patches, and how its root is laid out (standard library only).

    python3 -I -m unittest discover -s deploy/planner-router/tests
"""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import umarell_u3 as U  # noqa: E402


def checkout(path: Path, new_layout: bool, sha: str, dc60669: bool = False) -> Path:
    (path / "agents" / "trajectory" / ("backends" if new_layout else "tools")).mkdir(parents=True)
    if dc60669:
        (path / "utils").mkdir()
        (path / "utils" / "weld_surface.py").write_text("")
        (path / "agents" / "trajectory" / "plan_jobs.py").write_text("")
    (path / ".umarell.sha256").write_text(sha + "\n")
    return path


def cell_planner(path: Path) -> Path:
    (path / "umarell_fast" / "u3").mkdir(parents=True)
    (path / "umarell_fast" / "u3" / "__main__.py").write_text("")
    for d in ("patches", "patches_76ea13a"):
        (path / "docs" / "upstream" / d).mkdir(parents=True)
        (path / "docs" / "upstream" / d / "01_x.diff").write_text(d)
    return path


class BaseTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)

    def test_without_a_base_checkout_u3_patches_the_checkout_itself(self):
        umarell = checkout(self.tmp / "umarell", False, "a")
        self.assertEqual(U.u3_source(umarell), umarell)
        self.assertEqual(U.base_of(umarell), "eccb1bc")

    def test_a_patched_checkout_hands_u3_its_plain_base(self):
        umarell = checkout(self.tmp / "umarell", True, "patched")
        base = checkout(self.tmp / "umarell-base", True, "plain")
        self.assertEqual(U.u3_source(umarell), base.resolve())
        self.assertEqual(U.base_of(base), "76ea13a")

    def test_08d5a39_is_told_from_dc60669_and_linked_under_its_own_name(self):
        src = cell_planner(self.tmp / "src")
        root = self.tmp / "root"
        base = checkout(self.tmp / "umarell-base", True, "plain", dc60669=True)
        tools = base / "agents" / "trajectory" / "tools"
        tools.mkdir(parents=True, exist_ok=True)
        (tools / "fastgeom.py").write_text("")
        (tools / "plan_memo.py").write_text("")
        self.assertEqual(U.base_of(base), "08d5a39")
        U.sync_root(src, base, root)
        tp = root / "third_party"
        self.assertEqual((tp / "sample-umarell-08d5a39").resolve(), base.resolve())
        self.assertFalse((tp / "sample-umarell-dc60669").exists())

    def _08d5a39_layout(self, name: str, compacting: bool, default_cell: str) -> Path:
        base = checkout(self.tmp / name, True, "plain", dc60669=True)
        tools = base / "agents" / "trajectory" / "tools"
        tools.mkdir(parents=True, exist_ok=True)
        (tools / "fastgeom.py").write_text("")
        (tools / "plan_memo.py").write_text("")
        (base / "agents" / "trajectory" / "context_compaction.py").write_text(
            "def compact_for_model(state):\n    pass\n"
            + ("\n\ndef compacting_prompt(system_prompt: str):\n    pass\n" if compacting else ""))
        (base / "agents" / "cell_defaults.py").write_text(f'DEFAULT_CELL_ID = "{default_cell}"\n')
        return base

    def test_the_public_8a02db9_is_told_from_08d5a39_and_linked_under_its_own_name(self):
        src = cell_planner(self.tmp / "src")
        (src / "docs" / "upstream" / "patches_8a02db9").mkdir(parents=True)
        (src / "docs" / "upstream" / "patches_8a02db9" / "series.txt").write_text("01_x.diff\n")
        root = self.tmp / "root"
        base = self._08d5a39_layout("umarell-base", True, "welding_cell_2_small")
        self.assertEqual(U.base_of(base), "8a02db9")
        U.sync_root(src, base, root)
        tp = root / "third_party"
        self.assertEqual((tp / "sample-umarell-8a02db9").resolve(), base.resolve())
        self.assertFalse((tp / "sample-umarell-08d5a39").exists() or (tp / "sample-umarell-08d5a39").is_symlink(),
                         "an 08d5a39 link to the public tree would let U3 apply set v10 (with 28) to it")
        self.assertTrue((root / "docs" / "upstream" / "patches_8a02db9" / "series.txt").is_file())

    def test_08d5a39_stays_08d5a39_plain_or_with_fix_28(self):
        # plain 08d5a39: the pre_model_hook, welding_cell_2x; with fix 28 applied: the prompt callable, welding_cell_2x
        self.assertEqual(U.base_of(self._08d5a39_layout("plain", False, "welding_cell_2x")), "08d5a39")
        self.assertEqual(U.base_of(self._08d5a39_layout("fixed", True, "welding_cell_2x")), "08d5a39")
        # an 08d5a39 layout whose agents/ files are missing is not taken for the public tree
        bare = self._08d5a39_layout("bare", True, "welding_cell_2_small")
        (bare / "agents" / "cell_defaults.py").unlink()
        self.assertEqual(U.base_of(bare), "08d5a39")

    def test_dc60669_is_told_from_76ea13a_and_linked_under_its_own_name(self):
        src = cell_planner(self.tmp / "src")
        root = self.tmp / "root"
        base = checkout(self.tmp / "umarell-base", True, "plain", dc60669=True)
        self.assertEqual(U.base_of(base), "dc60669")
        U.sync_root(src, base, root)
        tp = root / "third_party"
        self.assertEqual((tp / "sample-umarell-dc60669").resolve(), base.resolve())
        for other in ("sample-umarell", "sample-umarell-76ea13a"):
            self.assertFalse((tp / other).exists() or (tp / other).is_symlink(),
                             "a 76ea13a link to a dc60669 tree would let U3 apply set v8 to it")

    def test_the_root_links_only_the_sources_own_base_and_copies_every_patch_folder(self):
        src = cell_planner(self.tmp / "src")
        root = self.tmp / "root"
        base = checkout(self.tmp / "umarell-base", True, "plain")
        U.sync_root(src, base, root)
        tp = root / "third_party"
        self.assertEqual((tp / "sample-umarell-76ea13a").resolve(), base.resolve())
        self.assertFalse((tp / "sample-umarell").exists() or (tp / "sample-umarell").is_symlink(),
                         "an eccb1bc link to a 76ea13a tree would let U3 apply the old patches to it")
        for d in ("patches", "patches_76ea13a"):
            self.assertEqual((root / "docs" / "upstream" / d / "01_x.diff").read_text(), d)

    def test_switching_base_relinks_and_restamps(self):
        src = cell_planner(self.tmp / "src")
        root = self.tmp / "root"
        old = checkout(self.tmp / "old", False, "e")
        U.sync_root(src, old, root)
        self.assertTrue((root / "third_party" / "sample-umarell").is_symlink())
        new = checkout(self.tmp / "new", True, "n")
        U.sync_root(src, new, root)
        self.assertFalse((root / "third_party" / "sample-umarell").is_symlink())
        self.assertEqual((root / "third_party" / "sample-umarell-76ea13a").resolve(), new.resolve())

    def test_a_new_patch_folder_changes_the_stamp(self):
        src = cell_planner(self.tmp / "src")
        umarell = checkout(self.tmp / "umarell", True, "x")
        before = U.stamp_of(src, umarell)
        (src / "docs" / "upstream" / "patches_76ea13a" / "02_y.diff").write_text("y")
        self.assertNotEqual(U.stamp_of(src, umarell), before)


if __name__ == "__main__":
    unittest.main()
