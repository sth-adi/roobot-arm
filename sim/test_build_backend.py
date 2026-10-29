"""firmware/ours/build_backend.py: the plain-Python patch applier, and rebuilding only when the sources change."""
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location("build_backend", ROOT / "firmware" / "ours" / "build_backend.py")
BB = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(BB)


class PatchApplier(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_same_result_as_git_apply(self):
        if not shutil.which("git"):
            self.skipTest("git is not installed")
        a, b = self.root / "py", self.root / "git"
        for d in (a, b):
            shutil.copytree(BB.SAM, d, ignore=shutil.ignore_patterns("__pycache__"))
        BB.apply_patch(a, BB.PATCH.read_text(encoding="utf-8"))
        subprocess.run(["git", "apply", "-p1", "--whitespace=nowarn", "-"], input=BB.PATCH.read_bytes(), cwd=b, check=True)
        for p in b.rglob("*"):
            if p.is_file():
                self.assertEqual((a / p.relative_to(b)).read_bytes(), p.read_bytes(), p.name)

    def test_a_changed_file_is_refused_and_nothing_is_written(self):
        d = self.root / "x"
        shutil.copytree(BB.SAM, d, ignore=shutil.ignore_patterns("__pycache__"))
        g = d / "GlobalVariables.py"
        g.write_text(g.read_text().replace("MAX_SPEED = 50.0", "MAX_SPEED = 40.0"))
        handlers_before = (d / "api" / "handlers.py").read_bytes()
        with self.assertRaises(BB.PatchError) as c:
            BB.apply_patch(d, BB.PATCH.read_text(encoding="utf-8"))
        self.assertIn("Has Sam's backend changed", str(c.exception))
        self.assertEqual((d / "api" / "handlers.py").read_bytes(), handlers_before)      # another file in the same patch was not half-applied

    def test_windows_line_endings_in_the_checkout_do_not_matter(self):
        d = self.root / "crlf"
        shutil.copytree(BB.SAM, d, ignore=shutil.ignore_patterns("__pycache__"))
        for p in d.rglob("*.py"):
            p.write_bytes(p.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
        BB.apply_patch(d, BB.PATCH.read_text(encoding="utf-8"))
        self.assertIn("MAX_JOINT_DPS", (d / "GlobalVariables.py").read_text())

    def test_unsafe_paths_in_a_patch_are_refused(self):
        evil = "--- a/../x.py\n+++ b/../x.py\n@@ -0,0 +1 @@\n+print(1)\n"
        with self.assertRaises(BB.PatchError):
            BB.apply_patch(self.root, evil)

    def test_new_file_patch(self):
        BB.apply_patch(self.root, "--- a/new.txt\n+++ b/new.txt\n@@ -0,0 +1,2 @@\n+one\n+two\n")
        self.assertEqual((self.root / "new.txt").read_text(), "one\ntwo\n")


class Build(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.out = Path(self.tmp.name) / "backend"

    def test_build_has_everything_and_listens_on_this_computer_only(self):
        BB.build(self.out)
        for f in ("main.py", "motion/safe_planner.py", "api/handlers.py", ".source_hash"):
            self.assertTrue((self.out / f).exists(), f)
        env = dict(os.environ)
        env.pop("BACKEND_HOST", None)
        env.pop("BACKEND_PORT", None)
        r = subprocess.run([sys.executable, "-c", "import GlobalVariables as g; print(g.SERVER_HOST, g.SERVER_PORT)"], cwd=self.out, env=env, capture_output=True, text=True)
        self.assertEqual(r.stdout.split(), ["127.0.0.1", "8000"])
        env.update(BACKEND_HOST="0.0.0.0", BACKEND_PORT="8123")
        r = subprocess.run([sys.executable, "-c", "import GlobalVariables as g; print(g.SERVER_HOST, g.SERVER_PORT)"], cwd=self.out, env=env, capture_output=True, text=True)
        self.assertEqual(r.stdout.split(), ["0.0.0.0", "8123"])

    def test_ensure_builds_once_and_again_only_when_something_changed(self):
        BB.ensure(self.out)
        marker = self.out / "marker.txt"
        marker.write_text("still here")
        BB.ensure(self.out)
        self.assertTrue(marker.exists())                                    # up to date: left alone
        (self.out / ".source_hash").write_text("stale")
        BB.ensure(self.out)
        self.assertFalse(marker.exists())                                   # sources changed: rebuilt
        self.assertEqual((self.out / ".source_hash").read_text(), BB.source_hash())

    def test_build_refuses_an_existing_folder(self):
        self.out.mkdir()
        with self.assertRaises(SystemExit):
            BB.build(self.out)


if __name__ == "__main__":
    unittest.main()
