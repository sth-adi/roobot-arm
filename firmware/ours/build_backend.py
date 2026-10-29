"""Build "our" backend: Sam's backend (firmware/sam/backend, untouched) + our patch + our checked planner.

    python firmware/ours/build_backend.py OUT_DIR

OUT_DIR must not exist. Run it with `python main.py` from inside OUT_DIR (port 8000), or build a Docker image from it.
What is added (see backend-safe-planner.patch): every move is checked before it is sent (reachable, inside the calibrated joint limits, no joint jump),
joints never exceed MAX_JOINT_DPS, a refused move says why, POST /target is checked against where the arm is now, and POST /hold stops wanting to go anywhere.
motion/safe_planner.py is dashboard/planner.py, copied so there is one source.
"""
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent


def build(out):
    out = Path(out)
    if out.exists():
        raise SystemExit("%s already exists" % out)
    shutil.copytree(ROOT / "firmware" / "sam" / "backend", out, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    patch = (ROOT / "firmware" / "ours" / "backend-safe-planner.patch").read_bytes()
    r = subprocess.run(["git", "apply", "-p1", "--whitespace=nowarn", "-"], input=patch, cwd=out, capture_output=True)
    if r.returncode:
        shutil.rmtree(out, ignore_errors=True)
        raise SystemExit("could not apply the patch to Sam's backend (has it changed?):\n" + r.stderr.decode("utf-8", "replace"))
    shutil.copyfile(ROOT / "dashboard" / "planner.py", out / "motion" / "safe_planner.py")
    return out


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    print("built", build(sys.argv[1]))
