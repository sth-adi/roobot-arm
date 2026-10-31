"""Build "our" backend: Sam's backend (firmware/sam/backend, untouched) + our patch + our checked planner.

    python firmware/ours/build_backend.py OUT_DIR

OUT_DIR must not exist. Run it with `python main.py` from inside OUT_DIR (port 8000, or BACKEND_PORT; this computer only, or BACKEND_HOST=0.0.0.0).
What is added (see backend-safe-planner.patch): every move is checked before it is sent (reachable, inside the calibrated joint limits, no joint jump),
joints never exceed MAX_JOINT_DPS, a refused move says why, POST /target is checked against where the arm is now, GET /position says where it is,
and POST /hold stops wanting to go anywhere. motion/safe_planner.py is dashboard/planner.py, copied so there is one source.
The patch is applied here in plain Python, so neither git nor patch needs to be installed.
"""
import hashlib
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
SAM = ROOT / "firmware" / "sam" / "backend"
PATCH = ROOT / "firmware" / "ours" / "backend-safe-planner.patch"
PATCH_LIVE = ROOT / "firmware" / "ours" / "backend-live-packets.patch"      # applied after PATCH: short constant-speed packets while targets stream in, one open connection
PLANNER = ROOT / "dashboard" / "planner.py"
_HUNK = re.compile(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@")


class PatchError(Exception):
    pass


def apply_patch(root, text):
    """Apply a unified diff (as `diff -ruN a b` writes it) to the files under root. Context must match exactly, else PatchError and nothing is half-written."""
    root = Path(root)
    lines = text.splitlines()
    files, i = {}, 0
    while i < len(lines):
        if not lines[i].startswith("--- "):
            i += 1
            continue
        name = lines[i][4:].split("\t")[0].strip()
        if not lines[i + 1].startswith("+++ "):
            raise PatchError("malformed patch near line %d" % (i + 1))
        name = name[2:] if name[:2] in ("a/", "b/") else name
        if ".." in Path(name).parts or Path(name).is_absolute():
            raise PatchError("unsafe path in patch: %s" % name)
        i += 2
        hunks = []
        while i < len(lines) and lines[i].startswith("@@"):
            m = _HUNK.match(lines[i])
            if not m:
                raise PatchError("malformed hunk header: %s" % lines[i])
            old_n = int(m.group(2)) if m.group(2) is not None else 1
            new_n = int(m.group(4)) if m.group(4) is not None else 1
            i += 1
            body, o, n = [], 0, 0
            while i < len(lines) and (o < old_n or n < new_n):
                tag, rest = lines[i][:1], lines[i][1:]
                if tag == "\\":
                    i += 1
                    continue
                if tag not in (" ", "+", "-", ""):
                    raise PatchError("unexpected line in hunk: %s" % lines[i])
                if tag == "":
                    tag, rest = " ", ""                       # an empty context line whose space was stripped
                body.append((tag, rest))
                o += tag in " -"
                n += tag in " +"
                i += 1
            hunks.append((int(m.group(1)), body))
        files[name] = hunks
    out = {}
    for name, hunks in files.items():
        path = root / name
        old = path.read_text(encoding="utf-8").splitlines() if path.exists() else []
        new, pos = [], 0
        for start, body in hunks:
            at = max(start - 1, 0) if start else 0
            if at < pos:
                raise PatchError("%s: overlapping hunks" % name)
            new += old[pos:at]
            pos = at
            for tag, rest in body:
                if tag in " -":
                    if pos >= len(old) or old[pos].rstrip("\r") != rest.rstrip("\r"):
                        raise PatchError("%s: the file is not what the patch expects (line %d). Has Sam's backend changed?" % (name, pos + 1))
                    if tag == " ":
                        new.append(old[pos])
                    pos += 1
                else:
                    new.append(rest)
        new += old[pos:]
        out[path] = "\n".join(new) + "\n"
    for path, content in out.items():                         # only now write: a failed patch leaves nothing behind
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8", newline="\n")


def source_hash():
    """Changes whenever anything the built backend is made from changes, so a launcher knows when to build again."""
    h = hashlib.sha1()
    for p in sorted(SAM.rglob("*")):
        if p.is_file() and "__pycache__" not in p.parts:
            h.update(p.relative_to(SAM).as_posix().encode() + p.read_bytes())
    h.update(PATCH.read_bytes() + PATCH_LIVE.read_bytes() + PLANNER.read_bytes() + Path(__file__).read_bytes())
    return h.hexdigest()[:12]


def build(out):
    out = Path(out)
    if out.exists():
        raise SystemExit("%s already exists" % out)
    shutil.copytree(SAM, out, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    try:
        apply_patch(out, PATCH.read_text(encoding="utf-8"))
        apply_patch(out, PATCH_LIVE.read_text(encoding="utf-8"))
    except PatchError as e:
        shutil.rmtree(out, ignore_errors=True)
        raise SystemExit("could not apply the patch to Sam's backend: %s" % e)
    shutil.copyfile(PLANNER, out / "motion" / "safe_planner.py")
    (out / ".source_hash").write_text(source_hash())
    return out


def ensure(out):
    """Build into out unless it is already built from the current sources. Returns out."""
    out = Path(out)
    if (out / ".source_hash").exists() and (out / ".source_hash").read_text() == source_hash():
        return out
    if out.exists():
        shutil.rmtree(out)
    return build(out)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    print("built", build(sys.argv[1]))
