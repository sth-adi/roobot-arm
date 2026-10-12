"""A short id of the dashboard code on disk, so a running hub can be told apart from the files that are there now."""
import hashlib
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def code_stamp():
    h = hashlib.sha1()
    for base in ("dashboard", "sim", "emu"):
        for d, _, files in sorted(os.walk(os.path.join(ROOT, base))):
            if "__pycache__" in d:
                continue
            for f in sorted(files):
                if f.endswith((".py", ".js", ".css", ".html", ".png", ".cpp", ".h")):
                    p = os.path.join(d, f)
                    try:
                        h.update(("%s:%d" % (os.path.relpath(p, ROOT), os.stat(p).st_size)).encode())
                    except OSError:
                        pass
    return h.hexdigest()[:7]
