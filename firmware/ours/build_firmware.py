"""Build "our" ESP32 firmware: Sam's 29 Oct firmware (firmware/sam/firmware, untouched) + firmware-local-backend.patch + a generated Secrets.h.

    python firmware/ours/build_firmware.py OUT_DIR HOST:PORT WIFI_SSID [WIFI_PASSWORD [WIFI_USER]]

OUT_DIR must not exist; it gets src/ (his sources, patched) ready for PlatformIO or the hub's arduino-cli build.
The only change to his code: the backend is reached over plain http at HOST:PORT (the backend of this repo, firmware/ours/build_backend.py, on this
computer's address) instead of https on port 443. Everything he sends and expects (/UpdateMovement, /movement-complete, /calibration, /status,
/controller) is unchanged, so the arm still moves only when something POSTs a /target to that backend: here, the dashboard.
"""
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_backend import PatchError, apply_patch  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent.parent
SAM = ROOT / "firmware" / "sam" / "firmware" / "src"
PATCH = ROOT / "firmware" / "ours" / "firmware-local-backend.patch"


def _c(s):
    return str(s).replace("\\", "\\\\").replace('"', '\\"')


def build(out, host, ssid, password="", user=""):
    out = Path(out)
    if out.exists():
        raise SystemExit("%s already exists" % out)
    shutil.copytree(SAM, out / "src", ignore=shutil.ignore_patterns("SecretsExample.h"))
    try:
        apply_patch(out, PATCH.read_text(encoding="utf-8"))
    except PatchError as e:
        shutil.rmtree(out, ignore_errors=True)
        raise SystemExit("could not apply the patch to Sam's firmware: %s" % e)
    (out / "src" / "config" / "Secrets.h").write_text(
        '#pragma once\n\n#define BACKEND_PLAIN_HTTP\n\n'
        'const char* WIFI_SSID = "%s";\nconst char* WIFI_User = "%s";\nconst char* WIFI_Password = "%s";\n\n'
        'const char* HOST = "%s";\n' % (_c(ssid), _c(user), _c(password), _c(host)), encoding="utf-8")
    return out


if __name__ == "__main__":
    if not 4 <= len(sys.argv) <= 6:
        sys.exit(__doc__)
    print("built", build(sys.argv[1], sys.argv[2], sys.argv[3], *sys.argv[4:]))
