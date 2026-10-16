"""The real arm's calibration (joint limits and homes), read from the end server the arm talks to.

Sam's firmware trains the calibration on the arm and keeps it on the end server (GET /calibration), so that server, not this PC, knows which
joints are reversed and where the real limits and homes are. The hub's virtual arm and the USB sketch's inverse kinematics must use the same
numbers or the arm is sent to the wrong place (a reversed joint is the usual way: up becomes down). This only ever READS from that server.
The last good answer is cached in .hub_calibration.json (git-ignored) so the hub still has it when the server cannot be reached.
"""
import json
import urllib.parse
import urllib.request
from pathlib import Path

CACHE = Path(__file__).resolve().parent.parent / ".hub_calibration.json"


def base_url(forward_url):
    """scheme://host of FORWARD_URL (the controller input address): where /calibration lives."""
    u = urllib.parse.urlparse((forward_url or "").strip())
    if u.scheme not in ("http", "https") or not u.netloc:
        raise ValueError("no usable FORWARD_URL in .env")
    return "%s://%s" % (u.scheme, u.netloc)


def normalise(d):
    """({"jointLimits": [[min, max] x4], "jointHomes" or "joint0s": [x4]}) -> (limits, homes) as whole degrees 0..180. ValueError otherwise.
    A reversed pair (min > max) is kept: it marks an inverted joint."""
    if not isinstance(d, dict):
        raise ValueError("the calibration is not an object")
    limits = d.get("jointLimits")
    homes = d.get("jointHomes", d.get("joint0s"))
    ok = isinstance(limits, list) and len(limits) == 4 and isinstance(homes, list) and len(homes) == 4
    ok = ok and all(isinstance(p, list) and len(p) == 2 for p in limits)
    if ok:
        flat = [v for p in limits for v in p] + list(homes)
        ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and (isinstance(v, int) or v.is_integer()) and 0 <= v <= 180 for v in flat)
    if not ok:
        raise ValueError("the calibration needs four [min, max] limits and four homes, whole degrees 0 to 180")
    if any(int(p[0]) == int(p[1]) for p in limits):
        raise ValueError("a joint's two limits must differ")
    return [[int(p[0]), int(p[1])] for p in limits], [int(h) for h in homes]


def fetch(forward_url, timeout=6.0):
    """GET <end server>/calibration -> (limits, homes). Raises on any failure; changes nothing on the server."""
    req = urllib.request.Request(base_url(forward_url) + "/calibration", headers={"Accept": "application/json", "User-Agent": "roobot-arm/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return normalise(json.loads(r.read(20000)))


def save_cache(limits, homes, path=CACHE):
    try:
        Path(path).write_text(json.dumps({"jointLimits": limits, "jointHomes": homes}), encoding="utf-8")
    except OSError:
        pass


def load_cache(path=CACHE):
    return normalise(json.loads(Path(path).read_text(encoding="utf-8")))
