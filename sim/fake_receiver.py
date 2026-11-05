"""A stand-in for Sam's Python code speaking the PLACEHOLDER JSON of dashboard/lowlevel.JsonSender: checks every step message has the right shape and
pretends to move for a few polls. No arm, no network. Used by sim/test_lowlevel.py."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))


class FakeReceiver:
    def __init__(self, busy_polls=2, refuse_at=None, fail_at=None, never_finish=False):
        self.busy_polls, self.refuse_at, self.fail_at, self.never_finish = busy_polls, refuse_at, fail_at, never_finish
        self.steps, self.stops, self.left, self.done, self.err = [], 0, 0, 0, None

    def __call__(self, method, path, body=None):
        if (method, path) == ("POST", "/step"):
            m = body
            assert m["v"] == 1 and m["type"] == "move" and set(m["target"]) == {"x", "y", "z", "pitch", "speed"} and m["magnet"] in (None, "on", "off"), m
            if m["step"] == self.refuse_at:
                return {"accepted": False, "error": {"code": "UNREACHABLE", "title": "Out of reach", "detail": "step %d" % m["step"]}}
            self.steps.append(m)
            self.left = self.busy_polls
            self.err = {"code": "ARM_STALLED", "title": "Joint stalled", "detail": ""} if m["step"] == self.fail_at else None
            return {"accepted": True}
        if (method, path) == ("GET", "/status"):
            if self.err:
                return {"state": "error", "error": self.err}
            if self.never_finish:
                return {"state": "moving", "step": self.done}
            if self.left > 0:
                self.left -= 1
                return {"state": "moving", "step": self.done}
            self.done = len(self.steps)
            return {"state": "idle", "step": self.done}
        if (method, path) == ("POST", "/stop"):
            self.stops += 1
            return {}
        raise AssertionError((method, path))
