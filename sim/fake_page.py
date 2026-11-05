"""A stand-in for Terra's index.html while the real format is unknown: builds the requests the page might send and checks the answers have the
contract's shape. It never touches an arm. Used by sim/test_contract.py; swap the shapes here (and in dashboard/contract.py) when the real ones are known."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "dashboard"))
import contract as C  # noqa: E402

REQUESTS = {
    "typed":        {"v": 1, "id": "a1", "text": "get me the second key"},
    "spoken":       {"v": 1, "id": "a2", "text": "put key three back please"},
    "button_fetch": {"v": 1, "id": "a3", "action": "fetch", "key": 2},
    "button_ret":   {"v": 1, "id": "a4", "action": "return", "key": 1},
    "button_home":  {"v": 1, "id": "a5", "action": "home"},
    "button_stop":  {"v": 1, "id": "a6", "action": "stop"},
    "legacy_fetch": {"command": "request_key"},
    "legacy_stop":  {"command": "stop"},
}


def lifecycle(job="7f3a", steps=3):
    """The status packets a page should see for one successful job: queued, each step running, done."""
    out = [C.job_status(job, "queued")]
    out += [C.job_status(job, "running", step=i, of=steps, message="step %d" % i) for i in range(1, steps + 1)]
    out.append(C.job_status(job, "done", step=steps, of=steps))
    return out
