#!/usr/bin/env python3
"""Live controller viewer: draws the Xbox pad + a packet log of every move sent.

Run:  python controller_gui.py [--url http://localhost:8000/input] [--no-send] [--index 0]

Left: the controller (sticks, triggers, bumpers, face buttons, d-pad light up live).
Right: a log of each state change, shown as the raw HTTP packet that goes to the server.
"""
import argparse
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
import urllib.error
from urllib.parse import urlparse

import controller_client as cc

BG, BODY, IDLE, ACTIVE = "#1e1f24", "#34363d", "#50535c", "#3ddc84"
COLORS = {"a": "#3ddc84", "b": "#ff5252", "x": "#4aa3ff", "y": "#ffd23f"}
AXIS_KEYS = ("lx", "ly", "rx", "ry", "lt", "rt")


def describe(prev, cur):
    """Human-readable list of what changed between two states."""
    if prev is None:
        return ["initial state"]
    out = []
    for k in AXIS_KEYS:
        if abs(cur[k] - prev[k]) >= 0.01:
            out.append(f"{k.upper()}={cur[k]:+.2f}")
    for k, v in cur["buttons"].items():
        if v != prev["buttons"][k]:
            out.append(f"{k.upper()} {'pressed' if v else 'released'}")
    if cur["dpad"] != prev["dpad"]:
        out.append(f"DPAD={cur['dpad']}")
    return out


def raw_packet(url, body):
    u = urlparse(url)
    return (f"POST {u.path or '/'} HTTP/1.1\n"
            f"Host: {u.netloc}\n"
            f"Content-Type: application/json\n"
            f"Content-Length: {len(body)}\n\n{body}")


class App:
    def __init__(self, root, js, pygame, url, send):
        self.root, self.js, self.pg, self.url, self.send = root, js, pygame, url, send
        self.prev = None
        self.seq = 0
        self.last_sent = 0.0
        self.status = "not sending" if not send else "connecting..."
        self.q = queue.Queue()
        root.title(f"Robot arm controller - {js.get_name()}")
        root.configure(bg=BG)

        self.cv = tk.Canvas(root, width=640, height=430, bg=BG, highlightthickness=0)
        self.cv.pack(side="left", padx=8, pady=8)
        right = tk.Frame(root, bg=BG)
        right.pack(side="right", fill="both", expand=True, padx=(0, 8), pady=8)
        self.status_lbl = tk.Label(right, text="", bg=BG, fg="#aaa", anchor="w", font=("Consolas", 10))
        self.status_lbl.pack(fill="x")
        frame = tk.Frame(right, bg=BG)
        frame.pack(fill="both", expand=True)
        self.log = tk.Text(frame, width=62, bg="#14151a", fg="#d8d8d8", font=("Consolas", 9),
                           wrap="none", state="disabled", relief="flat")
        sb = tk.Scrollbar(frame, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        self.log.tag_configure("hdr", foreground=ACTIVE)
        self.log.tag_configure("move", foreground="#ffd23f")
        self.log.tag_configure("pkt", foreground="#8a8f9c")
        tk.Button(right, text="Clear log", command=self.clear_log, bg="#2b2d34", fg="#ddd",
                  relief="flat").pack(anchor="e", pady=(4, 0))

        self.build_pad()
        threading.Thread(target=self.sender, daemon=True).start()
        self.tick()

    # ---- drawing -----------------------------------------------------------
    def build_pad(self):
        c = self.cv
        # triggers / bumpers
        self.trig = {}
        for name, x in (("lt", 90), ("rt", 470)):
            c.create_rectangle(x, 10, x + 80, 40, outline=IDLE, fill=BG)
            self.trig[name] = (c.create_rectangle(x, 10, x + 1, 40, outline="", fill=ACTIVE), x)
            c.create_text(x + 40, 25, text=name.upper(), fill="#ccc", font=("Segoe UI", 9, "bold"))
        self.bump = {}
        for name, x in (("lb", 70), ("rb", 450)):
            self.bump[name] = c.create_rectangle(x, 50, x + 120, 75, fill=IDLE, outline="")
            c.create_text(x + 60, 62, text=name.upper(), fill="white", font=("Segoe UI", 9, "bold"))
        # body
        c.create_oval(30, 85, 610, 400, fill=BODY, outline="#44464f", width=2)
        # sticks
        self.stick = {}
        for name, (cx, cy) in (("l", (170, 170)), ("r", (400, 290))):
            c.create_oval(cx - 45, cy - 45, cx + 45, cy + 45, fill="#25262b", outline=IDLE, width=2)
            dot = c.create_oval(cx - 18, cy - 18, cx + 18, cy + 18, fill=IDLE, outline="")
            self.stick[name] = (dot, cx, cy)
        self.sbtn = {"ls": self.stick["l"][0], "rs": self.stick["r"][0]}
        # d-pad
        self.dp = {}
        px, py = 260, 290
        for key, (dx, dy, w, h) in {"up": (0, -26, 22, 30), "down": (0, 26, 22, 30),
                                    "left": (-26, 0, 30, 22), "right": (26, 0, 30, 22)}.items():
            self.dp[key] = c.create_rectangle(px + dx - w / 2, py + dy - h / 2, px + dx + w / 2,
                                              py + dy + h / 2, fill=IDLE, outline="")
        # face buttons
        self.face = {}
        fx, fy = 475, 175
        for k, (dx, dy) in {"y": (0, -34), "a": (0, 34), "x": (-34, 0), "b": (34, 0)}.items():
            self.face[k] = c.create_oval(fx + dx - 17, fy + dy - 17, fx + dx + 17, fy + dy + 17,
                                         fill=IDLE, outline="")
            c.create_text(fx + dx, fy + dy, text=k.upper(), fill="white", font=("Segoe UI", 11, "bold"))
        # back / start
        self.mid = {}
        for k, x in (("back", 290), ("start", 350)):
            self.mid[k] = c.create_oval(x - 11, 175, x + 11, 197, fill=IDLE, outline="")
            c.create_text(x, 210, text=k.upper(), fill="#999", font=("Segoe UI", 7))
        self.axis_txt = c.create_text(320, 415, fill="#888", font=("Consolas", 9), text="")

    def paint(self, s):
        c = self.cv
        for name, (item, x) in self.trig.items():
            c.coords(item, x, 10, x + 1 + 79 * s[name], 40)
        for k in ("lb", "rb"):
            c.itemconfig(self.bump[k], fill=ACTIVE if s["buttons"][k] else IDLE)
        for name, kx, ky, btn in (("l", "lx", "ly", "ls"), ("r", "rx", "ry", "rs")):
            dot, cx, cy = self.stick[name]
            x, y = s[kx], s[ky]
            m = math.hypot(x, y)
            if m > 1:
                x, y = x / m, y / m
            px, py = cx + x * 27, cy + y * 27
            c.coords(dot, px - 18, py - 18, px + 18, py + 18)
            c.itemconfig(dot, fill=ACTIVE if s["buttons"][btn] else ("#7fb8ff" if m else IDLE))
        for k, item in self.face.items():
            c.itemconfig(item, fill=COLORS[k] if s["buttons"][k] else IDLE)
        for k, item in self.mid.items():
            c.itemconfig(item, fill=ACTIVE if s["buttons"][k] else IDLE)
        dx, dy = s["dpad"]
        on = {"up": dy == 1, "down": dy == -1, "left": dx == -1, "right": dx == 1}
        for k, item in self.dp.items():
            c.itemconfig(item, fill=ACTIVE if on[k] else IDLE)
        c.itemconfig(self.axis_txt, text="  ".join(f"{k}={s[k]:+.2f}" for k in AXIS_KEYS))

    # ---- logging / sending -------------------------------------------------
    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def write(self, parts):
        self.log.configure(state="normal")
        for text, tag in parts:
            self.log.insert("end", text, tag)
        n = int(self.log.index("end-1c").split(".")[0])
        if n > 1500:
            self.log.delete("1.0", f"{n - 1500}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def sender(self):
        while True:
            state = self.q.get()
            while not self.q.empty():  # only the newest state matters
                state = self.q.get_nowait()
            try:
                cc.post(self.url, state)
                self.status = f"sending to {self.url}  [OK]"
            except (urllib.error.URLError, OSError) as e:
                self.status = f"server unreachable: {e}"

    def tick(self):
        s = cc.read_state(self.js, self.pg)
        self.paint(s)
        now = time.time()
        changed = s != self.prev
        if changed:
            body = json.dumps(s)
            self.seq += 1
            moves = describe(self.prev, s)
            ts = time.strftime("%H:%M:%S") + f".{int(now * 1000) % 1000:03d}"
            self.write([(f"#{self.seq} {ts}  ", "hdr"), ("  ".join(moves) + "\n", "move"),
                        (raw_packet(self.url, body) + "\n\n", "pkt")])
            self.prev = s
        if self.send and (changed or now - self.last_sent >= cc.HEARTBEAT_S):
            self.q.put(s)
            self.last_sent = now
        self.status_lbl.config(text=self.status)
        self.root.after(int(1000 / cc.SEND_HZ), self.tick)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="http://localhost:8000/input")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--no-send", action="store_true", help="view only, don't POST to the server")
    args = p.parse_args()

    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.index:
        raise SystemExit("No controller found. Plug one in.")
    js = pygame.joystick.Joystick(args.index)
    js.init()

    root = tk.Tk()
    App(root, js, pygame, args.url, not args.no_send)
    root.mainloop()


if __name__ == "__main__":
    main()
