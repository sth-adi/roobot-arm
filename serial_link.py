"""Direct USB-serial link to the ESP32 (lowest latency path), used only while its COM port exists.

The port is found by USB ID (CP210x 10C4:EA60, CH340, ESP32-S3) or an explicit name, opened without
toggling DTR/RTS (so the board is NOT reset), and dropped cleanly when the cable is unplugged. While the
port is missing nothing is sent and nothing blocks; it is re-checked about once a second.

Wire format, one ASCII line per update (about 47 bytes, ~4 ms at 115200 baud):
    $lx,ly,rx,ry,lt,rt,buttonmask,dpadx,dpady,seq\\n
buttonmask bits 0..9 = a b x y lb rb back start ls rs.
Mirror mode sends absolute joint positions instead (the virtual arm's joint angles in servo degrees, 0..180):
    @d1,d2,d3,d4,seq\\n
or, when the board runs the inverse kinematics itself (the USB sketch says so: "mirror ik: supported"), the tool-tip pose in mm and degrees:
    #x,y,z,angle,seq\\n
Both position frames end with a checksum "*HH" before the newline: the XOR of every byte between the first character and the "*", as two hex
digits (e.g. #0.00,200.00,100.50,0.00,3*5A). The board refuses a frame without a matching checksum, because a byte lost or garbled on the wire
can still read as a valid but wrong number and swing the arm for a moment.
The board needs the real arm's calibration for that, sent as  !cal min1,max1,...,min4,max4,home1,...,home4  (whole degrees; a reversed pair
marks an inverted joint). It is sent every time the port opens and again if the board reports it has lost it.

Used by controller_gui.py (update() each frame) and by the dashboard hub (service() to keep the port open
and read the board's output, update() only while the output is ARMED).
"""
import re
import threading
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:  # pyserial missing: link stays permanently unavailable
    serial = None

BUTTON_ORDER = ("a", "b", "x", "y", "lb", "rb", "back", "start", "ls", "rs")
ESP32_USB_IDS = {(0x10C4, 0xEA60), (0x1A86, 0x7523), (0x303A, 0x1001)}  # CP210x, CH340, ESP32-S3
RESCAN_S = 1.0
HEARTBEAT_S = 0.1   # resend unchanged state so the ESP32 knows the link is alive
PULSES_RE = re.compile(r"mirror pulses: 0 deg = ([\d.]+) us, 180 deg = ([\d.]+) us")
IK_RE = re.compile(r"mirror ik: supported, calibration (set|missing)")
SETTINGS_RE = re.compile(r"settings: max (?:rate|speed at full stick) ([\d.]+) deg/s.*?range ([\d.]+) deg")


def encode(state, seq):
    mask = sum(1 << i for i, k in enumerate(BUTTON_ORDER) if state["buttons"].get(k))
    dx, dy = state["dpad"]
    return (f"${state['lx']:.3f},{state['ly']:.3f},{state['rx']:.3f},{state['ry']:.3f},"
            f"{state['lt']:.3f},{state['rt']:.3f},{mask},{dx},{dy},{seq}\n").encode("ascii")


def checksum(payload):
    """XOR of the bytes of `payload` (the text after a frame's first character and before its "*"), as two upper-case hex digits."""
    c = 0
    for b in payload.encode("ascii"):
        c ^= b
    return "%02X" % c


def _frame(kind, payload):
    return ("%s%s*%s\n" % (kind, payload, checksum(payload))).encode("ascii")


def encode_pose(pose, seq):
    return _frame("#", "%.2f,%.2f,%.2f,%.2f,%d" % (pose[0], pose[1], pose[2], pose[3], seq))


def cal_line(limits, homes):
    """The !cal line for the board: four [min, max] pairs then four homes (whole servo degrees)."""
    return "!cal " + ",".join(str(int(v)) for p in limits for v in p) + "," + ",".join(str(int(h)) for h in homes)


def encode_pos(degs, seq):
    return _frame("@", "%.2f,%.2f,%.2f,%.2f,%d" % (degs[0], degs[1], degs[2], degs[3], seq))


class SerialLink:
    def __init__(self, port=None, baud=115200, enabled=True, commands=()):
        self.port_hint, self.baud = port, baud
        self.commands = list(commands)   # settings lines (e.g. "!rate 5") sent each time the port opens
        self.enabled = enabled and serial is not None
        self.suspended = False           # True while another tool (firmware upload) needs the port
        self.ser = None
        self.device = None               # name of the port while open
        self.seq = 0
        self.last_scan = 0.0
        self.last_sent = 0.0
        self.last_state = None
        self.last_pos = None             # last joint angles sent in mirror mode
        self._rx = ""                    # partial line buffer
        self.on_line = None              # optional callable(str): every complete line the board prints
        self.board_rate = None           # last damper settings the board reported (deg/s, deg)
        self.board_range = None
        self.board_pulse = None          # (us at 0 degrees, us at 180 degrees) the board uses in mirror mode, once it has said
        self.board_ik = None             # None: the board has not said (an older sketch); "missing" or "set": it runs the IK and does / does not have the calibration
        self.cal_line = None             # the !cal line to give the board whenever it opens or says it lost the calibration
        self.last_pose = None            # last tool-tip pose sent in mirror mode
        self.phase = "off" if not self.enabled else "waiting"   # off | waiting | open | busy | suspended
        self.status = "serial: off" if not self.enabled else "serial: waiting for COM port"
        self._lock = threading.RLock()

    # ---- discovery -------------------------------------------------------------
    def find_port(self):
        """Name of the ESP32's COM port if it is plugged in (does not open it), else None."""
        if serial is None:
            return None
        for p in list_ports.comports():
            if self.port_hint:
                if p.device.upper() == self.port_hint.upper():
                    return p.device
            elif (p.vid, p.pid) in ESP32_USB_IDS:
                return p.device
        return None

    _find_port = find_port   # old name, kept for compatibility

    # ---- open / close ------------------------------------------------------------
    def _open(self, device):
        s = serial.Serial()
        s.port, s.baudrate, s.write_timeout = device, self.baud, 0.05
        s.dtr = s.rts = False          # set BEFORE open: avoids resetting the ESP32
        s.open()
        self.board_ik = None
        for cmd in self.commands + ([self.cal_line] if self.cal_line else []) + ["!show"]:     # !show: board replies with its saved settings
            s.write(cmd.encode("ascii") + b"\n")
        self.ser, self.device, self._rx = s, device, ""
        self.phase = "open"
        self.status = f"serial: sending on {device}" + (f"  ({', '.join(self.commands)})" if self.commands else "")

    def _drop(self, why, phase="waiting"):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser, self.device = None, None
        self.phase = phase
        self.status = f"serial: {why}"

    def _ensure_open(self, now):
        if not self.enabled:
            return False
        if self.suspended:
            if self.ser is not None:
                self._drop("released for upload", "suspended")
            self.phase, self.status = "suspended", "serial: released for upload"
            return False
        if self.ser is not None:
            return True
        if now - self.last_scan < RESCAN_S:
            return False
        self.last_scan = now
        device = self.find_port()
        if device is None:
            self.phase, self.status = "waiting", "serial: waiting for COM port"
            return False
        try:
            self._open(device)
        except (OSError, serial.SerialException) as e:   # busy (IDE monitor/upload/GUI) or vanished
            self._drop(f"{device} busy/unavailable ({str(e)[:40]})", "busy")
            return False
        return True

    def _drain(self):
        """Read whatever the board printed; deliver complete lines and pick up its settings reply."""
        n = self.ser.in_waiting
        if not n:
            return
        self._rx += self.ser.read(n).decode("ascii", "replace")
        *lines, self._rx = self._rx.split("\n")
        self._rx = self._rx[-300:]
        for line in lines:
            line = line.strip("\r ")
            if not line:
                continue
            mp = PULSES_RE.search(line)
            if mp:
                self.board_pulse = (float(mp.group(1)), float(mp.group(2)))
            m = SETTINGS_RE.search(line)
            if m:
                self.board_rate, self.board_range = float(m.group(1)), float(m.group(2))
            mi = IK_RE.search(line)
            if mi:
                self.board_ik = mi.group(1)
                if self.board_ik == "missing" and self.cal_line:          # a reset board has lost it: give it back (it refuses while following, which is right)
                    try:
                        self.ser.write(self.cal_line.encode("ascii") + b"\n")
                    except (OSError, serial.SerialException, serial.SerialTimeoutException):
                        pass
            if self.on_line:
                try:
                    self.on_line(line)
                except Exception:
                    pass

    # ---- main entry points ----------------------------------------------------------
    def service(self, now=None):
        """Keep the port open and read the board's output WITHOUT sending any input. Never raises."""
        now = time.time() if now is None else now
        with self._lock:
            if not self._ensure_open(now):
                return
            try:
                self._drain()
            except (OSError, serial.SerialException):
                self._drop("COM port lost (unplugged?), waiting")

    def update(self, state, now=None):
        """Call every loop with the latest state: service + send. Never raises, never blocks for long."""
        now = time.time() if now is None else now
        with self._lock:
            if not self._ensure_open(now):
                return
            try:
                self._drain()
                if state == self.last_state and now - self.last_sent < HEARTBEAT_S:
                    return
                self.seq += 1
                self.ser.write(encode(state, self.seq))
                self.last_state, self.last_sent = state, now
            except (OSError, serial.SerialException, serial.SerialTimeoutException):
                self._drop("COM port lost (unplugged?), waiting")

    def update_positions(self, degs, now=None):
        """Mirror mode: send the four joint angles (servo degrees), at most about 25 times a second, repeated at least every 0.1 s so the
        board knows the link is alive. Reads the board's output too. Never raises, never blocks for long."""
        now = time.time() if now is None else now
        degs = [float(v) for v in degs]
        with self._lock:
            if not self._ensure_open(now):
                return
            try:
                self._drain()
                moved = self.last_pos is None or max(abs(a - b) for a, b in zip(degs, self.last_pos)) >= 0.05
                if not ((moved and now - self.last_sent >= 0.04) or now - self.last_sent >= HEARTBEAT_S):
                    return
                self.seq += 1
                self.ser.write(encode_pos(degs, self.seq))
                self.last_pos, self.last_sent, self.last_state, self.last_pose = degs, now, None, None
            except (OSError, serial.SerialException, serial.SerialTimeoutException):
                self._drop("COM port lost (unplugged?), waiting")

    def update_pose(self, pose, now=None):
        """Mirror mode with the IK on the board: send the tool-tip pose (x, y, z mm, angle degrees), at most about 25 times a second, repeated at
        least every 0.1 s as a heartbeat. Reads the board's output too. Never raises, never blocks for long."""
        now = time.time() if now is None else now
        pose = [float(v) for v in pose]
        with self._lock:
            if not self._ensure_open(now):
                return
            try:
                self._drain()
                moved = self.last_pose is None or max(abs(a - b) for a, b in zip(pose, self.last_pose)) >= 0.05
                if not ((moved and now - self.last_sent >= 0.04) or now - self.last_sent >= HEARTBEAT_S):
                    return
                self.seq += 1
                self.ser.write(encode_pose(pose, self.seq))
                self.last_pose, self.last_sent, self.last_state, self.last_pos = pose, now, None, None
            except (OSError, serial.SerialException, serial.SerialTimeoutException):
                self._drop("COM port lost (unplugged?), waiting")

    def set_calibration(self, limits, homes):
        """Remember the real arm's calibration for the board and give it now if the port is open (the board refuses it while it is following the hub)."""
        line = cal_line(limits, homes)
        with self._lock:
            self.cal_line = line
            if self.ser is not None:
                try:
                    self.ser.write(line.encode("ascii") + b"\n")
                except (OSError, serial.SerialException, serial.SerialTimeoutException):
                    self._drop("COM port lost (unplugged?), waiting")

    @property
    def ik_ready(self):
        """True when the board runs the inverse kinematics and has the calibration, so the hub may send poses instead of joint angles."""
        return self.board_ik == "set"

    @property
    def connected(self):
        return self.ser is not None

    def command(self, text):
        """Send one settings line (e.g. '!live 12') now. False if the cable/port isn't available."""
        with self._lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(text.encode("ascii") + b"\n")
                return True
            except (OSError, serial.SerialException, serial.SerialTimeoutException):
                self._drop("COM port lost (unplugged?), waiting")
                return False

    def set_enabled(self, on):
        with self._lock:
            self.enabled = bool(on) and serial is not None
            if not self.enabled:
                self._drop("off", "off")
            else:
                self.phase, self.status, self.last_scan = "waiting", "serial: waiting for COM port", 0.0

    def suspend(self):
        """Release the port (e.g. so arduino-cli can flash the board) until resume()."""
        with self._lock:
            self.suspended = True
            self._drop("released for upload", "suspended")

    def resume(self):
        with self._lock:
            self.suspended, self.last_scan = False, 0.0

    def close(self):
        with self._lock:
            self._drop("closed", "off" if not self.enabled else "waiting")
