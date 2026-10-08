"""Direct USB-serial link to the ESP32 (lowest latency path), used only while its COM port exists.

The port is found by USB ID (CP210x 10C4:EA60, CH340, ESP32-S3) or an explicit name, opened without
toggling DTR/RTS (so the board is NOT reset), and dropped cleanly when the cable is unplugged. While the
port is missing nothing is sent and nothing blocks; it is re-checked about once a second.

Wire format, one ASCII line per update (about 47 bytes, ~4 ms at 115200 baud):
    $lx,ly,rx,ry,lt,rt,buttonmask,dpadx,dpady,seq\\n
buttonmask bits 0..9 = a b x y lb rb back start ls rs.

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
SETTINGS_RE = re.compile(r"settings: max (?:rate|speed at full stick) ([\d.]+) deg/s.*?range ([\d.]+) deg")


def encode(state, seq):
    mask = sum(1 << i for i, k in enumerate(BUTTON_ORDER) if state["buttons"].get(k))
    dx, dy = state["dpad"]
    return (f"${state['lx']:.3f},{state['ly']:.3f},{state['rx']:.3f},{state['ry']:.3f},"
            f"{state['lt']:.3f},{state['rt']:.3f},{mask},{dx},{dy},{seq}\n").encode("ascii")


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
        self._rx = ""                    # partial line buffer
        self.on_line = None              # optional callable(str): every complete line the board prints
        self.board_rate = None           # last damper settings the board reported (deg/s, deg)
        self.board_range = None
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
        for cmd in self.commands + ["!show"]:     # !show: board replies with its saved settings
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
            m = SETTINGS_RE.search(line)
            if m:
                self.board_rate, self.board_range = float(m.group(1)), float(m.group(2))
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
