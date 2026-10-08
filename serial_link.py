"""Direct USB-serial link to the ESP32 (lowest latency path), used only while its COM port exists.

The port is found by USB ID (CP210x 10C4:EA60) or an explicit name, opened without toggling
DTR/RTS (so the board is NOT reset), and dropped cleanly when the cable is unplugged. While the
port is missing nothing is sent and nothing blocks; it is re-checked about once a second.

Wire format, one ASCII line per update (about 40 bytes, ~4 ms at 115200 baud):
    $lx,ly,rx,ry,lt,rt,buttonmask,dpadx,dpady,seq\\n
buttonmask bits 0..9 = a b x y lb rb back start ls rs.
"""
import re
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
        self.ser = None
        self.seq = 0
        self.last_scan = 0.0
        self.last_sent = 0.0
        self.last_state = None
        self._rx = ""
        self.board_rate = None        # last damper settings the board reported (deg/s, deg)
        self.board_range = None
        self.status = "serial: off" if not self.enabled else "serial: waiting for COM port"

    def _find_port(self):
        for p in list_ports.comports():
            if self.port_hint:
                if p.device.upper() == self.port_hint.upper():
                    return p.device
            elif (p.vid, p.pid) in ESP32_USB_IDS:
                return p.device
        return None

    def _open(self, device):
        s = serial.Serial()
        s.port, s.baudrate, s.write_timeout = device, self.baud, 0.05
        s.dtr = s.rts = False          # set BEFORE open: avoids resetting the ESP32
        s.open()
        for cmd in self.commands + ["!show"]:     # !show: board replies with its saved settings
            s.write(cmd.encode("ascii") + b"\n")
        self.ser = s
        self.status = f"serial: sending on {device}" + (f"  ({', '.join(self.commands)})" if self.commands else "")

    def _drop(self, why):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.status = f"serial: {why}"

    def update(self, state, now=None):
        """Call every loop with the latest state. Never raises, never blocks for long."""
        if not self.enabled:
            return
        now = time.time() if now is None else now
        if self.ser is None:
            if now - self.last_scan < RESCAN_S:
                return
            self.last_scan = now
            device = self._find_port()
            if device is None:
                self.status = "serial: waiting for COM port"
                return
            try:
                self._open(device)
            except (OSError, serial.SerialException) as e:   # busy (IDE monitor/upload) or vanished
                self._drop(f"{device} busy/unavailable ({str(e)[:40]})")
                return
        if state == self.last_state and now - self.last_sent < HEARTBEAT_S:
            return
        try:
            if self.ser.in_waiting:                  # drain the board's prints; keep its settings reply
                self._rx = (self._rx + self.ser.read(self.ser.in_waiting).decode("ascii", "replace"))[-600:]
                m = re.findall(r"settings: max rate ([\d.]+) deg/s.*?range ([\d.]+) deg", self._rx)
                if m:
                    self.board_rate, self.board_range = float(m[-1][0]), float(m[-1][1])
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
        if self.ser is None:
            return False
        try:
            self.ser.write(text.encode("ascii") + b"\n")
            return True
        except (OSError, serial.SerialException, serial.SerialTimeoutException):
            self._drop("COM port lost (unplugged?), waiting")
            return False

    def close(self):
        self._drop("closed")
