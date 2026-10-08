/*
  Robot arm receiver (ESP32)

  Polls GET /state on the end server, unpacks the controller packet and
  prints every change to Serial (115200 baud). applyToArm() is the hook
  where servo/motor control goes later.

  Libraries (Library Manager): ArduinoJson 7.x.  Board: any ESP32 (esp32 core).
  Setup: copy secrets.example.h -> secrets.h and fill in WiFi + STATE_URL.
  TLS: server cert is verified against the root CAs in ca_certs.h (clock synced via NTP first).

  Packet shape (from /state):
  {"state":{"lx":0,"ly":0,"rx":0,"ry":0,"lt":0,"rt":0,
            "buttons":{"a":false,...,"rs":false},"dpad":[0,0]},
   "age_s":0.12,"count":42}
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include "ca_certs.h"   // root CAs used to verify the server's TLS certificate

const uint32_t POLL_MS = 50;       // ~20 Hz
const float    STALE_S = 1.5;      // packet older than this = sender gone
const uint32_t HTTP_TIMEOUT_MS = 1500;

struct ArmInput {
  float lx = 0, ly = 0, rx = 0, ry = 0;   // sticks -1..1
  float lt = 0, rt = 0;                   // triggers 0..1
  bool a = false, b = false, x = false, y = false;
  bool lb = false, rb = false, back = false, start = false, ls = false, rs = false;
  int8_t dpadX = 0, dpadY = 0;            // -1/0/1

  bool operator==(const ArmInput &o) const {
    return lx == o.lx && ly == o.ly && rx == o.rx && ry == o.ry && lt == o.lt && rt == o.rt &&
           a == o.a && b == o.b && x == o.x && y == o.y && lb == o.lb && rb == o.rb &&
           back == o.back && start == o.start && ls == o.ls && rs == o.rs &&
           dpadX == o.dpadX && dpadY == o.dpadY;
  }
};

ArmInput current, last;
bool haveLast = false;
uint32_t lastCount = 0;
uint32_t lastPoll = 0;

// Unpack the JSON body of /state into `out`. Returns false if malformed/empty/stale.
bool unpack(const String &body, ArmInput &out, uint32_t &count) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) return false;
  JsonObject s = doc["state"];
  if (s.isNull()) return false;                       // server hasn't got input yet
  if ((doc["age_s"] | 0.0f) > STALE_S) return false;  // sender stopped

  count = doc["count"] | 0;
  out.lx = s["lx"] | 0.0f;  out.ly = s["ly"] | 0.0f;
  out.rx = s["rx"] | 0.0f;  out.ry = s["ry"] | 0.0f;
  out.lt = s["lt"] | 0.0f;  out.rt = s["rt"] | 0.0f;
  JsonObject b = s["buttons"];
  out.a = b["a"] | false;   out.b = b["b"] | false;
  out.x = b["x"] | false;   out.y = b["y"] | false;
  out.lb = b["lb"] | false; out.rb = b["rb"] | false;
  out.back = b["back"] | false; out.start = b["start"] | false;
  out.ls = b["ls"] | false; out.rs = b["rs"] | false;
  out.dpadX = s["dpad"][0] | 0;
  out.dpadY = s["dpad"][1] | 0;
  return true;
}

void printInput(const ArmInput &i, uint32_t count) {
  Serial.printf("#%lu  L(%+.2f,%+.2f) R(%+.2f,%+.2f) LT=%.2f RT=%.2f  dpad(%d,%d)  btn:",
                (unsigned long)count, i.lx, i.ly, i.rx, i.ry, i.lt, i.rt, i.dpadX, i.dpadY);
  const struct { const char *n; bool v; } btn[] = {
    {"A", i.a}, {"B", i.b}, {"X", i.x}, {"Y", i.y}, {"LB", i.lb}, {"RB", i.rb},
    {"BACK", i.back}, {"START", i.start}, {"LS", i.ls}, {"RS", i.rs}};
  bool any = false;
  for (auto &p : btn) if (p.v) { Serial.printf(" %s", p.n); any = true; }
  if (!any) Serial.print(" -");
  Serial.println();
}

// ---- PWM outputs: the 4 thumbstick axes -------------------------------------
// Servo-style PWM: 50 Hz, pulse 1000..2000 us, stick centre = 1500 us.
//   OUT1  GPIO25  <- LX  left stick, left(-1)=1000us  right(+1)=2000us
//   OUT2  GPIO26  <- LY  left stick, down(-1)=1000us  up(+1)=2000us     (Y inverted)
//   OUT3  GPIO27  <- RX  right stick, left=1000us     right=2000us
//   OUT4  GPIO32  <- RY  right stick, down=1000us     up=2000us         (Y inverted)
// Pins avoid strapping/flash pins, so they stay quiet while the ESP32 boots.
const uint8_t  PWM_PINS[4]  = {25, 26, 27, 32};
const char    *PWM_NAMES[4] = {"LX", "LY", "RX", "RY"};
const uint32_t PWM_FREQ_HZ  = 50;
const uint8_t  PWM_BITS     = 14;                 // 16384 steps per 20 ms period
const uint16_t PULSE_MIN_US = 1000, PULSE_MID_US = 1500, PULSE_MAX_US = 2000;

uint32_t usToDuty(uint16_t us) {
  return (uint32_t)us * ((1UL << PWM_BITS) - 1) / (1000000UL / PWM_FREQ_HZ);
}

uint16_t axisToUs(float v) {                      // -1..1 -> 1000..2000 us
  v = constrain(v, -1.0f, 1.0f);
  return (uint16_t)(PULSE_MID_US + v * (PULSE_MAX_US - PULSE_MID_US));
}

void setupPwm() {
  for (int n = 0; n < 4; n++) {
    ledcAttach(PWM_PINS[n], PWM_FREQ_HZ, PWM_BITS);
    ledcWrite(PWM_PINS[n], usToDuty(PULSE_MID_US));   // start centred
  }
}

// Called only when the input changes. Stale/failed packets arrive as all-zero => centred.
void applyToArm(const ArmInput &i, bool log) {
  const float axes[4] = {i.lx, -i.ly, i.rx, -i.ry};   // Y inverted: stick up = high
  uint16_t us[4];
  for (int n = 0; n < 4; n++) {
    us[n] = axisToUs(axes[n]);
    ledcWrite(PWM_PINS[n], usToDuty(us[n]));
  }
  if (!log) return;
  Serial.printf("PWM  %s/GPIO%u=%uus  %s/GPIO%u=%uus  %s/GPIO%u=%uus  %s/GPIO%u=%uus\n",
                PWM_NAMES[0], PWM_PINS[0], us[0], PWM_NAMES[1], PWM_PINS[1], us[1],
                PWM_NAMES[2], PWM_PINS[2], us[2], PWM_NAMES[3], PWM_PINS[3], us[3]);
}

// Apply a new input. PWM is updated on every change; Serial logging is throttled to 10 Hz
// (always logged when returning to neutral) so printing never slows the control path.
// `ok` false (stale/failed/disconnected) => all inputs neutral: the arm must stop, not
// keep the last command.
uint32_t lastLogMs = 0;
void applyInput(bool ok, ArmInput in, uint32_t count) {
  if (!ok) in = ArmInput();
  if (!haveLast || !(in == last)) {
    bool log = (in == ArmInput()) || millis() - lastLogMs >= 100;
    if (log) { lastLogMs = millis(); printInput(in, count ? count : lastCount); }
    applyToArm(in, log);
    last = in;
    haveLast = true;
  }
  if (ok) lastCount = count;
}

// ---- Direct USB-serial input (PC -> ESP32, lowest latency) ---------------------
// The PC bridge (serial_link.py) sends  $lx,ly,rx,ry,lt,rt,buttonmask,dpadx,dpady,seq\n
// only while this board's COM port exists. While those lines keep arriving (< 400 ms old),
// serial is the active source and the WiFi paths are ignored; when it goes quiet the arm is
// centred and WiFi takes over again.
const uint32_t SERIAL_FRESH_MS = 400;
uint32_t lastSerialMs = 0;
bool serialWasActive = false;
String serBuf;

bool serialActive() { return lastSerialMs != 0 && millis() - lastSerialMs < SERIAL_FRESH_MS; }

bool parseSerialLine(const String &s, ArmInput &o, uint32_t &count) {
  if (s.length() < 12 || s[0] != '$') return false;
  float lx, ly, rx, ry, lt, rt;
  unsigned mask = 0;
  int dx = 0, dy = 0;
  unsigned long seq = 0;
  int n = sscanf(s.c_str() + 1, "%f,%f,%f,%f,%f,%f,%u,%d,%d,%lu",
                 &lx, &ly, &rx, &ry, &lt, &rt, &mask, &dx, &dy, &seq);
  if (n < 9) return false;
  o.lx = lx; o.ly = ly; o.rx = rx; o.ry = ry; o.lt = lt; o.rt = rt;
  o.a = mask & 1;       o.b = mask & 2;       o.x = mask & 4;     o.y = mask & 8;
  o.lb = mask & 16;     o.rb = mask & 32;     o.back = mask & 64; o.start = mask & 128;
  o.ls = mask & 256;    o.rs = mask & 512;
  o.dpadX = constrain(dx, -1, 1);
  o.dpadY = constrain(dy, -1, 1);
  count = seq;
  return true;
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      ArmInput in;
      uint32_t count = 0;
      if (parseSerialLine(serBuf, in, count)) {
        lastSerialMs = millis();
        applyInput(true, in, count);
      }
      serBuf = "";
    } else if (c != '\r' && serBuf.length() < 120) {
      serBuf += c;
    }
  }
  bool active = serialActive();
  if (active && !serialWasActive) Serial.println("serial: direct USB input active");
  if (!active && serialWasActive) {
    applyInput(false, ArmInput(), 0);                  // link went quiet: centre first
    Serial.println("serial: input lost, falling back to WiFi");
  }
  serialWasActive = active;
}

// WiFi paths (stream / HTTPS poll) feed in here; ignored while direct serial is active.
void handleInput(bool ok, ArmInput in, uint32_t count) {
  if (serialActive()) return;
  applyInput(ok, in, count);
}

// Waits for WiFi (max 20 s) but keeps serving serial input, so USB control works even when
// there is no WiFi at all. The WiFi driver keeps retrying in the background afterwards.
void connectWifi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis(), dot = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    pollSerial();
    delay(5);
    if (millis() - dot > 400) { dot = millis(); Serial.print('.'); }
  }
  if (WiFi.status() == WL_CONNECTED) Serial.printf("\nWiFi OK, IP %s\n", WiFi.localIP().toString().c_str());
  else Serial.println("\nWiFi not connected yet (will keep retrying)");
}

// TLS validates certificate dates, so the clock must be right before any HTTPS call.
void syncClock() {
  Serial.print("Syncing time");
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  time_t now = 0;
  for (int i = 0; i < 60 && now < 1700000000; i++) {   // wait until clock is sane
    for (int k = 0; k < 10; k++) { pollSerial(); delay(50); }   // 500 ms, serving serial meanwhile
    Serial.print('.');
    time(&now);
  }
  if (now < 1700000000) Serial.println("\nNTP failed - HTTPS will be rejected until time syncs");
  else Serial.printf("\nTime OK (%ld)\n", (long)now);
}

void setup() {
  Serial.setRxBufferSize(1024);   // room for bursts of serial input lines
  Serial.begin(115200);
  delay(300);
  setupPwm();      // outputs centred before WiFi comes up
  connectWifi();
#ifndef STREAM_HOST
  syncClock();     // only HTTPS needs a correct clock (certificate dates)
#endif
}

// Non-blocking WiFi check for the loops: never stalls serial input while WiFi is down.
bool wifiUp() {
  if (WiFi.status() == WL_CONNECTED) return true;
  static uint32_t t = 0;
  if (millis() - t > 5000) { t = millis(); WiFi.reconnect(); }
  return false;
}

#ifdef STREAM_HOST
// ---- Transport A: plain-TCP stream (no TLS, no polling) ----------------------
// One long-lived GET /stream; the server pushes a JSON line per new state plus a 0.5 s
// heartbeat. No handshake per update, so latency is just the network round trip.
WiFiClient stream;
String lineBuf;
uint32_t lastLine = 0, lastTry = 0;

bool openStream() {
  stream.stop();
  stream.setNoDelay(true);
  stream.setTimeout(2);   // seconds, for the header read below
  if (!stream.connect(STREAM_HOST, STREAM_PORT, 2000)) {
    Serial.printf("stream: cannot reach %s:%d\n", STREAM_HOST, (int)STREAM_PORT);
    return false;
  }
  stream.printf("GET /stream HTTP/1.0\r\nHost: %s\r\nUser-Agent: roobot-arm-esp32/0.1\r\n\r\n", STREAM_HOST);
  while (stream.connected()) {                         // skip response headers
    String h = stream.readStringUntil('\n');
    if (h == "\r") { lineBuf = ""; lastLine = millis(); Serial.println("stream: open"); return true; }
    if (h.isEmpty()) break;                            // timed out
  }
  stream.stop();
  return false;
}

void loop() {
  pollSerial();
  if (!wifiUp()) { delay(2); return; }

  if (!stream.connected()) {
    handleInput(false, ArmInput(), 0);                 // failsafe while the link is down
    if (millis() - lastTry > 1000) { lastTry = millis(); openStream(); }
    delay(5);
    return;
  }
  while (stream.available()) {
    char c = stream.read();
    if (c == '\n') {
      lastLine = millis();
      ArmInput in;
      uint32_t count = 0;
      bool ok = unpack(lineBuf, in, count);
      handleInput(ok, in, count);
      lineBuf = "";
    } else if (c != '\r' && lineBuf.length() < 1024) {
      lineBuf += c;
    }
  }
  if (millis() - lastLine > 1500) {                    // no data or heartbeat: link is dead
    Serial.println("stream: timeout, reconnecting");
    stream.stop();
  }
  delay(1);
}

#else
// ---- Transport B: HTTPS polling (fallback when STREAM_HOST is not defined) ----
void loop() {
  pollSerial();
  if (serialActive()) { delay(2); return; }   // direct USB input is driving: skip slow HTTPS polls
  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();
  if (!wifiUp()) { handleInput(false, ArmInput(), 0); return; }

  WiFiClientSecure client;
  client.setCACert(ROOT_CAS);   // verify the server certificate against pinned root CAs
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.setUserAgent("roobot-arm-esp32/0.1");   // Cloudflare rejects default agents

  ArmInput in;
  bool ok = false;
  uint32_t count = 0;
  if (http.begin(client, STATE_URL)) {
    int code = http.GET();
    if (code == 200) ok = unpack(http.getString(), in, count);
    else if (code < 0) Serial.printf("HTTPS error: %s\n", HTTPClient::errorToString(code).c_str());
    else Serial.printf("HTTP %d\n", code);
    http.end();
  }

  handleInput(ok, in, count);
}
#endif
