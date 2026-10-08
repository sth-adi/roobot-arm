/*
  Robot arm receiver (ESP32)

  Polls GET /state on the end server, unpacks the controller packet and
  prints every change to Serial (115200 baud). applyToArm() is the hook
  where servo/motor control goes later.

  Libraries (Library Manager): ArduinoJson 7.x.  Board: any ESP32 (esp32 core).
  Setup: copy secrets.example.h -> secrets.h and fill in WiFi + STATE_URL.

  Packet shape (from /state):
  {"state":{"lx":0,"ly":0,"rx":0,"ry":0,"lt":0,"rt":0,
            "buttons":{"a":false,...,"rs":false},"dpad":[0,0]},
   "age_s":0.12,"count":42}
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

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

// TODO: drive servos/motors here. Called only when the input changes.
void applyToArm(const ArmInput &i) {
  (void)i;
}

void connectWifi() {
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print('.'); }
  Serial.printf("\nWiFi OK, IP %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  connectWifi();
}

void loop() {
  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();
  if (WiFi.status() != WL_CONNECTED) { connectWifi(); return; }

  WiFiClientSecure client;
  client.setInsecure();   // skips cert check (fine for a hobby link; pin a root CA to harden)
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.setUserAgent("roobot-arm-esp32/0.1");   // Cloudflare rejects default agents

  ArmInput in;
  bool ok = false;
  uint32_t count = 0;
  if (http.begin(client, STATE_URL)) {
    int code = http.GET();
    if (code == 200) ok = unpack(http.getString(), in, count);
    else Serial.printf("HTTP %d\n", code);
    http.end();
  }

  // Failsafe: no fresh packet => all inputs neutral (arm must stop, not keep last command)
  if (!ok) in = ArmInput();

  if (!haveLast || !(in == last)) {
    printInput(in, count ? count : lastCount);
    applyToArm(in);
    last = in;
    haveLast = true;
  }
  if (ok) lastCount = count;
}
