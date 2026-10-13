// NavPlotter node - Raspberry Pi Pico W (or Pico 2 W) with the Waveshare Pico-LoRa-SX1262 module and an external GPS module
//
// A plotter without a screen: the board does the work and a phone (or a NavPlotter display) shows it:
//  - reads the GPS, exchanges positions with the other nodes over LoRa (same packet as the other NavPlotters),
//    relays packets (mesh) and computes the collision alarm;
//  - creates its own WiFi network and serves a web app (web/index.html) plus a small JSON API.
// The on-board LED shows the status.
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LEAmDNS.h>
#include <LittleFS.h>
#include <SD.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <pico/unique_id.h>
#include <vector>
#include <math.h>
#include "traffic_packet.h"
#include "web_page.h"

#define FIRMWARE_VERSION "0.3.1"

// ---- Pins ----
// Waveshare Pico-LoRa-SX1262 (fixed by the module, on SPI1)
#define PIN_LORA_NSS 3
#define PIN_LORA_SCK 10
#define PIN_LORA_MOSI 11
#define PIN_LORA_MISO 12
#define PIN_LORA_RST 15
#define PIN_LORA_BUSY 2
#define PIN_LORA_DIO1 20
// GPS module on UART0
#define PIN_GPS_TX 0     // board TX -> GPS RX (may be left unconnected)
#define PIN_GPS_RX 1     // board RX <- GPS TX
// Optional micro SD module (chart tiles) on SPI0
#define PIN_SD_MISO 16
#define PIN_SD_CS 17
#define PIN_SD_SCK 18
#define PIN_SD_MOSI 19

// ---- Radio and traffic ----
#define LORA_POWER_DBM 22
#define LORA_TX_MOVING_MS 10000UL
#define LORA_TX_STOPPED_MS 30000UL
#define MOVING_MIN_KNOTS 0.5
#define MAX_TARGETS 30
// Collision alarm: closest point of approach below a passing distance within a warning time, for targets
// inside the alarm range. Distance and time come from a profile chosen in the setup page, or are typed in.
#define ALARM_PROFILE_CUSTOM 4
struct AlarmProfile { float cpaNm; uint16_t tcpaMin; };
const AlarmProfile ALARM_PROFILES[ALARM_PROFILE_CUSTOM] = {
  {0.05, 2},   // harbour and narrow waters
  {0.1, 5},    // coastal
  {0.5, 10},   // open sea
  {1.0, 20},   // night or poor visibility
};

// Network scenarios chosen in the setup page
#define NET_OFFGRID 0    // the node's own WiFi, charts from the SD card
#define NET_DIRECT 1     // the node's own WiFi, charts from the internet (through the phone)
#define NET_BOAT 2       // the node joins the boat WiFi; if it is not found, its own WiFi comes back
#define STA_CONNECT_MS 20000UL   // how long to wait for the boat WiFi
#define STA_RETRY_MS 180000UL    // then try again this often

#define CONFIG_FILE "/config.txt"
#define USERDATA_FILE "/userdata.json"
#define USERDATA_MAX 32768

struct Config {
  char name[BOAT_NAME_LEN] = "";
  uint8_t type = NODE_TYPE_BOAT;
  bool share = true;            // transmit our own position; when off the node only listens and relays
  uint8_t alarmRangeNm = 3;     // 3, 6 or 12; 0 = no limit
  uint8_t alarmProfile = 1;     // index in ALARM_PROFILES, or ALARM_PROFILE_CUSTOM
  float alarmCpaNm = 0.1;       // the custom profile's passing distance and warning time
  uint16_t alarmTcpaMin = 5;
  uint8_t wifiOffMin = 0;       // objects only: minutes after power-up before the WiFi goes off (0 = never)
  char lang[3] = "en";          // "en" or "it"
  uint8_t tiles = 0;            // 0 = online, 1 = SD card, 2 = none
  bool sound = true;            // alarm sound on the phone
  char wifiPass[33] = "navplotter";
  uint8_t net = NET_DIRECT;
  char staSsid[33] = "";        // boat WiFi name and password, for NET_BOAT
  char staPass[65] = "";
};

struct Target {
  uint32_t id = 0;
  char name[BOAT_NAME_LEN] = "";
  uint8_t type = NODE_TYPE_BOAT;
  bool simulated = false;
  double lat = 0, lon = 0;
  float cog = 0, sog = 0;   // degrees, knots
  unsigned long lastSeen = 0;
  unsigned long staleMs = TARGET_STALE_MS, expireMs = TARGET_EXPIRE_MS; // longer for a node that sleeps between packets
  float rssi = 0;
  bool relayed = false;
  bool cpaValid = false;
  bool alarm = false;
  double cpaNm = 0, tcpaS = 0, distNm = 0, brg = 0;
};

Config config;
WebServer server(80);
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY, SPI1);
TinyGPSPlus gps;

std::vector<Target> targets;
uint32_t myId = 0;
uint32_t gpsBaud = 0;
bool radioOk = false, sdOk = false, fsOk = false;
volatile bool radioIrq = false;
bool transmitting = false, txIsRelay = false;
unsigned long txStart = 0, nextTx = 0;
uint32_t txCount = 0, rxCount = 0, relayCount = 0;
uint8_t seq = 0;
float lastCourse = 0;
bool courseKnown = false;
bool alarmActive = false;
char alarmName[BOAT_NAME_LEN] = "";
String apSsid, hostName;
bool wifiStarted = false, wifiOff = false;
bool apMode = true;             // true = our own WiFi network is on; false = joined (or joining) the boat WiFi
unsigned long staNextTry = 0;

// ============================================
// Helpers
// ============================================

// The passing distance (NM) and warning time (s) in use
float alarmCpaNm() { return config.alarmProfile < ALARM_PROFILE_CUSTOM ? ALARM_PROFILES[config.alarmProfile].cpaNm : config.alarmCpaNm; }
float alarmTcpaS() { return 60.0 * (config.alarmProfile < ALARM_PROFILE_CUSTOM ? ALARM_PROFILES[config.alarmProfile].tcpaMin : config.alarmTcpaMin); }

bool gpsFix() {
  return gps.location.isValid() && gps.location.age() < 5000;
}

bool moving() {
  return gps.speed.isValid() && gps.speed.knots() >= MOVING_MIN_KNOTS;
}

// Heading = GPS course. When stopped the GPS course is noise, so the last good one is kept.
float heading() {
  if(gps.course.isValid() && moving()) {
    lastCourse = gps.course.deg();
    courseKnown = true;
  }
  return lastCourse;
}

// Names travel inside JSON: keep only plain printable characters
void sanitizeName(char* name) {
  for(char* c = name; *c; c++) {
    if(*c < ' ' || *c > '~' || *c == '"' || *c == '\\' || *c == '<' || *c == '>' || *c == '&') *c = '?';
  }
}

// Text typed by the user (WiFi names) placed inside JSON
String jsonEscape(const char* text) {
  String out;
  for(const char* c = text; *c; c++) {
    if(*c == '"' || *c == '\\') { out += '\\'; out += *c; }
    else if((uint8_t)*c >= ' ') out += *c;
  }
  return out;
}

bool staConnected() {
  return !apMode && WiFi.status() == WL_CONNECTED;
}

// The Pico W measures its supply on a pin shared with the WiFi chip: not read, the web app shows "-"
float batteryVolts() {
  return 0;
}

IPAddress ownIp() {
  return apMode ? WiFi.softAPIP() : WiFi.localIP();
}

// ============================================
// Configuration
// ============================================

// The settings are a small text file on the internal flash, one "key=value" per line
void loadConfig() {
  String pass = "navplotter";
  File f = fsOk ? LittleFS.open(CONFIG_FILE, "r") : File();
  while(f && f.available()) {
    String line = f.readStringUntil('\n');
    line.replace("\r", "");
    int eq = line.indexOf('=');
    if(eq <= 0) continue;
    String k = line.substring(0, eq), v = line.substring(eq + 1);
    if(k == "name") strlcpy(config.name, v.c_str(), sizeof(config.name));
    else if(k == "type") config.type = v.toInt();
    else if(k == "share") config.share = v == "1";
    else if(k == "range") config.alarmRangeNm = v.toInt();
    else if(k == "profile") config.alarmProfile = v.toInt();
    else if(k == "cpa") config.alarmCpaNm = v.toFloat();
    else if(k == "tcpa") config.alarmTcpaMin = v.toInt();
    else if(k == "wifioff") config.wifiOffMin = v.toInt();
    else if(k == "lang") strlcpy(config.lang, v.c_str(), sizeof(config.lang));
    else if(k == "tiles") config.tiles = v.toInt();
    else if(k == "sound") config.sound = v == "1";
    else if(k == "pass") pass = v;
    else if(k == "net") config.net = v.toInt();
    else if(k == "stassid") strlcpy(config.staSsid, v.c_str(), sizeof(config.staSsid));
    else if(k == "stapass") strlcpy(config.staPass, v.c_str(), sizeof(config.staPass));
  }
  if(f) f.close();
  if(config.net > NET_BOAT) config.net = NET_DIRECT;

  if(config.name[0] == '\0') snprintf(config.name, sizeof(config.name), "NODE-%04X", (unsigned)(myId & 0xFFFF));
  sanitizeName(config.name);
  if(config.type > NODE_TYPE_OBJECT) config.type = NODE_TYPE_BOAT;
  if(config.alarmRangeNm != 0 && config.alarmRangeNm != 3 && config.alarmRangeNm != 6 && config.alarmRangeNm != 12) config.alarmRangeNm = 3;
  if(config.alarmProfile > ALARM_PROFILE_CUSTOM) config.alarmProfile = 1;
  config.alarmCpaNm = constrain(config.alarmCpaNm, 0.02f, 5.0f);
  config.alarmTcpaMin = constrain(config.alarmTcpaMin, 1, 60);
  if(strcmp(config.lang, "it") != 0) strcpy(config.lang, "en");
  if(config.tiles > 2) config.tiles = 0;
  if(pass.length() < 8 || pass.length() > 32) pass = "navplotter";
  strcpy(config.wifiPass, pass.c_str());
}

void saveConfig() {
  if(!fsOk) return;
  File f = LittleFS.open(CONFIG_FILE, "w");
  if(!f) return;
  f.printf("profile=%u\ncpa=%.2f\ntcpa=%u\nwifioff=%u\n", config.alarmProfile, config.alarmCpaNm, config.alarmTcpaMin, config.wifiOffMin);
  f.printf("name=%s\ntype=%u\nshare=%d\nrange=%u\nlang=%s\ntiles=%u\nsound=%d\npass=%s\nnet=%u\nstassid=%s\nstapass=%s\n",
    config.name, config.type, config.share ? 1 : 0, config.alarmRangeNm, config.lang, config.tiles, config.sound ? 1 : 0,
    config.wifiPass, config.net, config.staSsid, config.staPass);
  f.close();
}

// ============================================
// GPS
// ============================================

// Finds the GPS module speed by trying the common ones
void detectGpsBaud() {
  const uint32_t bauds[] = {9600, 38400, 115200, 4800, 57600};
  for(uint32_t baud : bauds) {
    Serial1.end();
    Serial1.begin(baud);
    unsigned long start = millis();
    int dollars = 0, printable = 0, total = 0;
    while(millis() - start < 1200) {
      while(Serial1.available()) {
        char c = Serial1.read();
        total++;
        if(c == '$') dollars++;
        if((c >= ' ' && c <= '~') || c == '\r' || c == '\n') printable++;
      }
    }
    if(dollars >= 2 && printable > total * 9 / 10) {
      gpsBaud = baud;
      Serial.printf("GPS found at %lu baud\n", (unsigned long)baud);
      return;
    }
  }
  Serial.println("GPS not found on GP1");
}

void readGps() {
  static unsigned long lastRetry = 0;
  while(Serial1.available()) gps.encode(Serial1.read());
  // No module found at boot: try again every now and then (it may have been plugged in later)
  if(!gpsBaud && millis() - lastRetry > 60000) {
    lastRetry = millis();
    detectGpsBaud();
  }
}

// ============================================
// LoRa traffic
// ============================================

void onRadioIrq() {
  radioIrq = true;
}

void initRadio() {
  SPI1.setSCK(PIN_LORA_SCK);
  SPI1.setTX(PIN_LORA_MOSI);
  SPI1.setRX(PIN_LORA_MISO);
  SPI1.begin();
  // The module powers the radio TCXO from DIO3 at 1.8 V and switches the antenna with DIO2.
  // If the radio does not start that way, it is tried as a version with a plain crystal.
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_POWER_DBM, LORA_PREAMBLE, 1.8);
  if(state != RADIOLIB_ERR_NONE && state != RADIOLIB_ERR_CHIP_NOT_FOUND) {
    Serial.printf("LoRa start with TCXO failed (%d), trying without\n", state);
    state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_POWER_DBM, LORA_PREAMBLE, 0);
  }
  if(state == RADIOLIB_ERR_CHIP_NOT_FOUND) Serial.println("LoRa module not found: is it plugged on the board?");
  else if(state != RADIOLIB_ERR_NONE) Serial.printf("LoRa start failed (%d)\n", state);
  Serial.flush();
  if(state == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    // The library starts with the transmitter limited to 60 mA, which holds the power well below the
    // 22 dBm asked for: the full power needs the limit raised
    radio.setCurrentLimit(140);
    radio.setCRC(2);
    radio.setDio1Action(onRadioIrq);
    radioOk = radio.startReceive() == RADIOLIB_ERR_NONE;
  }
  nextTx = millis() + 5000 + random(0, 3000);
  Serial.printf("LoRa %s: %.3f MHz, SF%d, %d dBm\n", radioOk ? "ready" : "NOT available", LORA_FREQ, LORA_SF, LORA_POWER_DBM);
}

void sendPosition() {
  TrafficPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic[0] = 'N'; pkt.magic[1] = 'P';
  pkt.version = 1;
  pkt.ttl = MESH_TTL;
  pkt.id = myId;
  pkt.seq = seq++;
  pkt.flags = (config.type & TRAFFIC_TYPE_MASK) << TRAFFIC_TYPE_SHIFT;
  pkt.lat = (int32_t)lround(gps.location.lat() * 1e7);
  pkt.lon = (int32_t)lround(gps.location.lng() * 1e7);
  pkt.cog = (uint16_t)lround(heading() * 10);
  pkt.sog = moving() ? (uint16_t)lround(gps.speed.knots() * 100) : 0;
  strncpy(pkt.name, config.name, BOAT_NAME_LEN - 1);

  if(radio.startTransmit((uint8_t*)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE) {
    transmitting = true;
    txIsRelay = false;
    txStart = millis();
  } else {
    radio.startReceive();
  }
  // A little random delay keeps two nodes from always transmitting at the same instant
  nextTx = millis() + (moving() ? LORA_TX_MOVING_MS : LORA_TX_STOPPED_MS) + random(0, 2000);
}

void processPacket(const TrafficPacket& pkt, float rssi) {
  if(pkt.magic[0] != 'N' || pkt.magic[1] != 'P' || pkt.version != 1 || pkt.id == myId) return;

  // Mesh: drop copies already received (and if somebody else relayed it, we do not need to)
  if(meshAlreadySeen(pkt.id, pkt.seq)) {
    meshCancelRelay(pkt.id, pkt.seq);
    return;
  }
  rxCount++;
  meshQueueRelay(pkt);

  Target* t = nullptr;
  for(auto& existing : targets) {
    if(existing.id == pkt.id) t = &existing;
  }
  if(!t) {
    if((int)targets.size() >= MAX_TARGETS) return;
    targets.push_back(Target());
    t = &targets.back();
    t->id = pkt.id;
  }
  strncpy(t->name, pkt.name, BOAT_NAME_LEN - 1);
  t->name[BOAT_NAME_LEN - 1] = '\0';
  sanitizeName(t->name);
  t->type = (pkt.flags >> TRAFFIC_TYPE_SHIFT) & TRAFFIC_TYPE_MASK;
  if(t->type > NODE_TYPE_OBJECT) t->type = NODE_TYPE_BOAT;
  t->simulated = pkt.flags & TRAFFIC_FLAG_SIMULATED;
  t->lat = pkt.lat / 1e7;
  t->lon = pkt.lon / 1e7;
  t->cog = pkt.cog / 10.0;
  t->sog = pkt.sog / 100.0;
  t->lastSeen = millis();
  t->staleMs = trafficStaleMs(pkt.flags);
  t->expireMs = trafficExpireMs(pkt.flags);
  t->rssi = rssi;
  t->relayed = pkt.ttl < MESH_TTL;
  Serial.printf("RX %s%s %.5f %.5f %.1f kn %.0f deg, RSSI %.0f\n", t->name, t->relayed ? " (relayed)" : "",
    t->lat, t->lon, t->sog, t->cog, rssi);
}

void handleRadio() {
  if(!radioOk) return;

  if(radioIrq) {
    radioIrq = false;
    if(transmitting) {
      radio.finishTransmit();
      transmitting = false;
      if(txIsRelay) relayCount++;
      else txCount++;
    } else {
      TrafficPacket pkt;
      if(radio.getPacketLength() == sizeof(pkt) && radio.readData((uint8_t*)&pkt, sizeof(pkt)) == RADIOLIB_ERR_NONE) {
        processPacket(pkt, radio.getRSSI());
      }
    }
    radio.startReceive();
  }

  // Transmission stuck: go back to receiving
  if(transmitting && millis() - txStart > 3000) {
    radio.finishTransmit();
    transmitting = false;
    radio.startReceive();
  }

  // Our own position goes out only if sharing is allowed in the setup page.
  // Receiving and relaying the others (below) do not depend on it.
  if(!transmitting && (long)(millis() - nextTx) >= 0) {
    if(config.share && gpsFix()) sendPosition();
    else nextTx = millis() + 2000;
  }

  // Mesh: relay packets from other nodes that still have ttl
  TrafficPacket relay;
  if(!transmitting && meshNextRelay(relay)) {
    if(radio.startTransmit((uint8_t*)&relay, sizeof(relay)) == RADIOLIB_ERR_NONE) {
      transmitting = true;
      txIsRelay = true;
      txStart = millis();
    } else {
      radio.startReceive();
    }
  }
}

// Distance, bearing, closest point of approach (CPA) and time to it (TCPA), on a local flat plane in metres
void updateTarget(Target& t) {
  t.cpaValid = false;
  t.alarm = false;
  if(!gpsFix()) return;

  const double KN = 0.514444; // knots -> m/s
  bool stale = millis() - t.lastSeen > t.staleMs;
  double age = (millis() - t.lastSeen) / 1000.0;
  double lat0 = gps.location.lat() * DEG_TO_RAD;

  double tgtSog = t.sog * KN, tgtCog = t.cog * DEG_TO_RAD;
  // Relative position; a fresh target is moved forward by the time since its last packet
  double rx = (t.lon - gps.location.lng()) * 111320.0 * cos(lat0) + (stale ? 0 : tgtSog * sin(tgtCog) * age);
  double ry = (t.lat - gps.location.lat()) * 110540.0 + (stale ? 0 : tgtSog * cos(tgtCog) * age);
  t.distNm = hypot(rx, ry) / 1852.0;
  double brg = atan2(rx, ry) * RAD_TO_DEG;
  t.brg = brg < 0 ? brg + 360 : brg;
  if(stale) return;

  double ownSog = moving() ? gps.speed.knots() * KN : 0;
  double ownCog = heading() * DEG_TO_RAD;
  double vx = tgtSog * sin(tgtCog) - ownSog * sin(ownCog);
  double vy = tgtSog * cos(tgtCog) - ownSog * cos(ownCog);
  double v2 = vx * vx + vy * vy;
  if(v2 < 0.06) return; // relative speed under half a knot: nothing is closing in
  double tcpa = -(rx * vx + ry * vy) / v2;
  if(tcpa < 0) return;  // moving apart

  t.cpaValid = true;
  t.tcpaS = tcpa;
  t.cpaNm = hypot(rx + vx * tcpa, ry + vy * tcpa) / 1852.0;
  t.alarm = (config.alarmRangeNm == 0 || t.distNm <= config.alarmRangeNm) && t.cpaNm <= alarmCpaNm() && t.tcpaS <= alarmTcpaS();
}

// Once a second: drop lost targets and recompute approaches
void updateTraffic() {
  static unsigned long last = 0;
  if(millis() - last < 1000) return;
  last = millis();

  for(auto it = targets.begin(); it != targets.end(); ) {
    if(millis() - it->lastSeen > it->expireMs) it = targets.erase(it);
    else ++it;
  }
  const Target* worst = nullptr;
  for(auto& t : targets) {
    updateTarget(t);
    if(t.alarm && (!worst || t.tcpaS < worst->tcpaS)) worst = &t;
  }
  alarmActive = worst != nullptr;
  if(worst) strcpy(alarmName, worst->name);
}

// ============================================
// Web server
// ============================================

void sendJson(const String& json) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/html; charset=utf-8", (const char*)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
}

String configJson() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"name\":\"%s\",\"type\":%u,\"share\":%s,\"alarmRange\":%u,\"lang\":\"%s\",\"tiles\":%u,\"sound\":%s,\"id\":\"%08lX\",\"version\":\"%s\",",
    config.name, config.type, config.share ? "true" : "false", config.alarmRangeNm, config.lang, config.tiles,
    config.sound ? "true" : "false", (unsigned long)myId, FIRMWARE_VERSION);
  String json = buf;
  json += "\"wifiOff\":" + String(config.wifiOffMin) + ",";
  json += "\"alarmProfile\":" + String(config.alarmProfile) + ",\"alarmCpa\":" + String(config.alarmCpaNm, 2) + ",\"alarmTcpa\":" + String(config.alarmTcpaMin) + ",";
  // The boat WiFi password is never sent back, only whether one is stored
  json += "\"net\":" + String(config.net) + ",\"staSsid\":\"" + jsonEscape(config.staSsid) + "\",\"staPassSet\":" +
    (config.staPass[0] ? "true" : "false") + ",\"ssid\":\"" + jsonEscape(apSsid.c_str()) + "\",\"host\":\"" + hostName + ".local\",\"oneWifi\":true}";
  return json;
}

void handleGetConfig() {
  sendJson(configJson());
}

void handlePostConfig() {
  bool restartWifi = false;
  if(server.hasArg("name")) {
    String n = server.arg("name");
    n.trim();
    if(n.length() > 0) {
      strncpy(config.name, n.c_str(), BOAT_NAME_LEN - 1);
      config.name[BOAT_NAME_LEN - 1] = '\0';
      sanitizeName(config.name);
    }
  }
  if(server.hasArg("type")) {
    int v = server.arg("type").toInt();
    if(v >= NODE_TYPE_BOAT && v <= NODE_TYPE_OBJECT) config.type = v;
  }
  if(server.hasArg("share")) config.share = server.arg("share") == "1";
  if(server.hasArg("alarmRange")) {
    int v = server.arg("alarmRange").toInt();
    if(v == 0 || v == 3 || v == 6 || v == 12) config.alarmRangeNm = v;
  }
  if(server.hasArg("alarmProfile")) {
    int v = server.arg("alarmProfile").toInt();
    if(v >= 0 && v <= ALARM_PROFILE_CUSTOM) config.alarmProfile = v;
  }
  if(server.hasArg("alarmCpa")) {
    float v = server.arg("alarmCpa").toFloat();
    if(v >= 0.02 && v <= 5.0) config.alarmCpaNm = v;
  }
  if(server.hasArg("alarmTcpa")) {
    int v = server.arg("alarmTcpa").toInt();
    if(v >= 1 && v <= 60) config.alarmTcpaMin = v;
  }
  if(server.hasArg("wifiOff")) {
    int v = server.arg("wifiOff").toInt();
    if(v == 0 || v == 5 || v == 10 || v == 30) config.wifiOffMin = v;
  }
  if(server.hasArg("lang")) {
    strcpy(config.lang, server.arg("lang") == "it" ? "it" : "en");
  }
  if(server.hasArg("tiles")) {
    int v = server.arg("tiles").toInt();
    if(v >= 0 && v <= 2) config.tiles = v;
  }
  if(server.hasArg("sound")) config.sound = server.arg("sound") == "1";
  if(server.hasArg("net")) {
    int v = server.arg("net").toInt();
    if(v >= NET_OFFGRID && v <= NET_BOAT && v != config.net) {
      config.net = v;
      restartWifi = true;
    }
  }
  if(server.hasArg("staSsid")) {
    String v = server.arg("staSsid");
    v.trim();
    if(v.length() <= 32 && v != config.staSsid) {
      strcpy(config.staSsid, v.c_str());
      if(config.net == NET_BOAT) restartWifi = true;
    }
  }
  // An empty password field means "keep the stored one"
  if(server.hasArg("staPass") && server.arg("staPass").length() > 0) {
    String v = server.arg("staPass");
    if(v.length() <= 64 && v != config.staPass) {
      strcpy(config.staPass, v.c_str());
      if(config.net == NET_BOAT) restartWifi = true;
    }
  }
  if(server.hasArg("wifiPass")) {
    String p = server.arg("wifiPass");
    if(p.length() >= 8 && p.length() <= 32 && p != config.wifiPass) {
      strcpy(config.wifiPass, p.c_str());
      restartWifi = true;
    }
  }
  saveConfig();
  String json = configJson();
  json.remove(json.length() - 1);
  json += restartWifi ? ",\"restart\":true}" : ",\"restart\":false}";
  sendJson(json);
  if(restartWifi) {
    // Network changes apply after a restart
    delay(500);
    rp2040.reboot();
  }
}

void handleState() {
  String json;
  json.reserve(512 + targets.size() * 260);
  char buf[320];

  bool fix = gpsFix();
  float hdg = heading();
  snprintf(buf, sizeof(buf),
    "{\"own\":{\"fix\":%s,\"lat\":%.7f,\"lon\":%.7f,\"sog\":%.2f,\"cog\":%.1f,\"cogValid\":%s,\"moving\":%s,\"sats\":%d,\"hdop\":%.1f,\"time\":\"%02d:%02d:%02d\"},",
    fix ? "true" : "false", fix ? gps.location.lat() : 0.0, fix ? gps.location.lng() : 0.0,
    gps.speed.isValid() ? gps.speed.knots() : 0.0, hdg, courseKnown ? "true" : "false", moving() ? "true" : "false",
    gps.satellites.isValid() ? (int)gps.satellites.value() : 0, gps.hdop.isValid() ? gps.hdop.hdop() : 99.9,
    gps.time.isValid() ? gps.time.hour() : 0, gps.time.isValid() ? gps.time.minute() : 0, gps.time.isValid() ? gps.time.second() : 0);
  json += buf;

  json += "\"targets\":[";
  bool first = true;
  for(const auto& t : targets) {
    snprintf(buf, sizeof(buf),
      "%s{\"id\":\"%08lX\",\"name\":\"%s\",\"type\":%u,\"sim\":%s,\"lat\":%.7f,\"lon\":%.7f,\"sog\":%.2f,\"cog\":%.1f,\"age\":%lu,"
      "\"rssi\":%.0f,\"relayed\":%s,\"stale\":%s,\"dist\":%.3f,\"brg\":%.0f,\"cpaValid\":%s,\"cpa\":%.3f,\"tcpa\":%.0f,\"alarm\":%s}",
      first ? "" : ",", (unsigned long)t.id, t.name, t.type, t.simulated ? "true" : "false", t.lat, t.lon, t.sog, t.cog,
      (millis() - t.lastSeen) / 1000, t.rssi, t.relayed ? "true" : "false",
      millis() - t.lastSeen > t.staleMs ? "true" : "false", t.distNm, t.brg,
      t.cpaValid ? "true" : "false", t.cpaNm, t.tcpaS, t.alarm ? "true" : "false");
    json += buf;
    first = false;
  }
  json += "],";

  snprintf(buf, sizeof(buf),
    "\"alarm\":%s,\"alarmName\":\"%s\",\"lora\":{\"ok\":%s,\"tx\":%lu,\"rx\":%lu,\"relay\":%lu,\"freq\":%.3f},"
    "\"gpsBaud\":%lu,\"sd\":%s,\"battery\":%.2f,\"uptime\":%lu,\"sta\":%s,\"staIp\":\"%s\",\"apIp\":\"%s\"}",
    alarmActive ? "true" : "false", alarmActive ? alarmName : "", radioOk ? "true" : "false",
    (unsigned long)txCount, (unsigned long)rxCount, (unsigned long)relayCount, LORA_FREQ,
    (unsigned long)gpsBaud, sdOk ? "true" : "false", batteryVolts(), millis() / 1000,
    staConnected() ? "true" : "false", staConnected() ? WiFi.localIP().toString().c_str() : "",
    apMode ? WiFi.softAPIP().toString().c_str() : "");
  json += buf;
  sendJson(json);
}

// Waypoints and other data the web app keeps: stored as one JSON document on the internal flash
void handleGetUserdata() {
  if(fsOk && LittleFS.exists(USERDATA_FILE)) {
    File f = LittleFS.open(USERDATA_FILE, "r");
    server.sendHeader("Cache-Control", "no-store");
    server.streamFile(f, "application/json");
    f.close();
  } else {
    sendJson("{}");
  }
}

void handlePostUserdata() {
  String body = server.arg("plain");
  if(!fsOk || body.length() == 0 || body.length() > USERDATA_MAX || body[0] != '{') {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  File f = LittleFS.open(USERDATA_FILE, "w");
  bool ok = f && f.print(body) == body.length();
  if(f) f.close();
  server.send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// Chart tiles from the SD card: /tiles/z/x/y.jpg, same layout as the T-Deck card
void handleNotFound() {
  String uri = server.uri();
  if(sdOk && uri.startsWith("/tiles/") && uri.endsWith(".jpg") && uri.indexOf("..") < 0 && uri.length() < 48) {
    File f = SD.open(uri);
    if(f && !f.isDirectory()) {
      server.sendHeader("Cache-Control", "max-age=86400");
      server.streamFile(f, "image/jpeg");
      f.close();
      return;
    }
    if(f) f.close();
  }
  server.send(404, "text/plain", "Not found");
}

// Our own WiFi network. The address is the one the other NavPlotter nodes use.
void startAp() {
  if(wifiStarted && !apMode) WiFi.disconnect(true);
  wifiStarted = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSsid.c_str(), config.wifiPass);
  apMode = true;
  Serial.printf("WiFi network %s, open http://%s\n", apSsid.c_str(), WiFi.softAPIP().toString().c_str());
}

// Joins the boat WiFi. The Pico W cannot keep its own network on at the same time: if the boat WiFi is not
// found within STA_CONNECT_MS, our own network comes back.
bool joinBoatWifi() {
  Serial.printf("Joining boat WiFi \"%s\"...\n", config.staSsid);
  if(wifiStarted && apMode) WiFi.softAPdisconnect(true);
  wifiStarted = true;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostName.c_str());
  WiFi.begin(config.staSsid, config.staPass);
  apMode = false;
  unsigned long start = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - start < STA_CONNECT_MS) {
    readGps();
    delay(20);
  }
  if(WiFi.status() == WL_CONNECTED) {
    Serial.printf("Boat WiFi joined: http://%s or http://%s.local\n", WiFi.localIP().toString().c_str(), hostName.c_str());
    return true;
  }
  Serial.println("Boat WiFi not reachable, back to the node WiFi");
  startAp();
  staNextTry = millis() + STA_RETRY_MS;
  return false;
}


// An object left alone (a buoy) does not need its WiFi once it is set up: it goes off some minutes after
// power-up, to save the battery. Never while a phone is connected; switching the node off and on brings it back.
void handleWifiOff() {
  static unsigned long quietSince = 0;
  if(wifiOff || config.type != NODE_TYPE_OBJECT || config.wifiOffMin == 0) return;
  if(millis() < config.wifiOffMin * 60000UL) return;
  if(WiFi.softAPgetStationNum() > 0) { quietSince = 0; return; }
  if(!quietSince) quietSince = millis();
  if(millis() - quietSince < 60000) return;
  server.stop();
  WiFi.mode(WIFI_OFF);
  wifiOff = true;
  Serial.println("WiFi switched off (object): switch the node off and on to get it back");
}

void handleNetwork() {
  if(wifiOff) return;
  MDNS.update();
  if(config.net != NET_BOAT || config.staSsid[0] == '\0') return;
  if(!apMode) {
    // On the boat WiFi: if it disappears for a while, go back to our own network
    static unsigned long lostSince = 0;
    if(WiFi.status() == WL_CONNECTED) { lostSince = 0; return; }
    if(!lostSince) lostSince = millis();
    if(millis() - lostSince > STA_CONNECT_MS) {
      lostSince = 0;
      Serial.println("Boat WiFi lost, back to the node WiFi");
      startAp();
      staNextTry = millis() + STA_RETRY_MS;
    }
  } else if((long)(millis() - staNextTry) >= 0) {
    // On our own network: look for the boat WiFi again, but never while somebody is using ours
    if(WiFi.softAPgetStationNum() > 0) staNextTry = millis() + 60000;
    else joinBoatWifi();
  }
}

void initWeb() {
  apSsid = String("NavPlotter-") + config.name;
  apSsid.replace(" ", "_");
  // Name on the boat network: http://<node name>.local
  hostName = "";
  for(const char* c = config.name; *c; c++) {
    if(isalnum((unsigned char)*c)) hostName += (char)tolower((unsigned char)*c);
    else if(hostName.length() && !hostName.endsWith("-")) hostName += '-';
  }
  while(hostName.endsWith("-")) hostName.remove(hostName.length() - 1);
  if(hostName.length() == 0) hostName = "navplotter";

  if(config.net != NET_BOAT || config.staSsid[0] == '\0' || !joinBoatWifi()) {
    if(!wifiStarted) startAp();
  }
  MDNS.begin(hostName.c_str());
  MDNS.addService("http", "tcp", 80);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/userdata", HTTP_GET, handleGetUserdata);
  server.on("/api/userdata", HTTP_POST, handlePostUserdata);
  server.onNotFound(handleNotFound);
  server.begin();
}

// ============================================
// Status on the on-board LED and on the serial port
// ============================================

// LED: short flash every 2 s = no GPS fix; on with a short dip every 2 s = fix; fast blinking = collision alarm
void showStatus() {
  unsigned long now = millis();
  bool on;
  if(alarmActive) on = (now / 150) % 2;
  else if(gpsFix()) on = now % 2000 > 100;
  else on = now % 2000 < 100;
  static int shown = -1;
  if((int)on != shown) { shown = on; digitalWrite(LED_BUILTIN, on); }

  static unsigned long last = 0;
  if(now - last < 5000) return;
  last = now;
  Serial.printf("status: %s %s, gps %lu baud %s sats %d, lora %s tx %lu rx %lu relay %lu, nodes %d, sd %d, wifi clients %d, free heap %d\n",
    wifiOff ? "WiFi off" : apMode ? "own WiFi" : "boat WiFi", wifiOff ? "-" : ownIp().toString().c_str(), (unsigned long)gpsBaud, gpsFix() ? "fix" : "no fix",
    gps.satellites.isValid() ? (int)gps.satellites.value() : 0, radioOk ? "ok" : "KO", (unsigned long)txCount,
    (unsigned long)rxCount, (unsigned long)relayCount, (int)targets.size(), sdOk, (apMode && !wifiOff) ? WiFi.softAPgetStationNum() : 0,
    rp2040.getFreeHeap());
}

// ============================================
// Setup and loop
// ============================================

// The board has no screen and no keys: the name can also be set from the computer, typing "name <name>"
// on the serial port. Everything else is set from the web app.
void handleSerial() {
  static String line;
  while(Serial.available()) {
    char c = Serial.read();
    if(c != '\n' && c != '\r') { if(line.length() < 40) line += c; continue; }
    if(line.startsWith("name ")) {
      String n = line.substring(5);
      n.trim();
      if(n.length() > 0) {
        strlcpy(config.name, n.c_str(), sizeof(config.name));
        sanitizeName(config.name);
        saveConfig();
        Serial.printf("Name set to %s, restarting\n", config.name);
        delay(300);
        rp2040.reboot();
      }
    }
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500); // lets the USB serial port come up, so the first lines are not lost

  // A number of our own from the board's unique serial number
  pico_unique_board_id_t uid;
  pico_get_unique_board_id(&uid);
  myId = 2166136261UL;
  for(uint8_t b : uid.id) myId = (myId ^ b) * 16777619UL;

  fsOk = LittleFS.begin();
  if(!fsOk) Serial.println("Internal storage not available");
  loadConfig();
  Serial.printf("NavPlotter %s (Pico W), node %s, id %08lX\n", FIRMWARE_VERSION, config.name, (unsigned long)myId);

  // Optional SD card with chart tiles
  SPI.setRX(PIN_SD_MISO);
  SPI.setTX(PIN_SD_MOSI);
  SPI.setSCK(PIN_SD_SCK);
  sdOk = SD.begin(PIN_SD_CS, SPI);
  Serial.println(sdOk ? "SD card ready" : "SD card not present");

  Serial1.setTX(PIN_GPS_TX);
  Serial1.setRX(PIN_GPS_RX);
  Serial1.setFIFOSize(1024);

  Serial.println("Starting the radio...");
  initRadio();
  Serial.println("Starting the WiFi...");
  Serial.flush();
  initWeb();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Looking for the GPS...");
  detectGpsBaud();
  Serial.println("Running");
}

void loop() {
  readGps();
  handleRadio();
  updateTraffic();
  handleNetwork();
  if(!wifiOff) server.handleClient();
  handleWifiOff();
  showStatus();
  handleSerial();
}
