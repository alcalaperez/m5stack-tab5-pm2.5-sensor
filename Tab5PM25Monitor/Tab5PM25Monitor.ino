/*
 * Tab5 + Module Air Quality (PMSA003) — screen dashboard
 *
 * Same hardware recipe as Tab5PM25Logger (see that sketch / README):
 *   - module stacked on the rear M5-BUS (UART on G7/G6, 9600 8N1)
 *   - module's own USB-C on a wall charger (fan power)
 *   - EXT_5V enabled via M5.Power.setExtOutput(true)
 *
 * Layout uses the wide screen horizontally:
 *   left = big 30 s-average PM2.5, middle = PM1.0/PM10 + US EPA AQI,
 *   right = particle size bands (derived from the cumulative counts),
 *   bottom = ~20 min PM2.5 history.
 *
 * Rendering is flicker-free: static scaffolding is drawn once; dynamic
 * values erase only their own bounding box before redrawing; the history
 * graph repaints only when a new sample lands (every 5 s).
 */

#include <Arduino.h>
#include <stdarg.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

// USB CDC writes block once the TX FIFO fills and no host is reading (PC
// unplugged / monitor closed) — that stalled the whole dashboard. Skip writes
// with no cable, and cap write time via setTxTimeoutMs() in setup.
static void logf(const char* fmt, ...) {
  if (!Serial.isPlugged()) return;
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  Serial.flush();
}

#define PM_UART_RX   7
#define PM_UART_TX   6

#define FLIP_DISPLAY 1   // 1 = mount upside-down (180°); 0 = normal

// Battery charger: OFF — re-tested and acquitted (v3.16): the cold-boot
// self-heal sequence runs identically with the charger disabled. With no
// pack fitted, the rail floats ~1.8-1.9 V (the official M5Stack app shows
// the same). Set to 1 if an NP-F550 pack is ever installed.
#define ENABLE_BATT_CHARGE 0

// Idle dimming: the backlight is the system's dominant power draw and heat
// source. After IDLE_AFTER_MS without touch, drop to IDLE_BRIGHTNESS; any
// touch restores full brightness.
#define FULL_BRIGHTNESS  125
#define IDLE_BRIGHTNESS  60
#define IDLE_AFTER_MS    120000UL   // 2 min

#define AVG_N        30     // frames averaged (~30 s at 1 frame/s)
#define HIST_N       256    // sparkline samples, one per 5 s = ~21 min
#define HIST_PERIOD  5000

static uint16_t ring1[AVG_N], ring25[AVG_N], ring10[AVG_N];
static uint16_t ringBins[6][AVG_N];   // >0.3, >0.5, >1.0, >2.5, >5, >10 um counts
static uint8_t  ringIdx = 0, ringCount = 0;

static uint16_t hist[HIST_N];
static uint16_t histIdx = 0;
static uint16_t histCount = 0, histMax = 20;   // autoscale floor 20 µg/m3
static uint32_t lastHistSample = 0;

static uint32_t frames = 0, bytesSeen = 0;
static uint32_t firstFrameMs = 0;
static uint32_t lastScreen = 0;

// --- Wi-Fi + MQTT ----------------------------------------------------------

#define MQTT_NODE       "tab5pm25"
#define MQTT_STATE_T    MQTT_NODE "/state"
#define MQTT_AVAIL_T    MQTT_NODE "/availability"
#define PUBLISH_PERIOD  10000UL   // HA update interval

static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);
static bool     discoverySent  = false;
static uint32_t lastMqttAttempt = 0;
static uint32_t lastPublish     = 0;

// UI state for partial redraws
static bool waitingShown = false;
static bool bannerOn = false;
static bool graphDirty = true;

// Idle-dim state
static bool     dimmed      = false;
static uint32_t lastTouchMs = 0;

// Sensor bus power sequencing (fan stall handling)
static uint32_t extOnMs = 0;
static bool     wifiStarted = false;
static uint32_t lastWifiBegin = 0;
static uint8_t  wifiBegins   = 0;
static uint32_t bootMs     = 0;

// Cold-boot display recovery: the module fan's inrush during M5.begin() can
// sag the rail mid panel-init (geometry 0x0). Once the fan is at steady
// state, re-running panel init succeeds — proven by post-flash boots, where
// the never-unpowered fan spares the panel.
static bool     displayFailed  = false;
static uint8_t  displayRetries = 0;
static uint32_t lastDispRetry  = 0;

static void applyOrientation() {
  if (M5.Display.height() > M5.Display.width()) {
    M5.Display.setRotation(M5.Display.getRotation() ^ 1);   // force landscape
  }
#if FLIP_DISPLAY
  M5.Display.setRotation(M5.Display.getRotation() ^ 2);     // 180° flip
#endif
}

// --- PMSA003 frame parser -------------------------------------------------

static uint8_t pmBuf[32];
static uint8_t pmLen = 0;

static uint16_t ringAvg(const uint16_t* r) {
  uint32_t s = 0;
  for (int i = 0; i < ringCount; i++) s += r[i];
  return ringCount ? (s + ringCount / 2) / ringCount : 0;
}

static void handlePmFrame(const uint8_t* f) {
  if (f[2] != 0x00 || f[3] != 0x1C) return;
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += f[i];
  if (sum != ((f[30] << 8) | f[31])) return;

  // Atmospheric (ambient) mass concentrations, µg/m3
  uint16_t pm1  = (f[10] << 8) | f[11];
  uint16_t pm25 = (f[12] << 8) | f[13];
  uint16_t pm10 = (f[14] << 8) | f[15];
  // CF=1 "standard particle" copies of the same three channels
  uint16_t s1   = (f[4]  << 8) | f[5];
  uint16_t s25  = (f[6]  << 8) | f[7];
  uint16_t s10  = (f[8]  << 8) | f[9];
  // Particle counts per 0.1 L in six size bins, >0.3 .. >10 µm
  uint16_t bins[6];
  for (int i = 0; i < 6; i++) bins[i] = (f[16 + 2 * i] << 8) | f[17 + 2 * i];

  ring1[ringIdx]  = pm1;
  ring25[ringIdx] = pm25;
  ring10[ringIdx] = pm10;
  for (int i = 0; i < 6; i++) ringBins[i][ringIdx] = bins[i];
  ringIdx = (ringIdx + 1) % AVG_N;
  if (ringCount < AVG_N) ringCount++;
  if (++frames == 1) firstFrameMs = millis();

  logf("[~%6lu s] PM1.0=%u/%u PM2.5=%u/%u PM10=%u/%u (atm/CF1)"
       "  cnt/0.1L: %u %u %u %u %u %u  fw=%u err=%u\n",
       millis() / 1000,
       pm1, s1, pm25, s25, pm10, s10,
       bins[0], bins[1], bins[2], bins[3], bins[4], bins[5],
       f[28], f[29]);
}

static void pollPms() {
  while (Serial2.available() > 0) {
    uint8_t b = Serial2.read();
    bytesSeen++;
    if (pmLen == 0) {
      if (b == 0x42) pmBuf[pmLen++] = b;
    } else if (pmLen == 1) {
      if (b == 0x4D) pmBuf[pmLen++] = b; else pmLen = 0;
    } else {
      pmBuf[pmLen++] = b;
      if (pmLen == sizeof(pmBuf)) { pmLen = 0; handlePmFrame(pmBuf); }
    }
  }
}

// --- Device telemetry ------------------------------------------------------
// M5Unified drives the Tab5's INA226 (battery rail, 5 mΩ shunt, configured
// in M5.begin): getBatteryVoltage() -> mV, getBatteryCurrent() -> mA with
// negative meaning "charging". Die temperature from the P4, RSSI from Wi-Fi.

static float    socTempC = 0, batV = 0, batA = 0;
static int32_t  rssiDb = 0;
static uint32_t lastTelemetry = 0;

static void readTelemetry() {
  socTempC = temperatureRead();
  batV     = M5.Power.getBatteryVoltage() / 1000.0f;   // 0 without a pack
  batA     = M5.Power.getBatteryCurrent() / 1000.0f;
  rssiDb   = (wifiStarted && WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
}

// --- Screen ---------------------------------------------------------------
// Every helper sets its own font AND text size: setTextSize persists across
// draw calls, so leaving it at a big value would blow up all following text.

static uint16_t pm25Color(uint16_t v) {
  if (v <= 9)   return TFT_GREEN;      // EPA 2024 PM2.5 bands
  if (v <= 35)  return TFT_YELLOW;
  if (v <= 55)  return TFT_ORANGE;
  if (v <= 125) return TFT_RED;
  if (v <= 225) return TFT_PURPLE;
  return TFT_MAROON;
}

static const char* pm25Label(uint16_t v) {
  if (v <= 9)   return "GOOD";
  if (v <= 35)  return "MODERATE";
  if (v <= 55)  return "SENSITIVE";
  if (v <= 125) return "UNHEALTHY";
  if (v <= 225) return "VERY BAD";
  return "HAZARDOUS";
}

// US EPA (2024 revision) PM2.5 -> AQI, piecewise linear between breakpoints.
static uint16_t pm25ToAqi(float c) {
  static const float edges[6][2] = {   // {ug/m3, AQI} upper edges; low edge {0,0}
    {  9.0f,  50 }, { 35.4f, 100 }, { 55.4f, 150 },
    { 125.4f, 200 }, { 225.4f,  300 }, { 325.4f, 500 },
  };
  float pc = 0.0f, pa = 0.0f;
  for (auto& e : edges) {
    if (c <= e[0]) {
      return (uint16_t)((e[1] - pa) / (e[0] - pc) * (c - pc) + pa + 0.5f);
    }
    pc = e[0]; pa = e[1];
  }
  return 500;   // above the top breakpoint
}

static void fmtUptime(char* out, size_t n) {
  uint32_t s = millis() / 1000;
  snprintf(out, n, "%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
}

static const char* bandLabels[6] =
    { "0.3-0.5", "0.5-1.0", "1.0-2.5", "2.5-5", "5-10", ">10" };

// HA auto-discovery: one retained config message per sensor; all sensors
// share a single JSON state topic and pick their field via value_template.
static void publishDiscovery() {
  struct { const char* name; const char* key; const char* unit; const char* dc; }
  sensors[] = {
    { "PM2.5",                 "pm25", "µg/m³",  "pm25" },
    { "PM1.0",                 "pm1",  "µg/m³",  "pm1"  },
    { "PM10",                  "pm10", "µg/m³",  "pm10" },
    { "Air Quality Index",     "aqi",  "AQI",    "aqi"  },
    { "Particles 0.3-0.5 µm",  "b035", "n/0.1L", ""     },
    { "Particles 0.5-1.0 µm",  "b051", "n/0.1L", ""     },
    { "Particles 1.0-2.5 µm",  "b102", "n/0.1L", ""     },
    { "Particles 2.5-5 µm",    "b25",  "n/0.1L", ""     },
    { "Particles 5-10 µm",     "b510", "n/0.1L", ""     },
    { "Particles >10 µm",      "b10",  "n/0.1L", ""     },
    { "SoC Temperature",       "tempc", "°C",   "temperature" },
    { "Battery Voltage",       "vbus",  "V",    "voltage" },
    { "Battery Current (est.)","amps",  "A",    "current" },
    { "Battery Power (est.)",  "watts", "W",    "power" },
    { "Wi-Fi RSSI",            "rssi",  "dBm",  "signal_strength" },
  };
  char topic[80], msg[480];
  for (auto& s : sensors) {
    int n = snprintf(msg, sizeof(msg),
      "{\"name\":\"%s\",\"state_topic\":\"" MQTT_STATE_T "\","
      "\"value_template\":\"{{ value_json.%s }}\",\"unit_of_measurement\":\"%s\","
      "\"state_class\":\"measurement\",\"unique_id\":\"" MQTT_NODE "_%s\"",
      s.name, s.key, s.unit, s.key);
    if (s.dc && *s.dc) {
      n += snprintf(msg + n, sizeof(msg) - n, ",\"device_class\":\"%s\"", s.dc);
    }
    snprintf(msg + n, sizeof(msg) - n,
      ",\"device\":{\"identifiers\":[\"" MQTT_NODE "\"],\"name\":\"Tab5 Air Quality\","
      "\"model\":\"Tab5 + PMSA003\",\"manufacturer\":\"M5Stack\"}}");
    snprintf(topic, sizeof(topic), "homeassistant/sensor/" MQTT_NODE "/%s/config", s.key);
    mqtt.publish(topic, msg, true);
  }
  discoverySent = true;
}

static void publishState() {
  uint16_t cum[6];
  for (int i = 0; i < 6; i++) cum[i] = ringAvg(ringBins[i]);
  int32_t band[6];
  for (int i = 0; i < 5; i++) {
    band[i] = (int32_t)cum[i] - (int32_t)cum[i + 1];
    if (band[i] < 0) band[i] = 0;
  }
  band[5] = cum[5];
  uint16_t avg25 = ringAvg(ring25);
  char payload[384];
  snprintf(payload, sizeof(payload),
    "{\"pm25\":%u,\"pm1\":%u,\"pm10\":%u,\"aqi\":%u,"
    "\"b035\":%ld,\"b051\":%ld,\"b102\":%ld,\"b25\":%ld,\"b510\":%ld,\"b10\":%ld,"
    "\"tempc\":%.1f,\"vbus\":%.2f,\"amps\":%.2f,\"watts\":%.2f,\"rssi\":%d}",
    avg25, ringAvg(ring1), ringAvg(ring10), pm25ToAqi(avg25),
    (long)band[0], (long)band[1], (long)band[2],
    (long)band[3], (long)band[4], (long)band[5],
    socTempC, batV, batA, batV * batA, (int)rssiDb);
  mqtt.publish(MQTT_STATE_T, payload, true);
}

// Non-blocking connect/reconnect with a 5 s backoff; Wi-Fi itself
// auto-reconnects, so only the MQTT session is managed here.
// Guarded by wifiStarted: calling WiFi.status() while the C6 co-processor
// is dead (the post-reset state) blocks the loop in hosted retries.
static void ensureConnectivity() {
  if (!wifiStarted) return;
  static bool lastWifiUp = false;
  bool up = (WiFi.status() == WL_CONNECTED);
  if (up && !lastWifiUp) logf("Wi-Fi connected\n");
  lastWifiUp = up;
  if (!up) {
    // A begin that landed while the C6 transport was dead doesn't get
    // retried by the Arduino layer — re-begin every 30 s, up to 10 tries.
    if (wifiBegins < 10 && millis() - lastWifiBegin >= 30000) {
      lastWifiBegin = millis();
      wifiBegins++;
      logf("Wi-Fi retry %d/10 (re-begin)\n", wifiBegins);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }
  if (mqtt.connected()) { mqtt.loop(); return; }
  uint32_t now = millis();
  if (now - lastMqttAttempt < 5000) return;
  lastMqttAttempt = now;
  discoverySent = false;
  char cid[20];
  snprintf(cid, sizeof(cid), "m5-%06llX",
           (unsigned long long)(ESP.getEfuseMac() & 0xFFFFFF));
  if (mqtt.connect(cid, MQTT_USER, MQTT_PASS, MQTT_AVAIL_T, 1, true, "offline")) {
    mqtt.publish(MQTT_AVAIL_T, "online", true);
    publishDiscovery();
    publishState();
    logf("MQTT connected to " MQTT_HOST "\n");
  } else {
    logf("MQTT connect failed, rc=%d (retry in 5 s)\n", mqtt.state());
  }
}

// Everything that never changes once drawn: titles, labels, axis, captions.
static void drawStatic() {
  auto& d = M5.Display;
  const int W = d.width(), H = d.height();
  const int cx = W * 28 / 100;

  d.fillScreen(TFT_BLACK);
  waitingShown = false;
  bannerOn = false;
  graphDirty = true;

  d.setTextSize(1);
  d.setTextDatum(top_left);

  // Header title
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.setCursor(20, 12);
  d.print("PM2.5  ·  PMSA003");

  // On-screen power-off button (top-right corner)
  d.fillRoundRect(W - 120, 8, 100, 36, 8, TFT_MAROON);
  d.setTextDatum(middle_center);
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setTextColor(TFT_WHITE, TFT_MAROON);
  d.drawString("OFF", W - 70, 28);
  // restore the defaults the rest of the static layer was written against
  d.setTextDatum(top_left);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);

  // LEFT: unit label under the big number. Its whole glyph box must stay
  // below the number's dynamic erase zone (y < H*46/100+100 = erase bottom),
  // or the 1 s repaint clips its top edge.
  d.setFont(&fonts::FreeSansBold18pt7b);
  d.drawCenterString("ug/m3  PM2.5", cx, H * 46 / 100 + 125);

  // MIDDLE: mass channel labels + AQI label
  d.setFont(&fonts::FreeSansBold18pt7b);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setCursor(W * 46 / 100, H * 22 / 100);
  d.print("PM1.0");
  d.setCursor(W * 46 / 100, H * 34 / 100);
  d.print("PM10");
  d.setFont(&fonts::FreeSans12pt7b);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.setCursor(W * 46 / 100, H * 44 / 100);
  d.print("30 s rolling average");
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setCursor(W * 46 / 100, H * 53 / 100);
  d.print("US EPA AQI");

  // RIGHT: band header + row labels (values are dynamic)
  d.setTextColor(TFT_CYAN, TFT_BLACK);
  d.setCursor(W * 76 / 100, H * 14 / 100);
  d.print("um bands / 0.1 L");
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  for (int i = 0; i < 6; i++) {
    d.setCursor(W * 76 / 100, H * 22 / 100 + i * H * 7 / 100);
    d.printf("%-7s", bandLabels[i]);
  }

  // BOTTOM: graph baseline + caption
  int gy0 = H * 72 / 100, gh = H - gy0 - 18, gw = W - 40, gx = 20;
  d.drawFastHLine(gx, gy0 + gh, gw, TFT_DARKGREY);
  d.setTextDatum(bottom_left);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setCursor(gx, H - 2);
  d.printf("PM2.5 last ~20 min   scale 0-%u ug/m3", histMax);
}

// Values only. Each element erases its own bounding box first — sized for
// the widest possible value — so partial updates never leave residue.
static void drawDynamic() {
  auto& d = M5.Display;
  const int W = d.width(), H = d.height();
  const int cx = W * 28 / 100;

  if (frames == 0) {
    if (!waitingShown) {
      d.setTextSize(1);
      d.setTextDatum(middle_center);
      d.setFont(&fonts::FreeSansBold18pt7b);
      d.setTextColor(TFT_YELLOW, TFT_BLACK);
      d.drawString("waiting for sensor data...", W / 2, H / 2);
      d.setFont(&fonts::FreeSans12pt7b);
      d.setTextColor(TFT_DARKGREY, TFT_BLACK);
      d.drawString("check the module is seated — if it stays dark, power-cycle", W / 2, H / 2 + 60);
      waitingShown = true;
    }
    return;
  }
  if (waitingShown) drawStatic();   // clears the waiting screen

  uint16_t avg25 = ringAvg(ring25);
  uint16_t avg1  = ringAvg(ring1);
  uint16_t avg10 = ringAvg(ring10);
  uint16_t col   = pm25Color(avg25);

  d.setTextSize(1);

  // Header right: uptime + frame count (kept clear of the OFF button)
  d.fillRect(W - 620, 8, 490, 34, TFT_BLACK);
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  char up[16];
  fmtUptime(up, sizeof(up));
  char hdr[48];
  snprintf(hdr, sizeof(hdr), "%s   %lu frames", up, frames);
  // print()/printf() ignores the text datum and draws right of the cursor —
  // that put the text on top of the OFF button. drawRightString() ends at x.
  d.drawRightString(hdr, W - 140, 12);

  // Warm-up banner on/off transitions. Sits at y=H*54/720..+40 so it ends
  // above the right column's "um bands" header (top at H*14/100).
  bool banner = millis() - firstFrameMs < 60000;
  if (banner != bannerOn) {
    d.fillRect(0, H * 54 / 720, W, 40, banner ? TFT_MAROON : TFT_BLACK);
    if (banner) {
      d.setTextDatum(middle_center);
      d.setFont(&fonts::FreeSansBold12pt7b);
      d.setTextColor(TFT_WHITE, TFT_MAROON);
      d.drawString("warming up — readings stabilize ~60 s after fan start", W / 2, H * 54 / 720 + 20);
    }
    bannerOn = banner;
  }

  // LEFT: status word + big number
  d.fillRect(cx - 200, H * 20 / 100 - 30, 400, 60, TFT_BLACK);
  d.setTextDatum(middle_center);
  d.setFont(&fonts::FreeSansBold24pt7b);
  d.setTextColor(col, TFT_BLACK);
  d.drawString(pm25Label(avg25), cx, H * 20 / 100);

  d.fillRect(cx - 220, H * 46 / 100 - 90, 440, 190, TFT_BLACK);
  d.setFont(&fonts::Font7);       // 48 px 7-segment digits
  d.setTextSize(3);               // -> ~144 px digits
  d.drawNumber(avg25, cx, H * 46 / 100);
  d.setTextSize(1);

  // MIDDLE: PM values + AQI value
  d.setTextDatum(top_left);
  d.setFont(&fonts::FreeSansBold18pt7b);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.fillRect(W * 46 / 100 + 130, H * 22 / 100 - 6, 220, 34, TFT_BLACK);
  d.setCursor(W * 46 / 100 + 140, H * 22 / 100);
  d.printf("%4u ug/m3", avg1);
  d.fillRect(W * 46 / 100 + 130, H * 34 / 100 - 6, 220, 34, TFT_BLACK);
  d.setCursor(W * 46 / 100 + 140, H * 34 / 100);
  d.printf("%4u ug/m3", avg10);

  uint16_t aqi = pm25ToAqi(avg25);
  d.fillRect(W * 46 / 100, H * 59 / 100 - 10, 200, 60, TFT_BLACK);
  d.setFont(&fonts::FreeSansBold24pt7b);
  d.setTextColor(col, TFT_BLACK);
  d.setCursor(W * 46 / 100, H * 59 / 100);
  d.printf("%u", aqi);

  // RIGHT: band values (labels are static)
  uint16_t cum[6];
  for (int i = 0; i < 6; i++) cum[i] = ringAvg(ringBins[i]);
  int32_t band[6];
  for (int i = 0; i < 5; i++) {
    band[i] = (int32_t)cum[i] - (int32_t)cum[i + 1];
    if (band[i] < 0) band[i] = 0;
  }
  band[5] = cum[5];
  d.setFont(&fonts::FreeSansBold12pt7b);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.fillRect(W * 76 / 100 + 85, H * 22 / 100 - 6, 110, 6 * H * 7 / 100 + 34, TFT_BLACK);
  for (int i = 0; i < 6; i++) {
    d.setCursor(W * 76 / 100 + 90, H * 22 / 100 + i * H * 7 / 100);
    d.printf("%5ld", (long)band[i]);
  }

  // Connectivity status (middle column, below AQI, above the graph)
  d.fillRect(W * 46 / 100, H * 67 / 100, 420, 22, TFT_BLACK);
  d.setTextDatum(top_left);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.setCursor(W * 46 / 100, H * 67 / 100 + 2);
  if (wifiStarted && WiFi.status() == WL_CONNECTED) {
    d.printf("WiFi %s   MQTT %s", WiFi.localIP().toString().c_str(),
             mqtt.connected() ? "connected" : "connecting");
  } else if (wifiStarted) {
    d.print("WiFi connecting...");
  } else {
    d.print("WiFi waiting for sensor");
  }

  // Device telemetry (right column, under the particle bands). V/A/W are
  // the battery-rail readings via M5Unified's INA226 driver — always shown:
  // with the charger off and no pack, the rail floats ~1.8-1.9 V at 0 A
  // (same reading as the official M5Stack app).
  d.fillRect(W * 76 / 100, H * 66 / 100, 300, 40, TFT_BLACK);
  d.setTextDatum(top_left);
  d.setFont(&fonts::FreeSans9pt7b);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.setCursor(W * 76 / 100, H * 66 / 100 + 2);
  d.printf("SoC %.1fC   V %.2f   A %.2f", socTempC, batV, batA);
  d.setCursor(W * 76 / 100, H * 66 / 100 + 22);
  d.printf("%.1fW   RSSI %ddBm", batV * batA, (int)rssiDb);

  // BOTTOM: history graph, only when a new sample landed
  if (graphDirty && histCount > 1) {
    int gy0 = H * 72 / 100, gh = H - gy0 - 18, gw = W - 40, gx = 20;
    d.fillRect(gx, gy0, gw, gh, TFT_BLACK);
    for (uint16_t i = 0; i < histCount; i++) {
      uint16_t idx = (histCount < HIST_N) ? i : (histIdx + i) % HIST_N;
      int h = (int)((uint32_t)hist[idx] * gh / histMax);
      if (h < 1) h = 1;
      int x = gx + i * gw / HIST_N;
      d.drawFastVLine(x, gy0 + gh - h, h, pm25Color(hist[idx]));
    }
    // caption carries the current autoscale ceiling, so refresh it too
    d.fillRect(gx, H - 24, 640, 22, TFT_BLACK);
    d.setTextDatum(bottom_left);
    d.setFont(&fonts::FreeSans9pt7b);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setCursor(gx, H - 2);
    d.printf("PM2.5 last ~20 min   scale 0-%u ug/m3", histMax);
    graphDirty = false;
  }
}

// Blank the screen before cutting power — avoids the power-off image burn
// reported for this panel (M5Stack issue #180).
static void powerOffNow() {
  logf("powering off via screen button\n");
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setBrightness(0);
  delay(200);
  M5.Power.powerOff();
  while (true) { delay(1000); }   // only reached if powerOff() fails
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(50);   // a full TX FIFO can stall a write for at most 50 ms
  delay(500);
  logf("\n=== Tab5PM25Monitor v3.18 (rail V/A/W always shown, charger off) ===\n");

  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  // NOTE: keep cfg.output_power at its default (true) — M5.begin's power
  // sequencing is the only configuration proven to boot this stack, and
  // disabling it correlated with dead display/C6 during debugging.
  M5.begin(cfg);

  applyOrientation();
  logf("Display geometry: %dx%d rotation %d\n",
       M5.Display.width(), M5.Display.height(), M5.Display.getRotation());
  if (M5.Display.width() == 0 || M5.Display.height() == 0) {
    displayFailed = true;
    logf("display init FAILED (0x0) — likely the module fan inrush during"
         " panel init; retrying from ~8 s every 5 s once the rail settles\n");
  }

  // Display self-test: backlight up, color flashes.
  M5.Display.setBrightness(FULL_BRIGHTNESS);
  lastTouchMs = millis();
  M5.Display.fillScreen(TFT_RED);   delay(400);
  M5.Display.fillScreen(TFT_BLUE);  delay(400);

  Serial2.begin(9600, SERIAL_8N1, PM_UART_RX, PM_UART_TX);

  // Bus power is already on (M5.begin enables it — it feeds the C6 too).
  // Wait out the boot surge, then let the fan see a settled rail.
  delay(1500);
  M5.Power.setExtOutput(true);
  extOnMs = millis();

  // Wi-Fi bring-up is deferred to loop(): starting the radio exactly as the
  // module's fan drew its inrush current was the cold-boot stall trigger.
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);   // HA discovery payloads exceed the 256-byte default

  drawStatic();
  drawDynamic();
  bootMs = millis();
  logf("setup complete\n");
}

void loop() {
  M5.update();   // physical buttons + touch state tracking

  pollPms();
  ensureConnectivity();

  if (mqtt.connected() && frames > 0 && millis() - lastPublish >= PUBLISH_PERIOD) {
    lastPublish = millis();
    publishState();
  }

  uint32_t now = millis();

  // Touch: while dimmed, any contact only wakes the screen (so the first
  // tap can never land on the OFF button by accident); while awake, the
  // OFF button (top-right) shuts the device down.
  if (M5.Touch.getCount() > 0) {
    lastTouchMs = now;
    if (dimmed) {
      dimmed = false;
      M5.Display.setBrightness(FULL_BRIGHTNESS);
    } else {
      auto t = M5.Touch.getDetail();
      if (t.wasClicked() && t.x >= M5.Display.width() - 130 && t.y <= 52) {
        powerOffNow();
      }
    }
  }

  // Idle dimming — the backlight is the dominant power draw and heater.
  if (!dimmed && now - lastTouchMs >= IDLE_AFTER_MS) {
    dimmed = true;
    M5.Display.setBrightness(IDLE_BRIGHTNESS);
  }

  if (now - lastTelemetry >= 2000) {
    lastTelemetry = now;
    readTelemetry();
  }

  // Wi-Fi starts only after the display has settled (recovered or given up)
  // AND the sensor is alive — keeping its bring-up out of the panel/fan
  // fragile window entirely.
  bool screenSettled = !displayFailed || displayRetries >= 5;
  if (!wifiStarted && screenSettled && (frames > 0 || now - bootMs > 20000)) {
    wifiStarted = true;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiBegin = now;
    wifiBegins = 1;
    logf("Wi-Fi starting (after screen settled)\n");
    M5.Power.setBatteryCharge(ENABLE_BATT_CHARGE);   // see define near the top
  }

  // Cold-boot display recovery: retry panel init from ~8 s, every 5 s (up
  // to 5 times). The fan inrush ends a few seconds after spin-up, so the
  // first retry usually attaches the panel. Post-flash boots (fan already
  // warm) confirm a steady rail always lets the panel through.
  if (displayFailed && displayRetries < 5 && now - bootMs > 8000 &&
      now - lastDispRetry >= 5000) {
    lastDispRetry = now;
    displayRetries++;
    logf("display retry %d/5...\n", displayRetries);
    M5.Display.init();
    if (M5.Display.width() > 0 && M5.Display.height() > 0) {
      displayFailed = false;
      applyOrientation();
      M5.Display.setBrightness(FULL_BRIGHTNESS);
      lastTouchMs = now;                 // don't idle-dim instantly
      drawStatic();
      logf("display RECOVERED: %dx%d rotation %d\n",
           M5.Display.width(), M5.Display.height(), M5.Display.getRotation());
    }
  }

  // The ESP32-C6 shares the EXT power domain — the bus must NEVER be cycled
  // after boot or Wi-Fi dies with it. If the fan stalled at cold boot, say
  // so and leave recovery to a power-cycle of the whole device.
  static bool stallNoted = false;
  if (!stallNoted && frames == 0 && now - bootMs > 30000) {
    stallNoted = true;
    logf("no PMSA003 frames 30 s after boot — fan may have stalled; power-cycle the device\n");
  }
  if (ringCount > 0 && now - lastHistSample >= HIST_PERIOD) {
    lastHistSample = now;
    uint16_t v = ringAvg(ring25);
    hist[histIdx] = v;
    histIdx = (histIdx + 1) % HIST_N;
    if (histCount < HIST_N) histCount++;
    if (v > histMax) histMax = v;   // autoscale
    graphDirty = true;
  }

  if (now - lastScreen >= 1000) {
    lastScreen = now;
    drawDynamic();
  }
}
