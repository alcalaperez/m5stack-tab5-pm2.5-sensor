/*
 * Tab5 + Module Air Quality (PMSA003) — serial logger
 *
 * Board (Arduino IDE, esp32 core >= 3.2):
 *   Tools -> Board -> esp32 -> "M5Stack Tab5"
 *   (fallback: "ESP32P4 Dev Module" with USB Mode: "Hardware CDC and JTAG")
 *
 * The module is stacked on the Tab5 rear M5-BUS (officially compatible per
 * M5Stack's stack table). Two power requirements, both mandatory:
 *   - Module's own USB-C -> wall charger (feeds the PMSA003 fan; the bus
 *     does not power it — power banks often won't even turn on).
 *   - Tab5 EXT_5V rail enabled in software (M5.Power.setExtOutput(true)),
 *     otherwise the module's logic side stays dead.
 *
 *   Bus pin 15 (G7, PC_RX)  <- module UART_TX   (PMSA003, 9600 8N1)
 *   Bus pin 16 (G6, PC_TX)  -> module UART_RX
 *
 * Every init stage prints a marker over USB CDC (115200) and a heartbeat
 * runs in loop(), so any hang is visible immediately.
 */

#include <Arduino.h>
#include <stdarg.h>
#include <M5Unified.h>   // Tab5 power manager: M5.begin() + Power.setExtOutput()

// USB CDC writes block once the TX FIFO fills and no host is reading (PC
// unplugged / monitor closed) — that stalled the whole sketch. Skip writes
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

#define PM_UART_RX   7    // G7  <- module TX (bus pin 15)
#define PM_UART_TX   6    // G6  -> module RX (bus pin 16)

static uint32_t lastHeartbeat = 0;

static void stage(const char* name) {
  logf("[%8lu ms] STAGE: %s ... ", millis(), name);
}

static void stageDone() {
  logf("OK\n");
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(50);   // a full TX FIFO can stall a write for at most 50 ms
  // Give the CDC port a moment to attach before printing the first markers.
  delay(500);
  logf("\n=== Tab5PM25Logger v1.5 (serial-safe when unplugged) ===\n");

  // EXT_5V feeds the rear M5-Bus 5V, side HY2.0 ports and USB-A. It is
  // software-switched: without enabling it, a stacked module gets no power.
  stage("M5.begin()");
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  stageDone();

  stage("M5.Power.setExtOutput(true) — bus 5V / USB-A output ON");
  M5.Power.setExtOutput(true);
  stageDone();

  stage("Serial2.begin(9600, G7/G6)");
  Serial2.begin(9600, SERIAL_8N1, PM_UART_RX, PM_UART_TX);
  stageDone();

  logf("setup complete, entering loop() — PMSA003 frames print as\n");
  logf("they arrive (~1/s once the module's fan is running).\n");
  logf("If output stops before this line, the hang is above at the last STAGE printed.\n");
}

// --- PMSA003 frame parser -------------------------------------------------

static uint8_t  pmBuf[32];
static uint8_t  pmLen = 0;
static uint32_t bytesSeen = 0;
static uint32_t frames    = 0;

static void handlePmFrame(const uint8_t* f) {
  if (f[2] != 0x00 || f[3] != 0x1C) return;   // payload length must be 28
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += f[i];
  if (sum != ((f[30] << 8) | f[31])) {
    Serial.println("PMSA003: checksum mismatch, frame dropped");
    return;
  }
  // Atmospheric (ambient) concentrations, µg/m3, big-endian u16 at offsets 10/12/14
  uint16_t pm1  = (f[10] << 8) | f[11];
  uint16_t pm25 = (f[12] << 8) | f[13];
  uint16_t pm10 = (f[14] << 8) | f[15];
  uint16_t n03  = (f[16] << 8) | f[17];       // particles > 0.3 µm per 0.1 L
  frames++;
  logf("[%8lu ms] PMSA003 #%lu  PM1.0=%u  PM2.5=%u  PM10=%u  cnt>0.3um=%u\n",
       millis(), frames, pm1, pm25, pm10, n03);
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

void loop() {
  pollPms();

  if (millis() - lastHeartbeat >= 5000) {
    lastHeartbeat = millis();
    logf("[%8lu ms] heartbeat: bytes=%lu frames=%lu\n", millis(), bytesSeen, frames);
    if (frames == 0) {
      logf("  no frames yet — is the module's USB-C on a wall charger? (fan audible?)\n");
    }
  }
}
