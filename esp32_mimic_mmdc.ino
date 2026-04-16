/*
 * mmdc emulator
 * Emulates a Medallion MMDC by polling for water depth via RS485.
 *
 * Toggle bit detection:
 *   Response byte [10] is a toggle bit that the gateway flips on every
 *   reply. We track it across consecutive responses to detect whether
 *   the gateway is sending fresh NMEA2k data or holding the last known
 *   good value (stale). When the toggle stops flipping the gateway has
 *   stopped responding entirely. When it keeps flipping but depth is
 *   unchanged for several seconds the gateway is in "stale hold" mode —
 *   which should cause the real MMDC display to blink the last value.
 */

#include <HardwareSerial.h>
#include <math.h>

// RS485 pins
#define RS485_RX_PIN    16
#define RS485_TX_PIN    17
#define RS485_DE_RE_PIN 21

const uint8_t  DEPTH_REQUEST[]    = {0x04, 0x09, 0x11, 0xE2};
const size_t   DEPTH_REQUEST_LEN  = sizeof(DEPTH_REQUEST);
const size_t   DEPTH_RESPONSE_LEN = 13;
const uint32_t BAUDRATE           = 76800;

// How long with no toggle change before we declare the gateway silent
#define TOGGLE_STALE_MS   3000
// How long with same depth + live toggle before we declare depth stale
#define DEPTH_STALE_MS    6000

HardwareSerial RS485Serial(2);

// ── Toggle tracking ───────────────────────────────────────────────────────────
static uint8_t  lastToggle       = 0xFF;   // 0xFF = never seen
static uint32_t lastToggleFlipMs = 0;
static bool     toggleFlipping   = false;

// ── Depth tracking ────────────────────────────────────────────────────────────
static float    lastDepthFt      = -1.0f;
static uint32_t lastDepthChangeMs= 0;
static bool     depthStaleHold   = false;  // toggle alive but depth unchanged

// ── Stats ─────────────────────────────────────────────────────────────────────
static uint32_t responseCount    = 0;
static uint32_t timeoutCount     = 0;
static uint32_t checksumFail     = 0;

uint8_t calculate_checksum(const uint8_t *data, uint8_t len) {
  uint8_t total = 0;
  for (uint8_t i = 0; i < len; i++) total = (total + data[i]) & 0xFF;
  return ((~total + 1) & 0xFF);
}

bool verifyChecksum(const uint8_t *response, size_t len) {
  if (len < 2) return false;
  return calculate_checksum(response, len - 1) == response[len - 1];
}

float extractDepthFeet(const uint8_t *response) {
  uint16_t encoded = ((uint16_t)response[5] << 8) | response[4];
  return encoded / 10.0f;
}

void setup() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  Serial.begin(115200);
  RS485Serial.begin(BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("\n=== MMDC Emulator ===");
  Serial.println("Polling for depth. Monitoring toggle bit for stale-data detection.");
  Serial.println("Toggle byte = response[10]");
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  // ── Send request ───────────────────────────────────────────────────────────
  Serial.printf("[%lus] TX: ", now / 1000);
  for (size_t i = 0; i < DEPTH_REQUEST_LEN; i++) Serial.printf("%02X ", DEPTH_REQUEST[i]);
  Serial.println();

  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(200);
  RS485Serial.write(DEPTH_REQUEST, DEPTH_REQUEST_LEN);
  RS485Serial.flush();
  delayMicroseconds(500);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // ── Read response ──────────────────────────────────────────────────────────
  uint8_t  response[DEPTH_RESPONSE_LEN];
  size_t   bytesRead = 0;
  uint32_t startTime = millis();

  while (bytesRead < DEPTH_RESPONSE_LEN && (millis() - startTime) < 500) {
    if (RS485Serial.available()) response[bytesRead++] = RS485Serial.read();
  }

  // ── Process ────────────────────────────────────────────────────────────────
  if (bytesRead == DEPTH_RESPONSE_LEN) {

    if (!verifyChecksum(response, DEPTH_RESPONSE_LEN)) {
      checksumFail++;
      Serial.printf("[%lus] Checksum FAIL (total fails: %lu)\n",
                    now / 1000, checksumFail);
      delay(1000);
      return;
    }

    responseCount++;
    Serial.printf("[%lus] RX: ", now / 1000);
    for (size_t i = 0; i < DEPTH_RESPONSE_LEN; i++) Serial.printf("%02X ", response[i]);
    Serial.println();

    uint8_t toggle   = response[10];
    float   depthFt  = extractDepthFeet(response);

    // ── Toggle analysis ─────────────────────────────────────────────────────
    bool toggleChanged = (lastToggle == 0xFF) || (toggle != lastToggle);
    if (toggleChanged) {
      lastToggle       = toggle;
      lastToggleFlipMs = now;
      toggleFlipping   = true;
    } else {
      // Toggle did not change this response
      if ((now - lastToggleFlipMs) > TOGGLE_STALE_MS) {
        toggleFlipping = false;
      }
    }

    // ── Depth change tracking ────────────────────────────────────────────────
    if (depthFt != lastDepthFt) {
      lastDepthFt       = depthFt;
      lastDepthChangeMs = now;
      depthStaleHold    = false;
    } else {
      // Depth unchanged — stale hold if toggle still flipping but depth frozen
      if (toggleFlipping && (now - lastDepthChangeMs) > DEPTH_STALE_MS) {
        depthStaleHold = true;
      }
    }

    // ── Determine display state ──────────────────────────────────────────────
    const char *state;
    if (!toggleFlipping) {
      state = "GATEWAY SILENT (toggle frozen - no RS485 responses)";
    } else if (depthStaleHold) {
      state = "STALE HOLD (toggle live, depth frozen - NMEA2k lost, display should BLINK)";
    } else {
      state = "OK";
    }

    // ── Print ────────────────────────────────────────────────────────────────
    Serial.printf("[%lus] depth=%.1f ft  toggle=0x%02X(%s)  state=%s\n",
                  now / 1000,
                  depthFt,
                  toggle,
                  toggleChanged ? "FLIPPED" : "same",
                  state);



  } else {
    timeoutCount++;
    toggleFlipping = false;
    Serial.printf("[%lus] TIMEOUT - got %u/%u bytes (timeouts: %lu)\n",
                  now / 1000, bytesRead, DEPTH_RESPONSE_LEN, timeoutCount);
  }

  delay(1000);
}
