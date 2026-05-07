#include <Arduino.h>

// ---------------------------------------------------------------------------
// Backup SRAM (BKPSRAM) — STM32H725, 4 KB at 0x38800000
// Direct memory read/write: no Flash erase, no blocking, ~microseconds.
// Data is retained while VBAT is supplied.
// ---------------------------------------------------------------------------

static constexpr uintptr_t BKPSRAM_BASE = 0x38800000UL;

// Magic word that marks a valid block (DAEDALUS → 0xDAEDA105)
static constexpr uint32_t EEPROM_MAGIC = 0xDAEDA105u;

struct EEPROMStore {
  uint32_t magic;           // validity sentinel
  uint8_t  crc;             // CRC-8 over all bytes that follow
  // ---- protected payload ----
  char     utc[9];          // "HH:MM:SS\0"   UTC time from GPS
  uint32_t packet_count;    // telemetry packet counter
  uint8_t  state;           // UserState enum (FSM phase)
  double   alt_ref;         // ground reference altitude (m MSL)
  float    pos_a;           // deployment servo A position (deg)
  float    pos_b;           // deployment servo B position (deg)
  double   servo_angles[3]; // paraglider servo target angles (deg)
  float    bearing_ema_alpha;     // guidance: at
  float    ctrl_ema_alpha;        // guidance: at_ctrl
  float    drift_correction_gain; // guidance: DRIFT_CORRECTION_GAIN
  float    heading_deadband_deg;  // guidance: HEADING_DEADBAND_DEG
  uint8_t  wake_from_sleep;      // set before deepSleep(), cleared on next boot
};

static_assert(sizeof(EEPROMStore) <= 4096u, "EEPROMStore exceeds BKPSRAM size");


// CRC-8 / SMBUS — over payload region (everything after magic + crc)
static uint8_t eeprom_crc8(const uint8_t *buf, size_t len) {
  uint8_t crc = 0x00u;
  for (size_t i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1u) ^ 0x07u)
                          : static_cast<uint8_t>(crc << 1u);
  }
  return crc;
}

// Payload start: byte offset just past magic(4) + crc(1) = 5
static constexpr size_t EEPROM_PAYLOAD_OFFSET = sizeof(uint32_t) + sizeof(uint8_t);

// Call once at startup — enables BKPSRAM clock and write access
inline void EEPROM_Init() {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
}
