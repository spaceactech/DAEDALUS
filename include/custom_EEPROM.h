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
  double   alt_ref;         // ground reference altitude (m MSL)
  float    pos_a;           // deployment servo A position (deg)
  float    pos_b;           // deployment servo B position (deg)
  double   servo_angles[3]; // paraglider servo target angles (deg)
  uint8_t  wake_from_sleep; // set before deepSleep(), cleared on next boot
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

// ---------------------------------------------------------------------------
// Flash Config Store — persistent across full power loss (no VBAT required)
// Stores flight state and guidance parameters.
//
// Target sector (STM32H725xG, 1 MB single-bank):
//   Bank 1, Sector 7 → 0x080E0000 (128 KB, last sector)
// For STM32H725xE (512 KB) use FLASH_SECTOR_3 → 0x08060000
// ---------------------------------------------------------------------------

static constexpr uintptr_t FLASH_CONFIG_ADDR   = 0x080E0000UL;
static constexpr uint32_t  FLASH_CONFIG_BANK   = FLASH_BANK_1;
static constexpr uint32_t  FLASH_CONFIG_SECTOR = FLASH_SECTOR_7;
static constexpr uint32_t  FLASH_CONFIG_MAGIC  = 0xDAEDA106u;

// Padded to 32 bytes (one STM32H7 flash word) and 32-byte aligned.
struct __attribute__((aligned(32))) FlashConfig {
  uint32_t magic;
  uint8_t  crc;
  uint8_t  state;               // UserState enum (FSM phase)
  uint8_t  _pad[2];             // align float fields to 4-byte boundary
  float    bearing_ema_alpha;   // guidance: at
  float    ctrl_ema_alpha;      // guidance: at_ctrl
  float    heading_deadband_deg;
  uint8_t  _pad2[12];           // pad to exactly 32 bytes
};
static_assert(sizeof(FlashConfig) == 32u, "FlashConfig must be exactly 32 bytes");

inline bool FLASH_Config_Read(FlashConfig &out) {
  memcpy(&out, reinterpret_cast<const void *>(FLASH_CONFIG_ADDR), sizeof(FlashConfig));
  if (out.magic != FLASH_CONFIG_MAGIC) return false;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&out) + EEPROM_PAYLOAD_OFFSET;
  return out.crc == eeprom_crc8(p, sizeof(FlashConfig) - EEPROM_PAYLOAD_OFFSET);
}

// Erases one 128 KB sector (~100 ms) then programs one 32-byte flash word.
// Do not call from interrupt context or during time-critical operations.
inline bool FLASH_Config_Write(const FlashConfig &src) {
  FlashConfig cfg = src;
  cfg.magic = FLASH_CONFIG_MAGIC;
  memset(cfg._pad,  0, sizeof(cfg._pad));
  memset(cfg._pad2, 0, sizeof(cfg._pad2));
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&cfg) + EEPROM_PAYLOAD_OFFSET;
  cfg.crc = eeprom_crc8(p, sizeof(FlashConfig) - EEPROM_PAYLOAD_OFFSET);

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef er = {};
  er.TypeErase    = FLASH_TYPEERASE_SECTORS;
  er.Banks        = FLASH_CONFIG_BANK;
  er.Sector       = FLASH_CONFIG_SECTOR;
  er.NbSectors    = 1u;
  er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t sector_err = 0u;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &sector_err);
  if (st != HAL_OK) { HAL_FLASH_Lock(); return false; }

  st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                         static_cast<uint32_t>(FLASH_CONFIG_ADDR),
                         reinterpret_cast<uint32_t>(&cfg));
  HAL_FLASH_Lock();
  return st == HAL_OK;
}
