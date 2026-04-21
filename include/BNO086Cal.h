#pragma once
// BNO086 Magnetometer Offset Calibration & North-Lock
//
// Usage (via serial commands after "CMD,1043," prefix):
//   CAL,MAG,START   — collect 30 s of raw mag samples; computes hard-iron offsets
//   CAL,NORTH       — point sensor to magnetic north, then send this to lock 0°
//   CAL,STATUS      — print current calibration state and stored values
//   CAL,RESET       — erase stored calibration; revert to compile-time defaults
//
// Offset values and mount_offset are persisted in STM32H725 Backup SRAM so
// they survive power cycles (while VBAT is present).

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "config/Main/UserConfig.h"

// ── BKPSRAM layout ──────────────────────────────────────────────────────────
// EEPROMStore occupies the first ~80 bytes at 0x38800000.
// We start BNO086Cal storage at +256 to leave ample headroom.
static constexpr uintptr_t BNO_CAL_ADDR  = 0x38800000UL + 256UL;
static constexpr uint32_t  BNO_CAL_MAGIC = 0xBCA10086u;

struct BNO086CalStore {
  uint32_t magic;
  float    hard_iron[3];  // residual hard-iron centre (µT) — X, Y, Z
  float    mount_offset;  // degrees — runtime replacement for BNO_MOUNT_OFFSET
};
static_assert(sizeof(BNO086CalStore) <= 128u, "BNO086CalStore too large");

// ── Calibration state ────────────────────────────────────────────────────────
enum class BNO086CalState : uint8_t {
  IDLE           = 0,
  MAG_COLLECTING = 1,  // gathering calibrated-mag samples for 30 s
  AWAITING_NORTH = 2,  // collection done; rotation vector re-enabled; awaiting north cmd
};

// ── Calibrator class ─────────────────────────────────────────────────────────
class BNO086Calibrator {
public:
  volatile BNO086CalState state = BNO086CalState::IDLE;

  // Runtime values — loaded from BKPSRAM on init(), else compile-time defaults
  float hard_iron[3] = {0.f, 0.f, 0.f};  // µT
  float mount_offset = 0.f;              // degrees

  static constexpr uint32_t CAL_DURATION_MS = 30000ul;

  // ── init() ─────────────────────────────────────────────────────────────────
  // Call once from setup() after EEPROM_Init().
  // default_mount_offset: the compile-time BNO_MOUNT_OFFSET value.
  void init(float default_mount_offset) {
    const BNO086CalStore *s = _ptr();
    if (s->magic == BNO_CAL_MAGIC) {
      hard_iron[0] = s->hard_iron[0];
      hard_iron[1] = s->hard_iron[1];
      hard_iron[2] = s->hard_iron[2];
      mount_offset = s->mount_offset;
      Serial.printf(
        "[MAGCAL] Restored: HardIron=(%.3f, %.3f, %.3f) µT  MountOffset=%.2f°\n",
        hard_iron[0], hard_iron[1], hard_iron[2], mount_offset);
    } else {
      mount_offset = default_mount_offset;
      Serial.printf("[MAGCAL] No saved calibration — default MountOffset=%.2f°\n",
                    mount_offset);
    }
  }

  // ── startMagCollection() ───────────────────────────────────────────────────
  // Begin 30-second magnetometer offset measurement.
  // bno must be owned by the caller (e.g. inside mtx_i2c).
  void startMagCollection(BNO08x &bno) {
    _mn[0] = _mn[1] = _mn[2] =  1e9f;
    _mx[0] = _mx[1] = _mx[2] = -1e9f;
    _samples     = 0;
    _start_ms    = millis();
    bno.enableMagnetometer();   // calibrated field; report ID 0x03
    bno.enableRotationVector(); // keep heading running in parallel
    state = BNO086CalState::MAG_COLLECTING;
    Serial.println("[MAGCAL] ── Started ──────────────────────────────────────");
    Serial.println("[MAGCAL] Slowly rotate sensor in figure-8 / tumble motion");
    Serial.printf ("[MAGCAL] Duration: %lu s\n", CAL_DURATION_MS / 1000);
  }

  // ── update() ───────────────────────────────────────────────────────────────
  // Call from ReadMAG() on every getSensorEvent() while MAG_COLLECTING.
  // Returns true when collection finishes (caller should re-enable sensors).
  bool update(BNO08x &bno) {
    if (state != BNO086CalState::MAG_COLLECTING) return false;

    if (bno.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
      const float x = bno.getMagX();
      const float y = bno.getMagY();
      const float z = bno.getMagZ();

      _mn[0] = min(_mn[0], x);  _mx[0] = max(_mx[0], x);
      _mn[1] = min(_mn[1], y);  _mx[1] = max(_mx[1], y);
      _mn[2] = min(_mn[2], z);  _mx[2] = max(_mx[2], z);
      ++_samples;

      // Progress report every 5 s
      uint32_t elapsed = millis() - _start_ms;
      if (_samples % 50 == 0) {
        uint8_t pct = (uint8_t)min(100UL, elapsed * 100 / CAL_DURATION_MS);
        Serial.printf("[MAGCAL] %3u%%  samples=%lu  mag_acc=%u\n",
                      pct, (unsigned long)_samples, bno.getMagAccuracy());
      }
    }

    if (millis() - _start_ms >= CAL_DURATION_MS) {
      _finalize(bno);
      return true;
    }
    return false;
  }

  // ── setNorthLock() ─────────────────────────────────────────────────────────
  // Call when the sensor is physically pointing to magnetic north.
  // raw_yaw_deg: bno.getYaw() converted to degrees (before any offset applied).
  // After this call, state returns to IDLE and heading reads 0° = magnetic north.
  // Add MAGNETIC_DECLINATION in ReadMAG to convert to true north.
  void setNorthLock(float raw_yaw_deg) {
    // We want: raw_yaw + mount_offset = 0  (magnetic north)
    // So mount_offset = -raw_yaw_deg, wrapped to (-180, +180].
    // Then add SPOOL_PHYSICAL_OFFSET to align control frame with spool 1.
    mount_offset = fmod(-raw_yaw_deg + SPOOL_PHYSICAL_OFFSET + 360.0f, 360.0f);
    if (mount_offset > 180.0f) mount_offset -= 360.0f;

    _save();
    state = BNO086CalState::IDLE;

    Serial.println("[MAGCAL] ── North lock set ───────────────────────────────");
    Serial.printf ("[MAGCAL]   Raw yaw at lock   : %.2f°\n", raw_yaw_deg);
    Serial.printf ("[MAGCAL]   Spool phys. offset: %.2f°\n", (float)SPOOL_PHYSICAL_OFFSET);
    Serial.printf ("[MAGCAL]   New mount_offset  : %.2f°\n", mount_offset);
    Serial.printf ("[MAGCAL]   Saved to BKPSRAM.\n");
    Serial.printf ("[MAGCAL]   To make permanent, update UserConfig.h:\n");
    Serial.printf ("             constexpr double BNO_MOUNT_OFFSET = %.2f;\n",
                   (double)mount_offset);
  }

  // ── autoNorthLock() ────────────────────────────────────────────────────────
  // Call once in setup() after bno.begin() and bno_cal.init().
  // Polls the BNO086 (up to timeout_ms) for the first valid rotation vector,
  // then automatically locks that reading as magnetic north.
  // Works because BNO086 restores its DCD from sensor flash on every boot —
  // it already knows where magnetic north is without any manual command.
  void autoNorthLock(BNO08x &bno, uint32_t timeout_ms = 5000) {
    Serial.println("[MAGCAL] Auto north-lock — waiting for first heading...");

    // After cold boot the BNO086 restores DCD from its internal flash before
    // asserting INT and producing reports — this can take up to 2 s.
    delay(2000);
    bno.enableRotationVector();  // re-issue in case the sensor soft-reset during boot

    const uint32_t t0 = millis();
    uint32_t last_reenable = t0;

    while (millis() - t0 < timeout_ms) {
      // Re-send enableRotationVector() every 2 s in case the sensor reset
      if (millis() - last_reenable > 2000) {
        bno.enableRotationVector();
        last_reenable = millis();
        Serial.println("[MAGCAL] Re-enabling rotation vector...");
      }

      if (bno.getSensorEvent()) {
        if (bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
          const float raw_yaw = 90.0f - bno.getYaw() * 180.0f / PI;
          mount_offset = fmod(-raw_yaw + SPOOL_PHYSICAL_OFFSET + 360.0f, 360.0f);
          if (mount_offset > 180.0f) mount_offset -= 360.0f;
          _save();
          state = BNO086CalState::IDLE;
          Serial.printf("[MAGCAL] North locked: raw_yaw=%.2f°  spool_offset=%.2f°  mount_offset=%.2f°  (acc=%u/3)\n",
                        raw_yaw, (float)SPOOL_PHYSICAL_OFFSET, mount_offset, bno.getQuatAccuracy());
          return;
        }
      }
      delay(10);
    }

    Serial.println("[MAGCAL] Warning: timed out — using existing mount_offset");
  }

  // ── printStatus() ──────────────────────────────────────────────────────────
  void printStatus(BNO08x &bno) const {
    Serial.println("[MAGCAL] ── Status ──────────────────────────────────────");
    Serial.printf ("[MAGCAL]   State        : %s\n", _stateName());
    Serial.printf ("[MAGCAL]   HardIron X   : %.3f µT\n", hard_iron[0]);
    Serial.printf ("[MAGCAL]   HardIron Y   : %.3f µT\n", hard_iron[1]);
    Serial.printf ("[MAGCAL]   HardIron Z   : %.3f µT\n", hard_iron[2]);
    Serial.printf ("[MAGCAL]   MountOffset  : %.2f°\n", mount_offset);
    Serial.printf ("[MAGCAL]   MagAccuracy  : %u / 3\n", bno.getMagAccuracy());
    Serial.printf ("[MAGCAL]   QuatAccuracy : %u / 3\n", bno.getQuatAccuracy());
    if (state == BNO086CalState::MAG_COLLECTING) {
      uint32_t rem = CAL_DURATION_MS - min(millis() - _start_ms, CAL_DURATION_MS);
      Serial.printf("[MAGCAL]   Remaining    : %lu s  Samples: %lu\n",
                    rem / 1000, (unsigned long)_samples);
    }
    const BNO086CalStore *s = _ptr();
    Serial.printf ("[MAGCAL]   BKPSRAM valid: %s\n",
                   s->magic == BNO_CAL_MAGIC ? "YES" : "NO");
  }

  // ── reset() ────────────────────────────────────────────────────────────────
  void reset(float default_mount_offset) {
    _ptr()->magic = 0u;
    hard_iron[0] = hard_iron[1] = hard_iron[2] = 0.f;
    mount_offset = default_mount_offset;
    state = BNO086CalState::IDLE;
    Serial.printf("[MAGCAL] Cleared. MountOffset reset to %.2f°\n",
                  mount_offset);
  }

  bool isCollecting()    const { return state == BNO086CalState::MAG_COLLECTING; }
  bool isAwaitingNorth() const { return state == BNO086CalState::AWAITING_NORTH; }
  bool isIdle()          const { return state == BNO086CalState::IDLE; }

private:
  float    _mn[3], _mx[3];
  uint32_t _samples   = 0;
  uint32_t _start_ms  = 0;

  BNO086CalStore *_ptr() const {
    return reinterpret_cast<BNO086CalStore *>(BNO_CAL_ADDR);
  }

  void _finalize(BNO08x &bno) {
    // Hard-iron offset = centre of field ellipsoid
    for (int i = 0; i < 3; ++i)
      hard_iron[i] = (_mx[i] + _mn[i]) * 0.5f;

    // Field magnitude estimate (half-span on each axis → sphere radius)
    const float span_x = (_mx[0] - _mn[0]) * 0.5f;
    const float span_y = (_mx[1] - _mn[1]) * 0.5f;
    const float span_z = (_mx[2] - _mn[2]) * 0.5f;
    const float field_magnitude = (span_x + span_y + span_z) / 3.0f;

    Serial.println("[MAGCAL] ── Hard-iron offsets computed ─────────────────");
    Serial.printf ("[MAGCAL]   X: offset=%.3f µT  (min=%.3f  max=%.3f  span=%.3f)\n",
                   hard_iron[0], _mn[0], _mx[0], _mx[0] - _mn[0]);
    Serial.printf ("[MAGCAL]   Y: offset=%.3f µT  (min=%.3f  max=%.3f  span=%.3f)\n",
                   hard_iron[1], _mn[1], _mx[1], _mx[1] - _mn[1]);
    Serial.printf ("[MAGCAL]   Z: offset=%.3f µT  (min=%.3f  max=%.3f  span=%.3f)\n",
                   hard_iron[2], _mn[2], _mx[2], _mx[2] - _mn[2]);
    Serial.printf ("[MAGCAL]   Field magnitude (avg half-span): %.2f µT\n",
                   field_magnitude);
    Serial.printf ("[MAGCAL]   Samples: %lu   MagAccuracy: %u/3\n",
                   (unsigned long)_samples, bno.getMagAccuracy());

    bno.saveCalibration();  // persist BNO086 internal DCD to its flash
    Serial.println("[MAGCAL]   BNO086 DCD saved to sensor flash");

    _save();
    Serial.println("[MAGCAL]   Offsets saved to BKPSRAM");

    // Re-enable rotation vector for heading; keep magnetometer for accuracy monitoring
    bno.enableRotationVector();

    state = BNO086CalState::AWAITING_NORTH;
    Serial.println("[MAGCAL] ── North-lock required ────────────────────────");
    Serial.println("[MAGCAL]   1. Point the sensor to MAGNETIC NORTH");
    Serial.println("[MAGCAL]   2. Hold steady, then send: CMD,1043,CAL,NORTH");
  }

  void _save() {
    BNO086CalStore *s = _ptr();
    s->magic       = BNO_CAL_MAGIC;
    s->hard_iron[0] = hard_iron[0];
    s->hard_iron[1] = hard_iron[1];
    s->hard_iron[2] = hard_iron[2];
    s->mount_offset = mount_offset;
    __DSB();  // flush write buffer before returning
  }

  const char *_stateName() const {
    switch (state) {
      case BNO086CalState::IDLE:           return "IDLE";
      case BNO086CalState::MAG_COLLECTING: return "MAG_COLLECTING";
      case BNO086CalState::AWAITING_NORTH: return "AWAITING_NORTH";
      default:                             return "UNKNOWN";
    }
  }
};
