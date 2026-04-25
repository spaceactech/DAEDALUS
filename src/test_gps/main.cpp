#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "UserPins.h"

// ─── Bus & sensor instances ───────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
SFE_UBLOX_GNSS m10s;

// ─── Config ───────────────────────────────────────────────────
static constexpr uint8_t  GPS_ADDR         = 0x42;
static constexpr uint32_t GPS_NAV_FREQ_HZ  = 10;
static constexpr uint16_t MAX_WAIT_MS      = 1100;
static constexpr uint16_t NUM_SAMPLES      = 100;

void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("\n=== M10S Airborne-4g GPS Test ===");

  // ── Reset M10S ───────────────────────────────────────────────
  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, HIGH);
  delay(20);
  digitalWrite(M10S_RESET, LOW);
  delay(100);
  digitalWrite(M10S_RESET, HIGH);
  delay(500);

  // ── I2C ─────────────────────────────────────────────────────
  i2c4.setClock(400000);
  i2c4.begin();
  i2c4.setTimeout(50);

  // ── Init M10S ────────────────────────────────────────────────
  Serial.print("[GPS] Init ... ");
  if (!m10s.begin(i2c4, GPS_ADDR)) {
    Serial.println("FAIL — check wiring");
    while (true) delay(1000);
  }
  Serial.println("OK");

  m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, MAX_WAIT_MS);
  m10s.setNavigationFrequency(GPS_NAV_FREQ_HZ, VAL_LAYER_RAM_BBR, MAX_WAIT_MS);
  m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, MAX_WAIT_MS);

  // ── Airborne 4g dynamic model ────────────────────────────────
  Serial.print("[GPS] Dynamic model → AIRBORNE_4g ... ");
  if (m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, MAX_WAIT_MS))
    Serial.println("OK");
  else
    Serial.println("FAIL (continuing anyway)");

  // ── Verify model readback ────────────────────────────────────
  const uint8_t model = m10s.getDynamicModel(VAL_LAYER_RAM, MAX_WAIT_MS);
  Serial.print("[GPS] Confirmed dynamic model: ");
  Serial.println(model);  // 8 = AIRBORNE_4g

  Serial.println("\nWaiting for GPS fix ...");
  Serial.println("n,lat,lon,alt_msl_m,vN_ms,vE_ms,vD_ms,siv,fix,hAcc_mm,vAcc_mm,sAcc_mm");
}

void loop() {
  static uint16_t sample_n   = 0;
  static bool     done       = false;
  static uint32_t last_ms    = 0;

  if (done) return;

  // getPVT() blocks until a fresh NAV-PVT packet arrives (or times out).
  if (!m10s.getPVT(MAX_WAIT_MS)) return;

  const uint8_t fix = m10s.getFixType();
  const uint8_t siv = m10s.getSIV();

  if (siv < 4) return;

  // Rate-limit to one sample per second
  const uint32_t now = millis();
  if (now - last_ms < 1000) return;
  last_ms = now;

  ++sample_n;

  const double   lat     = static_cast<double>(m10s.getLatitude())    * 1.e-7;
  const double   lon     = static_cast<double>(m10s.getLongitude())   * 1.e-7;
  const double   alt_msl = static_cast<double>(m10s.getAltitudeMSL()) * 1.e-3;
  const double   vN      = static_cast<double>(m10s.getNedNorthVel()) * 1.e-3;
  const double   vE      = static_cast<double>(m10s.getNedEastVel())  * 1.e-3;
  const double   vD      = static_cast<double>(m10s.getNedDownVel())  * 1.e-3;
  const uint32_t hAcc    = m10s.getHorizontalAccEst();  // mm
  const uint32_t vAcc    = m10s.getVerticalAccEst();    // mm
  const uint32_t sAcc    = m10s.getSpeedAccEst();       // mm/s

  Serial.print(sample_n);   Serial.print(',');
  Serial.print(lat, 7);     Serial.print(',');
  Serial.print(lon, 7);     Serial.print(',');
  Serial.print(alt_msl, 3); Serial.print(',');
  Serial.print(vN, 4);      Serial.print(',');
  Serial.print(vE, 4);      Serial.print(',');
  Serial.print(vD, 4);      Serial.print(',');
  Serial.print(siv);        Serial.print(',');
  Serial.print(fix);        Serial.print(',');
  Serial.print(hAcc);       Serial.print(',');
  Serial.print(vAcc);       Serial.print(',');
  Serial.println(sAcc);

  if (sample_n >= NUM_SAMPLES) {
    done = true;
    Serial.println("\n=== 100 samples collected — done ===");
  }
}
