#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <lib_xcore>
#include <xcore/math_module>
#include <cmath>

#include "UserPins.h"
#include "custom_kalman.h"

// ─── Bus & sensor instances ───────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
SFE_UBLOX_GNSS m10s;

// ─── Config ───────────────────────────────────────────────────
static constexpr uint8_t  GPS_ADDR        = 0x42;
static constexpr uint32_t GPS_NAV_FREQ_HZ = 10;
static constexpr uint16_t MAX_WAIT_MS     = 1100;
static constexpr uint16_t NUM_SAMPLES     = 200;
static constexpr double   DT              = 1.0 / GPS_NAV_FREQ_HZ;  // 0.1 s

// ─── GPS Kalman filters (mirrors main/main.cpp filter_nav_n / filter_nav_e) ──
// North axis: states [lat_deg, vN_m_s, aN_m_s2]
// East  axis: states [lon_deg, vE_m_s, aE_m_s2]
xcore::vdt<FILTER_ORDER - 1> vdt(DT);
FilterGPS filter_nav_n;
FilterGPS filter_nav_e;

void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("\n=== M10S GPS Kalman Filter Comparison ===");

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

  // ── Init KF transition matrices — same pattern as main/main.cpp setup() ──
  {
    auto F = vdt.generate_F();
    F[0][1] /= GPS_R_LAT;
    F[0][2] /= GPS_R_LAT;
    filter_nav_n.F = F;
    filter_nav_e.F = F;  // east starts with equatorial approx; updated on first GPS fix
  }

  Serial.println("\nWaiting for GPS fix ...");
  Serial.println(
    "n,"
    "lat_raw,lon_raw,alt_msl_m,"
    "vN_raw_ms,vE_raw_ms,"
    "lat_kf,lon_kf,"
    "vN_kf_ms,vE_kf_ms,"
    "dN_m,dE_m,"      // raw - filtered position, converted to metres
    "dvN_ms,dvE_ms,"  // raw - filtered velocity
    "siv,fix,hAcc_mm,vAcc_mm,sAcc_mm"
  );
}

void loop() {
  static uint16_t sample_n = 0;
  static bool     done     = false;

  if (done) return;

  // getPVT() blocks until a fresh NAV-PVT packet arrives (or times out)
  if (!m10s.getPVT(MAX_WAIT_MS)) return;

  const uint8_t fix = m10s.getFixType();
  const uint8_t siv = m10s.getSIV();

  if (siv < 4) return;

  // ── Raw GPS measurements ─────────────────────────────────────
  const double   lat  = static_cast<double>(m10s.getLatitude())    * 1.e-7;
  const double   lon  = static_cast<double>(m10s.getLongitude())   * 1.e-7;
  const double   alt  = static_cast<double>(m10s.getAltitudeMSL()) * 1.e-3;
  const double   vN   = static_cast<double>(m10s.getNedNorthVel()) * 1.e-3;
  const double   vE   = static_cast<double>(m10s.getNedEastVel())  * 1.e-3;
  const uint32_t hAcc = m10s.getHorizontalAccEst();  // mm
  const uint32_t vAcc = m10s.getVerticalAccEst();    // mm
  const uint32_t sAcc = m10s.getSpeedAccEst();       // mm/s

  // ── Update East F with current latitude — mirrors CB_ReadGNSS in main.cpp ─
  {
    auto         F_e   = vdt.generate_F();
    const double R_lon = GPS_R_LAT * std::cos(lat * (PI / 180.0));
    F_e[0][1] /= R_lon;
    F_e[0][2] /= R_lon;
    filter_nav_e.F = F_e;
  }

  // ── Predict → Update — same order as CB_EvalFSM + CB_ReadGNSS in main.cpp ─
  filter_nav_n.kf.predict();
  filter_nav_e.kf.predict();
  filter_nav_n.kf.update({lat, vN});
  filter_nav_e.kf.update({lon, vE});

  // ── Snapshot filtered states — mirrors the KF snapshot in CB_ReadGNSS ─────
  const double kf_lat = filter_nav_n.kf.state_vector()[0];
  const double kf_lon = filter_nav_e.kf.state_vector()[0];
  const double kf_vN  = filter_nav_n.kf.state_vector()[1];
  const double kf_vE  = filter_nav_e.kf.state_vector()[1];

  // ── Delta: raw − filtered, position converted to metres ──────────────────
  const double R_lon = GPS_R_LAT * std::cos(lat * (PI / 180.0));
  const double dN_m  = (lat - kf_lat) * GPS_R_LAT;
  const double dE_m  = (lon - kf_lon) * R_lon;
  const double dvN   = vN - kf_vN;
  const double dvE   = vE - kf_vE;

  ++sample_n;

  Serial.print(sample_n);   Serial.print(',');
  // Raw
  Serial.print(lat, 7);     Serial.print(',');
  Serial.print(lon, 7);     Serial.print(',');
  Serial.print(alt, 3);     Serial.print(',');
  Serial.print(vN, 4);      Serial.print(',');
  Serial.print(vE, 4);      Serial.print(',');
  // Filtered
  Serial.print(kf_lat, 7);  Serial.print(',');
  Serial.print(kf_lon, 7);  Serial.print(',');
  Serial.print(kf_vN, 4);   Serial.print(',');
  Serial.print(kf_vE, 4);   Serial.print(',');
  // Delta (raw - filtered)
  Serial.print(dN_m, 4);    Serial.print(',');
  Serial.print(dE_m, 4);    Serial.print(',');
  Serial.print(dvN, 4);     Serial.print(',');
  Serial.print(dvE, 4);     Serial.print(',');
  // Diagnostics
  Serial.print(siv);        Serial.print(',');
  Serial.print(fix);        Serial.print(',');
  Serial.print(hAcc);       Serial.print(',');
  Serial.print(vAcc);       Serial.print(',');
  Serial.println(sAcc);

  if (sample_n >= NUM_SAMPLES) {
    done = true;
    Serial.print("\n=== ");
    Serial.print(NUM_SAMPLES);
    Serial.println(" samples collected — done ===");
  }
}
