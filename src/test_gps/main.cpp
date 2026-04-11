#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "UserPins.h"

// u-blox timeout (ms)
constexpr uint32_t UBLOX_CUSTOM_MAX_WAIT = 250ul;

TwoWire         i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
SFE_UBLOX_GNSS  m10s;

bool gps_ok = false;

void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("\n[TEST_GPS] u-blox M10S GPS Test");

  // Reset GPS
  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, 1);
  delay(20);
  digitalWrite(M10S_RESET, 0);
  delay(100);
  digitalWrite(M10S_RESET, 1);
  delay(500);

  i2c4.begin();

  Serial.println("[TEST_GPS] Initializing M10S...");
  gps_ok = m10s.begin(i2c4, 0x42);
  Serial.print("[TEST_GPS] Init: ");
  Serial.println(gps_ok ? "OK" : "FAIL");

  if (gps_ok) {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(18, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    Serial.println("[TEST_GPS] Config applied.");
  }

  Serial.println("[TEST_GPS] Starting read loop...\n");
  Serial.println("UTC\t\tSIV\tLat\t\tLon\t\tAlt(m)\tVN(m/s)\tVE(m/s)\tEpoch");
}

void loop() {
  if (!gps_ok) {
    Serial.println("[TEST_GPS] GPS not initialized.");
    delay(1000);
    return;
  }

  uint8_t  siv = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
  uint8_t  hh  = m10s.getHour(UBLOX_CUSTOM_MAX_WAIT);
  uint8_t  mm  = m10s.getMinute(UBLOX_CUSTOM_MAX_WAIT);
  uint8_t  ss  = m10s.getSecond(UBLOX_CUSTOM_MAX_WAIT);

  char utc[9];
  snprintf(utc, sizeof(utc), "%02d:%02d:%02d", hh, mm, ss);

  if (m10s.getPVT(UBLOX_CUSTOM_MAX_WAIT)) {
    uint32_t timestamp_us   = 0;
    uint32_t timestamp_epoch = m10s.getUnixEpoch(timestamp_us, UBLOX_CUSTOM_MAX_WAIT);

    double lat   = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    double lon   = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    double alt   = static_cast<double>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3;
    double vel_n = static_cast<double>(m10s.getNedNorthVel(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3;
    double vel_e = static_cast<double>(m10s.getNedEastVel(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3;

    Serial.print(utc);
    Serial.print("\t");
    Serial.print(siv);
    Serial.print("\t");
    Serial.print(lat, 7);
    Serial.print("\t");
    Serial.print(lon, 7);
    Serial.print("\t");
    Serial.print(alt, 2);
    Serial.print("\t");
    Serial.print(vel_n, 3);
    Serial.print("\t");
    Serial.print(vel_e, 3);
    Serial.print("\t");
    Serial.println(timestamp_epoch);
  } else {
    Serial.print(utc);
    Serial.print("\t");
    Serial.print(siv);
    Serial.println("\t(no PVT)");
  }

  delay(100);
}
