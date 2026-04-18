#include <Arduino.h>
#include <SPI.h>
#include "Controlling.h"
#include "UserPins.h"
#include "UserSensors.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config/Main/UserConfig.h"

// ── Interfaces ──────────────────────────────────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
HardwareSerial ServoSerial(USER_GPIO_Half);

// ── Sensors ──────────────────────────────────────────────────────────────────
Altimeter_BMP581 baro(USER_GPIO_BMP581_NSS);
BNO08x           bno;
SFE_UBLOX_GNSS   gnss;

// ── State ─────────────────────────────────────────────────────────────────────
GPSCoordinate current_pos{};
GPSCoordinate target_pos = {13.722992512087279, 100.51463610518176};

double alt_msl   = 0;
double alt_ref   = 0;
double alt_agl   = 0;
float  yaw_deg   = 0;
double vel_n     = 0;
double vel_e     = 0;
bool   gps_fixed = false;

Controller controller;

numeric_vector<3> servo_target_angles{};

// ── Helpers ───────────────────────────────────────────────────────────────────
static void setup_spi() {
  SPI.setMISO(USER_GPIO_SPI1_MISO);
  SPI.setMOSI(USER_GPIO_SPI1_MOSI);
  SPI.setSCLK(USER_GPIO_SPI1_SCK);
  SPI.setSSEL(NC);
  SPI.begin();
}

static void setup_i2c() {
  i2c4.setClock(100000);
  i2c4.begin();
}

static void read_baro() {
  if (baro.read()) {
    alt_msl = baro.altitude_m();
    alt_agl = alt_msl - alt_ref;
  }
}

static void read_gnss() {
  if (gnss.checkUblox()) {
    uint8_t siv     = gnss.getSIV();
    gps_fixed       = siv > 0;
    current_pos.lat = static_cast<double>(gnss.getLatitude()) * 1e-7;
    current_pos.lon = static_cast<double>(gnss.getLongitude()) * 1e-7;
    vel_n           = static_cast<double>(gnss.getNedNorthVel()) * 1e-3;
    vel_e           = static_cast<double>(gnss.getNedEastVel()) * 1e-3;
  }
  current_pos = {current_pos.lat, current_pos.lon};
}

static void read_mag() {
  if (bno.wasReset())
    bno.enableRotationVector();

  if (bno.getSensorEvent() &&
      bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    double y = bno.getYaw() * 180.0 / PI;
    yaw_deg  = static_cast<float>((y < 0) ? y + 360.0 : y);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("[TEST_PATH] boot");

  setup_spi();
  setup_i2c();

  // Servo
  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(10);
  controller.driver.hlscl.pSerial = &ServoSerial;
  controller.driver.hlscl.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 10);
  for (size_t i = 0; i < sizeof(ServoDriver::IDS); ++i)
    controller.driver.hlscl.WheelMode(ServoDriver::IDS[i]);
  controller.init_pid();
  Serial.println("[SERVO] OK");

  // BMP581
  if (baro.begin())
    Serial.println("[BARO] OK");
  else
    Serial.println("[BARO] FAIL");

  // BNO08x
  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, HIGH);
  if (bno.begin(BNO08X_ADDR, i2c4, USER_GPIO_BNO_INT1, BNO08X_RESET)) {
    bno.enableRotationVector();
    Serial.println("[BNO] OK");
  } else {
    Serial.println("[BNO] FAIL");
  }

  // u-blox M10S
  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, HIGH);
  delay(20);
  digitalWrite(M10S_RESET, LOW);
  delay(100);
  digitalWrite(M10S_RESET, HIGH);
  if (gnss.begin(i2c4, 0x42)) {
    gnss.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    gnss.setNavigationFrequency(10, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    gnss.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    gnss.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    Serial.println("[GNSS] OK");
  } else {
    Serial.println("[GNSS] FAIL");
  }

  // Grab ground altitude reference (avg 20 samples)
  Serial.println("[BARO] zeroing alt...");
  double acc_alt = 0;
  for (int i = 0; i < 20; ++i) {
    delay(50);
    if (baro.read())
      acc_alt += baro.altitude_m();
  }
  alt_ref = acc_alt / 20.0;
  Serial.print("[BARO] alt_ref = ");
  Serial.println(alt_ref, 2);

  Serial.println("T(s),LAT,LON,YAW,AGL,BEARING,DIST_M,DIST_RATIO,FIX,TGT0,CUR0,TGT1,CUR1,TGT2,CUR2");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // All I2C sensor reads and the servo UART are sequenced inside one tick so
  // they never overlap — I2C finishes completely before any servo byte is sent.
  static xcore::NbDelay main_tick(50, millis);
  static xcore::NbDelay print_tick(100, millis);

  main_tick([&]() {
    // 1. All I2C reads first — baro every tick, gnss+mag every 2nd tick (100ms)
    static uint8_t slow_div = 0;
    read_baro();
    if (++slow_div >= 2) {
      read_gnss();
      read_mag();
      slow_div = 0;
    }

    // 2. Servo UART — I2C is fully done before the first byte is sent
    if (gps_fixed)
      servo_target_angles = controller.guidance.update(
        current_pos, target_pos, alt_agl, vel_n, vel_e, yaw_deg);
    controller.servo_pid_update(servo_target_angles);
  });

  print_tick([&]() {
    double bearing = Guidance::calculate_bearing(current_pos, target_pos, yaw_deg);
    double dist    = Guidance::calculate_distance(current_pos, target_pos);
    double t       = millis() * 0.001;

    double dLat   = (target_pos.lat - current_pos.lat) * DEG_TO_RAD;
    double dLon   = (target_pos.lon - current_pos.lon) * DEG_TO_RAD;
    double a      = std::sin(dLat / 2) * std::sin(dLat / 2) +
                    std::cos(current_pos.lat * DEG_TO_RAD) *
                    std::cos(target_pos.lat  * DEG_TO_RAD) *
                    std::sin(dLon / 2) * std::sin(dLon / 2);
    double dist_m = EARTH_RADIUS_M * 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    Serial.print(t, 2);
    Serial.print(',');
    Serial.print(current_pos.lat, 7);
    Serial.print(',');
    Serial.print(current_pos.lon, 7);
    Serial.print(',');
    Serial.print(yaw_deg, 1);
    Serial.print(',');
    Serial.print(alt_agl, 1);
    Serial.print(',');
    Serial.print(bearing, 1);
    Serial.print(',');
    Serial.print(dist_m, 2);
    Serial.print(',');
    Serial.print(dist, 4);
    Serial.print(',');
    Serial.print(gps_fixed ? 1 : 0);
    for (size_t i = 0; i < 3; ++i) {
      Serial.print(',');
      Serial.print(servo_target_angles[i], 2);
      Serial.print(',');
      Serial.print(controller.last_angles[i], 2);
    }
    Serial.println();
  });
}
