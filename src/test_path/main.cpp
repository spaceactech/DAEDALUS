#include <Arduino.h>
#include <SPI.h>
#include "Controlling.h"
#include "UserPins.h"
#include "UserSensors.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include "config/Main/UserConfig.h"

// ── Simulation ────────────────────────────────────────────────────────────────
// Flip to false to use real hardware sensors.
constexpr bool SIM_ENABLE = false;

namespace Sim {
  // Starting ~57 m NE of target at 100 m AGL — matches DISTANCE_TO_TARGET
  constexpr double START_LAT    = 13.722992512087279 + 0.000420;  // +46.7 m N
  constexpr double START_LON    = 100.51463610518176 + 0.000300;  // +32.5 m E
  constexpr double START_ALT    = 100.0;  // m AGL
  constexpr double DESCENT_RATE = 2.0;    // m/s
  constexpr float  YAW_SPIN     = 15.0;   // deg/s — parachute rotation rate
  constexpr double INIT_VN      = -0.9;   // m/s southward (toward target)
  constexpr double INIT_VE      = -0.6;   // m/s westward (toward target)

  static double lat = START_LAT;
  static double lon = START_LON;
  static double alt = START_ALT;
  static double vn  = INIT_VN;
  static double ve  = INIT_VE;
  static float  yaw = 30.0;

  static void update(double dt) {
    lat += (vn * dt) / EARTH_RADIUS_M * RAD_TO_DEG;
    lon += (ve * dt) / (EARTH_RADIUS_M * std::cos(lat * DEG_TO_RAD)) * RAD_TO_DEG;
    alt -= DESCENT_RATE * dt;
    if (alt < 0.0) alt = 0.0;
    yaw = std::fmod(yaw + YAW_SPIN * dt, 360.0f);
  }
}

// ── Interfaces ──────────────────────────────────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
HardwareSerial ServoSerial(USER_GPIO_Half);
HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

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
  Xbee.begin(115200);
  delay(3000);
  Xbee.println(SIM_ENABLE ? "[TEST_PATH] boot (SIM)" : "[TEST_PATH] boot");

  setup_spi();
  setup_i2c();

  // Servo — always needed
  ServoSerial.begin(1'000'000);
  ServoSerial.setTimeout(2);
  controller.driver.hlscl.pSerial = &ServoSerial;
  controller.driver.hlscl.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 2);
  for (size_t i = 0; i < sizeof(ServoDriver::IDS); ++i)
    controller.driver.hlscl.WheelMode(ServoDriver::IDS[i]);
  controller.init_pid();
  Xbee.println("[SERVO] OK");

  if (!SIM_ENABLE) {
    // BMP581
    if (baro.begin())
      Xbee.println("[BARO] OK");
    else
      Xbee.println("[BARO] FAIL");

    // BNO08x
    pinMode(BNO08X_RESET, OUTPUT);
    digitalWrite(BNO08X_RESET, HIGH);
    if (bno.begin(BNO08X_ADDR, i2c4, USER_GPIO_BNO_INT1, BNO08X_RESET)) {
      bno.enableRotationVector();
      Xbee.println("[BNO] OK");
    } else {
      Xbee.println("[BNO] FAIL");
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
      Xbee.println("[GNSS] OK");
    } else {
      Xbee.println("[GNSS] FAIL");
    }

    // Grab ground altitude reference (avg 20 samples)
    Xbee.println("[BARO] zeroing alt...");
    double acc_alt = 0;
    for (int i = 0; i < 20; ++i) {
      delay(50);
      if (baro.read())
        acc_alt += baro.altitude_m();
    }
    alt_ref = acc_alt / 20.0;
    Xbee.print("[BARO] alt_ref = ");
    Xbee.println(alt_ref, 2);
  }

  Xbee.println("T(s),LAT,LON,YAW,AGL,BEARING,DIST_M,DIST_RATIO,FIX,TGT0,CUR0,TGT1,CUR1,TGT2,CUR2");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // I2C sensor reads run at 50 ms; servo PID runs at 5 ms via its own NbDelay.
  // Guidance is updated inside main_tick so I2C is fully done before servo bytes are sent.
  static xcore::NbDelay main_tick(50, millis);
  static xcore::NbDelay print_tick(100, millis);

  main_tick([&]() {
    if (SIM_ENABLE) {
      Sim::update(0.05);
      current_pos = {Sim::lat, Sim::lon};
      alt_agl     = Sim::alt;
      yaw_deg     = Sim::yaw;
      vel_n       = Sim::vn;
      vel_e       = Sim::ve;
      gps_fixed   = Sim::alt > 0.0;
    } else {
      // 1. All I2C reads first — baro every tick, gnss+mag every 2nd tick (100 ms)
      static uint8_t slow_div = 0;
      read_baro();
      if (++slow_div >= 2) {
        read_gnss();
        read_mag();
        slow_div = 0;
      }
    }

    // 2. Guidance update — sensor data is ready before this runs
    if (true)
      servo_target_angles = controller.guidance.update(
        current_pos, target_pos, alt_agl, vel_n, vel_e, yaw_deg);
  });

  // Servo PID runs at 5 ms, gated by internal NbDelay — called every loop() iteration
  controller.servo_pid_update(servo_target_angles);

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
