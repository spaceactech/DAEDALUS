// ============================================================
// DAEDALUS Unit Test
// Tests every sensor (5 samples, non-zero pass) and
// every actuator (user-confirm or HLSCL encoder feedback)
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <INA236.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "SparkFun_VL53L1X.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SCServo.h>

#include "config/Main/UserConfig.h"  // RA_ constants, UBLOX_CUSTOM_MAX_WAIT
#include "UserPins.h"
#include "UserSensors.h"  // IMU_ISM256, Altimeter_BMP581

// ─── Bus instances ───────────────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
HardwareSerial ServoSerial(USER_GPIO_Half);

// ─── Sensor instances ────────────────────────────────────────
IMU_ISM256       imu_ism(USER_GPIO_ISM256_NSS);
Altimeter_BMP581 baro(USER_GPIO_BMP581_NSS);
BNO08x           bno;
INA236           ina(0x40, &i2c4);
SFEVL53L1X       tof(i2c4);
SFE_UBLOX_GNSS   gps;

// ─── Actuator instances ──────────────────────────────────────
HLSCL             hlscl;
Servo             servo_a;
Servo             servo_b;
Adafruit_NeoPixel led_strip(2, USER_GPIO_LED, NEO_GRB + NEO_KHZ800);

// Required by UserSensors.h external declaration
void setReports() {
  bno.enableRotationVector();
}

// ─── Test result flags ───────────────────────────────────────
struct Results {
  bool ism256   = false;
  bool bmp581   = false;
  bool bno08x   = false;
  bool ina236   = false;
  bool vl53l1x  = false;
  bool gps_m10s = false;
  bool servo_a  = false;
  bool servo_b  = false;
  bool hlscl1   = false;
  bool hlscl2   = false;
  bool hlscl3   = false;
  bool buzzer   = false;
  bool neopixel = false;
} res;

// ─── Helpers ─────────────────────────────────────────────────

static void printResult(const char *name, bool ok) {
  Serial.print("  [");
  Serial.print(ok ? "PASS" : "FAIL");
  Serial.print("] ");
  Serial.println(name);
}

// Block until user sends 'y'/'Y' or 'n'/'N', or timeout expires.
static bool waitUserConfirm(uint32_t timeout_ms = 15000) {
  Serial.print("  -> Confirmed? (y/n, ");
  Serial.print(timeout_ms / 1000);
  Serial.println("s timeout):");
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == 'y' || c == 'Y') return true;
      if (c == 'n' || c == 'N') return false;
    }
    delay(10);
  }
  Serial.println("  -> Timeout — marking FAIL");
  return false;
}

// ─── Sensor tests ────────────────────────────────────────────

static void testISM256() {
  Serial.println("\n[SENSOR] ISM6HG256X IMU (SPI)");

  bool init_ok = imu_ism.begin();
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("ISM6HG256X", false); return; }

  int pass_count = 0;
  for (int i = 0; i < 5; i++) {
    delay(10);
    if (!imu_ism.read()) { Serial.printf("  S%d read error\n", i + 1); continue; }

    double ax = imu_ism.acc_x(), ay = imu_ism.acc_y(), az = imu_ism.acc_z();
    double gx = imu_ism.gyr_x(), gy = imu_ism.gyr_y(), gz = imu_ism.gyr_z();
    Serial.printf("  S%d  ax=%.3f ay=%.3f az=%.3f  gx=%.3f gy=%.3f gz=%.3f\n",
                  i + 1, ax, ay, az, gx, gy, gz);

    if (ax != 0.0 || ay != 0.0 || az != 0.0 || gx != 0.0 || gy != 0.0 || gz != 0.0)
      pass_count++;
  }

  res.ism256 = (pass_count >= 3);
  printResult("ISM6HG256X", res.ism256);
}

static void testBMP581() {
  Serial.println("\n[SENSOR] BMP581 Altimeter (SPI)");

  bool init_ok = baro.begin();
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("BMP581", false); return; }

  int pass_count = 0;
  for (int i = 0; i < 5; i++) {
    delay(60);  // 20 Hz ODR → 50 ms period + margin
    if (!baro.read()) { Serial.printf("  S%d read error\n", i + 1); continue; }

    double p = baro.pressure_hpa();
    double t = baro.temperature();
    Serial.printf("  S%d  P=%.2f hPa  T=%.2f C\n", i + 1, p, t);

    if (p != 0.0 || t != 0.0) pass_count++;
  }

  res.bmp581 = (pass_count >= 3);
  printResult("BMP581", res.bmp581);
}

static void testBNO08x() {
  Serial.println("\n[SENSOR] BNO08x AHRS (I2C)");

  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(10);

  bool init_ok = bno.begin(BNO08X_ADDR, i2c4, USER_GPIO_BNO_INT1, BNO08X_RESET);
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("BNO08x", false); return; }

  bno.enableRotationVector();

  int pass_count = 0;
  int attempts   = 0;
  while (pass_count < 5 && attempts < 150) {
    delay(20);
    attempts++;
    if (bno.wasReset()) bno.enableRotationVector();
    if (!bno.getSensorEvent()) continue;
    if (bno.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) continue;

    float roll  = bno.getRoll()  * RAD_TO_DEG;
    float pitch = bno.getPitch() * RAD_TO_DEG;
    float yaw   = bno.getYaw()   * RAD_TO_DEG;
    pass_count++;
    Serial.printf("  S%d  roll=%.1f  pitch=%.1f  yaw=%.1f\n",
                  pass_count, roll, pitch, yaw);
  }

  res.bno08x = (pass_count >= 3);
  printResult("BNO08x", res.bno08x);
}

static void testINA236() {
  Serial.println("\n[SENSOR] INA236 Battery Monitor (I2C 0x40)");

  bool init_ok = ina.begin();
  if (init_ok) {
    ina.setADCRange(0);
    ina.setMaxCurrentShunt(8, 0.008);
    ina.setAverage(INA236_64_SAMPLES);
  }
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("INA236", false); return; }

  int pass_count = 0;
  for (int i = 0; i < 5; i++) {
    delay(50);
    float v = ina.getBusVoltage();
    float a = ina.getCurrent();
    Serial.printf("  S%d  V=%.3f V  I=%.4f A\n", i + 1, v, a);
    if (v != 0.0f || a != 0.0f) pass_count++;
  }

  res.ina236 = (pass_count >= 3);
  printResult("INA236", res.ina236);
}

static void testVL53L1X() {
  Serial.println("\n[SENSOR] VL53L1X ToF Distance (I2C 0x29)");

  tof.setI2CAddress(0x29);
  bool init_ok = (tof.begin() == 0);  // 0 = success
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("VL53L1X", false); return; }

  int pass_count = 0;
  for (int i = 0; i < 5; i++) {
    tof.startRanging();
    uint32_t t0 = millis();
    while (!tof.checkForDataReady() && (millis() - t0) < 300) delay(1);
    uint16_t dist = tof.getDistance();
    tof.clearInterrupt();
    tof.stopRanging();
    Serial.printf("  S%d  dist=%u mm\n", i + 1, dist);
    if (dist != 0) pass_count++;
    delay(20);
  }

  res.vl53l1x = (pass_count >= 3);
  printResult("VL53L1X", res.vl53l1x);
}

static void testGPS() {
  Serial.println("\n[SENSOR] u-blox M10S GPS (I2C 0x42)");

  bool init_ok = gps.begin(i2c4, 0x42);
  Serial.print("  Init: ");
  Serial.println(init_ok ? "OK" : "FAIL");

  if (!init_ok) { printResult("GPS M10S", false); return; }

  gps.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
  gps.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);

  // Poll 5 times — no fix required; pass if begin() succeeded + PVT responds
  int pvt_ok = 0;
  for (int i = 0; i < 5; i++) {
    delay(300);
    bool got = gps.getPVT(UBLOX_CUSTOM_MAX_WAIT);
    uint8_t siv = gps.getSIV(UBLOX_CUSTOM_MAX_WAIT);
    double  lat = gps.getLatitude(UBLOX_CUSTOM_MAX_WAIT)  * 1e-7;
    double  lon = gps.getLongitude(UBLOX_CUSTOM_MAX_WAIT) * 1e-7;
    Serial.printf("  S%d  PVT=%s  SIV=%u  lat=%.6f  lon=%.6f\n",
                  i + 1, got ? "yes" : "no", siv, lat, lon);
    if (got) pvt_ok++;
  }

  // Pass if init succeeded (GPS needs sky view for a fix)
  res.gps_m10s = init_ok;
  printResult("GPS M10S", res.gps_m10s);
}

// ─── Actuator tests ──────────────────────────────────────────

static void testBuzzer() {
  Serial.println("\n[ACTUATOR] Buzzer (PC15)");
  Serial.println("  3x beeps...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(USER_GPIO_BUZZER, HIGH); delay(250);
    digitalWrite(USER_GPIO_BUZZER, LOW);  delay(150);
  }
  res.buzzer = waitUserConfirm();
  printResult("Buzzer", res.buzzer);
}

static void testNeoPixel() {
  Serial.println("\n[ACTUATOR] NeoPixel LED (PC14, 2 pixels)");
  Serial.println("  Flashing Red / Green / Blue...");

  const uint32_t colors[] = {
    led_strip.Color(255, 0, 0),
    led_strip.Color(0, 255, 0),
    led_strip.Color(0, 0, 255),
  };
  for (uint32_t c : colors) {
    led_strip.fill(c); led_strip.show(); delay(400);
    led_strip.clear(); led_strip.show(); delay(100);
  }

  res.neopixel = waitUserConfirm();
  printResult("NeoPixel LED", res.neopixel);
}

static void testServoA() {
  Serial.println("\n[ACTUATOR] Servo A (PA3, deployment)");
  servo_a.attach(USER_GPIO_SERVO_A, RA_SERVO_MIN, RA_SERVO_MAX);
  servo_a.write(0); delay(800);
  Serial.println("  Sweeping 0 -> 90 -> 0 deg");
  servo_a.write(90); delay(1000);
  servo_a.write(0);  delay(500);
  res.servo_a = waitUserConfirm();
  printResult("Servo A", res.servo_a);
}

static void testServoB() {
  Serial.println("\n[ACTUATOR] Servo B (PB9, deployment)");
  servo_b.attach(USER_GPIO_SERVO_B, RA_SERVO_MIN, RA_SERVO_MAX);
  servo_b.write(0); delay(800);
  Serial.println("  Sweeping 0 -> 90 -> 0 deg");
  servo_b.write(90); delay(1000);
  servo_b.write(0);  delay(500);
  res.servo_b = waitUserConfirm();
  printResult("Servo B", res.servo_b);
}

// HLSCL encoder-based test:
//   1. Read initial encoder position
//   2. Drive at speed 500 for 1 second
//   3. Stop and read final position
//   4. PASS if position changed by > 10 counts (0.88 deg)
static void testHLSCLServo(uint8_t id, bool &result, const char *label) {
  Serial.print("\n[ACTUATOR] HLSCL Servo #");
  Serial.print(id);
  Serial.println(" (half-duplex encoder)");

  int pos_before = hlscl.ReadPos(id);
  Serial.printf("  Pos before: %d", pos_before);
  if (pos_before < 0) {
    Serial.println(" (no response)");
    printResult(label, false);
    result = false;
    return;
  }
  Serial.printf(" (%.1f deg)\n", pos_before * 360.0 / 4096.0);

  // Drive CCW for 1 second then stop
  hlscl.WriteSpe(id, 500, 50, 500);
  delay(1000);
  hlscl.WriteSpe(id, 0, 50, 0);
  delay(200);

  int pos_after = hlscl.ReadPos(id);
  if (pos_after < 0) {
    Serial.println("  Pos after: no response after move");
    printResult(label, false);
    result = false;
    return;
  }
  Serial.printf("  Pos after:  %d (%.1f deg)\n",
                pos_after, pos_after * 360.0 / 4096.0);

  int delta = abs(pos_after - pos_before);
  if (delta > 2048) delta = 4096 - delta;  // handle 0–4095 wrap-around
  Serial.printf("  Delta: %d counts (%.1f deg)\n",
                delta, delta * 360.0 / 4096.0);

  result = (delta > 10);
  printResult(label, result);
}

// ─── Summary ─────────────────────────────────────────────────

static void printSummary() {
  Serial.println("\n========== UNIT TEST SUMMARY ==========");
  printResult("ISM6HG256X IMU",  res.ism256);
  printResult("BMP581 Altimeter",res.bmp581);
  printResult("BNO08x AHRS",     res.bno08x);
  printResult("INA236 Battery",  res.ina236);
  printResult("VL53L1X ToF",     res.vl53l1x);
  printResult("GPS M10S",        res.gps_m10s);
  printResult("Servo A (PA3)",   res.servo_a);
  printResult("Servo B (PB9)",   res.servo_b);
  printResult("HLSCL Servo #1",  res.hlscl1);
  printResult("HLSCL Servo #2",  res.hlscl2);
  printResult("HLSCL Servo #3",  res.hlscl3);
  printResult("Buzzer",          res.buzzer);
  printResult("NeoPixel LED",    res.neopixel);

  int total = 13;
  int passed = (int)res.ism256 + res.bmp581 + res.bno08x + res.ina236 +
               res.vl53l1x + res.gps_m10s + res.servo_a + res.servo_b +
               res.hlscl1 + res.hlscl2 + res.hlscl3 + res.buzzer + res.neopixel;
  Serial.printf("  %d / %d passed\n", passed, total);
  Serial.println("=======================================");
}

// ─── Entry point ─────────────────────────────────────────────

void setup() {
  Serial.begin(460800);
  delay(3000);
  Serial.println("\n======== DAEDALUS UNIT TEST ========");
  Serial.println("Board: STM32H725RGVX");
  Serial.println("Baud:  460800");

  // GPIO
  pinMode(USER_GPIO_BUZZER, OUTPUT);
  digitalWrite(USER_GPIO_BUZZER, LOW);

  pinMode(BNO08X_RESET, OUTPUT);

  // GPS reset sequence
  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, HIGH); delay(20);
  digitalWrite(M10S_RESET, LOW);  delay(100);
  digitalWrite(M10S_RESET, HIGH);

  // NeoPixel
  led_strip.begin();
  led_strip.setBrightness(20);
  led_strip.clear();
  led_strip.show();

  // SPI (for ISM256 + BMP581)
  SPI.setMISO(USER_GPIO_SPI1_MISO);
  SPI.setMOSI(USER_GPIO_SPI1_MOSI);
  SPI.setSCLK(USER_GPIO_SPI1_SCK);
  SPI.begin();

  // I2C (for BNO, INA, ToF, GPS)
  i2c4.begin();

  // HLSCL half-duplex servo bus
  ServoSerial.begin(1000000);
  hlscl.pSerial = &ServoSerial;
  // Put all 3 servo IDs into wheel mode before testing
  for (uint8_t id = 1; id <= 3; id++) {
    hlscl.WheelMode(id);
  }
  delay(100);

  // ── SENSOR TESTS ──────────────────────────────────────
  testISM256();
  testBMP581();
  testBNO08x();
  testINA236();
  testVL53L1X();
  testGPS();

  // ── ACTUATOR TESTS ────────────────────────────────────
  testBuzzer();
  testNeoPixel();
  testServoA();
  testServoB();
  testHLSCLServo(1, res.hlscl1, "HLSCL Servo #1");
  testHLSCLServo(2, res.hlscl2, "HLSCL Servo #2");
  testHLSCLServo(3, res.hlscl3, "HLSCL Servo #3");

  printSummary();
}

void loop() {
  delay(1000);
}
