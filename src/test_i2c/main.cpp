/**
 * test_i2c — I2C + SPI sensor isolation test (FreeRTOS, 3 threads).
 *
 * Thread layout:
 *   CB_ReadSPI      — ISM6HG256X + BMP581 via SPI1, guarded by mtx_spi
 *   CB_ReadI2C      — INA236 + BNO08x + M10S + VL53L1X via I2C4, single owner
 *   CB_ConstructData— builds sd_buf + tx_buf from sensor snapshot at 5 Hz
 *   CB_Print        — snapshots sensor_data, prints at 1 Hz via Serial CDC
 *   CB_XbeeTx       — sends pre-built tx_buf over XBee at 1 Hz
 *   CB_XbeeRx       — receives bytes from XBee, echoes to Serial CDC
 *   CB_SDLogger     — appends pre-built sd_buf to SD card at 5 Hz, under mtx_spi
 *   CB_SDFlush      — flushes SD file to card every 1 s, under mtx_spi
 *
 * Init order (critical on STM32H7):
 *   SPI.begin() → SPI sensors → i2c4.begin() → I2C sensors
 *   SPI.begin() calls HAL_RCCEx_PeriphCLKConfig() which can change the
 *   peripheral clock tree and corrupt any I2C4 TIMINGR computed before it.
 *   Initialising SPI first ensures I2C timing is derived from the final
 *   stable clock configuration.
 */

#include <Arduino.h>
#include <SPI.h>
#include <STM32SD.h>
#include <File_Utility.h>

#if __has_include("STM32FreeRTOS.h")
#  include "hal_rtos.h"
#endif

#include "UserPins.h"
#include <INA236.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include "SparkFun_VL53L1X.h"
#include <ISM6HG256XSensor.h>
#include <SparkFun_BMP581_Arduino_Library.h>

// ── Buses ─────────────────────────────────────────────────────────────────────
TwoWire        i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);
HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

// ── I2C sensor instances ──────────────────────────────────────────────────────
INA236         ina(0x40, &i2c4);
BNO08x         bno;
SFE_UBLOX_GNSS m10s;
SFEVL53L1X     tof(i2c4);

// ── SPI sensor instances ──────────────────────────────────────────────────────
ISM6HG256XSensor ism(&SPI, USER_GPIO_ISM256_NSS);

BMP581                    bmp;
bmp5_osr_odr_press_config bmp_osr = {
  .osr_t    = BMP5_OVERSAMPLING_8X,
  .osr_p    = BMP5_OVERSAMPLING_64X,
  .press_en = 0,
  .odr      = 0
};

// ── Init flags ────────────────────────────────────────────────────────────────
bool ok_ina = false;
bool ok_bno = false;
bool ok_gps = false;
bool ok_tof = false;
bool ok_ism = false;
bool ok_bmp = false;
bool ok_sd  = false;

// ── SD card ───────────────────────────────────────────────────────────────────
FsUtil  fs_sd;
String  sd_buf;   // SD CSV row   — written by ConstructData, consumed by CB_SDLogger
String  tx_buf;   // XBee TX line — written by ConstructData, consumed by CB_XbeeTx

constexpr uint32_t UBLOX_TIMEOUT = 250ul;

// ── Shared data (guarded by mtx_data for the print thread) ───────────────────
struct SensorData {
  // INA236
  float batt_volt = 0.f;
  float batt_curr = 0.f;

  // BNO08x
  float roll = 0.f, pitch = 0.f, yaw = 0.f;
  bool  bno_fresh = false;

  // M10S GPS
  uint8_t siv = 0, hh = 0, mm = 0, ss = 0;
  double  lat = 0.0, lon = 0.0, alt = 0.0, vn = 0.0, ve = 0.0;
  bool    gps_fresh = false;

  // VL53L1X
  float tof_m     = 0.f;
  bool  tof_fresh = false;

  // ISM6HG256X
  ISM6HG256X_Axes_t accel{}, gyro{};
  bool ism_fresh = false;

  // BMP581
  bmp5_sensor_data bmp_data{};
  bool bmp_fresh = false;
} sensor_data;

hal::rtos::mutex_t mtx_spi;
hal::rtos::mutex_t mtx_data;  // guards sensor_data for SPI/I2C read threads
hal::rtos::mutex_t mtx_buf;   // guards sd_buf / tx_buf
hal::rtos::mutex_t mtx_uart;  // guards Xbee HardwareSerial

static uint32_t packet_count = 0;
static String   xbee_rx_buf;  // accumulates incoming XBee bytes between newlines

// ── I2C bus recovery ──────────────────────────────────────────────────────────
// Bit-bang 9 SCL pulses to free a slave stuck mid-byte, then reinit I2C4.
static void RecoverI2C4() {
  pinMode(USER_GPIO_I2C4_SDA, OUTPUT_OPEN_DRAIN);
  pinMode(USER_GPIO_I2C4_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(USER_GPIO_I2C4_SDA, HIGH);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(USER_GPIO_I2C4_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(USER_GPIO_I2C4_SCL, HIGH); delayMicroseconds(5);
  }
  // STOP condition
  digitalWrite(USER_GPIO_I2C4_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(USER_GPIO_I2C4_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(USER_GPIO_I2C4_SDA, HIGH); delayMicroseconds(5);
  i2c4.end();
  i2c4.setClock(100000);
  i2c4.begin();
}

// ── Thread: SPI reads ─────────────────────────────────────────────────────────
// Owns mtx_spi for every SPI transaction. Runs at 50 Hz.
void CB_ReadSPI(void *) {
  hal::rtos::interval_loop(50ul, [&]() -> void {
    ISM6HG256X_Axes_t accel{}, gyro{};
    bool ism_ok = false;

    mtx_spi.exec([&]() -> void {
      ism_ok = (ism.Get_X_Axes(&accel) == ISM6HG256X_OK &&
                ism.Get_G_Axes(&gyro)  == ISM6HG256X_OK);
    });

    bmp5_sensor_data bmp_data{};
    bool bmp_ok = false;
    mtx_spi.exec([&]() -> void {
      bmp_ok = (bmp.getSensorData(&bmp_data) == BMP5_OK);
    });

    mtx_data.exec([&]() -> void {
      sensor_data.ism_fresh = ism_ok;
      if (ism_ok) {
        sensor_data.accel = accel;
        sensor_data.gyro  = gyro;
      }
      sensor_data.bmp_fresh = bmp_ok;
      if (bmp_ok) sensor_data.bmp_data = bmp_data;
    });
  });
}

// ── Thread: I2C reads ─────────────────────────────────────────────────────────
// Single thread owns the entire I2C4 bus — no I2C mutex needed.
// Elapsed-time guard detects a hung bus and triggers GPIO-level recovery.
void CB_ReadI2C(void *) {
  hal::rtos::interval_loop(200ul, [&]() -> void {
    const uint32_t t0 = millis();

    // ── INA236 ───────────────────────────────────────────────────────────────
    float batt_volt = 0.f, batt_curr = 0.f;
    if (ok_ina) {
      batt_volt = ina.getBusVoltage();
      batt_curr = ina.getCurrent();
    }

    // ── BNO08x ───────────────────────────────────────────────────────────────
    float roll = 0.f, pitch = 0.f, yaw = 0.f;
    bool  bno_fresh = false;
    if (ok_bno) {
      if (bno.wasReset()) bno.enableRotationVector();
      if (digitalRead(USER_GPIO_BNO_INT1) == LOW && bno.getSensorEvent()) {
        if (bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
          roll      = bno.getRoll()  * 180.0f / PI;
          pitch     = bno.getPitch() * 180.0f / PI;
          yaw       = bno.getYaw()   * 180.0f / PI;
          if (yaw < 0) yaw += 360.0f;
          bno_fresh = true;
        }
      }
    }

    // ── M10S GPS ─────────────────────────────────────────────────────────────
    uint8_t siv = 0, hh = 0, mm_v = 0, ss = 0;
    double  lat = 0.0, lon = 0.0, alt = 0.0, vn = 0.0, ve = 0.0;
    bool    gps_fresh = false;
    if (ok_gps) {
        siv       = m10s.getSIV();
        hh        = m10s.getHour();
        mm_v      = m10s.getMinute();
        ss        = m10s.getSecond();
        lat       = static_cast<double>(m10s.getLatitude())    * 1e-7;
        lon       = static_cast<double>(m10s.getLongitude())   * 1e-7;
        alt       = static_cast<double>(m10s.getAltitudeMSL()) * 1e-3;
        vn        = static_cast<double>(m10s.getNedNorthVel()) * 1e-3;
        ve        = static_cast<double>(m10s.getNedEastVel())  * 1e-3;
        gps_fresh = true;
    }

    // ── VL53L1X ──────────────────────────────────────────────────────────────
    float tof_m     = 0.f;
    bool  tof_fresh = false;
    if (ok_tof) {
      tof.startRanging();
      const uint32_t deadline = millis() + 150;
      while (!tof.checkForDataReady()) {
        if (millis() > deadline) break;
        vTaskDelay(1);
      }
      if (tof.checkForDataReady()) {
        tof_m     = tof.getDistance() * 0.001f;
        tof_fresh = true;
        tof.clearInterrupt();
      }
      tof.stopRanging();
    }

    // ── I2C bus hung? ─────────────────────────────────────────────────────────
    if (millis() - t0 > 600) {
      RecoverI2C4();
      return;  // skip the data commit this cycle
    }

    // ── Commit to shared data ─────────────────────────────────────────────────
    mtx_data.exec([&]() -> void {
      sensor_data.batt_volt = batt_volt;
      sensor_data.batt_curr = batt_curr;

      sensor_data.bno_fresh = bno_fresh;
      if (bno_fresh) {
        sensor_data.roll  = roll;
        sensor_data.pitch = pitch;
        sensor_data.yaw   = yaw;
      }

      sensor_data.gps_fresh = gps_fresh;
      if (gps_fresh) {
        sensor_data.siv = siv;
        sensor_data.hh  = hh;
        sensor_data.mm  = mm_v;
        sensor_data.ss  = ss;
        sensor_data.lat = lat;
        sensor_data.lon = lon;
        sensor_data.alt = alt;
        sensor_data.vn  = vn;
        sensor_data.ve  = ve;
      }

      sensor_data.tof_fresh = tof_fresh;
      if (tof_fresh) sensor_data.tof_m = tof_m;
    });
  });
}

// ── Thread: Print at 1 Hz ────────────────────────────────────────────────────
void CB_Print(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    SensorData snap;
    mtx_data.exec([&]() { snap = sensor_data; });

    Serial.println("------------------------------------------------------------");

    if (ok_ina) {
      Serial.print("[INA236]  Vbus=");
      Serial.print(snap.batt_volt, 3);
      Serial.print(" V  I=");
      Serial.print(snap.batt_curr, 4);
      Serial.println(" A");
    } else {
      Serial.println("[INA236]  SKIP (init failed)");
    }

    if (ok_bno) {
      if (snap.bno_fresh) {
        Serial.print("[BNO08x]  roll=");
        Serial.print(snap.roll, 2);
        Serial.print(" deg  pitch=");
        Serial.print(snap.pitch, 2);
        Serial.print(" deg  yaw=");
        Serial.print(snap.yaw, 2);
        Serial.println(" deg");
      } else {
        Serial.println("[BNO08x]  no data yet");
      }
    } else {
      Serial.println("[BNO08x]  SKIP (init failed)");
    }

    if (ok_gps) {
      if (snap.gps_fresh) {
        Serial.printf("[M10S]    %02d:%02d:%02d  SIV=%d  lat=%.7f  lon=%.7f  alt=%.2f m  vN=%.3f vE=%.3f m/s\n",
                      snap.hh, snap.mm, snap.ss,
                      snap.siv, snap.lat, snap.lon, snap.alt,
                      snap.vn, snap.ve);
      } else {
        Serial.println("[M10S]    waiting for PVT...");
      }
    } else {
      Serial.println("[M10S]    SKIP (init failed)");
    }

    if (ok_tof) {
      if (snap.tof_fresh) {
        Serial.print("[VL53L1X] dist=");
        Serial.print(snap.tof_m, 3);
        Serial.println(" m");
      } else {
        Serial.println("[VL53L1X] no data yet");
      }
    } else {
      Serial.println("[VL53L1X] SKIP (init failed)");
    }

    if (ok_ism) {
      if (snap.ism_fresh) {
        constexpr double G = 9.80665;
        Serial.printf("[ISM256]  ax=%.3f ay=%.3f az=%.3f m/s2  gx=%.3f gy=%.3f gz=%.3f dps\n",
                      snap.accel.x * G / 1000.0,
                      snap.accel.y * G / 1000.0,
                      snap.accel.z * G / 1000.0,
                      snap.gyro.x  / 1000.0,
                      snap.gyro.y  / 1000.0,
                      snap.gyro.z  / 1000.0);
      } else {
        Serial.println("[ISM256]  read failed");
      }
    } else {
      Serial.println("[ISM256]  SKIP (init failed)");
    }

    if (ok_bmp) {
      if (snap.bmp_fresh) {
        Serial.printf("[BMP581]  P=%.2f hPa  T=%.2f C\n",
                      snap.bmp_data.pressure * 0.01,
                      snap.bmp_data.temperature);
      } else {
        Serial.println("[BMP581]  read failed");
      }
    } else {
      Serial.println("[BMP581]  SKIP (init failed)");
    }
  });
}

// ── Thread: XBee transmit at 1 Hz ────────────────────────────────────────────
// Snaps the pre-built tx_buf from ConstructData and sends it over XBee.
void CB_XbeeTx(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    String snap;
    mtx_buf.exec([&]() { snap = tx_buf; });
    mtx_uart.exec([&]() { Xbee.println(snap); });
  });
}

// ── Thread: XBee receive ──────────────────────────────────────────────────────
// Polls XBee for incoming bytes at 10 ms. Assembles lines and echoes them
// to the Serial CDC monitor so the user can see what the ground station sends.
void CB_XbeeRx(void *) {
  hal::rtos::interval_loop(10ul, [&]() -> void {
    bool line_ready = false;

    mtx_uart.exec([&]() -> void {
      while (Xbee.available()) {
        const char c = Xbee.read();
        if (c == '\n') {
          xbee_rx_buf.trim();
          line_ready = true;
        } else {
          xbee_rx_buf += c;
        }
      }
    });

    if (line_ready) {
      Serial.print("[XBee RX] ");
      Serial.println(xbee_rx_buf);
      xbee_rx_buf = "";
    }
  });
}

// ── Thread: SD logger at 5 Hz ────────────────────────────────────────────────
// Snaps the pre-built sd_buf from ConstructData and appends it to the SD file.
void CB_SDLogger(void *) {
  hal::rtos::interval_loop(200ul, [&]() -> void {
    if (!ok_sd) return;
    String snap;
    mtx_buf.exec([&]() { snap = sd_buf; });
    mtx_spi.exec([&]() { fs_sd.file() << snap; });
  });
}

// ── Thread: SD flush at 1 Hz ─────────────────────────────────────────────────
void CB_SDFlush(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    if (!ok_sd) return;
    mtx_spi.exec([&]() { fs_sd.file().flush(); });
  });
}

// ── ConstructData ─────────────────────────────────────────────────────────────
// Builds the SD CSV row (sd_buf) and the XBee TX line (tx_buf) from a snapshot
// of sensor_data. String construction happens outside all locks; only the final
// pointer swap is held under mtx_buf, matching the pattern in main.cpp.
//
// SD  format: MILLIS,VBUS,CURR,ROLL,PITCH,YAW,UTC,SIV,LAT,LON,ALT_M,TOF_M,
//             AX,AY,AZ,GX,GY,GZ,PRES_HPA,TEMP_C  (CRLF terminated)
// TX  format: INA,volt,curr,BNO,roll,pitch,yaw,GPS,utc,siv,lat,lon,alt,
//             TOF,tof,IMU,ax,ay,az,BARO,pres,temp
static void ConstructData() {
  SensorData snap;
  mtx_data.exec([&]() { snap = sensor_data; });

  constexpr double G = 9.80665;
  char utc[9];
  snprintf(utc, sizeof(utc), "%02d:%02d:%02d", snap.hh, snap.mm, snap.ss);

  // ── SD row ───────────────────────────────────────────────────────────────────
  String local_sd;
  local_sd.reserve(220);
  local_sd += millis();                                     local_sd += ',';
  local_sd += String(snap.batt_volt, 3);                   local_sd += ',';
  local_sd += String(snap.batt_curr, 4);                   local_sd += ',';
  local_sd += String(snap.roll,  2);                       local_sd += ',';
  local_sd += String(snap.pitch, 2);                       local_sd += ',';
  local_sd += String(snap.yaw,   2);                       local_sd += ',';
  local_sd += utc;                                          local_sd += ',';
  local_sd += snap.siv;                                     local_sd += ',';
  local_sd += String(snap.lat, 7);                          local_sd += ',';
  local_sd += String(snap.lon, 7);                          local_sd += ',';
  local_sd += String(snap.alt, 2);                          local_sd += ',';
  local_sd += String(snap.tof_m, 3);                        local_sd += ',';
  local_sd += String(snap.accel.x * G / 1000.0, 3);        local_sd += ',';
  local_sd += String(snap.accel.y * G / 1000.0, 3);        local_sd += ',';
  local_sd += String(snap.accel.z * G / 1000.0, 3);        local_sd += ',';
  local_sd += String(snap.gyro.x  / 1000.0, 3);            local_sd += ',';
  local_sd += String(snap.gyro.y  / 1000.0, 3);            local_sd += ',';
  local_sd += String(snap.gyro.z  / 1000.0, 3);            local_sd += ',';
  local_sd += String(snap.bmp_data.pressure * 0.01, 2);    local_sd += ',';
  local_sd += String(snap.bmp_data.temperature, 2);         local_sd += "\r\n";

  // ── TX line ──────────────────────────────────────────────────────────────────
  String local_tx;
  local_tx.reserve(160);
  local_tx  = "INA,";
  local_tx += String(snap.batt_volt, 3);  local_tx += ',';
  local_tx += String(snap.batt_curr, 4);  local_tx += ',';
  local_tx += "BNO,";
  local_tx += String(snap.roll,  2);      local_tx += ',';
  local_tx += String(snap.pitch, 2);      local_tx += ',';
  local_tx += String(snap.yaw,   2);      local_tx += ',';
  local_tx += "GPS,";
  local_tx += utc;                         local_tx += ',';
  local_tx += snap.siv;                    local_tx += ',';
  local_tx += String(snap.lat, 7);         local_tx += ',';
  local_tx += String(snap.lon, 7);         local_tx += ',';
  local_tx += String(snap.alt, 2);         local_tx += ',';
  local_tx += "TOF,";
  local_tx += String(snap.tof_m, 3);       local_tx += ',';
  local_tx += "IMU,";
  local_tx += String(snap.accel.x * G / 1000.0, 3);  local_tx += ',';
  local_tx += String(snap.accel.y * G / 1000.0, 3);  local_tx += ',';
  local_tx += String(snap.accel.z * G / 1000.0, 3);  local_tx += ',';
  local_tx += "BARO,";
  local_tx += String(snap.bmp_data.pressure * 0.01, 2);  local_tx += ',';
  local_tx += String(snap.bmp_data.temperature,      2);

  // Atomic swap — lock held only for the pointer exchange.
  mtx_buf.exec([&]() {
    sd_buf = std::move(local_sd);
    tx_buf = std::move(local_tx);
  });
  ++packet_count;
}

// ── Thread: Construct data at 5 Hz ───────────────────────────────────────────
void CB_ConstructData(void *) {
  hal::rtos::interval_loop(200ul, [&]() -> void {
    ConstructData();
  });
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(460800);
  delay(2000);
  Serial.println("\n[test_i2c] I2C + SPI isolation test (FreeRTOS)\n");

  // ── SDIO (SDMMC) — must come before SPI.begin() for the same clock reason ──
  // SDMMC1 on STM32H7 can also reconfigure the peripheral clock tree via
  // HAL_RCCEx_PeriphCLKConfig(). Init and open the log file before any sensor
  // is touched so the file name is allocated while the heap is clean.
  SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1,
           USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);
  Serial.print("[SD]      init... ");
  ok_sd = SD.begin();
  Serial.println(ok_sd ? "OK" : "FAIL");
  if (ok_sd) {
    fs_sd.find_file_name("TEST_I2C_", "CSV");
    fs_sd.open_one<FsMode::WRITE>();
    // CSV header
    fs_sd.file() << "MILLIS,VBUS,CURR,"
                    "ROLL,PITCH,YAW,"
                    "UTC,SIV,LAT,LON,ALT_M,"
                    "TOF_M,"
                    "AX,AY,AZ,GX,GY,GZ,"
                    "PRES_HPA,TEMP_C\r\n";
    fs_sd.file().flush();
    sd_buf.reserve(256);
    tx_buf.reserve(192);
  }

  // ── SPI first (fixes I2C4 TIMINGR on STM32H7) ────────────────────────────
  SPI.setMISO(USER_GPIO_SPI1_MISO);
  SPI.setMOSI(USER_GPIO_SPI1_MOSI);
  SPI.setSCLK(USER_GPIO_SPI1_SCK);
  SPI.begin();
  // Deprioritize SPI1 DMA IRQs so they cannot preempt an in-progress I2C4
  // HAL transfer.
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);

  Serial.print("[ISM256]  init... ");
  ok_ism = ism.begin()     == ISM6HG256X_OK &&
           ism.Enable_X()    == ISM6HG256X_OK &&
           ism.Enable_HG_X() == ISM6HG256X_OK &&
           ism.Enable_G()    == ISM6HG256X_OK;
  Serial.println(ok_ism ? "OK" : "FAIL");

  Serial.print("[BMP581]  init... ");
  ok_bmp = bmp.beginSPI(USER_GPIO_BMP581_NSS, 10'000'000) == BMP5_OK &&
           bmp.setOSRMultipliers(&bmp_osr)               == BMP5_OK &&
           bmp.setODRFrequency(BMP5_ODR_20_HZ)           == BMP5_OK;
  Serial.println(ok_bmp ? "OK" : "FAIL");

  // ── I2C after SPI clock is settled ───────────────────────────────────────
  i2c4.setClock(100000);
  i2c4.begin();

  Serial.print("[INA236]  init... ");
  ok_ina = ina.begin();
  Serial.println(ok_ina ? "OK" : "FAIL");
  if (ok_ina) {
    ina.setADCRange(0);
    ina.setMaxCurrentShunt(8, 0.008);
    ina.setAverage(INA236_64_SAMPLES);
  }

  Serial.print("[BNO08x]  init... ");
  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, 1);
  ok_bno = bno.begin(BNO08X_ADDR, i2c4, USER_GPIO_BNO_INT1, BNO08X_RESET);
  Serial.println(ok_bno ? "OK" : "FAIL");
  if (ok_bno) bno.enableRotationVector();

  Serial.print("[M10S]    reset... ");
  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, 1); delay(20);
  digitalWrite(M10S_RESET, 0); delay(100);
  digitalWrite(M10S_RESET, 1); delay(500);
  Serial.println("done");

  Serial.print("[M10S]    init... ");
  ok_gps = m10s.begin(i2c4, 0x42);
  Serial.println(ok_gps ? "OK" : "FAIL");
  if (ok_gps) {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_TIMEOUT);
    m10s.setNavigationFrequency(10, VAL_LAYER_RAM_BBR, UBLOX_TIMEOUT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_TIMEOUT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_TIMEOUT);
  }

  Serial.print("[VL53L1X] init... ");
  tof.setI2CAddress(0x29);
  ok_tof = (tof.begin() == 0);
  Serial.println(ok_tof ? "OK" : "FAIL");

  // ── XBee UART ─────────────────────────────────────────────────────────────
  Xbee.begin(115200);
  Serial.println("[XBee]    UART ready at 115200");

  Serial.println("\n[test_i2c] Starting RTOS threads...\n");

  // ── Launch threads ────────────────────────────────────────────────────────
  hal::rtos::scheduler.initialize();
  hal::rtos::scheduler.create(CB_ReadSPI,      {.name = "SPI",       .stack_size = 2048, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadI2C,      {.name = "I2C",       .stack_size = 4096, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ConstructData,{.name = "Construct",  .stack_size = 2048, .priority = osPriorityBelowNormal});
  hal::rtos::scheduler.create(CB_Print,        {.name = "Print",      .stack_size = 2048, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_XbeeTx,       {.name = "XbeeTx",    .stack_size = 1024, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_XbeeRx,       {.name = "XbeeRx",    .stack_size = 1024, .priority = osPriorityNormal});
  if (ok_sd) {
    hal::rtos::scheduler.create(CB_SDLogger,   {.name = "SDLog",     .stack_size = 2048, .priority = osPriorityNormal});
    hal::rtos::scheduler.create(CB_SDFlush,    {.name = "SDFlush",   .stack_size = 1024, .priority = osPriorityNormal});
  }
  hal::rtos::scheduler.start();
}

void loop() {
  // Execution never reaches here — RTOS scheduler takes over after start().
}
