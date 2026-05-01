/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>  // Arduino Framework
#include <lib_xcore>
#include <xcore/math_module>
#include <File_Utility.h>     // File Utility
#include <LibAvionics.h>      // Base Avionics Library and Utilities
#include "SystemFunctions.h"  // Function Declarations
#include "custom_kalman.h"    // Kalman Quick Table

#if __has_include("STM32FreeRTOS.h")
#  define USE_FREERTOS 1
#  include "hal_rtos.h"
#endif

// Hardware
#include <STM32SD.h>
#include <STM32LowPower.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <INA236.h>
#include "SparkFun_VL53L1X.h"
#include <Servo.h>

/* BEGIN INCLUDE USER'S IMPLEMENTATIONS */
#include "config/Main/UserConfig.h"
#include "UserPins.h"     // User's Pins Mapping
#include "UserSensors.h"  // User's Hardware Implementations
#include "UserFSM.h"      // User's FSM States
#include "Controlling.h"
#include "custom_EEPROM.h"
#include "BNO086Cal.h"
/* END INCLUDE USER'S IMPLEMENTATIONS */

/* BEGIN INCLUDE MAIN */
#include "./main.h"
/* END INCLUDE MAIN */

/* BEGIN SENSOR INSTANCES */
TwoWire i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);

HardwareSerial ServoSerial(USER_GPIO_Half);
HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

SFE_UBLOX_GNSS    m10s;
INA236            ina(0x40, &i2c4);
sh2_SensorValue_t sensorValue;
SFEVL53L1X        tof(i2c4);
BNO08x            bno;
BNO086Calibrator  bno_cal;

Adafruit_NeoPixel led(2, USER_GPIO_LED, NEO_GRB + NEO_KHZ800);

Servo servo_a;
Servo servo_b;

SensorIMU *imu[RA_NUM_IMU] = {
  new IMU_ISM256(USER_GPIO_ISM256_NSS),  // IMU #1
};
SensorAltimeter *altimeter[RA_NUM_ALTIMETER] = {
  new Altimeter_BMP581(USER_GPIO_BMP581_NSS),  // Altimeter #1
};
/* END SENSOR INSTANCES */

/* BEGIN SENSOR STATUSES */
struct SensorsHealth {
  SensorStatus imu[RA_NUM_IMU]{};
  SensorStatus altimeter[RA_NUM_ALTIMETER]{};
  SensorStatus gnss[RA_NUM_GNSS]{};
} sensors_health;
/* END SENSOR STATUSES */

/* BEGIN PERSISTENT STATE */
UserFSM  fsm;
double   acc;
double   alt_ref;  // Altitude at ground
double   alt_agl;  // Altitude above ground
double   apogee_raw;
uint32_t packet_count{};
/* END PERSISTENT STATE */

/* BEGIN DATA MEMORY */
DataMemory data;

String sd_buf;
String tx_buf;
/* END DATA MEMORY */

/* Control*/
GPSCoordinate     current_location;
Controller        controller;
numeric_vector<3> servo_target_angles;

/* BEGIN ACTUATORS */
float pos_a = RA_SERVO_A_LOCK;
float pos_b = RA_SERVO_B_LOCK;
/* END ACTUATORS */

volatile bool ins_thresholds_dirty = false;
volatile bool apogee_alt_dirty     = false;

/* EEPROM — backed by STM32H725 Backup SRAM (0x38800000)
   Direct memcpy: no Flash erase, no blocking, ~microseconds.*/

void EEPROM_Write() {
  EEPROMStore s{};
  s.magic = EEPROM_MAGIC;
  memcpy(s.utc, data.utc, sizeof(s.utc));
  s.packet_count = packet_count;
  // s.state        = static_cast<uint8_t>(fsm.state());
  s.alt_ref = alt_ref;
  s.pos_a   = pos_a;
  s.pos_b   = pos_b;
  for (size_t i = 0; i < 3; ++i)
    s.servo_angles[i] = controller.last_angles[i];

  s.crc = eeprom_crc8(
    reinterpret_cast<const uint8_t *>(&s) + EEPROM_PAYLOAD_OFFSET,
    sizeof(EEPROMStore) - EEPROM_PAYLOAD_OFFSET);

  memcpy(reinterpret_cast<void *>(BKPSRAM_BASE), &s, sizeof(s));
  __DSB();  // ensure write completes before returning
}

// Read only if magic + CRC are valid (data was previously *defined*).
// If either check fails ("ndef") returns without touching any live variable.
void EEPROM_Read() {
  EEPROMStore s{};
  memcpy(&s, reinterpret_cast<const void *>(BKPSRAM_BASE), sizeof(s));

  if (s.magic != EEPROM_MAGIC) return;

  const uint8_t expected = eeprom_crc8(
    reinterpret_cast<const uint8_t *>(&s) + EEPROM_PAYLOAD_OFFSET,
    sizeof(EEPROMStore) - EEPROM_PAYLOAD_OFFSET);
  if (s.crc != expected) return;

  memcpy(data.utc, s.utc, sizeof(data.utc));
  packet_count = s.packet_count;
  // fsm.transfer(static_cast<UserState>(s.state));
  alt_ref = s.alt_ref;
  pos_a   = s.pos_a;
  pos_b   = s.pos_b;
  for (size_t i = 0; i < 3; ++i)
    controller.last_angles[i] = s.servo_angles[i];
}

/* BEGIN SD CARD */
FsUtil fs_sd;
/* END SD CARD */

/* BEGIN FILTERS */
xcore::vdt<FILTER_ORDER - 1> vdt(static_cast<double>(RA_INTERVAL_FSM_EVAL) * 0.001);
FilterAcc                    filter_acc;
FilterAlt                    filter_alt;
FilterGPS                    filter_nav_n;  // state[0]=lat_deg (filtered),  state[1]=vn_m_s (filtered)
FilterGPS                    filter_nav_e;  // state[0]=lon_deg (filtered),  state[1]=ve_m_s (filtered)
/* END FILTERS */

/* BEGIN USER PRIVATE VARIABLES */
hal::rtos::mutex_t mtx_spi;
hal::rtos::mutex_t mtx_sdio;
hal::rtos::mutex_t mtx_i2c;  // guards i2c4 bus (BNO, GPS, INA, TOF)
hal::rtos::mutex_t mtx_cdc;
hal::rtos::mutex_t mtx_uart;
hal::rtos::mutex_t mtx_buf;  // guards tx_buf and sd_buf
hal::rtos::mutex_t mtx_nav;  // guards nav_state — held for microseconds only
hal::rtos::mutex_t mtx_kf;   // guards filter_acc, filter_alt, alt_agl, apogee_raw

/* BEGIN USER PRIVATE FUNCTIONS */
uint32_t LoggerInterval() {
  switch (fsm.state()) {
    case UserState::STARTUP:
    case UserState::IDLE_SAFE:
      return RA_SDLOGGER_INTERVAL_IDLE;

    case UserState::LAUNCH_PAD:
      return RA_SDLOGGER_INTERVAL_SLOW;

    case UserState::ASCENT:
    case UserState::APOGEE:
    case UserState::DESCENT:
    case UserState::PROBE_REALEASE:
    case UserState::PAYLOAD_REALEASE:
      return RA_SDLOGGER_INTERVAL_FAST;

    case UserState::LANDED:
    default:
      return RA_SDLOGGER_INTERVAL_IDLE;
  }
}
/* END USER PRIVATE FUNCTIONS */

// Navigation state written by GNSS/MAG tasks, read by CB_Control.
// Kept separate from mtx_i2c so CB_Control never blocks on an I2C transaction.
struct NavState {
  GPSCoordinate location{};
  double        vn      = 0;
  double        ve      = 0;
  double        yaw     = 0;
  double        heading = 0;  // GPS ground-track heading (deg, North-relative) from getHeading()
  bool          fixed   = false;
} nav_state;

// Kinematic sim — advanced by CB_Control each tick when simActivated is true.
namespace SimNav {
  constexpr double START_LAT    = 13.722992512087279 + 0.000420;
  constexpr double START_LON    = 100.51463610518176 + 0.000300;
  constexpr double START_ALT    = 100.0;
  constexpr double DESCENT_RATE = 2.0;
  constexpr float  YAW_SPIN     = 15.0;
  constexpr double INIT_VN      = -0.9;
  constexpr double INIT_VE      = -0.6;

  static double lat     = START_LAT;
  static double lon     = START_LON;
  static double alt     = START_ALT;
  static double vn      = INIT_VN;
  static double ve      = INIT_VE;
  static float  yaw     = 30.0;
  static double heading = 0.0;  // ground-track heading derived from vn/ve

  static void reset() {
    lat     = START_LAT;
    lon     = START_LON;
    alt     = START_ALT;
    vn      = INIT_VN;
    ve      = INIT_VE;
    yaw     = 30.0;
    heading = std::fmod(std::atan2(ve, vn) * RAD_TO_DEG + 360.0, 360.0);
  }

  static void update(double dt) {
    lat += (vn * dt) / EARTH_RADIUS_M * RAD_TO_DEG;
    lon += (ve * dt) / (EARTH_RADIUS_M * std::cos(lat * DEG_TO_RAD)) * RAD_TO_DEG;
    alt -= DESCENT_RATE * dt;
    if (alt < 0.0) alt = 0.0;
    yaw     = std::fmod(yaw + YAW_SPIN * dt, 360.0f);
    heading = std::fmod(std::atan2(ve, vn) * RAD_TO_DEG + 360.0, 360.0);
  }
}  // namespace SimNav

#ifdef RA_STACK_HWM_ENABLED
struct StackHWM {
  UBaseType_t imu       = 0;
  UBaseType_t altimeter = 0;
  UBaseType_t mag       = 0;
  UBaseType_t gnss      = 0;
  UBaseType_t ina       = 0;
  UBaseType_t tof       = 0;
  UBaseType_t fsm       = 0;
  UBaseType_t construct = 0;
  UBaseType_t transmit  = 0;
  UBaseType_t receive   = 0;
  UBaseType_t retain    = 0;
  UBaseType_t ins       = 0;
  UBaseType_t neo       = 0;
  UBaseType_t control   = 0;
  UBaseType_t sdlog     = 0;
  UBaseType_t debug     = 0;
} stack_hwm;
#endif
/* END USER PRIVATE VARIABLES */


/* BEGIN USER SETUP */
void UserSetupGPIO() {
  if constexpr (RA_LED_ENABLED) {
    pinMode(USER_GPIO_BUZZER, OUTPUT);
    digitalToggle(USER_GPIO_BUZZER);
    delay(100);
    digitalToggle(USER_GPIO_BUZZER);
  }

  led.begin();
  led.setBrightness(10);
  led.clear();
  led.setPixelColor(0, led.Color(0, 0, 255));
  led.setPixelColor(1, led.Color(0, 0, 255));
  led.show();
  led.clear();

  // Camera trigger pins — idle LOW, driven HIGH during ascent
  pinMode(USER_GPIO_CAM1, OUTPUT);
  pinMode(USER_GPIO_CAM2, OUTPUT);
  digitalWrite(USER_GPIO_CAM1, LOW);
  digitalWrite(USER_GPIO_CAM2, LOW);

  // Sensors reset GPIO
  pinMode(BNO08X_RESET, OUTPUT);  // idle HIGH — bno.begin() owns the reset sequence

  pinMode(M10S_RESET, OUTPUT);
  digitalWrite(M10S_RESET, 1);
  delay(20);
  digitalWrite(M10S_RESET, 0);
  delay(100);
  digitalWrite(M10S_RESET, 1);
}

void UserSetupLowPower() {
  LowPower.begin();
  LowPower.enableWakeupFrom(&Xbee, []() {});
}

void UserSetupActuator() {
  controller.init_pid();
  // Paraglider Servo
  ServoSerial.begin(1'000'000);
  // NVIC_SetPriority(UART7_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  ServoSerial.setTimeout(5);  // at 1 Mbaud a 4-byte response arrives in ~40 µs; 2 ms is generous without burning CPU
  controller.driver.hlscl.pSerial = &ServoSerial;

  // Initialize servo driver
  controller.driver.hlscl.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 5);
  for (size_t i = 0; i < sizeof(ServoDriver::IDS); ++i) {
    controller.driver.hlscl.WheelMode(ServoDriver::IDS[i]);
    controller.driver.hlscl.EnableTorque(ServoDriver::IDS[i], 1);
  }

  // Test
  for (size_t i = 0; i < 3; ++i) {
    controller.driver.write_speed(ServoDriver::IDS[i], 500);
    delay(50);
    controller.driver.write_speed(ServoDriver::IDS[i], -500);
    delay(50);
    controller.driver.write_speed(ServoDriver::IDS[i], 0);
  }


  //DEPLOYMENT SERVO
  servo_a.attach(USER_GPIO_SERVO_A, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
  servo_a.write(pos_a + 10);
  servo_a.write(pos_a);

  servo_b.attach(USER_GPIO_SERVO_B, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
  servo_b.write(pos_b + 10);
  servo_b.write(pos_b);
}

void UserSetupCDC() {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.begin(460800);
    delay(2000);
  }
}

void UserSetupSPI() {
  SPI.setMISO(USER_GPIO_SPI1_MISO);
  SPI.setMOSI(USER_GPIO_SPI1_MOSI);
  SPI.setSCLK(USER_GPIO_SPI1_SCK);
  SPI.setSSEL(NC);
  SPI.begin();
}

void UserSetupUSART() {
  Xbee.begin(115200);
}

void UserSetupI2C() {
  i2c4.setClock(100000);
  i2c4.begin();
  i2c4.setTimeout(50);
}

// Bit-bang 9 SCL pulses to release a slave stuck mid-byte, then reinit I2C4.
// Called when elapsed-time detection in CB_ReadI2C indicates a hung transfer.
static void RecoverI2C4() {
  pinMode(USER_GPIO_I2C4_SDA, OUTPUT_OPEN_DRAIN);
  pinMode(USER_GPIO_I2C4_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(USER_GPIO_I2C4_SDA, HIGH);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(USER_GPIO_I2C4_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(USER_GPIO_I2C4_SCL, HIGH);
    delayMicroseconds(5);
  }
  // Issue a STOP condition
  digitalWrite(USER_GPIO_I2C4_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(USER_GPIO_I2C4_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(USER_GPIO_I2C4_SDA, HIGH);
  delayMicroseconds(5);
  i2c4.end();
  UserSetupI2C();
}

void UserSetupSensor() {
  // ── INA236 Battery Monitor (I2C 0x40) ─────────────────────
  Serial.println("\n[SENSOR] INA236 Battery Monitor (I2C 0x40)");
  pvalid.ina = ina.begin();
  Serial.print("  Init: ");
  Serial.println(pvalid.ina ? "OK" : "FAIL");
  if (pvalid.ina) {
    ina.setADCRange(0);
    ina.setMaxCurrentShunt(8, 0.008);
    ina.setAverage(INA236_64_SAMPLES);
  }

  // ── BNO08x AHRS (I2C) ─────────────────────────────────────
  Serial.println("\n[SENSOR] BNO08x AHRS (I2C)");
  digitalWrite(BNO08X_RESET, 1);
  pvalid.bno = bno.begin(BNO08X_ADDR, i2c4, USER_GPIO_BNO_INT1, BNO08X_RESET);
  Serial.print("  Init: ");
  Serial.println(pvalid.bno ? "OK" : "FAIL");
  if (pvalid.bno) {
    bno.enableRotationVector();
  }

  // ── u-blox M10S GPS (I2C 0x42) ────────────────────────────
  Serial.println("\n[SENSOR] u-blox M10S GPS (I2C 0x42)");
  pvalid.m10s = m10s.begin(i2c4, 0x42);
  Serial.print("  Init: ");
  Serial.println(pvalid.m10s ? "OK" : "FAIL");
  if (pvalid.m10s) {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(10, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);

    // Poll up to 2 s for a confirmed PVT from the battery-backed RTC.
    // getConfirmedDate/Time is stricter than getTimeValid: the module must have
    // cross-checked the RTC against an incoming signal at least once.
    Serial.print("  RTC time: ");
    const uint32_t t_rtc     = millis();
    bool           rtc_valid = false;
    while (millis() - t_rtc < 2000) {
      if (m10s.checkUblox() && m10s.getConfirmedDate() && m10s.getConfirmedTime()) {
        data.hh = m10s.getHour();
        data.mm = m10s.getMinute();
        data.ss = m10s.getSecond();
        snprintf(data.utc, sizeof(data.utc), "%02d:%02d:%02d",
                 data.hh, data.mm, data.ss);
        Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\n",
                      m10s.getYear(), m10s.getMonth(), m10s.getDay(),
                      data.hh, data.mm, data.ss);
        rtc_valid = true;
        break;
      }
      delay(50);
    }
    if (!rtc_valid) Serial.println("N/A");
  }

  // ── VL53L1X ToF Distance (I2C 0x29) ──────────────────────
  Serial.println("\n[SENSOR] VL53L1X ToF Distance (I2C 0x29)");
  tof.setI2CAddress(0x29);
  pvalid.tof = (tof.begin() == 0);
  if (pvalid.tof) {
    tof.setDistanceModeLong();
    tof.setROI(4, 4, 50);
    tof.startTemperatureUpdate();
    tof.setSigmaThreshold(30);
    // tof.setDistanceThreshold(300, 65535, 1);  // WINDOW_ABOVE: data-ready only when distance > 500 mm (50 cm)
  }
  Serial.print("  Init: ");
  Serial.println(pvalid.tof ? "OK" : "FAIL");
}

void UserSetupSD() {
  // Force-reset SDMMC1 to flush stale DMA/FIFO state from the previous session.
  // Without this, HAL_SD_ReadBlocks blocks forever in the RXFIFOHF polling loop
  // on warm resets (watchdog, debugger, NVIC reset) where the peripheral is not
  // power-cycled.
  __HAL_RCC_SDMMC1_FORCE_RESET();
  delay(20);
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  delay(250);  // SD spec: 250 ms power-on stabilisation

  SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1, USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);
  pvalid.sd = SD.begin();

  // SDMMC1 IRQ defaults to priority 0 (above FreeRTOS configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY).
  // Any FreeRTOS API called from within the SDMMC ISR at priority 0 corrupts
  // scheduler internals.  Lower it here so the ISR sits below the syscall ceiling.
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

  if (pvalid.sd) {
    fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
    fs_sd.open_one<FsMode::WRITE>();
    fs_sd.file() << "ID,COUNT,EPOCH,MILLIS,MODE,"
                    "AX,AY,AZ,ACC,ACC_KF,"
                    "GX,GY,GZ,"
                    "MX,MY,MZ,"
                    "UTC,GPS_ALT,LAT,LON,SIV,"
                    "VEL,POS,ALT,TEMP,PRESS,AGL,ALT_REF,APOGEE\r\n";
    fs_sd.file().flush();
  }
}
/* END USER SETUP */

/* BEGIN USER THREADS */
void CB_ReadIMU(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_IMU_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.imu = uxTaskGetStackHighWaterMark(NULL);
#endif
    mtx_spi.exec(ReadIMU);

    const double &ax = data.imu[0].acc_x;
    const double &ay = data.imu[0].acc_y;
    const double &az = data.imu[0].acc_z;

    // Total acceleration
    acc = std::sqrt(std::abs(ax * ax) + std::abs(ay * ay) + std::abs(az * az));

    // Compensate for gravity
    acc = acc - G;

    // Update KF with measurement
    mtx_kf.exec([&]() { filter_acc.kf.update({acc}); });
  });
}

void CB_ReadAltimeter(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_ALTIMETER_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.altimeter = uxTaskGetStackHighWaterMark(NULL);
#endif
    mtx_spi.exec(ReadAltimeter);

    // Update KF with measurement (real or simulated — altitude_m is already set correctly)
    mtx_kf.exec([&]() {
      filter_alt.kf.update({data.altimeter[0].altitude_m});
      if (!simActivated && !simEnabled)
        alt_agl = data.altimeter[0].altitude_m - alt_ref;
      if (alt_agl > apogee_raw)
        apogee_raw = alt_agl;
    });
  });
}


// Each I2C sensor runs in its own thread at its own rate.
// All threads share mtx_i2c so they never access i2c4 concurrently.
// Per-read timing inside the mutex detects a hung bus and triggers recovery.

void CB_ReadMAG(void *) {
  static uint8_t hung_count = 0;
  hal::rtos::interval_loop(RA_INTERVAL_MAG_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.mag = uxTaskGetStackHighWaterMark(NULL);
#endif
    mtx_i2c.exec([&]() {
      const uint32_t t0 = millis();
      ReadMAG();
      if (millis() - t0 > 40) {
        if (++hung_count >= 2) {
          hung_count = 0;
          RecoverI2C4();
        }
      } else {
        hung_count = 0;
      }
    });
    mtx_nav.exec([&]() { nav_state.yaw = data.yaw; });
  });
}

void CB_ReadGNSS(void *) {
  static uint8_t hung_count = 0;
  hal::rtos::interval_loop(RA_INTERVAL_GNSS_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.gnss = uxTaskGetStackHighWaterMark(NULL);
#endif
    mtx_i2c.exec([&]() {
      const uint32_t t0 = millis();
      ReadGNSS();
      if (millis() - t0 > 40) {
        if (++hung_count >= 2) {
          hung_count = 0;
          RecoverI2C4();
        }
      } else {
        hung_count = 0;
      }
    });

    if (data.gps_fresh && gps_fixed) {
      mtx_kf.exec([&]() {
        // Update east F with current latitude so the coupling dt/(R_lat·cos(lat)) stays accurate
        auto         F_e   = vdt.generate_F();
        const double R_lon = GPS_R_LAT * std::cos(data.latitude * (PI / 180.0));
        F_e[0][1] /= R_lon;
        F_e[0][2] /= R_lon;
        filter_nav_e.F = F_e;

        // Feed position + velocity measurements into both GPS KFs
        filter_nav_n.kf.update({data.latitude, data.velocity_n});
        filter_nav_e.kf.update({data.longitude, data.velocity_e});
      });
    }

    // Snapshot filtered GPS state. KF always runs; guidance uses KF or raw depending on RA_USE_KF_GPS flag.
    double kf_lat, kf_lon, kf_vn, kf_ve;
    mtx_kf.exec([&]() {
      kf_lat = filter_nav_n.kf.state_vector()[0];
      kf_lon = filter_nav_e.kf.state_vector()[0];
      kf_vn  = filter_nav_n.kf.state_vector()[1];
      kf_ve  = filter_nav_e.kf.state_vector()[1];
    });
    if (gps_fixed) {
      if (RA_USE_KF_GPS) {
        data.latitude    = kf_lat;
        data.longitude   = kf_lon;
        data.velocity_n  = kf_vn;
        data.velocity_e  = kf_ve;
        current_location = {kf_lat, kf_lon};
      } else {
        current_location = {data.latitude, data.longitude};
      }
    }
    mtx_nav.exec([&]() {
      if (RA_USE_KF_GPS) {
        nav_state.location = {kf_lat, kf_lon};
        nav_state.vn       = kf_vn;
        nav_state.ve       = kf_ve;
      } else {
        nav_state.location = {data.latitude, data.longitude};
        nav_state.vn       = data.velocity_n;
        nav_state.ve       = data.velocity_e;
      }
      nav_state.heading = data.heading_gps;
      nav_state.fixed   = gps_fixed;
    });
  });
}

void CB_ReadINA(void *) {
  static uint8_t hung_count = 0;
  hal::rtos::interval_loop(RA_INTERVAL_INA_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.ina = uxTaskGetStackHighWaterMark(NULL);
#endif
    mtx_i2c.exec([&]() {
      const uint32_t t0 = millis();
      ReadINA();
      if (millis() - t0 > 40) {
        if (++hung_count >= 2) {
          hung_count = 0;
          RecoverI2C4();
        }
      } else {
        hung_count = 0;
      }
    });
  });
}

void CB_ReadTOF(void *) {
  // ReadTOF() splits its two I2C transactions across a 50 ms gap so the bus
  // is free while the sensor integrates.  Do NOT wrap it in mtx_i2c.exec()
  // here — that would deadlock on the non-recursive osMutex.
  static uint8_t hung_count  = 0;
  static uint8_t stale_count = 0;
  hal::rtos::interval_loop(RA_INTERVAL_TOF_READING, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.tof = uxTaskGetStackHighWaterMark(NULL);
#endif
    if (!pvalid.tof) return;

    data.tof_fresh    = false;  // cleared here; set true only on a successful read
    const uint32_t t0 = millis();
    ReadTOF();

    // Hung-bus detection: normal cycle is ~55 ms; a full hang (both I2C phases
    // aborting at the 50 ms HAL timeout) takes ~150 ms.  120 ms sits between them.
    if (millis() - t0 > 120) {
      if (++hung_count >= 2) {
        hung_count  = 0;
        stale_count = 0;
        mtx_i2c.exec([&]() { RecoverI2C4(); });
      }
    } else {
      hung_count = 0;
    }

    // Stale-data detection: sensor stopped delivering results
    if (data.tof_fresh) {
      stale_count = 0;
    } else if (++stale_count >= 10) {
      // 10 consecutive cycles (~1 s) with no fresh data — reinitialise sensor
      stale_count = 0;
      mtx_i2c.exec([&]() {
        tof.stopRanging();
        pvalid.tof = (tof.begin() == 0);
      });
    }
  });
}

void CB_EvalFSM(void *) {
  uint32_t true_interval;
  hal::rtos::interval_loop(RA_INTERVAL_FSM_EVAL, true_interval, [&]() -> void {
    const uint32_t delta_interval = true_interval < RA_INTERVAL_FSM_EVAL
                                      ? RA_INTERVAL_FSM_EVAL - true_interval
                                      : true_interval - RA_INTERVAL_FSM_EVAL;

    if (true_interval != 0 &&                             // Excluding the first run
        delta_interval > RA_JITTER_TOLERANCE_FSM_EVAL) {  // If tick jitter is too much
      // Update dt for KF
      vdt.update_dt(static_cast<double>(true_interval) * 0.001);

      // Regenerate F with the new dt
      filter_acc.F = vdt.generate_F();
      filter_alt.F = vdt.generate_F();
      {
        auto F = vdt.generate_F();
        F[0][1] /= GPS_R_LAT;
        F[0][2] /= GPS_R_LAT;
        filter_nav_n.F = F;
        // filter_nav_e.F is updated in CB_ReadGNSS with the current cos(lat) factor
      }
    }

#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.fsm = uxTaskGetStackHighWaterMark(NULL);
#endif

    // Predict states to "now" — locked so sensor update() tasks cannot race predict()
    mtx_kf.exec([&]() {
      filter_acc.kf.predict();
      filter_alt.kf.predict();
      filter_nav_n.kf.predict();
      filter_nav_e.kf.predict();
    });

    // FSM with predicted states
    EvalFSM();
  });
}

void CB_AutoZeroAlt(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_AUTOZERO, [&]() -> void {
    AutoZeroAlt();
  });
}

void CB_ConstructData(void *) {
  uint8_t cpu_div = 0;
  hal::rtos::interval_loop(RA_INTERVAL_CONSTRUCT, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.construct = uxTaskGetStackHighWaterMark(NULL);
#endif
    if (++cpu_div >= 10) {
      cpu_div       = 0;
      data.cpu_temp = ReadCPUTemp();
    }
    ConstructString();
  });
}

void CB_SDLogger(void *) {
  hal::rtos::interval_loop(LoggerInterval(), [&]() -> TickType_t { return LoggerInterval(); }, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.sdlog = uxTaskGetStackHighWaterMark(NULL);
#endif
    static String snap;
    mtx_buf.exec([&]() {
      snap = std::move(sd_buf);  // drain all accumulated rows atomically
      sd_buf.reserve(512);       // restore capacity so next += doesn't alloc from scratch
    });
    if (snap.length() == 0) return;

    bool ok = false;
    mtx_sdio.exec([&]() {
      const size_t written = fs_sd.file().write(snap.c_str(), snap.length());
      ok = (written == snap.length());
      if (ok) fs_sd.file().flush();
    });

    if (!ok) {  // re-initialize SD on write failure
      mtx_sdio.exec([&]() {
        fs_sd.close_one();
        UserSetupSD();
      });
    } });
}

void CB_Transmit(void *) {
  hal::rtos::interval_loop(RA_TX_INTERVAL_MS, [&]() -> TickType_t { return RA_TX_INTERVAL_MS; }, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
      stack_hwm.transmit = uxTaskGetStackHighWaterMark(NULL);
#endif
      if (telemetry_enabled) {
        String snap;
        mtx_buf.exec([&]() { snap = tx_buf; });
        mtx_uart.exec([&]() { Xbee.println(snap); });
        packet_count++;
      } });
}

void CB_Control(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_Controlling, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.control = uxTaskGetStackHighWaterMark(NULL);
#endif
    NavState snap;

    if (simActivated) {
      // Advance kinematic sim — dt matches the task interval
      SimNav::update(RA_INTERVAL_Controlling * 0.001);
      snap.location = {SimNav::lat, SimNav::lon};
      snap.vn       = SimNav::vn;
      snap.ve       = SimNav::ve;
      snap.yaw      = SimNav::yaw;
      snap.heading  = SimNav::heading;
      snap.fixed    = SimNav::alt > 0.0;
      mtx_kf.exec([&]() { alt_agl = SimNav::alt; });
    } else {
      mtx_nav.exec([&]() { snap = nav_state; });
    }


    // In real flight only run the spool servos during paraglider descent.
    if (fsm.state() != UserState::PAYLOAD_REALEASE) return;

    // Smooth yaw through sin/cos EMA to handle 0/360 wrap.  Raw body yaw had
    // 103 deg std in flight, injecting noise into theta_offset at 20 Hz and
    // causing constant sector crossings.  alpha=0.15 -> ~300 ms time constant.
    static double    yaw_sin = 0.0, yaw_cos = 1.0;
    constexpr double YAW_EMA_ALPHA = 0.15;
    double           yr            = (snap.yaw + SPOOL_PHYSICAL_OFFSET) * DEG_TO_RAD;
    yaw_sin                        = Guidance::ema_filter(std::sin(yr), yaw_sin, YAW_EMA_ALPHA);
    yaw_cos                        = Guidance::ema_filter(std::cos(yr), yaw_cos, YAW_EMA_ALPHA);
    double filtered_yaw            = std::atan2(yaw_sin, yaw_cos) * RAD_TO_DEG;
    if (filtered_yaw < 0.0) filtered_yaw += 360.0;

    servo_target_angles = controller.guidance.update(snap.location, target_location, alt_agl, snap.vn, snap.ve, filtered_yaw, snap.heading);
    controller.servo_pid_update(servo_target_angles);
  });
}

void CB_ReceiveCommand(void *) {
  hal::rtos::interval_loop(10ul, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.receive = uxTaskGetStackHighWaterMark(NULL);
#endif
    bool cmd_ready = false;

    // 1. Read bytes from Xbee — hold mtx_uart only while touching the UART
    mtx_uart.exec([&]() -> void {
      while (Xbee.available()) {
        char c = Xbee.read();
        if (c == '\n') {
          rx_message.trim();
          cmd_ready = true;
        } else {
          rx_message += c;
        }
      }
    });

    // 2. Process Xbee command — outside both mutexes; Serial prints inside mtx_cdc
    if (cmd_ready) {
      mtx_cdc.exec([&]() -> void {
        Serial.println(rx_message);
      });
      HandleCommand(rx_message);
      rx_message = "";
    }

    // 3. Mirror uplink on USB-CDC when debug is active (separate buffer — never races rx_message)
    if constexpr (RA_USB_DEBUG_ENABLED) {
      static String cdc_message;
      bool          cdc_ready = false;
      mtx_cdc.exec([&]() -> void {
        while (Serial.available()) {
          char c = Serial.read();
          if (c == '\n') {
            cdc_message.trim();
            cdc_ready = true;
          } else {
            cdc_message += c;
          }
        }
      });
      if (cdc_ready) {
        HandleCommand(cdc_message);
        cdc_message = "";
      }
    }
  });
}

void CB_DebugLogger(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.debug = uxTaskGetStackHighWaterMark(NULL);
#endif
    String snap;
    mtx_buf.exec([&]() { snap = tx_buf; });
    mtx_cdc.exec([&]() -> void {
      Serial.println(snap);
#ifdef RA_STACK_HWM_ENABLED
      // alloc_words = stack_size_bytes / 4  (StackType_t = 4 bytes on ARM Cortex-M)
      struct HwmEntry {
        const char *name;
        UBaseType_t hwm;
        uint32_t    alloc;
      };
      const HwmEntry hwm_tasks[] = {
        { "IMU",       stack_hwm.imu,  512}, // 2048 B
        { "ALT", stack_hwm.altimeter,  512}, // 2048 B
        { "MAG",       stack_hwm.mag,  512}, // 2048 B
        {"GNSS",      stack_hwm.gnss,  512}, // 2048 B
        { "INA",       stack_hwm.ina,  512}, // 2048 B
        { "TOF",       stack_hwm.tof,  512}, // 2048 B
        { "FSM",       stack_hwm.fsm,  512}, // 2048 B
        {"CSTR", stack_hwm.construct,  512}, // 2048 B
        {  "TX",  stack_hwm.transmit,  512}, // 2048 B — commented out
        {  "RX",   stack_hwm.receive, 1024}, // 4096 B — commented out
        {"RTIN",    stack_hwm.retain,  512}, // 2048 B
        { "INS",       stack_hwm.ins, 1024}, // 4096 B — commented out
        { "NEO",       stack_hwm.neo,  256}, // 1024 B
        {"CTRL",   stack_hwm.control, 1024}, // 4096 B — commented out
        {  "SD",     stack_hwm.sdlog,  512}, // 2048 B
        { "DBG",     stack_hwm.debug,  512}, // 2048 B
      };
      uint32_t total_used = 0, total_alloc = 0;
      Serial.print("[HWM free/alloc]");
      for (const auto &t: hwm_tasks) {
        if (t.hwm == 0) continue;  // task not started — skip
        const uint32_t used = t.alloc - t.hwm;
        total_used += used;
        total_alloc += t.alloc;
        Serial.print(' ');
        Serial.print(t.name);
        Serial.print(':');
        Serial.print(t.hwm);
        Serial.print('/');
        Serial.print(t.alloc);
        if (t.hwm < RA_STACK_HWM_MIN_WORDS) Serial.print('!');
      }
      Serial.print(" | USED:");
      Serial.print(total_used);
      Serial.print('/');
      Serial.println(total_alloc);
#endif
    });
  });
}

void CB_RetainDeployment(void *) {
  hal::rtos::interval_loop(100ul, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.retain = uxTaskGetStackHighWaterMark(NULL);
#endif
    RetainDeployment();
  });
}

void EvalINSDeploy() {
  static xcore::sampler_t<RA_INS_SAMPLES, double> sampler_tof;
  static xcore::sampler_t<RA_INS_SAMPLES, double> sampler_baro_near;
  static xcore::sampler_t<RA_INS_SAMPLES, double> sampler_baro_critical;

  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    sampler_tof.set_capacity(RA_INS_SAMPLES, /*recount*/ false);
    sampler_tof.set_threshold(RA_INS_TOF_THRESHOLD, /*recount*/ false);

    sampler_baro_near.set_capacity(RA_INS_SAMPLES, /*recount*/ false);
    sampler_baro_near.set_threshold(RA_INS_NEAR_THRESHOLD, /*recount*/ false);

    sampler_baro_critical.set_capacity(RA_INS_SAMPLES, /*recount*/ false);
    sampler_baro_critical.set_threshold(RA_INS_CRIT_THRESHOLD, /*recount*/ false);
  }

  if (ins_thresholds_dirty) {
    ins_thresholds_dirty = false;
    sampler_tof.set_threshold(RA_INS_TOF_THRESHOLD, /*recount*/ false);
    sampler_baro_near.set_threshold(RA_INS_NEAR_THRESHOLD, /*recount*/ false);
    sampler_baro_critical.set_threshold(RA_INS_CRIT_THRESHOLD, /*recount*/ false);
  }

  const auto state = fsm.state();
  if (state != UserState::PAYLOAD_REALEASE && state != UserState::LANDED) return;
  if (data.deploy) return;

  static double  baro_crit_log[50]{};
  static uint8_t baro_crit_idx  = 0;
  static bool    baro_crit_full = false;

  if (pvalid.tof && data.tof > 0.5f)
    sampler_tof.add_sample(static_cast<double>(data.tof));
  sampler_baro_near.add_sample(alt_agl);
  sampler_baro_critical.add_sample(alt_agl);

  baro_crit_log[baro_crit_idx] = alt_agl;
  baro_crit_idx                = (baro_crit_idx + 1) % 50;
  if (baro_crit_idx == 0) baro_crit_full = true;

  const bool tof_triggered = sampler_tof.is_sampled() &&
                             sampler_tof.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;

  const bool baro_near = sampler_baro_near.is_sampled() &&
                         sampler_baro_near.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;

  const bool baro_critical = sampler_baro_critical.is_sampled() &&
                             sampler_baro_critical.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;

  if (tof_triggered && baro_near)
    data.deploy = 1;  // Condition 1: TOF + barometric near-ground
  if (baro_critical)
    data.deploy = 2;  // Condition 2: barometric critical-altitude backup
  if ((tof_triggered && baro_near) || baro_critical) {
    ActivateDeployment(1);

    // Dump the last 50 alt_agl samples that drove the baro_critical decision
    const uint8_t count = baro_crit_full ? 50 : baro_crit_idx;
    const uint8_t start = baro_crit_full ? baro_crit_idx : 0;
    mtx_cdc.exec([&]() {
      Serial.print("[INS] baro_crit log (");
      Serial.print(count);
      Serial.println(" samples, oldest→newest):");
      for (uint8_t i = 0; i < count; ++i) {
        Serial.print("  ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(baro_crit_log[(start + i) % 50], 2);
      }
    });
  }
}

void CB_INSDeploy(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_FSM_EVAL, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.ins = uxTaskGetStackHighWaterMark(NULL);
#endif
    EvalINSDeploy();
  });
}

void CB_NeoPixelBlink(void *) {
  static uint16_t hue = 0;

  hal::rtos::interval_loop(200ul, [&]() -> void {
#ifdef RA_STACK_HWM_ENABLED
    stack_hwm.neo = uxTaskGetStackHighWaterMark(NULL);
#endif
    uint32_t color = led.gamma32(led.ColorHSV(hue));
    led.setPixelColor(0, color);
    led.setPixelColor(1, color);
    led.show();
    hue += 1820;  // full rainbow in ~36 steps (36 × 50 ms = 1.8 s)
  });
}

void CB_EEPROMWrite(void *) {
  hal::rtos::interval_loop(RA_EEPROM_WRITE_INTERVAL, [&]() -> void {
    EEPROM_Write();
  });
}

/* END USER THREADS */

void UserThreads() {
  hal::rtos::scheduler.create(CB_EvalFSM, {.name = "CB_EvalFSM", .stack_size = 2048, .priority = osPriorityRealtime});
  hal::rtos::scheduler.create(CB_Control, {.name = "CB_Control", .stack_size = 4096, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_INSDeploy, {.name = "CB_INSDeploy", .stack_size = 2048, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ReadIMU, {.name = "CB_ReadIMU", .stack_size = 2048, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.name = "CB_ReadAltimeter", .stack_size = 2048, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ReadMAG, {.name = "CB_ReadMAG", .stack_size = 2048, .priority = osPriorityAboveNormal});
  hal::rtos::scheduler.create(CB_ReadGNSS, {.name = "CB_ReadGNSS", .stack_size = 2048, .priority = osPriorityAboveNormal});
  hal::rtos::scheduler.create(CB_ReadINA, {.name = "CB_ReadINA", .stack_size = 2048, .priority = osPriorityAboveNormal});
  hal::rtos::scheduler.create(CB_ReadTOF, {.name = "CB_ReadTOF", .stack_size = 2048, .priority = osPriorityAboveNormal});

  if constexpr (RA_RETAIN_DEPLOYMENT_ENABLED)
    hal::rtos::scheduler.create(CB_RetainDeployment, {.name = "CB_RetainDeployment", .stack_size = 2048, .priority = osPriorityNormal});


  // if constexpr (RA_AUTO_ZERO_ALT_ENABLED)
  //   hal::rtos::scheduler.create(CB_AutoZeroAlt, {.name = "CB_AutoZeroAlt", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ConstructData, {.name = "CB_ConstructData", .stack_size = 2048, .priority = osPriorityNormal});

  if (pvalid.sd) {
    hal::rtos::scheduler.create(CB_SDLogger, {.name = "CB_SDLogger", .stack_size = 4096, .priority = osPriorityNormal});
  }

  if (RA_EEPROM_READ_ENABLED)
    hal::rtos::scheduler.create(CB_EEPROMWrite, {.name = "CB_EEPROMWrite", .stack_size = 2048, .priority = osPriorityLow});

  hal::rtos::scheduler.create(CB_Transmit, {.name = "CB_Transmit", .stack_size = 2048, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_ReceiveCommand, {.name = "CB_ReceiveCommand", .stack_size = 2048, .priority = osPriorityNormal});

  hal::rtos::scheduler.create(CB_NeoPixelBlink, {.name = "CB_NeoPixelBlink", .stack_size = 1024, .priority = osPriorityNormal});

  if constexpr (RA_USB_DEBUG_ENABLED)
    hal::rtos::scheduler.create(CB_DebugLogger, {.name = "CB_DebugLogger", .stack_size = 2048, .priority = osPriorityNormal});
}

void setup() {
  sd_buf.reserve(4096);

  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupSD();
  UserSetupGPIO();
  UserSetupLowPower(); // beware servoserial
  UserSetupActuator();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupSPI();
  UserSetupI2C();
  UserSetupSensor();
  EEPROM_Init();
  /* END GPIO AND INTERFACES SETUP */

  /* BEGIN FILTERS SETUP */
  filter_alt.F = vdt.generate_F();
  filter_acc.F = vdt.generate_F();
  {
    auto F = vdt.generate_F();
    F[0][1] /= GPS_R_LAT;
    F[0][2] /= GPS_R_LAT;
    filter_nav_n.F = F;
    filter_nav_e.F = F;  // east starts with equatorial approx; updated on first GPS fix
  }
  /* END FILTERS SETUP */

  // IMU
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (!imu[i])
      sensors_health.imu[i] = SensorStatus::SENSOR_NO;
    else if (imu[i]->begin())
      sensors_health.imu[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.imu[i] = SensorStatus::SENSOR_ERR;
  }

  // Altimeter
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (!altimeter[i])
      sensors_health.altimeter[i] = SensorStatus::SENSOR_NO;
    else if (altimeter[i]->begin())
      sensors_health.altimeter[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.altimeter[i] = SensorStatus::SENSOR_ERR;
  }
  /* END SENSORS SETUP */

  Serial.println(printable_sensor_status(sensors_health.altimeter[0]));
  Serial.println(printable_sensor_status(sensors_health.imu[0]));

  // Restore last flight state from EEPROM (only if data was previously defined)
  if constexpr (RA_EEPROM_READ_ENABLED) {
    // Zero altitude reference
    EEPROM_Read();
    delay(4000);
    ReadAltimeter();
    alt_ref = data.altimeter[0].altitude_m;
    servo_a.write(pos_a);  // re-sync servo hardware to restored position
    servo_b.write(pos_b);
    Serial.println("[EEPROM] Restore attempted");
  } else {
    // Zero altitude reference
    delay(4000);
    ReadAltimeter();
    alt_ref = data.altimeter[0].altitude_m;
  }

  bno_cal.init(static_cast<float>(BNO_MOUNT_OFFSET));
  bno_cal.autoNorthLock(bno);

  led.setPixelColor(0, led.Color(0, 0, 255));
  led.setPixelColor(1, led.Color(0, 0, 255));
  led.show();

  /* BEGIN SYSTEM/KERNEL SETUP */
  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
  /* END SYSTEM/KERNEL SETUP */
}

void EvalFSM() {
  static uint32_t                       state_millis_start   = 0;
  static uint32_t                       state_millis_elapsed = 0;
  static xcore::sampler_t<2048, double> sampler;
  static xcore::sampler_t<2048, double> sampler_sec;

  switch (fsm.state()) {
    case UserState::STARTUP: {
      // <--- Next: always transfer --->
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_BUZZER, 0);
      }
      fsm.transfer(UserState::IDLE_SAFE);
      break;
    }

    case UserState::IDLE_SAFE: {
      // <--- Next: always transfer (should wait for uplink) --->
      // <--- Wait for startup countdown instead to prevent accidental operations --->
      if (fsm.on_enter()) {  // Run once
        state_millis_start = millis();
      }

      state_millis_elapsed = millis() - state_millis_start;

      if constexpr (RA_STARTUP_COUNTDOWN_ENABLED) {
        if (state_millis_elapsed >= RA_STARTUP_COUNTDOWN)
          fsm.transfer(UserState::LAUNCH_PAD);
      }
      break;
    }

    case UserState::LAUNCH_PAD: {
      // !!!! Next: DETECT launch !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LAUNCH_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LAUNCH_ACC, /*recount*/ false);

        sampler_sec.reset();
        sampler_sec.set_capacity(RA_LAUNCH_SAMPLES, /*recount*/ false);
        sampler_sec.set_threshold(RA_LAUNCH_ALT, /*recount*/ false);
      }

      // sampler.add_sample(acc);                    // Use raw acceleration, unfiltered
      // sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration
      sampler.add_sample(0);            // Closed
      sampler_sec.add_sample(alt_agl);  // Use filtered alt


      if ((sampler.is_sampled() &&
           sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO) ||
          (sampler_sec.is_sampled() &&
           sampler_sec.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO)) {
        fsm.transfer(UserState::ASCENT);
      }

      break;
    }

    case UserState::ASCENT: {
      // !!!! Next: DETECT apogee !!!!
      if (fsm.on_enter()) {  // Run once
        digitalWrite(USER_GPIO_CAM1, HIGH);
        digitalWrite(USER_GPIO_CAM2, HIGH);
        sampler.reset();
        sampler.set_capacity(RA_APOGEE_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_APOGEE_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      if (apogee_alt_dirty) {
        apogee_alt_dirty = false;
        sampler.set_threshold(RA_APOGEE_ALT, /*recount*/ false);
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));

      if constexpr (RA_FSM_TIME_GUARD_ENABLED) {
        state_millis_elapsed = millis() - state_millis_start;
        if ((state_millis_elapsed >= RA_TIME_TO_APOGEE_MAX) ||
            (state_millis_elapsed >= RA_TIME_TO_APOGEE_MIN &&
             sampler.is_sampled() &&
             sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO))
          fsm.transfer(UserState::APOGEE);
      } else {
        if (sampler.is_sampled() && sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO)
          fsm.transfer(UserState::APOGEE);
      }
      break;
    }

    case UserState::APOGEE: {
      // <--- Next: always transfer --->
      // RA_MAIN_ALT_COMPENSATED = apogee_raw * 0.8;
      hal::rtos::delay_ms(1500);
      fsm.transfer(UserState::DESCENT);
      break;
    }

    case UserState::DESCENT: {
      // <--- Next: always transfer --->
      hal::rtos::delay_ms(1500);
      fsm.transfer(UserState::PROBE_REALEASE);
      break;
    }

    case UserState::PROBE_REALEASE: {
      // !!!! Next: DETECT Payload altitude !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_MAIN_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_MAIN_ALT_COMPENSATED, /*recount*/ false);
        state_millis_start = millis();
      }

      sampler.add_sample(alt_agl);

      const bool cond_alt_lower = sampler.is_sampled() &&
                                  sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;

      if constexpr (false) {
        state_millis_elapsed     = millis() - state_millis_start;
        const bool cond_timeout  = state_millis_elapsed >= RA_TIME_TO_MAIN_MAX;
        const bool cond_min_time = state_millis_elapsed >= RA_TIME_TO_MAIN_MIN;
        if (cond_timeout || (cond_min_time && cond_alt_lower)) {
          ActivateDeployment(0);
          fsm.transfer(UserState::PAYLOAD_REALEASE);
        }
      } else {
        if (cond_alt_lower) {
          ActivateDeployment(0);
          fsm.transfer(UserState::PAYLOAD_REALEASE);
        }
      }
      break;
    }

    case UserState::PAYLOAD_REALEASE: {
      // !!!! Next: DETECT landing !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LANDED_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LANDED_ALT, /*recount*/ false);
      }
      //landed
      sampler.add_sample(alt_agl);

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO) {
        fsm.transfer(UserState::LANDED);
      }

      break;
    }

    case UserState::LANDED: {
      if (fsm.on_enter()) {
        for (size_t i = 0; i < 3; ++i)
          controller.driver.write_speed(ServoDriver::IDS[i], 0);
        hal::rtos::delay_ms(60ul * 1000ul);  // let cameras record for 1 minute after landing
        digitalWrite(USER_GPIO_CAM1, LOW);
        digitalWrite(USER_GPIO_CAM2, LOW);
      }
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_BUZZER, 1);
      }
      break;
    }

    default:
      break;
  }
}


void ReadIMU() {
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (sensors_health.imu[i] != SensorStatus::SENSOR_OK ||
        !imu[i]->read())
      continue;
    data.imu[i].acc_x = imu[i]->acc_x();
    data.imu[i].acc_y = imu[i]->acc_y();
    data.imu[i].acc_z = imu[i]->acc_z() * -1.0;
    data.imu[i].gyr_x = imu[i]->gyr_x();
    data.imu[i].gyr_y = imu[i]->gyr_y();
    data.imu[i].gyr_z = imu[i]->gyr_z();
    data.imu_fresh    = true;
  }
}

void ReadAltimeter() {
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (sensors_health.altimeter[i] != SensorStatus::SENSOR_OK ||
        !altimeter[i]->read())
      continue;

    if (simActivated && simEnabled) {
      double sim_alt_msl             = altitude_msl_from_pressure(simPressure / 100.F);
      data.altimeter[i].altitude_m   = sim_alt_msl;
      data.altimeter[i].pressure_hpa = simPressure / 100.F;
      alt_agl                        = sim_alt_msl;
    } else {
      data.altimeter[i].pressure_hpa = altimeter[i]->pressure_hpa();
      data.altimeter[i].altitude_m   = altimeter[i]->altitude_m();
    }

    data.altimeter[i].temperature = altimeter[i]->temperature();
    data.alt_fresh                = true;
  }
}

void ReadGNSS() {
  if (pvalid.m10s) {
    // checkUblox() drains the I2C RX FIFO without blocking — autoPVT pushes
    // packets automatically. Only commit data when a fresh packet arrived,
    // exactly as getSensorEvent() gates the BNO reads.
    if (m10s.checkUblox()) {
      data.siv             = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
      data.hh              = m10s.getHour(UBLOX_CUSTOM_MAX_WAIT);
      data.mm              = m10s.getMinute(UBLOX_CUSTOM_MAX_WAIT);
      data.ss              = m10s.getSecond(UBLOX_CUSTOM_MAX_WAIT);
      data.timestamp_epoch = m10s.getUnixEpoch(data.timestamp_us);
      data.latitude        = static_cast<double>(m10s.getLatitude()) * 1.e-7;
      data.longitude       = static_cast<double>(m10s.getLongitude()) * 1.e-7;
      data.altitude_msl    = static_cast<float>(m10s.getAltitudeMSL()) * 1.e-3f;
      data.velocity_n      = static_cast<double>(m10s.getNedNorthVel()) * 1.e-3;
      data.velocity_e      = static_cast<double>(m10s.getNedEastVel()) * 1.e-3;
      data.heading_gps     = m10s.getHeading(UBLOX_CUSTOM_MAX_WAIT) * 1.e-5;

      snprintf(data.utc, sizeof(data.utc), "%02d:%02d:%02d",
               data.hh, data.mm, data.ss);

      gps_fixed        = data.siv != 0;
      current_location = {data.latitude, data.longitude};
      data.gps_fresh   = true;
    }
  }
}

void ReadINA() {
  if (pvalid.ina) {
    data.batt_volt = ina.getBusVoltage();
    data.batt_curr = ina.getCurrent();  // A;
    data.ina_fresh = true;
  }
}

void ReadMAG() {
  if (!pvalid.bno) return;

  if (bno.wasReset()) {
    RecoverI2C4();
    hal::rtos::delay_ms(250);
    bno.enableRotationVector();
    if (bno_cal.isCollecting()) bno.enableMagnetometer(10);
  }

  if (!bno.getSensorEvent()) return;

  // Feed raw mag samples into the calibrator while a collection is in progress.
  // update() returns true when the 30-second window closes (sensors already
  // switched back to rotation vector inside _finalize()).
  bno_cal.update(bno);

  if (bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    data.roll  = bno.getRoll() * 180.0 / PI;
    data.pitch = bno.getPitch() * 180.0 / PI;

    // raw_yaw: ENU convention from BNO086 (CCW-positive, 0 = east).
    // Convert to CW-from-north (compass bearing) with the dynamic mount offset.
    const double raw_yaw = 90.0 - bno.getYaw() * 180.0 / PI + bno_cal.mount_offset + MAGNETIC_DECLINATION;
    data.yaw             = std::fmod(std::fmod(raw_yaw, 360.0) + 360.0, 360.0);

    // Track accuracy for north-lock readout
    data.yaw_accuracy = bno.getQuatAccuracy();

    // Auto-save BNO086 DCD once fully calibrated (once per session).
    static bool cal_saved = false;
    if (!cal_saved && data.yaw_accuracy == 3 && !bno_cal.isCollecting()) {
      bno.saveCalibration();
      cal_saved = true;
    }

    data.bno_fresh = true;
  }
}

// Two-phase TOF read: start ranging (brief mutex hold), wait 50 ms with the
// bus free, then read the result (brief mutex hold).  VL53L1X is typically
// ready in ~33 ms, so 50 ms is a safe fixed delay.
// Sets data.tof_fresh = true only when a measurement was actually received.
void ReadTOF() {
  if (!pvalid.tof) return;

  // Phase 1: trigger — ~1 ms on the bus
  mtx_i2c.exec([&]() { tof.startRanging(); });

  // Wait outside the mutex so GPS / BNO / INA can use the bus freely
  hal::rtos::delay_ms(50);

  // Phase 2: read result — ~3 ms on the bus
  mtx_i2c.exec([&]() {
    if (tof.checkForDataReady()) {
      data.tof       = tof.getDistance() * 0.001f;  // mm → m
      data.tof_fresh = true;
      tof.clearInterrupt();  // only clear after a successful read
    }
    tof.stopRanging();
  });
}

void ActivateDeployment(const size_t index) {
  switch (index) {
    case 0: {  // Payload Deployment
      pos_a = RA_SERVO_A_RELEASE;
      servo_a.write(pos_a);
      break;
    }

    case 1: {  // Instrument Deployment
      pos_b = RA_SERVO_B_RELEASE;
      servo_b.write(pos_b);
      break;
    }

    default:
      break;
  }
}

void RetainDeployment() {
  servo_a.write(pos_a);
  servo_b.write(pos_b);
}

void AutoZeroAlt() {
  static xcore::sampler_t<RA_AUTOZERO_SAMPLES, double> sampler;
  sampler.set_threshold(RA_AUTOZERO_VEL, /*recount*/ false);

  switch (fsm.state()) {
    case UserState::IDLE_SAFE:
    case UserState::LAUNCH_PAD: {
      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));
      if (sampler.is_sampled()) {
        if (sampler.under_by_over<double>() > 3.0)  // 75%
          alt_ref = data.altimeter[0].altitude_m;
        sampler.reset();
      }
      break;
    }
    default:
      break;
  }
}

void HandleCommand(const String &rx) {
  xcore::command_parser_t<128, 8, ','> parser;
  if (!parser.parse(rx.c_str())) {
    ++last_nack;
    return;
  }

  // Validate "CMD,1043," prefix
  const char *p0 = parser.argv[0];
  const char *p1 = parser.argv[1];
  if (!p0 ||
      strcmp(p0, "CMD") != 0 ||
      !p1 ||
      strcmp(p1, "1043") != 0) {
    ++last_nack;
    return;
  }

  const char *p2 = parser.argv[2];  // primary command
  const char *p3 = parser.argv[3];  // sub-command or first arg
  const char *p4 = parser.argv[4];  // value

  if (!p2) {
    ++last_nack;
    return;
  }

  // Echo the command portion (rx is unmodified by the parser)
  strncpy(data.cmd_echo, rx.c_str() + 9, sizeof(data.cmd_echo) - 1);
  data.cmd_echo[sizeof(data.cmd_echo) - 1] = '\0';

  /* ========== CX ========== */
  if (strcmp(p2, "CX") == 0) {
    if (!p3) {
      ++last_nack;
      return;
    }
    if (strcmp(p3, "ON") == 0)
      telemetry_enabled = true;
    else if (strcmp(p3, "OFF") == 0)
      telemetry_enabled = false;
    else {
      ++last_nack;
      return;
    }

    /* ========== SIM ========== */
  } else if (strcmp(p2, "SIM") == 0) {
    if (!p3) {
      ++last_nack;
      return;
    }
    if (strcmp(p3, "ENABLE") == 0) {
      simEnabled = true;
    } else if (strcmp(p3, "ACTIVATE") == 0) {
      if (!simEnabled) {
        ++last_nack;
        return;
      }
      simActivated = true;
      SimNav::reset();  // restart kinematic sim from initial conditions
      data.mode[0] = 'S';
      data.mode[1] = '\0';
    } else if (strcmp(p3, "DISABLE") == 0) {
      simEnabled   = false;
      simActivated = false;
      data.mode[0] = 'F';
      data.mode[1] = '\0';
    } else {
      ++last_nack;
      return;
    }

    /* ========== KF ========== */
  } else if (strcmp(p2, "KF") == 0) {
    if (!p3) {
      ++last_nack;
      return;
    }
    if (strcmp(p3, "ON") == 0) {
      RA_USE_KF_GPS = true;
    } else if (strcmp(p3, "OFF") == 0) {
      RA_USE_KF_GPS = false;
    } else {
      ++last_nack;
      return;
    }

    /* ========== SIMP ========== */
  } else if (strcmp(p2, "SIMP") == 0) {
    if (!simEnabled || !simActivated) {
      ++last_nack;
      return;
    }
    if (!p3) {
      ++last_nack;
      return;
    }
    char      *end;
    const long val = strtol(p3, &end, 10);
    if (end == p3 || *end != '\0') {
      ++last_nack;
      return;
    }
    if (val >= 0) simPressure = static_cast<uint32_t>(val);

    /* ========== CAL ========== */
  } else if (strcmp(p2, "CAL") == 0) {
    if (!p3) {
      ReadAltimeter();
      alt_ref = data.altimeter[0].altitude_m;
    } else if (strcmp(p3, "TOF") == 0) {
      // VL53L1X offset calibration at given distance
      if (!pvalid.tof) {
        ++last_nack;
        return;
      }
      if (!p4) {
        ++last_nack;
        return;
      }
      char      *end;
      const long dist_mm = strtol(p4, &end, 10);
      if (end == p4 || *end != '\0' || dist_mm <= 0) {
        ++last_nack;
        return;
      }
      mtx_i2c.exec([&]() { tof.calibrateOffset(static_cast<uint16_t>(dist_mm)); });
    } else if (strcmp(p3, "MAG") == 0) {
      // BNO086 magnetometer calibration sub-commands
      if (!p4) {
        ++last_nack;
        return;
      }
      if (strcmp(p4, "START") == 0) {
        // Begin 30-second hard-iron offset measurement.
        // Rotate sensor through all orientations while this runs.
        if (!pvalid.bno) {
          ++last_nack;
          return;
        }
        mtx_i2c.exec([&]() { bno_cal.startMagCollection(bno); });
      } else if (strcmp(p4, "STATUS") == 0) {
        if (!pvalid.bno) {
          ++last_nack;
          return;
        }
        mtx_i2c.exec([&]() { bno_cal.printStatus(bno); });
      } else if (strcmp(p4, "RESET") == 0) {
        bno_cal.reset(static_cast<float>(BNO_MOUNT_OFFSET));
      } else {
        ++last_nack;
        return;
      }
    } else if (strcmp(p3, "NORTH") == 0) {
      // Lock current heading as 0° (magnetic north).
      // Send this command while the sensor is pointing steadily to magnetic north.
      if (!pvalid.bno) {
        ++last_nack;
        return;
      }
      if (!bno_cal.isAwaitingNorth() && !bno_cal.isIdle()) {
        ++last_nack;
        return;
      }
      // Capture raw yaw (before any offset correction) from the latest sensor read.
      // raw_yaw = 90 - bno.getYaw_deg, which is what ReadMAG uses before adding offsets.
      float raw_yaw = 0.f;
      mtx_i2c.exec([&]() {
        if (bno.getSensorEvent() &&
            bno.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
          raw_yaw = 90.0f - bno.getYaw() * 180.0f / PI;
        }
      });
      bno_cal.setNorthLock(raw_yaw);
    } else {
      ++last_nack;
      return;
    }

    /* ========== SET ========== */
  } else if (strcmp(p2, "SET") == 0) {
    if (!p3 || !p4) {
      ++last_nack;
      return;
    }
    if (strcmp(p3, "MAIN_ALT") == 0) {
      char        *end;
      const double raw = strtod(p4, &end);
      if (end == p4 || *end != '\0' || raw <= 0) {
        ++last_nack;
        return;
      }
      RA_MAIN_ALT_COMPENSATED = raw;
    } else if (strcmp(p3, "APOGEE_ALT") == 0) {
      char        *end;
      const double val = strtod(p4, &end);
      if (end == p4 || *end != '\0' || val <= 0) {
        ++last_nack;
        return;
      }
      RA_APOGEE_ALT    = val;
      apogee_alt_dirty = true;
    } else if (strcmp(p3, "TX_RATE") == 0) {
      char      *end;
      const long hz = strtol(p4, &end, 10);
      if (end == p4 || *end != '\0') {
        ++last_nack;
        return;
      }
      if (hz >= 1 && hz <= 10)
        RA_TX_INTERVAL_MS = 1000u / static_cast<uint32_t>(hz);
    } else if (strcmp(p3, "INS_TOF") == 0) {
      char        *end;
      const double val = strtod(p4, &end);
      if (end == p4 || *end != '\0' || val <= 0) {
        ++last_nack;
        return;
      }
      RA_INS_TOF_THRESHOLD = val;
      ins_thresholds_dirty = true;
    } else if (strcmp(p3, "INS_NEAR") == 0) {
      char        *end;
      const double val = strtod(p4, &end);
      if (end == p4 || *end != '\0' || val <= 0) {
        ++last_nack;
        return;
      }
      RA_INS_NEAR_THRESHOLD = val;
      ins_thresholds_dirty  = true;
    } else if (strcmp(p3, "INS_CRIT") == 0) {
      char        *end;
      const double val = strtod(p4, &end);
      if (end == p4 || *end != '\0' || val <= 0) {
        ++last_nack;
        return;
      }
      RA_INS_CRIT_THRESHOLD = val;
      ins_thresholds_dirty  = true;
    } else {
      ++last_nack;
      return;
    }

    /* ========== MEC ========== */
  } else if (strcmp(p2, "MEC") == 0) {
    if (!p3 || !p4) {
      ++last_nack;
      return;
    }
    if (strcmp(p3, "PL") == 0) {
      if (strcmp(p4, "ON") == 0) {
        pos_a = RA_SERVO_A_RELEASE;
        servo_a.write(pos_a);
      } else if (strcmp(p4, "OFF") == 0) {
        pos_a = RA_SERVO_A_LOCK;
        servo_a.write(pos_a);
      } else {
        ++last_nack;
        return;
      }
    } else if (strcmp(p3, "INS") == 0) {
      if (strcmp(p4, "ON") == 0) {
        pos_b = RA_SERVO_B_RELEASE;
        servo_b.write(pos_b);
      } else if (strcmp(p4, "OFF") == 0) {
        pos_b = RA_SERVO_B_LOCK;
        servo_b.write(pos_b);
      } else {
        ++last_nack;
        return;
      }
    } else if (strcmp(p3, "PAR") == 0) {
      if (strcmp(p4, "CW") == 0) {
        for (size_t i = 0; i < 3; ++i) controller.driver.write_speed(ServoDriver::IDS[i], 100);
        hal::rtos::delay_ms(100);
        for (size_t i = 0; i < 3; ++i) controller.driver.write_speed(ServoDriver::IDS[i], 0);
      } else if (strcmp(p4, "ACW") == 0) {
        for (size_t i = 0; i < 3; ++i) controller.driver.write_speed(ServoDriver::IDS[i], -100);
        hal::rtos::delay_ms(100);
        for (size_t i = 0; i < 3; ++i) controller.driver.write_speed(ServoDriver::IDS[i], 0);
      } else if (strcmp(p4, "OFF") == 0) {
        for (size_t i = 0; i < 3; ++i) controller.driver.write_speed(ServoDriver::IDS[i], 0);
      } else if (strcmp(p4, "ZERO") == 0) {
        servo_target_angles = {0.0, 0.0, 0.0};
        controller.servo_pid_update(servo_target_angles);
      } else {
        ++last_nack;
        return;
      }
    } else {
      ++last_nack;
      return;
    }

    /* ========== SERVO ========== */
  } else if (strcmp(p2, "SERVO") == 0) {
    if (!p3 || !p4) {
      ++last_nack;
      return;
    }
    char *end;
    if (strcmp(p3, "A") == 0) {
      const double a = strtod(p4, &end);
      if (end == p4 || *end != '\0') {
        ++last_nack;
        return;
      }
      pos_a = constrain(a, 0.0, 180.0);
      servo_a.write(pos_a);
    } else if (strcmp(p3, "B") == 0) {
      const double b = strtod(p4, &end);
      if (end == p4 || *end != '\0') {
        ++last_nack;
        return;
      }
      pos_b = constrain(b, 0.0, 180.0);
      servo_b.write(pos_b);
    } else {
      ++last_nack;
      return;
    }

    /* ========== TARGET ========== */
  } else if (strcmp(p2, "TARGET") == 0) {
    if (!p3 || !p4) {
      ++last_nack;
      return;
    }
    char        *end_lat, *end_lon;
    const double lat = strtod(p3, &end_lat);
    const double lon = strtod(p4, &end_lon);
    if (end_lat == p3 || *end_lat != '\0' || end_lon == p4 || *end_lon != '\0') {
      ++last_nack;
      return;
    }
    target_location = {lat, lon};

    /* ========== STATE ========== */
  } else if (strcmp(p2, "STATE") == 0) {
    if (!p3) {
      ++last_nack;
      return;
    }
    UserState target_state;
    if (strcmp(p3, "IDLE_SAFE") == 0)
      target_state = UserState::IDLE_SAFE;
    else if (strcmp(p3, "LAUNCH_PAD") == 0)
      target_state = UserState::LAUNCH_PAD;
    else if (strcmp(p3, "ASCENT") == 0)
      target_state = UserState::ASCENT;
    else if (strcmp(p3, "APOGEE") == 0)
      target_state = UserState::APOGEE;
    else if (strcmp(p3, "DESCENT") == 0)
      target_state = UserState::DESCENT;
    else if (strcmp(p3, "PROBE_REALEASE") == 0)
      target_state = UserState::PROBE_REALEASE;
    else if (strcmp(p3, "PAYLOAD_REALEASE") == 0)
      target_state = UserState::PAYLOAD_REALEASE;
    else if (strcmp(p3, "LANDED") == 0)
      target_state = UserState::LANDED;
    else {
      ++last_nack;
      return;
    }
    fsm.transfer(target_state);

    /* ========== SLEEP ========== */
  } else if (strcmp(p2, "SLEEP") == 0) {
    led.setPixelColor(0, led.Color(0, 0, 255));
    led.setPixelColor(1, led.Color(0, 0, 255));
    led.show();
    LowPower.deepSleep();
    __NVIC_SystemReset();

    /* ========== RESET ========== */
  } else if (strcmp(p2, "RESET") == 0) {
    if (pvalid.sd) {
      mtx_sdio.exec([&]() { fs_sd.close_one(); });
    }
    delay(100);
    __NVIC_SystemReset();

    /* ========== UNKNOWN ========== */
  } else {
    ++last_nack;
    return;
  }

  ++last_ack;
}

void ConstructString() {
  // Snapshot and clear freshness flags before building strings.
  // This mirrors how test_i2c uses local fresh variables per loop iteration:
  // CB_ConstructData only uses data that actually arrived since the last call.
  const bool snap_imu = data.imu_fresh;
  data.imu_fresh      = false;
  const bool snap_alt = data.alt_fresh;
  data.alt_fresh      = false;
  const bool snap_bno = data.bno_fresh;
  data.bno_fresh      = false;
  const bool snap_gps = data.gps_fresh;
  data.gps_fresh      = false;
  const bool snap_ina = data.ina_fresh;
  data.ina_fresh      = false;
  const bool snap_tof = data.tof_fresh;
  data.tof_fresh      = false;

  // Build into locals first — no lock held during string construction or ADC read.
  String local_sd, local_tx;

  csv_stream_lf(local_sd)
    << "DDL"
    << packet_count
    << data.utc
    << data.timestamp_epoch
    << millis()
    << state_string(fsm.state())

    << 1043          // TEAM_ID
    << data.utc      // MISSION_TIME
    << packet_count  // PACKET_COUNT
    << data.mode     // MODE

    // 5–6
    << state_string(fsm.state())                // STATE
    << String(data.altimeter[0].altitude_m, 1)  // ALTITUDE (m, 0.1)

    // 7–11
    << data.altimeter[0].temperature   // TEMPERATURE (°C, 0.1)
    << data.altimeter[0].pressure_hpa  // PRESSURE (kPa, 0.1)
    << data.batt_volt                  // VOLTAGE (V, 0.1)
    << data.batt_curr                  // CURRENT (A, 0.01)

    // 12–14 Gyro
    << data.imu[0].gyr_x  // GYRO_R
    << data.imu[0].gyr_y  // GYRO_P
    << data.imu[0].gyr_z  // GYRO_Y

    // 15–17 Accel
    << data.imu[0].acc_x  // ACCEL_R
    << data.imu[0].acc_y  // ACCEL_P
    << data.imu[0].acc_z  // ACCEL_Y

    // 18–22 GPS
    << data.utc                   // GPS_TIME
    << data.altitude_msl          // GPS_ALTITUDE (m, 0.1)
    << String(data.latitude, 4)   // GPS_LATITUDE
    << String(data.longitude, 4)  // GPS_LONGITUDE
    << data.siv                   // GPS_SATS

    << data.velocity_e
    << data.velocity_n

    << data.cmd_echo  // CMD_ECHO

    << data.heading

    << data.roll
    << data.pitch
    << data.yaw

    << data.tof
    << data.deploy

    << alt_agl
    << alt_ref
    << apogee_raw

    << pos_a  // Servo A
    << pos_b
    << data.cpu_temp
    << controller.last_angles[0]          // SERVO_1_ANGLE (deg)
    << controller.last_angles[1]          // SERVO_2_ANGLE (deg)
    << controller.last_angles[2]          // SERVO_3_ANGLE (deg)
    << servo_target_angles[0]             // SERVO_1_TARGET (deg)
    << servo_target_angles[1]             // SERVO_2_TARGET (deg)
    << servo_target_angles[2]             // SERVO_3_TARGET (deg)
    << controller.guidance.last_bearing;  // BEARING (deg)

  csv_stream_lf(local_tx)
    << 1043          // TEAM_ID
    << data.utc      // MISSION_TIME
    << packet_count  // PACKET_COUNT
    << data.mode     // MODE

    // 5–6
    << state_string(fsm.state())  // STATE
    << String(alt_agl, 1)

    // 7–11
    << String(data.altimeter[0].temperature, 1)          // TEMPERATURE (°C, 0.1)
    << String(data.altimeter[0].pressure_hpa / 10.F, 1)  // PRESSURE (kPa, 0.1)
    << String(data.batt_volt, 1)                         // VOLTAGE (V, 0.1)
    << data.batt_curr                                    // CURRENT (A, 0.01)

    // 12–14 Gyro
    << data.imu[0].gyr_x  // GYRO_R
    << data.imu[0].gyr_y  // GYRO_P
    << data.imu[0].gyr_z  // GYRO_Y

    // 15–17 Accel
    << data.imu[0].acc_x  // ACCEL_R
    << data.imu[0].acc_y  // ACCEL_P
    << data.imu[0].acc_z  // ACCEL_Y

    // 18–22 GPS
    << data.utc                      // GPS_TIME
    << String(data.altitude_msl, 1)  // GPS_ALTITUDE (m, 0.1)
    << String(data.latitude, 4)      // GPS_LATITUDE
    << String(data.longitude, 4)     // GPS_LONGITUDE
    << data.siv                      // GPS_SATS

    << data.velocity_e
    << data.velocity_n
    << data.cmd_echo  // CMD_ECHO

    << std::fmod(std::fmod(data.yaw - SPOOL_PHYSICAL_OFFSET, 360.0) + 360.0, 360.0)
    << data.heading_gps
    << data.tof
    << data.deploy
    << RA_APOGEE_ALT
    << RA_MAIN_ALT_COMPENSATED
    << RA_LAUNCH_ALT
    << RA_INS_CRIT_THRESHOLD
    << servo_target_angles[0]      // SERVO_1_TARGET (deg)
    << controller.last_angles[0]   // SERVO_1_ANGLE (deg)
    << servo_target_angles[1]      // SERVO_2_TARGET (deg)
    << controller.last_angles[1]   // SERVO_2_ANGLE (deg)
    << servo_target_angles[2]      // SERVO_3_TARGET (deg)
    << controller.last_angles[2];  // SERVO_3_ANGLE (deg);  // BEARING (deg)
  // FRESH bitmask: bit0=IMU bit1=ALT bit2=BNO bit3=GPS bit4=INA bit5=TOF
  // << (uint8_t) ((snap_imu << 0) | (snap_alt << 1) | (snap_bno << 2) | (snap_gps << 3) | (snap_ina << 4) | (snap_tof << 5));

  // Atomic update — lock held only for buffer mutation, not string building.
  // sd_buf accumulates rows; CB_SDLogger drains it via std::move (drain-and-clear).
  // tx_buf keeps only the latest frame (telemetry doesn't need history).
  mtx_buf.exec([&]() {
    sd_buf += local_sd;
    tx_buf = std::move(local_tx);
  });
}