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

#include <STM32SD.h>
#include <STM32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>
#include <INA236.h>
#include "SparkFun_VL53L1X.h"
/* END INCLUDE SYSTEM LIBRARIES */

/* BEGIN INCLUDE USER'S IMPLEMENTATIONS */
#include "UserConfig.h"
#include "UserPins.h"     // User's Pins Mapping
#include "UserSensors.h"  // User's Hardware Implementations
#include "UserFSM.h"      // User's FSM States
#include "Controlling.h"
/* END INCLUDE USER'S IMPLEMENTATIONS */

/* BEGIN INCLUDE MAIN */
#include "./main.h"
/* END INCLUDE MAIN */

/* BEGIN USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */
/* END USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */

/* BEGIN SENSOR INSTANCES */
SPIClass spi1(USER_GPIO_SPI1_MOSI, USER_GPIO_SPI1_MISO, USER_GPIO_SPI1_SCK);
TwoWire  i2c4(USER_GPIO_I2C4_SDA, USER_GPIO_I2C4_SCL);

HardwareSerial ServoSerial(USER_GPIO_Half);
HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

SFE_UBLOX_GNSS    m10s;
INA236            ina(0x40, &i2c4);
sh2_SensorValue_t sensorValue;
SFEVL53L1X        tof(i2c4);

Adafruit_NeoPixel led(2, USER_GPIO_LED, NEO_GRB + NEO_KHZ800);

SensorIMU *imu[RA_NUM_IMU] = {
  new IMU_ISM256(spi1, USER_GPIO_ISM256_NSS),  // IMU #1
};
SensorAltimeter *altimeter[RA_NUM_ALTIMETER] = {
  new Altimeter_BMP581(USER_GPIO_BMP581_NSS),  // Altimeter #1
};

numeric_vector<3> servo_target_angles;
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
struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];

  String mode = "F";

  uint32_t timestamp_epoch;
  uint32_t timestamp_us{};

  char    utc[9] = "00:00:00";
  uint8_t siv;
  double  latitude;
  double  longitude;
  double  altitude_msl;
  double  velocity_n;
  double  velocity_e;
  uint8_t hh, mm, ss;

  float batt_volt;
  float batt_curr;

  float tof;

  float yaw;
  float pitch;
  float roll;
  float heading;

  bool   deploy;
  String cmd_echo = "ECHO";

} data;

String sd_buf;
String tx_buf;
/* END DATA MEMORY */

/* EEPROM */


Controller controller;

/* BEGIN SD CARD */
FsUtil fs_sd;
/* END SD CARD */

/* BEGIN FILTERS */
xcore::vdt<FILTER_ORDER - 1> vdt(static_cast<double>(RA_INTERVAL_FSM_EVAL) * 0.001);
Filter1T                     filter_acc;
Filter1T                     filter_alt;
/* END FILTERS */

/* BEGIN ACTUATORS */
STM32ServoList servos(TIMER_SERVO);
float          pos_a = RA_SERVO_A_LOCK;
float          pos_b = 90;
/* END ACTUATORS */

/* BEGIN USER PRIVATE VARIABLES */
hal::rtos::mutex_t mtx_sdio;
hal::rtos::mutex_t mtx_spi;
hal::rtos::mutex_t mtx_cdc;
hal::rtos::mutex_t mtx_i2c;
hal::rtos::mutex_t mtx_uart;
/* END USER PRIVATE VARIABLES */

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
      return RA_SDLOGGER_INTERVAL_REALTIME;

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

/* BEGIN USER SETUP */
void UserSetupGPIO() {
  if constexpr (RA_LED_ENABLED) {
    pinMode(USER_GPIO_BUZZER, OUTPUT);
  }
  led.begin();
  led.setBrightness(10);
  led.clear();
  led.setPixelColor(0, led.Color(0, 0, 255));
  led.setPixelColor(1, led.Color(0, 0, 255));
  led.show();
  led.clear();
  digitalToggle(USER_GPIO_BUZZER);
  delay(100);
  digitalToggle(USER_GPIO_BUZZER);
}

void UserSetupActuator() {
  ServoSerial.begin(1'000'000);
  controller.driver.sms_sts.pSerial = &ServoSerial;
  delay(1000);

  // Initialize servo driver
  controller.driver.sms_sts.syncReadBegin(sizeof(ServoDriver::IDS), sizeof(controller.driver.rxPacket), 5);
  for (size_t i = 0; i < sizeof(ServoDriver::IDS); ++i) {
    controller.driver.sms_sts.WheelMode(ServoDriver::IDS[i]);
  }
  // servos.attach(USER_GPIO_SERVO_B, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
  // servos[0].write(0);
  // servos[0].write(180);
}

void UserSetupCDC() {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.begin(115200);
    while (!Serial)
      delay(1);
  }
}

void UserSetupSPI() {
  spi1.setMOSI(USER_GPIO_SPI1_MOSI);
  spi1.setMISO(USER_GPIO_SPI1_MISO);
  spi1.setSCLK(USER_GPIO_SPI1_SCK);
  spi1.begin();
}

void UserSetupUSART() {
  Xbee.begin(115200);
}

void UserSetupI2C() {
  i2c4.setSDA(USER_GPIO_I2C4_SDA);
  i2c4.setSCL(USER_GPIO_I2C4_SCL);
  i2c4.setClock(100000);
  i2c4.begin();
}

// i2c
void UserSetupSensor() {
  if (ina.begin()) {
    ina.setADCRange(0);
    ina.setMaxCurrentShunt(8, 0.008);
    ina.setAverage(INA236_64_SAMPLES);
    Serial.println("Ina Success");
    pvalid.ina = true;
  }

  if (bno086.begin_I2C(BNO08X_ADDR, &i2c4)) {
    Serial.println("bno086 Found!");
    pvalid.bno = true;
    setReports(reportType, reportIntervalUs);
    // bno086.enableReport(SH2_GAME_ROTATION_VECTOR);
  }

  if (m10s.begin(i2c4, 0x42)) {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    pvalid.m10s = true;
    Serial.println("M10S found!");
  }

  if (tof.begin() == 0)  //Begin returns 0 on a good init
  {
    Serial.println("TOF Success");
    pvalid.tof = true;
  }
}
/* END USER SETUP */

/* BEGIN USER THREADS */
void CB_ReadIMU(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_IMU_READING, [&]() -> void {
    mtx_spi.exec(ReadIMU);

    const double &ax = data.imu[0].acc_x;
    const double &ay = data.imu[0].acc_y;
    const double &az = data.imu[0].acc_z;

    // Total acceleration
    acc = std::sqrt(std::abs(ax * ax) + std::abs(ay * ay) + std::abs(az * az));

    // Compensate for gravity
    acc = acc - 1.0;

    // Update KF with measurement
    filter_acc.kf.update({acc});
  });
}

void CB_ReadAltimeter(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_ALTIMETER_READING, [&]() -> void {
    mtx_spi.exec(ReadAltimeter);

    // Update KF with measurement
    if (!simActivated && !simEnabled)
      filter_alt.kf.update({data.altimeter[0].altitude_m});

    // Update altitude above ground
    alt_agl = data.altimeter[0].altitude_m - alt_ref;

    // Update apogee
    if (alt_agl > apogee_raw)
      apogee_raw = alt_agl;
    // Serial.println("CB_ReadAltimeter");
  });
}

// void CB_ReadI2C(void *) {
//   hal::rtos::interval_loop(500ul, [&]() -> void {
//     mtx_i2c.exec([&]() -> void {
//       ReadGNSS();
//       ReadINA();
//       ReadMAG();
//       // ReadTOF();
//       Serial.println("I2C");
//     });
//   });
// }

void CB_ReadGNSS(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_GNSS_READING, [&]() -> void {
    mtx_i2c.exec(ReadGNSS);
  });
}

void CB_ReadINA(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_SENSORS_READING, [&]() -> void {
    mtx_i2c.exec(ReadINA);
  });
}

void CB_ReadMAG(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_MAG_READING, [&]() -> void {
    mtx_i2c.exec(ReadMAG);
  });
}

void CB_ReadTOF(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_TOF_READING, [&]() -> void {
    mtx_i2c.exec(ReadTOF);
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
    }

    // Predict states to "now"
    filter_acc.kf.predict();
    filter_alt.kf.predict();

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
  hal::rtos::interval_loop(RA_INTERVAL_CONSTRUCT, [&]() -> void {
    ConstructString();
  });
}

void CB_SDLogger(void *) {
  hal::rtos::interval_loop(200ul, [&]() -> void {  //LoggerInterval(), LoggerInterval, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file() << sd_buf;
      Serial.println("logged");
    });
  });
}

void CB_SDSave(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file().flush();
      Serial.println("flushed");
    });
  });
}

void CB_Transmit(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    if (telemetry_enabled) {
      mtx_uart.exec([&]() -> void {
        Xbee.println(tx_buf);
        packet_count++;
      });
    }
  });
}

void CB_Control(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_Controlling, [&]() -> void {
    GPSCoordinate current = {data.latitude, data.longitude};
    servo_target_angles   = controller.guidance.update(current, target, alt_agl, data.velocity_n, data.velocity_e, data.yaw);
    controller.servo_pid_update(servo_target_angles);
    // Serial.println("Control");
  });
}

void CB_ReceiveCommand(void *) {
  hal::rtos::interval_loop(10ul, [&]() -> void {
    mtx_uart.exec([&]() -> void {
      while (Xbee.available()) {
        char c = Xbee.read();

        if (c == '\n') {
          rx_message.trim();
          Serial.println(rx_message);
          HandleCommand(rx_message);
          rx_message = "";
        } else {
          rx_message += c;
        }
      }
    });
  });
}

void CB_DebugLogger(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_cdc.exec([&]() -> void {
      Serial.println(tx_buf);
    });
  });
}

void CB_RetainDeployment(void *) {
  hal::rtos::interval_loop(15ul, [&]() -> void {
    RetainDeployment();
  });
}

void CB_NeoPixelBlink(void *) {
  static xcore::FfTimer led_ff(950, 50, millis);

  hal::rtos::interval_loop(1ul, [&]() -> void {
    led_ff.on_rising([]() -> void {
            led.setPixelColor(0, led.Color(255, 0, 0));
            led.setPixelColor(1, led.Color(255, 0, 0));
            led.show();
          })
      .on_falling([]() -> void {
        led.clear();
        led.show();
      });
  });
}

/* END USER THREADS */

void UserThreads() {
  hal::rtos::scheduler.create(CB_EvalFSM, {.name = "CB_EvalFSM", .stack_size = 4096, .priority = osPriorityRealtime});
  hal::rtos::scheduler.create(CB_Control, {.name = "CB_Control", .stack_size = 4096, .priority = osPriorityRealtime});

  hal::rtos::scheduler.create(CB_ReadIMU, {.name = "CB_ReadIMU", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.name = "CB_ReadAltimeter", .stack_size = 8192, .priority = osPriorityHigh});

  // hal::rtos::scheduler.create(CB_ReadI2C, {.name = "CB_ReadI2C", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadGNSS, {.name = "CB_ReadGNSS", .stack_size = 4096, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadINA, {.name = "CB_ReadINA", .stack_size = 4096, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadMAG, {.name = "CB_ReadMAG", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadTOF, {.name = "CB_ReadTOF", .stack_size = 4096, .priority = osPriorityHigh});

  // if constexpr (RA_RETAIN_DEPLOYMENT_ENABLED)
  //   hal::rtos::scheduler.create(CB_RetainDeployment, {.name = "CB_RetainDeployment", .stack_size = 4096, .priority = osPriorityHigh});

  // if constexpr (RA_AUTO_ZERO_ALT_ENABLED)
  //   hal::rtos::scheduler.create(CB_AutoZeroAlt, {.name = "CB_AutoZeroAlt", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ConstructData, {.name = "CB_ConstructData", .stack_size = 4096, .priority = osPriorityBelowNormal});

  // hal::rtos::scheduler.create(CB_SDLogger, {.name = "CB_SDLogger", .stack_size = 16384, .priority = osPriorityNormal});
  // hal::rtos::scheduler.create(CB_SDSave, {.name = "CB_SDSave", .stack_size = 8192, .priority = osPriorityNormal});

  hal::rtos::scheduler.create(CB_Transmit, {.name = "CB_Transmit", .stack_size = 8192, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_ReceiveCommand, {.name = "CB_ReceiveCommand", .stack_size = 8192, .priority = osPriorityNormal});

  hal::rtos::scheduler.create(CB_NeoPixelBlink, {.name = "CB_NeoPixelBlink", .stack_size = 1028, .priority = osPriorityBelowNormal});

  if constexpr (RA_USB_DEBUG_ENABLED)
    hal::rtos::scheduler.create(CB_DebugLogger, {.name = "CB_DebugLogger", .stack_size = 4096, .priority = osPriorityBelowNormal});
}

void setup() {
  /* BEGIN STORAGES SETUP */
  SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1, USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);
  bool status = SD.begin();
  fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
  fs_sd.open_one<FsMode::WRITE>();
  sd_buf.reserve(1024);
  /* CSV Header */
  fs_sd.file() << "ID,COUNT,EPOCH,MILLIS,MODE,"
                  "AX,AY,AZ,ACC,ACC_KF,"
                  "GX,GY,GZ,"
                  "MX,MY,MZ,"
                  "UTC,GPS_ALT,LAT,LON,SIV,"
                  "VEL,POS,ALT,TEMP,PRESS,AGL,ALT_REF,APOGEE\r\n";
  fs_sd.file().flush();
  /* END STORAGES SETUP */

  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupGPIO();
  UserSetupActuator();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupI2C();
  UserSetupSPI();
  UserSetupSensor();
  /* END GPIO AND INTERFACES SETUP */

  /* BEGIN FILTERS SETUP */
  filter_alt.F = vdt.generate_F();
  filter_acc.F = vdt.generate_F();
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

  //ZeroALT
  delay(4000);
  ReadAltimeter();
  alt_ref = data.altimeter[0].altitude_m;
  Serial.println(alt_ref);

  Serial.println(status);
  if (!fs_sd.file()) {
    // Serial.println("SD FILE OPEN FAILED");
  }

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
        sampler_sec.set_threshold(RA_LAUNCH_ALT, /*recount*/ false);  //Balloon
      }

      sampler.add_sample(acc);                    // Use raw acceleration, unfiltered
      sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration

      sampler_sec.add_sample(alt_agl);  // Use filtered alt


      if (sampler.is_sampled() &&
            sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO ||
          sampler_sec.is_sampled() &&
            sampler_sec.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO) {
        if constexpr (RA_LED_ENABLED) {
          digitalWrite(USER_GPIO_BUZZER, 0);
        }
        fsm.transfer(UserState::ASCENT);
      }
      break;
    }

    case UserState::ASCENT: {
      // !!!! Next: DETECT apogee !!!!
      if (fsm.on_enter()) {  // Run once
        // sampler.reset();
        // sampler.set_capacity(RA_APOGEE_SAMPLES, /*recount*/ false);
        // sampler.set_threshold(RA_APOGEE_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      // const double vel = filter_alt.kf.state_vector()[1];
      // sampler.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_APOGEE_MAX)
        fsm.transfer(UserState::APOGEE);
      //     (state_millis_elapsed >= RA_TIME_TO_APOGEE_MIN &&
      //      sampler.is_sampled() &&
      //      sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))

      break;
    }

    case UserState::APOGEE: {
      // <--- Next: activate pyro/servo and always transfer --->
      fsm.transfer(UserState::DESCENT);
      break;
    }

    case UserState::DESCENT: {
      // <--- Next: activate pyro/servo and always transfer --->
      fsm.transfer(UserState::PROBE_REALEASE);
      break;
    }

    case UserState::PROBE_REALEASE: {
      // !!!! Next: DETECT main deployment altitude !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_MAIN_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_MAIN_ALT_COMPENSATED, /*recount*/ false);
        // sampler_overspeed.reset();
        // sampler_overspeed.set_threshold(RA_MAIN_OVERSPEED_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      sampler.add_sample(alt_agl);
      // const double vel = filter_alt.kf.state_vector()[1];
      // sampler_overspeed.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      // const bool cond_timeout   = state_millis_elapsed >= RA_TIME_TO_MAIN_MAX;
      const bool cond_min_time  = state_millis_elapsed >= RA_TIME_TO_MAIN_MIN;
      const bool cond_alt_lower = sampler.is_sampled() &&
                                  sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;
      // const bool cond_overspeed = sampler_overspeed.is_sampled() &&
      //                             sampler_overspeed.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO;

      if (cond_alt_lower) {
        ActivateDeployment(0);
        fsm.transfer(UserState::PAYLOAD_REALEASE);
      }
      break;
    }

    case UserState::PAYLOAD_REALEASE: {
      // !!!! Next: DETECT landing !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LANDED_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LANDED_VEL, /*recount*/ false);

        sampler_sec.reset();
        sampler_sec.set_capacity(RA_INS_SAMPLES, /*recount*/ false);
        sampler_sec.set_threshold(RA_LANDED_VEL, /*recount*/ false);
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));
      sampler_sec.add_sample(alt_agl);

      if (sampler_sec.is_sampled() &&
          sampler_sec.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        ActivateDeployment(1);

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::LANDED);
      break;
    }

    case UserState::LANDED: {
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
    data.imu[i].acc_z = imu[i]->acc_z();
    data.imu[i].gyr_x = imu[i]->gyr_x();
    data.imu[i].gyr_y = imu[i]->gyr_y();
    data.imu[i].gyr_z = imu[i]->gyr_z();
  }
}

void ReadAltimeter() {
  const double qnh_hpa = 1013.25;

  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (sensors_health.altimeter[i] != SensorStatus::SENSOR_OK ||
        !altimeter[i]->read())
      continue;
    if (simActivated && simEnabled) {
      alt_agl = altitude_msl_from_pressure(simPressure / 100.F, qnh_hpa);
    } else {
      data.altimeter[i].pressure_hpa = altimeter[i]->pressure_hpa();
    }

    data.altimeter[i].altitude_m  = altimeter[i]->altitude_m();
    data.altimeter[i].temperature = altimeter[i]->temperature();
  }
}

void ReadGNSS() {
  if (pvalid.m10s) {
    if (m10s.getPVT(UBLOX_CUSTOM_MAX_WAIT)) {
      data.timestamp_epoch = m10s.getUnixEpoch(data.timestamp_us, UBLOX_CUSTOM_MAX_WAIT);
      data.siv             = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
      data.latitude        = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
      data.longitude       = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
      data.altitude_msl    = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;

      data.velocity_n = static_cast<double>(m10s.getNedNorthVel(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;  //m/s
      data.velocity_e = static_cast<double>(m10s.getNedEastVel(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;

      data.hh = m10s.getHour(UBLOX_CUSTOM_MAX_WAIT);
      data.mm = m10s.getMinute(UBLOX_CUSTOM_MAX_WAIT);
      data.ss = m10s.getSecond(UBLOX_CUSTOM_MAX_WAIT);

      snprintf(data.utc, sizeof(data.utc),
               "%02d:%02d:%02d",
               data.hh, data.mm, data.ss);
    }
  }
}

void ReadINA() {
  if (pvalid.ina) {
    data.batt_volt = ina.getBusVoltage();
    data.batt_curr = ina.getCurrent();  // A;
  }
}

void ReadMAG() {
  if (pvalid.bno) {
    taskENTER_CRITICAL();
    if (bno086.wasReset()) {
      Serial.print("sensor was reset ");
      setReports(reportType, reportIntervalUs);
    }

    if (bno086.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }

      // data.roll  = ypr.roll;
      // data.pitch = ypr.pitch;

      // Serial.println(ypr.yaw);
      // Serial.print("  ");
      // Serial.print(ypr.roll);
      // Serial.print("  ");
      // Serial.println(ypr.pitch);
    }
    data.yaw = ypr.yaw;
    // Serial.println(data.yaw);
    taskEXIT_CRITICAL();
  }
}

void ReadTOF() {
  if (pvalid.tof) {
    tof.startRanging();  //Write configuration bytes to initiate measurement
    while (!tof.checkForDataReady()) {
      delay(1);
    }
    data.tof = tof.getDistance();  //Get the result of the measurement from the sensor
    tof.clearInterrupt();
    tof.stopRanging();
    Serial.println(data.tof);
  }
}

void ActivateDeployment(const size_t index) {
  switch (index) {
    case 0: {  // Drogue/First Deployment
      pos_a = RA_SERVO_A_RELEASE;
      // servo_a.write(pos_a);
      break;
    }

    case 1: {  // Main/Second Deployment
      break;
    }

    default:
      break;
  }
}

void RetainDeployment() {
  servos[0].write(pos_a);
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
  // CMD header check
  if (rx.substring(0, 9) != "CMD,1043,") {
    Serial.println("NACK");
    return;
  }

  String cmd = rx.substring(9);
  cmd.trim();

  data.cmd_echo = cmd;

  /* ===== COMMANDS ===== */

  /* ========== CX ========== */
  if (cmd == "CX,ON") {
    telemetry_enabled = true;
  } else if (cmd == "CX,OFF") {
    telemetry_enabled = false;
  }

  /* ========== ST ========== */
  // else if (cmd.substring(0, 3) == "ST,") {
  //   String arg = cmd.substring(3);
  //   arg.trim();

  //   if (arg == "GPS")
  //     missionTime = "GPS_TIME";  // hook GPS later
  //   else
  //     missionTime = arg;
  // }

  /* ========== SIM ========== */
  else if (cmd == "SIM,ENABLE") {
    simEnabled = true;
  } else if (cmd == "SIM,ACTIVATE") {
    if (simEnabled) {
      simActivated = true;
      data.mode    = "S";
    }
  } else if (cmd == "SIM,DISABLE") {
    simEnabled   = false;
    simActivated = false;
    data.mode    = "F";
  }

  /* ========== SIMP ========== */
  else if (cmd.substring(0, 5) == "SIMP,") {
    if (simEnabled && simActivated) {
      simPressure = cmd.substring(5).toInt();
    }
  }

  /* ========== CAL ========== */
  else if (cmd == "CAL") {
    Serial.println("Altitude calibrated to 0 m");
    ReadAltimeter();
    alt_ref = data.altimeter[0].altitude_m;
  }

  /* ========== MEC ========== */
  else if (cmd == "MEC,PL,ON") {
    Serial.println("Payload Release ON");
  } else if (cmd == "MEC,PL,OFF") {
    Serial.println("Payload Release OFF");
  } else if (cmd == "MEC,INS,ON") {
    Serial.println("Instrument Deploy ON");
  } else if (cmd == "MEC,INS,OFF") {
    Serial.println("Instrument Deploy OFF");
  } else if (cmd == "MEC,PAR,ON") {
    Serial.println("Paraglider Rotation ON");
  } else if (cmd == "MEC,PAR,OFF") {
    Serial.println("Paraglider Rotation OFF");
  }

  /* ========== RESET ========== */
  else if (cmd == "RESET") {
    __NVIC_SystemReset();
  }

  /* ========== UNKNOWN ========== */
  else {
    Serial.print("Unknown CMD: ");
    Serial.println(cmd);
    ++last_nack;
    --last_ack;
  }
}

void ConstructString() {
  sd_buf = "";
  csv_stream_lf(sd_buf)
    << "DDL"
    << packet_count
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

    << 0.f << 0.f << 0.f
    << 0.f << 0.f << 0.f

    // 18–22 GPS
    << data.utc                   // GPS_TIME
    << data.altitude_msl          // GPS_ALTITUDE (m, 0.1)
    << String(data.latitude, 4)   // GPS_LATITUDE
    << String(data.longitude, 4)  // GPS_LONGITUDE
    << data.siv                   // GPS_SATS

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
    << ReadCPUTemp();

  tx_buf = "";
  csv_stream_lf(tx_buf)
    << 1043          // TEAM_ID
    << data.utc      // MISSION_TIME
    << packet_count  // PACKET_COUNT
    << data.mode     // MODE

    // 5–6
    << state_string(fsm.state())  // STATE
    // << String(data.altimeter[0].altitude_m, 1)  // ALTITUDE (m, 0.1)
    << String(alt_agl, 1)

    // 7–11
    << String(data.altimeter[0].temperature, 1)          // TEMPERATURE (°C, 0.1)
    << String(data.altimeter[0].pressure_hpa / 10.F, 1)  // PRESSURE (kPa, 0.1)
    << String(data.batt_volt, 1)                         // VOLTAGE (V, 0.1)
    << data.batt_curr                                    // CURRENT (A, 0.01)

    // << 0.f << 0.f << 0.f
    // << 0.f << 0.f << 0.f

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

    << data.cmd_echo  // CMD_ECHO

    << data.yaw;
}