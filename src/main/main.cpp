/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>          // Arduino Framework
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
#include <INA236.h>
/* END INCLUDE SYSTEM LIBRARIES */

/* BEGIN INCLUDE USER'S IMPLEMENTATIONS */
#include "UserConfig.h"
#include "UserPins.h"     // User's Pins Mapping
#include "UserSensors.h"  // User's Hardware Implementations
#include "UserFSM.h"      // User's FSM States
/* END INCLUDE USER'S IMPLEMENTATIONS */

/* BEGIN INCLUDE MAIN */
#include "./main.h"
/* END INCLUDE MAIN */

/* BEGIN USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */
/* END USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */

/* BEGIN SENSOR INSTANCES */
SPIClass spi1(USER_GPIO_SPI1_MOSI, USER_GPIO_SPI1_MISO, USER_GPIO_SPI1_SCK);
// TwoWire  i2c1(USER_GPIO_I2C1_SDA, USER_GPIO_I2C1_SCL);

SFE_UBLOX_GNSS   m10s;
INA236           ina(0x40, &i2c1);
Adafruit_LIS3MDL lis;

HardwareSerial Xbee(USER_GPIO_XBEE_RX, USER_GPIO_XBEE_TX);

Adafruit_NeoPixel led(2, USER_GPIO_LED, NEO_GRB + NEO_KHZ800);

SensorIMU *imu[RA_NUM_IMU] = {
  new IMU_ISM256(spi1, USER_GPIO_ISM256_NSS),  // IMU #1
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
struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];

  String mode;

  uint32_t timestamp_epoch;
  uint32_t timestamp_us{};

  String  utc;
  uint8_t siv;
  double  latitude;
  double  longitude;
  double  altitude_msl;
  uint8_t hh, mm, ss;

  float batt_volt;
  float batt_curr;

  float mag_x;
  float mag_y;
  float mag_z;
  float heading;

  String cmd_echo;
} data;

String sd_buf;
String tx_buf;
/* END DATA MEMORY */

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
/* END USER PRIVATE VARIABLES */

/* BEGIN USER PRIVATE FUNCTIONS */
uint32_t LoggerInterval() {
  switch (fsm.state()) {
    case UserState::STARTUP:
    case UserState::IDLE_SAFE:
      return RA_SDLOGGER_INTERVAL_IDLE;

    case UserState::LAUNCH_PAD:
      return RA_SDLOGGER_INTERVAL_SLOW;

    case UserState::POWERED:
    case UserState::COASTING:
      return RA_SDLOGGER_INTERVAL_REALTIME;

    case UserState::DROGUE_DEPLOY:
    case UserState::DROGUE_DESCEND:
    case UserState::MAIN_DEPLOY:
    case UserState::MAIN_DESCEND:
      return RA_SDLOGGER_INTERVAL_FAST;

    case UserState::LANDED:
    case UserState::RECOVERED_SAFE:
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
  led.setPixelColor(0, led.Color(255, 0, 0));
  led.setPixelColor(1, led.Color(255, 0, 0));
  led.show();
  led.clear();
}

void UserSetupActuator() {
  servos.attach(USER_GPIO_SERVO_B, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
}

void UserSetupCDC() {
  if constexpr (RA_USB_DEBUG_ENABLED) {
    Serial.begin(115200);
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
  if (Xbee.available()) {
    Serial.println("Xbee Success");
    led.setPixelColor(0, led.Color(0, 0, 255));
    led.show();
  }
}

void UserSetupI2C() {
  i2c1.setSDA(USER_GPIO_I2C1_SDA);
  i2c1.setSCL(USER_GPIO_I2C1_SCL);
  i2c1.begin();
}

void UserSetupSensor() {
  if (ina.begin()) {
    Serial.println("Ina Success");
  }

  if (lis.begin_I2C(0x1c, &i2c1)) {  // hardware I2C mode, can pass in address & alt Wire
    lis.setPerformanceMode(LIS3MDL_HIGHMODE);
    lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis.setDataRate(LIS3MDL_DATARATE_560_HZ);
    lis.setRange(LIS3MDL_RANGE_16_GAUSS);
    lis.setIntThreshold(500);
    lis.configInterrupt(false, false, true,  // enable z axis
                        true,                // polarity
                        false,               // don't latch
                        true);               // enabled!
    Serial.println("lis Success");
  }

  if (m10s.begin(i2c1, 0x42)) {
    m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
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
    filter_alt.kf.update({data.altimeter[0].altitude_m});

    // Update altitude above ground
    alt_agl = data.altimeter[0].altitude_m - alt_ref;

    // Update apogee
    if (alt_agl > apogee_raw)
      apogee_raw = alt_agl;
  });
}

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
  hal::rtos::interval_loop(RA_INTERVAL_SENSORS_READING, [&]() -> void {
    mtx_i2c.exec(ReadMAG);
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
    sd_buf = "";
    csv_stream_lf(sd_buf)
      << "MFC"
      << packet_count++
      << millis()
      << state_string(fsm.state())

      << data.imu[0].acc_x
      << data.imu[0].acc_y
      << data.imu[0].acc_z
      << acc
      << filter_acc.kf.state()  // ACC

      << filter_alt.kf.state_vector()[1]  // VEL
      << filter_alt.kf.state_vector()[0]  // POS
      << data.altimeter[0].altitude_m
      << data.altimeter[0].pressure_hpa
      << alt_agl
      << alt_ref
      << apogee_raw

      << pos_a  // Servo A
      << ReadCPUTemp()
      //
      ;

    tx_buf = "";
    csv_stream_lf(tx_buf)
      << 1043          // TEAM_ID
      << millis()      // MISSION_TIME (hh:mm:ss UTC)
      << packet_count  // PACKET_COUNT
      << data.mode     // MODE ('F' or 'S')

      // 5–6
      << state_string(fsm.state())  // STATE
      << alt_agl                    // ALTITUDE (meters)

      // 7–11
      << data.altimeter[0].temperature   // TEMPERATURE (°C)
      << data.altimeter[0].pressure_hpa  // PRESSURE (hPa)
      << data.batt_volt                  // VOLTAGE (V)
      << data.batt_curr                  // CURRENT (A)

      // 12–14 Gyro (roll, pitch, yaw)
      << data.imu[0].gyr_x  // GYRO_R
      << data.imu[0].gyr_y  // GYRO_P
      << data.imu[0].gyr_z  // GYRO_Y

      // 15–17 Accel (roll, pitch, yaw)
      << data.imu[0].acc_x  // ACCEL_R
      << data.imu[0].acc_y  // ACCEL_P
      << data.imu[0].acc_z  // ACCEL_Y

      // 18–22 GPS
      << data.utc           // GPS_TIME
      << data.altitude_msl  // GPS_ALTITUDE
      << data.latitude      // GPS_LATITUDE
      << data.longitude     // GPS_LONGITUDE
      << data.siv           // GPS_SATS

      << data.cmd_echo  // CMD_ECHO

      << data.heading;
  });
}

void CB_SDLogger(void *) {
  hal::rtos::interval_loop(LoggerInterval(), LoggerInterval, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file() << "Hellooo, testtt";
    });
  });
}

void CB_SDSave(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file().flush();
    });
  });
}

void CB_Transmit(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    Xbee.println(tx_buf);
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

/* END USER THREADS */

void UserThreads() {
  // hal::rtos::scheduler.create(CB_EvalFSM, {.name = "CB_EvalFSM", .stack_size = 8192, .priority = osPriorityRealtime});

  hal::rtos::scheduler.create(CB_ReadIMU, {.name = "CB_ReadIMU", .stack_size = 8192, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.name = "CB_ReadAltimeter", .stack_size = 8192, .priority = osPriorityHigh});
  // hal::rtos::scheduler.create(CB_ReadGNSS, {.name = "CB_ReadGNSS", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ReadINA, {.name = "CB_ReadINA", .stack_size = 2048, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadMAG, {.name = "CB_ReadMAG", .stack_size = 2048, .priority = osPriorityHigh});

  // if constexpr (RA_RETAIN_DEPLOYMENT_ENABLED)
  // hal::rtos::scheduler.create(CB_RetainDeployment, {.name = "CB_RetainDeployment", .stack_size = 4096, .priority = osPriorityHigh});

  // if constexpr (RA_AUTO_ZERO_ALT_ENABLED)
  // hal::rtos::scheduler.create(CB_AutoZeroAlt, {.name = "CB_AutoZeroAlt", .stack_size = 4096, .priority = osPriorityHigh});

  hal::rtos::scheduler.create(CB_ConstructData, {.name = "CB_ConstructData", .stack_size = 8192, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_SDLogger, {.name = "CB_SDLogger", .stack_size = 8192, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_Transmit, {.name = "CB_Transmit", .stack_size = 2048, .priority = osPriorityNormal});

  if constexpr (RA_USB_DEBUG_ENABLED)
    hal::rtos::scheduler.create(CB_DebugLogger, {.name = "CB_DebugLogger", .stack_size = 4096, .priority = osPriorityBelowNormal});

  hal::rtos::scheduler.create(CB_SDSave, {.name = "CB_SDSave", .stack_size = 8192, .priority = osPriorityLow});
}

void setup() {
  /* BEGIN STORAGES SETUP */
  SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1, USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);
  SD.begin();
  fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
  fs_sd.open_one<FsMode::WRITE>();
  sd_buf.reserve(1024);
  /* END STORAGES SETUP */

  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupGPIO();
  UserSetupActuator();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupI2C();
  UserSetupSPI();
  UserSetupSensor();
  delay(2000);
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

  // GNSS
  // for (size_t i = 0; i < RA_NUM_GNSS; ++i) {
  //   if (!gnss[i])
  //     sensors_health.gnss[i] = SensorStatus::SENSOR_NO;
  //   else if (gnss[i]->begin())
  //     sensors_health.gnss[i] = SensorStatus::SENSOR_OK;
  //   else
  //     sensors_health.gnss[i] = SensorStatus::SENSOR_ERR;
  // }

  /* END SENSORS SETUP */
  Serial.println("sensor setuped");

  /* BEGIN SYSTEM/KERNEL SETUP */
  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
  /* END SYSTEM/KERNEL SETUP */
}

void EvalFSM() {
  static uint32_t                                            state_millis_start   = 0;
  static uint32_t                                            state_millis_elapsed = 0;
  static xcore::sampler_t<2048, double>                      sampler;
  static xcore::sampler_t<RA_MAIN_OVERSPEED_SAMPLES, double> sampler_overspeed;

  switch (fsm.state()) {
    case UserState::STARTUP: {
      // <--- Next: always transfer --->
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_LED, 0);
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
      }

      // sampler.add_sample(acc);  // Use raw acceleration, unfiltered
      sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration

      if (sampler.is_sampled() &&
          sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO) {
        if constexpr (RA_LED_ENABLED) {
          digitalWrite(USER_GPIO_LED, 0);
        }
        fsm.transfer(UserState::POWERED);
      }
      break;
    }

    case UserState::POWERED: {
      // !!!! Next: DETECT motor burnout !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_BURNOUT_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_BURNOUT_ACC, /*recount*/ false);
        state_millis_start = millis();
      }

      // sampler.add_sample(acc);  // Use raw acceleration, unfiltered
      sampler.add_sample(filter_acc.kf.state());  // Use filtered acceleration
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::COASTING);
      break;
    }

    case UserState::COASTING: {
      // !!!! Next: DETECT apogee !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_APOGEE_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_APOGEE_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_APOGEE_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_APOGEE_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::DROGUE_DEPLOY);
      break;
    }

    case UserState::DROGUE_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      ActivateDeployment(/*index*/ 0);
      fsm.transfer(UserState::DROGUE_DESCEND);
      break;
    }

    case UserState::DROGUE_DESCEND: {
      // !!!! Next: DETECT main deployment altitude !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_MAIN_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_MAIN_ALT_COMPENSATED, /*recount*/ false);
        sampler_overspeed.reset();
        sampler_overspeed.set_threshold(RA_MAIN_OVERSPEED_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      sampler.add_sample(alt_agl);
      const double vel = filter_alt.kf.state_vector()[1];
      sampler_overspeed.add_sample(std::abs(vel));
      state_millis_elapsed = millis() - state_millis_start;

      const bool cond_timeout   = state_millis_elapsed >= RA_TIME_TO_MAIN_MAX;
      const bool cond_min_time  = state_millis_elapsed >= RA_TIME_TO_MAIN_MIN;
      const bool cond_alt_lower = sampler.is_sampled() &&
                                  sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO;
      const bool cond_overspeed = sampler_overspeed.is_sampled() &&
                                  sampler_overspeed.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO;

      if ((cond_timeout) ||
          (cond_min_time && cond_alt_lower) ||
          (cond_min_time && cond_overspeed))
        fsm.transfer(UserState::MAIN_DEPLOY);

      break;
    }

    case UserState::MAIN_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      // ActivateDeployment(/*index*/ 1);
      fsm.transfer(UserState::MAIN_DESCEND);
      break;
    }

    case UserState::MAIN_DESCEND: {
      // !!!! Next: DETECT landing !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LANDED_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LANDED_VEL, /*recount*/ false);
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::LANDED);
      break;
    }

    case UserState::LANDED: {
      if constexpr (RA_LED_ENABLED) {
        digitalWrite(USER_GPIO_LED, 1);
      }
      break;
    }

    case UserState::RECOVERED_SAFE: {
      break;
    }
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
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (sensors_health.altimeter[i] != SensorStatus::SENSOR_OK ||
        !altimeter[i]->read())
      continue;
    data.altimeter[i].pressure_hpa = altimeter[i]->pressure_hpa();
    data.altimeter[i].altitude_m   = altimeter[i]->altitude_m();
    data.altimeter[i].temperature  = altimeter[i]->temperature();
  }
}

void ReadGNSS() {
  if (m10s.getPVT(UBLOX_CUSTOM_MAX_WAIT)) {
    data.timestamp_epoch = m10s.getUnixEpoch(data.timestamp_us, UBLOX_CUSTOM_MAX_WAIT);
    data.siv             = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
    data.latitude        = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    data.longitude       = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
    data.altitude_msl    = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;

    data.hh = m10s.getHour(UBLOX_CUSTOM_MAX_WAIT);
    data.mm = m10s.getMinute(UBLOX_CUSTOM_MAX_WAIT);
    data.ss = m10s.getSecond(UBLOX_CUSTOM_MAX_WAIT);
  }
}

void ReadINA() {
  data.batt_volt = ina.getBusVoltage();
  // data.batt_curr = ina.getCurrent_mA() / 1000;  // A;
}

void ReadMAG() {
  lis.read();
  sensors_event_t event;
  lis.getEvent(&event);

  // in uTesla units
  data.mag_x = event.magnetic.x;
  data.mag_y = event.magnetic.y;
  data.mag_z = event.magnetic.z;

  data.heading = atan2(data.mag_y, data.mag_x) * 180.0 / PI;
}


void ActivateDeployment(const size_t index) {
  switch (index) {
    case 0: {  // Drogue/First Deployment
      pos_a = RA_SERVO_A_RELEASE;
      servos[0].write(pos_a);
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
