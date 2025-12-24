#include <Arduino.h>
#include <STM32SD.h>
#include <File_Utility.h>

#include "UserPins.h"
#include "UserConfig.h"

#if __has_include("STM32FreeRTOS.h")
  #define USE_FREERTOS 1
  #include "hal_rtos.h"
#endif

/* =========================================================
   GLOBAL DATA
   ========================================================= */

uint32_t packet_count = 0;

/* IMU */
float acc = 0.12;
float acc_x = 0.01, acc_y = 0.02, acc_z = 0.03;
float gyr_x = 1.1,  gyr_y = 2.2,  gyr_z = 3.3;
float mag_x = 10.1, mag_y = 11.2, mag_z = 12.3;

/* GPS */
char    utc[]   = "12:34:56";
float   gps_alt = 123.4;
float   lat     = 13.7563;
float   lon     = 100.5018;
uint8_t siv     = 12;

/* ALT / KF */
float vel     = -3.21;
float pos     = 456.7;
float alt_m   = 450.2;
float temp    = 28.6;
float press   = 1013.2;
float alt_agl = 50.5;
float alt_ref = 399.7;
float apogee  = 512.3;

/* SD */
FsUtil fs_sd;
String shared_log_buf;

/* RTOS */
hal::rtos::mutex_t mtx_log;

/* =========================================================
   TASK: CONSTRUCT LOG STRING
   ========================================================= */
void Task_ConstructString(void *) {

  hal::rtos::interval_loop(100, [&]() {

    String local;

    local.reserve(256);

    local += "MFC,";
    local += String(packet_count++) + ",";
    local += String(1700000000UL) + ",";   // fake epoch
    local += String(millis()) + ",";
    local += "F,";

    /* ACC */
    local += String(acc_x, 4) + ",";
    local += String(acc_y, 4) + ",";
    local += String(acc_z, 4) + ",";
    local += String(acc, 4) + ",";
    local += String(acc, 4) + ",";

    /* GYRO */
    local += String(gyr_x, 3) + ",";
    local += String(gyr_y, 3) + ",";
    local += String(gyr_z, 3) + ",";

    /* MAG */
    local += String(mag_x, 2) + ",";
    local += String(mag_y, 2) + ",";
    local += String(mag_z, 2) + ",";

    /* GPS */
    local += String(utc) + ",";
    local += String(gps_alt, 1) + ",";
    local += String(lat, 4) + ",";
    local += String(lon, 4) + ",";
    local += String(siv) + ",";

    /* ALT */
    local += String(vel, 2) + ",";
    local += String(pos, 1) + ",";
    local += String(alt_m, 1) + ",";
    local += String(temp, 1) + ",";
    local += String(press, 1) + ",";
    local += String(alt_agl, 1) + ",";
    local += String(alt_ref, 1) + ",";
    local += String(apogee, 1) + "\r\n";

    /* Copy into shared buffer (SHORT mutex) */
    mtx_log.exec([&]() {
      shared_log_buf = local;
    });

    Serial.println("Log created");
  });
}

/* =========================================================
   TASK: SD SAVE
   ========================================================= */
void Task_SDSave(void *) {

  String local_copy;

  hal::rtos::interval_loop(1000, [&]() {

    /* Copy data only */
    mtx_log.exec([&]() {
      local_copy = shared_log_buf;
    });

    /* SD write OUTSIDE mutex */
    fs_sd.file() << local_copy;
    fs_sd.file().flush();

    Serial.println("Logged");
  });
}

/* =========================================================
   SETUP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("SD logger test start");

  /* SDIO pins */
  SD.setDx(
    USER_GPIO_SDIO_DAT0,
    USER_GPIO_SDIO_DAT1,
    USER_GPIO_SDIO_DAT2,
    USER_GPIO_SDIO_DAT3
  );
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);

  if (!SD.begin()) {
    Serial.println("❌ SD init failed");
    while (1);
  }
  Serial.println("✅ SD init OK");

  fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
  fs_sd.open_one<FsMode::WRITE>();

  if (!fs_sd.file()) {
    Serial.println("❌ File open failed");
    while (1);
  }

  /* CSV Header */
  fs_sd.file() <<
    "ID,COUNT,EPOCH,MILLIS,MODE,"
    "AX,AY,AZ,ACC,ACC_KF,"
    "GX,GY,GZ,"
    "MX,MY,MZ,"
    "UTC,GPS_ALT,LAT,LON,SIV,"
    "VEL,POS,ALT,TEMP,PRESS,AGL,ALT_REF,APOGEE\r\n";

  fs_sd.file().flush();

  /* RTOS */
  hal::rtos::scheduler.initialize();

  hal::rtos::scheduler.create(
    Task_ConstructString,
    { .name = "Construct", .stack_size = 8192, .priority = osPriorityHigh }
  );

  hal::rtos::scheduler.create(
    Task_SDSave,
    { .name = "SDSave", .stack_size = 8192, .priority = osPriorityNormal }
  );

  hal::rtos::scheduler.start();
}

/* =========================================================
   LOOP (NOT USED)
   ========================================================= */
void loop() {
  delay(1);
}
