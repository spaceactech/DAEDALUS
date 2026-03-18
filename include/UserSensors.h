#ifndef ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
#define ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H

#include <LibAvionics.h>
#include <lib_xcore>
#include "UserConfig.h"
#include <SPI.h>
#include <ISM6HG256XSensor.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x   bno086(BNO08X_RESET);

constexpr double G = 9.80665;

class IMU_ISM256 final : public SensorIMU {
protected:
  ISM6HG256XSensor imu;

  ISM6HG256X_Axes_t accel{};
  ISM6HG256X_Axes_t gyro{};

  double ax{}, ay{}, az{};
  double gx{}, gy{}, gz{};

public:
  IMU_ISM256(SPIClass &spi, int cs)
      : SensorIMU(), imu(&spi, cs) {}

  bool begin() override {
    return imu.begin() == ISM6HG256X_OK &&
           imu.Enable_X() == ISM6HG256X_OK &&
           imu.Enable_HG_X() == ISM6HG256X_OK &&
           imu.Enable_G() == ISM6HG256X_OK;
  }

  bool read() override {

    if (imu.Get_X_Axes(&accel) != ISM6HG256X_OK)
      return false;

    if (imu.Get_G_Axes(&gyro) != ISM6HG256X_OK)
      return false;

    ax = accel.x;
    ay = accel.y;
    az = accel.z;

    gx = gyro.x;
    gy = gyro.y;
    gz = gyro.z;

    return true; 
  }

  double acc_x() override { return ax * G / 1000; }
  double acc_y() override { return ay * G / 1000; }
  double acc_z() override { return az * G / 1000; }

  double gyr_x() override { return gx / 1000; }
  double gyr_y() override { return gy / 1000; }
  double gyr_z() override { return gz / 1000; }
};

class Altimeter_BMP581 final : public SensorAltimeter {
protected:
  BMP581                    bmp;
  bmp5_osr_odr_press_config bmp_osr =
    {
      .osr_t    = BMP5_OVERSAMPLING_8X,   // T Oversampling
      .osr_p    = BMP5_OVERSAMPLING_64X,  // P Oversampling
      .press_en = 0,                      // UNUSED
      .odr      = 0                       // UNUSED
    };
  bmp5_sensor_data data{};
  uint8_t          cs;

public:
  explicit Altimeter_BMP581(const uint8_t cs) : SensorAltimeter(), cs(cs) {
  }

  bool begin() override {
    return bmp.beginSPI(cs, 10'000'000) == BMP5_OK &&
           bmp.setOSRMultipliers(&bmp_osr) == BMP5_OK &&
           bmp.setODRFrequency(BMP5_ODR_10_HZ) == BMP5_OK;
  }

  bool read() override {
    return bmp.getSensorData(&data) == BMP5_OK;
  }

  double pressure_hpa() override {
    return data.pressure * 0.01;  // Pa -> kPa
  }

  double temperature() override {
    return data.temperature;
  }
};

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno086.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

// class GNSS_M10S final : public SensorGNSS {
// protected:
//   SFE_UBLOX_GNSS m10s;
//   uint32_t       timestamp{};
//   uint32_t       timestamp_us{};
//   String         utc{};
//   uint8_t        gps_siv{};
//   double         gps_lat{};
//   double         gps_lon{};
//   float          gps_alt{};
//   uint8_t        hh, mm, ss;

// public:
//   GNSS_M10S() : SensorGNSS(), m10s() {
//   }

//   bool begin() override {
//     if (m10s.begin(i2c1, 0x42)) {
//       // m10s.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
//       // m10s.setNavigationFrequency(25, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
//       // Serial.println("break");
//       // m10s.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
//       m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
//       Serial.println("break");
//     }
//   }

//   bool read() override {
//     if (m10s.getPVT(UBLOX_CUSTOM_MAX_WAIT)) {
//       timestamp  = m10s.getUnixEpoch(timestamp_us, UBLOX_CUSTOM_MAX_WAIT);
//       gps_siv    = m10s.getSIV(UBLOX_CUSTOM_MAX_WAIT);
//       gps_lat    = static_cast<double>(m10s.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
//       gps_lon    = static_cast<double>(m10s.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
//       gps_alt    = static_cast<float>(m10s.getAltitudeMSL(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f;
//       uint8_t hh = m10s.getHour(UBLOX_CUSTOM_MAX_WAIT);
//       uint8_t mm = m10s.getMinute(UBLOX_CUSTOM_MAX_WAIT);
//       uint8_t ss = m10s.getSecond(UBLOX_CUSTOM_MAX_WAIT);
//     }
//   }

//   uint32_t timestamp_epoch() override {
//     return timestamp;
//   }

//   uint8_t siv() override {
//     return gps_siv;
//   }

//   double latitude() override {
//     return gps_lat;
//   }

//   double longitude() override {
//     return gps_lon;
//   }

//   double altitude_msl() override {
//     return gps_alt;
//   }
// };

#endif  //ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
