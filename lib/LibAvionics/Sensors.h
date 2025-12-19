#ifndef ROCKET_AVIONICS_TEMPLATE_SENSORS_H
#define ROCKET_AVIONICS_TEMPLATE_SENSORS_H

#include <./Arduino_Extended.h>
#include <./ISA76.h>

enum class SensorStatus : uint8_t {
  SENSOR_OK = 0,    // Sensor healthy
  SENSOR_ERR,       // Sensor error
  SENSOR_NO,        // No sensor on the bus
  SENSOR_UNK = 255  // Unknown status
};

constexpr const char *printable_sensor_status(const SensorStatus status) {
  switch (status) {
    case SensorStatus::SENSOR_OK:
      return "SENSOR_OK";
    case SensorStatus::SENSOR_ERR:
      return "SENSOR_ERR";
    case SensorStatus::SENSOR_NO:
      return "SENSOR_NO";
    case SensorStatus::SENSOR_UNK:
    default:
      return "SENSOR_UNK";
  }
}

class SensorBase {
public:
  SensorBase() = default;

  virtual bool begin() = 0;
  virtual bool read()  = 0;

protected:
  ~SensorBase() = default;
};

class SensorIMU : public SensorBase {
public:
  struct Data {
    double acc_x;
    double acc_y;
    double acc_z;
    double gyr_x;
    double gyr_y;
    double gyr_z;
  };

  SensorIMU() = default;

  virtual double acc_x() = 0;
  virtual double acc_y() = 0;
  virtual double acc_z() = 0;

  virtual double gyr_x() = 0;
  virtual double gyr_y() = 0;
  virtual double gyr_z() = 0;

  auto acc(const bool update = false) -> xcore::tuple<double, double, double> {
    if (update)
      this->read();
    return xcore::make_tuple(this->acc_x(), this->acc_y(), this->acc_z());
  }

  auto gyr(const bool update = false) -> xcore::tuple<double, double, double> {
    if (update)
      this->read();
    return xcore::make_tuple(this->gyr_x(), this->gyr_y(), this->gyr_z());
  }

protected:
  virtual ~SensorIMU() = default;
};

class SensorAltimeter : public SensorBase {
public:
  struct Data {
    double pressure_hpa;
    double altitude_m;
    double temperature;
  };

  SensorAltimeter() = default;

  virtual double pressure_hpa() = 0;
  virtual double temperature()  = 0;

  double altitude_m(const bool update = false, const double qnh_hpa = 1013.25) {
    if (update)
      this->read();
    return altitude_msl_from_pressure(this->pressure_hpa(), qnh_hpa);
  }

protected:
  virtual ~SensorAltimeter() = default;
};

class SensorGNSS : public SensorBase {
public:
  struct Data {
    uint32_t timestamp_epoch;
    String   utc;
    uint8_t  siv;
    double   latitude;
    double   longitude;
    double   altitude_msl;
  };

  SensorGNSS() = default;

  virtual uint32_t timestamp_epoch() = 0;
  virtual uint8_t  siv()             = 0;
  virtual double   latitude()        = 0;
  virtual double   longitude()       = 0;
  virtual double   altitude_msl()    = 0;

protected:
  virtual ~SensorGNSS() = default;
};

#endif  //ROCKET_AVIONICS_TEMPLATE_SENSORS_H
