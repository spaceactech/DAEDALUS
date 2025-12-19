#ifndef ROCKET_AVIONICS_TEMPLATE_SENSORS_VARIANTNONE_H
#define ROCKET_AVIONICS_TEMPLATE_SENSORS_VARIANTNONE_H

#include <./Sensors.h>

class NoIMU final : public SensorIMU {
public:
  NoIMU() : SensorIMU() {
  }

  bool begin() override {
    return false;
  }

  bool read() override {
    return false;
  }

  double acc_x() override {
    return 0.;
  }

  double acc_y() override {
    return 0.;
  }

  double acc_z() override {
    return 0.;
  }

  double gyr_x() override {
    return 0.;
  }

  double gyr_y() override {
    return 0.;
  }

  double gyr_z() override {
    return 0.;
  }
};

class NoAltimeter final : public SensorAltimeter {
public:
  NoAltimeter() : SensorAltimeter() {
  }

  bool begin() override {
    return false;
  }

  bool read() override {
    return false;
  }

  double pressure_hpa() override {
    return 0.;
  }

  double temperature() override {
    return 0.;
  }
};

class NoGNSS final : public SensorGNSS {
public:
  NoGNSS() : SensorGNSS() {
  }

  bool begin() override {
    return false;
  }

  bool read() override {
    return false;
  }

  uint32_t timestamp_epoch() override {
    return 0ul;
  }

  uint8_t siv() override {
    return 0;
  }

  double latitude() override {
    return 0.;
  }

  double longitude() override {
    return 0.;
  }

  double altitude_msl() override {
    return 0.;
  }
};

#endif  //ROCKET_AVIONICS_TEMPLATE_SENSORS_VARIANTNONE_H
