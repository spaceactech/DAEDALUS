#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "UserPins.h"          // User's Pins Mapping
#include <ISM6HG256XSensor.h>

//Optional interrupt and shutdown pins.
SPIClass dev_spi(USER_GPIO_SPI1_MOSI, USER_GPIO_SPI1_MISO, USER_GPIO_SPI1_SCK);
/*
   @file    ISM6HG256X_DataLog_Terminal.ino
   @author  STMicroelectornics
   @brief   Example to use the ISM6HG256X inertial measurement sensor.
 *******************************************************************************
   Copyright (c) 2025, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/


ISM6HG256XSensor  sensor(&dev_spi, USER_GPIO_ISM256_NSS);
ISM6HG256X_Axes_t accel, angrate;

void setup() {
  pinMode(USER_GPIO_ISM256_INT1, OUTPUT);
  pinMode(USER_GPIO_ISM256_INT2, OUTPUT);
  digitalWrite(USER_GPIO_ISM256_INT1, 1);
  digitalWrite(USER_GPIO_ISM256_INT2, 1);

  // Serial.begin(115200);
  delay(4000);
  dev_spi.begin();
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();
  Serial.println("finished");
}

void loop() {
  sensor.Get_X_Axes(&accel);
  sensor.Get_G_Axes(&angrate);

  Serial.print("Accel-X[mg]:");
  Serial.print(accel.x);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel.y);
  Serial.print(",Accel-Z[mg]:");
  Serial.print(accel.z);

  Serial.print(",AngRate-X[mdps]:");
  Serial.print(angrate.x);
  Serial.print(",AngRate-Y[mdps]:");
  Serial.print(angrate.y);
  Serial.print(",AngRate-Z[mdps]:");
  Serial.println(angrate.z);

  delay(1000);
}
