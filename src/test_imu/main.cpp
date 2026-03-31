#include <SPI.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "UserPins.h"

// Create a new sensor object
SPIClass spi1(USER_GPIO_SPI1_MOSI, USER_GPIO_SPI1_MISO, USER_GPIO_SPI1_SCK);
BMP581   pressureSensor;

// SPI parameters
void setup() {

  // Start serial
  Serial.begin(115200);
  Serial.println("BMP581 Example2 begin!");

  // Initialize the SPI library
  spi1.begin();

  // Check if sensor is connected and initialize
  // Clock frequency is optional (defaults to 100kHz)
  pinMode(USER_GPIO_BMP581_NSS, OUTPUT);
  digitalWrite(USER_GPIO_BMP581_NSS, 0);

  while (pressureSensor.beginSPI(USER_GPIO_BMP581_NSS, 400'000) != BMP5_OK) {
    // Not connected, inform user
    Serial.println("Error: BMP581 not connected, check wiring and CS pin!");

    // Wait a bit to see if connection is established
    delay(1000);
  }

  Serial.println("BMP581 connected!");
}

void loop() {
  // Get measurements from the sensor
  bmp5_sensor_data data = {0, 0};
  int8_t           err  = pressureSensor.getSensorData(&data);

  // Check whether data was acquired successfully
  if (err == BMP5_OK) {
    // Acquisistion succeeded, print temperature and pressure
    Serial.print("Temperature (C): ");
    Serial.print(data.temperature);
    Serial.print("\t\t");
    Serial.print("Pressure (Pa): ");
    Serial.println(data.pressure);
  } else {
    // Acquisition failed, most likely a communication error (code -2)
    Serial.print("Error getting data from sensor! Error code: ");
    Serial.println(err);
  }

  // Only print every second
  delay(1000);
}