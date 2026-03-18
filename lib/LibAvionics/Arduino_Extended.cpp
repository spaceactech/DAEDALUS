#include <./Arduino_Extended.h>

void i2c_detect(Stream &output_stream, TwoWire &i2c_wire,
                const uint8_t addr_from, const uint8_t addr_to) {
  char buf[10];
  output_stream.println("I2C Detector");
  output_stream.print("   ");
  for (uint8_t i = 0; i < 16; i++) {
    sprintf(buf, "%3x", i);
    output_stream.print(buf);
  }

  for (uint8_t addr = 0; addr < 0x80; addr++) {
    if (addr % 16 == 0) {
      sprintf(buf, "\n%02x:", addr & 0xF0);
      output_stream.print(buf);
    }
    if (addr >= addr_from && addr <= addr_to) {
      i2c_wire.beginTransmission(addr);
      if (const uint8_t resp = i2c_wire.endTransmission(); resp == 0) {
        // device found
        // stream.printf(" %02x", addr);
        sprintf(buf, " %02x", addr);
        output_stream.print(buf);
      } else if (resp == 4) {
        // other resp
        output_stream.print(" XX");
      } else {
        // resp = 2: received NACK on transmit of addr
        // resp = 3: received NACK on transmit of data
        output_stream.print(" --");
      }
    } else {
      // addr not scanned
      output_stream.print("   ");
    }
  }
  output_stream.println("\n");
}