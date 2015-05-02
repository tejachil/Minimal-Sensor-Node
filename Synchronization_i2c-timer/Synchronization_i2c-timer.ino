#include <Wire.h>

#define I2C_ADDRESS_CENTRAL_SLAVE  0xFE
#define I2C_ADDRESS_SENSOR_NODE    0x23
#define I2C_ADDRESS_TIMER          0x03

static int counter = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS_TIMER);
}

void loop() {
  counter++;
  Wire.beginTransmission(I2C_ADDRESS_SENSOR_NODE);
  Wire.write(counter);
  Wire.endTransmission();
  Serial.println(counter);
  delay(10);
}
