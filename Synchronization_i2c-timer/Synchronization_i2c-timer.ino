#include <Wire.h>

#define I2C_ADDRESS_CENTRAL_SLAVE  0xFE
#define I2C_ADDRESS_SENSOR_NODE    0x23
#define I2C_ADDRESS_TIMER          0x03
#define TOGGLE_GPIO                9

static int counter = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS_TIMER);
  pinMode(TOGGLE_GPIO, OUTPUT);
}

void loop() {
  digitalWrite(TOGGLE_GPIO, true);
  counter++;
  Wire.beginTransmission(I2C_ADDRESS_SENSOR_NODE);
  Wire.write(counter);
  Wire.endTransmission();
  Serial.println(counter);
  digitalWrite(TOGGLE_GPIO, false);
  delay(10);
}
