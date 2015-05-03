#include <Wire.h>

#define I2C_ADDRESS_CENTRAL_SLAVE  0xFE

typedef struct SENSOR_NODE{
  float q[4];
} Node;

static Node nodes[4];

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS_CENTRAL_SLAVE);
  Wire.onReceive(receiveI2C);
  Serial.println("Succesfully setup Master Node");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(nodes[1].q[0],4);
  Serial.print(" "); Serial.print(nodes[1].q[1],4); 
  Serial.print(" "); Serial.print(nodes[1].q[2],4); 
  Serial.print(" "); Serial.print(nodes[1].q[3],4);
  Serial.print(", "); 
  Serial.print(nodes[2].q[0],4);
  Serial.print(" "); Serial.print(nodes[2].q[1],4); 
  Serial.print(" "); Serial.print(nodes[2].q[2],4); 
  Serial.print(" "); Serial.print(nodes[2].q[3],4);
  Serial.print(", "); 
  Serial.print(nodes[3].q[0],4);
  Serial.print(" "); Serial.print(nodes[3].q[1],4); 
  Serial.print(" "); Serial.print(nodes[3].q[2],4); 
  Serial.print(" "); Serial.print(nodes[3].q[3],4);
  Serial.println();
//  Serial.print(nodeID);
 // Serial.print('-');
  delay(10);
}

void receiveI2C(int howMany){
  //Serial.print(howMany);
  //Serial.print(": ");
  char buff[10];
  int count = 0;
  while (Wire.available() > 0) {
    buff[count] = Wire.read();
    ++count;
  }
  int nodeID = (int)(buff[0]);
  count = 1;
//  Serial.print(nodeID);
//  Serial.print("-");
  for(int i = 0; i < 4; ++i){
    int16_t temp = ((int16_t)(buff[count] & 0xFF)<< 8) | (int16_t)(buff[count+1] & 0xFF);
    //Serial.print(temp);
    nodes[nodeID].q[i] = (float)(temp)/10000.0;
    //Serial.print(", ");
    //Serial.print(nodes[nodeID].q[i]);
    //Serial.print("\t");
    count += 2;
  }
  //Serial.println();

}
