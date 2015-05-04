#include <Wire.h>
#include <float16.h>
#include <float16.c>

#define I2C_ADDRESS_CENTRAL_SLAVE  0xFE
#define FLOAT_PRINT_PRECISION      6

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
  Serial.print(nodes[1].q[0],FLOAT_PRINT_PRECISION);
  Serial.print(" "); Serial.print(nodes[1].q[1],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[1].q[2],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[1].q[3],FLOAT_PRINT_PRECISION);
  Serial.print(", "); 
  Serial.print(nodes[2].q[0],FLOAT_PRINT_PRECISION);
  Serial.print(" "); Serial.print(nodes[2].q[1],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[2].q[2],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[2].q[3],FLOAT_PRINT_PRECISION);
  Serial.print(", "); 
  Serial.print(nodes[3].q[0],FLOAT_PRINT_PRECISION);
  Serial.print(" "); Serial.print(nodes[3].q[1],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[3].q[2],FLOAT_PRINT_PRECISION); 
  Serial.print(" "); Serial.print(nodes[3].q[3],FLOAT_PRINT_PRECISION);
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
    uint16_t temp = ((uint16_t)(buff[count] & 0xFF)<< 8) | (uint16_t)(buff[count+1] & 0xFF);
    //Serial.print(temp);
    nodes[nodeID].q[i] = returnfloat32(&temp);//(float)(temp)/10000.0;
    //Serial.print(", ");
    //Serial.print(nodes[nodeID].q[i]);
    //Serial.print("\t");
    count += 2;
  }
  //Serial.println();

}
