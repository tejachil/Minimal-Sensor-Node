#include <SPI.h>
#include <avr/io.h>
#include <Wire.h>

// Software I2C
#define SDA_PORT PORTD
#define SDA_PIN 3
#define SCL_PORT PORTD
#define SCL_PIN 2
#define I2C_FASTMODE 1
#include <SoftI2CMaster.h>

// Float 16 libraries
#include <float16.h>
#include <float16.c>

typedef struct SENSOR_NODE{
  float q[4];
} Node;

static Node nodes[4];

// Multi-Master addresses
#define NODE_ID    1    // Needs to be different for each node
#define I2C_ADDRESS_CENTRAL_SLAVE  0xFE
#define I2C_ADDRESS_SENSOR_NODE    0x23

static char networkBuffer[9]; // Buffer for data on I2C bus for node network

#define MPU9150_ADDRESS 0x68  // Device address when ADO = 0
#define WHO_AM_I_MPU9150 0x75 // Should return 0x68
#define INT_STATUS       0x3A

static bool sendFlag = false;

// Specify sensor full scale
float aRes, gRes, mRes; // scale resolutions per LSB for the sensors
  
// Pin definitions
#define TOGGLE_PIN   9  // primarily for debugging

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer

uint32_t mcount = 0; // used to control magnetometer read rate
uint32_t MagRate;    // read rate for magnetometer data

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
        
void setup(){
    Serial.begin(115200);
    
    if (!i2c_init())
      Serial.println(F("Initialization error. SDA or SCL are low"));
    else
      Serial.println(F("...done"));
      
    Wire.begin(I2C_ADDRESS_SENSOR_NODE);
    Wire.onReceive(receiveI2C);
    
    pinMode(TOGGLE_PIN, OUTPUT);
    // Start device display with ID of sensor
    delay(10);

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t c = readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
    delay(10); 

    calibrateMPU9150(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    delay(10); 

    initMPU9150(); // Inititalize and configure accelerometer and gyroscope
    
    // Get magnetometer calibration from AK8975A ROM
    initAK8975A(magCalibration);
    
    MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
}

void loop(){
    // If intPin goes high or data ready status is TRUE, all data registers have new data
    if (readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
        readAccelData(accelCount);  // Read the x/y/z adc values
        getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes;   
        az = (float)accelCount[2]*aRes;    

        readGyroData(gyroCount);  // Read the x/y/z adc values
        getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;  
        gz = (float)gyroCount[2]*gRes;   

        mcount++;
        if (mcount > 200/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
            readMagData(magCount);  // Read the x/y/z adc values
            mRes = 10.*1229./4096.; // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
            // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
            // like the gyro and accelerometer biases
            magbias[0] = -5.;   // User environmental x-axis correction in milliGauss
            magbias[1] = -95.;  // User environmental y-axis correction in milliGauss
            magbias[2] = -260.; // User environmental z-axis correction in milliGauss

            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
            my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
            mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
            mcount = 0;
        }
    }

    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!  
    // Pass gyro rate as rad/s
    //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    
    if(sendFlag){
        uint16_t qTemp;
        int byteCount = 1;
        networkBuffer[0] = (char)NODE_ID;

        Serial.print("Original: ");
        for(int i = 0; i < 4; ++i){
            qTemp = returnfloat16(&q[i]);
            networkBuffer[byteCount] = (char)((qTemp & 0xFF00) >> 8);
            networkBuffer[byteCount+1] = (char)((qTemp & 0x00FF));
            byteCount += 2;
            Serial.print(qTemp); Serial.print(", "); Serial.print(q[i]); Serial.print("\t");
        }
        Serial.println();

      Wire.beginTransmission(I2C_ADDRESS_CENTRAL_SLAVE);
      Wire.write(networkBuffer);
      if(Wire.endTransmission() == 0)  sendFlag = false;

    }

}

void receiveI2C(int howMany){
  sendFlag = true;
  while (Wire.available() > 0) {
    char c = Wire.read();
  }
}
