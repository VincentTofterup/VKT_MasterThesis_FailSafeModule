//
//
// Author: Vincent Klyverts Tofterup, vitof13@student.sdu.dk
// SDU - Master Thesis, stud.polyt, Robotics w. Specialization in UAV Technology
// License: BSD 3-Clause
//
// Originated from: https://playground.arduino.cc/Main/MPU-9150
//
// Reconfigured for the specifications of the parachute project
//
//
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-9150 Product Specification Revision 4.0",
//     PS-MPU-9150A.pdf
//   - "MPU-9150 Register Map and Descriptions Revision 4.0",
//     RM-MPU-9150A-00.pdf
//   - "MPU-9150 9-Axis Evaluation Board User Guide"
//     AN-MPU-9150EVB-00.pdf
//
// The accuracy is 16-bits.
//
// Important datasheets for configuration:
// https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
// https://store.invensense.com/Datasheets/invensense/RM-MPU-9150A-00-v3.0.pdf
//

#include <Wire.h>
#include <Servo.h>
//#include <calib.ino>

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",

#define MPU9150_XA_OFFSET_H        0x06 // User-defined trim values for accelerometer, populate with calibration routine
#define MPU9150_XA_OFFSET_L_TC     0x07
#define MPU9150_YA_OFFSET_H        0x08
#define MPU9150_YA_OFFSET_L_TC     0x09
#define MPU9150_ZA_OFFSET_H        0x0A
#define MPU9150_ZA_OFFSET_L_TC     0x0B

#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_Z        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W

#define MPU9150_XG_OFFS_USRH       0x13  // User-defined trim values for gyroscope, populate with calibration routine
#define MPU9150_XG_OFFS_USRL       0x14
#define MPU9150_YG_OFFS_USRH       0x15
#define MPU9150_YG_OFFS_USRL       0x16
#define MPU9150_ZG_OFFS_USRH       0x17
#define MPU9150_ZG_OFFS_USRL       0x18

#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W

#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W

#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R
#define MPU9150_I2C_MST_STATUS     0x36   // R

#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R

#define MPU9150_ACCEL_XOUT_H       0x3B   // R
#define MPU9150_ACCEL_XOUT_L       0x3C   // R
#define MPU9150_ACCEL_YOUT_H       0x3D   // R
#define MPU9150_ACCEL_YOUT_L       0x3E   // R
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R
#define MPU9150_ACCEL_ZOUT_L       0x40   // R
#define MPU9150_TEMP_OUT_H         0x41   // R
#define MPU9150_TEMP_OUT_L         0x42   // R
#define MPU9150_GYRO_XOUT_H        0x43   // R
#define MPU9150_GYRO_XOUT_L        0x44   // R
#define MPU9150_GYRO_YOUT_H        0x45   // R
#define MPU9150_GYRO_YOUT_L        0x46   // R
#define MPU9150_GYRO_ZOUT_H        0x47   // R
#define MPU9150_GYRO_ZOUT_L        0x48   // R
#define MPU9150_EXT_SENS_DATA_00   0x49   // R
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R
#define MPU9150_EXT_SENS_DATA_07   0x50   // R
#define MPU9150_EXT_SENS_DATA_08   0x51   // R
#define MPU9150_EXT_SENS_DATA_09   0x52   // R
#define MPU9150_EXT_SENS_DATA_10   0x53   // R
#define MPU9150_EXT_SENS_DATA_11   0x54   // R
#define MPU9150_EXT_SENS_DATA_12   0x55   // R
#define MPU9150_EXT_SENS_DATA_13   0x56   // R
#define MPU9150_EXT_SENS_DATA_14   0x57   // R
#define MPU9150_EXT_SENS_DATA_15   0x58   // R
#define MPU9150_EXT_SENS_DATA_16   0x59   // R
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R
#define MPU9150_EXT_SENS_DATA_23   0x60   // R
#define MPU9150_MOT_DETECT_STATUS  0x61   // R
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R



//Adresses for DIRECT ACCESS of the AK8975A
#define AK8975A_ADDRESS  0x0C  // Adress of Compass in MPU9150 (see PS MPU9150 page 28)
#define WHO_AM_I_AK8975A 0x00 // should return 0x48
#define INFO             0x01
#define AK8975A_ST1      0x02  // data ready status bit 0
#define AK8975A_XOUT_L   0x03  // data
#define AK8975A_XOUT_H   0x04
#define AK8975A_YOUT_L   0x05
#define AK8975A_YOUT_H   0x06
#define AK8975A_ZOUT_L   0x07
#define AK8975A_ZOUT_H   0x08
#define AK8975A_ST2      0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975A_ASTC     0x0C  // Self test control
#define AK8975A_ASAX     0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY     0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ     0x12  // Fuse ROM z-axis sensitivity adjustment value


// I2C address 0x69 could be 0x68 depends on your wiring.
int MPU9150_ADDRESS = 0x69;


//Variables where our values can be stored
int cmps[3];
int accl[3];
int gyro[3];
int temp;
volatile int state = LOW;
int val = 0;

#define aRange 16.0       // 2G acceleration resolution (max. sensitivity)
#define Ascale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01
// Possible gyro scales (and their register bit settings) are
// 250 °/s (00), 500 °/s (01), 1000 °/s (10), and 2000 °/s  (11). 
#define gRange 250.0     // 250°/s gyro resolution (max. sensitivity)
#define Gscale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01

float magCalibration[3] = {0,0,0};  // Factory mag calibration scale sensitivity values
// User environmental x,y,z-axis correction in milliGauss (not in the raw values!)
//float magBias[3] = {-5.0, 3.5, -18.0}; //Values for MPU9150 Sensor 1
float magBias[3] = {1.8, 7.0, 8.0};     //Values for MPU9150 Sensor 2

float accG[3];
float gyroDegPerSec[3];




int thres = 700;
int res;
Servo myservo;

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest){  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
    dest[i++] = Wire.read(); 
  }         // Put read results in the Rx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress){
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data){
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void initMPU9150()
{  
 // wake up device
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  delay(200);
  
 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 011; this sets the sample rate at 1 kHz for both
 // Minimum delay time is 4.9 ms which sets the fastest rate at ~200 Hz
 /*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7 RESERVED
 */
  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9150_ADDRESS, MPU9150_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG);
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] (0xE0 = 0b11100000)
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]       (0x18 = 0b00011000)
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG);
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

  // Configure Interrupts and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9150_ADDRESS, MPU9150_INT_PIN_CFG, 0x02);  // Enable bypass 
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x00);  //Disable the MPU6050 as Master --> I2C Bus control by Arduino
}

void initAK8975A(float * destination)
{
  uint8_t rawData[3];  // x/y/z gyro register data stored here
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  delay(10);
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  delay(10);
  Serial.println(F("Magnetometer calibration values: "));
  Serial.print(F("X-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[0], 2);
  Serial.print(F("Y-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[1], 2);
  Serial.print(F("Z-Axis sensitivity adjustment scale value ")); Serial.println(magCalibration[2], 2);

}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// Special thanks for Kris Winers code example on github
void calibrateMPU9150()
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  float gyroBias[3]  = {0, 0, 0}, accelBias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x01);  
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_2, 0x00); 
  delay(200);
  
// Configure device for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9150_ADDRESS, MPU9150_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9150_ADDRESS, MPU9150_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9150_ADDRESS, MPU9150_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9150_ADDRESS, MPU9150_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9150_ADDRESS, MPU9150_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

 for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9150_ADDRESS, MPU9150_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
    }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers for consideration in calculation
  writeByte(MPU9150_ADDRESS, MPU9150_XG_OFFS_USRH, data[0]);
  writeByte(MPU9150_ADDRESS, MPU9150_XG_OFFS_USRL, data[1]);
  writeByte(MPU9150_ADDRESS, MPU9150_YG_OFFS_USRH, data[2]);
  writeByte(MPU9150_ADDRESS, MPU9150_YG_OFFS_USRL, data[3]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZG_OFFS_USRH, data[4]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZG_OFFS_USRL, data[5]);

// Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9150_ADDRESS, MPU9150_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, MPU9150_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers for consideration in calculation
  writeByte(MPU9150_ADDRESS, MPU9150_XA_OFFSET_H, data[0]);
  writeByte(MPU9150_ADDRESS, MPU9150_XA_OFFSET_L_TC, data[1]);
  writeByte(MPU9150_ADDRESS, MPU9150_YA_OFFSET_H, data[2]);
  writeByte(MPU9150_ADDRESS, MPU9150_YA_OFFSET_L_TC, data[3]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_H, data[4]);
  writeByte(MPU9150_ADDRESS, MPU9150_ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for display in the main program
   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
   
// Output for user feedback
   Serial.println(F("Calibrate gyro and accelerometers and load biases in bias registers...."));
   Serial.println(F("MPU9150 acceleration bias"));
   Serial.println(F("  x\t y\t z")); Serial.print("  ");
   Serial.print((int)(1000*accelBias[0])); Serial.print(" \t");
   Serial.print((int)(1000*accelBias[1])); Serial.print(" \t");
   Serial.print((int)(1000*accelBias[2])); Serial.println(" \tmg");       
   Serial.println(F("MPU9150 gyro bias"));
   Serial.println(F("  x\t y\t z")); Serial.print("  ");
   Serial.print(gyroBias[0], 1); Serial.print(" \t");
   Serial.print(gyroBias[1], 1); Serial.print(" \t");
   Serial.print(gyroBias[2], 1); Serial.println(" \tdeg/s");      
   Serial.println(F("Calibration done"));  
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Special thanks for Kris Winers code example on github
void MPU6050SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6]; 
   float SelfTestRes[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(MPU9150_ADDRESS, MPU9150_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(MPU9150_ADDRESS, MPU9150_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(MPU9150_ADDRESS, MPU9150_SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     SelfTestRes[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
   //Output
   Serial.print(F("x-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[0],1); Serial.println(F("% of factory value"));
   Serial.print(F("y-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[1],1); Serial.println(F("% of factory value"));
   Serial.print(F("z-axis self test: acceleration trim within : ")); Serial.print(SelfTestRes[2],1); Serial.println(F("% of factory value"));
   Serial.print(F("x-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[3],1); Serial.println(F("% of factory value"));
   Serial.print(F("y-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[4],1); Serial.println(F("% of factory value"));
   Serial.print(F("z-axis self test: gyration trim within : ")); Serial.print(SelfTestRes[5],1); Serial.println(F("% of factory value"));
   
   if(SelfTestRes[0] < 1.0f && SelfTestRes[1] < 1.0f && SelfTestRes[2] < 1.0f && SelfTestRes[3] < 1.0f && SelfTestRes[4] < 1.0f && SelfTestRes[5] < 1.0f) {
      Serial.println(F("Passed Selftest successful!"));  
   }
   else{
      Serial.println(F("MPU6050 Selftest failed!"));
   }
   
}



void setup() //Initial setup and interrupt setup for free fall detection
{
  pinMode(7,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //pinMode(2, INPUT_PULLUP); // Using pin 2 for interrupt pin
  //attachInterrupt(digitalPinToInterrupt(2), blink, RISING);
  //pinMode(2,INPUT);
  //digitalWrite(2, LOW);
  myservo.attach(9);

  // Initialize the Serial Bus for printing data.
  Serial.begin(9600);

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // Clear the 'sleep' bit to start the sensor.
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);
  
  MPU6050SelfTest(); 
  calibrateMPU9150();
  initMPU9150();

  uint8_t c = readByte(MPU9150_ADDRESS, MPU9150_WHO_AM_I);
  //INIT Compass for direct access
  c = readByte(AK8975A_ADDRESS, WHO_AM_I_AK8975A); 
  if(c==0x48) Serial.println(F("Communication test with AK8975A Magnetometer successful"));
  else{
    Serial.println(F("Communication test with AK8975A Magnetometer FAILED !!!! "));
    while(1) ; // Loop forever if communication doesn't happen
    }
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
  delay(10);
  initAK8975A(magCalibration);  //Initializing routine for AK8975A: Load x,y,z-axis sensitivity values 
  // Output the magnetic bias error which are calculated in a separate procedure - use only from time to time
  //magcalMPU9150(magBias);
  Serial.print(F("X-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[0], 2);
  Serial.print(F("Y-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[1], 2);
  Serial.print(F("Z-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[2], 2);
  


  MPU9150_setupCompass();

  //MPU9150_writeSensor(MPU9150_INT_PIN_CFG, 0x00); //Configuration of interrupt pin, active high, pull up, and active high untill event passed
  //MPU9150_writeSensor(MPU9150_INT_ENABLE, 0x80); // Enabling interrupt for Free Fall Detection
  MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, 0x18); // setting Accel full range +/- 16G instead of default +/-2G, No self-test
  //MPU9150_writeSensor(MPU9150_FF_THR, 0x40); // Free Fall Acceleration Threshold, 32mg/LSB increment, now it is set to pure free fall gravitation
  //MPU9150_writeSensor(MPU9150_FF_DUR, 0x64); // Free Fall Duration, free fall detection in 100ms


}


void loop()
{
  // Print all sensor values which the sensor provides
  // Formated all values as x, y, and z in order for
  // Compass, Gyro, Acceleration. The First value is
  // the temperature.


  //double dT = ( (double) MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H) + 12412.0) / 340.0;
  //Serial.print(dT);
  Serial.print("S ");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H));
  Serial.print(" ");


  // Only transmitting gyro and accelerometer data

  Serial.print(MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H));
  Serial.print(" ");
  Serial.print("");
  Serial.println();
  delay(100);

}

void MPU9150_setupCompass(){
  MPU9150_ADDRESS = 0x0C;      //change Address to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_ADDRESS = 0x69;      //change Address to MPU

  MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
  MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
  MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
  MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
  MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
  MPU9150_writeSensor(0x64, 0x01); //overvride register
  MPU9150_writeSensor(0x67, 0x03); //set delay rate
  MPU9150_writeSensor(0x01, 0x80);

  MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
  MPU9150_writeSensor(0x64, 0x00); //override register
  MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
  MPU9150_writeSensor(0x64, 0x01); //override register
  MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
  MPU9150_writeSensor(0x34, 0x13); //disable slv4
}

////////////////////////////////////////////////////////////
///////// I2C functions to get easier all values ///////////
////////////////////////////////////////////////////////////

int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H<<8)+L);
}

int MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}



/*
 *
 *  long acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_total_vector;
  float angle_pitch, angle_roll;
  boolean set_gyro_angles;
  float angle_roll_acc, angle_pitch_acc;
  float angle_pitch_output, angle_roll_output;


  acc_x = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H);
  acc_y = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H);
  acc_z = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H);

  res = sqrt( sq(acc_x) + sq(acc_y) + sq(acc_z) );

  myservo.write(0);

  //myservo.write(85);

  if(res < thres){
    //Serial.print(4000);
    //Serial.print(" ");
    //digitalWrite(LED_BUILTIN, HIGH);
    //myservo.write(85);

    //tone(7,2000);
    //delay(1000);
    //noTone(7);
  }
  else{
    //Serial.print(1000);
    //Serial.print(" ");
    //digitalWrite(LED_BUILTIN, LOW);
  }


 * //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians


  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians


  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value


  //Serial.print("pitch:  ");
  //Serial.print(angle_pitch_output);
  //Serial.print(", roll:    ");
  //Serial.print(angle_roll_output);

  //Serial.println();


// Angle checks, if pitch/roll is too large, UA is unrecoverable. Also freefall detection
  if(angle_pitch_output > 5.5){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else if(angle_pitch_output < -5.5){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else{
    //noTone(7);
  }
  if(angle_roll_output > 7.0){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else if(angle_roll_output < -7.0){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else{
    //noTone(7);
  }
  if(angle_pitch_output > 2.7 && angle_roll_output < -5.2){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else if(angle_pitch_output < -3.45 && angle_roll_output < 5.45){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else{
    //noTone(7);
  }
  if(res < 800){
    //tone(7,2000);
    //delay(500);
    //noTone(7);
  }else{
    //noTone(7);
  }
 */
