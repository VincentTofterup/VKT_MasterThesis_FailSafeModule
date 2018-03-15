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

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",

#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_X        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
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


// I2C address 0x69 could be 0x68 depends on your wiring. 
int MPU9150_I2C_ADDRESS = 0x69;


//Variables where our values can be stored
int cmps[3];
int accl[3];
int gyro[3];
int temp;
volatile int state = LOW;
int val = 0;



int thres = 700;
int res;
Servo myservo;



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
  long acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_total_vector; 
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

  //double dT = ( (double) MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H) + 12412.0) / 340.0;
  //Serial.print(dT);
  //Serial.print("  ");
  //Serial.print(MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H));
  //Serial.print(" ");
  //Serial.print(MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H));
  //Serial.print(" ");
  //Serial.print(MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H));
  //Serial.print(" ");


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
  Serial.println();



  //Gyro angle calculations
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

  delay(100);

}

void MPU9150_setupCompass(){
  MPU9150_I2C_ADDRESS = 0x0C;      //change Address to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_I2C_ADDRESS = 0x69;      //change Address to MPU

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
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H<<8)+L);
}

int MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}

void blink() {
  state = !state;
}
