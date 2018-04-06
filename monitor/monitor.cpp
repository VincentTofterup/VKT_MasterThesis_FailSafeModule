#include <stdio.h>
#include <fcntl.h>                                                   /* File Control Definitions           */
#include <termios.h>                                                 /* POSIX Terminal Control Definitions */
#include <unistd.h>                                                  /* UNIX Standard Definitions          */
#include <errno.h>                                                   /* ERROR Number Definitions           */
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include <cstring>
#include <ctime>
#include <chrono>
#include <unistd.h>

// Function for correction of yaw angle (replaces the atan2 function for more controll)
float wrap(float x_h, float y_h){
  //TODO: Toleranz einbauen
  float microT_tol = 0.2; //within +-0.2µT the value is set to 270 or 90 deg --> no devision by or near 0 of x_h in arctan function
  float angle = 0;

  if(x_h < 0) angle = M_PI-atan(y_h/x_h);
  if(x_h > 0 && y_h < 0) angle = -atan(y_h/x_h);
  if(x_h > 0 && y_h > 0) angle = 2*M_PI -atan(y_h/x_h);
  //In case of small values for x_h (no devision by 0) set the angle
  if(abs(x_h) < microT_tol && y_h < 0) angle = 0.5*M_PI;
  if(abs(x_h) < microT_tol && y_h > 0) angle = 1.5*M_PI;
  return angle;
}


int main(){
  int fd;

  fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);                       /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   					                                                   /* O_RDWR   - Read/Write access to serial port       */
								                                                     /* O_NOCTTY - No terminal will control the process   */
								                                                     /* Open in blocking mode,read will wait              */

  if(fd == 1){
    printf("\nError in opening ttyUSB0! \n");
  }

  /*------------------------ Setting the Attributes of the serial port using termios structure ----------------------- */

	struct termios SerialPortSettings;	                               /* Create the structure                          */
	tcgetattr(fd, &SerialPortSettings);	                               /* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings,B9600);                            /* Set Read  Speed as 9600                       */
	cfsetospeed(&SerialPortSettings,B9600);                            /* Set Write Speed as 9600                       */

  /* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;                             /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;                             /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	                             /* Clears the mask for setting the data size             */
  SerialPortSettings.c_cflag |=  CS8;                                /* Set the data bits = 8                                 */
	SerialPortSettings.c_cflag &= ~CRTSCTS;                            /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL;                      /* Enable receiver,Ignore Modem Control lines       */
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);             /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);     /* Non Cannonical mode                            */
	SerialPortSettings.c_oflag &= ~OPOST;                              /* No Output Processing*/

	/* Setting timeouts */
	SerialPortSettings.c_cc[VMIN] = 10;                                /* Read at least 10 characters */
	SerialPortSettings.c_cc[VTIME] = 0;                                /* Wait indefinetly   */

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0){              /* Set the attributes to the termios structure*/
	  printf("\n  ERROR ! in Setting attributes");
	}



  bool set_gyro_angles = false;

  float angle_roll_output = 0.0;
  float angle_pitch_output = 0.0;

  float a_pitch = 0.0;
  float a_roll = 0.0;
  float g_pitch = 0.0;
  float g_roll = 0.0;
  float g_yaw = 0.0;

  //Definition of variables for time and integral time calculation
  uint32_t now, lastupdate;
  double dT; //need to be precise due to short time intervalls
  float a_coeff = 0.95; // Coefficient for sensor fusion (how much weight put on the angle integration --> short term trust to gyro)
  float a_CompFilter = 0.9; // Coefficient for low pass filtering the raw comp data --> e.g. 0.9 is buidling the floating average of the last 10 values(?)

  float pitchAcc, rollAcc, yawMag;  //Define variables for the "raw" angles calculated directly from the Acc values
  //Define variables for the angles incl. start values. Those angles are in degrees
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
  //roll and pitch are needed in radians for compass tilt compensation
  float pitchRad, rollRad;
  float RadToDeg = (180.0/M_PI);

  std::cout << "Entering While Loop" << std::endl;

  while(true){
  tcflush(fd, TCIFLUSH);                                            /* Discards old data in the rx buffer            */

	char read_buffer[256];                                            /* Buffer to store the data received              */
	int  bytes_read = 0;                                              /* Number of bytes read by the read() system call */
	int i = 0;
  bool end_line = false;
  bool start_line = false;
  int count = 0;
  std::vector <double> raw_data;

  double gx,gy,gz,ax,ay,az,cx,cy,cz, cx_filter, cy_filter, cz_filter, cx_h, cy_h;


	bytes_read = read(fd,&read_buffer,256);                           /* Read the data                   */
  //printf("\nBytes Rxed -%d", bytes_read);                           /* Print the number of bytes read */

  if(bytes_read > 1){

    std::string tmp;
      for(i=0;i<bytes_read;i++){	                                    /* Write only the received characters*/

        if(read_buffer[i] == 'S'){
          end_line = false;
          start_line = true;
        }

        if(start_line){
          tmp+=read_buffer[i];
          //std::cout << "debug1" << std::endl;

          if((read_buffer[i] == ' ' )){//| read_buffer[i] == 'S')){
            //std::cout << "debug2, count = " << count << std::endl;
            count ++;

            if(count == 10){ // end of line check, 7 is because of serial data format
              end_line = true;
              start_line = false;
              count = 0;
            }
            try{
              raw_data.push_back(std::stoi(tmp));
            }
            catch (const std::invalid_argument& ia) {
              //std::cout << "Exception thrown: " << ia.what() << std::endl;
            }
            tmp.erase();
          }
        }
        //std::cout << "debug3" << std::endl;

        if(end_line){
          count = 0;

          cx = raw_data[0];
          cy = raw_data[1];
          cz = raw_data[2];
          gx = raw_data[3];
          gy = raw_data[4];
          gz = raw_data[5];
          ax = raw_data[6];
          ay = raw_data[7];
          az = raw_data[8];
          //std::cout << "debug5, size = " << raw_data.size() << std::endl;

          // Possible accelerometer scales (and their register bit settings) are:
          // 2 G (00), 4 G (01), 8 G (10), and 16 G  (11).
          #define aRange 2.0       // 2G acceleration resolution (max. sensitivity)
          #define Ascale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01
          // Possible gyro scales (and their register bit settings) are
          // 250 °/s (00), 500 °/s (01), 1000 °/s (10), and 2000 °/s  (11).
          #define gRange 250.0     // 250°/s gyro resolution (max. sensitivity)
          #define Gscale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01

          float accRes = aRange/32768.0;  // 16 bit, 2G --> 0,000061035 g = per LSB
          float gyroRes = gRange/32768.0; // 16 bit, 250 deg/sec --> = per LSB
          float compRes = 1229.0/4096.0;  // = 0,3µT per LSB --> 1 Milligauss [mG] =   0,1 Mikrotesla [µT]





          // Low-Pass Filter the magnetic raw data in x, y, z since a filter later is not so easy to implement
          cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;
          cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;
          cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;

          // Converting from raw data to the right units, from datasheet
          cx = (cx * (1229.0/4096.0))  * 1.0;
          cy = (cy * (1229.0/4096.0)) * 1.0;
          cz = cz * (1229.0/4096.0);
          gx = gx * (gRange/32768.0); //gx/131.0; // degrees pr sec.
          gy = gy * (gRange/32768.0); // gy/131.0;
          gz = (gz * (gRange/32768.0)) * 1.0; // gz/131.0;
          ax = ax * (aRange/32768.0);//(ax/2048) * 9.82; // meters pr sec, with the +/-16G resolution
          ay = ay * (aRange/32768.0);//(ay/2048) * 9.82;
          az = az * (aRange/32768.0); //(az/2048) * 9.82;



          /*** Angle Calculation and Sensor Fusion ***/
          //Angle calculation from accelerometer. x-Axis pointing to front (ATTENTION: IS THE PRINTED Y-AXIS ON THE SENSOR!)
          pitchAcc = atan2(ax, az); // pitch is angle between x-axis and z-axis
          pitchAcc = -pitchAcc * RadToDeg;      // pitch is positiv upwards (climb) --> Invert the angle!

          rollAcc = atan2(ay, az); // roll is angle between y-axis and z-axis
          rollAcc = rollAcc * RadToDeg;         // roll is clockwise positiv to the right (rotation clockwise in flight direction)

          // Calculation of the heading

          // With tilt compensation
          // Takes raw magnetometer values, pitch and roll and turns it into a tilt-compensated heading value ranging from -pi to pi (everything in this function should be in radians).
          // Basically we first "unturn" the roll-angle and then the pitch to have mx and my in the horizontal plane again
          pitchRad = -pitch/RadToDeg;  //angles have to be inverted again for unturing from actual plane to horizontal plane
          rollRad = -roll/RadToDeg;
          cx_h = cx*cos(pitchRad) + cy*sin(rollRad)*sin(pitchRad) - cz*cos(rollRad)*sin(pitchRad);
          cy_h = cy*cos(rollRad) + cz*sin(rollRad);
          yawMag = wrap(cx_h, cy_h);
          //yawMag = wrap(mx,my);  //without tilt compensation
          //Serial.print(mx);Serial.print("\t");Serial.print(my);Serial.print("\t");Serial.print(cx_h);Serial.print("\t");Serial.println(cy_h);Serial.println();

          yawMag = yawMag * RadToDeg;
          //yawMag = yawMag + declinationAngle; // Subtracking the 'Declination Angle' in Deg --> a positive (to east) declination angle has to be subtracked
          //float inclinationAngle = atan(mz/sqrt(mx*mx+my*my))* RadToDeg;
          //Serial.print("Inclination Angle = ");Serial.println(inclinationAngle);


          //Sensor fusion
          now = std::time(nullptr);
          dT = (now - lastupdate)/1000000.0;
          lastupdate = std::time(nullptr);

          pitch = a_coeff * (pitch + gy * dT) + (1-a_coeff) * pitchAcc;  // pitch is the rotation around the y-axis
          roll  = a_coeff * (roll  + gx * dT) + (1-a_coeff) * rollAcc;  // roll is the rotation around the x-axis
          // yaw is the rotation around the z-axis. ATTENTION: Simple filtering as for pitch and roll does not work here properly due to the change betwenn 0° - 360° and vice versa
          //       e.g "yaw = a_coeff * yaw + (1-a_coeff) * yawMag;" does not work
          yaw = yawMag;
          if(yaw>360) yaw = yaw-360; //care for the change from 0-->360 or 360-->0 near to north
          if(yaw<0)yaw = yaw+360;



          /*double pitch_offset = 0.0;
          double roll_offset = 0.0;

          //  calculate pitch (x-axis) and roll (y-axis) angles
          a_pitch = atan2f(ay, sqrt(pow(ax, 2) + pow(az, 2)) );
          a_roll = atan2f(-ax, az);

          float dt = 0.01;
          g_pitch += gx * dt;
          g_roll -= gy * dt;
          g_yaw += gz * dt;


          if(set_gyro_angles){                                                 //If the IMU is already started
            g_pitch = (g_pitch * 0.98) + (a_pitch * 0.02) ;//- pitch_offset;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
            g_roll = (g_roll * 0.98) + (a_roll * 0.02) ;//- roll_offset;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
          }
          else{                                                                //At first start
            g_pitch = a_pitch ;//- pitch_offset;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
            g_roll = a_roll ;//- roll_offset;                                       //Set the gyro roll angle equal to the accelerometer roll angle
            set_gyro_angles = true;                                            //Set the IMU started flag
          }


          //To dampen the pitch and roll angles a complementary filter is used
          angle_pitch_output = (angle_pitch_output * 0.9) + (g_pitch * 0.1);    // Take 90% of the output pitch value and add 10% of the raw pitch value
          angle_roll_output = (angle_roll_output * 0.9) + (g_roll * 0.1);      // Take 90% of the output roll value and add 10% of the raw roll value
*/
          std::cout << "roll: " << roll << std::endl;
          std::cout << "pitch: " << pitch << std::endl;
          std::cout << "yaw: " << yaw << std::endl;
          std::cout << std::endl;


          /*std::cout << "cx: " << cx_filter;
          std::cout << ", cy: "<< cy_filter;
          std::cout << ", cz: "<< cz_filter;
          std::cout << ", gx: "<< gx;
          std::cout << ", gy: "<< gy;
          std::cout << ", gz: "<< gz;
          std::cout << ", ax: "<< ax;
          std::cout << ", ay: "<< ay;
          std::cout << ", az: "<< az << std::endl;
          std::cout << std::endl;*/


          // Here to call the rpy method to recieve rpy
          // next to call xyz method
          // With rpy and xyz, call model based fail detection algorithm with rpy as arguments.
        }
      }
    }
  }
  close(fd);                                                       /* Close the serial port */
  return 0;
}
