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

void removeCharsFromString( std::string &str, char* charsToRemove ) {
   for ( unsigned int i = 0; i < std::strlen(charsToRemove); ++i ) {
      str.erase( std::remove(str.begin(), str.end(), charsToRemove[i]), str.end() );
   }
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



  double gx,gy,gz,ax,ay,az;
  long gx_conv, gy_conv, gz_conv, ax_conv, ay_conv, az_conv;
  float gx_conv_old, gy_conv_old, gz_conv_old, ax_conv_old, ay_conv_old, az_conv_old;

  long acc_total_vector;
  float angle_pitch, angle_roll, angle_roll_acc, angle_pitch_acc, angle_roll_output, angle_pitch_output;
  bool set_gyro_angles = false;

  angle_roll_output = 0.0;
  angle_pitch_output =0.0;

  std::cout << "Entering While Loop" << std::endl;

  while(true){
  tcflush(fd, TCIFLUSH);                                            /* Discards old data in the rx buffer            */

	char read_buffer[256];                                            /* Buffer to store the data received              */
	int  bytes_read = 0;                                              /* Number of bytes read by the read() system call */
	int i = 0;


  std::vector <int> raw_data;

	bytes_read = read(fd,&read_buffer,256);                           /* Read the data                   */

  //printf("\nBytes Rxed -%d", bytes_read);                           /* Print the number of bytes read */

  if(bytes_read > 1){
      std::string tmp;
      // line begin

        //std::cout << "Detected beginning of line" << std::endl;
        for(i=0;i<bytes_read;i++){	                                    /* Write only the received characters*/


          tmp+=read_buffer[i];

          if(read_buffer[i] == ' '){
            //std::cout << "Detected end of number" << std::endl;
            std::string::size_type sz;
            removeCharsFromString( tmp, " SE");
            if(read_buffer[i-1] != 'S' | read_buffer[i-1] != 'E'){
              try{
                raw_data.push_back(std::stoi(tmp));
              }
              catch (const std::invalid_argument& ia) {
                //raw_data.push_back(0);
                std::cout << "Exception thrown: " << ia.what() << std::endl;
              }
              tmp.erase();
            }
          }

        //line end and time for save.
        if(read_buffer[i] == 'E'){
          //std::cout << "Detected end of line" << std::endl;
          gx = raw_data[0];
          gy = raw_data[1];
          gz = raw_data[2];
          ax = raw_data[3];
          ay = raw_data[4];
          az = raw_data[5];


          /*gx_conv = gx/131.0;
          gy_conv = gy/131.0;
          gz_conv = gz/131.0;
          ax_conv = (ax/2048) * 9.82;
          ay_conv = (ay/2048) * 9.82;
          az_conv = (az/2048) * 9.82;*/

          std::cout << "gx: "<< gx;
          std::cout << ", gy: "<< gy;
          std::cout << ", gz: "<< gz;
          std::cout << ", ax: "<< ax;
          std::cout << ", ay: "<< ay;
          std::cout << ", az: "<< az << std::endl;
          /*
          //Gyro angle calculations
          // 1 / (8000Hz / 131)
          angle_pitch += gx * (1 / (8000 / 131)); //Calculate the traveled pitch angle and add this to the angle_pitch variable
          angle_roll += gy * (1 / (8000 / 131));  //Calculate the traveled roll angle and add this to the angle_roll variable

          //1 / (8000Hz / 131) * (3.142(PI) / 180degr)
          angle_pitch += angle_roll * sin(gz * ((1 / (8000 / 131)) * (3.142 / 180))); //If the IMU has yawed transfer the roll angle to the pitch angle
          angle_roll -= angle_pitch * sin(gz * ((1 / (8000 / 131)) * (3.142 / 180))); //If the IMU has yawed transfer the pitch angle to the roll angle


          //Accelerometer angle calculations
          acc_total_vector = sqrt((ax*ax)+(ay*ay)+(az*az));  //Calculate the total accelerometer vector

          //57.296 = 1 / (3.142(PI) / 180)
          angle_pitch_acc = asin((float)ay / acc_total_vector)* (1 / (3.142 / 180));       //Calculate the pitch angle
          angle_roll_acc = asin((float)ax / acc_total_vector)* -(1 / (3.142 / 180));       //Calculate the roll angle

          //Initially, the UAV is at rest, standing still on the ground.
          angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
          angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

          if(set_gyro_angles){                                                 //If the IMU is already started
            angle_pitch = (angle_pitch * 0.9996) + (angle_pitch_acc * 0.0004);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
            angle_roll = (angle_roll * 0.9996) + (angle_roll_acc * 0.0004);        //Correct the drift of the gyro roll angle with the accelerometer roll angle
          }
          else{                                                                //At first start
            angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
            angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
            set_gyro_angles = true;                                            //Set the IMU started flag
          }

          //To dampen the pitch and roll angles a complementary filter is used
          angle_pitch_output = (angle_pitch_output * 0.9) + (angle_pitch * 0.1);   //Take 90% of the output pitch value and add 10% of the raw pitch value
          angle_roll_output = (angle_roll_output * 0.9) + (angle_roll * 0.1);      //Take 90% of the output roll value and add 10% of the raw roll value

          std::cout << "pitch: " << angle_pitch_output  << std::endl;
          std::cout << "roll: " << angle_roll_output << std::endl;*/
        }




      }
      }
    }


  close(fd);                                                       /* Close the serial port */
  return 0;
}
