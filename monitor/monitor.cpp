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
#include <sys/time.h> // gettimeofday system call
#include <serial.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdio.h>
#include "definitions.h" // definition of variables used in this code
#include <wiringPi.h> // GPIO control
#include "mavlink_lora_lib.h" // Kjelds mavlink library
#include "LatLong-UTMconversion.h" // partially user implemented lat lon conversion class
#include "polygon.h" // polygon definition and used implemented code
#include <chrono> // sleep this thread
#include <thread> // sleep this thread
#include <fstream>

static sigset_t wait_mask;


// Function for correction of yaw angle (replaces the atan2 function for more controll)
float wrap(float x_h, float y_h){
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

/***************************************************************************/
void pos_parse_msg(unsigned char *msg, unsigned long now){
	switch (msg[ML_POS_MSG_ID]){
    case 0:
      heartbeat = true;
    case 1:
      {
        mavlink_sys_status_t sys_status = ml_unpack_msg_sys_status (msg + ML_POS_PAYLOAD);
        battery = sys_status.voltage_battery;

      }
      break;

		case 24:
			{
				mavlink_gps_raw_int_t gri = ml_unpack_msg_gps_raw_int (msg + ML_POS_PAYLOAD);
			  sprintf (s, "%.3f,%.7f,%.7f,%.3f\n", (double) gri.time_usec/1000000, (double) gri.lat/10000000, (double) gri.lon/10000000, (double) gri.alt/1000);
        pos_raw[0] =  (double) gri.lat/10000000;
        pos_raw[1] =  (double) gri.lon/10000000;
        pos_raw[2] =  (double) gri.alt/1000;

				//printf ("GPS_RAW_INT ");
				//printf ("%s", s);
				//fprintf (f, "GPS_RAW_INT,%s", s);
			}
			break;

		case 33:
			{
				mavlink_global_position_int_t gpi = ml_unpack_msg_global_position_int (msg + ML_POS_PAYLOAD);
				//sprintf (s, "%.3f,%.7f,%.7f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f\n", (double) gpi.time_boot_ms/1000, (double) gpi.lat/10000000, (double) gpi.lon/10000000, (double) gpi.alt/1000, (double) gpi.relative_alt/1000, (double) gpi.vx/100, (double) gpi.vy/100, (double) gpi.vz/100, (double) gpi.hdg/100);
        pos_glo[0] =  (double) gpi.lat/10000000;
        pos_glo[1] =  (double) gpi.lon/10000000;
        pos_glo[2] =  (double) gpi.alt/1000;
				//printf ("GLOBAL_POSITION_INT ");
				//printf ("%s", s);
				//fprintf (f, "GLOBAL_POSITION_INT,%s", s);
			}
			break;
	}
}

/***************************************************************************/
void ml_parse_msg(unsigned char *msg){
  pos_parse_msg(msg, millis_ml());
}
/***************************************************************************/
// WiringPi definitions
#define CUTOFF 5  // pin 24
#define PARARACHUTE 1 // pin 25
#define CLOSED 40  // needs to be calibrated
#define OPEN 70  // needs to be calibrated


int main(){
  std::cout << "Entering main()" << std::endl;
  wiringPiSetup () ;
  pinMode (CUTOFF, OUTPUT) ;
  digitalWrite (CUTOFF, LOW) ;	// Cutoff Motor system!
  pinMode (PARARACHUTE, PWM_OUTPUT);
  pwmSetMode (PWM_MODE_MS);
  pwmSetClock(384); //clock at 50kHz (20us tick)
  pwmSetRange(1000); //range at 1000 ticks (20ms)
  pwmWrite(PARARACHUTE,CLOSED);

  // Thau Observer
  // Initial state estimate is 0.0 in each state
  for (int i = 0; i < 12; i++) {
    Pxdot[i] = 0.0;
    old_Pxdot[i] = 0.0;
    Px[i] = 0.0;
    old_Px[i] = 0.0;

  }
  std::ofstream log_file("log_file.txt", std::ios_base::out | std::ios_base::app );


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

  int firstrun = 1;

  // Mavlink init:

  char ser_dev[50];
  int ser_err;
  char param_id[17];
  unsigned char ser_dev_set;

  strcpy (ser_dev, CFG_SER_DEV); /* default serial device */
  /* try to open the serial device */
  ser_err = ser_open (&ser, &oldtio, ser_dev, CFG_SER_BAUD);
  /* if everything ok */
  if (! ser_err){
    //printf("Opend serial device!\n");
    ml_init();
    pos_init();
    ml_set_monitor_all();
  }else{
    printf ("Unable to open serial device\n");
  }

  bool breach = false;
  int activation = 0;
  int freefall = 0;
  int n = 0;

  auto start = std::chrono::system_clock::now();
  std::time_t start_time = std::chrono::system_clock::to_time_t(start);

  log_file << std::endl;
  log_file << "BEGINNING OF LOG, LOGGING TIME: " << std::ctime(&start_time) << std::endl;
  log_file << std::endl;


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


    bytes_read = read(fd,&read_buffer,256);                             /* Read the data                   */
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

            if((read_buffer[i] == ' ' )){
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
          if(end_line){
            //auto start = std::chrono::system_clock::now();
            count = 0;
            n++;

            cx = raw_data[0];
            cy = raw_data[1];
            cz = raw_data[2];
            gx = raw_data[3];
            gy = raw_data[4];
            gz = raw_data[5];
            ax = raw_data[6];
            ay = raw_data[7];
            az = raw_data[8];

            // Low-Pass Filter the magnetic raw data in x, y, z since a filter later is not so easy to implement
            cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;
            cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;
            cx_filter = a_CompFilter * cx_filter + (1-a_CompFilter) * cx;


            // Converting from raw data to the right units, from datasheet
            cx = ((cx_filter * (1229.0/4096.0)) * magCalib[0]) - magBias[0];
            cy = ((cy_filter * (1229.0/4096.0)) * magCalib[1]) - magBias[1];
            cz = ((cz_filter * (1229.0/4096.0)) * magCalib[2])- magBias[2];
            gx = gx * (gRange/32767.0); //gx/131.0; // degrees pr sec.
            gy = gy * (gRange/32767.0); // gy/131.0;
            gz = (gz * (gRange/32767.0)) * -1.0; // gz/131.0;
            ax = ax * (aRange/32767.0)  * -1.0;//(ax/2048) * 9.82; // meters pr sec, with the +/-16G resolution
            ay = ay * (aRange/32767.0) * -1.0 ;//(ay/2048) * 9.82;
            az = az * (aRange/32767.0); //(az/2048) * 9.82;



            //auto end = std::chrono::system_clock::now();
            //std::chrono::duration<double> elapsed_seconds = end-start;
            //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            //std::cout << "Elapsed time in this loop nr:" << n << ", time: " << elapsed_seconds.count() << std::endl;



            /*** Angle Calculation and Sensor Fusion ***/
            //Angle calculation from accelerometer. x-Axis pointing to front (ATTENTION: IS THE PRINTED Y-AXIS ON THE SENSOR!)
            pitchAcc = atan2(ay, az);             // pitch is angle between x-axis and z-axis
            pitchAcc = -pitchAcc * RadToDeg;      // pitch is positiv upwards (climb) --> Invert the angle!

            rollAcc = atan2(ax, az);              // roll is angle between y-axis and z-axis
            rollAcc = rollAcc * RadToDeg;         // roll is clockwise positiv to the right (rotation clockwise in flight direction)

            // Calculation of the heading

            // With tilt compensation
            // Takes raw magnetometer values, pitch and roll and turns it into a tilt-compensated heading value ranging from -pi to pi (everything in this function should be in radians).
            // Basically we first "unturn" the roll-angle and then the pitch to have mx and my in the horizontal plane again
            pitchRad = -pitch/RadToDeg;            //angles have to be inverted again for unturing from actual plane to horizontal plane
            rollRad = -roll/RadToDeg;
            cx_h = cx*cos(pitchRad) + cy*sin(rollRad)*sin(pitchRad) - cz*cos(rollRad)*sin(pitchRad);
            cy_h = cy*cos(rollRad) + cz*sin(rollRad);
            yawMag = wrap(cx_h, cy_h);
            //yawMag = wrap(cx,cy);  //without tilt compensation

            //yawMag = yawMag * RadToDeg;
            yawMag = (yawMag - declinationAngle); //* RadToDeg; // Subtracking the 'Declination Angle' in Deg --> a positive (to east) declination angle has to be subtracked

            //Sensor fusion
            now = std::time(nullptr);
            dT = (now - lastupdate)/10000000.0;
            lastupdate = std::time(nullptr);

            pitch = (a_coeff * (pitch + gy * dT) + (1-a_coeff) * pitchAcc); //- pitch_offset;  // pitch is the rotation around the y-axis
            roll  = (a_coeff * (roll  + gx * dT) + (1-a_coeff) * rollAcc);  //- roll_offset;  // roll is the rotation around the x-axis
            // yaw is the rotation around the z-axis. ATTENTION: Simple filtering as for pitch and roll does not work here properly due to the change betwenn 0° - 360° and vice versa
            // e.g "yaw = a_coeff * yaw + (1-a_coeff) * yawMag;" does not work
            yaw = yawMag;

            //yaw = atan(cy/cx) * RadToDeg;
            if(yaw>360)
              yaw = yaw-360; //care for the change from 0-->360 or 360-->0 near to north
            if(yaw<0)
              yaw = yaw+360;

            float degTorad = (2 * M_PI) / 360;

            // Thau Observer
            //start = std::time(nullptr);
            //float dt = (start - end)/10000.0;
            //end = std::time(nullptr);
            float dt = 0.10;


            /*if (firstrun == 1) { // first run
              state[0] = (roll-roll_offset) * degTorad;
              state[1] = (pitch-pitch_offset) * degTorad;
              state[2] = yaw * degTorad;
              state[3] = 0;
              state[4] = 0;
              state[5] = 0;
              state[6] = 0;
              state[7] = 0;
              state[8] = 0;
              state[9] = pos[0];
              state[10] = pos[1];
              state[11] = pos[2];
            }
            if(firstrun == 2){ // actual state, that has correct velocities calculated, with dt = 0.0005
              state[0] = (roll-roll_offset) * degTorad;
              state[1] = (pitch-pitch_offset) * degTorad;
              state[2] = yaw * degTorad;
              state[3] = (state[0]-old_state[0])/dt;
              state[4] = (state[1]-old_state[1])/dt;
              state[5] = (state[2]-old_state[2])/dt;
              state[6] = (pos[0]-old_pos[0])/dt;
              state[7] = (pos[1]-old_pos[1])/dt;
              state[8] = (pos[2]-old_pos[2])/dt;
              state[9] = pos[0];
              state[10] = pos[1];
              state[11] = pos[2];
            }*/
            // State Estimate change, from matlab:
            /*Pxdot[0] = (Px[3] + Px[5] * Px[1] + Px[4] * Px[0] * Px[1]); // Px(4)+Px(6)*Px(2)+Px(5)*Px(1)*Px(2)
            Pxdot[1] = (Px[4] - Px[5] * Px[0]); // Px(5)-Px(6)*Px(1)
            Pxdot[2] = (Px[5] + Px[4] * Px[0]); // Px(6)+Px(5)*Px(1)
            Pxdot[3] = (((Iy-Iz)/Ix) * Px[5] * Px[4] + (taux/Ix));  // ((Iy-Iz)/Ix)*Px(6)*Px(5)+(tauxP/Ix)
            Pxdot[4] = (((Iz-Ix)/Iy) * Px[3] * Px[5] + (tauy/Iy));  // ((Iz-Ix)/Iy)*Px(4)*Px(6)+(tauyP/Iy)
            Pxdot[5] = (((Ix-Iy)/Iz) * Px[3] * Px[4] + (tauz/Iz));  // ((Ix-Iy)/Iz)*Px(4)*Px(5)+(tauzP/Iz)
            Pxdot[6] = (Px[5] * Px[7] - Px[4] * Px[10] - (g * Px[1]));  // Px(6)*Px(8)-Px(5)*Px(11)-g*Px(2)
            Pxdot[7] = (Px[3] * Px[8] - Px[5] * Px[6] + (g * Px[0])); // Px(4)*Px(9)-Px(6)*Px(7)+g*Px(1)
            Pxdot[8] = (Px[4] * Px[6] - Px[3] * Px[7] + (g * (ft/m)));  // Px(5)*Px(7)-Px(4)*Px(8)+g*(ftP/m)
            Pxdot[9] = (Px[8] * (Px[0] * Px[2] + Px[1]) - Px[7] * (Px[2] - Px[0] * Px[1]) + Px[6]); // Px(9)*(Px(1)*Px(3)+Px(2))-Px(8)*(Px(3)-Px(1)*Px(2))+Px(7)
            Pxdot[10] = (Px[7] * (1 + Px[0] * Px[1] * Px[2]) - Px[8] * (Px[0] - Px[1] * Px[2]) + Px[6] * Px[2]);  // Px(8)*(1+Px(1)*Px(2)*Px(3))-Px(9)*(Px(1)-Px(2)*Px(3))+Px(7)*Px(3)
            Pxdot[11] = (Px[8] - Px[6] * Px[1] + Px[7] * Px[0]); // Px(9)-Px(7)*Px(1)+Px(8)*Px(1)

            */
            // Matlab functions for the following:
            //PThau = lyap((A+0.5*eye(12))',-C'*C);
            //PThau = inv(PThau)*C';
            // Just copy pasted from matlab solution and writting into the following equations:
            // State estimate update, first Pxdot, then Px.
            // Pxdot = Pxdot + PThau*(C*x-C*Px);
            // Px    = Px + Pxdot * dt;

            /*Px[0] = Px[0] + ((Pxdot[0] + (1.5 * (state[0] - Px[0]) + 0.5 * (state[3] - Px[3]))) * dt);
            Px[1] = Px[1] + ((Pxdot[1] + (1.5 * (state[1] - Px[1]) + 0.5 * (state[4] - Px[4]))) * dt);
            Px[2] = Px[2] + ((Pxdot[2] + (1.5 * (state[2] - Px[2]) + 0.5 * (state[5] - Px[5]))) * dt);
            Px[3] = Px[3] + ((Pxdot[3] + (0.5 * (state[0] - Px[0]) + 0.5 * (state[3] - Px[3]))) * dt);
            Px[4] = Px[4] + ((Pxdot[4] + (0.5 * (state[1] - Px[1]) + 0.5 * (state[4] - Px[4]))) * dt);
            Px[5] = Px[5] + ((Pxdot[5] + (0.5 * (state[2] - Px[2]) + 0.5 * (state[5] - Px[5]))) * dt);
            Px[6] = Px[6] + ((Pxdot[6] + (state[9] - Px[9])) * dt);
            Px[7] = Px[7] + ((Pxdot[7] + (state[10] - Px[10])) * dt);
            Px[8] = Px[8] + ((Pxdot[8] + (state[11] - Px[11])) * dt);
            Px[9] = Px[9] + ((Pxdot[9] + 2.0 * (state[9] - Px[9])) * dt);
            Px[10] = Px[10] + ((Pxdot[10] + 2.0 * (state[10] - Px[10])) * dt);
            Px[11] = Px[11] + ((Pxdot[11] + 2.0 * (state[11] - Px[11])) * dt);

            for (int i = 0; i < 12; i++) {
              if (Px[i] != Px[i] ) { // check for nan
                  Pxdot[i] = old_Pxdot[i];
                  Px[i] = old_Px[i];
              }
            }*/

            // mavlink gps extraction
            char result;
          	unsigned long now = millis_ml();
          	serbuf_cnt = SER_BUF_SIZE;
          	serbuf_cnt = ser_receive (ser, serbuf, serbuf_cnt);

          	/* if we received new data */
          	if (serbuf_cnt > 0){
          		result = ml_rx_update(now, serbuf, serbuf_cnt);
          	}


            //std::cout << "Battery voltage: " << battery << std::endl;
            //std::cout << "pos_raw[lat,lon,alt]: " << pos_raw[0] << ", " << pos_raw[1] << ", " << pos_raw[2] << std::endl;

            double lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4;

            // Geofence of odense modelflyve plads
            lat1 = 55.47215865774781;
            lon1 = 10.414617955684662;

            lat2 = 55.47217082009681;
            lon2 = 10.41502296924591;

            lat3 = 55.471783143376314;
            lon3 = 10.415057837963104;

            lat4 = 55.47176641998102;
            lon4 = 10.414628684520721;

            double e1, n1, e2, n2, e3, n3, e4, n4;

            e1 = 589422;
            n1 = 6148244;
            e2 = 589447;
            n2 = 6148246;
            e3 = 589450;
            n3 = 6148203;
            e4 = 589423;
            n4 = 6148200;


            double northing, easting;
            int northing_int, easting_int;
            int zone = 32;
            int ellip = 22; // WGS84

            /*LLtoUTM(ellip, lat1, lon1, n1, e1, zone);
            LLtoUTM(ellip, lat2, lon2, n2, e2, zone);
            LLtoUTM(ellip, lat3, lon3, n3, e3, zone);
            LLtoUTM(ellip, lat4, lon4, n4, e4, zone);

            std::cout << std::setprecision(16) << e1 << ", " << n1 << std::endl;
            std::cout << std::setprecision(16) << e2 << ", " << n2 << std::endl;
            std::cout << std::setprecision(16) << e3 << ", " << n3 << std::endl;
            std::cout << std::setprecision(16) << e4 << ", " << n4 << std::endl;*/

            LLtoUTM(ellip, pos_raw[0], pos_raw[1], northing, easting, zone);

            northing_int = northing;
            easting_int = easting;

            std::cout << "lat: " << pos_raw[0] << "lon:" << pos_raw[1] <<std::endl;

            //std::cout << "UTM(northing,easting, ellipsoid: WGS84, zone:32): " << northing << ", " << easting << std::endl;

            Point tmp = {northing, easting};
            //Point poly1[] = {{northing-10.0,easting-10.0}, {northing+10.0, easting-10.0}, {northing,easting+10.0}};
            Point poly1[] = {{n1,e1}, {n2,e2}, {n3,e3}, {n4,e4}};// New polygon

            if (!isInside(poly1, 4, tmp)) {
              breach = true;
              digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
              std::this_thread::sleep_for (std::chrono::milliseconds(1400)); // scales to almost 10 meters freefall
              pwmWrite(PARARACHUTE,OPEN);
              activation = 1;
            }else{
              breach = false;
              activation = 0;
            }

            double mag = sqrt(pow(ax,2.0) + pow(ay,2.0) + pow(az,2.0));
            /*if(!breach){
              double mag = sqrt(pow(ax,2) + pow(ay,2) + pow(az,2)) / 3;
              if(mag < 1.5)
                freefall ++;
              if(freefall > 14 && pos_raw[2] > 10.0) { // scales to almost 10 meter freefall, and freefall can only occur once we're 10 meterabove ground
                digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
                //std::this_thread::sleep_for (std::chrono::milliseconds(100)); // just a little delay after motor stop
                pwmWrite(PARARACHUTE,OPEN);
                activation = 1;
              }else{
                activation = 0;
              }
            }*/
            //if (isInside(poly1, 3, tmp)) {
              //std::cout << "Current position inside defined polygon! (supposed to be inside at all times) " << std::endl;
            //}

            //Point poly2[] = {{northing-10.0,easting-10.0}, {northing+10.0, easting-10.0}, {northing,easting-10.0}};
            //if (! isInside(poly2, 3, tmp)) {
              //std::cout << "Current position outside defined polygon! (supposed to be outside at all times) " << std::endl;

            //}

            //if (roll-roll_offset > 45.0) {
            //  std::cout << "roll exceed(lower) limit!" << std::endl;
            //  digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
            //  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
            //  pwmWrite(PARARACHUTE,OPEN);
            //}else if (roll-roll_offset < -45.0) {
            //  std::cout << "roll exceed(larger) limit!" << std::endl;
            //  digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
            //  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
            //  pwmWrite(PARARACHUTE,OPEN);
            //}

            //if (pitch-pitch_offset > 45.0) {
            //  std::cout << "pitch exceed(large) limit!" << std::endl;
            //  digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
            //  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
            //  pwmWrite(PARARACHUTE,OPEN);
            //}else if (pitch-pitch_offset < -45.0) {
            //  std::cout << "pitch exceed(lower) limit!" << std::endl;
            //  digitalWrite (CUTOFF, HIGH) ;	// Cutoff Motor system!
            //  std::this_thread::sleep_for (std::chrono::milliseconds(1000));
            //  pwmWrite(PARARACHUTE,OPEN);
            //}


            log_file << std::setprecision(8)<< n << ", "<< mag << ", " << ax << ", " << ay << ", " << az << ", " << easting_int << ", " << northing_int << ", " << pos_raw[2] << ", " << activation << std::endl;

            //old_pos[0] = pos[0]; old_pos[1] = pos[1]; old_pos[2] = pos[2]; // old_pos update
            //memmove( old_state, state, sizeof(state) ); // old state update
            //memmove( old_Px, Px, sizeof(Px) );  // old_Px update
            //memmove( old_Pxdot, Pxdot, sizeof(Pxdot) ); // old_Pxdot update

            firstrun = 2; // first time run variable, now velocities can be set correctly
            // reset heartbeat bool for check
            heartbeat = false;


          }
        }
      }
    }
    close(fd);                                                       /* Close the serial port */
    return 0;
}
