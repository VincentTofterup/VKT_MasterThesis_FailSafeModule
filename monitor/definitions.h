#include <iostream>
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
#include <wiringPi.h> // GPIO control
#include "mavlink_lora_lib.h"

using namespace std;


//----------------------------------------------------------------------------//
//--------------------- Definitions, not cleaned up --------------------------//
//----------------------------------------------------------------------------//



#define true					1
#define false					0

#define SCHED_INTERVAL  		2e4 /**< schedule interval set to 50 Hz */

#define APP_INIT_OK				0
#define APP_INIT_ERR			1

#define CFG_SER_DEV				"/dev/serial0" // default uart device for raspberry pi 3b
#define CFG_SER_BAUD			115200


#define SER_BUF_SIZE 1000
unsigned long millis_ml(void);


/***************************************************************************/
/* variables */

static char quit = 0;
static int ser = 0;
static struct termios oldtio;
static unsigned char serbuf[SER_BUF_SIZE];
static short serbuf_cnt;
static unsigned char cmd;

/* implementation of the arduino style millis_ml() to return current number of
   millis_mleconds since application launch */
static unsigned long secs_init = 0;

unsigned long millis_ml(void)
{
    struct timeval te;
    gettimeofday(&te, NULL); /* get current time */

	if (secs_init == 0)
	{
		secs_init = te.tv_sec;
	}

	return ((unsigned long) (te.tv_sec - secs_init)*1000 + te.tv_usec/1000);
}

void ml_tx_update (void)
{
		ser_send (ser, txbuf, txbuf_cnt);
		txbuf_cnt = 0;
}



/* static variables */
FILE *f;
char s[80];
double battery = 0.0;
double pos_raw[3];
double pos_glo[3];
bool heartbeat = false;

/***************************************************************************/
void pos_init(void){
	f = fopen("position.log", "a");
	fprintf (f, "#time_boot,lat,lon,alt,relative_alt,vx,vy,vz,heading\n");
}
/***************************************************************************/


// Possible accelerometer scales (and their register bit settings) are:
// 2 G (00), 4 G (01), 8 G (10), and 16 G  (11).
float aRange = 2.0;       // 2G acceleration resolution (max. sensitivity)
float Ascale = 0.0;         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01
// Possible gyro scales (and their register bit settings) are
// 250 °/s (00), 500 °/s (01), 1000 °/s (10), and 2000 °/s  (11).
float gRange = 250.0;     // 250°/s gyro resolution (max. sensitivity)
float Gscale = 0.0;         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01

float accRes = aRange/32768.0;  // 16 bit, 2G --> 0,000061035 g = per LSB
float gyroRes = gRange/32768.0; // 16 bit, 250 deg/sec --> = per LSB
float compRes = 1229.0/4096.0;  // = 0,3µT per LSB --> 1 Milligauss [mG] =   0,1 Mikrotesla [µT]
float magBias[3] = {1.8, 7.0, 8.0}; // found in arduino code, finds all these calibrations values, and does factory test in initialization
float magCalib[3] = {1.12, 1.14, 1.18}; // found in arduino code

//Definition of variables for time and integral time calculation
uint32_t now, lastupdate, start, end;
double dT; //need to be precise due to short time intervalls
float a_coeff = 0.95; // Coefficient for sensor fusion (how much weight put on the angle integration --> short term trust to gyro)
float a_CompFilter = 0.9; // Coefficient for low pass filtering the raw comp data --> e.g. 0.9 is buidling the floating average of the last 10 values(?)

float pitchAcc, rollAcc, yawMag;  //Define variables for the "raw" angles calculated directly from the Acc values
//Define variables for the angles incl. start values. Those angles are in degrees
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

//roll and pitch are needed in radians for compass tilt compensation
float pitchRad, rollRad;
float RadToDeg = (180.0/M_PI);
// Declination Angle, can be found on  http://www.magnetic-declination.com/
float declinationAngle = 2.59;

float roll_offset = 4.0;
float pitch_offset = 0.5;

// thau observer varibles:
double pos[3];
double old_pos[3];
double state[12];
double old_state[12];
double Px[12];
double old_Px[12];
double Pxdot[12];
double old_Pxdot[12];


void thau(Px[12],Pxdot[12],state[12]){

  // parameters
  float g = 9.81;
  float m = 1.0; // Kg;
  float L = 0.3; // m
  float k = 1.5e-6; // thrust coefficient
  float b = 1e-7; // drag coefficient
  float Ix = 5e-3;  // Kgm^2
  float Iy = 5e-3;  // Kgm^2
  float Iz = 10e-3; // Kgm^2

  float omega1 = m * g;
  float omega2 = m * g;
  float omega3 = m * g;
  float omega4 = m * g;

  // force and torque
  float ft = k*(pow(omega1,2)+pow(omega2,2)+pow(omega3,2)+pow(omega4,2)); // T_B
  float taux = k*L*(pow(omega3,2)-pow(omega1,2));
  float tauy = k*L*(pow(omega4,2)-pow(omega2,2));
  float tauz = b*(pow(omega2,2)+pow(omega4,2)-pow(omega1,2)-pow(omega3,2));


  // State Estimate change, from matlab:
  Pxdot[0] = (Px[3] + Px[5] * Px[1] + Px[4] * Px[0] * Px[1]); // Px(4)+Px(6)*Px(2)+Px(5)*Px(1)*Px(2)
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


  // Matlab functions for the following:
  //PThau = lyap((A+0.5*eye(12))',-C'*C);
  //PThau = inv(PThau)*C';
  // Just copy pasted from matlab solution and writting into the following equations:

  // State estimate update, first Pxdot, then Px.
  // Pxdot = Pxdot + PThau*(C*x-C*Px);
  // Px    = Px + Pxdot * dt;

  Px[0] = Px[0] + ((Pxdot[0] + (1.5 * (state[0] - Px[0]) + 0.5 * (state[3] - Px[3]))) * dt);
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
  }

}
