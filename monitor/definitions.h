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
