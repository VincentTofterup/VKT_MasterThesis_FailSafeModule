#include <iostream>
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
