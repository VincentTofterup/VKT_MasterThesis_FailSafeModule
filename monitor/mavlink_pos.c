#include <stdio.h>
#include <string.h>

#include <sys/time.h>
#include "mavlink_lora_lib.h"


#include <signal.h>
#include <fcntl.h>
/* #include <sys/shm.h> */
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>


/***************************************************************************/
/* defines */

#define SER_BUF_SIZE 1000

#define CMD_NONE 0
#define CMD_MONITOR_ALL 1
#define CMD_MONITOR_FILTERED 2
#define CMD_MISSION_GET 3
#define CMD_MISSION_SET 4
#define CMD_PARAM_GET 5
#define CMD_PARAM_SET 6
#define CMD_PARAM_LIST 7
#define CMD_POSITION 8

unsigned long millis(void);

#define APP_INIT_OK 9
#define APP_INIT_ERR 10

#define true					1
#define false					0

#define SCHED_INTERVAL  		2e4 /**< schedule interval set to 50 Hz */

#define CFG_SER_DEV				"/dev/ttyACM0"
#define CFG_SER_BAUD			57600
/* #define CFG_SER_BAUD			921600 */

/***************************************************************************/
/* variables */
/*
static char quit = 0;
static int ser = 0;
static struct termios oldtio;
static unsigned char serbuf[SER_BUF_SIZE];
static short serbuf_cnt;
static unsigned char cmd;*/

/***************************************************************************/
/* implementation of the arduino style millis() to return current number of
   milliseconds since application launch */
static unsigned long secs_init = 0;

unsigned long millis(void){
    struct timeval te;
    gettimeofday(&te, NULL); /* get current time */

	if (secs_init == 0)
	{
		secs_init = te.tv_sec;
	}

	return ((unsigned long) (te.tv_sec - secs_init)*1000 + te.tv_usec/1000);
}
/***************************************************************************/
/* Initialzation method for opening serial device and init pos methods     */
/***************************************************************************/
int initpos(){
  char ser_dev[50];
	int ser_err;
	char param_id[17];
	unsigned char ser_dev_set;
	int i;
	int status;

	ser_dev_set = 0;
	status = APP_INIT_OK;
	cmd = CMD_NONE;
	param_id[0] = 0;

	strcpy (ser_dev, CFG_SER_DEV); /* default serial device */

  if (ser_dev_set == 0)
		printf ("Serial device (default): %s\n\n", ser_dev);
	else
		printf ("Serial device: %s\n\n", ser_dev);

	/* try to open the serial device */
	ser_err = ser_open (&ser, &oldtio, ser_dev, CFG_SER_BAUD);

	/* if everything ok */
	if (! ser_err){
    status = APP_INIT_OK;
    ml_init();                 // mavlink me thod, mavlink_lora_lib.h
    pos_init();                // init position
    ml_set_monitor_all();      // mavlink method, mavlink_lora_lib.h
  }
  else{
    status = APP_INIT_ERR;
    printf ("Unable to open serial device\n");
  }
  return status;
}

/****************************************************************************/
double *getpos(){
  double pos[3];
  char result;
	unsigned long now = millis();
	serbuf_cnt = SER_BUF_SIZE;
	serbuf_cnt = ser_receive (ser, serbuf, serbuf_cnt);

  /* if we received new data */
	if (serbuf_cnt > 0){
		result = ml_rx_update(now, serbuf, serbuf_cnt);
		if (result > 0){
			short i, pos;
			pos = 0;
			for (i=0; i < result; i++){
				while (rxbuf_filtered[pos] != ML_NEW_PACKET_IDENT_V10)
					pos++;
				pos++;
			}
			rxbuf_filtered_cnt = 0;
		}
	}
  pos_update();
return pos;
}
/***************************************************************************/
void ml_parse_msg(unsigned char *msg)
{
  double *res;
	switch (cmd)
	{
		case CMD_MONITOR_ALL:
		case CMD_MONITOR_FILTERED:
			//mon_parse_msg(msg, millis());
			break;
		case CMD_MISSION_GET:
			//mission_get_parse_msg(msg, millis());
			break;
		case CMD_MISSION_SET:
			//mission_set_parse_msg(msg, millis());
			break;
		case CMD_PARAM_GET:
			//param_get_parse_msg(msg, millis());
			break;
		case CMD_PARAM_SET:
			//param_set_parse_msg(msg, millis());
			break;
		case CMD_PARAM_LIST:
			//param_list_parse_msg(msg, millis());
			break;
		case CMD_POSITION:
			res = pos_parse_msg(msg, millis());
			break;
	}
}
/***************************************************************************/
/*int main (int argc, char **argv){

}*/
/***************************************************************************/
