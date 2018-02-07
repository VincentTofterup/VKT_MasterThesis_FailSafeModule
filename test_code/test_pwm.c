#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// To compile run:
// gcc -Wall -o test_pwm test_pwm.c -lwiringPi -lpthread
int main (void){
  int l ;
  printf ("PWM test program\n") ;

  //using wiringPi Pin Number
  int pin = 7;
  if (wiringPiSetup () == -1){
    printf("Cannot Access WiringPi, exiting...\n");
    exit (1);
  }

  /*
  //using Physical Pin Number
  int pin = 38;
  if (wiringPiSetupPhys() == -1)
    exit (1) ;
  */

  /*
  //using BCM Pin Number
  int pin = 5;
  if (wiringPiSetupGpio() == -1)
    exit (1);
  */

  pinMode (pin, PWM_OUTPUT);
  for (;;) {
    for (l = 0 ; l < 1024 ; ++l) {
      pwmWrite (pin, l) ;
      delay (1) ;
    }
    for (l = 1023 ; l >= 0 ; --l) {
      pwmWrite (pin, l) ;
      delay (1) ;
    }
  }
  return 0 ;
}
