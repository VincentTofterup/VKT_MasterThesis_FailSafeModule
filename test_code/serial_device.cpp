/*====================================================================================================*/
/* Serial Port reading in Linux                                                                       */
/* Non Cannonical mode                                                                                */
/*----------------------------------------------------------------------------------------------------*/
/* Program reads a string from the serial port at 9600 bps 8N1 format                                 */
/* Baudrate - 9600                                                                                    */
/* Stop bits -1                                                                                       */
/* No Parity                                                                                          */
/*----------------------------------------------------------------------------------------------------*/
/* Compiler/IDE  : gcc                                                                                */
/* Library       :                                                                                    */
/* Commands      : g++ -std=c++11 -o serial_device serial_device.cpp                                  */
/* Programmer    : Vincent Klyverts Tofterup                                                          */
/* Inspiration   : http://xanthium.in/Serial-Port-Programming-on-Linux                                */
/* Date	         : 22-Februray-2018                                                                   */
/*----------------------------------------------------------------------------------------------------*/
/* BSD 3-Clause License                                                                               */
/*                                                                                                    */
/* Copyright (c) 2018, Vincent Tofterup                                                               */
/* All rights reserved.                                                                               */
/*                                                                                                    */
/* Redistribution and use in source and binary forms, with or without                                 */
/* modification, are permitted provided that the following conditions are met:                        */
/*                                                                                                    */
/*   * Redistributions of source code must retain the above copyright notice, this                    */
/*     list of conditions and the following disclaimer.                                               */
/*                                                                                                    */
/*   * Redistributions in binary form must reproduce the above copyright notice,                      */
/*     this list of conditions and the following disclaimer in the documentation                      */
/*     and/or other materials provided with the distribution.                                         */
/*                                                                                                    */
/*   * Neither the name of the copyright holder nor the names of its                                  */
/*     contributors may be used to endorse or promote products derived from                           */
/*     this software without specific prior written permission.                                       */
/*                                                                                                    */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"                        */
/* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE                          */
/* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                     */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE                       */
/* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL                         */
/* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR                         */
/* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER                         */
/* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,                      */
/* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE                      */
/* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                               */
/*                                                                                                    */
/*====================================================================================================*/
#include <stdio.h>
#include <fcntl.h>                                                   /* File Control Definitions           */
#include <termios.h>                                                 /* POSIX Terminal Control Definitions */
#include <unistd.h>                                                  /* UNIX Standard Definitions          */
#include <errno.h>                                                   /* ERROR Number Definitions           */
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

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

  /*----------------------------------------- Read data from serial port ---------------------------------------*/

  //std::vector <int> raw_data;
  std::ofstream myfile;
  myfile.open ("IMU_data.txt",  std::ios_base::app);


for (int k = 0; k < 5; k++) {                                     /* Should be a while true ? What about closing file? */
  tcflush(fd, TCIFLUSH);                                            /* Discards old data in the rx buffer            */

	char read_buffer[256];                                            /* Buffer to store the data received              */
	int  bytes_read = 0;                                              /* Number of bytes read by the read() system call */
	int i = 0;

	bytes_read = read(fd,&read_buffer,256);                           /* Read the data                   */

  //printf("\nBytes Rxed -%d", bytes_read);                           /* Print the number of bytes read */

  if(bytes_read > 1){

    //std::string tmp;

    for(i=0;i<bytes_read;i++){	                                    /* Write only the received characters*/
      //printf("%c",read_buffer[i]);
      myfile << read_buffer[i];
      //tmp+=read_buffer[i];

      if(read_buffer[i] == ' '){
        //printf(", ");
        myfile << ", ";

        //std::string::size_type sz;
        //raw_data.push_back(std::stoi(tmp, &sz));
        //tmp.erase();
      }
    }
  }
}

//for(int i=0; i<raw_data.size(); i++){
  //std::cout << raw_data[i] << std::endl;
//}


  //myfile.close();
	// printf("\n+----------------------------------+\n\n\n");
	close(fd);                                                       /* Close the serial port */
  return 0;
}
