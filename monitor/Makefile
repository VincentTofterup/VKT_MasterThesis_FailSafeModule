# MavLink LoRa application
# Copyright (c) 2017, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# All rights reserved.

PROJECT=failsafe_module
SOURCES= definitions.h LatLong-UTMconversion.h polygon.h mavlink_lora_lib.c serial.c monitor.cpp
CC = g++
CFLAGS = -std=c++11 -lwiringPi

default:
	@echo ""
	@echo "make............build the $(PROJECT) file"
	@echo "make clean......clean up object and executable files"
	@echo ""

	$(CC) $(CFLAGS) -I./ -o $(PROJECT) $(SOURCES)

clean:
	rm -f $(PROJECT)
