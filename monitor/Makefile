# MavLink LoRa application
# Copyright (c) 2017, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# All rights reserved.

PROJECT=failsafe_module
SOURCES=  monitor.cpp checksum.h mavlink_lora_lib.c serial.c
CC = g++
CFLAGS = -std=c++11

default:
	@echo ""
	@echo "make............build the $(PROJECT) file"
	@echo "make clean......clean up object and executable files"
	@echo ""

	$(CC) $(CFLAGS) -I./ -o $(PROJECT) $(SOURCES)

clean:
	rm -f $(PROJECT)
