#pragma once




#ifndef SERIAL_HEADER_H
#define SERIAL_HEADER_H

#define TEST_SERIAL

bool OpenLpLidar(/*HANDLE hComm*/);
void lidarSerial_write(uint8_t* header, uint16_t length);
int lidarSerial_read();
int lidarClear_serial();

#endif