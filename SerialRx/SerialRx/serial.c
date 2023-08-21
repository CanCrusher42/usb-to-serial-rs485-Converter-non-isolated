// Serial Abstraction Layer


#if defined(_WIN32)
#include <Windows.h>
#include <dos.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>	
#include <stdbool.h>
#include <math.h>

#include "serial.h"

#define TEST_SERIAL

bool serialTestMode = false;
#ifdef TEST_SERIAL
int testBuffer[12800];
int16_t testBufferIndex = 0;
int16_t maxTextBufferIndex = 0;
bool testEmpty = false;
#endif

#if defined(_WIN32)
HANDLE hComm;
#else
#include "../mcc_generated_files/uart2.h"



bool OpenLpLidar()
{
    return 1;
}

void lidarSerial_write(uint8_t* header, uint16_t length)
{
	for (int i = 0; i < length; i++)
	{
        UART2_Write(header[i]);
	}
}

int16_t lidarSerial_read()
{

    if (UART2_IsRxReady()== false)
	{
	  	return -1;
	}
	else
	{
        return UART2_Read();
	}
}

int lidarClear_serial()
{
	UART2_Purge();
    
	return true;
}
#endif

#if defined (_WIN32)

bool OpenLpLidar(/*HANDLE hComm*/)
{
	//	HANDLE hComm;                          // Handle to the Serial port
	TCHAR* pcCommPort = TEXT("COM9");
	TCHAR* pcCommPort5 = TEXT("COM5");
	BOOL  Status;                          // Status of the various operations 
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
	BOOL match = false;

	#ifdef TEST_SERIAL
	return true;
	#endif

	int i = 0;

	printf("\n\n +==========================================+");
	printf("\n |    RS485 Mode - Reception (Win32 API)    |");
	printf("\n +==========================================+\n");
	/*---------------------------------- Opening the Serial Port -------------------------------------------*/

	hComm = CreateFile(pcCommPort /* ComPortName*/,                  // Name of the Port to be Opened
		GENERIC_READ | GENERIC_WRITE, // Read/Write Access
		0,                            // No Sharing, ports cant be shared
		NULL,                         // No Security
		OPEN_EXISTING,                // Open existing port only
		0,                            // Non Overlapped I/O
		NULL);                        // Null for Comm Devices

	if (hComm == INVALID_HANDLE_VALUE)
	{
		printf("\n    Error! - Port %S can't be opened Trying COM5\n", pcCommPort);

		hComm = CreateFile(pcCommPort5 /* ComPortName*/,                  // Name of the Port to be Opened
			GENERIC_READ | GENERIC_WRITE, // Read/Write Access
			0,                            // No Sharing, ports cant be shared
			NULL,                         // No Security
			OPEN_EXISTING,                // Open existing port only
			0,                            // Non Overlapped I/O
			NULL);                        // Null for Comm Devices
		
		if (hComm == INVALID_HANDLE_VALUE)
		{
			printf("\n    Error! - Port %S can't be opened Either \n", pcCommPort5);
			return false;
		}
		else
		{
			printf("\n    Port %S Opened\n ", pcCommPort5);
		}
	}
	else
		printf("\n    Port %S Opened\n ", pcCommPort);

	/*------------------------------- Setting the Parameters for the SerialPort ------------------------------*/

	DCB dcbSerialParams = { 0 };                         // Initializing DCB structure
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	Status = GetCommState(hComm, &dcbSerialParams);      //retreives  the current settings

	if (Status == FALSE)
	{
		printf("\n    Error! in GetCommState()");
		return Status;
	}

	dcbSerialParams.BaudRate = CBR_115200;      // Setting BaudRate = 9600
	dcbSerialParams.ByteSize = 8;             // Setting ByteSize = 8
	dcbSerialParams.StopBits = ONESTOPBIT;    // Setting StopBits = 1
	dcbSerialParams.Parity = NOPARITY;        // Setting Parity = None 
	dcbSerialParams.fDtrControl = 0;          // Enable DTR for the motor
	dcbSerialParams.fRtsControl = 0;//.fDtrControl = 0;          // Enable DTR for the motor

	Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB 

	if (Status == FALSE)
	{
		printf("\n    Error! in Setting DCB Structure");
		return Status;
	}
	else //If Successfull display the contents of the DCB Structure
	{
		printf("\n\n    Setting DCB Structure Successfull\n");
		printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
		printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
		printf("\n       StopBits = %d", dcbSerialParams.StopBits);
		printf("\n       Parity   = %d", dcbSerialParams.Parity);
		printf("\n       fDtrCont = %d", dcbSerialParams.fDtrControl);
		printf("\n       fRtsCont = %d", dcbSerialParams.fRtsControl);

	}

	/*------------------------------------ Setting Timeouts --------------------------------------------------*/

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(hComm, &timeouts) == FALSE)
	{
		printf("\n\n    Error! in Setting Time Outs");
		return Status;
	}
	else
		printf("\n\n    Setting Serial Port Timeouts Successfull");

	/*------------------------------------ Setting Receive Mask ----------------------------------------------*/

	 Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

	if (Status == FALSE)
		printf("\n\n    Error! in Setting CommMask");
	else
		printf("\n\n    Setting CommMask successfull");

	return Status;

}


uint8_t rxBuffer[50] = { 0 };
uint8_t rxAllBuffer[512] = { 0 };
uint32_t rxTimeStamps[512] = { 0 };
uint16_t rxCount = 0;
uint8_t txAllBuffer[512] = { 0 };
uint16_t txCount = 0;
uint32_t txTimeStamps[512] = { 0 };


void lidarSerial_write(uint8_t* header, uint16_t length)
{
	DWORD dNoOFBytestoWrite = length;
	DWORD  dNoOfBytesWritten = 0;

	for (int i = 0; i < length; i++)
	{
		txTimeStamps[txCount] = GetTickCount();
		txAllBuffer[txCount++] = header[i];
		txCount &= 0x1FF;
	}


	bool Status = WriteFile(hComm,               // Handle to the Serialport
		header,            // Data to be written to the port 
		dNoOFBytestoWrite,   // No of bytes to write into the port
		&dNoOfBytesWritten,  // No of bytes written to the port
		NULL);

	if (dNoOFBytestoWrite != dNoOfBytesWritten)
	{
		printf("SOMETHING BAD HAS HAPPENED.  Not ALL DATA WRITTEN\n");
	}

};

int lidarSerial_read()
{

#ifdef TEST_SERIAL

	if ((((testBufferIndex+1) % 33) == 0) && (testEmpty == false))
	{
		testEmpty = true;
	}
	else 
		testEmpty = false;

	if ((testBufferIndex < maxTextBufferIndex) && (testEmpty==false))
	{
		return testBuffer[testBufferIndex++];
	}
	else
		return -1;
#endif


	DWORD NoBytesExpected = 1;
	DWORD NoBytesRecieved = 0;
	bool Status;
	//Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
	rxTimeStamps[rxCount] = GetTickCount();
	Status = ReadFile(hComm, &rxBuffer[0], NoBytesExpected, &NoBytesRecieved, NULL);

	if (NoBytesRecieved == 0)
	{
	  	return -1;
	}
	else
	{
		rxAllBuffer[rxCount++] = rxBuffer[0];
		rxCount &= 0x1FF;
		return rxBuffer[0];
	}
}

int lidarClear_serial()
{
	bool success = PurgeComm(hComm, PURGE_RXCLEAR);
	return success;
}

#endif