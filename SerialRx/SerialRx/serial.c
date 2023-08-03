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

#if defined(_WIN32)
HANDLE hComm;
#else

#endif


#if defined (_WIN32)

bool OpenLpLidar(/*HANDLE hComm*/)
{
	//	HANDLE hComm;                          // Handle to the Serial port
	TCHAR* pcCommPort = TEXT("COM5");
	BOOL  Status;                          // Status of the various operations 
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
	BOOL match = false;

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
		printf("\n    Error! - Port %S can't be opened\n", pcCommPort);
		return false;
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

void lidarSerial_write(uint8_t* header, uint16_t length)
{
	DWORD dNoOFBytestoWrite = length;
	DWORD  dNoOfBytesWritten = 0;

	bool Status = WriteFile(hComm,               // Handle to the Serialport
		header,            // Data to be written to the port 
		dNoOFBytestoWrite,   // No of bytes to write into the port
		&dNoOfBytesWritten,  // No of bytes written to the port
		NULL);

};

uint8_t rxBuffer[10];
uint8_t lidarSerial_read()
{
	DWORD NoBytesExpected = 1;
	DWORD NoBytesRecieved = 0;
	bool Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

	Status = ReadFile(hComm, &rxBuffer[0], NoBytesExpected, &NoBytesRecieved, NULL);
	return rxBuffer[0];
}



#endif