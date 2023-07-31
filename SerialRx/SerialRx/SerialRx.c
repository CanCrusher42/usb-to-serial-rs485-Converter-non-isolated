
	//====================================================================================================//
	// USB to RS485 conversion  using USB2SERIAL (Read) (Receiver Side) (WIN32 API)                       //
	//----------------------------------------------------------------------------------------------------//
	// The Program runs on the PC side and uses Win32 API to communicate with USB2SERIAL hardware board-  //
	// -which converts the USB signals to RS485.                                                          //       
	//----------------------------------------------------------------------------------------------------//
	// Program runs on the PC side (Windows) and receives a string of characters.                         //            
	// Program uses CreateFile() function to open a connection serial port(COMxx).                        //
	// Program then sets the parameters of Serial Comm like Baudrate,Parity,Stop bits in the DCB struct.  //
	// EsapeCommFunction() is used to control DTR and RTS and to set the direction of MAX485 chip         //
	// After setting the Time outs,the Program waits for the reception of string of characters by setting-//
	// -up the  WaitCommEvent().                                                                          //
	// When a character is reeived bythe PC UART WaitCommEvent() returns and the received string is read- //
	// -using ReadFile(); function.The characters are then displayed on the Console.                      //
	//----------------------------------------------------------------------------------------------------// 
	// BaudRate     -> 9600                                                                               //
	// Data formt   -> 8 databits,No parity,1 Stop bit (8N1)                                              //
	// Flow Control -> None                                                                               //
	//----------------------------------------------------------------------------------------------------//
	//                                                                                                    //
	//  |------------|                                                                                    //
	//  | RS485      |                                               +---------+     +-----------+        //
	//  | Read.exe   |             USB2SERIAL V2.0                   |		 RO|---->|P1.1(RXD)  |        //
	//  |------------|            +--------------+                   |      ~RE|-----|P2.1       |        //
	//   \ [][][][][] \===========| USB <- RS485 |~~~~~~~~~~~~~~~~~~~|A,B      |     |           |        //
	//    \------------\   USB    +--------------+      Twisted      |       DE|-----|P2.0       |        //
	//        Laptop                                     Pair        |	     DI|<----|P1.2(TXD)  |        //
	//                                                               +---------+     +-----------+        //
	//                              						            MAX485        MSP430G2553         //
	//                                                                                                    //
	//      [Reciever] <---------------------------------------   [--------Transmitter-----------]        //
	//                            direction of data                                                       //
	//====================================================================================================//
	// Pin Configuration for  USB2SERIAL V2.0 Board                                                       //
	//                                                                                                    //
	//  		+==========================================================+                              //
	//			|	 +-----------+               +---------+               |                              //
	//  		|    |   	  TXD|---------------|DI       |               |                              //
	// 	 	 +--|	 |        RXD|<--------------|RO      A|------+        |                              //
	//	USB	 |	     |USB        |               |         |      |        |                              //
	// 	     +--|    |       ~RTS|-------------->|~RE     B|------|--+     |                              //
	//  		|    |	     ~DTR|-------------->|DE       |      |  |     |                              //
	//  		|    +-----------+               +---------+      |  |     |                              //
	//			|		FT232RL                    MAX485         |  |     |                              //
	//  		+================================================[A]=[B]===+                              //
	//                                                             RS485                                  //
	//                                                                                                    //
	//			                                                                                          //
	//====================================================================================================//
	// Compiler/IDE  :	Microsoft Visual Studio Express 2013 for Windows Desktop(Version 12.0)            //
	//               :  gcc 4.8.1 (MinGW)                                                                 //
	//                                                                                                    //
	// Library       :  Win32 API,windows.h,                                                              //
	// Commands      :  gcc -o USB2SERIAL_RS485_Read_W32 USB2SERIAL_RS485_Read_W32.c -std=c99             //
	// OS            :	Windows(Windows 7)                                                                //
	// Programmer    :	Rahul.S                                                                           //
	// Date	         :	30-November-2014                                                                  //
	//====================================================================================================//

	//====================================================================================================//
	// Sellecting the COM port Number                                                                     //
	//----------------------------------------------------------------------------------------------------//
	// Use "Device Manager" in Windows to find out the COM Port number allotted to USB2SERIAL converter-  // 
	// -in your Computer and substitute in the  "ComPortName[]" array.                                    //
	//                                                                                                    //
	// for eg:-                                                                                           //
	// If your COM port number is COM32 in device manager(will change according to system)                //
	// then                                                                                               //
	//			char   ComPortName[] = "\\\\.\\COM32";                                                    //
	//====================================================================================================//

#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>	
#include <stdbool.h>
#include <math.h>
#include <dos.h>

HANDLE hComm;

#pragma pack(1)
struct sample_struct {
	uint8_t quality;
	uint16_t angle;
	uint16_t distance;
};

#define SERIAL_BUFFER_SIZE 4096
#define LINE_BUFFER_SIZE 512

char  SerialBuffer[SERIAL_BUFFER_SIZE];               // Buffer Containing Rxed Data
struct sample_struct lineBuffer[LINE_BUFFER_SIZE];
uint8_t scanMode = 0;
enum eSCAN_MODES { BASIC_SCAN = 0x20, EXPRESS_SCAN = 0x82 };


void PrintHexBuffer(uint8_t* buffer, int bufferSize)
{
	for (int i = 0; i < bufferSize; i++)
	{
		if (i < (bufferSize - 1))
			printf("0x%02x, ", buffer[i]);
		else
			printf("0x%02x", buffer[i]);
	}
}

void PrintRawBuffer(uint8_t* buffer, int numChars)
{
	/*------------Printing the RXed String to Console----------------------*/

	


	for (int j = 0; j < numChars - 1; j++)		// j < i-1 to remove the dupliated last character
	{
		if (((j % 5) == 0) && ((buffer[j] & 1) == 1))
			printf("* %d \n", j);
		if ((j % 20) == 0)
			printf("\n");

		printf("0x%02x, ", (uint8_t)buffer[j]);

	}

}

void DisplayLineToRoom(struct sample_struct sampleLine[LINE_BUFFER_SIZE], int numSamples, uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	// Middle Angle = startAngle+90;
	// Get Max Neg and Pos X
	// Get Max Neg and Pos Y
	int xValue[180];
	int yValue[180];
	int minX = 1000, maxX = -1000;
	int maxY = -1000;

	int angleOffset = startAngle;
	int yPerRow, xPerColumn;

	// Get Scale Values
	for (uint16_t angle = startAngle; angle < (startAngle + 180); angle++)
	{
		
		xValue[angle-startAngle] = round((-1.0 * cos((angle - angleOffset)* 0.0174533)) * (float)sampleLine[angle].distance);
		yValue[angle - startAngle] = round((sin((angle - angleOffset) * 0.0174533)) * (float)sampleLine[angle].distance);

		if (xValue[angle - startAngle] < minX)
			minX = xValue[angle - startAngle];
		if (xValue[angle - startAngle] > maxX)
			maxX = xValue[angle - startAngle];
		if (yValue[angle - startAngle] > maxY)
			maxY = yValue[angle - startAngle];
	}
	yPerRow = maxY / maxHeight ;
	xPerColumn = ( max(abs(minX), abs(maxX)) * 2) / 180;
	int yPerRow = maxY / maxHeight;





	
}
void DisplayLineDistance(struct sample_struct sampleLine[LINE_BUFFER_SIZE],int numSamples,  uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	char displayLine[365];
	uint16_t width = endAngle - startAngle;
	if (width > 360)
		width = 360;
	char barChar = '#';

	// get tallest bar
	uint16_t highest = 0;  sampleLine[0].distance;
	// ERROR THIS SHOULD CHECK BE CHECKING BETWEEN THE TARGETS
	for (uint16_t check = startAngle; check < endAngle; check++)
	{
		if (sampleLine[check].distance > highest)
			highest = sampleLine[check].distance;
	}

	int scale = highest / maxHeight;
	for (uint16_t rows = maxHeight; rows >= 1; rows--)
	{
		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (sampleLine[cols].distance >= (rows*scale))
				displayLine[cols-startAngle] = barChar;
			else
				displayLine[cols-startAngle] = ' ';

			if ((rows == 1) && (sampleLine[cols].quality<3))
				displayLine[cols-startAngle] = '?';
		}
		
		displayLine[width] = 0;
		printf("%05d|", rows * scale);
		printf("%s\n", displayLine);
	}

	printf("%s","      ");
	for (int loc = startAngle; loc < endAngle; loc++)
		printf("%c",'-');
	
	printf("\n");


	for (int loc = 2; loc >= 0; loc--)
	{
		unsigned int power=0;
		if (loc == 2) power = 100;
		if (loc == 1) power = 10;

		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (cols >= power)
				if (power == 100)
				{
					displayLine[cols] = '0' + ((cols / power) );
				}
				else if (power == 10)
				{
					displayLine[cols] = '0' + ((cols / power) % power);
				}
				else
				{
					displayLine[cols] = '0' + (cols  % 10);
				}
			else
				displayLine[cols] = '0';
		}
		displayLine[width] = 0;
		printf("      %s\n", displayLine);

	}
	

}

void DisplayLineQuality(struct sample_struct sampleLine[LINE_BUFFER_SIZE], int numSamples, uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight)
{
	char displayLine[365];
	uint16_t width = endAngle - startAngle;
	if (width > 360)
		width = 360;
	char barChar = '#';

	// get tallest bar
	uint16_t highest = 0;  sampleLine[0].distance;
	// ERROR THIS SHOULD CHECK BE CHECKING BETWEEN THE TARGETS
	for (uint16_t check = startAngle; check < endAngle; check++)
	{
		if (sampleLine[check].quality > highest)
			highest = sampleLine[check].quality;
	}

	int scale = highest / maxHeight;
	for (uint16_t rows = maxHeight; rows >= 1; rows--)
	{
		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (sampleLine[cols].quality >= (rows * scale))
				displayLine[cols - startAngle] = barChar;
			else
				displayLine[cols - startAngle] = ' ';
		}
		displayLine[width] = 0;
		printf("%05d|", rows * scale);
		printf("%s\n", displayLine);
	}

	printf("%s", "      ");
	for (int loc = startAngle; loc < endAngle; loc++)
		printf("%c", '-');

	printf("\n");


	for (int loc = 2; loc >= 0; loc--)
	{
		unsigned int power = 0;
		if (loc == 2) power = 100;
		if (loc == 1) power = 10;

		for (uint16_t cols = startAngle; cols < endAngle; cols++)
		{
			if (cols >= power)
				if (power == 100)
				{
					displayLine[cols] = '0' + ((cols / power));
				}
				else if (power == 10)
				{
					displayLine[cols] = '0' + ((cols / power) % power);
				}
				else
				{
					displayLine[cols] = '0' + (cols % 10);
				}
			else
				displayLine[cols] = '0';
		}
		displayLine[width] = 0;
		printf("      %s\n", displayLine);

	}


}


int TransferBasicArrayToLine(uint8_t* rawBuffer, struct sample_struct* lineBuffer)
{
	int lineSize = 0;
	int charCount = 0;

	int index;

	while (lineSize == 0)
	{
		// Use angle as index into array of 360 items
		index = ((rawBuffer[charCount + 1] >> 1) + (((uint16_t)rawBuffer[charCount + 2]) << 7) ) >> 6;
		
		if ((rawBuffer[charCount] >> 2) != 0)
		{
			lineBuffer[index].quality = rawBuffer[charCount] >> 2;


			//lineBuffer[index].angle    = (rawBuffer[charCount + 1] >> 1) + (((uint16_t)rawBuffer[charCount + 2]) << 7);

			lineBuffer[index].angle = rawBuffer[charCount + 2];
			lineBuffer[index].angle = (lineBuffer[index].angle << 7) | (rawBuffer[charCount + 1] >> 1);

			lineBuffer[index].distance = (((uint16_t)rawBuffer[charCount + 4]) << 8) + rawBuffer[charCount + 3];
			lineBuffer[index].distance = lineBuffer[index].distance / 4;  // Sensor is /4 mm
		}
		// look for start of next line
		if ((charCount > 0) & ((rawBuffer[charCount] & 3) == 1))
			lineSize++;
		else
			charCount += 5;
	}
	return charCount;
}

#define EXPRESSED_PACKET_SIZE 84
#define ANGLES_DIV_RATIO 6



int TransferExpressGroupToLine(uint8_t rawBuffer[], struct sample_struct* lineBuffer)
{
	int lineSize = 0;
	int charCount = 0;
	uint16_t startAngleQ6w0, startAngleQ6w1;
	int cabin;
	int angleDiff;
	int index;
	uint8_t *cabinStart;
	uint16_t dist1, dist2;
	uint16_t ang1, ang2;
	uint16_t d0,d1;

	if (((rawBuffer[0] & 0xF0) == 0xA0) && ((rawBuffer[1] & 0xF0) == 0x50))
	{
		startAngleQ6w0 = rawBuffer[3] & 0x7F;
		startAngleQ6w0 = (startAngleQ6w0 << 8) + rawBuffer[2];
		startAngleQ6w1 = rawBuffer[EXPRESSED_PACKET_SIZE + 3] & 0x7F;
		startAngleQ6w1 = (startAngleQ6w1 << 8 ) + rawBuffer[EXPRESSED_PACKET_SIZE + 2];

		if (startAngleQ6w0 <= startAngleQ6w1)
			angleDiff = (startAngleQ6w1 - startAngleQ6w0) ;
		else
			angleDiff = (360 + startAngleQ6w1 - startAngleQ6w0) ;

		uint16_t anglePerCabin = angleDiff;

		angleDiff = angleDiff / 32;  //(angle per sample)


		cabinStart = rawBuffer + 4;
		for (cabin = 0; cabin < 16; cabin++)
		{
			dist1 = (cabinStart[cabin * 5 + 1] <<6);
			dist1 = dist1 + (cabinStart[cabin * 5] >> 2);
			dist2 = (cabinStart[cabin * 5 + 3] << 6);
			dist2 = dist1 + (cabinStart[cabin * 5 + 2] >> 2);
			d0 = ((cabinStart[cabin  ] & 0x2) << 4) +  (cabinStart[4] & 0xF);
			d1 = ((cabinStart[cabin+2] & 0x2) << 4) + ((cabinStart[4]>>4) & 0xF);
			
			ang1 = startAngleQ6w0 + (angleDiff * 2*cabin) - d0;
			ang2 = startAngleQ6w0 + (angleDiff * (2*cabin+1)) - d1;

			if (dist1 > 0)
			{
				lineBuffer[ang1 >> ANGLES_DIV_RATIO].angle = ang1;
				lineBuffer[ang1 >> ANGLES_DIV_RATIO].distance = dist1;
			}
			if (dist2 > 0)
			{
				lineBuffer[ang2 >> ANGLES_DIV_RATIO].angle = ang2;
				lineBuffer[ang2 >> ANGLES_DIV_RATIO].distance = dist2;
			}

		}
	}
	return charCount;
}

bool TransferExpressArrayToLine(uint8_t *SerialBuffer, struct sample_struct* lineBuffer)
{
	uint8_t *buffer = SerialBuffer;
	bool done = false;
	int count;

	while (done == false)
	{
		count = TransferExpressGroupToLine(buffer, lineBuffer);
		buffer += EXPRESSED_PACKET_SIZE;
		
		if (buffer[3] & 0x80)
			done = true;

	}
}


extern uint8_t ExpressTestSample1[];

bool TestTransferExpressGroupToLine()
{

	//struct sample_struct lineBuffer[LINE_BUFFER_SIZE];
	
	TransferExpressGroupToLine(ExpressTestSample1, lineBuffer);
}








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

bool ResetAndStartCapture(HANDLE hComm, enum eSCAN_MODES scanMode)
{
	uint8_t   lpBuffer[20];		       // lpBuffer should be  char or byte array, otherwise write wil fail
	DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
	BOOL  Status;                          // Status of the various operations 
	DWORD dwEventMask;                     // Event mask to trigger
	uint8_t tempChar[100];
	DWORD NoBytesRecieved;


	
	// --------------------------------------
    // First Stop anything working now.
	// --------------------------------------
	lpBuffer[0] = 0x25;  // STOP

	dNoOFBytestoWrite = 1; // Calculating the no of bytes to write into the port

	bool success = PurgeComm(hComm,PURGE_RXCLEAR);
	if (success == false)
	{
		printf("\n\n\nPurge Status = %d \n", success);
		return false;
	}

	Status = WriteFile(hComm,               // Handle to the Serialport
		lpBuffer,            // Data to be written to the port 
		dNoOFBytestoWrite,   // No of bytes to write into the port
		&dNoOfBytesWritten,  // No of bytes written to the port
		NULL);
	Sleep(20);





	if (scanMode == BASIC_SCAN)
	{
		printf("\n    BASIC SCAN    \n");
		lpBuffer[0] = 0xA5;
		lpBuffer[1] = 0x20;
		lpBuffer[2] = 0x0;

		dNoOFBytestoWrite = 2; // Calculating the no of bytes to write into the port

		success = PurgeComm(hComm, PURGE_RXCLEAR);
		if (success == false)
		{
			printf("\n\n\nBasic Purge Status = %d \n", success);
			return false;
		}

		Status = WriteFile(hComm,               // Handle to the Serialport
			lpBuffer,            // Data to be written to the port 
			dNoOFBytestoWrite,   // No of bytes to write into the port
			&dNoOfBytesWritten,  // No of bytes written to the port
			NULL);

		if (Status == TRUE)
			printf("\n\n    0x%02x 0x%02x - Written to Com Port", (uint8_t)lpBuffer[0], (uint8_t)lpBuffer[1]);
		else
		{
			printf("\n\n   Error %d in Writing to Serial Port", GetLastError());
			return false;
		}

	
		Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received

		printf("\n\n    Basic Characters Received");
		Status = ReadFile(hComm, &tempChar[0], 50, &NoBytesRecieved, NULL);
		if (Status == FALSE)
		{
			printf("\n    Error! in Setting WaitCommEvent()");
			return false;
		}
		else
		{
			PrintHexBuffer(tempChar, NoBytesRecieved);
			printf("\n");
		}
	}



	if (scanMode == EXPRESS_SCAN)
	{
		printf("\n    EXPRESS SCAN    \n");
		lpBuffer[0] = 0xA5;
		lpBuffer[1] = 0x82;
		lpBuffer[2] = 0x05;
		lpBuffer[3] = 0x00;
		lpBuffer[4] = 0x00;
		lpBuffer[5] = 0x00;
		lpBuffer[6] = 0x00;
		lpBuffer[7] = 0x00;
		lpBuffer[8] = 0x22;

		dNoOFBytestoWrite = 9; // Calculating the no of bytes to write into the port

		success = PurgeComm(hComm, PURGE_RXCLEAR);
		if (success == false)
		printf("\n\n\nPurge Status = %d \n", success);

		Status = WriteFile(hComm,               // Handle to the Serialport
			lpBuffer,            // Data to be written to the port 
			dNoOFBytestoWrite,   // No of bytes to write into the port
			&dNoOfBytesWritten,  // No of bytes written to the port
			NULL);

		if (Status == TRUE)
		{
			printf("\n  Command to sensor \n");
			PrintHexBuffer(lpBuffer, dNoOFBytestoWrite);
		}
		else
			printf("\n\n   Error %d in Writing to Serial Port", GetLastError());

		printf("\n\n    Waiting for Data Reception");

		Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received

		printf("\n\n    Waiting for response characters\n");
		Status = ReadFile(hComm, &tempChar[0], 7, &NoBytesRecieved, NULL);
		if (Status == FALSE)
		{
			printf("\n    Error! in Setting WaitCommEvent()");
		}
		else
		{
			PrintHexBuffer(tempChar, NoBytesRecieved);
			printf("\n");
		}
	}




};

int getRawBasicScanLineOfData(HANDLE hComm, int numChars)
{
	BOOL  Status;                          // Status of the various operations 
	DWORD dwEventMask;                     // Event mask to trigger
	char  TempChar[10];                        // Temperory Character

	DWORD NoBytesRecieved;                 // Bytes read by ReadFile()


	DWORD  dNoOFBytestoRead;              // No of bytes to write into the port
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
	BOOL match = false;

	int i = 0;

	printf("\n\n    Waiting for Data Reception");

	Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received



	/*-------------------------- Program will Wait here till a Character is received ------------------------*/
	int loops = 0;
	bool startCapture = false;
	if (Status == FALSE)
	{
		printf("\n    Error! in Setting WaitCommEvent()");
	}
	else 
	{
		startCapture = false;
		dNoOFBytestoRead = 1;
		do
		{
			Status = ReadFile(hComm, &TempChar[0], 1, &NoBytesRecieved, NULL);

			for (DWORD h = 0; h < NoBytesRecieved; h++)
				SerialBuffer[i + h] = TempChar[h];
			i += NoBytesRecieved;
#define NUM_SYNCS 7
			if (startCapture == false)
			{
				// Start checking once we have received at least the number of samples needed to sync into the data stream
				if (i == ((NUM_SYNCS * 5) - 3))
				{
					// Look for the Start Scan seqeunce
					startCapture = (((SerialBuffer[0] & 3) == 1) && ((SerialBuffer[1] & 1) == 1));

					// followed by the rest of the samples being normal.
					for (int sync_num = 5; (sync_num < (NUM_SYNCS * 3) && startCapture); sync_num += 5)
					{
						startCapture = startCapture && (((SerialBuffer[sync_num] & 3) == 2) && ((SerialBuffer[sync_num + 1] & 1) == 1));
					}

					if (startCapture == false)
					{
						for (int shift = 0; shift < ((NUM_SYNCS * 5) - 4)/*11*/; shift++)
							SerialBuffer[shift] = SerialBuffer[shift + 1];
						i--;
					}
				}
			}
		} while (i < numChars);// (NoBytesRecieved > 0);
	}
}


int getRawExpressScanLineOfData(HANDLE hComm, int numChars)
{
	BOOL  Status;                          // Status of the various operations 
	DWORD dwEventMask;                     // Event mask to trigger
	char  TempChar[10];                        // Temperory Character

	DWORD NoBytesRecieved;                 // Bytes read by ReadFile()


	DWORD  dNoOFBytestoRead;              // No of bytes to write into the port
	DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
	BOOL match = false;

	int i = 0;

	printf("\n\n    Waiting for Data Reception");

	Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received

	//EXPRESSED_PACKET_SIZE

	/*-------------------------- Program will Wait here till a Character is received ------------------------*/
	int loops = 0;
	bool startCapture = false;
	if (Status == FALSE)
	{
		printf("\n    Error! in Setting WaitCommEvent()");
	}
	else
	{
		startCapture = false;
		dNoOFBytestoRead = 1;
		do
		{
			Status = ReadFile(hComm, &TempChar[0], 1, &NoBytesRecieved, NULL);

			for (DWORD h = 0; h < NoBytesRecieved; h++)
				SerialBuffer[i + h] = TempChar[h];
			i += NoBytesRecieved;


#define NUM_EXPRESS_SYNCS 1
			if (startCapture == false)
			{
				// Start checking once we have received at least the number of samples needed to sync into the data stream
				if (i == (((NUM_EXPRESS_SYNCS -1 ) * EXPRESSED_PACKET_SIZE) + 4))
				{
					// Look for the Start Scan seqeunce
					startCapture = (((SerialBuffer[0] & 0xF0) == 0xA0) && ((SerialBuffer[1] & 0xF0) == 0x50));
					startCapture = startCapture && ((SerialBuffer[3] & 0x80) != 0);

					if (startCapture == false)
					{
						for (int shift = 0; shift < 3; shift++)
							SerialBuffer[shift] = SerialBuffer[shift + 1];
						i--;
					}
					else
					{
						startCapture = startCapture;
					}
				}
			}
		} while (i < numChars);// (NoBytesRecieved > 0);
	}
}

//#define DO_EPRESS
#define DO_BASIC 
	void main(void)
		{
			BOOL  Status;                          // Status of the various operations 

		//	TestTransferExpressGroupToLine();
			printf("\n\n");
	//		DisplayLineDistance(lineBuffer, 100, 0, 180, 80, 50);

			Status = OpenLpLidar();

#ifdef DO_BASIC

			ResetAndStartCapture(hComm, BASIC_SCAN);
			getRawBasicScanLineOfData(hComm, 1500);
			//getBasicScanLineOfData(hComm,1500);
#else
			ResetAndStartCapture(hComm, EXPRESS_SCAN);
			getRawExpressScanLineOfData(hComm, 2000);
#endif
			system("cls");
			PrintRawBuffer(SerialBuffer, 1500);

#ifdef DO_BASIC 
			int bytesInLine = TransferBasicArrayToLine(SerialBuffer, lineBuffer);
#else
			int bytesInLine = TransferExpressArrayToLine(SerialBuffer, lineBuffer);
#endif
			system("cls");
			DisplayLineQuality(lineBuffer, bytesInLine / 5, 0, 180, 80, 10);

		//	system("cls");
			printf("\n\n");
			DisplayLineDistance(lineBuffer, bytesInLine / 5, 0, 180, 80, 50);

			DisplayLineToRoom(lineBuffer, bytesInLine / 5, 0, 180, 80, 50);
			CloseHandle(hComm);//Closing the Serial Port
			printf("\n +==========================================+\n");
			_getch();
		}//End of Main()