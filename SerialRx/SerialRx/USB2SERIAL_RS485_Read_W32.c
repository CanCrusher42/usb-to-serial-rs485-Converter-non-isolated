
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
#include <stdint.h>	
	void main(void)
		{
			HANDLE hComm;                          // Handle to the Serial port
			char  ComPortName[] = "\\\\.\\COM5";  // Name of the Serial port(May Change) to be opened,
			TCHAR* pcCommPort = TEXT("COM5");
			BOOL  Status;                          // Status of the various operations 
			DWORD dwEventMask;                     // Event mask to trigger
			char  TempChar[10];                        // Temperory Character
			char  SerialBuffer[1024];               // Buffer Containing Rxed Data
			DWORD NoBytesRecieved;                 // Bytes read by ReadFile()

			uint8_t   lpBuffer[5];		       // lpBuffer should be  char or byte array, otherwise write wil fail
			DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
			DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port


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
				printf("\n    Error! - Port %s can't be opened\n", pcCommPort);
			else
				printf("\n    Port %S Opened\n ", pcCommPort);

			/*------------------------------- Setting the Parameters for the SerialPort ------------------------------*/
			
			DCB dcbSerialParams = { 0 };                         // Initializing DCB structure
			dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

			Status = GetCommState(hComm, &dcbSerialParams);      //retreives  the current settings

			if (Status == FALSE)
				printf("\n    Error! in GetCommState()");

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
			timeouts.ReadIntervalTimeout         = 50;
			timeouts.ReadTotalTimeoutConstant    = 50;
			timeouts.ReadTotalTimeoutMultiplier  = 10;
			timeouts.WriteTotalTimeoutConstant   = 50;
			timeouts.WriteTotalTimeoutMultiplier = 10;
			
			if (SetCommTimeouts(hComm, &timeouts) == FALSE)
				printf("\n\n    Error! in Setting Time Outs");
			else
				printf("\n\n    Setting Serial Port Timeouts Successfull");

			/*------------------------------------ Setting Receive Mask ----------------------------------------------*/
			
			Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
	
			if (Status == FALSE)
				printf("\n\n    Error! in Setting CommMask");
			else
				printf("\n\n    Setting CommMask successfull");

			/*------------------------- Putting MAX485 chip in USB2SERIAL in Receive Mode ----------------------------*/
			//                                                                                                        //
			//	----+			+-----------+              L  +-----------+                                           //
			//		|			| 	    ~RTS| --------------> |~RE        |                                           //
			//	 PC |<==========| FT232     |                 |   MAX485  +(A,B)<~~~~~~~~~~~~~~~Data in(RS485)        //
			//	    |    USB    |       ~DTR| --------------> | DE        |        Twisted Pair                       //
			//  ----+			+-----------+              L  +-----------+                                           //
			//                                                                                                        //
			//--------------------------------------------------------------------------------------------------------//
			//RxMode - DE->Low,~RE -> Low

			/*
			Status = EscapeCommFunction(hComm, SETDTR);// SETDTR will make FT232 DTR-LOW (inverted),DE-LOW for Reception

			if (Status == TRUE)
				printf("\n\n    DE of MAX485 is Low (Receive Mode)");
			else
				printf("\n\n   Error! in EscapeCommFunction(hComm, SETDTR)");

			Status = EscapeCommFunction(hComm, SETRTS);//SETRTS will make FT232 RTS-LOW(inverted),~RE-LOW for Reception
			*/
			if (Status == TRUE)
				printf("\n   ~RE of MAX485 is Low (Receive Mode)");
			else
				printf("\n   Error! in EscapeCommFunction(hComm, SETRTS)");

	

			lpBuffer[0] = 0xA5;
			lpBuffer[1] = 0x20;
			lpBuffer[2] = 0x0;

			dNoOFBytestoWrite = 2; // Calculating the no of bytes to write into the port

			Status = WriteFile(hComm,               // Handle to the Serialport
				lpBuffer,            // Data to be written to the port 
				dNoOFBytestoWrite,   // No of bytes to write into the port
				&dNoOfBytesWritten,  // No of bytes written to the port
				NULL);

			if (Status == TRUE)
				printf("\n\n    0x%02x 0x%02x - Written to %S", (uint8_t)lpBuffer[0], (uint8_t)lpBuffer[1], pcCommPort);
			else
				printf("\n\n   Error %d in Writing to Serial Port", GetLastError());


			/*------------------------------------ Setting WaitComm() Event   ----------------------------------------*/

			printf("\n\n    Waiting for Data Reception");

			Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received



			/*-------------------------- Program will Wait here till a Character is received ------------------------*/				
			int loops = 0;
			if (Status == FALSE)
				{
					printf("\n    Error! in Setting WaitCommEvent()");
				}
			else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
				{
					printf("\n\n    Characters Received");
					do
						{
							//printf("\n ReadFile();");
							//Status = ReadFile(hComm, &TempChar[0], sizeof(TempChar), &NoBytesRecieved, NULL);
							Status = ReadFile(hComm, &TempChar[0], 3, &NoBytesRecieved, NULL);
							printf("%d ", NoBytesRecieved);
							//printf("Status -> %d", Status);
							for (int h=0; h< NoBytesRecieved; h++)
								SerialBuffer[i+h] = TempChar[h];
							//printf(" SerialChar -> %c", TempChar);
							//printf(" i =%d", i);
							i+=NoBytesRecieved;
					} while (i < 512);// (NoBytesRecieved > 0);

					printf("\n reached here\n\n\n");

					/*------------Printing the RXed String to Console----------------------*/

					printf("\n\n    ");
					for (int j = 0; j < i - 1; j++)		// j < i-1 to remove the dupliated last character
					{
						if ((j % 40) == 0)
							printf("\n");

						printf("0x%02x, ", (uint8_t)SerialBuffer[j]);
						
					}
		
				}	
		
				CloseHandle(hComm);//Closing the Serial Port
				printf("\n +==========================================+\n");
				_getch();
		}//End of Main()