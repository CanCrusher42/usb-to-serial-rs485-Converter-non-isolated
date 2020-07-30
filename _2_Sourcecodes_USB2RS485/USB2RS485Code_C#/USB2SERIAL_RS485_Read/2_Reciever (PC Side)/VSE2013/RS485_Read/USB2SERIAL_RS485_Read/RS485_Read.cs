
//====================================================================================================//
//   Program to receive the data send from the microcontroller (MSP430G2553) through RS485 network    //
//====================================================================================================//


//====================================================================================================//
// Program opens a  connection to USB2SERIAL board and configures the RS485 chip in Receive mode.     //
// The code then waits for the data send by the microcontroller board.When data is rxed,its displayed //
// on the console.                                                                                    //
//----------------------------------------------------------------------------------------------------//
// BaudRate     -> 9600                                                                               //
// Data formt   -> 8 databits,No parity,1 Stop bit (8N1)                                              //
// Flow Control -> None                                                                               //
//----------------------------------------------------------------------------------------------------//
// www.xanthium.in										                                              //
// Copyright (C) 2015 Rahul.S                                                                         //
//====================================================================================================//


//====================================================================================================//
// Compiler/IDE  :	Microsoft Visual Studio Express 2013 for Windows Desktop(Version 12.0)            //
//               :  SharpDevelop                                                                      //
//                                                                                                    //
// Library       :  .NET Framework                                                                    //
// Commands      :                                                                                    //
// OS            :	Windows(Windows 7)                                                                //
// Programmer    :	Rahul.S                                                                           //
// Date	         :	30-March-2015                                                                     //
//====================================================================================================//

//====================================================================================================//
// Sellecting the COM port Number                                                                     //
//----------------------------------------------------------------------------------------------------//
// Use "Device Manager" in Windows to find out the COM Port number allotted to USB2SERIAL converter-  // 
// -in your Computer and substitute in the  MyCOMPort.PortName                                        //
//                                                                                                    //
// for eg:-                                                                                           //
// If your COM port number is COM32 in device manager(will change according to system)                //
// then                                                                                               //
//			MyCOMPort.PortName = "COM32"                                                              //
//====================================================================================================//

using System;
using System.IO.Ports;//namespace containing SerialPort class

namespace USB2SERIAL_RS485_Read
{
    class RS485_Read
    {
        static void Main(string[] args)
        {
            string COMPortName;                  // Variable for Holding COM port number ,eg:- COM23
            string RxedData;                     // Variable for Holding received data

            Menu();                              // Used for displaying the banner

            Console.Write("\t  Enter COM Port Number(eg :- COM32) ->");
            COMPortName = Console.ReadLine();     //Store COM number in COMPortName

            COMPortName = COMPortName.Trim();     // Remove any trailing whitespaces
            COMPortName = COMPortName.ToUpper();  // Convert the string to upper case

            SerialPort COMPort = new SerialPort();// Create a SerialPort Object called COMPort

            COMPort.PortName = COMPortName;       // Assign the COM port number
            COMPort.BaudRate = 4800;              // Set Baud rate = 9600
            COMPort.DataBits = 8;                 // Number of data bits = 8
            COMPort.Parity   = Parity.None;       // No parity
            COMPort.StopBits = StopBits.One;      // One stop bit

            Console.WriteLine();
            Console.WriteLine("\t  {0} Selected  \n",COMPortName);
            Console.WriteLine("\t  Baud rate = {0}",COMPort.BaudRate);
            Console.WriteLine("\t  Data Bits = {0}",COMPort.DataBits);
            Console.WriteLine("\t  Parity    = {0}",COMPort.Parity);
            Console.WriteLine("\t  Stop Bits = {0}",COMPort.StopBits);

            COMPort.Open();                       // Open the serial port
            Console.WriteLine("\n\t  {0} opened \n", COMPortName);

            COMPort.RtsEnable = true;            // Since RTS = 1, ~RTS = 0 So ~RE = 0 Receive  Mode enabled
            COMPort.DtrEnable = true;            // Since DTR = 1. ~DTR = 0 So  DE = 0 
                                                 //~RE and DE LED's on USB2SERIAL board will be off

            Console.WriteLine("\t  RTS = 1 so ~RTS = 0, ~RE = 0 Receive  Mode enabled");
            Console.WriteLine("\t  DTR = 1 so ~DTR = 0,  DE = 0 ");
           
            RxedData = COMPort.ReadLine();       // Wait for data reception

            Console.WriteLine("\n\t  Data Received ");
            Console.WriteLine("\n\t \" {0} \" ", RxedData); 

            Menu_End();
            Console.Read();                       // Press to Exit
        }//end of Main

        static void Menu()
        {
            Console.Clear();//Clear the console Window 
            Console.WriteLine();
            Console.WriteLine("\t+---------------------------------------------------+");
            Console.WriteLine("\t|              USB2SERIAL RS485 Read                |");
            Console.WriteLine("\t|               (c) www.xanthium.in                 |");
            Console.WriteLine("\t+---------------------------------------------------+");
        }//End of Menu()

        static void Menu_End()
        {
            Console.WriteLine("\n\t+---------------------------------------------------+");
            Console.WriteLine("\t|            Press Any Key to Exit                  |");
            Console.WriteLine("\t+---------------------------------------------------+");
        }//End of Menu_End()

    }//end of Class RS485_Read
}//end of namespace USB2SERIAL_RS485_Read
