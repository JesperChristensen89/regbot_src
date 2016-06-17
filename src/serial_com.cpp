/*

*/
#include <string.h>
#include "serial_com.h"
#include "QueueList.h"

// Constants
//#define Serial1 Serial1 // RX1 = Teensy Digital 0, TX1 = Teensy Digital 1

// Variables
static QueueList<String> queue;

// Serial functions
void serialInit(int baud) { Serial1.begin(baud); }
void usbInit() { Serial.begin(115200); } // USB is always 12 Mbit/sec

// USB
// int usbWrite(const char *format, ...)
// {
// 	if (format!= NULL) {
// 		char buffer[1024];
// 		// Saves arguments to va_list and prints to string
// 		va_list arg;
// 		va_start (arg, format);
// 		vsprintf(buffer, format, arg);
// 		va_end (arg);
// 		// Writes string to serial port
// 		// Serial.flush();
// 		return Serial.print(buffer);
// 	}
// 	else return -1; // Error: Empty string.
// }

// void usbClear()
// {
// 	// Clear screen
// 	Serial.write(27);
// 	Serial.print("[2J"); // clear screen
// 	Serial.write(27); 	 // ESC
// 	Serial.print("[H");  // cursor to home
// }

// char *usbRead()
// {
//   char incomingBytes[255];			// Max length of string
//   //char incomingByte;
//   int i = 0;
//   while (Serial.available() > 0) {
//     //incomingBytes.append((char)Serial1.read());		// Returns -1 if nothing available
//     incomingBytes[i] = (char)Serial.read();
//     i++;
//     //incomingByte = (char)Serial1.read();
//     //strcat( incomingBytes, *incomingByte );
//   }
//   incomingBytes[i] = '\0';
//
//   return (char *)incomingBytes;
// }

// Serial
// int serialWrite(const char *format, ...)
// {
// 	if (format!= NULL) {
// 		char buffer[1024];
// 		// Saves arguments to va_list and prints to string
// 		va_list arg;
// 		va_start (arg, format);
// 		vsprintf(buffer, format, arg);
// 		va_end (arg);
// 		// Writes string to serial port
// 		Serial1.flush();
// 		return Serial1.print(buffer);
// 	}
// 	else return -1;
// }

// Serial data to queue
// int serialDataToQueue(const char *format, ...)
// {
// 	if (format!= NULL) {
// 		char buffer[512];
// 		// Saves arguments to va_list and prints to string
// 		va_list arg;
// 		va_start (arg, format);
// 		vsnprintf(buffer, 512, format, arg);
// 		va_end (arg);
// 
// 		// Writes string to serial port
// 		/*for (int i = 0; i < (unsigned)strlen(buffer); i++)
// 		{
// 			queue.push(buffer[i]);
// 		}*/
// 		queue.push(buffer);
// 		return 1;
// 	}
// 	else return -1;
// }

// Serial
// bool serialWriteQueue()
// {
// 	if ( !queue.isEmpty() )
// 	{
// 		//
// 		String buffer = queue.peek();
// 		queue.pop();
// 		// Writes string to serial port
// 		Serial1.flush();
// 		Serial1.print(buffer);
// 		return true;
// 	}
// 	else return false;
// }
