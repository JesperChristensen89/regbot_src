//#include <stdio.h>
//#include <math.h>
//#include <string.h>
#include "WProgram.h"

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

// Initialization
void serialInit(int baud);
void usbInit();

/**
 * convinient way to send debug strings to terminal */
inline void usb_write(const char * s)
{
  usb_serial_write(s, strlen(s));
}

#endif // SERIAL_COM_H