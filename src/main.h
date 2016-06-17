/***************************************************************************
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
 *   has an IMU and a dual motor controller with current feedback.
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H

#define REGBOT

#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>

// control period is time between control calls
// and is in units of 10us, ie 125 is 1.25ms or 800Hz
#define CONTROL_PERIOD_10us 100
#define SAMPLETIME  (0.00001 * CONTROL_PERIOD_10us)

extern int robotId; // robot number [1..999]
extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt;
extern bool missionStart; // start from Gui
extern bool missionStop;  // stop from Gui
extern float time; // mission time
extern bool localEcho;
extern bool button; // start switch 
extern bool motorPreEnabled;
extern int usb_not_idle_cnt;
extern bool sendStatusWhileRunning;
extern uint16_t controlUsedTime[2];
// ADC setup
const bool useADCInterrupt = true;
const int useADCresolution = 12;
const float useADCMaxADCValue = 4096*2;
// AD conversion
const int MAX_ADC = 13;
/// battery voltage measurement
extern uint16_t batVoltInt;
/// conversion from battery voltage integer to floating point in Volts
static const float batVoltIntToFloat = 1.2 / useADCMaxADCValue * (15.0 + 1.2)/1.2;

uint16_t getRevisionNumber();

// holds distance to both gmk and to leader robot when needed
extern float distanceToGmk;
// holds angle to gmk and to leader robot
extern float angleToGmk;
// flag to sign ready between follower and raspi
extern int rdyToProceed;
// not used
extern int tooClose;
// sets the followers speed
extern float followVelocity;
// sets the turn radius of leader
extern float raspiTurnRadius;
// flag to signal if wifi is ready to receive commands
extern bool isWifiReady;
// is wifi setup
extern bool wifiIsSet;
// holds the override distance for follower
extern double overrideDistance;
// holds the override angle for follower
extern double overrideAngle;

/**
 * Send string to USB channel */
inline void usb_send_str(const char * str)
{ 
  //if (usb_idle())
  // if no space, then do not send (usb serial is of low priority)
  usb_serial_write(str, strlen(str));
}

/**
 * send string to regbot (serial1)
 * @param str
 */
inline void serial1_send_str(const char *  str)
{
    Serial1.write(str);
}

#endif