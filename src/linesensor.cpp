/***************************************************************************
 *   Copyright (C) 2015 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Line sensor functions
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

#include "main.h"
#include "control.h"
#include "linesensor.h"
#include "robot.h"
#include "eeconfig.h"

bool lineSensorOn = true;

int16_t adcLSH[8] = {0,111,221,331,441,551,661,771};
int16_t adcLSL[8] = {0,111,222,332,442,552,666,772};
int16_t adcLSD[8] = {600,611,622,633,644,655,666,677};

bool lsIsWhite = false;
int16_t blackLevel[8] = {100, 101, 102, 103, 104, 105, 106, 107};
int16_t whiteLevel[8] = {100, 101, 102, 103, 104, 105, 106, 107};
float lsGain[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float lineSensorValue[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float lsLeftSide = 0.0;
float lsRightSide = 0.0;
bool lsLeftValid = false;
bool lsRightValid = false;
bool crossingWhiteLine = false;
bool crossingBlackLine = false;
int16_t crossingWhiteCnt = 0;
int16_t crossingBlackCnt = 0;
int16_t crossingCntLimit = 20;
bool lsPowerHigh;
bool lsPowerAuto;

void sendLSfind();

//////////////////////////////////////////////

void sendLineSensorPosition()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "lip %d %d %.4f %d %.4f %d %d %d %d %d %d %d %d\r\n" ,
           lineSensorOn, lsIsWhite,
           lsLeftSide, lsLeftValid,
           lsRightSide, lsRightValid, 
           regul_line_followLeft,
           crossingWhiteLine, crossingBlackLine, crossingWhiteCnt, crossingBlackCnt,
           lsPowerHigh, lsPowerAuto
  );
  usb_send_str(reply);
}

//////////////////////////////////////////////

void sendStatusLineSensorLimits()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "liw %d %d %d %d %d %d %d %d\r\n" ,
           whiteLevel[0],
           whiteLevel[1],
           whiteLevel[2],
           whiteLevel[3],
           whiteLevel[4],
           whiteLevel[5],
           whiteLevel[6],
           whiteLevel[7]
  );
  usb_send_str(reply);
  snprintf(reply, MRL, "lib %d %d %d %d %d %d %d %d\r\n" ,
           blackLevel[0],
           blackLevel[1],
           blackLevel[2],
           blackLevel[3],
           blackLevel[4],
           blackLevel[5],
           blackLevel[6],
           blackLevel[7]
  );
  usb_send_str(reply);
  sendLineSensorPosition();
}

//////////////////////////////////////////////

void sendStatusLineSensor()
{
  const int MRL = 150;
  char reply[MRL];
  sendLineSensorPosition();
  //if (useLineSensor)
  {
    snprintf(reply, MRL, "liv %d %d %d %d %d %d %d %d\r\n" ,
            adcLSD[0],
            adcLSD[1],
            adcLSD[2],
            adcLSD[3],
            adcLSD[4],
            adcLSD[5],
            adcLSD[6],
            adcLSD[7]
    );
    usb_send_str(reply);
    // send also position
    sendStatusLineSensorLimits();
  }
}


//////////////////////////////////////////////


void sendADLineSensor()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "#u9 %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d\r\n", //    %d %d %d %d %d %d %d %d\r\n" ,
           adcLSH[0], adcLSL[0],
           adcLSH[1], adcLSL[1],
           adcLSH[2], adcLSL[2],
           adcLSH[3], adcLSL[3],
           adcLSH[4], adcLSL[4],
           adcLSH[5], adcLSL[5],
           adcLSH[6], adcLSL[6],
           adcLSH[7], adcLSL[7] //,
           //adcErr0[3], adcErr0[4], adcErr0[5], adcErr0[6], adcErr0[7], adcErr0[8], adcErr0[9], adcErr0[10] 
  );
  usb_send_str(reply);
//   snprintf(reply, MRL, "#u9 white black gain %d %d %f  %d %d %f  %d %d %f  %d %d %f  %d %d %f  %d %d %f  %d %d %f  %d %d %f\r\n" ,
//            whiteLevel[0], blackLevel[0], lsGain[0],
//            whiteLevel[1], blackLevel[1], lsGain[1],
//            whiteLevel[2], blackLevel[2], lsGain[2],
//            whiteLevel[3], blackLevel[3], lsGain[3],
//            whiteLevel[4], blackLevel[4], lsGain[4],
//            whiteLevel[5], blackLevel[5], lsGain[5],
//            whiteLevel[6], blackLevel[6], lsGain[6],
//            whiteLevel[7], blackLevel[7], lsGain[7]
//   );
//   usb_send_str(reply);
  sendStatusLineSensorLimits();
  sendLSfind();
  snprintf(reply, MRL, "#gain %f %f %f %f %f %f %f %f\r\n" ,
            lsGain[0],
            lsGain[1],
            lsGain[2],
            lsGain[3],
            lsGain[4],
            lsGain[5],
            lsGain[6],
            lsGain[7]
  );
  usb_send_str(reply);
}

//////////////////////////////////////////////

bool setLineSensor(const char * buf)
{
  bool used = false;
  { // is for the line sensor
    if (strncmp(buf, "lip", 3) == 0)
    { // assumed white line
      char * p1 = (char *)&buf[4];
      used = true;
      lineSensorOn = strtol(p1, &p1, 10);
      lsIsWhite = strtol(p1, &p1, 10);
      lsPowerHigh = strtol(p1, &p1, 10);
      lsPowerAuto = strtol(p1, &p1, 10);
      //usb_send_str("# got a lip\n");
    }
    else if (strncmp(buf, "licw", 4) == 0)
    { // calibrate white
      int16_t v =  adcLSH[0] - adcLSL[0];
      used = true;
      if (v < 256)
        usb_send_str("#surface is NOT white!\n");
      else
      {
        for (int i = 0; i < 8; i++)
        {
          whiteLevel[i] = adcLSH[i] - adcLSL[i];
          lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
        }
      }
    } 
    else if (strncmp(buf, "licb", 4) == 0)
    { // calibrate black
      int16_t v =  adcLSH[0] - adcLSL[0];
      used = true;
      if (v > 1024)
        usb_send_str("#surface is NOT black!\n");
      else
      {
        for (int i = 0; i < 8; i++)
        {
          v = adcLSH[i] - adcLSL[i];
          if (v < 0)
            v = 0;
          blackLevel[i] = v;
          lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
        }
      }
    } 
  }
  return used;
}

//////////////////////////////////////////////

float vmin = 1.0, vmax = 0.0;
float vmid, span, vmidFilt = -1.0, spanFilt = -1.0;
int lv = -1;
int rv = -1;


void sendLSfind()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "#u9 min max %.3f %.3f dif %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f, mid %f, span %f, lv,rv %d %d, l: %f, r:%f\r\n" ,
           vmin, vmax,
           lineSensorValue[0], lineSensorValue[1], lineSensorValue[2], lineSensorValue[3], lineSensorValue[4], 
           lineSensorValue[5], lineSensorValue[6], lineSensorValue[7],
           vmid, span, lv, rv, lsLeftSide, lsRightSide
  );
  usb_send_str(reply);
}


void findLineEdge(void)
{
//   float vmin = 1.0, vmax = 0.0;
//   float vdif[8];
//   float vmid, span;
  lsLeftValid = false;
  lsRightValid = false;
  if (lineSensorOn)
  { // LED power control
    if (lsPowerAuto)
      // if LED's are close to surface - positive tilt
      // then turn power down
      lsPowerHigh = pose[3] < -0.15;
    // estimate edge position
    vmin = 1.0;
    vmax = 0.0;
    for (int i = 0; i < 8; i++)
    {
      int16_t d = whiteLevel[i] - blackLevel[i];
      int16_t v = adcLSH[i] - adcLSL[i];
      adcLSD[i] = v;
      v -= blackLevel[i];
      if (v < 0)
        v = 0;
      else if (v > d)
        v = d;
      lineSensorValue[i] = v * lsGain[i];
      if (lineSensorValue[i] > vmax)
        vmax = lineSensorValue[i];
      if (lineSensorValue[i] < vmin)
        vmin = lineSensorValue[i];
    }
    vmid = (vmax + vmin) / 2.0;
    span = vmax - vmin;
    if (vmidFilt < 0.0)
    { // uninitialized
      vmidFilt = vmid;
      spanFilt = span;
    }
    else
    { // find crossing white line
      vmidFilt = (vmidFilt * 500.0 + vmid) / 501.0;
      spanFilt = (spanFilt * 500.0 + span) / 501.0;
    }
    // are levels sufficient for estimate
    if (span > 0.05)
    { // there is potentially a line edge
      lv = -1;
      rv = -1;
      for (int i = 0; i < 8; i++)
      {
        if (lsIsWhite)
        { // looking for white line
          if (lineSensorValue[i] > vmid and lv < 0)
            lv = i;
          if (lineSensorValue[7 - i] > vmid and rv < 0)
            rv = 7 - i;
        }
        else
        { // looking for black line
          if (lineSensorValue[i] < vmid and lv < 0)
            lv = i;
          if (lineSensorValue[7 - i] < vmid and rv < 0)
            rv = 7 - i;
        } 
        if (lv >= 0 and rv >= 0)
          break;
      }
      if (lv < 1)
        lv = 1;
      if (rv > 6)
        rv = 6;
      // make a small interpolation
      // left side
      float h = lineSensorValue[lv] - lineSensorValue[lv - 1];
      float e1 = -10.0, e2 = 10.0;
      bool e1v, e2v;
      if ((h > 0.001 and lsIsWhite) or (h < -0.001 and not lsIsWhite))
        e1 = lv + (vmid - lineSensorValue[lv])/ h;
      if (e1 < -2.0)
      {
        e1 = -2.0;
        e1v = false;
      }
      else if (e1 > 9.0)
      {
        e1 = 9.0;
        e1v = false;
      }
      else
        e1v = true;
      // right side
      h = lineSensorValue[rv] - lineSensorValue[rv + 1];
      if ((h > 0.001 and lsIsWhite) or (h < -0.001 and not lsIsWhite))
        e2 = rv + (lineSensorValue[rv] - vmid)/ h;
      if (e2 < - 2.0)
      {
        e2 = -2;
        e2v = false;
      }
      else if (e2 > 9.0)
      {
        e2 = 9.0;
        e2v = false;
      }
      else
        e2v = true;
      // assign to global values in meters
      lsLeftSide = (e1 - 3.5) * 0.76;
      lsLeftValid = e1v;
      lsRightSide = (e2 - 3.5) * 0.76;
      lsRightValid = e2v;
      // not crossing a line if a line is detected
      // so decrease count
      if (crossingWhiteCnt > 0)
        crossingWhiteCnt--;
      else
        crossingWhiteLine = false;
      if (crossingBlackCnt > 0)
        crossingBlackCnt--;
      else
        crossingBlackLine = false;
    }
    else
    { // crossing lines test when no line is detected
      // if mid value increases and span is low, or
      // if span decreases, then 
      // assumed crossing line - white or black
      if (vmid > 0.7)
      {
        if (crossingWhiteCnt < crossingCntLimit)
          crossingWhiteCnt ++;
        else
          crossingWhiteLine = true;
        if (crossingBlackCnt > 0)
          crossingBlackCnt--;
        else
          crossingBlackLine = false;
      }
      else if (vmid < 0.05)
      {
        if (crossingWhiteCnt > 0)
          crossingWhiteCnt--;
        else
          crossingWhiteLine = false;
        if (crossingBlackCnt < crossingCntLimit)
          crossingBlackCnt++;
        else
          crossingBlackLine = true;
      }
      else if ((span < (spanFilt * 0.5) and (spanFilt > 0.04)) or (span < 0.07 and abs(vmid - vmidFilt) > 0.05))
      { // span is fast going down - line stopped
        if ((vmid - vmidFilt) > (spanFilt / 8))
        { // area is more bright
          if (crossingWhiteCnt < crossingCntLimit)
            crossingWhiteCnt ++;
          else
            crossingWhiteLine = true;
          if (crossingBlackCnt > 0)
            crossingBlackCnt--;
          else
            crossingBlackLine = false;
        }
        else if ((vmid - vmidFilt) < (-spanFilt / 8))
        { // area is darker now
          if (crossingWhiteCnt > 0)
            crossingWhiteCnt--;
          else
            crossingWhiteLine = false;
          if (crossingBlackCnt < crossingCntLimit)
            crossingBlackCnt++;
          else
            crossingBlackLine = true;
        }
        else
        {
          if (crossingWhiteCnt > 0)
            crossingWhiteCnt--;
          else
            crossingWhiteLine = false;
          if (crossingBlackCnt > 0)
            crossingBlackCnt--;
          else
            crossingBlackLine = false;
        }
      }
      else 
      { // not crossing a line (any-more)
        if (crossingWhiteCnt > 0)
          crossingWhiteCnt--;
        else
          crossingWhiteLine = false;
        if (crossingBlackCnt > 0)
          crossingBlackCnt--;
        else
          crossingBlackLine = false;
      }
    }
  }
  else
  { // LEDs not on, so no lines
    crossingBlackLine = false;
    crossingWhiteLine = false;
    crossingBlackCnt = 0;
    crossingWhiteCnt = 0;
  }
}

/////////////////////////////////////////////////////

void eePromSaveLinesensor()
{
  char v = 0x00;
  if (lineSensorOn)
    v |= 0x01;
  if (lsIsWhite)
    v |= 0x02;
  if (lsPowerHigh)
    v |= 0x04;
  if (lsPowerAuto)
    v |= 0x08;
  eeConfig.pushByte(v);
  //eeprom_write_byte((uint8_t*)eePushAdr++, v);
  // debug
//   usb_send_str("# eesave line sensor ");
//   const int MSL = 40;
//   char s[MSL];
  // debug end
  for (int i = 0; i < 8; i++)
  {
    eeConfig.pushWord(blackLevel[i]);
    eeConfig.pushWord(whiteLevel[i]);
    //     eeprom_write_word((uint16_t*)eePushAdr, blackLevel[i]);
//     eePushAdr += 2;
//     eeprom_write_word((uint16_t*)eePushAdr, whiteLevel[i]);
//     eePushAdr += 2;
    // debug
//     snprintf(s, MSL, "%d %d ", whiteLevel[i], blackLevel[i]);
//     usb_send_str(s);
    // debug end
  }
  // debug
//   usb_send_str("\r\n");
  // debug end
}

/////////////////////////////////////////////////////

void eePromLoadLinesensor()
{
  char v = eeConfig.readByte();
//  char v = eeprom_read_byte((uint8_t*)eePushAdr++);
  lineSensorOn = (v & 0x01) == 0x01;
  lsIsWhite = (v & 0x02) == 0x02;
  lsPowerHigh = (v & 0x04) == 0x04;
  lsPowerAuto = (v & 0x08) == 0x08;
  // debug
//   usb_send_str("# eeload line sensor ");
//   const int MSL = 40;
//   char s[MSL];
  // debug end
  for (int i = 0; i < 8; i++)
  {
    blackLevel[i] = eeConfig.readWord();
    whiteLevel[i] = eeConfig.readWord();
//     blackLevel[i] = eeprom_read_word((uint16_t*)eePushAdr);
//     eePushAdr += 2;
//     whiteLevel[i] = eeprom_read_word((uint16_t*)eePushAdr);
//     eePushAdr += 2;
    // debug
//     snprintf(s, MSL, "%d %d ", whiteLevel[i], blackLevel[i]);
//     usb_send_str(s);
    // debug end
  }
  // debug
//   usb_send_str("\r\n");
  // debug end
  for (int i = 0; i < 8; i++)
  { // set gains from new values
    lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
  }
}
