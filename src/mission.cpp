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
#include <string.h>
#include <stdio.h>
#include "math.h"
#include "mission.h"
#include "serial_com.h"
#include "main.h"
#include "eeconfig.h"

UMissionLine mLines[mLinesCntMax];
int mLinesCnt = 0;

char missionErrStr[missionErrStrMaxCnt];


void UMissionLine::clear()
{
  accUse = false;
  logUse = false;
  trUse = false;
  velUse = false;
  lineLUse = false;
  lineRUse = false;
  lineWhiteUse = false;
  balUse = false;
  gotoUse = false;
  drivePosUse = false;
  irSensorUse = false;
  irDistanceUse = false;
  label = 0;
  //
  distUse = false;
  timeUse = false;
  turnUse = false;
  countUse = false;
  xingWhiteUse = false;
  xingBlackUse = false;
  lineValidUse = false;
  irDist1Use = '\0';
  irDist2Use = '\0';
  tiltUse = false;
  // 
  valid = false;
  visits = 0;
}


int UMissionLine::toString(char* bf, int bfCnt, bool frame)
{
  char * ms = bf;
  char * mc;
  int n = 0;
  const char sep = ',';
  if (not valid)
  {
    strncpy(ms, "# not valid", bfCnt - n);
    n += strlen(ms);
    ms = &bf[n];
  }
  else if (frame)
  {
    strcpy(ms, "<m ");
    n+=3;
    ms = &bf[n];
  }
  if (velUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "vel=%g", vel);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (accUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "acc=%g", acc);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (trUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "tr=%g", tr);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (lineLUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "lineL=%g", lineRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (lineRUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "lineR=%g", lineRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (lineWhiteUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "white=%d", lineWhite);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (logUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "log=%g", log);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (balUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "bal=%d", bal);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irSensorUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irsensor=%d", irSensor);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irDistanceUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irdist=%g", irDistance);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (drivePosUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "drpos=%g", drivePos);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (label > 0)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "label=%d", label);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (gotoUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "goto=%d", gotoDest);
    n += strlen(ms);
    ms = &bf[n];
  }
  // now the condition part - if it exist
  mc = ms;
  *mc++ = ':';
  n++;
  if (distUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "dist%c%g", distUse, dist);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (timeUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "time%c%g", timeUse, time);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (turnUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "turn%c%g", turnUse, turn);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (countUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "count%c%d", countUse, count);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (xingBlackUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "xb%c%d", xingBlackUse, xingBlackVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (xingWhiteUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "xw%c%d", xingWhiteUse, xingWhiteVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (lineValidUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "lv%c%d", lineValidUse, lineValidVal);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (irDist1Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir1%c%g", irDist1Use, irDist1);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (irDist2Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir2%c%g", irDist2Use, irDist2);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (tiltUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "tilt%c%g", tiltUse, tilt);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (frame)
  {
    strncpy(mc, "\n\r", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}

/////////////////////////////////////////////////

int UMissionLine::toTokenString(char* bf, int bfCnt)
{
  char * ms = bf;
  char * mc;
  int n = 0;
  const char sep = ',';
  if (not valid)
  {
    ms[0] = '\0';
  }
  else
  {
    if (velUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n, "%c%g", MP_VEL, vel);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (accUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n, "%c%g", MP_ACC, acc);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (trUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_TR, tr);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (lineWhiteUse)
    {
      if (n > 3) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_LINE_WHITE, lineWhite);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (lineLUse)
    {
      if (n > 3) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_LINE_L, lineRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (lineRUse)
    {
      if (n > 3) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_LINE_R, lineRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (logUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_LOG, log);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (balUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_BAL, bal);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irSensorUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_IR_SENSOR, irSensor);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irDistanceUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_IR_DIST, irDistance);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (drivePosUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_DRIVE_DIST, drivePos);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (label > 0)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_LABEL, label);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (gotoUse)
    {
      if (n > 0) {*ms++=sep; n++;}
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_GOTO, gotoDest);
      n += strlen(ms);
      ms = &bf[n];
    }
    // not the condition part - if it exist
    mc = ms;
    *mc++ = ':';
    n++;
    if (distUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_DIST, distUse, dist);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (timeUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TIME, timeUse, time);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (turnUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TURN, turnUse, turn);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (countUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_COUNT, countUse, count);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (xingBlackUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XINGB, xingBlackUse, xingBlackVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (xingWhiteUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XINGW, xingWhiteUse, xingWhiteVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (lineValidUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_LINE_VALID, lineValidUse, lineValidVal);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (irDist1Use)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST1, irDist1Use, irDist1);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (irDist2Use)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST2, irDist2Use, irDist2);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (tiltUse)
    {
      if (mc - ms > 1) {*mc++=sep; n++;}
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TILT, tiltUse, tilt);
      n += strlen(mc);
      mc = &bf[n];
    }
    strncpy(mc, "\n", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}

bool UMissionLine::decodeLine(const char* buffer)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  char * p3 = strchr(p1, '=');
  bool err = false;
  // reset use flags
  clear();
  // strip white space
  while (*p1 <= ' ' and *p1 > '\0') p1++;
  // find all parameters until ':'
  while ((p1 < p2 or p2 == NULL) and p3 != NULL)
  {
    if (strncmp (p1, "acc", 3) == 0)
    {
      accUse = true;
      acc = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "vel", 3) == 0)
    {
      velUse = true;
      vel = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "tr", 2) == 0)
    {
      trUse = true;
      tr = fabsf(strtof(++p3, &p1));
    }
    else if (strncmp (p1, "linel", 5) == 0)
    {
      lineLUse = true;
      lineRef = strtof(++p3, &p1);
      if (lineRef > 2.0)
        lineRef = 2.0;
      if (lineRef < -2.0)
        lineRef = -2.0;
    }
    else if (strncmp (p1, "liner", 5) == 0)
    {
      lineRUse = true;
      lineRef = strtof(++p3, &p1);
      if (lineRef > 2.0)
        lineRef = 2.0;
      if (lineRef < -2.0)
        lineRef = -2.0;
    }
    else if (strncmp (p1, "white", 5) == 0)
    {
      lineWhiteUse = true;
      lineWhite = strtol(++p3, &p1, 10);
      if (lineWhite)
        lineWhite = true;
    }
    else if (strncmp (p1, "log", 3) == 0)
    {
      logUse = true;
      log = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "bal", 3) == 0)
    {
      balUse = true;
      bal = strtof(++p3, &p1) > 0.5;
    }
    else if (strncmp (p1, "irsensor", 8) == 0)
    {
      irSensorUse = true;
      irSensor = strtol(++p3, &p1, 10);
      if (irSensor > 2)
        irSensor = 2;
      else if (irSensor < 1)
        irSensor = 1;
    }
    else if (strncmp (p1, "irdist", 6) == 0)
    {
      irDistanceUse = true;
      irDistance = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "drpos", 5) == 0)
    {
      drivePosUse = true;
      drivePos = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "label", 5) == 0)
    {
      label = strtol(++p3, &p1, 10);
    }
    else if (strncmp (p1, "goto", 3) == 0)
    {
      gotoUse = true;
      gotoDest = strtol(++p3, &p1, 0);
    }
    else
    { // error, just skip
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed parameter at %s", p1);
      p1 = ++p3;
      err = true;
    }
    // remove white space
    while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
    p3 = strchr(p1, '=');
    
  }
  if (p2 != NULL)
  {
    p1 = p2 + 1;
    while (*p1 <= ' ' and *p1 > '\0') p1++;
    p3 = strchr(p1, '=');
    if (p3 == NULL)
      p3 = strchr(p1, '<');
    if (p3 == NULL)
      p3 = strchr(p1, '>');    
    while (*p1 > '\0' and p3 != NULL)
    {
      if (strncmp (p1, "dist", 4) == 0)
      { // distance is always positive (even if reversing)
        distUse = *p3;
        dist = fabsf(strtof(++p3, &p1));
      }
      else if (strncmp (p1, "turn", 4) == 0)
      {
        turnUse = *p3;
        turn = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "time", 4) == 0)
      {
        timeUse = *p3;
        time = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "count", 4) == 0)
      {
        countUse = *p3;
        count = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "xb", 2) == 0)
      {
        xingBlackUse = *p3;
        xingBlackVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "xw", 2) == 0)
      {
        xingWhiteUse = *p3;
        xingWhiteVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "lv", 2) == 0)
      {
        lineValidUse = *p3;
        lineValidVal = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "ir1", 3) == 0)
      {
        irDist1Use = *p3;
        irDist1 = strtof(++p3, &p1);
        //usb_send_str("#ir1\n");
      }
      else if (strncmp (p1, "ir2", 3) == 0)
      {
        irDist2Use = *p3;
        irDist2 = strtof(++p3, &p1);
        //usb_send_str("#ir2\n");
      }
      else if (strncmp (p1, "tilt", 4) == 0)
      {
        tiltUse = *p3;
        tilt = strtof(++p3, &p1);
      }
      else
      { // error, just skip
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed condition at %s", p1);
        p1 = ++p3;
        err = true;
      }
      // remove white space
      while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
      p3 = strchr(p1, '=');
      if (p3 == NULL)
        p3 = strchr(p1, '<');
      if (p3 == NULL)
        p3 = strchr(p1, '>');    
    }
  }
  valid = not err;
  
//   if (true)
//   {
//     const int MSL = 180;
//     char s[MSL];
//     snprintf(s, MSL, "# decoded ir1Use=%c, ir2use=%c from %s\n", irDist1Use, irDist2Use, buffer);
//     usb_send_str(s);
//   }
  
  return err;
}

bool UMissionLine::decodeToken(const char* buffer)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  bool err = false;
  // reset use flags
  clear();
//   const int MSL = 80;
//   char  s[MSL];
//   missionErrStr[0]='\0';
  // find all parameters until ':'
  while (p1 < p2)
  {
//     snprintf(s, MSL, "#pi1=%s\n", p1);
//     usb_send_str(s);
    if (*p1 == MP_ACC) 
    {
      accUse = true;
      acc = strtof(++p1, &p1);
    }
    else if (*p1 == MP_VEL)
    {
      velUse = true;
      vel = strtof(++p1, &p1);
    }
    else if (*p1 == MP_TR)
    {
      trUse = true;
      tr = fabsf(strtof(++p1, &p1));
    }
    else if (*p1 == MP_LINE_L)
    {
      lineLUse = true;
      lineRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_LINE_R)
    {
      lineRUse = true;
      lineRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_LINE_WHITE)
    {
      lineWhiteUse = true;
      lineWhite = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_LOG)
    {
      logUse = true;
      log = strtof(++p1, &p1);
    }
    else if (*p1 == MP_BAL)
    {
      balUse = true;
      bal = strtof(++p1, &p1) > 0.5;
    }
    else if (*p1 == MP_IR_SENSOR)
    {
      irSensorUse = true;
      irSensor = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_IR_DIST)
    {
      irDistanceUse = true;
      irDistance = strtof(++p1, &p1);
    }
    else if (*p1 == MP_DRIVE_DIST)
    {
      drivePosUse = true;
      drivePos = strtof(++p1, &p1);
    }
    else if (*p1 == MP_LABEL)
    {
      label = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_GOTO)
    {
      gotoUse = true;
      gotoDest = strtol(++p1, &p1, 10);
    }
    else if (*p1 > ' ')
    { // error, just skip
      err = true;
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed line P at %s\n", p1);
    }
    // remove seperator
    while (*p1 != ',' and *p1 > '\0' and *p1 != ':') 
      p1++;
    if (p1 > '\0')
      p1++;
  }
  if (p2 != NULL)
  {
    p1 = p2 + 1;
    while (*p1 > ' ')
    {
//       snprintf(s, MSL, "#pi2=%s\n", p1);
//       usb_send_str(s);
      if (*p1 == MC_DIST)
      { // distance is always positive (even if reversing)
        distUse = *(++p1);
        dist = fabsf(strtof(++p1, &p1));
      }
      else if (*p1 == MC_TURN)
      {
        turnUse = *(++p1);
        turn = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TIME)
      {
        timeUse = *(++p1);
        time = strtof(++p1, &p1);
      }
      else if (*p1 == MC_COUNT)
      {
        countUse = *(++p1);;
        count = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_XINGB)
      {
        xingBlackUse = *(++p1);
        xingBlackVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_XINGW)
      {
        xingWhiteUse = *(++p1);
        xingWhiteVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_LINE_VALID)
      {
        lineValidUse = *(++p1);
        lineValidVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_IR_DIST1)
      {
        irDist1Use = *(++p1);
        irDist1 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_IR_DIST2)
      {
        irDist2Use = *(++p1);
        irDist2 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TILT)
      {
        tiltUse = *(++p1);
        tilt = strtof(++p1, &p1);
      }
      else
      { // error, just skip
        err = true;
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed line C at %s\n", p1);
      }
      // remove seperators
      while (*p1 != ',' and *p1 != '\n' and *p1 != ':') 
        p1++;
      if (*p1 > ' ')
        p1++;
    }
  }
  valid = not err;
  return err;
}


//////////////////////////////////////////////////

void eePromSaveMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 100;
//   char s[MSL];
  int n;
  uint32_t adr = eeConfig.getAddr();
  //uint32_t adr = eePushAdr;
  // reserve space for size
  eeConfig.setAddr(adr + 2);
  //eePushAdr += 2;
  for (int i = 0; i < mLinesCnt; i++)
  {
    //usb_write("# line\n");
    n = mLines[i].toTokenString(reply, MRL);
    //if (eePushAdr + n < 2048 - 2)
    if (eeConfig.getAddr() + n < 2048 - 2)
    {
      eeConfig.busy_wait();
      eeConfig.write_block(reply, n);
//       eeConfig.write_block(reply, (void*)eePushAdr, n);
//       eePushAdr += n;
    }
    else
    {
      snprintf(reply, MRL, "# failed to save mission from line %d (of %d)\n", i, mLinesCnt);
      usb_send_str(reply);
      break;
    }
    // debug
    // remove line feed
//     reply[n-1] = '\0';
//     snprintf(s, MSL, "line %d saved '%s'\r\n", i, reply);
//     usb_send_str(s);
    // debug end
  }
  // write size of missin in characters
  eeConfig.write_word(adr, eeConfig.getAddr() - adr);
  //   eeprom_write_byte((uint8_t*)eePushAdr++, '\n');
//   eeprom_write_byte((uint8_t*)eePushAdr++, '\0');
//   eeprom_write_byte((uint8_t*)eePushAdr++, '\0');
}

////////////////////////////////////

void eePromLoadMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 100;
//   char s[MSL];
  int n = 0, m;
  mLinesCnt = 0;
  bool isOK;
  // read number of bytes in mission
  m = eeConfig.readWord() - 1;
//  m = eeprom_read_word((uint16_t*)eePushAdr) - 1;
//  eePushAdr += 2;
  while (m > 1 and n < MRL)
  {
    snprintf(missionErrStr, 10, "#none\n");
    reply[n] = eeConfig.readByte();
    //reply[n] = eeprom_read_byte((const uint8_t *)eePushAdr++);
    m--;
    if (reply[n] <= ' ')
    {
      if (n > 2)
      { // this is a line
        //usb_send_str(reply);
        isOK = not mLines[mLinesCnt].decodeToken(reply);
        //         snprintf(reply, MRL, "Line %d valid %d\n", mLinesCnt, not mLines[mLinesCnt].valid);
        //         usb_send_str(reply);
        //         reply[0] = '#';
        //         mLines[mLinesCnt].toString(&reply[1], MRL - 1);
        //         usb_send_str(reply);
        if (isOK)
          mLinesCnt++;
        else
        {
          //usb_send_str(missionErrStr);
        }
        // debug
        // remove line feed
//         reply[n] = '\0';
//         snprintf(s, MSL, "line %d loaded '%s' OK=%d\r\n", mLinesCnt, reply, isOK);
//         usb_send_str(s);
        // debug end
        
      }
      n = 0;
    }
    else
      n++;
  }
}
