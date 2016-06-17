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

#ifndef REGBOT_MISSION_H
#define REGBOT_MISSION_H

#include <string.h>
#include <stdlib.h>
#include "WProgram.h"

void eePromSaveMission();
void eePromLoadMission();


class UMissionLine
{
private:
  enum MPTYP {MP_VEL='a', 
               MP_ACC, 
               MP_TR, 
               MP_LOG, 
               MP_BAL, 
               MP_LINE_L, 
               MP_LINE_R, 
               MP_LINE_WHITE, 
               MP_LABEL, 
               MP_DRIVE_DIST, 
               MP_IR_SENSOR, 
               MP_IR_DIST,
               MP_GOTO
  };
  enum MPTYC {MC_DIST='A', 
              MC_TIME, 
              MC_TURN, 
              MC_COUNT, 
              MC_XINGB, 
              MC_XINGW, 
              MC_LINE_VALID, 
              MC_IR_DIST1, /// distance sensor 1 limit
              MC_IR_DIST2, /// distance sensor 2 limit
              MC_TILT /// tilt angle
  };
public:
  // drive parameters
  float vel;
  bool velUse;
  float acc;
  bool accUse;
  float log;
  bool logUse;
  bool bal;
  bool balUse;
  float tr;
  bool trUse;
  float lineRef;
  bool lineLUse;
  bool lineRUse;
  bool lineWhiteUse;
  bool lineWhite;
  uint16_t label;
  uint16_t gotoDest;
  bool gotoUse;
  int8_t irSensor;
  int8_t irSensorUse; // 0=false, positive (1 0r 2) use sensor for turning, negative (-1or -2)use sensor for velocity control
  float irDistance; // reference distance for ir sensor, if negative then sign change for controller
  bool irDistanceUse;
  float drivePos;
  bool drivePosUse;
  /// continue conditions
  float dist;
  char distUse;
  float turn;
  char turnUse;
  float time;
  char timeUse;
  char xingWhiteUse;
  char xingBlackUse;
  int8_t xingBlackVal;
  int8_t xingWhiteVal;
  char lineValidUse;
  char lineValidVal;  
  uint16_t count;
  char countUse;
  float irDist1;
  char irDist1Use;
  float irDist2;
  char irDist2Use;
  float tilt;
  char tiltUse;
public:
  bool valid; // no error found
  int visits; // number of times this line has beed executed
  
public:
  /** clear all valid flags */
  void clear();
  /** convert to string - for return message 
   * \param bf is a C char array 
   * \param bfCnt is the count of bytes in bf
   * \param frame adds linefeed if true
   * \returns buffer (bf) filled from class and the used number of bytes */
  int toString(char * bf, int bfCnt, bool frame = true);
  /** decode mission line 
   * \param buffer is expected to hold a mission line in a terminated C string
   * \returns true if loaded with no errors */
  bool decodeLine(const char * buffer);
  /** convert to token string - for save in ee-prom 
   * \param bf is a C char array (destination)
   * \param bfCnt is the count of bytes in bf
   * \returns buffer (bf) filled from class and the used number of bytes */
  int toTokenString(char* bf, int bfCnt);
  /** decode mission line from token string 
   * \param buffer is expected to hold a mission line as tokens in a terminated C string
   * \returns true if loaded with no errors */
  bool decodeToken(const char* buffer);
};

const int mLinesCntMax = 50;
extern UMissionLine mLines[mLinesCntMax];
extern int mLinesCnt;

const int missionErrStrMaxCnt = 50;
extern char missionErrStr[missionErrStrMaxCnt];

#endif