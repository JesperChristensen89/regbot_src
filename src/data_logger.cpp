/***************************************************************************
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Data logger for small regulation control object (regbot)
 *   intended for 31300 Linear control
 *   datalogger is intended to store recorded logfile in ram
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

#include "data_logger.h"
#include "motor_controller.h"
#include "serial_com.h"
#include "mpu9150.h"
#include "control.h"
#include "robot.h"
#include "main.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"

// rx from datalogger
// #define LOG_RXBUFFER_MAX 50
// char rxbuffer[LOG_RXBUFFER_MAX];
// int rxbufferCnt = 0;
// int rxCharCnt = 0;
//
int logRowSize = 1;
int logRowCnt;
int logRowsCntMax;
#define LOG_FMT_MAX 40
char logRowFlags[LOG_FMT_MAX];
uint16_t logRowPos[LOG_FMT_MAX];
uint8_t logRowItemSize[LOG_MAX_CNT*2] ;
int logTimeFactor = 1000;
int logInterval = 1;
bool logAllow = true;
bool toLog = false;
int col = 0;
int8_t logBuffer[LOG_BUFFER_MAX];
int debug=77;
/**
 * Start logging with current log flags.
 * \param logInterval 0: unchanged, else set to this number of milliseconds
 * */
void startLogging(int loginterval)
{
  if (loginterval > 0)
    logInterval = loginterval;
  // initialize log - if not already
  if (not toLog)
  { // logger not active, so start
    initLogStructure(100000 / CONTROL_PERIOD_10us);
    // start logging
    toLog = true;
  }
}

void stopLogging(void)
{
  toLog = false;
}

//////////////////////////////////////////

void stateToLog()
{ 
  //   const int MSL = 100;
  //   char s[MSL];
  // //  snprintf(s, MSL, "# state to log, flags, time=%d, mission=%d,...\n", logRowFlags[LOG_TIME], logRowFlags[LOG_MISSION]);
  //   snprintf(s, MSL, "#time %.3f mission %d, state %d.%d, logger line %d/%d\r\n", 
  //            time, mission, missionState, misLineNum, logRowCnt, logRowsCntMax);
  //   usb_send_str(s);
  if (logAllow)
  {
    if (addNewRowToLog())
    {
      addToLog(LOG_TIME, &time, sizeof(time));
      if (logRowFlags[LOG_MISSION])
      {
        if (mission == 3)
          // log line number not state
          addToLog(LOG_MISSION, &misLineNum, sizeof(missionState));
        else
          addToLog(LOG_MISSION, &missionState, sizeof(missionState));
      }
      if (logRowFlags[LOG_ACC])
        addToLog(LOG_ACC, imuAcc, sizeof(imuAcc));
      if (logRowFlags[LOG_GYRO])
        addToLog(LOG_GYRO, imuGyro, sizeof(imuGyro));
      //       if (logRowFlags[LOG_MAG])
      //         addToLog(LOG_MAG, imuMag, sizeof(imuMag));
      if (logRowFlags[LOG_MOTV_REF])
        addToLog(LOG_MOTV_REF, regul_vel_tot_ref, sizeof(regul_vel_tot_ref));
      if (logRowFlags[LOG_MOTV])
      {
        int16_t mv[2];
        mv[0] = int(motorAnkerVoltage[0] * 1000);
        mv[1] = int(motorAnkerVoltage[1] * 1000);
        addToLog(LOG_MOTV, mv, sizeof(mv));
      }
      if (logRowFlags[LOG_ENC])
        addToLog(LOG_ENC, encoder, sizeof(encoder));
      if (logRowFlags[LOG_MOTA])
      { // need to sort out sign first
        int16_t mc[2];
        //      int16_t mc[4];
        //       if (directionFWD[0])
        //         mc[0] = motorCurrent[0];
        //       else
        //         mc[0] = -motorCurrent[0];
        //       if (directionFWD[1])
        //         mc[1] = motorCurrent[1];
        //       else
        //         mc[1] = -motorCurrent[1];
        mc[0] = motorCurrentM[0];
        mc[1] = motorCurrentM[1];
        // save signed current
        addToLog(LOG_MOTA, mc, sizeof(mc));
      }
      if (logRowFlags[LOG_WHEELVEL])
        addToLog(LOG_WHEELVEL, wheelVelocityEst, sizeof(wheelVelocityEst));
      if (logRowFlags[LOG_TURNRATE])
        addToLog(LOG_TURNRATE, &turnRate, sizeof(turnRate)); 
      if (logRowFlags[LOG_POSE])
        addToLog(LOG_POSE, pose, sizeof(pose));
      if (logRowFlags[LOG_LINE])
      { // pack to 16 bit
        int16_t ldv[15];
        float * fdv = (float*)&ldv[8];
        ldv[0] = adcLSH[0] - adcLSL[0];
        ldv[1] = adcLSH[1] - adcLSL[1];
        ldv[2] = adcLSH[2] - adcLSL[2];
        ldv[3] = adcLSH[3] - adcLSL[3];
        ldv[4] = adcLSH[4] - adcLSL[4];
        ldv[5] = adcLSH[5] - adcLSL[5];
        ldv[6] = adcLSH[6] - adcLSL[6];
        ldv[7] = adcLSH[7] - adcLSL[7];
        fdv[0] = lsLeftSide;  // word  8 and  9 (32 bit float)
        fdv[1] = lsRightSide; // word 10 and 11 (32 bit float)
        ldv[12] = 0;
        if (lineSensorOn)
          ldv[12] = 1;
        if (lsIsWhite)
          ldv[12] |= 2;
        if (lsLeftValid)
          ldv[12] |= 4;
        if (lsRightValid)
          ldv[12] |= 8;
        if (lsPowerHigh)
          ldv[12] |= 0x10;
        if (lsPowerAuto)
          ldv[12] |= 0x20;
        if (crossingBlackLine)
          ldv[12] |= 0x40;
        if (crossingWhiteLine)
          ldv[12] |= 0x80;
        ldv[13] = crossingBlackCnt;
        ldv[14] = crossingWhiteCnt;
 /*       if (crossingWhiteCnt > 0)
        {
          const int MSL = 100;
          char s[MSL];
          snprintf(s, MSL, "# xing white log %g %d %d\n", time, crossingWhiteCnt, crossingWhiteLine);
          usb_send_str(s);
        }
 */       
        addToLog(LOG_LINE, &ldv, sizeof(ldv));
      }
      if (logRowFlags[LOG_DIST])
        addToLog(LOG_DIST, irRaw, sizeof(irRaw));
      if (logRowFlags[LOG_BATT])
        addToLog(LOG_BATT, &batVoltInt, sizeof(batVoltInt));
      if (logRowFlags[LOG_CTRLTIME])
        addToLog(LOG_CTRLTIME, &controlUsedTime, sizeof(controlUsedTime));
      if (logRowFlags[LOG_BAL_CTRL])
      {
        float b[5] = {regBalE[0], regul_bal_uvel, regBalUI[0], regBalUD[0], balTiltRef};
        float v[5] = {regBalVelE[0], regBalVelU[0], regBalVelUI[0], regBalVelUD[0], regul_balvel_reduc};
        
        if (false)
          // log balance controller
          addToLog(LOG_BAL_CTRL, b, sizeof(b)); 
        else
          // log balance velocity
          addToLog(LOG_BAL_CTRL, v, sizeof(v)); 
      }
      //     if (logRowFlags[LOG_EXTRA])
      //     {
      //       float val[4] = {regTurnE[0], regTurnU[0], regul_turn_vel_reduc[0], regul_turn_vel_reduc[1]};
      //       addToLog(LOG_EXTRA, val, sizeof(val));
      //     }
      if (logRowFlags[LOG_EXTRA])
      {
        //float val[4] = {regBalE[0], regBalU[0], regBalVelE[0], regBalVelU[0]};
        //float val[4] = {tiltu1, 0, accAng, gyroTilt};
        // extra as values in velocity controller - left motor
        float val[4] = {regVelELeft[0], regVelUDLeft[0], regul_vel_tot_ref[0], regVelULeft[0]};
        addToLog(LOG_EXTRA, val, sizeof(val));
      }
    }
    else
      stopLogging();
  }
}


// bool loggerRead()
// { // read from serial connection from logger - pt one-way to logger
//   // so no data expected
//   if (Serial1.available())
//   {
//     char c = Serial1.read();
//     if (c >= '\n')
//     {
//       rxbuffer[rxbufferCnt++] = Serial1.read();
//     }
//     if (rxbufferCnt >= LOG_RXBUFFER_MAX - 1)
//       // flush buffer
//       rxbufferCnt = 0;
//     if ((c == '\n' or c == '\r') and rxbufferCnt > 1)
//     { // terminate as string
//       rxbuffer[rxbufferCnt] = '\0';
//       return true;
//     }
//     rxCharCnt++;
//   }
//   return false;
// }


void writeTime(int8_t * data, int row, bool toSD)
{ // write time in seconds
  float v = *(float*)data;
  const int MSL = 50;
  char s[MSL];
  if (row == 0)
    snprintf(s, MSL, "%% %s (%d)\r\n%% %2d    time %.3f sec\r\n", robotname[robotId], robotId, col++, v);
  else
    snprintf(s, MSL, "%.3f ", v);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeMission(int8_t * data, int row, bool toSD)
{
  int16_t * v = (int16_t *)data;
  const int MSL = 50;
  char s[MSL];
  if (row == 0)
  {
    if (mission == 3) // user mission, then state is line number
      snprintf(s, MSL, "%% %2d    mission (%d) line %d\r\n", col++, mission, v[0]);
    else
      snprintf(s, MSL, "%% %2d    mission (%d) state %d\r\n", col++, mission, v[0]);
  }
  else
    snprintf(s, MSL, "%d ", v[0]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeAcc(int8_t * data, int row, bool toSD)
{
  const int MSL = 90;
  char s[MSL];
//  const float tompersec2 = 9.82 / 16384.0;
  int16_t * v = (int16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d %2d Acc x,y,z [m/s2]: %g %g %g\r\n", col, col+1, col+2, v[0]* accScaleFac, v[1]* accScaleFac, v[2]* accScaleFac);
    col += 3;
  }
  else
    snprintf(s, MSL, "%f %f %f ", v[0]* accScaleFac, v[1]* accScaleFac, v[2]* accScaleFac);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeGyro(int8_t * data, int row, bool toSD)
{
  const int MSL = 90;
  char s[MSL];
//  const float toDegPerSec = 250.0 / 32768;
  int16_t * v = (int16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d %2d Gyro x,y,z [deg/s]: %g %g %g\r\n", 
             col, col+1, col+2, v[0] * gyroScaleFac, v[1] * gyroScaleFac, v[2] * gyroScaleFac);
    col += 3;
  }
  else
    snprintf(s, MSL, "%f %f %f ", v[0] * gyroScaleFac, v[1] * gyroScaleFac, v[2] * gyroScaleFac);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeMag(int8_t * data, int row, bool toSD)
{
  const int MSL = 50;
  char s[MSL];
  int16_t * v = (int16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d %2d Mag x,y,z [int]: %d %d %d\r\n", col, col+1, col+2, v[0], v[1], v[2]);
    col += 3;
  }
  else
    snprintf(s, MSL, "%d %d %d ", v[0], v[1], v[2]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeCurrent(int8_t * data, int row, bool toSD)
{
  const int MSL = 90;
  char s[MSL];
  uint16_t * v = (uint16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Motor current left, right [A]: %.3f %.3f\r\n", 
             col, col+1, getMotorCurrentM(0, v[0]), getMotorCurrentM(1, v[1])
    );
    col += 2;
//     snprintf(s, MSL, "%% %2d %2d Motor current left, right [A]: %.3f %.3f\r\n", col, col+1, float(v[0])*1.2/1024.0/0.525, float(v[1])*1.2/1024.0/0.525);
//     col += 2;
  }
  else
  {
    snprintf(s, MSL, "%.3f %.3f ", 
             getMotorCurrentM(0, v[0]), getMotorCurrentM(1, v[1])
            );
    //snprintf(s, MSL, "%.3f %.3f ", float(v[0])*1.2/1024.0/0.525, float(v[1])*1.2/1024.0/0.525);
  }
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeVel(int8_t * data, int row, bool toSD)
{
  const int MSL = 60;
  char s[MSL];
  float * v = (float*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Wheel velocity [m/s] left, right: %.4f %.4f\r\n", col, col +1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(s, MSL, "%.4f %.4f ", v[0], v[1]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeTurnrate(int8_t * data, int row, bool toSD)
{
  const int MSL = 50;
  char s[MSL];
  float * v = (float*)data;
  if (row == 0)
    snprintf(s, MSL, "%% %d    Turnrate [r/s]: %.4f\r\n", col++, v[0]);
  else
    snprintf(s, MSL, "%.4f ", v[0]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}


void writeEnc(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  uint32_t * v = (uint32_t *)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Encoder left, right: %lu %lu\r\n", col, col+1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(s, MSL, "%lu %lu ", v[0], v[1]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

void writeMotPWM(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  int16_t * v = (int16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Motor voltage [PWM] left, right: %d %d\r\n", col, col+1, v[0], v[1]);
    col +=2;
  }
  else
    snprintf(s, MSL, "%d %d ", v[0], v[1]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}


void writeMotVRef(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  float * v = (float*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Motor velocity ref left, right: %.2f %.2f\r\n", col, col+1, v[0], v[1]);
    col += 2;
  }
  else
    snprintf(s, MSL, "%.2f %.2f ", v[0], v[1]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

/////////////////////////////////////////////////

void writeMotVolt(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  int16_t * v = (int16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Motor voltage [V] left, right: %.1f %.1f\r\n", col, col+1, float(v[0])/1000.0, float(v[1])/1000.0);
    col += 2;
  }
  else
    snprintf(s, MSL, "%.1f %.1f ", float(v[0])/1000.0, float(v[1])/1000.0);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

//////////////////////////////////////////////////

void writeBaro(int8_t * data, int row, bool toSD)
{
  const int MSL = 100;
  char s[MSL];
  int16_t * v = (int16_t*)data;
  uint32_t * u = (uint32_t*)&data[2];
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d %2d Barometer temp, pressure and height T, P, H: %.1f %ld %.2f\r\n", col, col+1, col+2, *v / 10.0, *u, v[3]/100.0);
    col += 3;
  }
  else
    snprintf(s, MSL, "%.1f %ld %.2f ", float(*v) / 10.0, *u, v[3]/100.0);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

/////////////////////////////////////////////////

void writePose(int8_t * data, int row, bool toSD)
{
  const int MSL = 100;
  char s[MSL];
  float * v = (float*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d %2d %2d Pose x,y,h,tilt [m,m,rad,rad]: %g %g %g %g\r\n", col, col+1, col+2, col+3, v[0], v[1], v[2], v[3]);
    col += 4;
  }
  else
    snprintf(s, MSL, "%.4f %.4f %.6f %.6f ", v[0], v[1], v[2], v[3]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

////////////////////////////////////////////////

void writeBatt(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  uint16_t * v = (uint16_t*)data;
  if (row == 0)
    snprintf(s, MSL, "%% %2d    Battery voltage [V]: %.2f\r\n", col++, *v / useADCMaxADCValue * 1.2 * (1.2 + 15)/1.2);
  else
    snprintf(s, MSL, "%.2f ", *v / useADCMaxADCValue * 1.2 * (1.2 + 15)/1.2);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

////////////////////////////////////////////////

void writeCtrlTime(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  uint16_t * v = (uint16_t*)data;
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Get data time [us]: %u +ctrl %u\r\n", col, col+1, v[0] * 10, v[1] * 10);
    col += 2;
  }
  else
    snprintf(s, MSL, "%u %u ", v[0] * 10, v[1] * 10);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

///////////////////////////////////////////////

void writeBalCtrl(int8_t * data, int row, bool toSD)
{
  const int MSL = 180;
  char s[MSL];
  float * v = (float*)data;
  const char * name;
  if (row == 0)
  {
    if (false)
      name = "tilt";
    else
      name = "velocity";
    snprintf(s, MSL, "%% %2d %2d %2d %2d %2d Ballance %s, e %f [rad], out %f [rad], I-out %f [m/s], D-out %f [rad], ref [m/s] %f\r\n", 
             col, col+1,col+2,col+3, col+4, name, v[0], v[1], v[2], v[3], v[4]);
    col += 5;
  }
  else
    snprintf(s, MSL, "%f %f %f %f %f ", v[0], v[1], v[2], v[3], v[4]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

/////////////////////////////////////////////

void writeExtra(int8_t * data, int row, bool toSD)
{
  const int MSL = 120;
  char s[MSL]; // space for 4 values?
  float * v = (float*)data;
  if (row == 0)
  {
//     snprintf(s, MSL, "%% %2d %2d %2d %2d Extra pt.: turn err, u, turn u[0] u[1]: %.2f %.4f %.4f %.4f\r\n", 
//              col, col+1, col+2, col+3, v[0], v[1], v[2], v[3]);
//    snprintf(s, MSL, "%% %2d %2d %2d %2d Extra pt.: balE[0], balU[0], balVelE[0], balVelU[0]: %.2f %.4f %.4f %.4f\r\n", 
    snprintf(s, MSL, "%% %2d %2d %2d %2d Extra : %.4f %.4f %.4f %.4f\r\n", 
                      col, col+1, col+2, col+3, v[0], v[1], v[2], v[3]);
    col += 4;
  }
  else
    snprintf(s, MSL, "%.2f %.2f %.4f %.4f ", v[0], v[1], v[2], v[3]);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

/////////////////////////////////////

void writeLineSensor(int8_t * data, int row, bool toSD)
{
  /*
   *        ldv[0] = adcLSH[0] - adcLSL[0];
   *        ldv[1] = adcLSH[1] - adcLSL[1];
   *        ldv[2] = adcLSH[2] - adcLSL[2];
   *        ldv[3] = adcLSH[3] - adcLSL[3];
   *        ldv[4] = adcLSH[4] - adcLSL[4];
   *        ldv[5] = adcLSH[5] - adcLSL[5];
   *        ldv[6] = adcLSH[6] - adcLSL[6];
   *        ldv[7] = adcLSH[7] - adcLSL[7];
   *        fdv[0] = lsLeftSide; 8,9
   *        fdv[1] = lsRightSide; 10,11
   *        ldv[12] = 0;
   *        if (useLineSensor)
   *          ldv[12] = 1;
   *        if (lsIsWhite)
   *          ldv[12] |= 2;
   *        if (lsLeftValid)
   *          ldv[12] |= 4;
   *        if (lsRightValid)
   *          ldv[12] |= 8;
   */
  const int MSL = 220;
  char s[MSL]; // space for 4 values?
  int16_t * v = (int16_t*)data;
  float * vf = (float*)&v[8];
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d .. %2d Line sensor: left %f %d, right %f %d, values %d %d %d %d %d %d %d %d, "
                     "white %d, used %d, LEDhigh=%d, xb=%d xw=%d xbc=%d xwc=%d\r\n", 
             col, col+19, 
             vf[0], (v[12] & 0x04) == 0x04, vf[1], (v[12] & 0x08) == 0x08,
             v[0],v[1], v[2], v[3], v[4], v[5], v[6], v[7],
             (v[12] & 0x02) == 0x02, (v[12] & 0x01) == 0x01, (v[12] & 0x10) == 0x10,
             (v[12] & 0x40) == 0x40, (v[12] & 0x80) == 0x80,
             v[13], v[14]
    );
    col += 19;
  }
  else
    snprintf(s, MSL, "%.4f %d %.4f %d  %d %d %d %d %d %d %d %d  %d %d %d %d %d %d %d ", 
             vf[0], (v[12] & 0x04) == 0x04, vf[1], (v[12] & 0x08) == 0x08,
             v[0],v[1], v[2], v[3], v[4], v[5], v[6], v[7],
             (v[12] & 0x02) == 0x02, (v[12] & 0x01) == 0x01, (v[12] & 0x10) == 0x10,
             (v[12] & 0x40) == 0x40, (v[12] & 0x80) == 0x80,
             v[13], v[14]
    );
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

/////////////////////////////////////////////////

void writeDistSensor(int8_t * data, int row, bool toSD)
{
  const int MSL = 70;
  char s[MSL];
  uint16_t * v = (uint16_t*)data;
  float d1 = 0, d2 = 0;
  if (v[0] > 0)
    d1 = irA[0] + irB[0]/v[0];
  if (v[1] > 1)
    d2 = irA[1] + irB[1]/v[1];
  
  if (row == 0)
  {
    snprintf(s, MSL, "%% %2d %2d Distance sensor raw: %.3f %.3f\r\n", col, col+1, d1, d2);
    col += 2;
  }
  else
    snprintf(s, MSL, "%.3f %.3f ", d1, d2);
  if (toSD)
    Serial1.write(s);
  else
    Serial.write(s);
}

///////////////////////////////////////////////

void initLogStructure(int timeFactor)
{
  // start at first position
  logRowSize = 0;
  logTimeFactor = timeFactor;
  logRowFlags[LOG_TIME] = true;
  // set log entry positions
  for (int i = 0; i <  LOG_MAX_CNT; i++)
  {
    if (logRowFlags[i])
    {
      int bz;
      logRowPos[i] = logRowSize;
      switch (logRowItemSize[i * 2 + 1])
      {
        case LOG_FLOAT  : bz = sizeof(float); break;
        case LOG_DOUBLE : bz = sizeof(double); break;
        case LOG_INT8   : bz = sizeof(int8_t); break;
        case LOG_UINT8  : bz = sizeof(uint8_t); break;
        case LOG_INT16  : bz = sizeof(int16_t); break;
        case LOG_UINT16 : bz = sizeof(uint16_t); break;
        case LOG_INT32  : bz = sizeof(int32_t); break;
        case LOG_UINT32 : bz = sizeof(uint32_t); break;
        default: bz = 1; break;
      }
      logRowSize += bz * logRowItemSize[i * 2];
    }
  }
  const int MSL = 50;
  char s[MSL];
  snprintf(s, MSL,"initLogStructure size=%d bytes/line\n", logRowSize);
  usb_send_str(s);
  logRowsCntMax = LOG_BUFFER_MAX / logRowSize;
  // clear buffer
  logRowCnt = 0;
}

////////////////////////////////////////////

int posDbg = 0;

void addToLog(logItem item, void * data, int dataCnt)
{
  int8_t * pd = logBuffer + (logRowCnt - 1) * logRowSize + logRowPos[item];
//   if (false)
//   {
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "#add_to_log 0 <= (%d + %d) < %d, line %d/%d, from %x cnt %d\n", 
//              (logRowCnt - 1) * logRowSize, 
//              logRowPos[item],
//              LOG_BUFFER_MAX, 
//              logRowCnt, logRowsCntMax,
//              (int)data, dataCnt
//             );
//     usb_send_str(s);
//   }
  memcpy(pd, data, dataCnt);
}

//////////////////////////////////////////////////////

bool addNewRowToLog()
{ // logRowCnt is next log entry - uses (logRowCnt-1)
  if (logRowCnt >= logRowsCntMax)
    return false;
  int8_t * pd = logBuffer + logRowCnt * logRowSize;
  if (uint32_t(0x20000000) > (uint32_t)pd and (uint32_t(0x20000000) < ((uint32_t)pd + (uint32_t)logRowSize))) 
  { // skip the row that spans address 0x20000000
    const int MSL = 70;
    char s[MSL];
    snprintf(s, MSL,"#mem-hole skipped row %d from %x to %x\n", logRowCnt, (unsigned int)pd, (unsigned int)pd + logRowSize);
    usb_send_str(s);
    logRowCnt++;
    if (logRowCnt >= logRowsCntMax)
      return false;    
  }
  logRowCnt++;
  return true;
}

//////////////////////////////////////////////////

void setLogSize(logItem item, int count, char type)
{
  logRowItemSize[item * 2] = count;
  logRowItemSize[item * 2 + 1] = type;
}

//////////////////////////////////////////////////

int logWriteBufferTo(int row)
{
//  int row;
  logItem item;
  int8_t * bp;
  // write first logged items
  if (row == 0)
    col = 1;
//  for (row = 0; row < logRowCnt; row++)
  { // write all recorded rows
    bp = logBuffer + row * logRowSize;
    if (uint32_t(0x20000000) > (uint32_t)bp and (uint32_t(0x20000000) < ((uint32_t)bp + (uint32_t)logRowSize))) 
    { // there is problems at address 0x20000000, so must be skipped
      row++;
      if (row >= logRowCnt)
        // it was last usable row
        return row;
      bp += logRowSize;
    }
    for (item = LOG_TIME; item < LOG_MAX_CNT; item = logItem(int(item) + 1))
    { // go througt all possible log items
      if (logRowFlags[item])
      { // this log item is recorded
//         if (false and row==0)
//         {
//           const int MSL = 50;
//           char s[MSL];
//           snprintf(s, MSL,"# log read used item=%d\n", item);
//           usb_send_str(s);
//         }
        switch (item)
        {
          case LOG_TIME:  writeTime(&bp[logRowPos[item]], row, false); break;
          case LOG_MISSION: writeMission(&bp[logRowPos[item]], row, false); break;
          case LOG_ACC:   writeAcc(&bp[logRowPos[item]], row, false); break;
          case LOG_GYRO:  writeGyro(&bp[logRowPos[item]], row, false); break;
//           case LOG_MAG:   writeMag(&bp[logRowPos[item]], row, false); break;
          case LOG_MOTV_REF:  writeMotVRef(&bp[logRowPos[item]], row, false); break;
          case LOG_MOTV:  writeMotVolt(&bp[logRowPos[item]], row, false); break;
          case LOG_MOTA:  writeCurrent(&bp[logRowPos[item]], row, false); break;
          case LOG_ENC:   writeEnc(&bp[logRowPos[item]], row, false); break;
          case LOG_WHEELVEL:   writeVel(&bp[logRowPos[item]], row, false); break;
          case LOG_TURNRATE:   writeTurnrate(&bp[logRowPos[item]], row, false); break;
          case LOG_POSE:  writePose(&bp[logRowPos[item]], row, false); break;
          case LOG_BATT:  writeBatt(&bp[logRowPos[item]], row, false); break;
          //case LOG_BARO:  writeBaro(&bp[logRowPos[item]], row, false); break;
          case LOG_LINE:  writeLineSensor(&bp[logRowPos[item]], row, false); break;
          case LOG_DIST:  writeDistSensor(&bp[logRowPos[item]], row, false); break;
          case LOG_CTRLTIME: writeCtrlTime(&bp[logRowPos[item]], row, false); break;
          case LOG_BAL_CTRL: writeBalCtrl(&bp[logRowPos[item]], row, false); break;
          case LOG_EXTRA: writeExtra(&bp[logRowPos[item]], row, false); break;
          default: break;
        }
      }
//       else if (false and row==0)
//         {
//           const int MSL = 50;
//           char s[MSL];
//           snprintf(s, MSL, "# log read skip item=%d\n", item);
//           usb_send_str(s);
//         }
    }
    //add a new line
    if (false)
      Serial1.write("\r\n");
    else
      Serial.write("\r\n");
//    serialWrite("\n");
  }
  return row;
}

/**
 * Mission init is called befor any control is attempted,
 * this can be used to initialize ant variables dependent on measured values, i.e.
 * battery voltage or gyro.
 * and to set data logger options */
void setLogFlagDefault()
{
  // log data size
  // data log size and count must be initialized here
  // there must further be a write function and other parts in data_logger.cpp 
  // to add new logged data items
  setLogSize(LOG_TIME, 1, LOG_FLOAT);
  setLogSize(LOG_MISSION, 1, LOG_INT16);
  setLogSize(LOG_ACC,  3, LOG_INT16);
  setLogSize(LOG_GYRO, 3, LOG_INT16);
  //   setLogSize(LOG_MAG,  3, LOG_INT16);
  setLogSize(LOG_MOTV_REF, 2, LOG_FLOAT);
  setLogSize(LOG_MOTV, 2, LOG_INT16);
  setLogSize(LOG_MOTA, 4, LOG_INT16);
  setLogSize(LOG_ENC,  2, LOG_UINT32);
  setLogSize(LOG_POSE, 4, LOG_FLOAT);
  setLogSize(LOG_WHEELVEL,  2, LOG_FLOAT);
  setLogSize(LOG_TURNRATE,  1, LOG_FLOAT);
  setLogSize(LOG_LINE, 15, LOG_UINT16);
  setLogSize(LOG_DIST, 2, LOG_UINT16);
  setLogSize(LOG_BATT, 1, LOG_UINT16);
  setLogSize(LOG_CTRLTIME, 2, LOG_INT16);
  //setLogSize(LOG_BARO, 4, LOG_INT16);
  setLogSize(LOG_BAL_CTRL, 5, LOG_FLOAT);
  setLogSize(LOG_EXTRA, 4, LOG_FLOAT);
  logRowFlags[LOG_TIME] = 1; // not tested - time always on
  //
  // log flags (default)
  missionState = 0;
  logRowFlags[LOG_TIME] = true; // state number in mission
  logRowFlags[LOG_MISSION] = true; // state number in mission
  logRowFlags[LOG_ACC] = false;    // in ?
  logRowFlags[LOG_GYRO] = true;    // in ?
  //logRowFlags[LOG_MAG] = false;    // not used
  logRowFlags[LOG_MOTV] = true;    // orderd anchor voltage (before PWM)
  logRowFlags[LOG_MOTA] = false;   // measured anchor current in Amps
  logRowFlags[LOG_WHEELVEL] = true;  // wheel velocity in rad/s
  logRowFlags[LOG_ENC] = false;    // raw encoder counter
  logRowFlags[LOG_POSE] = true;    // calculated pose x,y,th
  logRowFlags[LOG_LINE] = false;    // line sensor
  logRowFlags[LOG_DIST] = false;    // distance sensor
  logRowFlags[LOG_BATT] = true;    // battery oltage in Volts
  //logRowFlags[LOG_BARO] = false;   // not implemented
  logRowFlags[LOG_BAL_CTRL] = false; // ballance control details
}


void eePromSaveStatusLog()
{
  int n = LOG_MAX_CNT / 4 + 1;
  const uint32_t * flags = (uint32_t *) logRowFlags;
  for (int i = 0; i < n; i++)
  {
    eeConfig.push32(flags[i]);
  }
  eeConfig.push32(logInterval);
  eeConfig.push32(logAllow);
}

void eePromLoadStatusLog()
{
  int n = LOG_MAX_CNT / 4 + 1;
  uint32_t * flags = (uint32_t *) logRowFlags;
  for (int i = 0; i < n; i++)
  {
    flags[i] = eeConfig.read32();
  }
  logInterval = eeConfig.read32();
  logAllow = eeConfig.read32();
}



