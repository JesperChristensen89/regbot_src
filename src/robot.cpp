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

#include "math.h"
#include "motor_controller.h"
#include "robot.h"
#include "main.h"
#include "mpu9150.h"
#include "eeconfig.h"

//float velocity[2];
float pose[4] = {0,0,0,0}; // x,y,h,tilt
float distance = 0.0; // distance in meters (forward) reverse is negative.
float turnRate = 0.0;

float odoWheelBase = 0.155; // distance between wheels
float odoWheelRadius[2] = {0.0595/2.0, 0.0595/2.0};
float wheelVelocity[2] = {0, 0}; // in radians per second
/// estimated velocity - same as wheelVelocity in most cases
float wheelVelocityEst[2] = {0, 0}; // in radians per second
/// should velocity at low speed be estimated (observer)
//bool regul_vel_est = true;
float wheelPosition[2] = {0, 0}; // in radians since start

float gear = 22.0 * 22.0 * 24.0 / (12.0 * 10.0 * 10.0);
uint16_t pulsPerRev = 48; // using all edges
float anglePerPuls = 2.0 * M_PI / (pulsPerRev *  gear);

float balanceOffset = 0.0;
bool batteryUse = true;
uint16_t batteryIdleVoltageInt = int(9.9 / batVoltIntToFloat);

const char * robotname[MAX_ROBOT_NAMES] = 
{ "invalid", // 0
  "Emma",
  "Sofia",
  "Ida",
  "Freja",
  "Clara", // 5
  "Laura",
  "Anna",
  "Ella",
  "Isabella",
  "Karla", // 10
  "Alma",
  "Josefine",
  "Olivia",
  "Alberte",
  "Maja", // 15
  "Sofie", // 16
  "Mathilde",
  "Agnes",
  "Lily",
  "Caroline", // 20
  "Liva",
  "Emily",
  "Sara",
  "Victoria",
  "Emilie", // 25
  "Mille",
  "Frida",
  "Marie",
  "Ellen",
  "Rosa", // 30
  "Lea",
  "Signe",
  "Filippa",
  "Julie",
  "Nora", // 35
  "Liv",
  "Vigga",
  "Nanna",
  "Naja",
  "Alba", // 40
  "Astrid",
  "Aya",
  "Asta",
  "Luna",
  "Malou", // 45
  "Esther",
  "Celina",
  "Johanne",
  "Andrea", // 49
  "Silje", // 50
  "Thea", 
  "Carla", 
  "Dicte", 
  "Silke", 
  "Eva", // 55
  "Gry", 
  "Emilia", 
  "Sigrid", 
  "Augusta"  // 59
};


void clearPose()
{
  pose[0] = 0.0;
  pose[1] = 0.0;
  pose[2] = 0.0;
  distance = 0.0;
  wheelPosition[0] = 0;
  wheelPosition[1] = 0;
}

void setRobotID(const char * buf)
{
  char * p1 = (char *) buf;
  float f = 0;
  robotId = strtol(p1, (char**)&p1, 10);
  if (robotId != 0)
  {
    odoWheelBase = strtof(p1, (char**)&p1);
    gear = strtof(p1, (char**)&p1);
    pulsPerRev = strtof(p1, (char**)&p1);
    odoWheelRadius[0] = strtof(p1, (char**)&p1);
    odoWheelRadius[1] = strtof(p1, (char**)&p1);
    balanceOffset = strtof(p1, (char**)&p1);
    anglePerPuls = 2.0 * M_PI / (pulsPerRev * gear);
    batteryUse = strtol(p1, (char**)&p1, 0);
    f = strtof(p1, (char**)&p1);
    batteryIdleVoltageInt = int(f) / batVoltIntToFloat;
  }
  usb_send_str("# set robot ID called:");
  usb_send_str(buf);
  usb_send_str("\r\n");
  // debug
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "#battery on %d at volt=%f (int %d) now %d\r\n", batteryUse, f, batteryIdleVoltageInt, batVoltInt);
  usb_send_str(s);
  // debug end
}

/////////////////////////////////////

void eePromSaveRobotId()
{
  eeConfig.pushWord(robotId);
  eeConfig.pushFloat(odoWheelBase);
  eeConfig.pushFloat(gear);
  eeConfig.pushWord(pulsPerRev);
  eeConfig.pushFloat(odoWheelRadius[0]);
  eeConfig.pushFloat(odoWheelRadius[1]);
  eeConfig.pushFloat(balanceOffset);
  eeConfig.pushByte(batteryUse);
  eeConfig.pushWord(batteryIdleVoltageInt);
}

/////////////////////////////////////

void eePromLoadRobotId()
{
  int id = eeConfig.readWord();
  if (id > 0)
  {
    robotId = id;
    odoWheelBase = eeConfig.readFloat();
    gear = eeConfig.readFloat();
    pulsPerRev = eeConfig.readWord();
    odoWheelRadius[0] = eeConfig.readFloat();
    odoWheelRadius[1] = eeConfig.readFloat();
    balanceOffset = eeConfig.readFloat();
    batteryUse = eeConfig.readByte();
    batteryIdleVoltageInt = eeConfig.readWord();
  }
  else
  { // just read and dump all values, leaving default settings
    for (int i = 0; i < 6; i++)
      eeConfig.read32();
    usb_send_str("# IMU eeProm data ignored\r\n");
  }
}


void sendStatusRobotID()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "rid %d %g %g %d %g %g %g %d %g %s\r\n",
           robotId, 
           odoWheelBase, gear, 
           pulsPerRev,
           odoWheelRadius[0], odoWheelRadius[1],
           balanceOffset,
           batteryUse, batteryIdleVoltageInt * batVoltIntToFloat,
           robotname[robotId]
          );
  usb_send_str(reply);
}

////////////////////////////////////////////

// float estWv[2] = {0.0, 0.0};  // estimated motor velocity
// float estMV[2] = {00, 0.0};  // motor voltage saved
// float estWvF[2] = {0.0, 0.0}; // high pass filtered output
// float ht = 0.0; // global for debug
// 
// void zeroVelEstimator()
// { // prepare velocity filter with initial values
//   estWv[0] = 0.0;  // estimated motor velocity
//   estMV[0] = 0.0;  // motor voltage saved
//   estWvF[0] = 0.0; // high pass filtered output
//   estWv[1] = 0.0;  // estimated motor velocity
//   estMV[1] = 0.0;  // motor voltage saved
//   estWvF[1] = 0.0; // high pass filtered output
//   wheelVelocity[0] = 0.0;
//   wheelVelocity[1] = 0.0;
//   wheelVelocityEst[0] = 0.0;
//   wheelVelocityEst[1] = 0.0;
// }

/**
 * Estimate filtered velocity based on motor voltage, using
 * 
 *         wheel_vel   0.1268 z + 0.1268
 * model   --------- = -----------------
 *         motor_V        z - 0.9713
 * 
 * highpass filter with a pole at w=2/zoh time for velocity calculation 
 *
 *                     ht (z - 1)
 * filter(z)  = --------------------------
 *              (ht+0.0005)z - (ht-0.0005)
 * 
 *           pi
 * ht = ------------------------   half of time betw. pulses
 *      vel * puls_per_wheel_rev
 * 
 * \param vel is current calculated wheeel velocity (rad/s)
 * \param mv  is current motor voltage [V]
 * \param zwv is estimated wheel velocity for k-1 (rad/s)
 * \param zmv is motor voltage from k-1
 * \param zvf is filtered estimated wheel velocity from k-1
 * \returns new filtered estimated wheel velocity */
// float estWheelVelLowPass(float vel, float mv, float * zwv, float * zmv, float * zvf)
// { // calculate zero order hold time - divided by 2
// //   float ht = 0.0;
//   if (fabs(vel) > 0.001) 
//     ht = M_PI/ (pulsPerRev * gear) / fabs(vel);
//   else
//     ht = 0.0;
//   if (ht > 0.001)
//   { // calculate new estimated wheel velocity
//     float wv = 0.9718 * *zwv + 0.1268 * mv + 0.1268 * *zmv;
//     // support konstant
//     float kp = ht/(ht + 0.0005);
//     float km = ht/(ht - 0.0005);
//     // new filtered estimated wheel velocity
//     float wvf = kp / km * *zvf + kp * (wv - *zwv);
//     // save new values
//     *zwv = wv;
//     *zmv = mv;
//     *zvf = wvf;
//   }
//   else
//   {
//     *zwv = vel;
//     *zmv = mv;
//     *zvf = 0.0;
//   }
//   return *zvf;
// }

/**
 * estimate tilt angle, as komplentary filter with gyro and acc 
 *       1     tau s                     1
 * Gyro ---  ----------  + acc_pitch --------
 *       s    tau s + 1              tau s + 1
 *
 *     1        T/(T+2.*tau) + *T/(T+2.*tau) * z^-1
 * --------- = -------------------------------------
 *  tau s + 1     1 + (T-2.*tau)/(T+2.*tau) * z^-1
 *
 * T = 0.001;
 * tau = 0.05;
 *
 *  (0.0099 + 0.0099 * z^-1)
 *  ------------------------
 *   (1  - 0.98 * z^-1)
 
 Discrete-time transfer function.  */
/// tilt angle estimator
float tiltu1  = 0; // old value for complementary filter
float accAng;   // for debug
float gyroTiltRate; // for debug
void estimateTilt()
{
  static const float T = 0.001;
  static const float tau = 1.0; // seems to give good responce
  static const float b = T/(T + 2 * tau);
  static const float a = -(T - 2 * tau)/(T + 2 * tau);
  float u; // input to filter
  float est; // estimated angle
  // gyro mounted on top plate!
  accAng = atan2f(-float(imuAcc[0]),-float(imuAcc[2]));
  // offset with value that makes the robot balance
  accAng -= balanceOffset;
  // New and old angle must be in same revolution
  if ((accAng - pose[3]) > M_PI)
    accAng -= 2*M_PI;
  else if ((accAng - pose[3]) < -M_PI)
    accAng += 2*M_PI;
  // gyro is running in mode 0= 250 grader/sek, 1 = 500 deg/s, 2=1000 deg/s 3=2000 deg/s)
  gyroTiltRate = -float(imuGyro[1]) * gyroScaleFac * M_PI / 180.0; // radianer pr sekund
  // add gyro and accelerometer reading
  u = accAng + gyroTiltRate * tau;
  if (true) // imuGyro[0] < 245 and imuGyro[0] > -245)
  { // gyro not saturated
    // filter
    if (accAng > 0.0 and pose[3] < -M_PI/2.0)
      est = a * (pose[3] + 2 * M_PI) + b * u + b * tiltu1; 
    else if (accAng < 0.0 and pose[3] > M_PI/2.0)
      est = a * (pose[3] - 2 * M_PI) + b * u + b * tiltu1;
    else
      est = a * pose[3] + b * u + b * tiltu1;
  }
  else
    // else use angle as is from accelerometer
    est = accAng;
  //
  if (est > M_PI)
  { // folded angle
    est -= 2 * M_PI;
    // save last value of u in right angle space
    tiltu1 = accAng - 2 * M_PI + gyroTiltRate * tau;
  }
  else if (est < -M_PI)
  { // folded
    est += 2 * M_PI;
    tiltu1 = accAng - 2 * M_PI + gyroTiltRate * tau;
  }
  else
  { // no foldeing
    tiltu1 = u;
  }
  //
  pose[3] = est; // exfav[0]; // est;
}

/**
 * Update robot pose and velocity 
 * This function is called at every sample time (about 1ms) 
 * and updates wheel velocity [m/s] and wheel position [m] as well as robot pose [m,m,rad] */
void updatePose(uint32_t loop)
{
  float v1, v2; // encoder tick speed (ticks/sec)
  // velocity
  if (not encTimeOverload[0] and encPeriod10us[0] > 0)
  { // time since last encoder pulse
    int32_t ta = (int32_t)hb10us - (int32_t)encStartTime[0];
    // calculate ticks per second
    if (ta > (int32_t)encPeriod10us[0])
      // very slow, more than sample time since last encoder ticks
      // so use time from last encoder tick to now
      v1 = 100000/ta;
    else
      v1 = 100000/encPeriod10us[0];
    // calculate speed as radians per second (positive forward)
    if (encCCV[0])
      wheelVelocity[0] = -v1 * anglePerPuls;
    else
      wheelVelocity[0] = v1 * anglePerPuls;
//     // debug
//     if (hbTimerCnt % 100 == 0)
//     { 
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "# UpdatePose: vel=%f, app=%f, v1=%f, ta=%ld\n", wheelVelocity[0], anglePerPuls, v1, ta);
//       usb_send_str(s);      
//     }
//     // debug end
  }
  else
    wheelVelocity[0] = 0.0;
  if (not encTimeOverload[1] and encPeriod10us[1] > 0)
  { // time since last encoder pulse
    int32_t ta = (int32_t)hb10us - (int32_t)encStartTime[1];
    // calculate ticks per second
    if (ta > (int32_t)encPeriod10us[1])
      // very slow, more than sample time since last encoder ticks
      // so use time from last encoder tick to now
      v2 = 100000/ta;
    else
      // last encoder tick period shorter than sample time
      // so use time between encoder pulses
      v2 = 100000/encPeriod10us[1];
    // calculate speed as radians per second (positive forward)
    if (encCCV[1])
      wheelVelocity[1] = -v2 * anglePerPuls;
    else
      wheelVelocity[1] = v2 * anglePerPuls;
  }
  else
    // very very long time since last encoder tick (timer overload), so
    // velocity must be zero (or very high speed, ie <10us between encoder ticks)
    wheelVelocity[1] = 0.0;
  //
  // add low speed kompensation for long zero order hold - time between vel calculation
//   if (fabs(wheelVelocity[0]) < 20.0 and regul_vel_est)
//     wheelVelocityEst[0] = wheelVelocity[0] * odoWheelRadius[0] + 
//            estWheelVelLowPass(wheelVelocity[0], motorAnkerVoltage[0], &estWv[0], &estMV[0], &estWvF[0]);
//   else
    wheelVelocityEst[0] = wheelVelocity[0] * odoWheelRadius[0];
  //    
//   if (fabs(wheelVelocity[1]) < 20.0 and regul_vel_est)
//     wheelVelocityEst[1] = wheelVelocity[1] * odoWheelRadius[1] + 
//            estWheelVelLowPass(wheelVelocity[1], motorAnkerVoltage[1], &estWv[1], &estMV[1], &estWvF[1]);
//   else
    wheelVelocityEst[1] = wheelVelocity[1] * odoWheelRadius[1];
//   if (true and loop % 10 == 0)
//   {
//     const int MSL = 50;
//     char s[MSL];
//     snprintf(s, MSL, "#w est ht=%.g, vel1=%g, est1=%g\r\n", ht, wheelVelocity[1], wheelVelocityEst[1]);
//     usb_send_str(s);
//   }
  //
  turnRate = (wheelVelocityEst[1] - wheelVelocityEst[0])/odoWheelBase ;
  //
  // calculate movement and pose based on encoder count
  // encoder count is better than velocity based on time.
  // encoder position now
  uint32_t p1 = encoder[0];
  uint32_t p2 = encoder[1];
  // position change in encoder tics since last update
  int dp1 = (int32_t)p1 - (int32_t)encoderLast[0];
  int dp2 = (int32_t)p2 - (int32_t)encoderLast[1];
  // save current tick position to next time
  encoderLast[0] = p1;
  encoderLast[1] = p2;
  // angle movement with forward as positive
  v1 =  dp1 * anglePerPuls * odoWheelRadius[0];
  v2 =  dp2 * anglePerPuls * odoWheelRadius[1];
  // integrate wheel position for each wheel
  wheelPosition[0] += v1;
  wheelPosition[1] += v2;
  // heading change in radians
  float dh = (v2 - v1) / odoWheelBase;
  // distance change in meters
  float ds = (v1 + v2) / 2.0;
  distance += ds;
  // add half the angle
  pose[2] += dh/2.0;
  // update pose position
  pose[0] += cosf(pose[2]) * ds;
  pose[1] += sinf(pose[2]) * ds;
  // add other half angle
  pose[2] += dh/2.0;
  // fold angle
  if (pose[2] > M_PI)
    pose[2] -= M_PI * 2;
  else if (pose[2] < -M_PI)
    pose[2] += M_PI * 2;
  
}
