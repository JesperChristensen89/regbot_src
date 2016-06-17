/***************************************************************************
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Motor controller functions - controlling Pololu1213 dual motor controller
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
 
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "main.h"

#define M1IN2    2  // M1IN2 - Teensy Digital 2 (direction)
#define M1IN1    3  // M1IN1 - Teensy Digital 3 (PWM)
#define M1D2     4  // M1D2  - Teensy Digital 4 (disable (hiz)))
                            // M1D1  - Teensy GND
#define M1FB    15  // M1FB  - Teensy Analog A1 (15)
#define SF       5  // M1SF  - Teensy Digital 5
                            // M2SF  - Teensy Digital 5
//#define STANDBY  6  // EN    - Teensy Digital 6
#define SLEW     7  // SLEW  - Teensy Digital 7
#define M2IN2    8  // M2IN2 - Teensy Digital 8 (direction)
#define M2IN1    9  // M2IN1 - Teensy Digital 9 (PWM)
#define M2D2    10  // M2D2  - Teensy Digital 10 (disable (hi-z))
                            // M2D1  - Teensy GND
#define M2FB    14  // M2SF  - Teensy Analog A0 (14)
                    // GND   - Teensy GND
                    // VDD   - Teensy 5 V
                    // VIN   - Power Source (+) 5 V
                    // GND   - Power Source (-)
                    // M1OUT1 and M1OUT2 to Motor

#define M1ENC_A 20  // Encoder A - Teensy Digital 20
#define M1ENC_B 19  // Encoder B - Teensy Digital 19
#define M2ENC_A 22  // Encoder A - Teensy Digital 22
#define M2ENC_B 21  // Encoder B - Teensy Digital 21

#define MOTOR1   0
#define MOTOR2   1

/**
 * Motor ankor voltage - assuming battery voltage is 12V */
//extern float batVoltFloat;
extern float motorAnkerVoltage[2];
extern float max_motor_voltage;
extern int16_t motorAnkerPWM[2];
/**
 * Motor enable flages - not enabled means high impedanze for the motor. */
extern int8_t  motorEnable[2];
/**
 * Motor direction flag. */
extern bool    directionFWD[2];

/// encoder
extern uint32_t encoder[2];
extern uint32_t encoderLast[2];
extern uint32_t encStartTime[2];
extern uint32_t  encPeriod10us[2];
extern bool     encCCV[2];
extern bool     encTimeOverload[2];

extern float motorCurrentA[2];
extern uint16_t motorCurrentM[2];
extern uint16_t motorCurrentMOffset[2];
extern uint32_t motorCurrentMLowPass[2];

/**
 * initialize motor port directions and sets PWM mode */
void motorInit();
/**
 * Reads motor status flag (overheat) */
bool motorError();
/**
 * Set motor speed - NB! one of the speed should change sign to run same way
 * \param m1PWM motor 1 speed in range +/- 1024
 * \param m2PWM motor 2 speed in range +/- 1024
 */
void motorSetAnkerPWM(int m1PWM, int m2PWM);
/**
 * Set motor anker voltage
 * The voltage is valid only if battery voltage is 12V
 * Else the voltage scales acordingly */
inline void motorSetAnchorVoltage()
{
  const float batteryNominalVoltage = batVoltInt * batVoltIntToFloat;
  const float zeroOffset = 0.14; // voltage across H-bridge transistors
  const float offsetFactor = 1.034; // scale error - from measurement
  float offFac = offsetFactor * 1024.0 / batteryNominalVoltage;
  int iOffset = int((zeroOffset * 1024.0) / batteryNominalVoltage);
//   const float max_motor_voltage = 9;
//   // deadband compensate
//   if (false)
//   { // compensate for too low motor voltage
//     const float minMotorVolt = 0.75;
//     if (true)
//     { // skip low voltage by a shift
//       if (motorAnkerVoltage[0] > 0.1)
//         motorAnkerVoltage[0] += minMotorVolt;
//       if (motorAnkerVoltage[0] < -0.1)
//         motorAnkerVoltage[0] -= minMotorVolt;
//       if (motorAnkerVoltage[1] > 0.1)
//         motorAnkerVoltage[1] += minMotorVolt;
//       if (motorAnkerVoltage[1] < -0.1)
//         motorAnkerVoltage[1] -= minMotorVolt;
//     }
//     else
//     { // skip low voltage by minimum
//       if (motorAnkerVoltage[0] > 0.01 and motorAnkerVoltage[0] < minMotorVolt)
//         motorAnkerVoltage[0] = minMotorVolt;
//       if (motorAnkerVoltage[0] < -0.01 and motorAnkerVoltage[0] > -minMotorVolt)
//         motorAnkerVoltage[0] = -minMotorVolt;
//       if (motorAnkerVoltage[1] > 0.01 and motorAnkerVoltage[1] < minMotorVolt)
//         motorAnkerVoltage[1] = minMotorVolt;
//       if (motorAnkerVoltage[1] < -0.01 and motorAnkerVoltage[1] > -minMotorVolt)
//         motorAnkerVoltage[1] = -minMotorVolt;
//     }
//   }
  if (motorAnkerVoltage[0] > max_motor_voltage)
    motorAnkerVoltage[0] = max_motor_voltage;
  else if (motorAnkerVoltage[0] < -max_motor_voltage)
    motorAnkerVoltage[0] = -max_motor_voltage;
  if (motorAnkerVoltage[1] > max_motor_voltage)
    motorAnkerVoltage[1] = max_motor_voltage;
  else if (motorAnkerVoltage[1] < -max_motor_voltage)
    motorAnkerVoltage[1] = -max_motor_voltage;
  motorSetAnkerPWM( int(motorAnkerVoltage[0] * offFac) + iOffset, 
                    int(-motorAnkerVoltage[1] * offFac) + iOffset);
}

/**
 * Set motor driver voltage
 * NB! can be set higher than 6V and will be implemented - up to supply (battVoltFloat) voltage.
 * \param left is new anchor voltage for left wheel
 * \param right is new anchor voltage for left wheel */
// inline void addMotorVoltage(float left, float right)
// {
//   motorAnkerVoltage[0] += left;
//   motorAnkerVoltage[1] += right;
// }

/**
 * Set motor driver voltage
 * NB! can be set higher than 6V and will be implemented - up to supply (battVoltFloat) voltage.
 * \param turnrate approximately turnrage in rad/sec positive is CCV
 */
// inline void addMotorTurnrate(float turnrate)
// {
//   float av = turnrate * 0.25;
//   motorAnkerVoltage[0] -= av;
//   motorAnkerVoltage[1] += av;
// }

/**
 * Enable one, two or no motors 
 * \param e1 is enable of motor 1 (0 is disable)
 * \param e2 is enable of motor 2 (0 is disable)
 * */
void motorSetEnable(uint8_t e1, uint8_t e2);

/**
 * get motor current for motor 0 or 1.
 * NB! no test for valid index.
 * \returns current in amps */
// inline float getMotorCurrent(int m)
// {
//   if (directionFWD[m])
//     return -float(motorCurrent[m]) * 1.2 / useADCMaxADCValue / 0.525;
//   else
//     return  float(motorCurrent[m]) * 1.2 / useADCMaxADCValue / 0.525;
// }

inline float getMotorCurrentM(int m, uint16_t value)
{ // sensor: 2.5V (5V/2) is zero and 185mV/A
  // offset to zero about 0.7V and still 185mV/A
  // A/D max=1.2V 10bit
  const float scale = 1.2 / useADCMaxADCValue / 0.185 ; // 185 mV/A
  if (m == 0)
    return float(value - motorCurrentMOffset[0]) * scale;
  else
    // right motor runs backwards, so current is negative,
    // change sign so that forward requires positive current
    return -float(value - motorCurrentMOffset[1]) * scale;
}

/**
 * interrupt for motor encoder */
void m1EncoderA();
void m2EncoderA();
void m1EncoderB();
void m2EncoderB();


#endif // MOTOR_CONTROLLER_H
