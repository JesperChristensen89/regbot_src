 /***************************************************************************
 * definition file for the regbot.
 * The main function is the controlTick, that is called 
 * at a constant time interval (of probably 1.25 ms, see the intervalT variable)
 * 
 * The main control functions are in control.cpp file
 * 
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
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

#ifndef REGBOT_CONTROL_H
#define REGBOT_CONTROL_H

#include <stdint.h>
#include "main.h"
 
// mission time in seconds, is reset when mission state is 0
//extern float time; 
// motor ctrl - is the output to the motor controller in range -2.0 to 2.0 - positive is forward
//extern float motorCtrl[2]; 
// idle means that motor controller has released the motor terminals (no breaking effect)
//extern bool  motorIdle;
/// Button state - is 0 (false) for not pressed and 1 (true) for pressed.
/// make state visible to e.g. data logger
extern int16_t missionState; 
/// mission number
extern int mission;
extern const int missionMax;
extern int16_t misLineNum;
extern float mission_vel_ref;
// reference for motor controller
extern float regul_vel_tot_ref[2];
extern float regul_turn_vel_reduc[2];
extern float regVelULeft[];
extern float regVelURight[];
extern float regVelUILeft[];
extern float regVelUIRight[];
extern float regul_balvel_reduc;
extern float mission_turn_ref;
extern float regTurnE[2];
extern float regTurnU[];

extern float regBalE[2];
extern float regBalU[];
extern float regBalUI[];
extern float regBalUD[];
extern float regBalVelE[];
extern float regBalVelU[];
extern float regBalVelUI[];
extern float regBalVelUD[];
extern float balTiltRef; // reference value for tilt controller (for data logger)
extern float regul_bal_uvel; // out of tilt controller

extern float regTVE[2];
extern float regTVU[];
extern float regTVUI[2];
//extern float balTVRef; // reference value for tilt velocity controller (for data logger)
// line sensor control
extern bool regul_line_followLeft;

// debug values for controller
extern float regVelELeft[], regVelUILeft[], regVelUDLeft[];
/**
 * This function is called every time it is time to do a new control calculation
 * Before the call the motor control values (motorCtrl) are set to 0.0 indication zero speed
 * To make the motor go, the motor Idle must be set to false (0), and
 * The motorCtrl value(s) set to a value between +1.0 and -1.0, positive for forward.
 * In short intervals the motorCtrl values may go as high as +2.0 to -2.0, this will
 * give the motor up to +/- 12 V - twice the voltage it is rated for. This could be used for 
 * fast acceleration. 
 * The new motor values are implemented once this functon returns. 
 * */ 
void controlTick(void); 
 
//////////////////////////////////////////////////////////////
/*
  support functions 
*/

/**
 * Support function to get battry voltage - this is updated about 10 times a second only.
 * \returns value in Volts. */
float getBatVoltage();

/**
 * Start data logger to RAM (continues until no more space (40kB reserved for log) 
 * \param loginterval is number of control ticks between each log entry
 *                    if set to 0, then log interval is unchanged */
void startLogging(int loginterval);
/**
 * Tests if logger is actually logging,
 * \returns false if log buffer is full */
bool loggerLogging();
/**
 * send all logged data to SD card - finished after a while - maybe 20 seconds - wait until light goes back ti idle */
void logSendToSD();

/**
 * send mission status if any of the data has changed */
void sendMissionStatusChanged();
/**
 * send status about control */
void sendStatusControl();
/**
 * send status about regulators */
bool setRegulator(const char * line);
/**
 * save controller configuration to EE Prom */
void eePromSaveCtrl();
/**
 * load controller configuration from EE Prom
 * same order as when saved
 */
void eePromLoadCtrl();

#endif
