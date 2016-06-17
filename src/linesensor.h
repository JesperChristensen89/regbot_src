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

extern int16_t adcLSH[8];
extern int16_t adcLSL[8];
extern bool lsIsWhite;
extern bool lsPowerHigh;
extern bool lsPowerAuto;

/**
 * Send line sensor difference values */
void sendStatusLineSensor();
/**
 * Send status for aAD converter values directly */
void sendADLineSensor();
/**
 * set linesensor parameters from GUI.
 * \param buf is the command string
 * \returns true if used */
bool setLineSensor(const char * buf);
/**
 * estimate edges of line */
void findLineEdge(void);

/** save line sensor calibration */
void eePromSaveLinesensor();
/** load line sensor calibration */
void eePromLoadLinesensor();

/**
 * Use line sensor */
extern bool lineSensorOn;
/**
 * difference between illuminated and not, normalized between calibration values */
extern float lineSensorValue[8];
/**
 * Line sensor result */
extern float lsLeftSide;
extern float lsRightSide;
extern bool lsLeftValid;
extern bool lsRightValid;
extern bool crossingWhiteLine;
extern bool crossingBlackLine;
extern int16_t crossingWhiteCnt;
extern int16_t crossingBlackCnt;

/**
 * line color
 * */
extern bool lsIsWhite;

