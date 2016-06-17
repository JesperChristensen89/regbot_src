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

#include <stdint.h>

#ifndef DATA_LOGGER_H2
#define DATA_LOGGER_H2
// Initialization
//void loggerInit(int interval, int samples);

#define LOG_BUFFER_MAX 32000
// data logging buffer
typedef enum
{
  LOG_TIME = 0,
  LOG_MISSION,
  LOG_ACC,
  LOG_GYRO,
//   LOG_MAG,
  LOG_MOTV_REF,
  LOG_MOTV,
  LOG_MOTA, 
  LOG_ENC,
  LOG_WHEELVEL,
  LOG_TURNRATE,
  LOG_POSE,
  LOG_LINE,
  LOG_DIST,
  LOG_BATT,
  LOG_CTRLTIME,
  LOG_BAL_CTRL,
  LOG_EXTRA,
  LOG_MAX_CNT
} logItem;
// size code    f=float, d=double (64 bit), i=int8_t, I=uint8_t, j=int16_t, J=uint16_t, k=int32_t, K=uint32
#define LOG_INT8   'i'
#define LOG_UINT8  'I'
#define LOG_INT16  'j'
#define LOG_UINT16 'J'
#define LOG_INT32  'k'
#define LOG_UINT32 'K'
#define LOG_FLOAT  'f'
#define LOG_DOUBLE 'd'
//

extern int logRowCnt;
extern int logRowsCntMax;
extern char logRowFlags[];
// 1,2.. = send status or log at this sample interval count
extern int logInterval; 
extern bool logAllow;

//extern char rxbuffer[];
extern int rxbufferCnt;
extern int rxCharCnt;

extern uint32_t timeAtMissionStart;
extern bool toLog;
extern int8_t logBuffer[LOG_BUFFER_MAX];

/**
 * Start logging with current log flags.
 * \param logInterval 0: unchanged, else set to this number of milliseconds
 * */
void startLogging(int loginterval);
void stopLogging(void);
inline bool loggerLogging() 
{
  return toLog;
}
/**
 * Save requested data tolog buffer */
void stateToLog();
/**
 * Mission init is called before and sets the log default values
 * battery voltage. */
void setLogFlagDefault();

/**
 * init log structure after changing any of the log flags */
void initLogStructure(int timeFactor);
/**
 * add data for this item
 * \param item is the data logger object (ACC, GYRO, pose ...)
 * \param data is a pointer to the data to log
 * \param dataCnt is the number of bytes to log */
void addToLog(logItem item, void * data, int dataCnt);
/**
 * start new log row
 * \returns true if space for one more row */
bool addNewRowToLog();
// {
//   if (logRowCnt >= logRowsCntMax - 1)
//     return false;
//   logRowCnt++;
//   return true;
// }

/**
 * should this item be logged
 * \param item
 * \returns true if item is to be logged */
inline bool logThisItem(logItem item)
{
  return logRowFlags[item];
}

/**
 * Set size and type of data logger item
 * \param item is logger item
 * \param count is number of values to log for this item
 * \param type id data type to be logged - implicit gives the byte size of each value
 */
void setLogSize(logItem item, int count, char type);
/**
 * Transfer log to USB connection */
int logWriteBufferTo(int row);
/**
 * read communication from data logger to a buffer
 * \returns true if a \\n or a \\r is detected
 * data is available in rxBuffer and there is rxBufferCnt characters */
bool loggerRead();

void eePromSaveStatusLog();
void eePromLoadStatusLog();



#endif // DATA_LOGGER_H