/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * read and save configuration as string
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

#ifndef REGBOT_EESTRING_H
#define REGBOT_EESTRING_H

#include <string.h>
#include <stdlib.h>
#include "main.h"

class EEConfig
{
public:
  // constructor
  EEConfig();
  /** send configuration to USB
   * \param configBuffer set to NULL to use the just saved configuration or to another configuration to fetch.
   * Sends the configuration as byte hex code  */
  void stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength);
  /**
   * is stringbuffer in use */
  bool isInUse()
  {
    return stringConfig;
  }
  /** set in use flag and clear buffer */
  inline void setStringBuffer(uint8_t * string2kBuffer,  bool initializeEEprom)
  {
    config = string2kBuffer;
    configAddr = 0;
    configAddrMax = 0;
    if (initializeEEprom)
      eeprom_initialize();
  }
  inline void clearStringBuffer()
  {
    config = NULL;
  }
  /* get 2k config buffer pointer */
//   uint8_t * get2KConfigBuffer()
//   {
//     return config;
//   }
  /**
   * Load configuration from either a config string or from eeProm (flash) 
   * dependent on the "stringConfig" flag
   * \param from2Kbuffer if true, then load from string buffer (must be loaded first), if false, then read for eeprom (flask).
   * */
  void eePromLoadStatus(bool from2Kbuffer);
  /**
   * Save configuration to eeProm (flash) or sent configuration to USB in hex format.
   * \param toUSB set to true to read to USB, or false to save to eeProm */
  void eePromSaveStatus(bool toUSB);
  
public:
  /** save a 32 bit value */
  void push32(uint32_t value);
  /** save a byte */
  void pushByte(uint8_t value);
  /** save a word in configuration stack */
  void pushWord(uint16_t value);
  /** get a 32 bit integer from configuration stack */
  uint32_t read32();
  /** get a byte from configuration stack */
  uint8_t readByte();
  /** get a 16 bit integer from configuration stack */
  uint16_t readWord();
  /** save a 32 bit float to configuration stack */
  inline void pushFloat(float value)
  {
    union {float f; uint32_t u32;} u;
    u.f = value;
    push32(u.u32);
  }
  // read 32 bit as float from configuration stack
  inline float readFloat()
  {
    union {float f; uint32_t u32;} u;
    u.u32 = read32();
    return u.f;  
  }
  /** write a word to a specific place in configuration stack
   * typically a size that is not known before pushing all the data */
  inline void write_word(int adr, uint16_t v)
  {
    if (not stringConfig)
      eeprom_write_word((uint16_t*)adr, v);
    else if (config != NULL)
    {
      memcpy(&config[adr], &v, 2);
    }
    else
      usb_send_str("# failed to save word\n");
    if (adr > configAddr - 2)
      configAddr = adr + 2;
  }
  /**
   * a busy wit if the flash write system is busy */
  inline void busy_wait()
  {
    if (not stringConfig)
    {
      eeprom_busy_wait();
    }
  }
  /** push a block of data to the configuration stack */
  inline void write_block(const char * data, int n)
  {
    if (not stringConfig)
    {
      eeprom_write_block(data, (void*)configAddr, n);
    }
    else
    {
      memcpy(&config[configAddr], data, n);
    }
    configAddr += n;
  }
  /** set the adress for the next push or read operation on the configuration stack */
  void setAddr(int newAddr)
  {
    configAddr = newAddr;
  }
  /** get the address of the next push or read operation on the configuration stack */
  int getAddr()
  {
    return configAddr;
  }
  /**
   * Implement one of the hard-coded configurations 
   * \param hardConfigIdx is index to the hardConfig array, as defined in eeconfig.h and set in the constructor.
   * \param andToUsb is a debug flag, that also will return the just loaded configuration to the USB
   * */
  bool hardConfigLoad(int hardConfigIdx, bool andToUsb);
  
protected:
  /**
   * Get hard coded configuration string */
  int getHardConfigString(uint8_t * buffer, int configIdx);
  
private:
  /** full configuration buffer, as real eeProm 
   * is either NULL or points to a 2048 byte array */
  uint8_t * config;
  /** max number of bytes in string buffer */
//   static const int sbufMaxCnt = 64;
//   /** string buffer for data reply to client */
//   uint8_t sbuf[sbufMaxCnt];
  /** number of bytes written to string buffer */
  int sbufCnt;
  /** is string buffer in use - else flash is in use */
  bool stringConfig;
  /** current read/write adress in config array */
  int configAddr;
  /** highest number used in configuration in config array */
  int configAddrMax;
  /** hard coded configuration */
  const char * hardConfigFollowWall = 
  "#cfg00:f7 01 00 00 d2 76 00 00 2f 00 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c ec 51 38 3d\
  #cfg01:01 c0 13 1e 00 00 00 d7 ff ff ff fe ff ff ff 01 01 00 00 01 01 00 00 01 00 01 00 00 01 00 01 00\
  #cfg02:00 00 00 0a 00 00 00 01 00 00 00 03 01 00 00 a0 41 cd cc 4c 3e 00 00 00 00 00 00 00 00 00 00 80\
  #cfg03:3f 00 00 80 3f 00 00 00 3f 33 33 33 3f 01 01 00 00 00 41 01 00 00 a0 40 00 00 00 00 00 00 00 00\
  #cfg04:00 00 00 00 00 00 00 00 9a 99 99 3e cd cc cc 3d 00 00 80 3f 00 cd cc cc 3e 01 00 00 a0 40 00 00\
  #cfg05:00 00 50 8d 17 3d cd cc cc 3d 00 00 a0 40 00 00 a0 40 00 00 80 3f 00 00 80 40 cd cc 4c 3d cd cc\
  #cfg06:4c 3e 00 00 00 40 00 01 00 00 c0 c0 29 5c 8f 3d 1b 2f dd 3c cd cc 4c 3e 00 00 80 3f 00 00 c0 40\
  #cfg07:00 00 a0 40 00 00 00 00 9a 99 99 3e 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg08:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3e 00 00 80 3f 42 60 e5 3b\
  #cfg09:00 00 20 41 00 00 20 41 cd cc 4c 3d cd cc 4c 3e 01 04 00 00 a0 40 00 00 00 00 00 00 00 00 00 00\
  #cfg10:00 00 00 00 00 00 00 00 00 00 cd cc 4c 3e cd cc cc 3e 00 00 00 3f 00 00 00 3f 05 00 00 80 40 00\
  #cfg11:00 00 00 cd cc cc 3d cd cc cc 3d 00 00 00 00 cd cc cc 3e 00 00 00 40 cd cc 4c 3e 00 00 80 3e 00\
  #cfg12:00 80 3f 53 00 61 30 2c 65 31 3a 42 3d 33 0a 61 30 2e 32 37 2c 64 35 2c 69 31 3a 48 3c 30 2e 32\
  #cfg13:35 2c 49 3c 30 2e 34 0a 63 30 3a 43 3d 34 35 2c 49 3e 30 2e 34 0a 61 30 2e 32 37 2c 6b 31 2c 6c\
  #cfg14:30 2e 31 38 3a 48 3e 30 2e 35 0a 6d 31 3a 0a 61 30 3a 42 3d 31 0a 00 64 00 64 00 65 00 65 00 66\
  #cfg15:00 66 00 67 00 67 00 68 00 68 00 69 00 69 00 6a 00 6a 00 6b 00 6b 00";
  const char * hardConfigFactoryReset = 
  "#cfg00:ad 01 00 00 d2 76 00 00 2f 00 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c ec 51 38 3d\
  #cfg01:01 c0 13 1c 00 00 00 da ff ff ff fb ff ff ff 01 01 00 00 01 01 01 00 01 00 01 00 00 01 00 00 00\
  #cfg02:00 00 00 01 00 00 00 01 00 00 00 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c4\
  #cfg03:42 9a 99 99 3e 66 66 06 40 33 33 83 40 01 01 00 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg04:00 00 00 00 00 00 c2 42 00 00 00 00 00 00 00 00 00 00 00 00 01 00 80 95 43 00 00 00 00 00 00 00\
  #cfg05:00 00 00 00 00 00 00 00 00 00 00 00 c2 42 00 80 95 43 00 00 00 00 00 00 00 00 00 00 80 3f 00 00\
  #cfg06:40 40 00 00 00 40 01 00 00 00 c8 42 00 00 00 00 cd cc 4c 3d cd cc 4c 3e 00 00 be 42 00 00 47 43\
  #cfg07:00 00 00 3f 00 00 00 00 00 00 00 00 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg08:00 00 00 00 00 96 43 00 00 00 3f 00 00 00 00 cd cc 4c 3e 00 9a 99 99 3e 00 00 00 00 cd cc cc 3d\
  #cfg09:f4 fd 34 3f 9a 99 99 3e 00 00 00 00 9a 99 99 3e 01 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg10:00 00 00 00 c2 42 00 80 95 43 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 00 00 80 3f 00\
  #cfg11:00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f 00 80 95 43 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg12:00 00 00 09 00 61 31 3a 42 3d 32 0a 00 64 00 64 00 65 00 65 00 66 00 66 00 67 00 67 00 68 00 68\
  #cfg13:00 69 00 69 00 6a 00 6a 00 6b 00 6b 00";
  const char * hardConfigBalanceStep = 
  "#cfg00:f7 01 00 00 d2 76 00 00 2f 00 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c ec 51 38 3d\
  #cfg01:01 c0 13 1e 00 00 00 d7 ff ff ff fe ff ff ff 01 01 00 00 00 00 00 00 01 00 01 00 00 01 00 00 00\
  #cfg02:00 00 00 14 00 00 00 01 00 00 00 02 01 00 00 a0 41 cd cc 4c 3e 00 00 00 00 00 00 00 00 00 00 80\
  #cfg03:3f 00 00 80 3f 00 00 00 3f 33 33 33 3f 01 01 00 00 00 41 01 00 00 a0 40 00 00 00 00 00 00 00 00\
  #cfg04:00 00 00 00 00 00 00 00 9a 99 99 3e cd cc cc 3d 00 00 80 3f 00 cd cc cc 3e 01 00 00 a0 40 00 00\
  #cfg05:00 00 50 8d 17 3d cd cc cc 3d 00 00 a0 40 00 00 a0 40 00 00 80 3f 00 00 80 40 cd cc 4c 3d cd cc\
  #cfg06:4c 3e 00 00 00 40 00 01 00 00 c0 c0 29 5c 8f 3d 1b 2f dd 3c cd cc 4c 3e 00 00 80 3f 00 00 c0 40\
  #cfg07:00 00 a0 40 00 00 00 00 9a 99 99 3e 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg08:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3e 00 00 80 3f 42 60 e5 3b\
  #cfg09:00 00 20 41 00 00 20 41 cd cc 4c 3d cd cc 4c 3e 01 04 00 00 a0 40 00 00 00 00 00 00 00 00 00 00\
  #cfg10:00 00 00 00 00 00 00 00 00 00 cd cc 4c 3e cd cc cc 3e 00 00 00 3f 00 00 00 3f 05 00 00 80 40 00\
  #cfg11:00 00 00 cd cc cc 3d cd cc cc 3d 00 00 00 00 cd cc cc 3e 00 00 00 40 cd cc 4c 3e 00 00 80 3e 00\
  #cfg12:00 80 3f 53 00 61 30 2c 65 31 3a 42 3d 33 0a 61 30 2e 32 37 2c 64 35 2c 69 31 3a 48 3c 30 2e 32\
  #cfg13:35 2c 49 3c 30 2e 34 0a 63 30 3a 43 3d 34 35 2c 49 3e 30 2e 34 0a 61 30 2e 32 37 2c 6b 31 2c 6c\
  #cfg14:30 2e 31 38 3a 48 3e 30 2e 35 0a 6d 31 3a 0a 61 30 3a 42 3d 31 0a 00 64 00 64 00 65 00 65 00 66\
  #cfg15:00 66 00 67 00 67 00 68 00 68 00 69 00 69 00 6a 00 6a 00 6b 00 6b 00";
  const char * hardConfigWallWallAlign=
  "#cfg00:61 02 00 00 ff 77 00 00 2f 00 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c ec 51 38 3d\
  #cfg01:01 c0 13 1e 00 00 00 d7 ff ff ff fe ff ff ff 01 01 00 00 00 00 00 00 01 00 01 00 00 01 00 00 00\
  #cfg02:00 00 00 14 00 00 00 01 00 00 00 03 01 00 00 a0 41 cd cc 4c 3e 00 00 00 00 00 00 00 00 00 00 80\
  #cfg03:3f 00 00 80 3f 00 00 00 3f 33 33 33 3f 01 01 00 00 00 41 01 00 00 a0 40 00 00 00 00 00 00 00 00\
  #cfg04:00 00 00 00 00 00 00 00 9a 99 99 3e cd cc cc 3d 00 00 80 3f 00 cd cc cc 3e 01 00 00 a0 40 00 00\
  #cfg05:00 00 50 8d 17 3d cd cc cc 3d 00 00 a0 40 00 00 a0 40 00 00 80 3f 00 00 80 40 cd cc 4c 3d cd cc\
  #cfg06:4c 3e 00 00 00 40 00 01 00 00 c0 c0 29 5c 8f 3d 1b 2f dd 3c cd cc 4c 3e 00 00 80 3f 00 00 c0 40\
  #cfg07:00 00 a0 40 00 00 00 00 9a 99 99 3e 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg08:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3e 00 00 80 3f 42 60 e5 3b\
  #cfg09:00 00 20 41 00 00 20 41 cd cc 4c 3d cd cc 4c 3e 01 04 00 00 a0 40 00 00 00 00 00 00 00 00 00 00\
  #cfg10:00 00 00 00 00 00 00 00 00 00 cd cc 4c 3e cd cc cc 3e 00 00 00 3f 00 00 00 3f 05 00 00 80 40 00\
  #cfg11:00 00 00 cd cc cc 3d cd cc cc 3d 00 00 00 00 cd cc cc 3e 00 00 00 40 cd cc 4c 3e 00 00 80 3e 00\
  #cfg12:00 80 3f bd 00 61 2d 30 2e 32 2c 69 31 3a 42 3d 31 2e 35 0a 61 30 2e 32 3a 41 3d 30 2e 32 0a 65\
  #cfg13:31 3a 41 3d 30 2e 33 0a 63 30 3a 43 3d 2d 32 37 30 0a 3a 41 3d 30 2e 31 0a 61 2d 30 2e 34 3a 41\
  #cfg14:3d 30 2e 32 35 2c 42 3d 33 0a 61 30 2e 31 3a 42 3d 30 2e 35 0a 61 2d 30 2e 32 2c 65 30 3a 42 3d\
  #cfg15:33 0a 61 30 3a 42 3d 32 0a 61 30 2e 32 3a 41 3d 30 2e 33 0a 65 31 3a 41 3d 30 2e 33 0a 63 30 3a\
  #cfg16:43 3d 32 37 30 0a 3a 41 3d 30 2e 31 0a 61 2d 30 2e 34 3a 41 3d 30 2e 33 2c 42 3d 33 0a 61 30 2e\
  #cfg17:31 3a 42 3d 30 2e 35 0a 61 2d 30 2e 33 2c 65 30 3a 42 3d 32 0a 61 30 3a 42 3d 35 0a 6d 31 3a 0a\
  #cfg18:00 64 00 64 00 65 00 65 00 66 00 66 00 67 00 67 00 68 00 68 00 69 00 69 00 6a 00 6a 00 6b 00 6b\
  #cfg19:00";
  const char * hardConfigSquare = 
  "#cfg00:79 02 00 00 ff 77 00 00 2f 00 52 b8 1e 3e 48 e1 1a 41 30 00 6a 4d f3 3c 6a 4d f3 3c ec 51 38 3d\
  #cfg01:01 c0 13 1e 00 00 00 d7 ff ff ff fe ff ff ff 01 01 00 00 00 00 00 00 01 00 01 00 00 01 00 00 00\
  #cfg02:00 00 00 42 00 00 00 01 00 00 00 03 01 00 00 a0 41 cd cc 4c 3e 00 00 00 00 00 00 00 00 00 00 80\
  #cfg03:3f 00 00 80 3f 00 00 00 3f 33 33 33 3f 01 01 00 00 00 41 01 00 00 a0 40 00 00 00 00 00 00 00 00\
  #cfg04:00 00 00 00 00 00 00 00 9a 99 99 3e cd cc cc 3d 00 00 80 3f 00 cd cc cc 3e 01 00 00 a0 40 00 00\
  #cfg05:00 00 50 8d 17 3d cd cc cc 3d 00 00 a0 40 00 00 a0 40 00 00 80 3f 00 00 80 40 cd cc 4c 3d cd cc\
  #cfg06:4c 3e 00 00 e0 40 00 01 00 00 c0 c0 29 5c 8f 3d 1b 2f dd 3c cd cc 4c 3e 00 00 80 3f 00 00 c0 40\
  #cfg07:00 00 a0 40 00 00 00 00 9a 99 99 3e 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg08:00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3e 00 00 80 3f 42 60 e5 3b\
  #cfg09:00 00 20 41 00 00 20 41 cd cc 4c 3d cd cc 4c 3e 01 04 00 00 a0 40 00 00 00 00 00 00 00 00 00 00\
  #cfg10:00 00 00 00 00 00 00 00 00 00 cd cc 4c 3e cd cc cc 3e 00 00 00 3f 00 00 00 3f 05 00 00 80 40 00\
  #cfg11:00 00 00 cd cc cc 3d cd cc cc 3d 00 00 00 00 cd cc cc 3e 00 00 00 40 cd cc 4c 3e 00 00 80 3e 00\
  #cfg12:00 80 3f d5 00 61 30 2e 33 2c 62 32 2c 64 36 36 3a 42 3d 30 2e 31 0a 69 36 3a 0a 3a 41 3d 30 2e\
  #cfg13:34 0a 63 30 2e 32 3a 43 3d 2d 39 30 0a 6d 36 3a 44 3d 33 0a 61 30 2e 32 2c 65 31 2c 69 37 3a 0a\
  #cfg14:3a 41 3d 30 2e 35 0a 63 30 2e 30 35 3a 43 3d 2d 39 30 0a 6d 37 3a 44 3d 33 0a 61 30 2c 62 37 3a\
  #cfg15:42 3d 35 0a 61 30 2e 32 35 3a 41 3d 30 2e 35 0a 61 2d 30 2e 32 2c 69 38 3a 0a 3a 41 3d 30 2e 34\
  #cfg16:0a 63 30 2e 30 35 3a 43 3d 2d 39 30 0a 6d 38 3a 44 3d 33 0a 61 30 2e 32 3a 42 3d 30 2e 31 0a 61\
  #cfg17:2d 30 2e 32 2c 65 30 2c 69 39 3a 0a 3a 41 3d 30 2e 35 0a 63 30 2e 32 3a 43 3d 2d 39 30 0a 6d 39\
  #cfg18:3a 44 3d 33 0a 3a 41 3d 30 2e 35 0a 61 30 2c 62 37 3a 42 3d 30 2e 35 0a 00 64 00 64 00 65 00 65\
  #cfg19:00 66 00 66 00 67 00 67 00 68 00 68 00 69 00 69 00 6a 00 6a 00 6b 00 6b 00";
  static const int hardConfigCnt = 5;
  const char * hardConfig[hardConfigCnt];
};

/**
 * Instans af ee og string config */
extern EEConfig eeConfig;


#endif