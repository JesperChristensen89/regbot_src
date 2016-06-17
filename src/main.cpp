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
 
#define REV_MAIN 2
#define REV "$Rev: 308 $"
#define REV_MINOR 19

#define REV_ID "$Id: main.cpp 308 2016-03-20 17:31:04Z jcan $" 


//#include "arm_math.h"
#include <malloc.h>
#include "IntervalTimer.h"
#include "mpu9150.h"
#include "motor_controller.h"
#include "serial_com.h"
#include "data_logger.h"
#include "control.h"
//#include "baro180.h"
#include "robot.h"
#include "main.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
//#include "pins_arduino.h"
//#include <ADC.h>
//#include <DMAChannel.h>
#include <ADC.h>
#include <../Snooze/Snooze.h>
//#define __MK20DX256__

int robotId = 0;
/** main heartbeat timer to service source data and control loop interval */
IntervalTimer hbTimer;
float time = 0.0; // system time in seconds
uint32_t timeAtMissionStart = 0;
const char * lfcr = "\n\r";
bool pushToUSB = 1;
//bool logToSD = 0;  // no SD-card anymore
bool logToUSB = 0;
// reply from ardulog send back to usb connection
bool loggerReplyToUSB = false;
// should teensy echo commands to usb
bool localEcho = 0;
// stop sending status to usb after this count
int pushCnt = 30;
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
int batLowCnt = 0;
uint16_t batVoltInt = 0;
//float batVoltIntToFloat = 1.2 / useADCMaxADCValue * (15.0 + 1.2)/1.2;
// flag reset at when motor is enabled first time
bool motorPreEnabled = 1;
uint32_t motorCurrentMLowPass[2];
//
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
// flag for start of new control period
volatile bool startNewCycle = false;
uint16_t controlUsedTime[2] = {0,0};
int pushInterval = 0;
int pushTimeLast = 0;
int pushHBlast = 0;
int pushStatus = 0;
// line sensor full or reduced power (output is high power)
bool pinMode18 = OUTPUT;
//
// EEprom adress - max 2kb (512 words)
//uint32_t eePushAdr = 0;

// mission stop and start
bool button;
uint16_t buttonCnt;
bool missionStart = false;
bool missionStop = false;
bool sendStatusWhileRunning = false;
/**
 * usb command buffer space */
#define RX_BUF_SIZE 110
char usbRxBuf[RX_BUF_SIZE];
int usbRxBufCnt = 0;
int usb_not_idle_cnt = 0;

/**
 * uart command buffer space */
char serial1RxBuf[RX_BUF_SIZE];
int serial1BufCnt = 0;

/**
* vision-project globals */
// holds distance to both gmk and to leader robot when needed
float distanceToGmk = 0;
// holds angle to gmk and to leader robot
float angleToGmk = 0;
// flag to sign ready between follower and raspi
int rdyToProceed = 0;
// not used
int tooClose = 0;
// sets the followers speed
float followVelocity = 0;
// sets the turn radius of leader
float raspiTurnRadius = 0.0;
// flag to signal if wifi is ready to receive commands
bool isWifiReady = false;
// state controlled wifi setup case-holder
int wifiSetupCounter = 0;
// is wifi setup
bool wifiIsSet = false;
// flag to signal if wifi got ip
bool wifiGotIp = false;
// flag to signal a client connected to leaders tcp server
bool wifiClientConnected = false;
// holds the override distance for follower
double overrideDistance = 0;
// holds the override angle for follower
double overrideAngle = 0;

/**
 * Heart beat interrupt service routine */
void hbIsr(void);
/**
 * write to I2C sensor (IMU) */
int writeSensor(uint8_t i2caddr, int reg, int data);
/**
 * Parse commands from the USB connection and implement those commands.
 * The function is served by the main loop, when time allows. */
void parse_and_execute_command(const char *buf, uint8_t num);
/**
 * log state to ram-buffer
 * - stops when no more space */
void stateToLog();
/**
 * send state as text to USB (direct readable) */
void stateToUsb();
/// send push status
void sendHartBeat();
void sendStatusSensor();
void sendStatusLogging();
void sendStatusVersion();
/// eeprom functions
void eePromSaveStatus(uint8_t * configBuffer);
void eePromLoadStatus(const uint8_t * configBuffer);

/// testcode
void processingTimeTest();
/// debug power save routine
void stopTeensy(void);


typedef enum {BARO_TEMP_START, BARO_TEMP_WAIT, BARO_PRES_WAIT} UBaroMeasState;
UBaroMeasState baroMeasState = BARO_TEMP_START;

ADC * adc = new ADC();
//uint16_t adcErr0[MAX_ADC] = {0,811,822,833,844,855,866,877, 888, 899, 800, 811, 8};
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
uint16_t adcStartCnt = 0, adcHalfCnt = 0;
const int LS_FIRST = 5;
uint16_t * adcDest[LS_FIRST] = {&irRaw[0], &irRaw[1], &batVoltInt, &motorCurrentM[0], &motorCurrentM[1]};
int adcPin[MAX_ADC] = {1, 0, 9, 10, 11, 12, 13, 26, 27, 28, 29, 30, 31};
int adcSeq = 0; 
bool adcHalf; // for double ADC conversion for LS
uint32_t adcStartTime, adcConvertTime;
uint32_t adcHalfStartTime, adcHalfConvertTime;
///
int imuReadFail = 0;


/**
 * Get SVN revision number */
uint16_t getRevisionNumber()
{
  return strtol(&REV[5], NULL, 10) * 100 + REV_MINOR;
}

////////////////////////////////////////////

void sendStatusLS()
{
  const int MRL = 250;
  char reply[MRL];
  //                       #1 #2   LS0    LS1    LS2    LS3    LS4    LS5    LS6    LS7    timing (us)
  snprintf(reply, MRL, "ls %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %ld %ld\r\n",
           adcStartCnt, adcHalfCnt, 
           adcLSH[0], adcLSL[0], adcLSH[1], adcLSL[1], adcLSH[2], adcLSL[2], adcLSH[3], adcLSL[3], 
           adcLSH[4], adcLSL[4], adcLSH[5], adcLSL[5], adcLSH[6], adcLSL[6], adcLSH[7], adcLSL[7], 
           adcConvertTime * 10, adcHalfConvertTime * 10);
  usb_send_str(reply);
}

////////////////////////////////////////////

void sendStatusCurrentVolt()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "VA %d %.3f  %d %d %.3f  %d %d %.3f\r\n",
           batVoltInt, batVoltInt * batVoltIntToFloat, 
           motorCurrentM[0], motorCurrentMOffset[0], getMotorCurrentM(0, motorCurrentM[0]),
           motorCurrentM[1], motorCurrentMOffset[1], getMotorCurrentM(1, motorCurrentM[1])
          );
  usb_send_str(reply);
}

//////////////////////////////////////////////////////

/**
 * Test all I2C adresses and print reply. */
void testAddr(void)
{
  int ak;
  const int MRL = 100;
  char reply[MRL];
  for (int i= 0; i < 0x7f; i++)
  {
    Wire.beginTransmission(i);
    Wire.write(0);
    ak = Wire.endTransmission(I2C_STOP,1000);
    snprintf(reply, MRL, "addr test addr %d (%x) gave %d", i, i, ak);
    usb_send_str(reply);
  }
}

//////////////////////////////////////////


void initialization()
{ // INITIALIZATION
  pinMode(LED_BUILTIN, OUTPUT);
  // init USB connection (parameter is not used - always 12MB/s
  Serial.begin(115200); 
  // init serial to ardulog
  Serial1.begin(115200);    // Ardulog does - for some reason - not function at baud rate beyond 9600.
// init IMU
  //delay(100);             // Required for MPU to get out of sleep mode after restart
  // init motor control
  motorInit();           // init ports PWM out, direction out, encoder in
  // init AD converter
  if (useADCInterrupt)
  { // AD using interrupt
    adc->adc0->setReference(ADC_REF_1V2);
    adc->adc1->setReference(ADC_REF_1V2);
    // not needed
    adc->adc0->recalibrate();
    adc->adc1->recalibrate();
    adc->setResolution(12, 0);
    adc->setResolution(12, 1);
    adc->setConversionSpeed(ADC_MED_SPEED, 0);
    adc->setConversionSpeed(ADC_MED_SPEED, 1);
  }
  else
  { // AD poll
    analogReference(INTERNAL);
  }
  // more pins
  pinMode(6,INPUT); // start switch - version 2B
  pinMode(23,INPUT); // battery voltage
  pinMode(11,INPUT); // start switch - version 2A
  pinMode(18,OUTPUT); // line sensor LED full power
  pinMode(32,OUTPUT); // line sensor LED half power
  pinMode18 = OUTPUT;
  //
  pinMode(A12,INPUT); // Line sensor sensor value
  pinMode(A13,INPUT); // Line sensor sensor value
  pinMode(A15,INPUT); // Line sensor sensor value
  pinMode(A16,INPUT); // Line sensor sensor value
  pinMode(A17,INPUT); // Line sensor sensor value
  pinMode(A18,INPUT); // Line sensor sensor value
  pinMode(A19,INPUT); // Line sensor sensor value
  pinMode(A20,INPUT); // Line sensor sensor value
  lineSensorOn = false;
  digitalWriteFast(18, lineSensorOn);
  
  // start 10us timer (heartbeat timer)
  hbTimer.begin(hbIsr, (unsigned int)  10); // heartbeat timer, value in usec
  //
  // data logger init
  setLogFlagDefault();
  initLogStructure(100000 / CONTROL_PERIOD_10us);
  // init encoder interrupts
  attachInterrupt(M1ENC_A, m1EncoderA, CHANGE);
  attachInterrupt(M2ENC_A, m2EncoderA, CHANGE);
  attachInterrupt(M1ENC_B, m1EncoderB, CHANGE);
  attachInterrupt(M2ENC_B, m2EncoderB, CHANGE);
  //
  // I2C init
  // Setup for Master mode, pins 18/19, external pullups, 400kHz
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  // Initialization of MPU9150
  digitalWriteFast(LED_BUILTIN,1);  
  imuAvailable = MPU9150_init();
  digitalWriteFast(LED_BUILTIN,0);
  if (not imuAvailable)
    usb_send_str("# failed to find IPU\r\n");  
  // baro180_init
  //   if (true)
  //     baro180_init();
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.eePromLoadStatus(NULL);
  //
  if (imuAvailable)
  {
    if (true)
    {
      bool isOK = mpuRequestData();
      if (not isOK)
        usb_send_str("#first ACC read request failed\r\n");
      isOK = mpuReadData();
      if (not isOK)
        usb_send_str("#first ACC read failed (too)\r\n");
    }
    else
    {
      int a = readAccGyro();
      if (a != 0)
        usb_send_str("#first ACCGyro failed\r\n");
    }
    readMagnetometer();
  }
  // feedback value from motor controller
  //   motorCurrent[0] = analogRead(AnalogA0);
  //   motorCurrent[1] = analogRead(AnalogA1);
  // from current-sensor
  if (useADCInterrupt)
  { // initialize analogue values
    motorCurrentM[0] = adc->analogRead(10);
    motorCurrentM[1] = adc->analogRead(11);
    batVoltInt = adc->analogRead(23);
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = adc->analogRead(10) * 100;
    motorCurrentMLowPass[1] = adc->analogRead(11) * 100;
    // enable interrupt for the remaining ADC oprations
    adc->enableInterrupts(0);
    adc->enableInterrupts(1);
  }
  else
  {
    motorCurrentM[0] = analogRead(10);
    motorCurrentM[1] = analogRead(11);
    // battery voltage
    batVoltInt = analogRead(23);
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = analogRead(10) * 100;
    motorCurrentMLowPass[1] = analogRead(11) * 100;
  }
  
}

//////////////////////////////////////////////////////////

/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBuf[++usbRxBufCnt] = '\0';
  }
  if (localEcho)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if ((n == '\n' or n=='\r') and usbRxBufCnt > 0)
  {
    parse_and_execute_command(usbRxBuf, usbRxBufCnt);
    if (localEcho)
    {
      usb_send_str("\r\n>>");
      //usb_serial_flush_output();
    }
    // flush remaining input
    // usb_serial_flush_input();
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    usbRxBufCnt = 0;
  }
}

/**
 * Got a new character from serial1 channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromSerial1(uint8_t n)
{
    if (n >= ' ')
    {
        serial1RxBuf[serial1BufCnt] = n;
        
        // init next char space
        if (serial1BufCnt < RX_BUF_SIZE - 1) 
        {
            serial1RxBuf[++serial1BufCnt] = '\0';
        }
    }

     
    // check if end of string and parse
    if ((n == '\n' or n == '\r') and serial1BufCnt > 0) 
    {
      if (mission == 9) // mission 9 will be running on, and here some wifi commands coming from UART needs to be handled
	{
	  usb_send_str(serial1RxBuf);
	  if (strncmp(serial1RxBuf, "WIFI GOT IP", 11) == 0) // the esp8266 got an IP
	  {
	    wifiGotIp = true;
	  }
	  else if (strncmp(serial1RxBuf, "0,CONNECT", 9) == 0) // client has connect to the tcp server
	  {
	    wifiClientConnected = true;
	  }
	  else if (strncmp(serial1RxBuf, "0,CLOSED", 8) == 0) // client has disconnected
	  {
	    wifiClientConnected = false;
	  }
	  else if ( strncmp(serial1RxBuf, "OK", 2) == 0  or strncmp(serial1RxBuf, "SEND OK", 7) == 0) // confirmations of completed commands
	  {
	    
	    isWifiReady = true;
	  }
	  // this one will handle two incomming commands from the tcp client
	  else if ( strncmp(serial1RxBuf, "+IPD,0,",7) == 0 and not strncmp(serial1RxBuf, "+IPD,0,2",8) == 0)
	  {
	    // unwrap the data from the ESP 8266 protocol
	    char* token = strtok(serial1RxBuf,":");
	    token = strtok(NULL,":");
	    
	    // check if incoming data is start
	    if (strncmp(token, "start", 5) == 0)
	    {
	      missionStart = true; // start the mission
	    }
	    // check if it is a ready flag
	    else if (strncmp(token, "readyToGo", 9) == 0)
	    {
	      rdyToProceed = true; // set the ready flag
	    }
    
	  }
	  
	}
      // if the mission is not 9 -> we're on the follower robot
      else
      {
	// get the data
	char command[serial1BufCnt-2];
	for (int i = 2; i < serial1BufCnt; i++)
	  command[i-2] = serial1RxBuf[i];
	
	// get the value
	float val = atof(command);

	// match incoming data with method
	switch(serial1RxBuf[0])
	{
	  case 'D': // update distance to gmk or leader
	    distanceToGmk = val;
	    	    
	    break;
	    
	  case 'A': // update angle to gmk or leader
	    angleToGmk = val;
	    
	    break;
	  case 'Y': // ready flag
	    rdyToProceed = (int)val;
	    
	    break;
	  case 'S': // start mission
	    missionStart = true;
	    break;
	  case 'V':
	    if (serial1RxBuf[1] == 'A') // vel and angle in one string
	    {
	      char angleStr[10];
	      char velStr[10];
	      
	      if (serial1RxBuf[2] == '-') // check for negative value
	      {
		for (int i = 2; i < 8; i++)
		{
		  velStr[i-2] = serial1RxBuf[i];
		}
		for (int i = 8; i < serial1BufCnt; i++)
		{
		  angleStr[i-8] = serial1RxBuf[i];
		}
	      }
	      else // else it is positive
	      {
		for (int i = 2; i < 7; i++)
		{
		  velStr[i-2] = serial1RxBuf[i];
		}
		for (int i = 7; i < serial1BufCnt; i++)
		{
		  angleStr[i-7] = serial1RxBuf[i];
		}
	      }
	      
	      // update the variables
	      angleToGmk = atof(angleStr);
	      followVelocity = atof(velStr);

	    }
	    else // data contained only a speed
	    {
	      followVelocity = val;
	      
	    }
	    break;
	    
	  case 'M': // change mission state
	    missionState = (int)val; 
	    
	    break;
	    
	  case 'T': // update the turn radius
	    raspiTurnRadius = val;
	    //serial1_send_str((char*)"ready\n");
	    break;
	    
	  case 'O': // used for updating override variables
	    if (serial1RxBuf[1] == 'A') // angle
	    {
	      char angleStr[10] = "";
	      for (uint i = 0; i < strlen(serial1RxBuf)-3; i++)
	      {
		angleStr[i] = serial1RxBuf[i+3];
	      }
	      overrideAngle = atof(angleStr);
	      
	      serial1_send_str((char*)"ready\n"); // send ready command to raspi
	      
	    }
	    else if ( serial1RxBuf[1] == 'D') // distance
	    {
	      char distStr[10] = "";
	      for (uint i = 0; i < strlen(serial1RxBuf)-3; i++)
	      {
		distStr[i] = serial1RxBuf[i+3];
	      }
	      
	      overrideDistance = atof(distStr);
	      
	      
	      serial1_send_str((char*)"ready\n"); // send ready command to raspi
	      
	    }
	    
	    break;
	  default:
	    break;
	
	}
      }
      // reset counter
      serial1BufCnt = 0;
    }
    else if (serial1BufCnt >= RX_BUF_SIZE - 1) 
    {
        // garbage in buffer -> discard
        serial1RxBuf[serial1BufCnt] = 0;
        const char * msg = "** Discarded (missing \\n)\n";
        usb_send_str(msg);
        serial1BufCnt = 0;
    }
}


/**
 * Read data from sensors, that is the relevant sensors
 * \returns true if IMU could be read */
void readSensors()
{
  bool b;
//   #define AnalogA0 15 // motor 1 (left)
//   #define AnalogA1 14 // motor 2
  if (lsPowerHigh and pinMode18 == INPUT)
  { // high power mode - use both pin 18 and pin 32
    pinMode(18,OUTPUT); // Line sensor sensor value
    pinMode18 = OUTPUT;
  }
  else if (not lsPowerHigh and pinMode18 == OUTPUT)
  { // low power mode - use pin 32 only
    pinMode(18,INPUT); // Line sensor sensor value
    pinMode18 = INPUT;
  }
  if (useADCInterrupt)
  { // start first AC conversion
    adcSeq = 0;
    adcHalf = false;
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[0], -1); // + 400;
    adcStartTime = hb10us;
    adcStartCnt++;
  }
  // read start button
  if (robotId >15)
    // mainboard version 2B
    b = digitalRead(6);
  else
    // mainboard version 1A
    b = digitalRead(11);
  // prell prohibit
  if (b)
  { // butten pressed
    if (buttonCnt == 49)
    {
      button = true;
      // debug 
      // stopTeensy();
      // debug end
    }
    if (buttonCnt < 50)
    buttonCnt++;
  }
  else
  { // button not pressed
    if (buttonCnt == 1)
      button = false;
    if (buttonCnt > 0)
      buttonCnt--;
  }
  //
  if (imuAvailable)
  { // read from IMU
    //
    // NB! this takes 480us to read from gyro
    // this should be using interrupt !!!!!!!!!!!!!!!!
    //readAccGyro();
    if (true)
    {
      int isOK;
      isOK = mpuReadData();
      if (not isOK)
      {
        imuReadFail++;
        if (imuReadFail % 32 == 31)
        {
          const int MSL = 70;
          char s[MSL];
          snprintf(s, MSL, "# failed to get IMU data, got = %d (tried %d times)\r\n", isOK, imuReadFail);
          usb_send_str(s);
        }
      }
      else
        imuReadFail = 0;
      isOK = mpuRequestData();
      if (not isOK)
        usb_send_str("# failed to request IMU data\r\n");
    }
    else
    {
      int a = readAccGyro();
      if (a != 0)
        usb_send_str("#ACCGyro read failed\r\n");
//       else
//         usb_send_str("#ACCGyro data OK\r\n");
    }      
  }
//   if ((magCntMax > 0) and (--magCnt <= 0))
//   { // read from compas
//     readMagnetometer();
//     magCnt = magCntMax;
//   }
  //
  // start read of analogue values
  if (useADCInterrupt)
  { // convert last value to float
    // measured over a 15kOhm and 1.2kOhm divider with 1.2V reference
    //batVoltFloat = batVoltInt * 1.2 / useADCMaxADCValue * (15.0 + 1.2)/1.2;
    //
    if (motorPreEnabled)
    { // low pass input values (using long integer) - about 100ms time constant (if currentCntMax==1)
      // running until first motor is enabled
      motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 199)/200 + motorCurrentM[0];
      motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 199)/200 + motorCurrentM[1];
      // save as direct usable value
      motorCurrentMOffset[0] = motorCurrentMLowPass[0] / 200;
      motorCurrentMOffset[1] = motorCurrentMLowPass[1] / 200;
    }
    motorCurrentA[0] = getMotorCurrentM(0, motorCurrentM[0]);
    motorCurrentA[1] = getMotorCurrentM(1, motorCurrentM[1]);
  }
  else
  {
//     if (--currentCnt <= 0)
    { // time to read motor current
      motorCurrentM[0] = analogRead(10); // pin A10 and A11
      motorCurrentM[1] = analogRead(11);
      if (motorPreEnabled)
      { // low pass input values (using long integer) - about 100ms time constant (if currentCntMax==1)
        motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 99)/100 + motorCurrentM[0];
        motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 99)/100 + motorCurrentM[1];
        // save as direct usable value
        motorCurrentMOffset[0] = motorCurrentMLowPass[0] / 100;
        motorCurrentMOffset[1] = motorCurrentMLowPass[1] / 100;
      }
//      currentCnt = currentCntMax;
    }
    if (adcSeq < 8)
    {
      adcLSH[adcSeq] = analogRead(adcPin[adcSeq + 3]);
      adcSeq++;
    }
    else
      adcSeq = 0;
  }
  //if (--batVoltCnt <= 0)
  { // read voltage from pin 23 - scaled by 15k over 3.6k and Vref=3.3
    if (not useADCInterrupt)
    {
//       for (int a = 0; a < 8; a++)
//       { // try to make 16 A/D every 5th ms
//         // approx 12 us per AD
//         batVoltInt = analogRead(a);
//         batVoltInt = analogRead(a + 10);
//       }
      batVoltInt = analogRead(23);
      //batVoltFloat = batVoltInt * 1.2 / 1024 * (15.0 + 1.2)/1.2;
    }
    //batVoltCnt = batVoltCntMax;
  }
}

//////////////////////////////////////////

void debugLoop()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(23,INPUT); // battery voltage
  pinMode(11,INPUT); // start switch - version 2A
  digitalWriteFast(LED_BUILTIN, 1);
  while (true)
  {
    int b;
    // read start button
    // mainboard version 1A
    b = digitalRead(11);
    // prell prohibit
    if (b)
    { // butten pressed
      if (buttonCnt == 49)
      {
        button = true;
        // debug 
        stopTeensy();
        // debug end
      }
      if (buttonCnt < 50)
        buttonCnt++;
    }
    else
    { // button not pressed
      if (buttonCnt == 1)
      {
        button = false;
        // button released and not in stop mode
        digitalWriteFast(LED_BUILTIN, 1);
      }
      if (buttonCnt > 0)
        buttonCnt--;
    }
  }
}


/**
 * @brief method to set up the wifi radio on the leader robot
 * 
 * @return void
 */
void wifiSetup()
{
  if (not wifiIsSet) // then set it up!
  { 
      switch (wifiSetupCounter) // state controlled
      {
	case 0: // first reset to clear the module
	  serial1_send_str((char*)"AT+RST\r\n"); // reset - does not reset when when teensy reboots otherwise
	  usb_send_str((char*)"Resetting wifi module\n");
	  wifiSetupCounter =1;
	  break;
	case 1: // then wait for the esp to get an ip and send the cipmux command
	  if (wifiGotIp)
	  {
	    isWifiReady = false;
	    serial1_send_str((char*)"AT+CIPMUX=1\r\n"); // enable multiple connections
	    usb_send_str((char*)"Enabling multiple connections\n");
	    wifiSetupCounter = 2;
	  }
	  break;
	case 2: // wait for respond and set the listening port on the tcp server
	  if (isWifiReady)
	  {
	    isWifiReady = false;
	    serial1_send_str((char*)"AT+CIPSERVER=1,1336\r\n"); // set module to listen
	    usb_send_str((char*)"Setting module to listen\n");
	    wifiSetupCounter = 3;
	  }
	  break;
	case 3: // wait for respond and get the ip adress
	  if (isWifiReady)
	  {
	    isWifiReady = false;
	    serial1_send_str((char*)"AT+CIFSR\r\n");
	    wifiSetupCounter = 4;
	  }
	  break;
	case 4: // wait for a client to connect
	  if (isWifiReady)
	  {
	    isWifiReady = false;
	    wifiSetupCounter = 5;
	    usb_send_str((char*)"Waiting for client to connect\n");
	    
	  }
	  break;
	    
	case 5: // when a client is connect the setup is completed
	  if (wifiClientConnected)
	  {
	    wifiIsSet = true;
	    usb_send_str((char*)"WiFi is set!\n");
	  }
	  break;
	default:
	  break;
      }
    
  }
}

//////////////////////////////////////////

int * m1;
/**
 * Main loop
 * primarily for initialization,
 * non-real time services and
 * synchronisation with heartbeat.*/
extern "C" int main(void)
{
  uint8_t n, n2, m = 0;
  uint32_t loop = 0;
  int ltc, lastTimerCnt = 0; /// heartbeat timer loopcoun
  uint32_t loggerRowWait = 0;
  int row = 0;
  uint32_t start10us;
  // debug main
  if (false)
    debugLoop();
  // debug end
  // wait a bit - to allow usb to connect in order to see init errors
  delay(1000); // ms
  // 
  initialization();
  n = 0;
  motorAnkerVoltage[0] = 0.0;
  motorAnkerVoltage[1] = 0.0;
  // addMotorVoltage(0, 0);
  motorSetAnchorVoltage();
  // turn on linesensor diodes
  digitalWriteFast(18, LOW);
  // time starts now (in seconds)
  time = 0.0;
  //
  // then just listen for commands and process them
  while (1)
  { // main loop
    
    // check if wifi is setup
    wifiSetup();
    
    
    // get data from usb - if available
    n = usb_serial_getchar();
    if (n >= '\n' && n < 0xfe)
    { // command arriving from USB
      //usb_send_str("#got a char from USB\r\n");
      receivedCharFromUSB(n);
    }
    
    // get data from serial1 - if available
    
    n2 = Serial1.read();
    if (n2 >= '\n' && n2 < 0xfe)
    { 
     // char arriving from serial1
     receivedCharFromSerial1(n2);
    }
    
    //
    ltc = hbTimerCnt;
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle)
    { // start of new control cycle
      startNewCycle = false;
      start10us = hb10us;      
      // read new sensor values
      readSensors();
      // battery flat check
      if (batteryUse)
      { // keep an eye on battery voltage - if on USB, then battery is between 0 and 3 Volts - no error
        if (batVoltInt < batteryIdleVoltageInt and batVoltInt > int(4.0 / batVoltIntToFloat))
        {
          batLowCnt++;
          if (batLowCnt % 374 == 100 and batLowCnt < 10000 )
          { // send warning first 10 seconds and stop mission
            usb_send_str("#Battery low!\r\n");
            missionStop = true;
          }
          if (batLowCnt > 10000 and batLowCnt % 1000 == 0)
          { // stop processor to save a bit more current
            usb_send_str("#Battery low! (going IDLE NOW!)\r\n");
            // stop processor - goes down to about 30mA@12V (from about 60mA@12V)
            stopTeensy();
          }
        }
        else
          batLowCnt = 0;
      }
      // record read sensor time
      controlUsedTime[0] = hb10us - start10us;
      // calculate sensor-related values
      updatePose(loop);
      estimateTilt();
      estimateIrDistance();
      if (lineSensorOn)
        findLineEdge();
      else
      {
        lsLeftValid = false;
        lsRightValid = false;
        crossingBlackLine = false;
        crossingWhiteLine = false;
      }
      //
      if (missionState == 0)
      {  // we are waiting to start, so time since start is zero
        timeAtMissionStart = hb10us;
        if (time > 18000.0)
          // restart time (after 5 hours) - max usable time is 32768, as added in 1ms bits
          time = 0.0;
        // and allow manual setting of anchor voltage
      }
      // do control
      controlTick();
      // implement the new controlled motor voltage
//       if ((motorAnkerVoltage[0] != 0.0 or motorAnkerVoltage[1] != 0.0) and (loop % 200 == 0))
//       {
//         const int MSL = 50;
//         char s[MSL];
//         snprintf(s, MSL, "# main motor volt %g %g\r\n", motorAnkerVoltage[0], motorAnkerVoltage[1]);
//         usb_send_str(s);
//       }
      motorSetAnchorVoltage();
      //
//       if (missionStart)
//       {
//         // usb_send_str("#** mission started\n");
//         missionStart = false;
//       }
//       if (missionStop)
//       { usb_send_str("#** mission stopped\n");
//         missionStop = false;
//       }
      //
      // record read sensor time + control time
      controlUsedTime[1] = hb10us - start10us;
    }
    // request more IMU data  
    if (false)
    {
      bool isOK = mpuRequestData();
      if (not isOK)
        usb_send_str("# failed to request IMU data\r\n");
    }
    //
    if ((pushInterval > 0) && (ltc - pushTimeLast) >= pushInterval)
    {
      pushTimeLast = ltc;
      if ((missionState == 0 or sendStatusWhileRunning) and gyroOffsetDone)
      { // not in a mission, so
        // send status
        pushStatus++;
        if (pushStatus % 2 == 0)
        {
          sendHartBeat();
          sendStatusSensor();
        }
        else
        {
          switch(pushStatus / 2)
          {
            case 0 : 
              sendStatusControl();
              break;
            case 1 : 
              sendMissionStatusChanged(); 
              break;
            case 2 : 
              sendStatusLogging(); 
              break;
            case 3 : 
              sendStatusRobotID(); 
              break;
            case 4:
              sendStatusLineSensor();
              break;
            case 5:
              sendStatusDistIR();
            case 6:
              sendStatusVersion();
              break;
            default:
              pushStatus = 0;
          }
        }
      }
      else if (not gyroOffsetDone and hbTimerCnt % 1000 == 0)
      {
        usb_send_str("# IMU: gyro not calibrated\n");
      }
    }
    else
    { // send hartbeat anyhow - if not in interactive mode from putty
      if (not localEcho and ((ltc - pushHBlast) > 1000))
      { // send hartbeat every second - if no other communication
        pushHBlast = ltc;
        sendHartBeat();
      }
    }
    if ((ltc - lastTimerCnt) >= logInterval)
    {
      lastTimerCnt = ltc;
      m++;
//       if (pushToUSB)
//         stateToUsb();
      if (loggerLogging())
      {
        digitalWriteFast(LED_BUILTIN,(m & 0xff) < 128);
        stateToLog();
      }
      else if (logToUSB and (hbTimerCnt - loggerRowWait) > 10)
      { // attempt to wait a bit after a few lines send to logger
        // but do not seem to work, so set to just 10ms wait after 8 rows
        int row20 = 0;
        // signal log read using on-board LED
        digitalWriteFast(LED_BUILTIN,HIGH);
        // transfer 8 rows at a time
        for (row20 = 0; row < logRowCnt and row20 < 8; row20++)
        { // write buffer log to destination
          row = logWriteBufferTo(row);
          row++;
        }
        // set pause time
        loggerRowWait = hbTimerCnt;
        if (row >= logRowCnt)
        { // finished
          logToUSB = false;
          row = 0;
        }
        digitalWriteFast(LED_BUILTIN,LOW);
      }
    }
    loop++;
    if (loggerLogging())
      digitalWriteFast(LED_BUILTIN,(ltc & 0xff) < 127);
    else
      digitalWriteFast(LED_BUILTIN,(ltc & 0x3ff) < 10);
  }
  return 0;
}

/**
 * Heartbeat interrupt routine
 * schedules data collection and control loop timing.
 * */
void hbIsr(void)
{ // called every 10 microsecond
  // as basis for all timing
  hb10us++;
  if (hb10us % CONTROL_PERIOD_10us  == 0)
  {
    time += 1e-5 * CONTROL_PERIOD_10us; 
    hbTimerCnt++;
    startNewCycle = true;
    //
    // overload for encoder speed based on period
    if (not encTimeOverload[0])
      if ((int32_t)hb10us - (int32_t)encStartTime[0] > 256*8*256)
        encTimeOverload[0] = true;
    if (not encTimeOverload[1])
      if ((int32_t)hb10us - (int32_t)encStartTime[1] > 256*8*256)
        encTimeOverload[1] = true;
  } 
  if (hb10us % CONTROL_PERIOD_10us  == 60)
  { // start half-time ad conversion
    if (adcSeq >= MAX_ADC)
    {
      adcHalfStartTime = hb10us;
      adcSeq = 0;
      /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[0], -1); // + 300;
      adcHalf = true;
      adcHalfCnt++;
    }
  }
}

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr() 
{
  uint16_t v = 0;
  v = adc->adc0->readSingle();
  if (adcSeq < LS_FIRST)
    // low-pass filter non-line sensor values at about 2ms time constant
    // result is in range 0..8196 (for measured between 0v and 1.2V)
    *adcDest[adcSeq] = ((*adcDest[adcSeq]) >> 1) + v;
  else if (adcHalf)
    // line sensor raw value
    adcLSH[adcSeq - LS_FIRST] = v;
  else
    adcLSL[adcSeq - LS_FIRST] = v;
  adcSeq++;
  if (adcSeq < MAX_ADC)
  { // start new and re-enable interrupt
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[adcSeq], -1); // + 100;
  }
  else
  { // finished
    if (adcHalf)
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast(18, lineSensorOn);
      digitalWriteFast(32, lineSensorOn);
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast(18, LOW);
      digitalWriteFast(32, LOW);
    }
  }
  adcInt0Cnt++;
}

//#if defined(ADC_TEENSY_3_1)
void adc1_isr() 
{
  uint16_t v = adc->adc1->readSingle();
  if (adcSeq < LS_FIRST)
    *adcDest[adcSeq] = ((*adcDest[adcSeq]) >> 1) + v;
  else if (adcHalf)
    adcLSH[adcSeq - LS_FIRST] = v;
  else
    adcLSL[adcSeq - LS_FIRST] = v;
  adcSeq++;
  if (adcSeq < MAX_ADC)
  { // start new and re-enable interrupt
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[adcSeq], -1); // + 200;
  }
  else
  { // finished
    if (adcHalf)
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast(18, lineSensorOn);
      digitalWriteFast(32, lineSensorOn);
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast(18, LOW);
      digitalWriteFast(32, LOW);
    }
  }
  adcInt1Cnt++;
}
//#endif

void sendHartBeat()
{
  const int MRL = 35;
  char reply[MRL];
//   float mc0 = getMotorCurrent(0);
//   float mc1 = getMotorCurrent(1);
  snprintf(reply, MRL,  "hbt %g\r\n", time);
  usb_send_str(reply);
}

void sendStatusSensor()
  {
    //int switch2 = digitalRead(11);
    const int MRL = 350;
    char reply[MRL];
    //   float mc0 = getMotorCurrent(0);
    //   float mc1 = getMotorCurrent(1);
    snprintf(reply, MRL,"acw %g %g %g\n"
                        "gyw %g %g %g\n"
                        "gyo %g %g %g %d\n"
                        "mca %g %g\n" //%d %d %d %d %ld %ld %g %g\n"
                        "enc 0x%lx 0x%lx\n"
                        "wve %g %g\n"
                        "wpo %g %g\n"
                        "pse %g %g %g %g %g\n"
                        "swv %d\n"
                        "bat %g\r\n" ,
          imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac,
          imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac,
           offsetGyro[0] * gyroScaleFac, offsetGyro[1] * gyroScaleFac, offsetGyro[2]* gyroScaleFac, gyroOffsetDone,
//            mc0, mc1, motorCurrentM[0], motorCurrentM[1], 
//            motorCurrentMOffset[0], motorCurrentMOffset[1],
//           motorCurrentMLowPass[0], motorCurrentMLowPass[1],
           getMotorCurrentM(0, motorCurrentM[0]), getMotorCurrentM(1, motorCurrentM[1]),
          encoder[0], encoder[1], 
          wheelVelocityEst[0], wheelVelocityEst[1],
          wheelPosition[0], wheelPosition[1],
          pose[0], pose[1], pose[2], distance, pose[3],
          button, 
          batVoltInt * batVoltIntToFloat
  );
  usb_send_str(reply);
}


void sendStatusLogging()
{ 
  const int MRL = 175;
  char reply[MRL];
  snprintf(reply, MRL, "lms %d\n"
                       "lac %d\n"
                       "lgy %d\n"
//                        "lma %d\n"
                       "lvr %d\n"
                       "lmv %d\n"
                        "lma %d\n"
                       "lmr %d\n"
                       "ltr %d\n"
                       "lme %d\n"
                       "lpo %d\n"
                       "line %d\n"
                       "ldi %d\n"
                       "lbt %d\n"
                       "lbc %d\n"
                       "lex %d\n"
                       "lin %d %d\n"
                       "lct %d\n"
                       "lcn %d %d\r\n",
           logRowFlags[LOG_MISSION], // state number in mission
          logRowFlags[LOG_ACC],    // in ?
          logRowFlags[LOG_GYRO],   // in ?
//           logRowFlags[LOG_MAG],    // not used
           logRowFlags[LOG_MOTV_REF],   // motor controller reference
           logRowFlags[LOG_MOTV],   // orderd anchor voltage before PWM
          logRowFlags[LOG_MOTA],   // measured anchor current in Amps
          logRowFlags[LOG_WHEELVEL], // motor axle in rad/s
          logRowFlags[LOG_TURNRATE], // motor axle in rad/s
           logRowFlags[LOG_ENC],    // raw encoder counter
          logRowFlags[LOG_POSE],   // calculated pose x,y,th
           logRowFlags[LOG_LINE],   // line sensor values
           logRowFlags[LOG_DIST],   // distance sensor values
           logRowFlags[LOG_BATT],  // battery oltage in Volts
           logRowFlags[LOG_BAL_CTRL], // ballance controller info
           logRowFlags[LOG_EXTRA],   // not implemented
           logInterval, logAllow,
           logRowFlags[LOG_CTRLTIME],   // time spend on control
           logRowCnt, logRowsCntMax      
  );  
  usb_send_str(reply);
}

void sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  const char * p1 = REV;
  snprintf(reply, MRL, "version %d.%ld.%d %d\n", REV_MAIN,  strtol(&p1[5], NULL, 10), REV_MINOR, imuAvailable);
  usb_send_str(reply);
}


// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(const char *buf, uint8_t num)
{ // first character is the port letter
  const int MSL = 100;
  char s[MSL];
  char * p2;
  //
  if (buf[1] == '=')
  { // one character assignment command
    bool isOK;
    switch (buf[0])
    {
      case 'i':  localEcho = buf[2] == '1';      break;
      case 'M':
        if (missionState == 0)
        { // mission can not be changed while another mission is running
          mission = strtol(&buf[2], NULL, 10); 
          if (mission >= missionMax)
            mission = missionMax - 1;
        }
        else
          usb_send_str("#* sorry, a mission is in progress.\n");
        break;
      case 's':  
        logInterval = strtol(&buf[2], &p2, 10);
        if (*p2 >= ' ')
          logAllow = strtol(p2, &p2, 10); 
        break;
      case 'S':  pushInterval = strtol(&buf[2], NULL, 10); break;
      case 'R':  sendStatusWhileRunning = strtol(&buf[2], NULL, 10); break;
      case 'n':  pushCnt = strtol(&buf[2], NULL, 10); break;
      //case 'm':  magCntMax = strtol(&buf[2], NULL, 10); break;
      case 'a':  
        usb_send_str("# trying to access IMU: sending request\r\n");
        isOK = mpuRequestData();
        if (isOK)
          usb_send_str("# sending request was OK\r\n");
        else
          usb_send_str("# sending request failed\r\n");
        mpuReadData();        
        break;
      //case 'c':  currentCntMax = strtol(&buf[2], NULL, 10); break;
      default: 
        usb_send_str("#** unknown command!\n");
        break;
    }
  }
  else if (setRegulator(buf))
  { // regulator parameters
  }
  else if (setLineSensor(buf))
  { // line sensor settings
  }
  else if (setIrCalibrate(buf))
  { // regulator parameters
  }
  else if (buf[0] == '<')
  { // mission line
    if (strncmp(&buf[1], "clear", 5) == 0)
      mLinesCnt = 0;
    else if (strncmp(&buf[1], "add", 3) == 0)
    {
      if (mLinesCnt < mLinesCntMax)
      {
        mLines[mLinesCnt].decodeLine(&buf[4]);
        if (mLines[mLinesCnt].valid)
          mLinesCnt++;
        else
        {
          const int MRL = 50;
          char reply[MRL];
          snprintf(reply, MRL, "\r\n# add line %d failed: %s\r\n", mLinesCnt + 1, missionErrStr);
          usb_write(reply);
        }
      }
    }
    else if (strncmp(&buf[1], "get", 3) == 0)
    {
      const int MRL = 100;
      char reply[MRL];
      for (int i = 0; i < mLinesCnt; i++)
      {
        //usb_write("# line\n");
        mLines[i].toString(reply, MRL);
        usb_send_str(reply);
      }
      //usb_send_str("# send all lines?\n");
    }
    else if (strncmp(&buf[1], "token", 5) == 0)
    {
      const int MRL = 100;
      char reply[MRL];
//      UMission m;
      for (int i = 0; i < mLinesCnt; i++)
      {
        //usb_write("# line\n");
        reply[0]='<';
        mLines[i].toTokenString(&reply[1], MRL-1);
        usb_write(reply);
//         m.decodeToken(&reply[1]);
//         m.toString(&reply[1], MRL-1);
//         usb_write(reply);
      }
      usb_write("<done tokens>\n");
    }
    else
      usb_write("# no such user mission command\n");
  }
  else if (buf[0] == 'u')
  { // reply on user data request
    switch (buf[1])
    {
      case '0': sendStatusVersion(); break;
      case '1': 
        sendHartBeat();
        sendStatusSensor();  break;
      case '2': 
        sendMissionStatusChanged(); break;
      case '3': sendStatusLogging();  break;
      case '4': sendStatusRobotID(); break;
      case '5': sendStatusControl(); break;
      case '6': sendStatusMag(); break;
      case '7': sendStatusLS(); break;
      case '8': sendStatusCurrentVolt(); break;
      case '9': sendADLineSensor(); break;
      default:
        usb_send_str("#** unknown U status request!\n");
        break;
    }
  }
  else if (buf[0] == 'v')
  { // reply on user data request
    switch (buf[1])
    {
      case '0': sendStatusDistIR(); break;
      default:
        usb_send_str("#** unknown V status request!\n");
        break;
    }
  }
  else if (strncmp(buf, "halt", 4) == 0)
    stopTeensy();
  else if (buf[0] == 'h' || buf[0] == 'H')
  {
    const int MRL = 200;
    char reply[MRL];
    snprintf(reply, MRL, "# RegBot USB interface " REV_ID " (usb not idle cnt %d)\r\n", usb_not_idle_cnt);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   M=N    Select mission 0=motor test, 1=turn test, 2=Balance, 3=user, 4=SS, 5=linefollow (M=%d)\r\n", mission);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   i=1    Interactive - use local echo of all commands (i=%d)\r\n", localEcho);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   s=N A  Log interval in milliseconds (s=%d) and allow A=1 (is %d)\r\n", logInterval, logAllow);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   n=N    Push in total N status messages (n=%d)\r\n", pushCnt);
    usb_send_str(reply);
//     snprintf(reply, MRL, "#   m=N    Measure magnetometer every N ms (m=%d) 0=off\r\n", magCntMax);
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   a=N    Measure Accelerometer and gyro now\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   c=N    Measure motor current every N ms  (c=%d)\r\n", currentCntMax);
//     usb_send_str(reply);
    snprintf(reply, MRL, "#   log+/-item  Log item (mis %d, acc %d, gyro %d, " /*"mag %d, " */ "motR %d, "
                               "motV %d, motA %d, enc %d, mvel %d, tr %d, pose %d, line %d, dist %d, bat %d, bcl %d, ct %d)\r\n",
             logRowFlags[LOG_MISSION], logRowFlags[LOG_ACC], logRowFlags[LOG_GYRO], //logRowFlags[LOG_MAG],
             logRowFlags[LOG_MOTV_REF], logRowFlags[LOG_MOTV], 
             logRowFlags[LOG_MOTA], logRowFlags[LOG_ENC], logRowFlags[LOG_WHEELVEL], logRowFlags[LOG_TURNRATE], logRowFlags[LOG_POSE], 
             logRowFlags[LOG_LINE], logRowFlags[LOG_DIST],
             logRowFlags[LOG_BATT], logRowFlags[LOG_BAL_CTRL], logRowFlags[LOG_CTRLTIME]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   log start   Start logging to %dkB RAM (is=%d, logged %d/%d lines)\r\n", LOG_BUFFER_MAX/1000, loggerLogging(), logRowCnt, logRowsCntMax);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   log get     Transfer log to USB (active=%d)\r\n", logToUSB);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   motw m1 m2   Set motor PWM -1024..1024 (is=%d %d)\r\n", motorAnkerPWM[0], motorAnkerPWM[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   motv m1 m2   Set motor voltage -6.0 .. 6.0 (is=%.2f %.2f)\r\n", motorAnkerVoltage[0], motorAnkerVoltage[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   mote m1 m2   Set motor enable (left right) (is=%d %d)\r\n", motorEnable[0], motorEnable[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   uX           Status: u0:ver,  u1:measurements, u2:mission, u3:log, u4:robot, u5:ctrl, u6:mag, u7:LS, u8:VA, u9: LSdif\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   vX           Status: v0:IR sensor value\r\n");
    usb_send_str(reply);
    usb_send_str(        "#   rX=d d d ... Settingsfor regulator X - use GUI or see code for details\r\n");
//     snprintf(reply, MRL, "#   rt=d d d d   Regulator turn set: use Kp Ti Td Alpha - see u5\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   rv=d d d d d d d d d Regulator velocity set: use, Kp, Ti, Td, Alpha, stepTime from, to, acc_limit, estimate vel - see u5\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   rb=d d d d d d d d d Balance PID ctrl set: use, Kp, Ti, Td, Alpha, i limit, step time, from, to, lead_fwd, out limit, lead gyro\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   rm=d d d d d d d d d Mission vel PID ctrl set: use, Kp, Ti, Td, Alpha, Zeta, i-limit, Lead_fwd, out limit\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   rs=d d d d d d d d d Balance ss ctrl set: use, Tilt, Gyroi, Pos, Vel, Motor, limit, stepTime, from, to, step is position\r\n");
//     usb_send_str(reply);
//     snprintf(reply, MRL, "#   rid=d d d d d d d  Robot ID set: ID wheelBase gear PPR RadLeft RadRight balanceOffset\r\n");
//     usb_send_str(reply);
    snprintf(reply, MRL, "#   eew          Save configuration to EE-Prom\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eeW          Get configuration as string\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eeR=X        Read config and mission from hard coded set X=0: empty, X=1 follow wall\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   S=N          Push status every N milliseconds (is %d)\r\n", pushInterval);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   R=N          Push status while running 1=true (is %d)\r\n", sendStatusWhileRunning);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   posec        Reset pose and position\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   gyroo        Make gyro offset calibration\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   mem          Some memory usage info\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   start   start mission (and logging) now\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   stop   terminate mission now\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   test   processing time test (see code for details)\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <add user-mission-line>  add a user mission line (%d lines loaded)\r\n", mLinesCnt);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <clear>                  clear all user mission lines\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <get>                    get all user mission lines\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <token>                  get all user mission lines as tokens\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   halt   enter power save mode\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   help   This help text\r\n");
    usb_send_str(reply);
  }
  else if (strncmp(buf, "eew", 3) == 0)
  { // initalize ee-prom
    eeConfig.setStringBuffer(NULL, true);
    // save to ee-prom
    eeConfig.eePromSaveStatus(false);
  }
  else if (strncmp(buf, "eeW", 3) == 0)
  { // save config to string buffer (and do not touch ee-prom)
    uint8_t buffer2k[2048];
    eeConfig.setStringBuffer(buffer2k, false);
    eeConfig.eePromSaveStatus(true);
    // send string-buffer to client
    eeConfig.stringConfigToUSB(NULL, 0);
    eeConfig.clearStringBuffer();
  }
  else if (strncmp(buf, "eer", 3) == 0)
    // load from flash to ccurrent configuration
    eeConfig.eePromLoadStatus(NULL);
  else if (strncmp(buf, "eeR", 3) == 0)
  { // load from hard coded configuration to current configuration
    // get config number
    char * p1 = strchr(buf,'=');
    int m = 1;
    if (p1 != NULL)
    { // skip the equal sign
      p1++;
      // get the configuration number
      m = strtol(p1, NULL, 10);
      if (m < 0)
        m = 0; // factory reset
    }
    // 2. parameter is a debug flag to send 
    // the newly loaded configuration to USB
    eeConfig.hardConfigLoad(m, true);
  }
  else if (strncmp(buf, "stop", 4) == 0)
    missionStop = true;
  else if (strncmp(buf, "test", 4) == 0)
    processingTimeTest();
  else if (strncmp(buf, "start", 5) == 0)
    missionStart = true;
  else if (strncmp(buf, "posec", 5) == 0)
    clearPose();
  else if (strncmp(buf, "log", 3) == 0)
  { // add or remove
    if (strstr(&buf[3], "start") != NULL)
    { // if not mission timing, then zero here
      if (missionState == 0)
        time = 0.0;
      startLogging(0);
    }
    if (strstr(&buf[3], "stop") != NULL)
    { // 
      stopLogging();
    }
    //     else if (strcasestr(&buf[3], "tosd") != NULL)
//     {
//       logToSD = true;
//       isOK = true;
//     }
    else if (strcasestr(&buf[3], "get") != NULL)
    {
      logToUSB = true;
      if (logRowCnt == 0)
        usb_send_str("% log is empty\n");
    }
    else
    {
      int plus = buf[3] == '+';
      if (strncasecmp(&buf[4], "mis", 3) == 0)
        logRowFlags[LOG_MISSION] = plus;
      else if (strncasecmp(&buf[4], "acc", 3) == 0)
	logRowFlags[LOG_ACC] = plus;
      else if (strncasecmp(&buf[4], "gyro", 4) == 0)
	logRowFlags[LOG_GYRO] = plus;
//       else if (strncasecmp(&buf[4], "mag", 3) == 0)
// 	logRowFlags[LOG_MAG] = plus;
      else if (strncasecmp(&buf[4], "motV", 4) == 0)
	logRowFlags[LOG_MOTV] = plus;
      else if (strncasecmp(&buf[4], "motR", 4) == 0)
        logRowFlags[LOG_MOTV_REF] = plus;
      else if (strncasecmp(&buf[4], "motA", 4) == 0)
	logRowFlags[LOG_MOTA] = plus;
      else if (strncasecmp(&buf[4], "enc", 3) == 0)
	logRowFlags[LOG_ENC] = plus;
      else if (strncasecmp(&buf[4], "mvel", 4) == 0)
        logRowFlags[LOG_WHEELVEL] = plus;
      else if (strncasecmp(&buf[4], "tr", 2) == 0)
        logRowFlags[LOG_TURNRATE] = plus;
      else if (strncasecmp(&buf[4], "pose", 4) == 0)
	logRowFlags[LOG_POSE] = plus;
      else if (strncasecmp(&buf[4], "line", 4) == 0)
        logRowFlags[LOG_LINE] = plus;
      else if (strncasecmp(&buf[4], "dist", 4) == 0)
        logRowFlags[LOG_DIST] = plus;
      else if (strncasecmp(&buf[4], "bat", 3) == 0)
	logRowFlags[LOG_BATT] = plus;
      else if (strncasecmp(&buf[4], "ct", 2) == 0)
        logRowFlags[LOG_CTRLTIME] = plus;
      else if (strncasecmp(&buf[4], "bcl", 3) == 0)
        logRowFlags[LOG_BAL_CTRL] = plus;
      else if (strncasecmp(&buf[4], "extra", 5) == 0)
        logRowFlags[LOG_EXTRA] = plus;
      //
      initLogStructure(100000 / CONTROL_PERIOD_10us);

    }
  } 
  else if (strncmp(buf, "mote", 4) == 0)
  { // motor enable
    uint8_t m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtol(p1, (char**)&p1, 10);
    m2 = strtol(p1, (char**)&p1, 10);
    motorSetEnable(m1, m2);
  }
  else if (strncmp(buf, "motw", 4) == 0)
  {
    int m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtol(p1, (char**)&p1, 10);
    m2 = strtol(p1, (char**)&p1, 10);
    motorSetAnkerPWM(m1, m2);
  }
  else if (strncmp(buf, "motv", 4) == 0)
  {
    float m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtof(p1, (char**)&p1);
    m2 = strtof(p1, (char**)&p1);
    // limit to 6.0 volt
    if (m1 > 6) 
      m1 = 6.0;
    else if (m1 < -6)
      m1 = -6;
    if (m2 > 6) 
      m2 = 6.0;
    else if (m2 < -6)
      m2 = -6;
    motorAnkerVoltage[0] = m1;
    motorAnkerVoltage[1] = m2;
    //addMotorVoltage(m1, m2);
    // transfer to PWM
    motorSetAnchorVoltage();
  }
  else if (strncmp(buf, "rid", 3) == 0)
  { // robot ID and other permanent stuff
    setRobotID(&buf[4]);
  }
  else if (strncmp(buf, "gyroo", 5) == 0)
  {
    gyroOffsetDone = false;
  }
  else if (strncmp(buf,"mem", 3) == 0)
  { // memory usage
    snprintf(s, MSL, "# Main 3 areana=%d (m1=%x &time=%x s=%x)\n", mallinfo().arena, 
             (unsigned int)m1, (unsigned int)&time, (unsigned int)s);
    usb_send_str(s);      
    snprintf(s, MSL, "# Main log buffer from %x to %x\n", (unsigned int)logBuffer, (unsigned int)&logBuffer[LOG_BUFFER_MAX-1]);
    usb_send_str(s);      
    snprintf(s, MSL, "# Mission line takes %d bytes (bool is %d bytes)\r\n", 
             sizeof(UMissionLine), sizeof(bool));
    usb_send_str(s);      
  }
  else
  {
    usb_send_str("#** Unknown '");
    usb_serial_putchar(buf[0]);
    usb_send_str("' command (may try: stty -echo --file=/dev/ttyACM0)\r\n");
    usb_serial_flush_input();
  }
}

/////////////////////////////////////////////////////



///////////////////////////////////

void stateToUsb()
{ // read start switch
  //int switch2 = digitalRead(11);
  if ((pushCnt > 0) and (logInterval > 0))
  {
    const int MRL = 300;
    char reply[MRL];
    float mc0;
    float mc1;
//     if (useADCInterrupt)
//     {
//       mc0 = float(motorCurrent[0]) * 1.2 / useADCMaxADCValue / 0.525;
//       mc1 = float(motorCurrent[1]) * 1.2 / useADCMaxADCValue / 0.525;
//     }
//     else
//     {
//       mc0 = float(motorCurrent[0]) * 1.2 / useADCMaxADCValue / 0.525;
//       mc1 = float(motorCurrent[1]) * 1.2 / useADCMaxADCValue / 0.525;
//     }
    if (directionFWD[0])
      mc0 = -mc0;
    if (directionFWD[1])
      mc1 = -mc1;    
    snprintf(reply, MRL, "hb %lu a:%6.1f%6.1f%6.1f g:%6.1f%6.1f%6.1f c %5.3f %5.3f %5d %5d e %3lx %3lx v %5.3f %5.3f pose %6.3f %6.3f %5.1f sw %d bat %.2f\r\n",
//                         " baro %.1f %d %d\r\n",
            hbTimerCnt,
             imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac,
             imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac,
//            imuMag[0], imuMag[1], imuMag[2],
            motorCurrentA[0], motorCurrentA[1], motorCurrentM[0], motorCurrentM[1],
            encoder[0] & 0xfff, encoder[1] & 0xfff, wheelVelocity[0], wheelVelocity[1],
            pose[0], pose[1], pose[2] * 180 / M_PI,
            button, batVoltInt * batVoltIntToFloat
//              baro180data.tempC / 10.0, baro180data.presPascal, baro180data.measCnt
            );
    usb_send_str(reply);
    pushCnt--;
  }
}

/////////////////////////////////////////

// void logSendToSD()
// {
//   logToSD = true;
// }

//////////////////////////////////////////

int getUInt16FromBuffer()
{
  uint8_t h = Wire.read();
  uint8_t l = Wire.read();
  int a = (h << 8) + l;
  return a;
}

////////////////////////////////////////////

int getInt16FromBuffer()
{ // get signed int from 2 bytes
  int8_t h = Wire.read();
  int8_t l = Wire.read();
  int a = ((int)h << 8) | (int)l;
  return a;
}

/////////////////////////////////////////////


////////////////////////////////////////////////


/////////////////////////////////////////////////


void processingTimeTest() 
{
  uint32_t start;
  int32_t t1, t2;
  float a = 1.0;
  float b1 = 0.9999;
  float b2 = 0.9998;
  const float b3 = 0.9996;
  float b4 = 0.9997;
  float b5 = 0.9994;
  int32_t ia = 1;
  int32_t ib = 3;
  int16_t ja = 1;
  int8_t jb0 = 2, jb1, jb2, jb3, jb4, jb5, jb6, jb7, jb8, jb9;
  const int MSL = 100;
  char s[MSL];
  //
  usb_send_str("# Timing start\n");
  start= hb10us;
  for (int i= 0; i < 10000; i++)
  {
    a *= b1;
    a *= b2;
    a *= b3;
    a *= b4;
    a *= b5;
    a *= b5;
    a *= b2;
    a *= b3;
    a *= b4;
    a *= b5;
  }
  t1 = hb10us - start;
  snprintf(s, MSL, "# Timing float multiplication * 100000: %ld - %ld = %ld usec %g\n", hb10us, start, t1*10, a);
  usb_send_str(s);
  start= hb10us;
  for (int i= 0; i < 100000; i++)
  {
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * 2;
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * ib;
    ia = ia * ib;
  }
  t2 = hb10us - start;
  snprintf(s, MSL, "# Timing int32 multiplication * 1000000: %ld - %ld = %ld usec %ld\n", hb10us, start, t2*10, ia);
  usb_send_str(s);
  if (s[3] < ' ')
  {
    jb0 = 1;
    jb1 = 2;
    jb2 = 5;
    jb3 = 7;
    jb4 = 8;
    jb5 = 1;
    jb6 = 1;
    jb7 = 1;
    jb8 = 1;
    jb9 = 1;
  }
  else
  {
    jb0 = 1;
    jb1 = 3;
    jb2 = 4;
    jb3 = 17;
    jb4 = 8;
    jb5 = 1;
    jb6 = 3;
    jb7 = 4;
    jb8 = 1;
    jb9 = 1;
  }
  start= hb10us;
  for (int i= 0; i < 100000; i++)
  {
    ja = ja * jb1;
    ja = ja * jb2;
    ja = ja * jb3;
    ja = ja * jb4;
    ja = ja * jb5;
    ja = ja * jb6;
    ja = ja * jb7;
    ja = ja * jb8;
    ja = ja * jb9;
    ja = ja * jb0;
  }
  t2 = hb10us - start;
  snprintf(s, MSL, "# Timing int16 multiplication * 1000000: %ld - %ld = %ld usec %d\n", hb10us, start, t2*10, ja);
  usb_send_str(s);
}

#define SMC_PMCTRL_VLLSM_0 0x01
#define SMC_PMCTRL_VLLSM_1 0x02
#define SMC_PMCTRL_VLLSM_2 0x04
#define SMC_PMCTRL_STOPM_0 0x01
#define SMC_PMCTRL_STOPM_1 0x02
#define SMC_PMCTRL_STOPM_2 0x04


void stopTeensy(void)
{
  uint32_t a, b;
  adc->disableInterrupts(0);
  adc->disableInterrupts(1);
  detachInterrupt(M1ENC_A);
  detachInterrupt(M2ENC_A);
  detachInterrupt(M1ENC_B);
  detachInterrupt(M2ENC_B);
  hbTimer.end();
  // send mesage to USB - if connected
  usb_send_str("Press reboot to restart!\r\n");
  digitalWriteFast(LED_BUILTIN, 1);
  const int mod = 0x10000;
  b = mod;
  // fade on-board LED (replacing a delay)
  for (a = 0; a < mod * 256; a++)
  {
    if (a % mod == 0)
    { // turn on
      digitalWriteFast(LED_BUILTIN, 1);
      b -= mod / 256; 
    }    
    if (a % mod  > b)
    { // turn off
      digitalWriteFast(LED_BUILTIN, 0);
    }    
  }
  // turn off line sensor diodes (after delay, 
  // as an A/D series may be in progress
  digitalWriteFast(18, LOW);
  digitalWriteFast(32, LOW);
  //   SnoozeBlock sleep_config;
  //   Snooze.deepSleep(sleep_config, VLLS2);
  enter_vlls1();
}

