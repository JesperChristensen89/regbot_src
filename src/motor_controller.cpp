/*
  This file contains all the functions used for calculating
  the frequency, real Power, Vrms and Irms.
*/
#include "motor_controller.h"
#include <stdlib.h>
#include "serial_com.h"
#include "main.h"

// Variables
// motor ctrl - is the output to the motor controller in range -2.0 to 2.0 - positive is forward
//float motorCtrl[2]; 
// idle means that motor controller has released both motor terminals (no breaking effect)
//bool  motorIdle;
//
bool  directionFWD[2] = {0}; 
static volatile int   rotations[2] = {0};
//static volatile int   speed[2]     = {0};
//float batVoltFloat = 12.0;
float motorAnkerVoltage[2];
float max_motor_voltage = 9;
int16_t motorAnkerPWM[2] = {0,0};
int8_t  motorEnable[2] = {0,0};
// Return functions
//float motorTorque   (bool motor) { return motorGetCurrent(motor)*525; } // Torque constant (kt) =
//float motorDutyCycle(bool motor) { return dutyCycle[motor];            }
// bool  motorDirection(bool motor) { return direction[motor];        }
// int   motorRotations(bool motor) { return (rotations[motor]*465)/1000;        }
// int   motorSpeed    (bool motor) { return speed[motor];            }

/// encoder
uint32_t encoder[2];
uint32_t encoderLast[2] = {0,0};
uint32_t encStartTime[2];
uint32_t  encPeriod10us[2];
bool     encCCV[2];
bool     encTimeOverload[2];

float motorCurrentA[2];  // in amps
uint16_t motorCurrentM[2]; // external current sensor
uint16_t motorCurrentMOffset[2];

/**
 * Read motor current
 * \param motor 0 or 1 for motor 0 or 1.
 * \returns motor current in amps */
// float motorGetCurrent  (bool motor)
// {
// 	int16_t currentRaw;
// 	if (!motor) currentRaw = analogRead(M1FB);
// 	else currentRaw = analogRead(M2FB);
// 	// Calculates from input voltage (3.33V/1024bit) to amps (525mV/A)
// 	return ( (float)(currentRaw - 1)*3330.0/1024.0/525.0 );
// }

// Timing
// static IntervalTimer counterTimer;
// elapsedMicros sinceLastM1;
// elapsedMicros sinceLastM2;

void motorInit(void)
{
  // Output
  //pinMode(STANDBY,OUTPUT);
  pinMode(SLEW,OUTPUT);
  pinMode(M1IN1,OUTPUT);
  pinMode(M1IN2,OUTPUT);
  pinMode(M2IN1,OUTPUT);
  pinMode(M2IN2,OUTPUT);
  pinMode(M1D2,OUTPUT);
  pinMode(M2D2,OUTPUT);
  // Input
  pinMode(M1ENC_A,INPUT);
  pinMode(M1ENC_B,INPUT);
  pinMode(M2ENC_A,INPUT);
  pinMode(M2ENC_B,INPUT);
  pinMode(M1FB,INPUT);
  pinMode(M2FB,INPUT);
  // Writes init values
  //digitalWrite(STANDBY,HIGH);
  digitalWrite(SLEW,HIGH);
  digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  // set PWM output mode and frequency
  analogWriteRes(10); /// resolution (10 bit)
  analogWriteFrequency(M1IN1, 20000); /// frequency (20kHz)
  analogWriteFrequency(M2IN1, 20000);
  // find offset for motor current
}


bool motorError()
{
  return digitalRead(SF);
}

// static long mapValue(long x, long in_min, long in_max, long out_min, long out_max)
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

void motorSetEnable(uint8_t e1, uint8_t e2)
{
  if (motorPreEnabled)
  { // switch off current zero offset calculation
    const char MSL = 50;
    char s[MSL];
    motorPreEnabled = false;
    snprintf(s, MSL, "# motor zero current A/D value  %d %d.\n", 
             motorCurrentMOffset[0], motorCurrentMOffset[1]);
    usb_send_str(s);
  }
  // enable motors
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  // write to motor controller pins
  digitalWrite(M1D2, motorEnable[0]);
  digitalWrite(M2D2, motorEnable[1]);
}

/** 
 * allowed input is +/- 1024, where 1024 is full battery voltage
 * */
void motorSetAnkerPWM(int m1PWM, int m2PWM)
{ // too small PWM will not implement
//   if (m1PWM > -55 && m1PWM < 55)
//     m1PWM = -0;
//   if (m2PWM > -55 && m2PWM < 55)
//     m2PWM = 0;
  motorAnkerPWM[0] = m1PWM;
  motorAnkerPWM[1] = m2PWM;
  if (m1PWM >= 0)
  { // put H-bridge side 2 to high
    digitalWrite(M1IN2, HIGH);
    // mark direction for current value
    directionFWD[0] = true;    
    // make side 1 switch with low pulses down to fully low 
    analogWrite(M1IN1, 1024 - m1PWM);
  }
  else
  { // put H-bridge side 2 to low
    digitalWrite(M1IN2, LOW);
    // mark direction for current value
    directionFWD[0] = false;    
    // make side 1 switch with high pulses up to fully high 
    analogWrite(M1IN1, -m1PWM);
  }
  if (m2PWM >= 0)
  { // put H-bridge side 2 to high
    digitalWrite(M2IN2, HIGH);
    // mark direction for current value
    directionFWD[1] = true;    
    // make side 1 switch with low pulses down to fully low 
    analogWrite(M2IN1, 1024 - m2PWM);
  }
  else
  { // put H-bridge side 2 to low
    digitalWrite(M2IN2, LOW);
    // mark direction for current value
    directionFWD[1] = false;    
    // make side 1 switch with high pulses up to fully high 
    analogWrite(M2IN1, -m2PWM);
  }
}


////////////////////////////////////////////////////////////

void m1EncoderA()
{ // encoder A change
  // get timestamp now
  uint32_t e = hb10us;
  uint32_t dt = e - encStartTime[0];
  uint8_t  a = digitalRead(M1ENC_A);
  // read other channel for direction
  if (a == 0)
    encCCV[0] = digitalRead(M1ENC_B);
  else
    encCCV[0] = digitalRead(M1ENC_B) == 0;
  // velocity baset on encoder period
  if (encTimeOverload[0])
  { // too long time to count
    encPeriod10us[0] = 0;
    encTimeOverload[0] = false;
  }
  else if (dt < CONTROL_PERIOD_10us/2)
  { // We are running fast, more than ine enc pulse per 
    // sample-time
    encPeriod10us[0] = (encPeriod10us[0]*3 + dt)/4 ;
  }
  else if (dt < CONTROL_PERIOD_10us)
  { // We are running fast, more than ine enc pulse per 
    // sample-time
    encPeriod10us[0] = (encPeriod10us[0] + dt) / 2 ;
  }
  else
  { // period acceptable - longer than sample time and shorter than 2.5 seconds
    encPeriod10us[0] = dt;
  }
  encStartTime[0] = e;
  if (encCCV[0])
    encoder[0]--;
  else
    encoder[0]++;
}

//////////////////////////////////////////////////////////////

void m2EncoderA()
{
  // get timestamp now
  uint32_t e = hb10us;
  uint32_t dt = e - encStartTime[1];
  uint8_t  a = digitalRead(M2ENC_A);
  // read other channel for direction
  if (a)
    encCCV[1] = digitalRead(M2ENC_B);
  else
    encCCV[1] = digitalRead(M2ENC_B) == 0;
  // velocity baset on encoder period
  if (encTimeOverload[1])
  { // too long time to count
    encPeriod10us[1] = 0;
    encTimeOverload[1] = false;
  }
  else if (dt < CONTROL_PERIOD_10us/2)
  { // We are running fast, more than two enc pulse per 
    // control period, so average over about 4 samples
    encPeriod10us[1] = (encPeriod10us[1]*3 + dt)/4 ;
  }
  else if (dt < CONTROL_PERIOD_10us)
  { // We are running fast, more than one enc pulse per 
    // control period, so average over about 2 samples
    encPeriod10us[1] = (encPeriod10us[1] + dt) / 2 ;
  }
  else
  { // period acceptable - less than 2.5 seconds and more than 
    // one control period
    encPeriod10us[1] = e - encStartTime[1];
  }
  encStartTime[1] = e;
  if (encCCV[1])
    encoder[1]--;
  else
    encoder[1]++;
}

void m1EncoderB()
{ // encoder pin B
  // get timestamp now
  uint32_t e = hb10us;
  uint32_t dt = e - encStartTime[0];
  uint8_t  b = digitalRead(M1ENC_B);
  // read other channel for direction
  if (b)
    encCCV[0] = digitalRead(M1ENC_A);
  else
    encCCV[0] = digitalRead(M1ENC_A) == 0;
  // velocity baset on encoder period
  if (encTimeOverload[0])
  { // too long time to count
    encPeriod10us[0] = 0;
    encTimeOverload[0] = false;
  }
  else if (dt < CONTROL_PERIOD_10us/2)
  { // We are running fast, more than two enc pulse per 
    // control period, so average over about 4 samples
    encPeriod10us[0] = (encPeriod10us[0]*3 + dt) / 4 ;
  }
  else if (dt < CONTROL_PERIOD_10us)
  { // We are running fast, more than one enc pulse per 
    // control period, so average over about 2 samples
    encPeriod10us[0] = (encPeriod10us[0] + dt) / 2 ;
  }
  else
  { // period acceptable - less than 2.5 seconds and more than 
    // one control period
    encPeriod10us[0] = dt;
  }
  encStartTime[0] = e;
  if (encCCV[0])
    encoder[0]--;
  else
    encoder[0]++;
}

void m2EncoderB()
{ // encoder pin B
  // get timestamp now
  uint32_t e = hb10us;
  uint32_t dt = e - encStartTime[1];
  uint8_t  b = digitalRead(M2ENC_B);
  // read other channel for direction
  if (b == 0)
    encCCV[1] = digitalRead(M2ENC_A);
  else
    encCCV[1] = digitalRead(M2ENC_A) == 0;
  // velocity baset on encoder period
  if (encTimeOverload[1])
  { // too long time to count
    encPeriod10us[1] = 0;
    encTimeOverload[1] = false;
  }
  else if (dt < CONTROL_PERIOD_10us/2)
  { // We are running fast, more than two enc pulse per 
    // control period, so average over about 4 samples
    encPeriod10us[1] = (encPeriod10us[1]*3 + dt)/4 ;
  }
  else if (dt < CONTROL_PERIOD_10us)
  { // We are running fast, more than one enc pulse per 
    // control period, so average over about 2 samples
    encPeriod10us[1] = (encPeriod10us[1] + dt) / 2 ;
  }
  else
  { // period acceptable - less than 2.5 seconds and more than 
    // one control period
    encPeriod10us[1] = dt;
  }
  encStartTime[1] = e;
  if (encCCV[1])
    encoder[1]--;
  else
    encoder[1]++;
}


