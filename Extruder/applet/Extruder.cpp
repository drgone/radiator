/*
 * 5D GCode Interpreter
 * Arduino code to load into the extruder controller
 * 
  * Adrian Bowyer 3 July 2009
 */
 

 /*
 ***NOTE*** 
 This program changes the frequency of Timer 0 so that PWM on pins H1E and H2E goes at
 a very high frequency (64kHz see: 
 http://tzechienchu.typepad.com/tc_chus_point/2009/05/changing-pwm-frequency-on-the-arduino-diecimila.html)
 This will mess up timings in the delay() and similar functions; they will no longer work in
 milliseconds, but 64 times faster.
 */
  
#ifndef __AVR_ATmega168__
#error Oops!  Make sure you have 'Arduino Diecimila' selected from the boards menu.
#endif

#include <ctype.h>
#include <HardwareSerial.h>
#include "WProgram.h"
#include "configuration.h"
#include "extruder.h"
#include "intercom.h"



#include "WProgram.h"
void setup();
void loop();
void blink(bool on);
void delayMicrosecondsInterruptible(unsigned int us);
extruder ex;
intercom talker(&ex);
byte blk;

void setup() 
{
  pinMode(DEBUG_PIN, OUTPUT);
  rs485Interface.begin(RS485_BAUD);
  blk = 0;
  
  // Change the frequency of Timer 0 so that PWM on pins H1E and H2E goes at
  // a very high frequency (64kHz see: 
  // http://tzechienchu.typepad.com/tc_chus_point/2009/05/changing-pwm-frequency-on-the-arduino-diecimila.html)

  //TCCR0B &= ~(0x07); 
  //TCCR0B |= 1;
} 

void loop() 
{ 
  // Handle RS585
  
  talker.tick();
  
  // Keep me at the right temp etc.
  
  ex.manage();
}

// Blink the LED 

void blink(bool on)
{
  if(on)
  {
    blk = 1 - blk;
    digitalWrite(DEBUG_PIN, blk);
  } else
    digitalWrite(DEBUG_PIN, 0);
} 

void delayMicrosecondsInterruptible(unsigned int us)
{
  // for a one-microsecond delay, simply return.  the overhead
  // of the function call yields a delay of approximately 1 1/8 us.
  if (--us == 0)
    return;

  // the following loop takes a quarter of a microsecond (4 cycles)
  // per iteration, so execute it four times for each microsecond of
  // delay requested.
  us <<= 2;

  // account for the time taken in the preceeding commands.
  us -= 2;

  // busy wait
  __asm__ __volatile__ ("1: sbiw %0,1" "\n\t" // 2 cycles
"brne 1b" : 
  "=w" (us) : 
  "0" (us) // 2 cycles
    );
}


#include "configuration.h"
#include "extruder.h"
#include "temperature.h"

extruder::extruder()
{
  pinMode(H1D, OUTPUT);
  pinMode(H1E, OUTPUT);  
  pinMode(H2D, OUTPUT);
  pinMode(H2E, OUTPUT);
  pinMode(OUTPUT_A, OUTPUT);
  pinMode(OUTPUT_B, OUTPUT);
  pinMode(OUTPUT_C, OUTPUT);
  pinMode(E_STEP_PIN, INPUT);
  pinMode(E_DIR_PIN, INPUT);  
  pinMode(POT, INPUT);
#ifdef MAX6675_THERMOCOUPLE
  pinMode(SO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(TC_0, OUTPUT); 
  digitalWrite(TC_0,HIGH);  // Disable MAX6675
#else
  pinMode(TEMP_PIN, INPUT);
#endif

  disableStep();
 
#ifdef  PID_CONTROL

   pGain = TEMP_PID_PGAIN;
   iGain = TEMP_PID_IGAIN;
   dGain = TEMP_PID_DGAIN;
   temp_iState = 0;
   temp_dState = 0;
   temp_iState_min = -TEMP_PID_INTEGRAL_DRIVE_MAX/iGain;
   temp_iState_max = TEMP_PID_INTEGRAL_DRIVE_MAX/iGain;
   iState = 0;
   dState = 0;
   previousTime = millis()/MILLI_CORRECTION;

#endif
 

  // Defaults

  coilPosition = 0;  
  forward = true;
  pwmValue =  STEP_PWM;
  targetTemperature = 0;
  currentTemperature = 0;
  manageCount = 0;
  stp = 0;
  potVal = 0;
  potSum = 0;
  potCount = 0;
  usePot = true;
  
#ifdef PASTE_EXTRUDER
  pinMode(OPTO_PIN, INPUT); 
  valveAlreadyRunning = false;
  valveEndTime = 0;
  valveAtEnd = false;
  seenHighLow = false;
  valveState = false;
  requiredValveState = true;
  kickStartValve();
#endif
}

#ifdef  PID_CONTROL

// With thanks to Adam at Makerbot and Tim at BotHacker
// see http://blog.makerbot.com/2009/10/01/open-source-ftw/

byte extruder::pidCalculation(int dt)
{
  int output;
  int error;
  float pTerm, iTerm, dTerm;

  error = targetTemperature - currentTemperature;

  pTerm = pGain * error;

  temp_iState += error;
  temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
  iTerm = iGain * temp_iState;

  dTerm = dGain * (currentTemperature - temp_dState);
  temp_dState = currentTemperature;

  output = pTerm + iTerm - dTerm;
  output = constrain(output, 0, 255);

  return output;
}

#endif

void extruder::controlTemperature()
{
  currentTemperature = internalTemperature(); 
    
#ifdef PID_CONTROL

  int dt;
  unsigned long time = millis()/MILLI_CORRECTION;  // Correct for fast clock
  dt = time - previousTime;
  previousTime = time;
  if (dt > 0) // Don't do it when millis() has rolled over
    analogWrite(OUTPUT_C, pidCalculation(dt));

#else

  // Simple bang-bang temperature control

  if(targetTemperature > currentTemperature)
    digitalWrite(OUTPUT_C, 1);
  else
    digitalWrite(OUTPUT_C, 0);

#endif 
}



void extruder::slowManage()
{
  manageCount = 0;
  
  potSum += (potVoltage() >> 2);
  potCount++;
  if(potCount >= 10)
  {
    potVal = (byte)(potSum/10);
    potCount = 0;
    potSum = 0;
  }

  //blink(true);  

  controlTemperature();
}

void extruder::manage()
{
  byte s = digitalRead(E_STEP_PIN);
  if(s != stp)
  {
    stp = s;
    sStep(0);
  }

#ifdef PASTE_EXTRUDER
  valveMonitor();
#endif

  manageCount++;
  if(manageCount > SLOW_CLOCK)
    slowManage();   
}


/* 
 Temperature reading function  
 With thanks to: Ryan Mclaughlin - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1230859336
 for the MAX6675 code
 */

int extruder::internalTemperature()
{
#ifdef USE_THERMISTOR
  int raw = analogRead(TEMP_PIN);

  int celsius = raw;
  byte i;

  // TODO: This should do a binary chop

  for (i=1; i<NUMTEMPS; i++)
  {
    if (temptable[i][0] > raw)
    {
      celsius  = temptable[i-1][1] + 
        (raw - temptable[i-1][0]) * 
        (temptable[i][1] - temptable[i-1][1]) /
        (temptable[i][0] - temptable[i-1][0]);

      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == NUMTEMPS) celsius = temptable[i-1][1];
  // Clamp to byte
  if (celsius > 255) celsius = 255; 
  else if (celsius < 0) celsius = 0; 

  return celsius;
#endif

#ifdef AD595_THERMOCOUPLE
  return ( 5.0 * analogRead(TEMP_PIN) * 100.0) / 1024.0; //(int)(((long)500*(long)analogRead(TEMP_PIN))/(long)1024);
#endif  

#ifdef MAX6675_THERMOCOUPLE
  int value = 0;
  byte error_tc;


  digitalWrite(TC_0, 0); // Enable device

  /* Cycle the clock for dummy bit 15 */
  digitalWrite(SCK,1);
  digitalWrite(SCK,0);

  /* Read bits 14-3 from MAX6675 for the Temp
   	 Loop for each bit reading the value 
   */
  for (int i=11; i>=0; i--)
  {
    digitalWrite(SCK,1);  // Set Clock to HIGH
    value += digitalRead(SO) << i;  // Read data and add it to our variable
    digitalWrite(SCK,0);  // Set Clock to LOW
  }

  /* Read the TC Input inp to check for TC Errors */
  digitalWrite(SCK,1); // Set Clock to HIGH
  error_tc = digitalRead(SO); // Read data
  digitalWrite(SCK,0);  // Set Clock to LOW

  digitalWrite(TC_0, 1); //Disable Device

  if(error_tc)
    return 2000;
  else
    return value/4;

#endif

}

void extruder::waitForTemperature()
{

}

void extruder::valveSet(bool closed)
{
#ifdef PASTE_EXTRUDER
  requiredValveState = closed;
  kickStartValve();
#endif
}

void extruder::setDirection(bool direction)
{
  forward = direction;  
}

void extruder::setCooler(byte e_speed)
{
  analogWrite(OUTPUT_B, e_speed);   
}

void extruder::setTemperature(int tp)
{
  targetTemperature = tp;
}

int extruder::getTemperature()
{
  return currentTemperature;  
}

void extruder::sStep(byte dir)
{
#ifndef PASTE_EXTRUDER
  byte pwm;
  
  if(usePot)
    pwm = potVal;
  else
    pwm = pwmValue;

  // This increments or decrements coilPosition then writes the appropriate pattern to the output pins.

  switch(dir)
  {
    case 1:
      coilPosition++;
      break;
      
    case 2:
      coilPosition--;
      break;
      
    default:
      if(digitalRead(E_DIR_PIN))
        coilPosition++;
      else
        coilPosition--;
      break;
  }
  
  coilPosition &= 7;

  // Which of the 8 possible patterns do we want?
  // The pwm = (pwm >> 1) + (pwm >> 3); lines
  // ensure (roughly) equal power on the half-steps

#ifdef FULL_STEP
  switch((coilPosition&3) << 1)
#else
  switch(coilPosition)
#endif 
  {
  case 7:
    pwm = (pwm >> 1) + (pwm >> 3);
    digitalWrite(H1D, 1);    
    digitalWrite(H2D, 1);
    analogWrite(H1E, pwm);
    analogWrite(H2E, pwm);    
    break;

  case 6:
    digitalWrite(H1D, 1);    
    digitalWrite(H2D, 1);
    analogWrite(H1E, pwm);
    analogWrite(H2E, 0);   
    break; 

  case 5:
    pwm = (pwm >> 1) + (pwm >> 3);
    digitalWrite(H1D, 1);
    digitalWrite(H2D, 0);
    analogWrite(H1E, pwm);
    analogWrite(H2E, pwm); 
    break;

  case 4:
    digitalWrite(H1D, 1);
    digitalWrite(H2D, 0);
    analogWrite(H1E, 0);
    analogWrite(H2E, pwm); 
    break;

  case 3:
    pwm = (pwm >> 1) + (pwm >> 3);
    digitalWrite(H1D, 0);
    digitalWrite(H2D, 0);
    analogWrite(H1E, pwm);
    analogWrite(H2E, pwm); 
    break; 

  case 2:
    digitalWrite(H1D, 0);
    digitalWrite(H2D, 0);
    analogWrite(H1E, pwm);
    analogWrite(H2E, 0); 
    break;

  case 1:
    pwm = (pwm >> 1) + (pwm >> 3);
    digitalWrite(H1D, 0);
    digitalWrite(H2D, 1);
    analogWrite(H1E, pwm);
    analogWrite(H2E, pwm); 
    break;

  case 0:
    digitalWrite(H1D, 0);
    digitalWrite(H2D, 1);
    analogWrite(H1E, 0);
    analogWrite(H2E, pwm); 
    break; 

  }
#endif
}


void extruder::enableStep()
{
  // Nothing to do here - step() automatically enables the stepper drivers appropriately.  
}

void extruder::disableStep()
{
  analogWrite(H1E, 0);
  analogWrite(H2E, 0);  
}

int extruder::potVoltage()
{
  return (int)analogRead(POT);  
}

void extruder::setPWM(int p)
{
  pwmValue = p;
  usePot = false;
  sStep(1);
  sStep(2);
}

void extruder::usePotForMotor()
{
  usePot = true;
  sStep(1);
  sStep(2);
}

char* extruder::processCommand(char command[])
{
  reply[0] = 0;
  switch(command[0])
  {
  case WAIT_T:
    waitForTemperature();
    break;

  case VALVE:
    valveSet(command[1] != '1');
    break;

  case DIRECTION:
    // setDirection(command[1] == '1'); // Now handled by hardware.
    break;

  case COOL:
    setCooler(atoi(&command[1]));
    break;

  case SET_T:
    setTemperature(atoi(&command[1]));
    break;

  case GET_T:
    itoa(getTemperature(), reply, 10);
    break;

  case STEP:
    //sStep(0); // Now handled by hardware.
    break;

  case ENABLE:
    enableStep();
    break;

  case DISABLE:
    disableStep();
    break;

  case PREAD:
    itoa(potVoltage(), reply, 10);
    break;

  case SPWM:
    setPWM(atoi(&command[1]));
    break;

  case UPFM:
    usePotForMotor();
    break;  

  case PING:
    break;

  default:
    return 0; // Flag up dud command
  }
  return reply; 
}

#ifdef PASTE_EXTRUDER

bool extruder::valveTimeCheck(int millisecs)
{
  if(valveAlreadyRunning)
  {
    if(millis() >= valveEndTime)
    {
      valveAlreadyRunning = false;
      return true;
    }
    return false;
  }

// MILLI_CORRECTION not needed here; makes time resolution too coarse
  valveEndTime = millis() + millisecs; //*MILLI_CORRECTION;
  valveAlreadyRunning = true;
  return false;
}

void extruder::valveTurn(bool close)
{
  if(valveAtEnd)
    return;
    
  byte valveRunningState = VALVE_STARTING;
  if(digitalRead(OPTO_PIN))
  {
    seenHighLow = true;
    valveRunningState = VALVE_RUNNING;
  } else
  {
    if(!seenHighLow)
     valveRunningState = VALVE_STARTING;
    else
     valveRunningState = VALVE_STOPPING; 
  }    
   
  switch(valveRunningState)
  {
  case VALVE_STARTING: 
          if(close)
             digitalWrite(H1D, 1);
          else
             digitalWrite(H1D, 0);
          digitalWrite(H1E, HIGH);
          break;
          
  case VALVE_RUNNING:
          return;
  
  case VALVE_STOPPING:
          if(close)
            digitalWrite(H1D, 0);
          else
            digitalWrite(H1D, 1);
            
          if(!valveTimeCheck(10))
            return;
            
          digitalWrite(H1E, LOW);
          valveState = close;
          valveAtEnd = true;
          seenHighLow = false;
          break;
          
  default:
          break;
  }  
}

void extruder::valveMonitor()
{
  if(valveState == requiredValveState)
    return;
  valveAtEnd = false;
  valveTurn(requiredValveState);
} 

void extruder::kickStartValve()
{
  if(digitalRead(OPTO_PIN))
  {
     if(requiredValveState)
       digitalWrite(H1D, 1);
     else
       digitalWrite(H1D, 0);
     digitalWrite(H1E, HIGH);    
  }
} 
#endif
   
   


/*
 * Class to handle internal communications in the machine via RS485
 *
 * Adrian Bowyer 3 July 2009
 *
 */

#include "intercom.h"

#if MOTHERBOARD > 1


#if RS485_MASTER == 1
intercom::intercom()
#else
intercom::intercom(extruder* e)
#endif
{
#if !(RS485_MASTER == 1)
  ex = e;
#endif
  pinMode(RX_ENABLE_PIN, OUTPUT);
  pinMode(TX_ENABLE_PIN, OUTPUT);
  digitalWrite(RX_ENABLE_PIN, 0); // Listen is always on
  reset();
}

// Switch to listen mode

bool intercom::listen()
{
   if(inPacket)
   {
      listenCollision();
      return false;
   }
   digitalWrite(TX_ENABLE_PIN, 0);
   state = RS485_LISTEN;
   delayMicrosecondsInterruptible(RS485_STABILISE);
   resetWait();
   return true;
}

// Switch to talk mode

bool intercom::talk()
{
   if(state == RS485_TALK)
   {
      talkCollision();
      return false;
   }
   digitalWrite(TX_ENABLE_PIN, 1);
   state = RS485_TALK;
   delayMicrosecondsInterruptible(RS485_STABILISE);
   while(rs485Interface.available()) rs485Interface.read(); // Empty any junk from the input buffer
   resetWait();
   return true; 
}

// Reset to the initial satate

void intercom::reset()
{
  resetOutput();
  resetInput();
  listen();
}

// Reset the output buffer and its associated variables

void intercom::resetOutput()
{
  outBuffer[0] = 0;
  outPointer = 0;
}

// Reset the input buffer and its associated variables

void intercom::resetInput()
{
  inBuffer[0] = 0;
  inPointer = 0;
  inPacket = false;
  packetReceived = false;  
}

// Something useful has happened; reset the timeout time

void intercom::resetWait()
{
   wait_zero = millis();
}

// Have we waited too long for something to happen?

bool intercom::tooLong()
{
  return (millis() - wait_zero > TIMEOUT);
}


// Set the checksum for a packet.  This is the least-significant 6 bits of the sum
// of the packet's bytes added to the character RS485_CHECK.  It can thus take
// one of 64 values, all printable.

void intercom::checksum(char* packet)
{
  packet[P_SUM] = 1;  // Can't use 0, as that would terminate the packet...
  int cs = 0;
  int i = 0;
  while(packet[i]) 
  {
    cs += packet[i];
    i++;
  }
  cs--;               // Allow for the 1 at the start
  cs &= 0x3F;
  packet[P_SUM] = (char)(RS485_CHECK + cs);
}

// Check the checksum of a packet

bool intercom::checkChecksum(char* packet)
{
  char cs = packet[P_SUM];
  checksum(packet);
  return (cs == packet[P_SUM]);
}

// Build a packet to device to from an input string.  See intercom.h for the
// packet structure.  ack should either be RS485_ACK or RS485_ERROR.

void intercom::buildPacket(char to, char ack, char* string)
{
  byte i, j;
  j = 0;
  while(j < RS485_START_BYTES)
  {
     outBuffer[j] = RS485_START;
     j++;
  }
  outBuffer[j] = to;
  j++;
  outBuffer[j] = MY_NAME;
  j++; // Checksum goes here
  j++;
  outBuffer[j] = ack;
  j++;
  i = 0;
  while(string[i] && j < RS485_BUF_LEN - 4)
  {
    outBuffer[j] = string[i];
    j++;
    i++;
  }
  outBuffer[j] = 0;
  checksum(&outBuffer[RS485_START_BYTES]);
  outBuffer[j] = RS485_END;
  j++;
  outBuffer[j] = 0;
}


// The master processing function.  Call this in a fast loop, or from a fast repeated interrupt

void intercom::tick()
{
  char b = 0;
    
  switch(state)
  {
  case RS485_TALK:
  
      // Has what we last sent (if anything) been echoed?
      
      if(rs485Interface.available())
      {
        b = rs485Interface.read();
        resetWait();
        blink(true);
      } else
      {
        // Have we waited too long for an echo?
        
        if(tooLong())  
        {
          talkTimeout();
          return;  
        }
      }
      
      // Was the echo (if any) the last character of a packet?
      
      if(b == RS485_END)
      {
        // Yes - reset everything and go back to listening
        
        reset();
        return;            
      }
        
      // Do we have anything to send?
  
      b = outBuffer[outPointer];
      if(!b)
        return;
      
      // Yes - send it and reset the timeout timer
      
      rs485Interface.print(b, BYTE);
      outPointer++;
      if(outPointer >= RS485_BUF_LEN)
              outputBufferOverflow();
      resetWait();
      break;
      
  // If we have timed out while sending, reset everything and go
  // back to listen mode
      
  case RS485_TALK_TIMEOUT:
      resetOutput();
      resetInput();
      listen();
      break;
      
  case RS485_LISTEN:
      if(rs485Interface.available())  // Got anything?
      {
        blink(true);
        b = rs485Interface.read();
        switch(b)
        {
        case RS485_START:  // Start character - reset the input buffer
          inPointer = 0;
          inPacket = true;
          break;
        
        case RS485_END:   // End character - terminate, then process, the packet
          if(inPacket)
          {
            inPacket = false;
            inBuffer[inPointer] = 0;
            processPacket();
          }
          break;

        default:     // Neither start or end - if we're in a packet it must be data
                     // if not, ignore it.
          if(inPacket)
          {
            inBuffer[inPointer] = b;
            inPointer++;
            if(inPointer >= RS485_BUF_LEN)
              inputBufferOverflow();
          }
        }
        
        // We just received something, so reset the timeout time
        
        resetWait();
      } else
      {
        
        // If we're in a packet and we've been waiting too long for the next byte
        // the packet has timed out.
        
        if(inPacket && tooLong())
          listenTimeout();
          
        //blink(false);
      }
      break;
        
  case RS485_LISTEN_TIMEOUT:
      resetInput();
      listen();
      break;
      
  default:
      corrupt();
      break;
  }
}

// We are busy if we are talking, or in the middle of receiving a packet

bool intercom::busy()
{
  return (state == RS485_TALK) || inPacket;
}


// Send string to device to.

bool intercom::queuePacket(char to, char ack, char* string)
{
  if(busy())
  {
    queueCollision();
    return false;
  }
  buildPacket(to, ack, string);
  talk();
  return true;
}

// Wait for a packet to arrive.  The packet will be in inBuffer[ ]

bool intercom::waitForPacket()
{
  long timeNow = millis();  // Can't use tooLong() as tick() is using that
  while(!packetReceived)
  {
     tick();
     if(millis() - timeNow > TIMEOUT)
     {
       waitTimeout();
       packetReceived = false;
       return false;
     }
  }
  packetReceived = false;
  return true;
}

// Send a packet and get an acknowledgement - used when no data is to be returned.

bool intercom::sendPacketAndCheckAcknowledgement(char to, char* string)
{
  if(!queuePacket(to, RS485_ACK, string))
  {
    queueError();
    return false;
  }
  
  if(!waitForPacket())
  {
    waitError();
    return false;
  }
  
  if(!checkChecksum(inBuffer))
  {
    checksumError();
    return false;
  }
  
  if(inBuffer[P_ACK] != RS485_ACK)
  {
    ackError();
    return false;
  }
  
  return true;
  
/*  byte retries = 0;
  bool ok = false;
  while((inBuffer[P_TO] != MY_NAME || inBuffer[P_ACK] != RS485_ACK) && retries < RS485_RETRIES && !ok)
  {
    if(queuePacket(to, RS485_ACK, string))
      ok = waitForPacket();
    ok = ok && checkChecksum(inBuffer);
    if(!ok)
     delay(2*TIMEOUT);  // Wait twice timeout, and everything should have reset itself
    retries++;   
  }
  return ok; 
 */ 
}

// Send a packet and get data in reply.  The string returned is just the data;
// it has no packet housekeeping information in.

char* intercom::sendPacketAndGetReply(char to, char* string)
{
  if(!sendPacketAndCheckAcknowledgement(to, string))
    inBuffer[P_DATA] = 0;
  return &inBuffer[P_DATA]; //strcpy(reply, &inBuffer[P_DATA]);
  //return reply;
}

// This function is called when a packet has been received

void intercom::processPacket()
{
  char* erep = 0;
  char err;
  if(inBuffer[P_TO] != MY_NAME)
  {
    resetInput();
    return;
  }  
#if !(RS485_MASTER == 1)

  if(checkChecksum(inBuffer))
  {
    erep = ex->processCommand(&inBuffer[P_DATA]);
    if(erep) 
      queuePacket(inBuffer[P_FROM], RS485_ACK, erep);
  }
  
  if(!erep)
  {
    err = 0;
    queuePacket(inBuffer[P_FROM], RS485_ERROR, &err);
  }
  
  resetInput();
  
#endif
  packetReceived = true;
}


// *********************************************************************************

// Error functions

// The output buffer has overflowed

void intercom::outputBufferOverflow()
{
  outPointer = 0;
#if RS485_MASTER == 1
  strcpy(debugstring, "E1");
#endif  
}


// The input buffer has overflowed

void intercom::inputBufferOverflow()
{
  resetInput();
#if RS485_MASTER == 1
  strcpy(debugstring, "E2");
#endif   
}

// An attempt has been made to start sending a new message before
// the old one has been fully sent.

void intercom::talkCollision()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "E3");
#endif
}

// An attempt has been made to get a new message before the old one has been
// fully received or before the last transmit is finished.

void intercom::listenCollision()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "E4");
#endif  
}

// An attempt has been made to queue a new message while the system is busy.

void intercom::queueCollision()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "E5");
#endif  
}

// (Part of) the data structure has become corrupted

void intercom::corrupt()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "E6");
#endif   
}


// We have been trying to send a message, but something is taking too long

void intercom::talkTimeout()
{
  state = RS485_TALK_TIMEOUT;
#if RS485_MASTER == 1
  strcpy(debugstring, "E7");
#endif    
}

// We have been trying to receive a message, but something has been taking too long

void intercom::listenTimeout()
{
  state = RS485_LISTEN_TIMEOUT;
#if RS485_MASTER == 1
  strcpy(debugstring, "E8");
#endif    
}

// We have been waiting too long for an incomming packet

void intercom::waitTimeout()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "E9");
#endif     
}

void intercom::queueError()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "EA");
#endif     
}


void intercom::waitError()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "EB");
#endif     
}


void intercom::checksumError()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "EC ");
  strcat(debugstring, inBuffer);
#endif     
}

  
void intercom::ackError()
{
#if RS485_MASTER == 1
  strcpy(debugstring, "ED ");
  strcat(debugstring, inBuffer);  
#endif     
}



#endif

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

