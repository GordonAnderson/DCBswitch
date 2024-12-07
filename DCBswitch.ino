#include "Arduino.h"
#include "Hardware.h"
#include "DCBswitch.h"
#include <SPI.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Thread.h>
#include <ThreadController.h>

#include <RingBuffer.h>
#include <commandProcessor.h>
#include <charAllocate.h>
#include <debug.h>
#include <Errors.h>
#include <Devices.h>
#include "DIN.h"

#include <SerialBuffer.h>

// DCswitch application
//
//  This firmware runs on the Teensy 4.0 that is located on the DCBswitch MIPS module. This module
//  can be intergrated into a MIPS system or operated in a standalone mode where commands are sent
//  to this processor using the USB interface.
//  
//  This module has the following features:
//    - 4 DC bias output channels, +/- 250V range
//      - voltage readbacks
//      - power supply ON/OFF
//      - voltage error protection
//    - 4 Digital inputs
//    - 4 Digital outputs
//    - 2 high speed FET switches
//        - Switch 1 uses DCbias channels 1 and 2
//        - Switch 2 uses DCbias channels 3 and 4
//    - 2 channel pulse generator capability
//        - Use DCbias channel switching
//        - Use FET fast switching between DCbias voltages
//
//  Pulse generation features:
//    - Two independent pulse generators
//    - Can be armed with an external trigger
//    - Can be started with an external trigger
//    - Supports delay from trigger to first pulse
//    - Allows skipping triggers
//    - Support multiple pulses with user defined period
//    - User defined pulse width
//    - Pulse voltage
//    - Trigger output on pulse
//    - Control FET fast switch
//    - re-triggerable
//
// To do:
//  - ADD TWI address change command
//  - Add counter function
//  - Add dual command string
//    - if-then-else
//    - increment/decrement
//
// MIPS interface using one TWI address. Need to emulate the serial EEPROM that uses two TWI
// addresses. This will requires MIPS updates to support extended addressing of the EEPROM
// emulation. The normal access requires first sending a one byte address then read or write data.
// Two TWI addresses are supported for addresses over 256. The emulation works as follows:
//    - If the address byte == 0x80 then the address word follows.
//    - If the MSB of the address byte is set and the remaining 7 bits are not 0
//         then a command is send and processed acordingly
//  
// Version history:
//  V1.0, Dec 29, 2023
//    - Initial release
//  V1.1, Feb 4, 2024
//    - Two ramp channels
//    - Define DCB channel
//    - Define ramp with a string of voltage,time points
//    - External trigger start of ramp
//    - Number of cycles
//    - Period, common for both channels
//    - Enable/disable
//    - Implementation details
//      - in a tight loop calculate the voltages and apply
//      - use interval timer for period
//      - when no change process serial
//      - start interval with command or external trigger
//      - ramp structure
//        - DC bias channel
//        - time,value array
//  V1.2, Sept 2, 2024
//    - Added MIPS bus interface
//    - Added TWI interface and commands
//    - Added EEPROM emulation and commands using one TWI addtess
//    - Added TWI talk support
//    - Added commands to change TWI address
//
// Gordon Anderson
//


const char *Version = "DCBswitch, version 1.2 Sept 2, 2024";

DIN *din[4];

SerialBuffer sb;
SerialBuffer db;

bool    ReturnAvailable = false;

DCBswitch dcbs = {
    sizeof(DCBswitch),"DCBswitch",1,
    0x54,           // TWI address
    false,true,  
    0,0,0,0,
    {               // Pulse generator 1
      false,true,
      0,RISING,
      'Q',RISING,
      0,5,15,
      'A','1',0,-100,
      1,100,
                    // Pulse generator 2
      false,true,
      0,RISING,
      0,RISING,
      0,0,10,
      0,0,0,0,
      1,100
    },
    {               // Ramp generator
      0,RISING,
      false,0,0,
      1000,25,
      false,1,6,{0,2,3,5,7,9},{0,0,10,10,0,0},
      false,2,6,{0,2,3,5,7,9},{0,0,10,10,0,0},
    },
    {               // DAC output channels
      CH1_CTRL,120,32668,
      CH2_CTRL,120,32668,
      CH3_CTRL,120,32668,
      CH4_CTRL,120,32668,
    },
    {               // ADC readback channels
      CH1_MON,120,32668,
      CH2_MON,120,32668,
      CH3_MON,120,32668,
      CH4_MON,120,32668,
    },
    SIGNATURE
};

uint8_t *dataPRT        = (uint8_t *)&dcbs; //NULL;

State state;

float DCBrb[4]        = {0,0,0,0};                   // DC bias channel readbacks
// Digital IO pin assignments
int   Douts[4]        = {DIOA,DIOB,DIOC,DIOD};       // A thru D digital outputs
int   Dins[4]         = {DIOQ,DIOR,DIOS,DIOT};       // Q thru t digital outputs
// FET switch parameters
int   FETs[2]         = {SW1,SW2};
int   FETon[2]        = {LOW,HIGH};
int   FEToff[2]       = {HIGH,LOW};
char  FETfollow[2]    = {0,0};
void  switchFollowISR(int sw);
void  followISR1(void){switchFollowISR(0);}
void  followISR2(void){switchFollowISR(1);}
void  (*switchFollowISRs[2])(void) = {followISR1,followISR2};

bool          enableRG = false;
bool          EEPROMenable = true;
IntervalTimer rampTimer;

Command dbsCmds[] =
{
  // Main application commands
  {"GVER",CMDstr,(void *)Version,                 "Firmware version"},
  {"?NAME",CMDstr,(void *)dcbs.Name,              "Name of this device"},
  {"SAVE",CMDfunction,(void *)SaveSettings,       "Save system setting to flash"},
  {"RESTORE",CMDfunction,(void *)RestoreSettings, "Restore system setting from flash"},
  {"FORMAT",CMDfunction,(void *)formatFlashDrive, "Formats the flash storage, all data will be lost"},
  {"STWIADD",CMDfunction,(void *)setTWIaddress,   "Sets the TWI address, hex"},
  {"GTWIADD",CMDfunction,(void *)getTWIaddress,   "Returns the TWI address, hex"},
  // DC bias voltage commands
  {"SDCBS",CMDfunction,(void *)setDCBvoltage,     "Set DCbias voltage, channel, voltage"},
  {"GDCBS",CMDfunction,(void *)getDCBvoltage,     "Get DCbias voltage setpoint, channel"},
  {"GDCBSV",CMDfunction,(void *)getDCBreadback,   "Get DCbias voltage readback, channel"},
  {"SDCPWR",CMDfunction,(void *)setDCBpwr,        "Set DC power supply, ON or OFF"},
  {"GDCPWR",CMDfunction,(void *)getDCBpwr,        "Return DC power supply status, ON or OFF"},
  {"SDIO",CMDfunction,(void *)setDOUT,            "Set digital output,A|B|C|D,0|1"},
  {"GDIO",CMDfunction,(void *)getDIN,             "Return digital input states,A|B|C|D|Q|R|S|T"},
  {"SSW",CMDfunction,(void *)setSW,               "Set FET state,switch#,0|1"},
  {"GSW",CMDfunction,(void *)getSW,               "Return FET state,switch#"},
  {"PSW",CMDfunction,(void *)pulseSW,             "Pulse FET switch,switch#,width in uS"},
  {"STRGSW",CMDfunction,(void *)setSwitchTrigger, "Set FET switch follow input,switch#,0|Q|R|S|T"},
  {"GTRGSW",CMDfunction,(void *)getSwitchTrigger, "Return FET switch follow input,switch#"},
  {"?RBTST",CMDbool,(void *)&dcbs.rbTest,         "Enable read back testing if TRUE"},
  {"CALCH",CMDfunction,(void *)calibrate,         "Calibrate DCbias channel,channel#"},
  // Pulse generator command
  {"SPGENA",CMDfunction,(void *)setPGena,         "Set Pulse Generator enable,Ch,TRUE|FALSE"},
  {"GPGENA",CMDfunction,(void *)getPGena,         "Return Pulse Generator enable,Ch"},
  {"SPGRET",CMDfunction,(void *)setPGretrig,      "Set Pulse Generator re-trigger,Ch,TRUE|FALSE"},
  {"GPGRET",CMDfunction,(void *)getPGretrig,      "Return Pulse Generator re-trigger,Ch"},
  {"SPGARMT",CMDfunction,(void *)setPGarmT,       "Set Pulse Generator arm input,Ch,0|Q|R|S|T"},
  {"GPGARMT",CMDfunction,(void *)getPGarmT,       "Return Pulse Generator arm input,Ch"},
  {"SPGARML",CMDfunction,(void *)setPGarmL,       "Set Pulse Generator arm level,Ch,CHANGE|RISING|FALLING"},
  {"GPGARML",CMDfunction,(void *)getPGarmL,       "Return Pulse Generator arm level,Ch"},
  {"SPGTRG",CMDfunction,(void *)setPGtrig,        "Set Pulse Generator trigger input,Ch,0|Q|R|S|T"},
  {"GPGTRG",CMDfunction,(void *)getPGtrig,        "Return Pulse Generator trigger input,Ch"},
  {"SPGTRGL",CMDfunction,(void *)setPGtrigL,      "Set Pulse Generator trigger level,Ch,CHANGE|RISING|FALLING"},
  {"GPGTRGL",CMDfunction,(void *)getPGtrigL,      "Return Pulse Generator trigger level,Ch"},
  {"SPGSKIP",CMDfunction,(void *)setPGskip,       "Set Pulse Generator trigger skip count,Ch,Count"},
  {"GPGSKIP",CMDfunction,(void *)getPGskip,       "Return Pulse Generator trigger skip count,Ch"},
  {"SPGDLY",CMDfunction,(void *)setPGdly,         "Set Pulse Generator trigger delay in uS,Ch,Delay"},
  {"GPGDLY",CMDfunction,(void *)getPGdly,         "Return Pulse Generator trigger delay in uS,Ch"},
  {"SPGWDTH",CMDfunction,(void *)setPGwdth,       "Set Pulse Generator pulse width in uS in uS,Ch,Width"},
  {"GPGWDTH",CMDfunction,(void *)getPGwdth,       "Return Pulse Generator pulse width in uS in uS,Ch"},
  {"SPGTRGO",CMDfunction,(void *)setPGtrgOut,     "Set Pulse Generator trigger output,Ch,0|A|B|D|D"},
  {"GPGTRGO",CMDfunction,(void *)getPGtrgOut,     "Return Pulse Generator trigger output,Ch"},
  {"SPGTRGF",CMDfunction,(void *)setPGtrhFET,     "Set Pulse Generator FET switch output,Ch,0|1|2"},
  {"GPGTRGF",CMDfunction,(void *)getPGtrhFET,     "Return Pulse Generator FET switch output,Ch"},
  {"SPGOUTCH",CMDfunction,(void *)setPGoutCh,     "Set Pulse Generator DC bias output,Ch,0|1|2|3|4"},
  {"GPGOUTCH",CMDfunction,(void *)getPGoutCh,     "Return Pulse Generator DC bias output,Ch"},
  {"SPGPV",CMDfunction,(void *)setPGvoltage,      "Set Pulse Generator pulse voltage,Ch,Voltage"},
  {"GPGPV",CMDfunction,(void *)getPGvoltage,      "Return Pulse Generator pulse voltage,Ch"},
  {"SPGNUM",CMDfunction,(void *)setPGnum,         "Set Pulse Generator number of pulses,Ch,Count"},
  {"GPGNUM",CMDfunction,(void *)getPGnum,         "Return Pulse Generator number of pulses,Ch"},
  {"SPGPER",CMDfunction,(void *)setPGperiod,      "Set Pulse Generator period in uS,Ch,Value"},
  {"GPGPER",CMDfunction,(void *)getPGperiod,      "Return Pulse Generator period in uS,Ch"}, 
  // Ramp commands
  {"SRPENA",CMDfunction,(void *)setRPena,         "Set ramp generator enable, TRUE or FALSE"},
  {"GRPENA",CMDfunction,(void *)getRPena,         "Return ramp generator enable, TRUE or FALSE"},
  {"SRPTRG",CMDfunction,(void *)setRPtrig,        "Set Ramp Generator trigger input,Ch,0|Q|R|S|T"},
  {"GRPTRG",CMDfunction,(void *)getRPtrig,        "Return Ramp Generator trigger input,Ch"},
  {"SRPTRGL",CMDfunction,(void *)setRPtrigL,      "Set Ramp Generator trigger level,Ch,CHANGE|RISING|FALLING"},
  {"GRPTRGL",CMDfunction,(void *)getRPtrigL,      "Return Ramp Generator trigger level,Ch"},
  {"SRPDCBC",CMDfunction,(void *)setRPDCBchan,    "Set ramp channel DC bias output number,ramp ch, DCB ch"},
  {"GRPDCBC",CMDfunction,(void *)getRPDCBchan,    "Return ramp channel DC bias output number,ramp ch, DCB ch"},
  {"SRPCHENA",CMDfunction,(void *)setRPchENA,     "Set ramp channel enable, ramp ch"},
  {"GRPCHENA",CMDfunction,(void *)getRPchENA,     "Return ramp channel enable, ramp ch"},
  {"SRAMP",CMDfunction,(void *)setRPramp,         "Set ramp channel data, ramp ch, time, value...."},
  {"GRAMP",CMDfunction,(void *)getRPramp,         "Return ramp channel data, ramp ch, time, value...."},
  {"SRPPER",CMDfunction,(void *)setRPperiod,      "Set ramp period"},
  {"GRPPER",CMDfunction,(void *)getRPperiod,      "Return ramp period"},
  {"SRPCYC",CMDfunction,(void *)setRPcycl,        "Set ramp number of cycles"},
  {"GRPCYC",CMDfunction,(void *)getRPcycl,        "Return ramp number of cycles"},
  //
  {NULL}
};
static CommandList dbsList = {dbsCmds, NULL};

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

commandProcessor cp;
debug dbg(&cp);

LittleFS_Program myfs;
File systemData;

// This function is called from the main polling loop to generate a voltage
// ramp on up to two DC bias outputs.
// This function does not block.
bool processRamps(Ramps *ramps)
{
  float setDCBV[2];
  int   ch;

  // Exit if not generating
  if(!ramps->generating) return false;
  // Calculate the cycle time
  ramps->ctime = (float)(micros() - ramps->stime)/1000.0;
  if(ramps->ctime >= ramps->period)
  {
    ramps->generating = false;
    return false;
  }
  // Process updates in each ramp
  for(int i=0;i<2;i++)
  {
    if(!ramps->ramp[i].enable) continue;
    ch = ramps->ramp[i].chan - 1;
    // Find the voltage at this time point in table.
    for(int j=ramps->ramp[i].numTP-1;j>=0;j--)
    {
      if(ramps->ctime > ramps->ramp[i].tp[j])
      {
        // Here with the index of the time point greater than the current time
        // If this is the first time point then set the voltage to this time points value
        if(j == (ramps->ramp[i].numTP-1)) setDCBV[i] = ramps->ramp[i].val[ramps->ramp[i].numTP-1];
        else if(ramps->ramp[i].val[j] == ramps->ramp[i].val[j+1]) setDCBV[i] = ramps->ramp[i].val[j];
        else
        {
          // Calculate the voltage
          setDCBV[i] = \
          ((ramps->ctime - ramps->ramp[i].tp[j]) / (ramps->ramp[i].tp[j+1] - ramps->ramp[i].tp[j]) * \
          (ramps->ramp[i].val[j+1] - ramps->ramp[i].val[j])) + ramps->ramp[i].val[j];
        }
        // Set the output voltage
        dcbs.DCBV[ch] = setDCBV[i];
        UpdateDACvalue(AD5592, CS, &dcbs.DCBctrl[ch], &dcbs.DCBV[ch], &state.DCBV[ch], state.update);
        break;
      }
    }
  }
  return true;
}

void switchFollowISR(int sw)
{
  if(FETfollow[sw] == 0) return;
  if(digitalRead(Dins[FETfollow[sw] - 'Q']) == 0) digitalWrite(FETs[sw],FEToff[sw]);
  else digitalWrite(FETs[sw],FETon[sw]);
}

// Pulse generator functions and variables
// Width - 2uS if no DAC channel
// Delay - 2uS 
// Period - 2uS if no DAC
// DAC adds 5uS conversion delay
void armISR1(void)      {pulseGenArm(0);}
void triggerISR1(void)  {pulseGenTrig(0);}
void pulseISR1(void)    {pulseGenPulse(0);}
void widthISR1(void)    {pulseGenWidth(0);}
void armISR2(void)      {pulseGenArm(1);}
void triggerISR2(void)  {pulseGenTrig(1);}
void pulseISR2(void)    {pulseGenPulse(1);}
void widthISR2(void)    {pulseGenWidth(1);}
IntervalTimer pulseTimer[2], pulseWidth[2];
void (*armISRs[2])(void)      = {armISR1,armISR2};
void (*triggerISRs[2])(void)  = {triggerISR1,triggerISR2};
void (*pulseISRs[2])(void)    = {pulseISR1,pulseISR2};
void (*widthISRs[2])(void)    = {widthISR1,widthISR2};
int  skipCounter[2]   = {0,0};
int  pulseCounter[2]  = {0,0};
int  DACrawLow[2]     = {0,0};
int  DACrawHigh[2]    = {0,0};
bool pulseGenStart[2] = {false,false};
int  delayCorrection[2] = {0,0};

// ISR function to set pulse generator function in active.
void pulseGenWidth(int ch)
{
  // Set select DC bias channel to pulse voltage to default
  if(dcbs.PG[ch].outputCh != 0) AD5592writeDAC(CS, dcbs.DCBctrl[dcbs.PG[ch].outputCh-1].Chan, DACrawLow[ch]);
  // Set select digital output low
  if(dcbs.PG[ch].trigOut != 0) digitalWrite(Douts[dcbs.PG[ch].trigOut - 'A'],LOW);
  // Set selected FET switch output low
  if(dcbs.PG[ch].trigFET != 0) digitalWrite(FETs[dcbs.PG[ch].trigFET - '1'],FEToff[dcbs.PG[ch].trigFET - '1']);
  // Stop the interval timer
  pulseWidth[ch].end();
  if(pulseCounter[ch] >= dcbs.PG[ch].numPulse)
  {
    if(!dcbs.PG[ch].reTrig) dcbs.PG[ch].enable = false;
    pulseGenStart[ch] = true;
  }
}

// This function is generally called by ISR to set the pulse active. The width
// ISR function is setup to set the pulse in active.
void pulseGenPulse(int ch)
{
  if(pulseCounter[ch] >= dcbs.PG[ch].numPulse)
  {
    pulseTimer[ch].end();
    return;
  }
  // Schedule the pulse width timer, if width is 0 do nothing
  if(dcbs.PG[ch].width > 0)
  {
    if(dcbs.PG[ch].width <= delayCorrection[ch]) dcbs.PG[ch].width = delayCorrection[ch] + 1;
    // Set select DC bias channel to pulse voltage
    if(dcbs.PG[ch].outputCh != 0) AD5592writeDAC(CS, dcbs.DCBctrl[dcbs.PG[ch].outputCh-1].Chan, DACrawHigh[ch]);
    // Set select digital output high
    if(dcbs.PG[ch].trigOut != 0) digitalWrite(Douts[dcbs.PG[ch].trigOut - 'A'],HIGH);
    // Set selected FET switch output high
    if(dcbs.PG[ch].trigFET != 0) digitalWrite(FETs[dcbs.PG[ch].trigFET - '1'],FETon[dcbs.PG[ch].trigFET - '1']);
    pulseWidth[ch].begin(widthISRs[ch],dcbs.PG[ch].width - delayCorrection[ch]);
  }
  // Set pulse timer to period or stop on first call
  if(pulseCounter[ch] == 0)
  {
    if(dcbs.PG[ch].numPulse <= 1) pulseTimer[ch].end();
    else
    {
      int period = dcbs.PG[ch].period - delayCorrection[ch];
      if(period <= 0) period = 1;
      pulseTimer[ch].begin(pulseISRs[ch],period);
    }
  }
  pulseCounter[ch]++;
}

// This function is called to enable the pulse generator external trigger function.
// This function will setup an ISR tied to the requested trigger input. If no tigger
// input is defined this function will call the pulse generation function.
void pulseGenTrig(int ch)
{
  if(skipCounter[ch]++ < dcbs.PG[ch].skip) return;
  int delay = dcbs.PG[ch].delay - delayCorrection[ch];
  if(delay < 0) delay = 0;
  if(delay == 0)
  {
    pulseGenPulse(ch);
    return;
  }
  pulseTimer[ch].begin(pulseISRs[ch],delay);
  if(dcbs.PG[ch].trig != 0) din[dcbs.PG[ch].trig - 'Q']->detach(triggerISRs[ch]);
}

// This function is called after the pulse generator is enabled and waits for an
// arm signal to advance to waiting for a pulse generator trigger. If arm
// input is not enabled then this function will call the trigger enable function.
void pulseGenArm(int ch)
{
  // Detach ISR
  for(int i=0;i<4;i++) din[i]->detach(armISRs[ch]);
  // Setup trigger ISR or call  
  if(dcbs.PG[ch].trig == 0)
  {
     skipCounter[ch] = dcbs.PG[ch].skip;
     pulseGenTrig(ch);
     return;  
  }
  skipCounter[ch] = 0;
  din[dcbs.PG[ch].trig - 'Q']->attach(dcbs.PG[ch].trigLevel, triggerISRs[ch]);
}

// Process pulse generator flag. This function is called from the command processing
// function after the flag value has been written. The pulse generator parameters should
// be set before this call and not changed until flag is false.
// The pulse generator logic will set flag false when finished.
// If called when pulse genertor is running it will be restarted with current settings.
// On call:
//  - Init all parameters, stop interval timers
//  - detach isrs from digital inputs
//    - armed and triggered
//  - clear pulse counter
//  - clear skip counter
//  - Prep the DAC buffer to fast update
// If flag set to enable
//  - Set up arm ISR
//  - Call armed if arm trigger not enabled
void pulseGenEnable(int ch)
{
  delayCorrection[ch] = 2;
  // Detach all ISRs
  for(int i=0;i<4;i++)
  {
    din[i]->detach(armISRs[ch]);
    din[i]->detach(triggerISRs[ch]);
  }
  // Stop timers
  pulseTimer[ch].end();
  pulseWidth[ch].end();
  // Init variables
  skipCounter[ch] = 0;
  pulseCounter[ch] = 0;
  // Calculate DAC counts if enabled
  if(dcbs.PG[ch].outputCh != 0) 
  {
    delayCorrection[ch] = 7;
    DACrawHigh[ch] = Value2Counts(dcbs.PG[ch].pulseV,&dcbs.DCBctrl[dcbs.PG[ch].outputCh-1]);
    DACrawLow[ch] = Value2Counts(dcbs.DCBV[dcbs.PG[ch].outputCh-1],&dcbs.DCBctrl[dcbs.PG[ch].outputCh-1]);
  }
  //
  if(!dcbs.PG[ch].enable) return;
  // Here if pulse generator is enabled
  if(dcbs.PG[ch].armTrig == 0)
  {
     pulseGenArm(ch);
     return;  
  }
  din[dcbs.PG[ch].armTrig - 'Q']->attach(dcbs.PG[ch].armLevel, armISRs[ch]);
}

// End of pulse generator functions

void Debug(void)
{
  if(dcbs.ramps.generating) cp.println("Generating");
  else cp.println("not");
}

bool checkEnabled(int setHoldoff = 0)
{
  static int holdoff=HOLDOFF;

  if(!dcbs.rbTest) return false;
  if(setHoldoff > 0)
  {
    holdoff = setHoldoff;
    return false;
  }
  if(holdoff > 0)
  {
    holdoff--;
    return false;
  }
  if(!dcbs.pwrEnable) return false;
  return true;
}

void Update()
{
  if(((!dcbs.PG[0].enable) || (dcbs.PG[0].outputCh == 0)) && ((!dcbs.PG[1].enable) || (dcbs.PG[1].outputCh == 0)))
  {
    for(int i=0;i<4;i++) 
    {
      if(dcbs.DCBV[i] != state.DCBV[i]) checkEnabled(HOLDOFF);
      UpdateDACvalue(AD5592, CS, &dcbs.DCBctrl[i], &dcbs.DCBV[i], &state.DCBV[i], state.update);
    }
    for(int i=0;i<4;i++) UpdateADCvalue(AD5592, CS, &dcbs.DCBmon[i], &DCBrb[i]);
  }
  if((state.pwrEnable != dcbs.pwrEnable) || state.update)
  {
    if(dcbs.pwrEnable) digitalWrite(PWROFF, LOW);
    else digitalWrite(PWROFF, HIGH);
    state.pwrEnable = dcbs.pwrEnable;
    checkEnabled(HOLDOFF);
  }
  // Look for readback errors and turn off supply if there is an error
  if(checkEnabled())
  {
    for(int i=0;i<4;i++)
    {
      if(abs(DCBrb[i] - dcbs.DCBV[i]) > TRIPVOLTAGE)
      {
        dcbs.pwrEnable = false;
        break;
      }
    }
  }
  state.update = false;
}

// Init the AD5592 (Analog and digital IO chip).
void AD5592init(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration
   AD5592write(addr, 3, 0x0100);
   // Set internal reference
   AD5592write(addr, 11, 0x0200);
   // Set LDAC mode
   AD5592write(addr, 7, 0x0000);
   // Set DO outputs channels
   AD5592write(addr, 8, 0x0000);
   // Set DAC outputs channels
   AD5592write(addr, 5, 0x000F);
   // Set ADC input channels
   AD5592write(addr, 4, 0x00F0);
   // Turn off all pulldowns
   AD5592write(addr, 6, 0x0000);
   
   // Set default values
   // Init DAC channels to mid range
   AD5592writeDAC(addr, 0, 32768);
   AD5592writeDAC(addr, 1, 32768);
   AD5592writeDAC(addr, 2, 32768);
   AD5592writeDAC(addr, 3, 32768);
}

void receiveEventProcessor(int cmd)
{
  int8_t  b,c;
  int16_t w;

  if(cp.selectedStream() == &sb)
  {
    while(true)
    {
      //Serial.println(cmd);
      if (cmd == ESC) cp.selectStream(&Serial);
      else cp.rb->put(cmd);
      if(Wire.available() <= 0) break;
      cmd = Wire.read();
    }
  }
  else switch (cmd & 0x7F)
  {
    case TWI_DCBS_SERIAL:
      sb.clear();
      cp.selectStream(&sb);
      break;
    case GET_DCBS_AVALIBLE:
      // Set flag to return bytes avalible on the next read from TWI
      ReturnAvailable = true;
      break;
    case TWI_DCBS_EEPROM:
      // Set the EEPROM enable flag, true by default
      if(!ReadByte(&b)) break;
      EEPROMenable = b;
      break;
    case GET_DCBS_RBTST:
      SendBuffer((uint8_t *)&DCBrb[0], sizeof(float)*4);
      break;
    case SET_DCBS_SDOUT:
      if(!ReadByte(&b)) break;     // Read the port, A thru D
      if(!ReadByte(&c)) break;     // Read the state 0, or 1
      b = b - 'A';
      if((b >= 0) && (b <= 3) && ((c == 0) || (c == 1))) digitalWrite(Douts[b],c);
      break;
    case GET_DCBS_SDIN:
      if(!ReadByte(&b)) break;;     // Read the port, A|B|C|D|Q|R|S|T
      c = b - 'A';
      if((c >= 0) && (c <= 3)) SendByte(digitalRead(Douts[c]));
      c = b - 'Q';
      if((c >= 0) && (c <= 3)) SendByte(digitalRead(Dins[c]));
      break;
    case SET_DCBS_PWR:
      if(!ReadByte(&b)) break;
      dcbs.pwrEnable = b;
      break;
    case GET_DCBS_PWR:
      SendByte(dcbs.pwrEnable);
      break;
    case SET_DCBS_FETSW:
      if(!ReadByte(&b)) break;     // Read the FET switch number, 1|2
      if(!ReadByte(&c)) break;     // Read the state 0|1
      if(((b != 1) && (b != 2)) || ((c != 0) && (c != 1))) break;
      if(c == 1) digitalWrite(FETs[b-1],FETon[b-1]);
      else digitalWrite(FETs[b-1],FEToff[b-1]);
      break;
    case GET_DCBS_FETSW:
      if(!ReadByte(&b)) break;     // Read the FET switch number, 1|2
      if((b != 1) && (b != 2)) break;
      c = digitalRead(FETs[b-1]);
      if(c == FETon[b-1]) SendByte(1);
      else SendByte(0);
      break;
    case SET_DCBS_FETPULSE:
      if(!ReadByte(&b)) break;      // Read the FET switch number, 1|2
      if(!Read16bitInt(&w)) break;  // Read width 1 to 10000 uS
      if(((b != 1) && (b != 2)) || ((w < 1) || (w > 10000))) break;
      if(digitalRead(FETs[b-1]) == 1)
      {
        digitalWrite(FETs[b-1],0);
        delayMicroseconds(w);
        digitalWrite(FETs[b-1],1);
      }
      else
      {
        digitalWrite(FETs[b-1],1);
        delayMicroseconds(w);
        digitalWrite(FETs[b-1],0);    
      }
      break;
    case SET_DCBS_SWFOL:
      if(!ReadByte(&b)) break;      // Read the FET switch number, 1|2
      if(!ReadByte(&c)) break;      // Read input to follow, "0|Q|R|S|T"
      if(c == '0') c = 0;
      else c = c - 'Q' + 1;
      if(((b != 1) && (b != 2)) || ((c < 0) || (c > 5))) break;
      b--;
      if(FETfollow[b] != 0) din[FETfollow[b] - 'Q']->detach(switchFollowISRs[b]);
      FETfollow[b] = 0;
      if(c != 0)
      {
        c--;
        FETfollow[b] = 'Q' + c;
        if(FETfollow[b] != 0) din[FETfollow[b] - 'Q']->attach(CHANGE,switchFollowISRs[b]);
      }
      break;
    case GET_DCBS_SWFOL:
      if(!ReadByte(&b)) break;      // Read the FET switch number, 1|2
      if((b != 1) && (b != 2)) break;
      c = FETfollow[b-1];
      if(FETfollow[b-1] == 0) SendByte(0);
      else SendByte(c);
      break;
    case SET_DCBS_RBTST:
      if(!ReadByte(&b)) break;
      dcbs.rbTest = b;
      break;
    default:
      break;
  }
}

// Read from data buffer and send to TWI interface, data is placed
// in the serial buffer and sent on request
void TWIdataRead(int address)
{
   if(dataPRT == NULL) return;
   for(int i=0;i<32;i++) sb.write(dataPRT[address + i]);
}

// Write data from TWI interface to data buffer. This is done by placing the data
// in a buffer. The first two bytes are the address, the next byte is the number
// of data bytes, next the data bytes and written and finally a flag set to true if
// no errors were detected. The buffer is processed in the main idle loop and the 
// data structure updated.
void TWIdataWrite(int address, int num)
{
  bool  status = true;
  // Place address in the buffer
  db.write(address & 0xFF);
  db.write((address >> 8) & 0xFF);
  // Place byte count in the buffer
  db.write(num);
  // write the data
  for(int i=0;i<num;i++)
  {
    if(Wire.available()) db.write(Wire.read());
    else
    {
      db.write(0);
      status = false;
    }
  }
  db.write(status);
}

void processDatagrams(void)
{
  int     address, num;
  uint8_t buf[32];
  uint8_t *prt = (uint8_t *)&dcbs;

  if(db.available())
  {
    // Read the address
    address  =  db.read();
    address |= (db.read() << 8) & 0xFF00;
    // Read the number of bytes
    num  =  db.read();
    if(num > 32)
    {
      db.clear();
      return;
    }
    // Read the data to temp buffer
    for(int i=0;i<num;i++) buf[i] = db.read();
    // Read the error code
    if(db.read())
    {
      // Write the data to memory
      for(int i=0;i<num;i++) prt[address + i] = buf[i];
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint16_t twiAddress;

  if(howMany == 0) return;
  uint8_t twiByte = Wire.read();
  if(!EEPROMenable)
  {
    receiveEventProcessor(twiByte);
    return;
  }
  if((twiByte & 0x80) == 0)
  {
    // If here the received byte is the address for the next read or write.
    // If there is more data then we write it to the address, if this is the
    // only data then send 32 bytes of data.
    twiAddress = twiByte;
    if(Wire.available() <= 0) TWIdataRead(twiAddress);
    else TWIdataWrite(twiAddress,Wire.available());
    return;
  }
  // Here if the byte received is a command or an extended address command
  if(twiByte == 0x80)
  {
    // Read extended address from twi interface
    if(Wire.available() < 2)
    {
      // Error exit clear the interface and exit
      while(Wire.available()) Wire.read();
      return;
    }
    twiAddress  = Wire.read();
    twiAddress |= Wire.read() << 8;
    if(Wire.available() > 0) TWIdataWrite(twiAddress,Wire.available());
    else TWIdataRead(twiAddress);
    return;
  }
  // If here the request is a command so call the receive event processor
  receiveEventProcessor(twiByte & 0x7F);
}

// This function is called when the master asks for data.
// Send up to 32 bytes.
void requestEvent(void)
{
  int num = sb.available();
  if(ReturnAvailable)
  {
    ReturnAvailable = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;    
  }
  if(num > 32) num = 32;
  if(cp.selectedStream() == &sb) num=32;
  for (int i = 0; i < num; i++)
  {
    Wire.write(sb.read());
  }
}

void setup() 
{
  // Turn off the HV power supply
  pinMode(PWROFF,OUTPUT); digitalWrite(PWROFF, HIGH);
  // Setup the input bits, A,B,C,and D
  pinMode(DIOQ,INPUT);
  pinMode(DIOR,INPUT);
  pinMode(DIOS,INPUT);
  pinMode(DIOT,INPUT);
  din[0] = new DIN(DIOQ);
  din[1] = new DIN(DIOR);
  din[2] = new DIN(DIOS);
  din[3] = new DIN(DIOT);
  // Set outputs
  pinMode(DIOA,OUTPUT); digitalWrite(DIOA, LOW);
  pinMode(DIOB,OUTPUT); digitalWrite(DIOB, LOW);
  pinMode(DIOC,OUTPUT); digitalWrite(DIOC, LOW);
  pinMode(DIOD,OUTPUT); digitalWrite(DIOD, LOW);
  pinMode(SW1,OUTPUT); SW1OFF;
  pinMode(SW2,OUTPUT); SW2OFF;

  myfs.begin(0x1024 * 64);

  Restore();
  
  sb.begin();
  Serial.begin(0);
  //Serial1.begin(9600);
  cp.registerStream(&Serial);
  cp.registerStream(&sb);
  cp.DoNotprocessStream(&sb,true);
  cp.selectStream(&Serial);
  cp.registerCommands(&dbsList);
  cp.registerCommands(dbg.debugCommands());
  dbg.registerDebugFunction(Debug);

  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(100);
  // Add thread to the controller
  control.add(&SystemThread);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));

    // Setup TWI as slave to communicate with MIPS.
  Wire.begin(dcbs.TWIadd);              // join i2c bus
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  AD5592init(CS);
  state.update = true;
}

void loop() 
{
  static bool PGenable[2] = {false,false};

  cp.processStreams();
  cp.processCommands();
  control.run();
  
  processRamps(&dcbs.ramps);
  for(int i=0;i<2;i++)
  {
    if(pulseGenStart[i])
    {
      pulseGenStart[i] = false;
      pulseGenEnable(i);
    }
  }
  processDatagrams();
  // See if pulse generator enable status has changed from false to true
  for(int i = 0; i<2 ; i++)
  {
    if(!PGenable[i] && dcbs.PG[i].enable) pulseGenEnable(i);
    PGenable[i] = dcbs.PG[i].enable;
  }
}

// Host commands

void SaveSettings(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  dcbs.Signature = SIGNATURE;
  // Save all system data
  myfs.remove("config");
  systemData = myfs.open("config", FILE_WRITE);
  systemData.write(&dcbs, sizeof(DCBswitch));
  systemData.close();
  
  // Save calibration data
  myfs.remove("cal");
  systemData = myfs.open("cal", FILE_WRITE);
  for(int i=0;i<4;i++) systemData.write(&dcbs.DCBctrl[i], sizeof(DACchan));
  for(int i=0;i<4;i++) systemData.write(&dcbs.DCBmon[i], sizeof(ADCchan));
  systemData.close();
  cp.sendACK();
}

bool Restore(void)
{
  static DCBswitch dcb;

  dcb.Signature = 0;
  // Restore calibration data
  if(systemData = myfs.open("cal", FILE_READ))
  {
    for(int i=0;i<4;i++) systemData.read(&dcbs.DCBctrl[i], sizeof(DACchan));
    for(int i=0;i<4;i++) systemData.read(&dcbs.DCBmon[i], sizeof(ADCchan));
    systemData.close();
  }
  // Restore all system data
  systemData = myfs.open("config", FILE_READ);
  systemData.read(&dcb, sizeof(DCBswitch));
  systemData.close();
  if(dcb.Signature == SIGNATURE) 
  {
    dcbs = dcb;
    return true;
  }
  return false;
}

void RestoreSettings(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  if(Restore()) cp.sendACK(); 
  else
  {
    cp.sendNAK(ERR_EEPROMWRITE);
    return;
  }
}

void formatFlashDrive(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  myfs.quickFormat();
  cp.sendACK();
}

void setDCBvoltage(void)
{ 
  int   chan;
  float val;
  
  if(!cp.checkExpectedArgs(2)) return;
  if(!cp.getValue(&chan, 1, 4)) {cp.sendNAK(ERR_BADARG); return;}
  if(!cp.getValue(&val, -250, 250)) {cp.sendNAK(ERR_BADARG); return;}
  dcbs.DCBV[chan-1] = val;
  cp.sendACK();
}

void getDCBvoltage(void)
{
  int   chan;
  
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&chan, 1, 4)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK(false);
  cp.println(dcbs.DCBV[chan-1]);
}

void getDCBreadback(void)
{
  int   chan;
  
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&chan, 1, 4)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK(false);
  cp.println(DCBrb[chan-1]);  
}

void setDCBpwr(void)
{
  char *res;
  
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&res, "ON,OFF")) {cp.sendNAK(ERR_BADARG); return;}
  if(strcmp(res,"ON")==0) 
  {
    dcbs.pwrEnable = true;
  }
  else dcbs.pwrEnable = false;
  cp.sendACK();
}

void getDCBpwr(void)
{
  if(!cp.checkExpectedArgs(0)) return;
  cp.sendACK(false);
  if(dcbs.pwrEnable) cp.println("ON");
  else cp.println("OFF");
}

// Pulse generator and ramp generator helper functions
int setPVvalue(int *val1, int *val2, int ll, int ul, bool readCh = true)
{
  int ch=1,*val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(2)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(1)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  if(!cp.getValue(val, ll, ul)) {cp.sendNAK(ERR_BADARG); return -1;}
  cp.sendACK();
  return ch-1;
}

int setPVvalue(float *val1, float *val2, float ll, float ul, bool readCh = true)
{
  int   ch=1;
  float *val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(2)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(1)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  if(!cp.getValue(val, ll, ul)) {cp.sendNAK(ERR_BADARG); return -1;}
  cp.sendACK();
  return ch-1;
}

int setPVvalue(char *val1, char *val2, char *options, bool readCh = true)
{
  int ch=1;
  char *val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(2)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(1)) return -1;
  if(!cp.getValue(&val, (char *)options)) {cp.sendNAK(ERR_BADARG); return -1;}
  if(val[0] == '0') val[0] = 0;
  if(ch==1) *val1 = val[0];
  else *val2 = val[0];
  cp.sendACK();
  return ch-1;
}

int setPVvalue(bool *val1, bool *val2, bool readCh = true)
{
  int  ch=1;
  char *res;
  bool val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(2)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(1)) return -1;
  if(!cp.getValue(&res, (char *)"TRUE,FALSE")) {cp.sendNAK(ERR_BADARG); return -1;}
  if(strcmp(res,"TRUE")==0) val = true;
  else val = false;
  if(ch == 1) *val1 = val;
  else *val2 = val;
  cp.sendACK();
  return ch-1;
}

int setPVlevel(int *val1, int *val2, bool readCh = true)
{
  int  ch=1;
  char *res;
  int  val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(2)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(1)) return -1;
  if(!cp.getValue(&res, (char *)"CHANGE,RISING,FALLING")) {cp.sendNAK(ERR_BADARG); return -1;}
  if(strcmp(res,"CHANGE")==0) val = CHANGE;
  else if(strcmp(res,"RISING")==0) val = RISING;
  else val = FALLING;
  if(ch == 1) *val1 = val;
  else *val2 = val;
  cp.sendACK();
  return ch-1;
}


int getPVvalue(int val1, int val2, bool readCh = true)
{
  int ch=1,val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(1)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(0)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  cp.sendACK(false);
  cp.println(val);
  return ch-1;
}

int getPVvalue(float val1, float val2, bool readCh = true)
{
  int   ch=1;
  float val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(1)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(0)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  cp.sendACK(false);
  cp.println(val);
  return ch-1;
}

int getPVvalue(char val1, char val2, bool readCh = true)
{
  int  ch=1;
  char val;
  char res[2];

  if(readCh)
  {
    if(!cp.checkExpectedArgs(1)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(0)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  cp.sendACK(false);
  res[0] = val;
  res[1] = 0;
  if(val == 0) cp.println(0);
  else cp.println(res);
  return ch-1;
}

int getPVvalue(bool val1, bool val2, bool readCh = true)
{
  int   ch=1;
  bool  val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(1)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(0)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  cp.sendACK(false);
  if(val) cp.println("TRUE");
  else cp.println("FALSE");
  return ch-1;
}

int getPVlevel(int val1, int val2, bool readCh = true)
{
  int   ch=1;
  int  val;

  if(readCh)
  {
    if(!cp.checkExpectedArgs(1)) return -1;
    if(!cp.getValue(&ch, 1, 2)) {cp.sendNAK(ERR_BADARG); return -1;}
  }
  else if(!cp.checkExpectedArgs(0)) return -1;
  if(ch==1) val = val1;
  else val = val2;
  cp.sendACK(false);
  if(val == CHANGE) cp.println("CHANGE");
  else if(val == RISING) cp.println("RISING");
  else cp.println("FALLING");
  return ch-1;
}

// Pulse generator command processing functions
void setPGena(void)     {int ch = setPVvalue(&dcbs.PG[0].enable, &dcbs.PG[1].enable);}
void getPGena(void)     {getPVvalue(dcbs.PG[0].enable, dcbs.PG[1].enable);}
void setPGretrig(void)  {setPVvalue(&dcbs.PG[0].reTrig, &dcbs.PG[1].reTrig);}
void getPGretrig(void)  {getPVvalue(dcbs.PG[0].reTrig, dcbs.PG[1].reTrig);}
void setPGarmT(void)    {setPVvalue(&dcbs.PG[0].armTrig, &dcbs.PG[1].armTrig, (char *)"0,Q,R,S,T");}
void getPGarmT(void)    {getPVvalue(dcbs.PG[0].armTrig, dcbs.PG[1].armTrig);}
void setPGarmL(void)    {setPVlevel(&dcbs.PG[0].armLevel, &dcbs.PG[1].armLevel);}
void getPGarmL(void)    {getPVlevel(dcbs.PG[0].armLevel, dcbs.PG[1].armLevel);}
void setPGtrig(void)    {setPVvalue(&dcbs.PG[0].trig, &dcbs.PG[1].trig, (char *)"0,Q,R,S,T");}
void getPGtrig(void)    {getPVvalue(dcbs.PG[0].trig, dcbs.PG[1].trig);}
void setPGtrigL(void)   {setPVlevel(&dcbs.PG[0].trigLevel, &dcbs.PG[1].trigLevel);}
void getPGtrigL(void)   {getPVlevel(dcbs.PG[0].trigLevel, dcbs.PG[1].trigLevel);}
void setPGskip(void)    {setPVvalue(&dcbs.PG[0].skip, &dcbs.PG[1].skip, 0, 10000);}
void getPGskip(void)    {getPVvalue(dcbs.PG[0].skip, dcbs.PG[1].skip);}
void setPGdly(void)     {setPVvalue(&dcbs.PG[0].delay, &dcbs.PG[1].delay, 0, 10000000);}
void getPGdly(void)     {getPVvalue(dcbs.PG[0].delay, dcbs.PG[1].delay);}
void setPGwdth(void)    {setPVvalue(&dcbs.PG[0].width, &dcbs.PG[1].width, 0, 10000000);}
void getPGwdth(void)    {getPVvalue(dcbs.PG[0].width, dcbs.PG[1].width);}
void setPGtrgOut(void)  {setPVvalue(&dcbs.PG[0].trigOut, &dcbs.PG[1].trigOut, (char *)"0,A,B,C,D");}
void getPGtrgOut(void)  {getPVvalue(dcbs.PG[0].trigOut, dcbs.PG[1].trigOut);}
void setPGtrhFET(void)  {setPVvalue(&dcbs.PG[0].trigFET, &dcbs.PG[1].trigFET, (char *)"0,1,2");}
void getPGtrhFET(void)  {getPVvalue(dcbs.PG[0].trigFET, dcbs.PG[1].trigFET);}
void setPGoutCh(void)   {setPVvalue(&dcbs.PG[0].outputCh, &dcbs.PG[1].outputCh, 0, 4);}
void getPGoutCh(void)   {getPVvalue(dcbs.PG[0].outputCh, dcbs.PG[1].outputCh);}
void setPGvoltage(void) {setPVvalue(&dcbs.PG[0].pulseV, &dcbs.PG[1].pulseV, -250.0, 250.0);}
void getPGvoltage(void) {getPVvalue(dcbs.PG[0].pulseV, dcbs.PG[1].pulseV);}
void setPGnum(void)     {setPVvalue(&dcbs.PG[0].numPulse, &dcbs.PG[1].numPulse, 0, 1000000);}
void getPGnum(void)     {getPVvalue(dcbs.PG[0].numPulse, dcbs.PG[1].numPulse);}
void setPGperiod(void)  {setPVvalue(&dcbs.PG[0].period, &dcbs.PG[1].period, 0, 10000000);}
void getPGperiod(void)  {getPVvalue(dcbs.PG[0].period, dcbs.PG[1].period);}

void setDOUT(void)
{
  char *res;
  int  val;
  if(!cp.checkExpectedArgs(2)) return;
  if(!cp.getValue(&res, (char *)"A,B,C,D")) {cp.sendNAK(ERR_BADARG); return;}
  if(!cp.getValue(&val, 0,1)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK();
  digitalWrite(Douts[res[0] - 'A'],val);
}

void getDIN(void)
{
  int  i;
  char *res;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&res, (char *)"A,B,C,D,Q,R,S,T")) {cp.sendNAK(ERR_BADARG); return;}
  if(res[0] < 'Q') i = digitalRead(Douts[res[0] - 'A']);
  else i = digitalRead(Douts[res[0] - 'Q']);
  cp.sendACK(false);
  cp.println(i);
}

void setSW(void)
{
  int sw,i;
  if(!cp.checkExpectedArgs(2)) return;
  if(!cp.getValue(&sw, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  if(!cp.getValue(&i, 0,1)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK();
  if(i == 1) digitalWrite(FETs[sw-1],FETon[sw-1]);
  else digitalWrite(FETs[sw-1],FEToff[sw-1]);
}

void getSW(void)
{
  int sw,i;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&sw, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  i = digitalRead(FETs[sw-1]);
  cp.sendACK(false);
  if(i == FETon[sw-1]) cp.println(1);
  else cp.println(0);
}

void pulseSW(void)
{
  int sw,width;
  if(!cp.checkExpectedArgs(2)) return;
  if(!cp.getValue(&sw, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  if(!cp.getValue(&width, 1,10000)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK();
  if(digitalRead(FETs[sw-1]) == 1)
  {
    digitalWrite(FETs[sw-1],0);
    delayMicroseconds(width);
    digitalWrite(FETs[sw-1],1);
  }
  else
  {
    digitalWrite(FETs[sw-1],1);
    delayMicroseconds(width);
    digitalWrite(FETs[sw-1],0);    
  }
}

void setSwitchTrigger(void)
{
  int  sw;
  char *res;
  if(!cp.checkExpectedArgs(2)) return;
  if(!cp.getValue(&sw, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  if(!cp.getValue(&res, (char *)"0,Q,R,S,T")) {cp.sendNAK(ERR_BADARG); return;}
  sw--;
  cp.println(res);
  if(FETfollow[sw] != 0) din[FETfollow[sw] - 'Q']->detach(switchFollowISRs[sw]);
  if(res[0] != '0')
  {
    FETfollow[sw] = res[0];
    if(FETfollow[sw] != 0) din[FETfollow[sw] - 'Q']->attach(CHANGE,switchFollowISRs[sw]);
  }
  cp.sendACK();
}

void getSwitchTrigger(void)
{
  int  sw;
  char res[2] = {0};
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&sw, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  sw--;
  res[0] = FETfollow[sw];
  cp.sendACK(false);
  if(FETfollow[sw] == 0) cp.println(0);
  else cp.println(res);
}

// Calibrates a DC bias channel.
void calibrate(void)
{
  int ch,DAC0,DAC1;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&ch, 1,4)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK(false);
  cp.print("Calibrate DCbias channel: "); cp.println(ch);
  ch--;
  // Set voltage to 0 and read the actual voltage and the readback ADC counts
  AD5592writeDAC(CS, dcbs.DCBctrl[ch].Chan, DAC0 = Value2Counts(0,&dcbs.DCBctrl[ch]));
  float V0 = cp.userInputFloat("Enter measured voltage: ");
  int ADC0 = AD5592readADC(CS, dcbs.DCBmon[ch].Chan,10);
  // Set voltage to 125 and read the actual voltage and the readback ADC counts
  AD5592writeDAC(CS, dcbs.DCBctrl[ch].Chan, DAC1 = Value2Counts(125.0,&dcbs.DCBctrl[ch]));
  float V1 = cp.userInputFloat("Enter measured voltage: ");
  int ADC1 = AD5592readADC(CS, dcbs.DCBmon[ch].Chan,10);
  // Calculate the DAC calibration parameters
  // counts = value * m + b
  // DAC0 = V0 * m + b
  // DAC1 = V1 * m + b
  // DAC0 - DAC1 = (V0 - V1) * m
  // m = (DAC0 - DAC1)/(V0 - V1)
  // b = DAC0 - V0 * m
  dcbs.DCBctrl[ch].m = (float)(DAC0 - DAC1)/(V0 - V1);
  dcbs.DCBctrl[ch].b = (float)DAC0 - V0 * dcbs.DCBctrl[ch].m;
  // Report the DAC calibration parameters
  cp.println("DAC calibration parameters:");
  cp.print  ("  m = "); cp.println(dcbs.DCBctrl[ch].m);
  cp.print  ("  b = "); cp.println(dcbs.DCBctrl[ch].b);
  // Calculate the ADC calibration parameters
  dcbs.DCBmon[ch].m = (float)(ADC0 - ADC1)/(V0 - V1);
  dcbs.DCBmon[ch].b = (float)ADC0 - V0 * dcbs.DCBmon[ch].m;
  // Report the ADC calibration parameters
  cp.println("ADC calibration parameters:");
  cp.print  ("  m = "); cp.println(dcbs.DCBmon[ch].m);
  cp.print  ("  b = "); cp.println(dcbs.DCBmon[ch].b);  
  // Restore voltage to setpoint
  AD5592writeDAC(CS, dcbs.DCBctrl[ch].Chan, DAC1 = Value2Counts(dcbs.DCBV[ch],&dcbs.DCBctrl[ch]));
}

// Ramp host interface commands

void setRPramp(void)
{
  int ch;
  int args = cp.getNumArgs();
  if((args < 3) || (!cp.getValue(&ch, 1,2))) {cp.sendNAK(ERR_BADARG); return;}
  if((args-1)/2 > MAXRAMPPOINTS) {cp.sendNAK(ERR_BADARG); return;}
  dcbs.ramps.ramp[ch-1].numTP=0;
  for(int i=0;i<(args-1)/2;i++)
  {
    if(!cp.getValue(&dcbs.ramps.ramp[ch-1].tp[i], 0,10000)) {cp.sendNAK(ERR_BADARG); return;}
    if(!cp.getValue(&dcbs.ramps.ramp[ch-1].val[i], -250,250)) {cp.sendNAK(ERR_BADARG); return;}
    dcbs.ramps.ramp[ch-1].numTP++;
  }
  cp.sendACK();
}

void getRPramp(void)
{
  int ch;
  if(!cp.checkExpectedArgs(1)) return;
  if(!cp.getValue(&ch, 1,2)) {cp.sendNAK(ERR_BADARG); return;}
  cp.sendACK(false);
  for(int i=0;i<dcbs.ramps.ramp[ch-1].numTP;i++)
  {
    if(i>0) cp.print(",");
    cp.print(dcbs.ramps.ramp[ch-1].tp[i]);
    cp.print(",");
    cp.print(dcbs.ramps.ramp[ch-1].val[i]);
  }
  cp.println("");
}

void setRPtrig(void)    {setPVvalue(&dcbs.ramps.trig, NULL, (char *)"0,Q,R,S,T", false);}
void getRPtrig(void)    {getPVvalue(dcbs.ramps.trig, dcbs.ramps.trig, false);}
void setRPtrigL(void)   {setPVlevel(&dcbs.ramps.trigLevel, NULL, false);}
void getRPtrigL(void)   {getPVlevel(dcbs.ramps.trigLevel, dcbs.ramps.trigLevel, false);}
void setRPDCBchan(void) {setPVvalue(&dcbs.ramps.ramp[0].chan, &dcbs.ramps.ramp[1].chan, 1, 4);}
void getRPDCBchan(void) {getPVvalue(dcbs.ramps.ramp[0].chan, dcbs.ramps.ramp[1].chan);}
void setRPchENA(void)   {setPVvalue(&dcbs.ramps.ramp[0].enable, &dcbs.ramps.ramp[1].enable);}
void getRPchENA(void)   {getPVvalue(dcbs.ramps.ramp[0].enable, dcbs.ramps.ramp[1].enable);}
void setRPperiod(void)  {setPVvalue(&dcbs.ramps.period, &dcbs.ramps.period,0,10000,false);}
void getRPperiod(void)  {getPVvalue(dcbs.ramps.period, dcbs.ramps.period,false);}
void setRPcycl(void)    {setPVvalue(&dcbs.ramps.cycles, &dcbs.ramps.cycles,1,1000000,false);}
void getRPcycl(void)    {getPVvalue(dcbs.ramps.cycles, dcbs.ramps.cycles,false);}

int rampCycle;

void rampISR(void)
{
  rampCycle++;
  if(rampCycle >= dcbs.ramps.cycles)
  {
    // Finished ramp cycles
    dcbs.ramps.generating = false;
    rampTimer.end();
    return;
  }
  dcbs.ramps.generating = true;
  dcbs.ramps.stime = micros();
}

void rampTrigISR(void)
{
  // Start the ramp generation
  rampCycle = 0;
  dcbs.ramps.generating = true;
  dcbs.ramps.stime = micros();
  rampTimer.begin(rampISR,dcbs.ramps.period * 1000);
}

void setRPena(void)
{
  if(setPVvalue(&enableRG, &enableRG, false)==-1) return;
  if(enableRG)
  {
    // Here if enabled
    if(dcbs.ramps.trig == 0) rampTrigISR();
    else din[dcbs.ramps.trig - 'Q']->attach(dcbs.ramps.trigLevel, rampTrigISR);
  }
  else
  {
    // Here if disabled
    for(int i=0;i<4;i++) din[i]->detach(rampTrigISR);
    rampTimer.end();
    dcbs.ramps.generating = false;
  }
}
void getRPena(void)     {getPVvalue(enableRG, enableRG, false);}

void setTWIaddress(void)
{
  char *twi;

  if(!cp.checkExpectedArgs(1)) return;
  if(cp.getValue(&twi))
  {
    sscanf(twi,"%x",&dcbs.TWIadd);
    cp.sendACK();
    return;
  }
  cp.sendNAK();
}
void getTWIaddress(void) {if(!cp.checkExpectedArgs(0)) return; cp.sendACK(false); cp.println(dcbs.TWIadd,HEX);}
