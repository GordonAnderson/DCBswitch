#include "DIN.h"
#include <algorithm>

void genericISR(int index)
{
  int pinState = digitalRead(DIN::ports[index]);
  for(int i=0;i<MaxDIcallbacks;i++) if(DIN::callbacks[index][i] != NULL)
  {
    if(DIN::modes[index][i] == CHANGE) DIN::callbacks[index][i]();
    else if(pinState == HIGH)
    {
      if(DIN::modes[index][i] == HIGH) DIN::callbacks[index][i]();
      else if(DIN::modes[index][i] == RISING) DIN::callbacks[index][i]();
    }
    else if(pinState == LOW)
    {
      if(DIN::modes[index][i] == LOW) DIN::callbacks[index][i]();
      else if(DIN::modes[index][i] == FALLING) DIN::callbacks[index][i]();
    }
  }
}

#if NumPorts >= 1
static void DI1_IRQ(void) { genericISR(0); }
#endif
#if NumPorts >= 2
static void DI2_IRQ(void) { genericISR(1); }
#endif
#if NumPorts >= 3
static void DI3_IRQ(void) { genericISR(2); }
#endif
#if NumPorts >= 4
static void DI4_IRQ(void) { genericISR(3); }
#endif
#if NumPorts >= 5
static void DI5_IRQ(void) { genericISR(4); }
#endif
#if NumPorts >= 6
static void DI6_IRQ(void) { genericISR(5); }
#endif
#if NumPorts >= 7
static void DI7_IRQ(void) { genericISR(6); }
#endif
#if NumPorts >= 8
static void DI8_IRQ(void) { genericISR(7); }
#endif
#if NumPorts == 1 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ}; 
#endif
#if NumPorts == 2 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ}; 
#endif
#if NumPorts == 3 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ}; 
#endif
#if NumPorts == 4 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ,DI4_IRQ}; 
#endif
#if NumPorts == 5 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ,DI4_IRQ,DI5_IRQ}; 
#endif
#if NumPorts == 6 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ,DI4_IRQ,DI5_IRQ,DI6_IRQ}; 
#endif
#if NumPorts == 7 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ,DI4_IRQ,DI5_IRQ,DI6_IRQ,DI7_IRQ}; 
#endif
#if NumPorts == 8 
void (*DIN::DI_ISRs[NumPorts])(void) = {DI1_IRQ,DI2_IRQ,DI3_IRQ,DI4_IRQ,DI5_IRQ,DI6_IRQ,DI7_IRQ,DI8_IRQ}; 
#endif
bool DIN::init = false;
int  DIN::ports[NumPorts];
int  DIN::modes[NumPorts][MaxDIcallbacks];
void (*DIN::callbacks[NumPorts][MaxDIcallbacks])(void);

DIN::DIN(int di_pin)
{
  if(!init)
  {
    init = true;
    for(int i=0;i<NumPorts;i++)
    {
      ports[i] = -1;
      for(int j=0;j<MaxDIcallbacks;j++)
      {
        callbacks[i][j] = NULL;
      }
    }
  }
  // If this port is defined then exit, nothing to do
  for(int i=0;i<NumPorts;i++) if(ports[i] == di_pin) return;
  // Find space for this port 
  for(int i=0;i<NumPorts;i++)
  {
    if(ports[i] == -1)
    {
      ports[i] = di = di_pin;
      pinMode(di,INPUT);
      attachInterrupt(di, DI_ISRs[i], CHANGE);
      return;
    }
  }
  // If here no space for this port!
  pinMode(di,INPUT);
  di = di_pin;
}

DIN::~DIN()
{
  // Find this port and clear all callbacks and detach interrupt
  for(int i=0;i<NumPorts;i++)
  {
    if(ports[i] == di)
    {
      detachInterrupt(di);
      ports[i] = -1;
      for(int j=0;j<MaxDIcallbacks;j++) callbacks[i][j] = NULL;
    }
  }
}

int DIN::state(void)
{
  return digitalRead(di);
}

bool DIN::attach(int mode,void (*isr)(void))
{
  if(isr == NULL) return false;
  if(isAttached(isr)) return true;
  for(int i=0;i<NumPorts;i++)
  {
    if(ports[i] == di)
    {
      for(int j=0;j<MaxDIcallbacks;j++)
      {
        if(callbacks[i][j] == NULL)
        {
          callbacks[i][j] = isr;
          modes[i][j] = mode;
          return true;
        }
      }
    }
  }
  return false;
}

bool DIN::isAttached(void (*isr)(void))
{
  if(isr == NULL) return false;
  for(int i=0;i<NumPorts;i++)
  {
    if(ports[i] == di)
    {
      for(int j=0;j<MaxDIcallbacks;j++)
      {
        if(callbacks[i][j] == isr) return true;
      }
    }
  }
  return false;  
}

bool DIN::detach(void (*isr)(void))
{
  if(isr == NULL) return false;
  for(int i=0;i<NumPorts;i++)
  {
    if(ports[i] == di)
    {
      for(int j=0;j<MaxDIcallbacks;j++)
      {
        if(callbacks[i][j] == isr) 
        {
          callbacks[i][j] = NULL;
          return true;
        }
      }
    }
  }
  return false;  
}
