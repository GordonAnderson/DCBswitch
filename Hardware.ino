#include <Arduino.h>
#include <AtomicBlock.h>
#include <SPI.h>
#include "Hardware.h"

bool UpdateADCvalue(Devices dev, uint8_t add, ADCchan *achan, float *value, float filter)
{
  int   val=0;
  float fval;

  if(dev == AD5592) if((val = AD5592readADC(add, achan->Chan,10)) == -1) return false;
  fval = Counts2Value(val,achan);
  if(*value == -1) *value = fval;
  else *value = filter * fval + (1 - filter) * *value;
  return true;
}

bool UpdateDACvalue(Devices dev, uint8_t add, DACchan *dchan, float *value, float *svalue, bool update)
{
  if((update) || (*value != *svalue))
  {
    if(dev == AD5592) AD5592writeDAC(add, dchan->Chan, Value2Counts(*value,dchan));
    *svalue = *value;
    return true;
  }
  return false;
}
