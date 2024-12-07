#ifndef Hardware_h
#define Hardware_h
#include <Devices.h>

bool UpdateDACvalue(Devices dev, uint8_t add, DACchan *dchan, float *value, float *svalue, bool update);
bool UpdateADCvalue(Devices dev, uint8_t add, ADCchan *achan, float *value, float filter = 0.1);

#endif
