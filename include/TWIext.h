#ifndef TWIext_h
#define TWIext_h

#include <Arduino.h>
#include <Wire.h>
#include <SerialBuffer.h>

extern SerialBuffer sb;

// Helpers for reading/writing multi-byte values on the TWI (I2C) slave interface.
// Read* functions pull from the Wire receive buffer and return false/-1 if the
// data is incomplete. Send* functions queue bytes in 'sb' for the next master read.
int ReadUnsignedWord(void);
bool ReadInt(int *i);
int ReadUnsignedByte(void);
bool ReadByte(int8_t *b);
bool Read16bitInt(int16_t *shortint);
bool ReadFloat(float *fval);
void SendByte(byte bval);
void SendInt24(int ival);
void SendFloat(float fval);
void SendBuffer(uint8_t *buf, int size);

#endif
