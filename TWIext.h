#ifndef TWIext_h
#define TWIext_h

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
