#ifndef DIN_h
#define DIN_h
#include <Arduino.h>
#include <inttypes.h>

#define NumPorts        4
#define MaxDIcallbacks  10

class DIN
{
  public:
    static bool init;
    static void (*DI_ISRs[NumPorts])(void);
    static int  ports[NumPorts];
    static int  modes[NumPorts][MaxDIcallbacks];
    static void (*callbacks[NumPorts][MaxDIcallbacks])(void);
    
    DIN(int pi_pin);
    ~DIN(void);
    int  state(void);     
    bool attach(int mode,void (*isr)(void)); 
    bool isAttached(void (*isr)(void));
    bool detach(void (*isr)(void));    
  private:
    char   di;                                // Digital input pin number
    int    mode[MaxDIcallbacks];              // Mode used for this call back, Pos, Neg, or Change
    void   DI_IRQ(void);
    void   (*userISR[MaxDIcallbacks])(void);   // function called when a valid Interrupt happens
};

#endif
