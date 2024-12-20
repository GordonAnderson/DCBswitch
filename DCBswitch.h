#ifndef DCBSWITCH_H_
#define DCBSWITCH_H_

// TWI commands and constants
// DCBswitch TWI commands
// Set parameter command
#define TWI_DCBS_SERIAL     0x27        // This command enables the TWI port to process serial commands
#define TWI_DCBS_EEPROM     0x01        // This command enables the EEPROM interface, true or false
#define SET_DCBS_SDOUT      0x02        // Set digitial output,A|B|C|D,0|1
#define SET_DCBS_PWR        0x03        // Set the DCB power supply on|off, true or false
#define SET_DCBS_FETSW      0x04        // Set FET switch state, on|off, true or false
#define SET_DCBS_FETPULSE   0x05        // FET switch output, switch number, width in uS
#define SET_DCBS_SWFOL      0x06        // Set FET switch to follow input, switch number, 0|Q|R|S|T
#define SET_DCBS_RBTST      0x07        // Set voltage readback testing, true or false
// Return status commands
#define GET_DCBS_AVALIBLE   0x82        // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define GET_DCBS_SDIN       0x41        // Returns digitial input,A|B|C|D|Q|R|S|T
#define GET_DCBS_VRB        0x42        // Returns the DCBS channels voltage readbacks 
#define GET_DCBS_PWR        0x43        // Returns the DCB power supply on|off, true or false
#define GET_DCBS_FETSW      0x44        // Returns FET switch state, on|off, true or false
#define GET_DCBS_SWFOL      0x45        // Returns FET switch follow input state, switch number
#define GET_DCBS_RBTST      0x46        // Returns voltage readback testing, true or false

#define ESC   27
#define ENQ   5

#define FILTER        0.1

// Readback voltage test trip point
#define TRIPVOLTAGE   5
#define HOLDOFF       50

#define SIGNATURE     0xAA55A5A5

// Tennys 4.0 Pin assinments
// Digital outputs
#define   DIOA    23
#define   DIOB    22
#define   DIOC    21
#define   DIOD    20
// Digotal inputs  
#define   DIOQ    17
#define   DIOR    16
#define   DIOS    15
#define   DIOT    14

#define   CS      10
#define   SW1     9
#define   SW2     8
#define   PWROFF  5

// AD5592 channel assignment
#define   CH1_CTRL  0
#define   CH2_CTRL  1
#define   CH3_CTRL  2
#define   CH4_CTRL  3
#define   CH1_MON   4
#define   CH2_MON   5
#define   CH3_MON   6
#define   CH4_MON   7

#define   SW1ON     digitalWrite(SW1,LOW)
#define   SW1OFF    digitalWrite(SW1,HIGH)
#define   SW2ON     digitalWrite(SW2,HIGH)
#define   SW2OFF    digitalWrite(SW2,LOW)

#define MAXRAMPPOINTS 10

typedef struct
{
  bool  enable;                 // Channel enable
  int   chan;                   // DC bias channel 1 thru 4
  int   numTP;                  // Number of time points
  float tp[MAXRAMPPOINTS];      // Ramp time point, in mS
  float val[MAXRAMPPOINTS];     // Ramp time point voltage in volts
} Ramp;

typedef struct
{
  char      trig;                   // Ramp generator trigger, Q,R,S,T,or 0 to disable
  int       trigLevel;              // Ramp generator trigger level
  bool      generating;             // True when a cycle is in process
  float     ctime;                  // Cycle time in mS
  uint32_t  stime;                  // Start time in uS
  int       cycles;                 // Number of cycles
  float     period;                 // Period in mS
  Ramp      ramp[2];                // Channel 1 and 2 ramps
} Ramps;

typedef struct
{
  bool          update;
  bool          pwrEnable;
  float         DCBV[4];
} State;

typedef struct
{
  bool          enable;
  bool          reTrig;
  char          armTrig;                // Arm the pulse generator, Q,R,S,T,or 0 to disable
  int           armLevel;               // Arm trigger level
  char          trig;                   // Pulse generator trigger, Q,R,S,T,or 0 to disable
  int           trigLevel;              // Pulse generator trigger level
  int           skip;                   // Number of trigger to skip
  int           delay;                  // Delay in uS from trigger to pulse
  int           width;                  // Pulse width in uS
  char          trigOut;                // Output signal for pulse, A,B,C,D,or 0 to disable
  char          trigFET;                // FET switch to toggle, 1,2,or 0 to disable
  int           outputCh;               // Output voltage channel, 1,2,3,4, or 0 to disable
  float         pulseV;                 // Pulse active voltage
  int           numPulse;               // -1 = forever
  int           period;                 // Pulse period in uS
} PulseGen;

typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the module name
  int8_t        Rev;                    // Holds the board revision number
  int           TWIadd;                 // TWI address
  // DCBswitch general parameters
  bool          pwrEnable;              // If true turns on power supply
  bool          rbTest;                 // If true enable readback testing
  float         DCBV[4];                // DC bias channel setpoints
  PulseGen      PG[2];                  // Pulse generator data structures
  Ramps         ramps;                  // Ramp generators
  // Calibration parameters
  DACchan       DCBctrl[4];             // DAC control for DC bias channels
  ADCchan       DCBmon[4];              // ADC monitor channels for readbacks
  // Data structure signature
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} DCBswitch;

void SaveSettings(void);
void RestoreSettings(void);
void formatFlashDrive(void);

void setDCBvoltage(void);
void getDCBvoltage(void);
void getDCBreadback(void);
void setDCBpwr(void);
void getDCBpwr(void);

void setPGena(void);
void getPGena(void);
void setPGretrig(void);
void getPGretrig(void);
void setPGarmT(void);
void getPGarmT(void);
void setPGarmL(void);
void getPGarmL(void);
void setPGtrig(void);
void getPGtrig(void);
void setPGtrigL(void);
void getPGtrigL(void);
void setPGskip(void);
void getPGskip(void);
void setPGdly(void);
void getPGdly(void);
void setPGwdth(void);
void getPGwdth(void);
void setPGtrgOut(void);
void getPGtrgOut(void);
void setPGtrhFET(void);
void getPGtrhFET(void);
void setPGoutCh(void);
void getPGoutCh(void);
void setPGvoltage(void);
void getPGvoltage(void);
void setPGnum(void);
void getPGnum(void);
void setPGperiod(void);
void getPGperiod(void);

void setDOUT(void);
void getDIN(void);

void setSW(void);
void getSW(void);
void pulseSW(void);

void setSwitchTrigger(void);
void getSwitchTrigger(void);

void calibrate(void);

#endif
