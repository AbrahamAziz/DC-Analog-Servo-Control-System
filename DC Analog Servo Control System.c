#include <stdio.h>
#include "pic24_all.h"
#include <dataXfer.h>


#ifndef PWM_PERIOD
#define PWM_PERIOD 20000  // desired period, in us
#define MIN_PW 600
#define MAX_PW 2400
#endif

#ifndef _LPOSCEN
#error "This example only works with a device that has a secondary oscillator."
#endif
#ifndef _RTCSYNC
#error "This example only works with a device that has an RTCC module."
#endif

typedef union _unionRTCC {
  struct {  //four 16 bit registers
    uint8_t yr;
    uint8_t null;
    uint8_t date;
    uint8_t month;
    uint8_t hour;
    uint8_t wday;
    uint8_t sec;
    uint8_t min;
  } u8;
  uint16_t regs[4];
} unionRTCC;

unionRTCC u_RTCC;

uint8_t getBCDvalue(char *sz_1) {
  char sz_buff[8];
  uint16_t u16_bin;
  uint8_t  u8_bcd;
  outString(sz_1);
  inStringEcho(sz_buff,7);
  sscanf(sz_buff,"%d", (int *)&u16_bin);
  u8_bcd = u16_bin/10;   //most significant digit
  u8_bcd = u8_bcd << 4;
  u8_bcd = u8_bcd | (u16_bin%10);
  return(u8_bcd);
}

void getDateFromUser(void) {
  u_RTCC.u8.yr = getBCDvalue("Enter year (0-99): ");
  u_RTCC.u8.month = getBCDvalue("Enter month (1-12): ");
  u_RTCC.u8.date = getBCDvalue("Enter day of month (1-31): ");
  u_RTCC.u8.wday = getBCDvalue("Enter week day (0-6): ");
  u_RTCC.u8.hour = getBCDvalue("Enter hour (0-23): ");
  u_RTCC.u8.min = getBCDvalue("Enter min (0-59): ");
  u_RTCC.u8.sec = getBCDvalue("Enter sec(0-59): ");
}

//set date
void setRTCC(void) {
  uint8_t u8_i;
  __builtin_write_RTCWEN();   //enable write to RTCC, sets RTCWEN
  RCFGCALbits.RTCEN = 0;      //disable the RTCC
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) RTCVAL = u_RTCC.regs[u8_i];
  RCFGCALbits.RTCEN = 1;     //Enable the RTCC
  RCFGCALbits.RTCWREN = 0;   //can clear without unlock
}

void readRTCC(void) {
  uint8_t u8_i;
  RCFGCALbits.RTCPTR = 3;     //set pointer reg to start
  for (u8_i=0; u8_i<4; u8_i++) u_RTCC.regs[u8_i] = RTCVAL;
}

void printRTCC(void) {
  printf ("day(wday)/mon/yr: %2x(%2x)/%2x/%2x, %02x:%02x:%02x \n",
          (uint16_t) u_RTCC.u8.date,(uint16_t) u_RTCC.u8.wday, (uint16_t) u_RTCC.u8.month,
          (uint16_t) u_RTCC.u8.yr, (uint16_t) u_RTCC.u8.hour, (uint16_t) u_RTCC.u8.min, (uint16_t) u_RTCC.u8.sec);
}

uint16_t u16_minPWTicks, u16_maxPWTicks;

void  configTimer2(void) {
  T2CON = T2_OFF | T2_IDLE_CON | T2_GATE_OFF
          | T2_32BIT_MODE_OFF
          | T2_SOURCE_INT
          | T2_PS_1_64;
  PR2 = usToU16Ticks(PWM_PERIOD, getTimerPrescale(T2CONbits)) - 1;
  TMR2  = 0;       //clear timer2 value
  _T2IF = 0;
  _T2IP = 1;
  _T2IE = 1;    //enable the Timer2 interrupt
}

volatile int ccw;
volatile int cw;

void configOutputCompare2(void) {
    
    
  u16_minPWTicks = usToU16Ticks(ccw, getTimerPrescale(T2CONbits));
  u16_maxPWTicks = usToU16Ticks(cw, getTimerPrescale(T2CONbits));
  
  
  T2CONbits.TON = 0;          //disable Timer when configuring Output compare
  CONFIG_OC1_TO_RP(RB2_RP);   //map OC1 to RB4
  OC1RS = 0;  //clear both registers
  OC1R = 0;
#ifdef OC1CON1
//turn on the compare toggle mode using Timer2
  OC1CON1 = OC_TIMER2_SRC |     //Timer2 source
            OC_PWM_CENTER_ALIGN;  //PWM
  OC1CON2 = OC_SYNCSEL_TIMER2;   //synchronize to timer2
#else
//older families, this PWM mode is compatible with center-aligned, OC1R=0
//as writes to OC1RS sets the pulse widith.
  OC1CON = OC_TIMER2_SRC |     //Timer2 source
           OC_PWM_FAULT_PIN_DISABLE;  //PWM, no fault detection
#endif
}

void _ISR _T2Interrupt(void) {
  uint32_t u32_temp;
  _T2IF = 0;    //clear the timer interrupt bit
  //update the PWM duty cycle from the ADC value
  u32_temp = ADC1BUF0;  //use 32-bit value for range
  //compute new pulse width that is 0 to 99% of PR2
  // pulse width (PR2) * ADC/1024

  //u32_temp = (u32_temp * (PR2))>> 10 ;  // >>10 is same as divide/1024
  u32_temp = ((u32_temp * (u16_maxPWTicks - u16_minPWTicks)) >>10 ) + u16_minPWTicks;

  OC1RS = u32_temp;  //update pulse width value

  //SET_SAMP_BIT_ADC1();      //start sampling and conversion
  AD1CON1bits.SAMP = 1;

}

#if (HARDWARE_PLATFORM == EMBEDDED_C1)
# define RS_HIGH()        (_LATC2 = 1)
# define RS_LOW()         (_LATC2 = 0)
# define CONFIG_RS()      CONFIG_RC2_AS_DIG_OUTPUT()

# define RW_HIGH()        (_LATC1 = 1)
# define RW_LOW()         (_LATC1 = 0)
# define CONFIG_RW()      CONFIG_RC1_AS_DIG_OUTPUT()

# define E_HIGH()         (_LATC0 = 1)
# define E_LOW()          (_LATC0 = 0)
# define CONFIG_E()       CONFIG_RC0_AS_DIG_OUTPUT()

# define LCD4O          (_LATC4)
# define LCD5O          (_LATC5)
# define LCD6O          (_LATC6)
# define LCD7O          (_LATC7)
# define LCD7I          (_RC7)

# define CONFIG_LCD4_AS_INPUT() CONFIG_RC4_AS_DIG_INPUT()
# define CONFIG_LCD5_AS_INPUT() CONFIG_RC5_AS_DIG_INPUT()
# define CONFIG_LCD6_AS_INPUT() CONFIG_RC6_AS_DIG_INPUT()
# define CONFIG_LCD7_AS_INPUT() CONFIG_RC7_AS_DIG_INPUT()

# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RC4_AS_DIG_OUTPUT()
# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RC5_AS_DIG_OUTPUT()
# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RC6_AS_DIG_OUTPUT()
# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RC7_AS_DIG_OUTPUT()
#else
# define RS_HIGH()        (_LATB9 = 1)
# define RS_LOW()         (_LATB9 = 0)
# define CONFIG_RS()      CONFIG_RB9_AS_DIG_OUTPUT()

# define RW_HIGH()        (_LATB13 = 1)
# define RW_LOW()         (_LATB13 = 0)
# define CONFIG_RW()      CONFIG_RB13_AS_DIG_OUTPUT()

# define E_HIGH()         (_LATB14 = 1)
# define E_LOW()          (_LATB14 = 0)
# define CONFIG_E()       CONFIG_RB14_AS_DIG_OUTPUT()

# define LCD4O          (_LATB5)
# define LCD5O          (_LATB6)
# define LCD6O          (_LATB7)
# define LCD7O          (_LATB8)
# define LCD7I          (_RB8)

# define CONFIG_LCD4_AS_INPUT() CONFIG_RB5_AS_DIG_INPUT()
# define CONFIG_LCD5_AS_INPUT() CONFIG_RB6_AS_DIG_INPUT()
# define CONFIG_LCD6_AS_INPUT() CONFIG_RB7_AS_DIG_INPUT()
# define CONFIG_LCD7_AS_INPUT() CONFIG_RB8_AS_DIG_INPUT()

# define CONFIG_LCD4_AS_OUTPUT() CONFIG_RB5_AS_DIG_OUTPUT()
# define CONFIG_LCD5_AS_OUTPUT() CONFIG_RB6_AS_DIG_OUTPUT()
# define CONFIG_LCD6_AS_OUTPUT() CONFIG_RB7_AS_DIG_OUTPUT()
# define CONFIG_LCD7_AS_OUTPUT() CONFIG_RB8_AS_DIG_OUTPUT()
#endif

#define GET_BUSY_FLAG()  (LCD7I)





/**
 Functions above this line must be redefined for
 your particular PICmicro-to-LCD interface
*/

//Configure 4-bit data bus for output
void configBusAsOutLCD(void) {
  RW_LOW();                  //RW=0 to stop LCD from driving pins
  CONFIG_LCD4_AS_OUTPUT();   //D4
  CONFIG_LCD5_AS_OUTPUT();   //D5
  CONFIG_LCD6_AS_OUTPUT();   //D6
  CONFIG_LCD7_AS_OUTPUT();   //D7
}

//Configure 4-bit data bus for input
void configBusAsInLCD(void) {
  CONFIG_LCD4_AS_INPUT();   //D4
  CONFIG_LCD5_AS_INPUT();   //D5
  CONFIG_LCD6_AS_INPUT();   //D6
  CONFIG_LCD7_AS_INPUT();   //D7
  RW_HIGH();                   // R/W = 1, for read
}

//Output lower 4-bits of u8_c to LCD data lines
void outputToBusLCD(uint8_t u8_c) {
  LCD4O = u8_c & 0x01;          //D4
  LCD5O = (u8_c >> 1)& 0x01;    //D5
  LCD6O = (u8_c >> 2)& 0x01;    //D6
  LCD7O = (u8_c >> 3)& 0x01;    //D7
}

//Configure the control lines for the LCD
void configControlLCD(void) {
  CONFIG_RS();     //RS
  CONFIG_RW();     //RW
  CONFIG_E();      //E
  RW_LOW();
  E_LOW();
  RS_LOW();
}

//Pulse the E clock, 1 us delay around edges for
//setup/hold times
void pulseE(void) {
  DELAY_US(1);
  E_HIGH();
  DELAY_US(1);
  E_LOW();
  DELAY_US(1);
}

/* Write a byte (u8_Cmd) to the LCD.
u8_DataFlag is '1' if data byte, '0' if command byte
u8_CheckBusy is '1' if must poll busy bit before write, else simply delay before write
u8_Send8Bits is '1' if must send all 8 bits, else send only upper 4-bits
*/
void writeLCD(uint8_t u8_Cmd, uint8_t u8_DataFlag,
              uint8_t u8_CheckBusy, uint8_t u8_Send8Bits) {

  uint8_t u8_BusyFlag;
  uint8_t u8_wdtState;
  if (u8_CheckBusy) {
    RS_LOW();            //RS = 0 to check busy
    // check busy
    configBusAsInLCD();  //set data pins all inputs
    u8_wdtState = _SWDTEN;  //save WDT enable state
    CLRWDT();          //clear the WDT timer
    _SWDTEN = 1;            //enable WDT to escape infinite wait
    do {
      E_HIGH();
      DELAY_US(1);  // read upper 4 bits
      u8_BusyFlag = GET_BUSY_FLAG();
      E_LOW();
      DELAY_US(1);
      pulseE();              //pulse again for lower 4-bits
    } while (u8_BusyFlag);
    _SWDTEN = u8_wdtState;   //restore WDT enable state
  } else {
    DELAY_MS(10); // don't use busy, just delay
  }
  configBusAsOutLCD();
  if (u8_DataFlag) RS_HIGH();   // RS=1, data byte
  else    RS_LOW();             // RS=0, command byte
  outputToBusLCD(u8_Cmd >> 4);  // send upper 4 bits
  pulseE();
  if (u8_Send8Bits) {
    outputToBusLCD(u8_Cmd);     // send lower 4 bits
    pulseE();
  }
}

// Initialize the LCD, modify to suit your application and LCD
void initLCD() {
  DELAY_MS(50);          //wait for device to settle
  writeLCD(0x20,0,0,0); // 4 bit interface
  writeLCD(0x28,0,0,1); // 2 line display, 5x7 font
  writeLCD(0x28,0,0,1); // repeat
  writeLCD(0x06,0,0,1); // enable display
  writeLCD(0x0C,0,0,1); // turn display on; cursor, blink is off
  writeLCD(0x01,0,0,1); // clear display, move cursor to home
  DELAY_MS(3);
}

//Output a string to the LCD
void outStringLCD(char *psz_s) {
  while (*psz_s) {
    writeLCD(*psz_s, 1, 1,1);
    psz_s++;
  }
}
////////////////////////////////////////////////////////////////////////////////
#define C0 _RB15
#define C1 _RB12
#define C2 _RB11
#define C3 _RB10

static inline void CONFIG_COLUMN() {
  CONFIG_RB15_AS_DIG_INPUT();
  ENABLE_RB15_PULLUP();
  CONFIG_RB12_AS_DIG_INPUT();
  ENABLE_RB12_PULLUP();
  CONFIG_RB11_AS_DIG_INPUT();
  ENABLE_RB11_PULLUP();
  CONFIG_RB10_AS_DIG_INPUT();
  ENABLE_RB10_PULLUP();
}

#define R0 _LATA3
#define R1 _LATB3
#define R2 _LATA2

#define CONFIG_R0_DIG_OUTPUT() CONFIG_RA3_AS_DIG_OUTPUT()
#define CONFIG_R1_DIG_OUTPUT() CONFIG_RB3_AS_DIG_OUTPUT()
#define CONFIG_R2_DIG_OUTPUT() CONFIG_RA2_AS_DIG_OUTPUT()

void CONFIG_ROW() {
  CONFIG_R0_DIG_OUTPUT();
  CONFIG_R1_DIG_OUTPUT();
  CONFIG_R2_DIG_OUTPUT();
}

static inline void DRIVE_ROW_LOW() {
  R0 = 0;
  R1 = 0;
  R2 = 0;
}

static inline void DRIVE_ROW_HIGH() {
  R0 = 1;
  R1 = 1;
  R2 = 1;
}

void configKeypad(void) {
  CONFIG_ROW();
  DRIVE_ROW_LOW();
  CONFIG_COLUMN();
  DELAY_US(1);     //wait for pullups to stabilize inputs
}

//drive one row low
void setOneRowLow(uint8_t u8_x) {
  switch (u8_x) {
    case 0:
      R0 = 0;
      R1 = 1;
      R2 = 1;
      break;
    case 1:
      R0 = 1;
      R1 = 0;
      R2 = 1;
      break;
    default:
      R0 = 1;
      R1 = 1;
      R2 = 0;
      break;
  }
}
#define NUM_ROWS 3
#define NUM_COLS 4
const uint8_t au8_keyTable[NUM_ROWS][NUM_COLS] = { {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'}
};

#define KEY_PRESSED() (!C0 || !C1 || !C2 || !C3)   //any low
#define KEY_RELEASED() (C0 && C1 && C2 && C3)  //all high

uint8_t doKeyScan(void) {
  uint8_t u8_row, u8_col;
  //determine column
  if (!C0) u8_col = 0;
  else if (!C1) u8_col = 1;
  else if (!C2) u8_col = 2;
  else if (!C3) u8_col = 3;
  else return('E'); //error
  //determine row
  for (u8_row = 0; u8_row < NUM_ROWS; u8_row++) {
    setOneRowLow(u8_row); //enable one row low
    if (KEY_PRESSED()) {
      DRIVE_ROW_LOW(); //return rows to driving low
      return(au8_keyTable[u8_row][u8_col]);
    }
  }
  DRIVE_ROW_LOW(); //return rows to driving low
  return('E'); //error
}


typedef enum  {
  STATE_WAIT_FOR_PRESS = 0,
  STATE_WAIT_FOR_PRESS2,
  STATE_WAIT_FOR_RELEASE,
} ISRSTATE;

ISRSTATE e_isrState = STATE_WAIT_FOR_PRESS;
volatile uint8_t u8_newKey = 0;

//Interrupt Service Routine for Timer3
void _ISR _T3Interrupt (void) {
  switch (e_isrState) {
    case STATE_WAIT_FOR_PRESS:
      if (KEY_PRESSED() && (u8_newKey == 0)) {
        //ensure that key is sampled low for two consecutive interrupt periods
        e_isrState = STATE_WAIT_FOR_PRESS2;
      }
      break;
    case STATE_WAIT_FOR_PRESS2:
      if (KEY_PRESSED()) {
        // a key is ready
        u8_newKey = doKeyScan();
        e_isrState = STATE_WAIT_FOR_RELEASE;
      } else e_isrState = STATE_WAIT_FOR_PRESS;
      break;

    case STATE_WAIT_FOR_RELEASE:
      //keypad released
      if (KEY_RELEASED()) {
        e_isrState = STATE_WAIT_FOR_PRESS;
      }
      break;
    default:
      e_isrState = STATE_WAIT_FOR_PRESS;
      break;
  }
  _T3IF = 0;                 //clear the timer interrupt bit
}

#define ISR_PERIOD     15      // in ms

void  configTimer3(void) {
  //ensure that Timer2,3 configured as separate timers.
  T2CONbits.T32 = 0;     // 32-bit mode off
  //T3CON set like this for documentation purposes.
  //could be replaced by T3CON = 0x0020
  T3CON = T3_OFF | T3_IDLE_CON | T3_GATE_OFF
          | T3_SOURCE_INT
          | T3_PS_1_64 ;  //results in T3CON= 0x0020
  PR3 = msToU16Ticks (ISR_PERIOD, getTimerPrescale(T3CONbits)) - 1;
  TMR3  = 0;                       //clear timer3 value
  _T3IF = 0;                       //clear interrupt flag
  _T3IP = 1;                       //choose a priority
  _T3IE = 1;                       //enable the interrupt
  T3CONbits.TON = 1;               //turn on the timer
}
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_LED1() CONFIG_RA2_AS_DIG_OUTPUT()
#define LED1 (_LATA2)     //led1 state

int k = 0;
int i = 0;
int u = 0;
void self_check()
{
    for(k = 0; k < 2; k ++)
    {
        LED1 = 1;
        DELAY_MS(500);
        LED1 = !LED1;
    }
}

void LED_blink2()
{
    for(i = 0; i<2; i++)
    {
        LED1 = 1;
        DELAY_MS(500);
        LED1 = !LED1;
        DELAY_MS(500);
    }
}
void LED_blink3()
{
    for(u = 0; u<3; u++)
    {
        LED1 = 1;
        DELAY_MS(500);
        LED1 = !LED1;
        DELAY_MS(500);
    }
}

void options()
{
    writeLCD(0x80,0,0,1);
    outStringLCD("1.Set up 2.Run");
    writeLCD(0xC0,0,0,1);
    outStringLCD("3.Download");

}


void options2()
{
    writeLCD(0x01,0,0,1);
    writeLCD(0x80,0,0,1);
    outStringLCD("1.CCW 2.CW");
}

void LCD_build2(unsigned char location, unsigned char *ptr)
{
    unsigned char i;
    if(location < 8)
    {
        writeLCD(0x40+location*8,0,0,1);
        for (i = 0; i < 8; i++)
        {
            writeLCD(ptr[i],1,1,1);
        }
    }
}


int main (void) { 
  uint32_t u32_pw;
  char words[20];
  uint32_t percent1;
  uint32_t percent2;
  uint32_t barpercent1;
  uint32_t barpercent2;
  int tt;
  int tenten;
  int ten;
  int t;
  int h;
  int tt2;
  int tenten2;
  int ten2;
  int t2;
  
  CONFIG_LED1();
  configBasic(HELLO_MSG);      // Set up heartbeat, UART, print hello message and diags
  configKeypad(); //keypad
  configTimer3(); //keypad
  configControlLCD();      //configure the LCD control lines
  initLCD();               //initialize the LCD

  outStringLCD("Self check");
  writeLCD(0xC0,0,0,1);
  outStringLCD("Blink once");

  self_check();
  DELAY_MS(500);
  writeLCD(0x01,0,0,1); //CLEAR




  char pattern1[8] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
  char pattern2[8] = {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18};
  char pattern3[8] = {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C};
  char pattern4[8] = {0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E};
  char pattern5[8] = {0xF,0xF,0xF,0xF,0xF,0xF,0xF,0xF};
  char pattern6[8] = {0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7};
  char pattern7[8] = {0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3};
  char pattern8[8] = {0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1};

  LCD_build2(0,pattern1);
  LCD_build2(1,pattern2);
  LCD_build2(2,pattern3);
  LCD_build2(3,pattern4);
  LCD_build2(4,pattern5);
  LCD_build2(5,pattern6);
  LCD_build2(6,pattern7);
  LCD_build2(7,pattern8);


  __builtin_write_OSCCONL(OSCCON | 0x02);    //  OSCCON.SOSCEN=1;
  getDateFromUser();    //get initial date/time
  setRTCC();            //set the date/time

  
  while (1) {
      
      options();
      int a = doKeyScan();

      
      
       switch(a)
       {
               case '1':
               {
                   
                   while(1)
                   {                       
                       options2();
                       DELAY_MS(300);
                       //configOutputCompare2();
                       int h = doKeyScan();
                       
                       if(h == '1')
                       {
                            while(h =='1')
                            {
                                writeLCD(0x01,0,0,1);
                                writeLCD(0x80,0,0,1);
                                outStringLCD("Type Value in Ms");
                                writeLCD(0xC0,0,0,1);
                                outStringLCD("600-1499");
                                u8_newKey = 0;
                                DELAY_MS(1000);
                                    while(1)
                                    {

                                        int p = doKeyScan();         
                                        
                                            while (p == '0' || p == '1' || p == '2'|| p == '3'|| p == '4'|| p == '5'|| p == '6'|| p == '7'|| p == '8'
                                                    || p == '9')
                                            {                          
                                                writeLCD(0x01,0,0,1);
                                                writeLCD(0x80,0,0,1);
                                                outStringLCD("Type Value in Ms");
                                                writeLCD(0xC0,0,0,1);
                                                tt = p-48;
                                                sprintf(words,"%d",tt);
                                                outStringLCD(words);
                                                p = 0;
                                                DELAY_MS(1000);
                                                    while(1)
                                                    {
                                                        int e = doKeyScan();
                                                        
                                                        while (e == '0' || e == '1' || e == '2'|| e == '3'|| e == '4'|| e == '5'|| e == '6'|| e == '7'|| e == '8'
                                                            || e == '9')
                                                        {
                                                            writeLCD(0x01,0,0,1);
                                                            writeLCD(0x80,0,0,1);
                                                            outStringLCD("Type Value in Ms");
                                                            writeLCD(0xC0,0,0,1);                                    
                                                            tenten = e-48;
                                                            sprintf(words,"%d",tt*10+tenten*1);
                                                            outStringLCD(words);
                                                            e = 0;
                                                            DELAY_MS(1000);
                                                            while(1)
                                                            {
                                                                int r = doKeyScan();
                                                                while (r == '0' || r == '1' || r == '2'|| r == '3'|| r == '4'|| r == '5'|| r == '6'|| r == '7'|| r == '8'
                                                                        || r == '9')
                                                                        {
                                                                            writeLCD(0x01,0,0,1);
                                                                            writeLCD(0x80,0,0,1);
                                                                            outStringLCD("Type Value in Ms");
                                                                             writeLCD(0xC0,0,0,1);
                                                                             ten = r-48;
                                                                             ccw = tt*100+tenten*10+ten;
                                                                            
                                                                             sprintf(words,"%d",ccw);
                                                                             outStringLCD(words);
                                                                             r = 0;
                                                                             DELAY_MS(1000);
                                                                        while(1)
                                                                        {
                                                                            ccw = tt*100+tenten*10+ten;
                                                                            int y = doKeyScan();
                                                                            if(y == '*')
                                                                                        {
                                                                                            goto c;
                                                                                        }
                                                                                while (y == '0' || y == '1' || y == '2'|| y == '3'|| y == '4'|| y == '5'|| y == '6'|| y == '7'|| y == '8'
                                                                                         || y == '9')
                                                                                {
                                                                                    writeLCD(0x01,0,0,1);
                                                                                    writeLCD(0x80,0,0,1);
                                                                                    outStringLCD("Type Value in Ms");
                                                                                    writeLCD(0xC0,0,0,1);
                                                                                    t = y-48;
                                                                                    ccw = tt*1000+tenten*100+ten*10+t;
                                                                                    
                                                                                    sprintf(words,"%d",ccw);
                                                                                    outStringLCD(words);
                                                                                
                                  
                                                                                    y = 0;
                                                                                        while(1)
                                                                                        {
                                                                                            int y = doKeyScan();   
                                                                                            if(y == '*')
                                                                                            {
                                                                                                goto c;
                                                                                            }
                                                                                        }
                                                                                        
                                                                                }
                                                                        }
                                                                        }
                                                            }
                                                        }
                                                    }                    
                                            }
                                    }
                            }
                       }
                       if(h == '2')
                       {
                            while(h =='2')
                            {
                                
                                writeLCD(0x01,0,0,1);
                                writeLCD(0x80,0,0,1);
                                outStringLCD("Type Value in Ms");
                                writeLCD(0xC0,0,0,1);
                                outStringLCD("1501-2400");
                                u8_newKey = 0;
                                DELAY_MS(1000);
                                    while(1)
                                    {

                                        int p = doKeyScan();         
                                        
                                            while (p == '0' || p == '1' || p == '2'|| p == '3'|| p == '4'|| p == '5'|| p == '6'|| p == '7'|| p == '8'
                                                    || p == '9')
                                            {                          
                                                writeLCD(0x01,0,0,1);
                                                writeLCD(0x80,0,0,1);
                                                outStringLCD("Type Value in Ms");
                                                writeLCD(0xC0,0,0,1);
                                                tt2 = p-48;
                                                sprintf(words,"%d",tt2);
                                                outStringLCD(words);
                                                p = 0;
                                                DELAY_MS(1000);
                                                    while(1)
                                                    {
                                                        int e = doKeyScan();
                                                        
                                                        while (e == '0' || e == '1' || e == '2'|| e == '3'|| e == '4'|| e == '5'|| e == '6'|| e == '7'|| e == '8'
                                                            || e == '9')
                                                        {
                                                            writeLCD(0x01,0,0,1);
                                                            writeLCD(0x80,0,0,1);
                                                            outStringLCD("Type Value in Ms");
                                                            writeLCD(0xC0,0,0,1);                                    
                                                            tenten2 = e-48;
                                                            sprintf(words,"%d",tt2*10+tenten2*1);
                                                            outStringLCD(words);
                                                            e = 0;
                                                            DELAY_MS(1000);
                                                            while(1)
                                                            {
                                                                int r = doKeyScan();
                                                                while (r == '0' || r == '1' || r == '2'|| r == '3'|| r == '4'|| r == '5'|| r == '6'|| r == '7'|| r == '8'
                                                                        || r == '9')
                                                                        {
                                                                            writeLCD(0x01,0,0,1);
                                                                            writeLCD(0x80,0,0,1);
                                                                            outStringLCD("Type Value in Ms");
                                                                             writeLCD(0xC0,0,0,1);
                                                                             ten2 = r-48;
                                                                             cw = tt2*100+tenten2*10+ten2;
                                                                             sprintf(words,"%d",cw);
                                                                             outStringLCD(words);
                                                                             r = 0;
                                                                             DELAY_MS(1000);
                                                                        while(1)
                                                                        {
                                
                                                                            int y = doKeyScan();
                                                                            if(y == '*')
                                                                                        {
                                                                                            goto c;
                                                                                        }
                                                                                while (y == '0' || y == '1' || y == '2'|| y == '3'|| y == '4'|| y == '5'|| y == '6'|| y == '7'|| y == '8'
                                                                                         || y == '9')
                                                                                {
                                                                                    writeLCD(0x01,0,0,1);
                                                                                    writeLCD(0x80,0,0,1);
                                                                                    outStringLCD("Type Value in Ms");
                                                                                    writeLCD(0xC0,0,0,1);
                                                                                    t2 = y-48;
                                                                                    cw = tt2*1000+tenten2*100+ten2*10+t2;
                                                                            
                                                                                    sprintf(words,"%d",cw);
                                                                                    outStringLCD(words);
                                                                               
                                          
                                                                                    
                                                                                    
                                                                                    configTimer2();
                                                                                    configOutputCompare2();
                                                                                    CONFIG_RA1_AS_ANALOG();
                                                                                    configADC1_ManualCH0(RA1_AN, 31, 0);
                                                                                    SET_SAMP_BIT_ADC1();      //start sampling and conversion
                                                                                    T2CONbits.TON = 1;       //turn on Timer2 to start PWM
                                                                                    
                                                                                    LED_blink2();
                                                                                    
                                                                                    goto c;
                                                                                    y = 0;
                                                                                        while(1)
                                                                                        {
                                                                                            int y = doKeyScan();   
                                                                                            if(y == '*')
                                                                                            {
                                                                                                goto c;
                                                                                            }
                                                                                        }
                                                                                        
                                                                                }
                                                                        }
                                                                        }
                                                            }
                                                        }
                                                    }                    
                                            }
                                    }
                            }
                       }
                   }                 
                }
                 
               
                   
                break;
                
                case '2':
                {

                    writeLCD(0x01,0,0,1);
               
                        char x;
                        
                    while(1)
                    {
 
                        int o = doKeyScan();
                        u32_pw = ticksToUs(OC1RS, getTimerPrescale(T2CONbits));
                        printf("PWM PW (us): %ld \n", u32_pw);
                        percent1 = (1500-u32_pw)/8.9;
                        percent2 = (u32_pw-1500)/8.9;
                        barpercent1 = 0.8*percent1;
                        barpercent2 = 0.8*percent2;
                         
                      
                        writeLCD(0x80,0,0,1);

                        outStringLCD(words);
                        
                        if (0 <= barpercent1 && barpercent1 < 10)
                        {
                            x = 0;
                        }
                        if (10 <= barpercent1 && barpercent1 < 20)
                        {
                            x = 1;
                        }
                        if (20 < barpercent1 && barpercent1 < 30)
                        {
                            x = 2;
                        }
                        if (30 <= barpercent1 && barpercent1 < 40)
                        {
                            x = 3;
                        }
                        if (40 <= barpercent1 && barpercent1 < 50)
                        {
                            x = 4;
                        }
                        if (50 <= barpercent1 && barpercent1 < 60)
                        {
                            x = 5;
                        }
                        if (60 <= barpercent1 && barpercent1 < 70)
                        {
                            x = 6;
                        }
                        if (70 <= barpercent1 && barpercent1 < 80)
                        {
                            x = 7;
                        }
                        if (80 <= barpercent1 && barpercent1 < 90)
                        {
                            x = 8;
                        }
                        
                        if (0 <= barpercent2 && barpercent2 < 10)
                        {
                            x = 0;
                        }
                        if (10 <= barpercent2 && barpercent2 < 20)
                        {
                            x = 1;
                        }
                        if (20 < barpercent2 && barpercent2 < 30)
                        {
                            x = 2;
                        }
                        if (30 <= barpercent2 && barpercent2 < 40)
                        {
                            x = 3;
                        }
                        if (40 <= barpercent2 && barpercent2 < 50)
                        {
                            x = 4;
                        }
                        if (50 <= barpercent2 && barpercent2 < 60)
                        {
                            x = 5;
                        }
                        if (60 <= barpercent2 && barpercent2 < 70)
                        {
                            x = 6;
                        }
                        if (70 <= barpercent2 && barpercent2 < 80)
                        {
                            x = 7;
                        }
                        if (80 <= barpercent2 && barpercent2 < 90)
                        {
                            x = 8;
                        }
                        if(u32_pw >=600 && u32_pw <= 1499)
                        {
                            if (barpercent1 > 0+10*x && barpercent1 < 2+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(7,1,1,1);
                            }
                            if(barpercent1 >= 2+10*x && barpercent1 < 4+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(6,1,1,1);
                            }
                            if(barpercent1 >= 4+10*x && barpercent1 < 6+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(5,1,1,1);
                            }
                            if(barpercent1 >= 6+10*x && barpercent1 < 8+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(4,1,1,1);
                            }
                            if(barpercent1 >= 8+10*x && barpercent1 < 10+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(0xFF,1,1,1);
                            }
                            if(barpercent1 == 10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"-%ld(%%)           ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC7-x,0,0,1);
                                writeLCD(0x20,1,1,1);
                            }

                            if(o == '6')
                                {
                                    goto b;
                                }
                        }
                        if(u32_pw == 1500)
                        {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"0(%%)         ",percent1/2);
                                outStringLCD(words);
                                writeLCD(0xC8,0,0,1);
                                writeLCD(0xEF,1,1,1);
                        }
                        if(u32_pw >=1501 && u32_pw <= 2399)
                        {
                            if (barpercent2 > 0+10*x && barpercent2 < 2+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(0,1,1,1);
                            }
                            if(barpercent2 >= 2+10*x && barpercent2 < 4+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(1,1,1,1);
                            }
                            if(barpercent2 >= 4+10*x && barpercent2 < 6+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(2,1,1,1);
                            }
                            if(barpercent2 >= 6+10*x && barpercent2 < 8+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(3,1,1,1);
                            }
                            if(barpercent2 >= 8+10*x && barpercent2 < 10+10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(0xFF,1,1,1);
                            }
                            if(barpercent2 == 10*x)
                            {
                                writeLCD(0x80,0,0,1);                      
                                sprintf(words,"%ld(%%)           ",percent2/2);
                                outStringLCD(words);
                                writeLCD(0xC8+x,0,0,1);
                                writeLCD(0x20,1,1,1);
                            }

                            if(o == '6')
                                {
                                    goto b;
                                }
                        }
                         
                    }
                    }
    
                break;
                
                case '3':
                {
          
                    while(1)
                    {
                        writeLCD(0x01,0,0,1);
                        writeLCD(0x80,0,0,1);
                        //outStringLCD("");
                        //writeLCD(0xC0,0,0,1);
                        sprintf(words,"PW (us): %ld",u32_pw);
                        outStringLCD(words);
                        
                        int k = doKeyScan();
                     
                        while (!RCFGCALbits.RTCSYNC) doHeartbeat();
                        readRTCC();
                        printRTCC();
                        //u32_pw = ticksToUs(OC1RS, getTimerPrescale(T2CONbits));
                        printf("PWM PW (us): %ld \n", u32_pw);
                        DELAY_MS(30);
                        LED_blink3();
                         goto b;

              
                            if(k == '6')
                            {
                                goto b;
                            }
                    }
                   
                }
                break;
                case '4':
                {
                   
                        while(1)
                        {
                            int k = doKeyScan();
                            if(k == '6')
                            {
                                goto b;
                            }
                        }
                }
                break;
                case '5':
                {
                   
                        while(1)
                        {
                            int k = doKeyScan();
                            if(k == '6')
                            {
                                goto b;
                            }
                        }
                }
                
                case '6':
                {
                    b:

                    writeLCD(0x01,0,0,1);
                }
                break;
                default:
                    c:
                    writeLCD(0x01,0,0,1);
                    u8_newKey = 0;
       }
    //writeLCD(0xC0,0,0,1);  // cursor to 2nd line                
    //writeLCD(0x18,0,1,1);  // shift left
    //DELAY_MS(200);
    doHeartbeat();
  }
}
