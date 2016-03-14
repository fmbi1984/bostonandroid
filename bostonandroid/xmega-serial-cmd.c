/*
  xmega-serial-cmd.c

  This is a simple utility that allows testing a number of functions of the Xmega
  including the ADC, DAC, GPIOs, Clock selection and Sleep function

  To use you need an EVAL-USB board (or equiv) with PORTF configured for USART at 9600 baud

  Connect to PC with appropriate COM port at 9600baud, 8-N-1 no flow control

  You will get a commandline terminal to set and configure a number of functions

*/

#include <stdio.h>
#include <stddef.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>
#include <avr\sleep.h>
#include <avr\pgmspace.h>

#define VERSION "0.0.3"

uint8_t ReadCalibrationByte( uint8_t index );

void Config32MHzClock(void);
void Config32KHzClock(void);
void Config2MHzClock(void);
void UsartWriteChar(unsigned char data);
unsigned char UsartReadChar(void);
void UsartWriteString(char *string);
void UsartWriteLine(char *string);
void Error(char *string);
void PrintUsage();
int IsAlpha(char val);
void ParseCommand(char *string);
int GetParams(char *string, unsigned int *Params);
unsigned int StringToInt(char *string);
void IntToString(int value, char *string);
void DoBlink(unsigned int freq, unsigned int count);
void DoDAC(unsigned int ch, unsigned int value, unsigned int ref);
void DoADC(unsigned int pos, unsigned int neg, unsigned int sign, unsigned int ref);
void DoADCInternal(unsigned int pos, unsigned int sign, unsigned int ref);
void DoADCGain(unsigned int pos, unsigned int neg, unsigned int gain, unsigned int ref);
void DoADCCalibration(void);
void DoInput(char port, unsigned int pin);
void DoOutput(char port, unsigned int pin, unsigned int val);
void DoSleep(unsigned int mode);
void DoOscillator(unsigned int osc);
void DoUsartConfig(char port, unsigned int num, unsigned int bsel, unsigned int bscale);
void DoUsartTx(char port, unsigned int num, char *string);
void DoSpiConfig(char port, unsigned int div);
void DoSpiTx(char port, char *string);

int main(void)
{
  int data;
  int index=0;
  char buffer[100];
  Config32MHzClock();

  CLK.PSCTRL = 0x00; // no division on peripheral clock

  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout

  // configure PORTF, USARTF0 (PORTF:3=Tx, PORTF:2=Rx) as asynch serial port
  // This will connect to the USB-Serial chip on EVAL-USB boards
  // For other boards rewrite all occurences of USARTF0 below with USARTE0
  // then you can use PORTE:2,3 as asynch serial port (EVAL-01, EVAL-04 boards)
  PORTF.DIR |= (1<<3) | (1<<0); // set PORTF:3 transmit pin as output
  PORTF.OUT |= (1<<3);          // set PORTF:3 hi 
  USARTF0.BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
//  USARTF0.BAUDCTRLA = 103; // 19200b  (BSCALE=103,BSEL=0)
//  USARTF0.BAUDCTRLA = 34;  // 57600b  (BSCALE=34,BSEL=0)
//  USARTF0.BAUDCTRLA = 33; USARTF0.BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
//  USARTF0.BAUDCTRLA = 31; USARTF0.BAUDCTRLB = (-2<<4); // 230.4kb (BSCALE=31,BSEL=-2)
//  USARTF0.BAUDCTRLA = 27; USARTF0.BAUDCTRLB = (-3<<4); // 460.8kb (BSCALE=27,BSEL=-3)
//  USARTF0.BAUDCTRLA = 19; USARTF0.BAUDCTRLB = (-4<<4); // 921.6kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1; USARTF0.BAUDCTRLB = (1<<4); // 500kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1;   // 1Mb (BSCALE=1,BSEL=0)

  USARTF0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART


  UsartWriteString("\n\r\n\rXmega EVAL-USB Terminal\n\rv ");
  UsartWriteString(VERSION);
  UsartWriteString("\n\r");
  PrintUsage();
  UsartWriteString(": ");
  while(1)
  {
    data=UsartReadChar(); // read char
	// check for carriage return and try to match/execute command
	if((data == '\r')||(index==sizeof(buffer)))
	{
	    PORTF.OUT ^= (1<<0);      // switch LED
		buffer[index]=0;          // null terminate
		index=0;                  // reset buffer index
		UsartWriteString("\n\r"); // echo newline
		ParseCommand(buffer);     // attempt to parse command
    	  UsartWriteString(": ");
	}
	else if(data==8)              // backspace character
	{
	    if(index>0)
	        index--;                  // backup one character
		UsartWriteChar(data);
	}
	else
	{
	    buffer[index++]=data;
		UsartWriteChar(data);
	};

//	UsartWriteChar(data); // write char
//	_delay_ms(100);
//	PORTF.OUT ^= (1<<0); // toggle LED

  };
};

void PrintUsage()
{
  UsartWriteLine("==============================================================================");
  UsartWriteLine("h                          : Help");
  UsartWriteLine("b <freq> <num>             : Blink red user led using _delay_ms() function");
  UsartWriteLine("d <ch> <val> <ref>         : Output 12bit val on DAC channel");
  UsartWriteLine("                             use ref (0=1V,1=AVCC,2=AREFA,3=AREFB)");
  UsartWriteLine("a <pos> <neg> <sign> <ref> : Read ADC, use PORTA:<pos> - PORTA:<neg> ");
  UsartWriteLine("                             use ref (0=1V,1=VCC-0.6, 2=AREFA,3=AREFB)");
  UsartWriteLine("ai <ch> <sign> <ref>       : Read ADC, with ref, Internal channel ch");
  UsartWriteLine("                             (0=temp,1=bandgap,2=VCC scaled,3=DAC)");
  UsartWriteLine("ag <pos> <neg> <gain> <ref>: Read ADC, use PORTA:<pos> - PORTA:<neg>, with ref");
  UsartWriteLine("                             gain(0=1x,1=2x,2=4x,3=8x,4=16x,5=32x,6=64x)");
  UsartWriteLine("C                          : Calibrate ADC using prod signature row");
  UsartWriteLine("o<port> <pin> <val>        : Set output port(a,b,etc) pin=val (0/1)");
  UsartWriteLine("i<port> <pin>              : Read input port(a,b,etc) pin");
  UsartWriteLine("s <mode>                   : Sleep mode (0=idle,2=pwrdown,3=pwrsave,");
  UsartWriteLine("                             6=standby,7=extstandby)");
  UsartWriteLine("U<port> <num> <sel> <scale>: Config USART port(c,d,e,f), num(0,1)");
  UsartWriteLine("T<port> <num>              : Transmit \"01234567890\" on USART<port><num>");
  UsartWriteLine("                             BSEL=sel, BAUDSCALE=scale");
  UsartWriteLine("S<port> <div>              : Config SPI port(c,d,e) div clk(0=4,1=16,2=64,3=128)");
  UsartWriteLine("X<port>                    : Transmit \"0123456789\" on SPI<port>");
  UsartWriteLine("O <osc>                    : Select Oscillator osc (0=2MHz,1=32MHz,2=32KHz)");
};


void UsartWriteChar(unsigned char data)
{
    USARTF0.DATA = data; // transmit ascii 3 over and over
	if(!(USARTF0.STATUS&USART_DREIF_bm))
		while(!(USARTF0.STATUS & USART_TXCIF_bm)); // wait for TX complete
  	USARTF0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
};

unsigned char UsartReadChar(void)
{
	while(!(USARTF0.STATUS&USART_RXCIF_bm));  // wait for RX complete

  	return USARTF0.DATA;
};

// write out a simple '\0' terminated string
void UsartWriteString(char *string)
{

    while(*string != 0)
	  UsartWriteChar(*string++);
};

// write out a simple '\0' terminated string and print "\n\r" at end
void UsartWriteLine(char *string)
{
   UsartWriteString(string);
   UsartWriteString("\n\r");

};

void Error(char *string)
{
   UsartWriteString("Err: ");
   UsartWriteLine(string);
};

#define MAX_COMMAND_PARAMS 10
void ParseCommand(char *string)
{
  char Command;
  char Command2 = 0;
  unsigned int Params[MAX_COMMAND_PARAMS];
  unsigned int NumParams=0;

/*  UsartWriteString("Command Received: ");
  UsartWriteString(string);
  UsartWriteString("\n\r");
*/
  // assume commands are single character followed by numerical parameters sep by spaces
  // e.g. "s 1 5", "b 7", "b 100 120 001 212 123"
  Command = string[0];
  if(IsAlpha(string[1])) // multi-char command (e.g. pa, oa, ia, etc)
	  Command2 = string[1];

  if(Command != 0)
  {
    NumParams=GetParams(string,Params); // read any optional parameters after command

/*
    UsartWriteString("CommandID: ");
    UsartWriteChar(Command);
    if(Command2 != 0)
      UsartWriteChar(Command2);
    UsartWriteString(" #Params: ");
    UsartWriteChar(48+NumParams);
    UsartWriteString("\n\r");
*/
  }
  else
  {
  UsartWriteString("No Command\n\r");
  };

  switch(Command)
  {
   	case 'b': // blink(freq,num)
	  DoBlink(Params[0],Params[1]);
	  break;

    case 'd': // DAC set
	  if(NumParams==2)
	    DoDAC(Params[0],Params[1],1); // default use AVCC as reference
	  else if(NumParams==3)
	    DoDAC(Params[0],Params[1],Params[2]);
	  break;

    case 'i': // GPIO input
	  DoInput(Command2,Params[0]);
	  break;

    case 'o': // GPIO output
	  DoOutput(Command2,Params[0],Params[1]);
	  break;

    case 'a': // ADC do imediate conversion
	  if(Command2=='i')
	    DoADCInternal(Params[0],Params[1],Params[2]);
	  else if(Command2=='g')
	    DoADCGain(Params[0],Params[1],Params[2],Params[3]);
      else
		DoADC(Params[0],Params[1],Params[2],Params[3]);
	  break;

    case 'C': // calibrate ADC
	  DoADCCalibration();
	  break;

    case 'U': // configure USART
	  DoUsartConfig(Command2,Params[0],Params[1],Params[2]);
	  break;

    case 'T': // Usart TX test
	  DoUsartTx(Command2,Params[0],"0123456789");
	  break;

    case 'S': // Config SPI port
	  DoSpiConfig(Command2,Params[0]);
	  break;

    case 'X': // SPI TX test
	  DoSpiTx(Command2,"0123456789");
	  break;

    case 's': // Sleep CPU
	  DoSleep(Params[0]); // this will not return 
	  break;

    case 'O': // Set oscillator source
	  DoOscillator(Params[0]); // this will not return 
	  break;

    case 'h': // Usage
	  PrintUsage();
	  break;

  };	  


return;
};


void DoBlink(unsigned int freq, unsigned int count)
{
int i;
int period = 500/freq;

  PORTF.DIR |= (1<<0);

  for(i=0;i<count;i++)
  {
    PORTF.OUT |= (1<<0);
    _delay_ms(period);
    PORTF.OUT &= ~(1<<0);
    _delay_ms(period);
  };

};

void DoDAC(unsigned int ch, unsigned int value, unsigned int ref)
{
  if(ref<4)
    DACB.CTRLC = (ref << 3);           // select reference 
  else return; // early return for bad parameter

  DACB.CTRLB = DAC_CHSEL_DUAL_gc;      // select dual output DAC
  if(ch==0)
  {
	  DACB.CTRLA = (1<<2) | (1<<0);    // enable DACB, CH0
	  DACB.CH0DATA = value;
  }
  if(ch==1)
  {
      DACB.CTRLA |= (1<<3) | (1<<0);   // enable DACB, CH1
      DACB.CH1DATA = value;
  };

};


void DoADC(unsigned int pos, unsigned int neg, unsigned int sign, unsigned int ref)
{
  int result;
  static char buffer[10];

  if(ref>3)
  {
    Error("Ref can't be > 3");
	return; // early return for bad parameter
  };



  ADCA.CTRLA = 0x01; // enable ADC circuit
  ADCA.PRESCALER = ADC_PRESCALER_DIV128_gc;
  ADCA.CH0.MUXCTRL = (pos << ADC_CH_MUXPOS_gp) |  (neg << ADC_CH_MUXNEG_gp);

  if(sign==1) // implicit differential selection
  {
    if(neg > 3)
    {
      Error("ADC Diff requires neg pin 0-3");
      return;
    };

    ADCA.CTRLB = 0x10; // set conversion mode signed
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFF_gc;
  }
  else
  {
    ADCA.CTRLB = 0x00; // set conversion mode unsigned
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;

  };

  // clear any old interrupt flags
  ADCA.CH0.INTFLAGS |= 1;

  ADCA.CTRLA |= 0x4;   // start conversion on channel 0

  while(!ADCA.CH0.INTFLAGS);  // wait for conversion complete
  
  result = ADCA.CH0RES;

  UsartWriteString(" = ");
  IntToString(result,&buffer[0]);
  UsartWriteLine(buffer);

  return;
};


void DoADCInternal(unsigned int pos, unsigned int sign, unsigned int ref)
{
  int result;
  static char buffer[10];

  if(ref<4)
  {
    ADCA.REFCTRL = (ref << 4) | (1 << 2);  // select reference and enable bandgap
  }
  else 
  {
    Error("Ref can't be > 3");
	return; // early return for bad parameter
  };

  ADCA.CTRLA = 0x01; // enable ADC circuit
  ADCA.PRESCALER = ADC_PRESCALER_DIV128_gc;
  ADCA.CH0.MUXCTRL = (pos << ADC_CH_MUXPOS_gp);

  if(sign==1) // implicit differential selection
  {
    ADCA.CTRLB = 0x10; // set conversion mode signed
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;
  }
  else
  {
    ADCA.CTRLB = 0x00; // set conversion mode unsigned
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_INTERNAL_gc;

  };

  // clear any old interrupt flags
  ADCA.CH0.INTFLAGS |= 1;

  ADCA.CTRLA |= 0x4;   // start conversion on channel 0

  while(!ADCA.CH0.INTFLAGS);  // wait for conversion complete
  
  result = ADCA.CH0RES;

  UsartWriteString(" = ");
  IntToString(result,&buffer[0]);
  UsartWriteLine(buffer);

  return;
};

void DoADCGain(unsigned int pos, unsigned int neg, unsigned int gain, unsigned int ref)
{
  int result;
  static char buffer[10];

  if(ref<4)
  {
    ADCA.REFCTRL = (ref << 4) | (1 << 2);  // select reference and enable bandgap
  }
  else 
  {
    Error("Ref can't be > 3");
	return; // early return for bad parameter
  };

  if(neg < 4 || neg > 7)
  {
    Error("ADC w/gain requires neg pin 4-7");
    return;
  };

  ADCA.CTRLA = 0x01; // enable ADC circuit
  ADCA.PRESCALER = ADC_PRESCALER_DIV128_gc;
  ADCA.CH0.MUXCTRL = (pos << ADC_CH_MUXPOS_gp) |  (neg << ADC_CH_MUXNEG_gp);

  ADCA.CTRLB = 0x10;   // set conversion mode signed
  ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | (gain << ADC_CH_GAINFAC_gp); // set diff w/gain

  // clear any old interrupt flags
  ADCA.CH0.INTFLAGS |= 1;

  ADCA.CTRLA |= 0x4;   // start conversion on channel 0

  while(!ADCA.CH0.INTFLAGS);  // wait for conversion complete
  
  result = ADCA.CH0RES;

  UsartWriteString(" = ");
  IntToString(result,&buffer[0]);
  UsartWriteLine(buffer);

  return;
};

void DoADCCalibration()
{
  // read low ADCA calibration byte from NVM signature row into register
  ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );

  // read low ADCA calibration byte from NVM signature row into register
  ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );

  UsartWriteLine("ADC Calibrated");
};


void DoInput(char port, unsigned int pin)
{
  PORT_t *Port;

  switch(port)
  {
    case 'a':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTA);
	  break;
    case 'b':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTB);
	  break;
    case 'c':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTC);
	  break;
    case 'd':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTD);
	  break;
    case 'e':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTE);
	  break;
    case 'f':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTF);
	  if(pin>3) { Error("PORTF>3 blocked"); return; }; // don't allow user to screw up PORTF where serial port is connected
	  break;
    default:
	  Error("Illegal port");
	  return; // if no valid port, return
  };

  if(pin > 7) { Error("Illegal Pin > 7"); return; };

  Port->DIR &= ~(1<<pin);  // configure port as input
  if(Port->IN & (1<<pin))
    UsartWriteLine(" = 1 (hi)");
  else
    UsartWriteLine(" = 0 (lo)");

};

void DoOutput(char port, unsigned int pin, unsigned int val)
{
  PORT_t *Port;

  switch(port)
  {
    case 'a':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTA);
	  break;
    case 'b':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTB);
	  break;
    case 'c':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTC);
	  break;
    case 'd':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTD);
	  break;
    case 'e':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTE);
	  break;
    case 'f':
	  Port = (PORT_t*)_SFR_IO_ADDR(PORTF);
	  if(pin>3) { Error("PORTF>3 blocked"); return; }; // don't allow user to screw up PORTF where serial port is connected
	  break;
    default:
	  Error("Illegal port");
	  return; // if no valid port, return
  };

  if(pin > 7) { Error("Illegal Pin > 7"); return; };

  Port->DIR |= (1<<pin);  // configure port as output
  if(val==1)
	  Port->OUT |= (1<<pin);
  else if(val==0)
      Port->OUT &= ~(1<<pin);

};

void DoSleep(unsigned int mode)
{
  switch(mode)
  {
    case 0:
	  set_sleep_mode(SLEEP_MODE_IDLE);
	  break;
    case 2:
	  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	  break;
    case 3:
	  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	  break;
    case 6:
	  set_sleep_mode(SLEEP_MODE_STANDBY);
	  break;
    case 7:
	  set_sleep_mode(SLEEP_MODE_EXT_STANDBY);
	  break;
    default:
	  Error("Unused sleep mode");
	  return;
  };


  sleep_enable();
  sleep_cpu();

};

void DoOscillator(unsigned int osc)
{
  switch(osc)
  {
    case 0:
	  Config2MHzClock();
	  break;
    case 1:
	  Config32MHzClock();
	  break;
    case 2:
	  Config32KHzClock();
	  break;
    default:
	  Error("Unsupported Selectrion");
	  return;
  };
};

void DoUsartConfig(char port, unsigned int num, unsigned int bsel, unsigned int bscale)
{
  USART_t *Port=NULL;

  if(num==0)
  {
    switch(port)
    {
      case 'c':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTC0);
	    break;
      case 'd':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTD0);
	    break;
      case 'e':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTE0);
	    break;
      case 'f':
	    Error("PORTF blocked"); return; // don't allow user to screw up PORTF where serial port is connected
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };
  }
  else if(num==1)
  {
    switch(port)
    {
      case 'c':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTC1);
	    break;
      case 'd':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTD1);
	    break;
      case 'e':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTE1);
	    break;
      case 'f':
	    Error("PORTF blocked"); return; // don't allow user to screw up PORTF where serial port is connected
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };
  }
  else
  {
    Error("Illegal port num");
    return;
  };

  if(Port != NULL)
  {
    Port->BAUDCTRLA = 0xFF & bsel; // copy lower 8 of BSEL to BAUDCTRLA
    Port->BAUDCTRLB = ((bsel & 0xF00)>>8) | (bscale << 4); // copy upper BSEL and BSCALE to BAUDCTRLB
    Port->CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
  };

};


void DoUsartTx(char port, unsigned int num, char *string)
{
  USART_t *Port;

  if(num==0)
  {
    switch(port)
    {
      case 'c':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTC0);
		PORTC.DIR |= (1<<3);
	    break;
      case 'd':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTD0);
		PORTD.DIR |= (1<<3);
	    break;
      case 'e':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTE0);
		PORTE.DIR |= (1<<3);
	    break;
      case 'f':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTF0);
		PORTF.DIR |= (1<<3);
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };
  }
  else if(num==1)
  {
    switch(port)
    {
      case 'c':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTC1);
		PORTC.DIR |= (1<<7);
	    break;
      case 'd':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTD1);
		PORTD.DIR |= (1<<7);
	    break;
      case 'e':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTE1);
		PORTE.DIR |= (1<<7);
	    break;
      case 'f':
	    Port = (USART_t*)_SFR_IO_ADDR(USARTF1);
		PORTF.DIR |= (1<<7);
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };
  }
  else
  {
    Error("Illegal port num");
    return;
  };

    while(*string != 0)
	{
		Port->DATA = *string++;
        	if(!(Port->STATUS&USART_DREIF_bm))
		while(!(Port->STATUS & USART_TXCIF_bm)); // wait for TX complete
  		Port->STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
	};

};


void DoSpiConfig(char port, unsigned int div)
{
  SPI_t *Port;

    switch(port)
    {
      case 'c':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPIC);
		PORTC.DIR |= (1<<4) | (1<<5) | (1<<7);
	    break;
      case 'd':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPID);
		PORTD.DIR |= (1<<4) | (1<<5)| (1<<7);
	    break;
      case 'e':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPIE);
		PORTE.DIR |= (1<<4) | (1<<5)| (1<<7);
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };

	Port->CTRL = 0x50 | (div & 0x3);
};


void DoSpiTx(char port, char *string)
{
  SPI_t *Port;

    switch(port)
    {
      case 'c':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPIC);
	    break;
      case 'd':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPID);
	    break;
      case 'e':
	    Port = (SPI_t*)_SFR_IO_ADDR(SPIE);
	    break;
      default:
	    Error("Illegal port");
	    return; // if no valid port, return
    };

    while(*string != 0)
	{
		Port->DATA = *string++;
        	if(!(Port->STATUS&(1<<7)))
		while(!(Port->STATUS & (1<<7))); // wait for TX complete
  		Port->STATUS |= (1<<7);  // clear TX interrupt flag
	};
};


int GetParams(char *string, unsigned int *Params)
{
char buffer[10]; // max parameter length is 10 characters
int NumParams=0;
int index_in=0;
int index_buf=0;
int NumFound=0;

NumParams=0; // clear every time called

while(string[index_in] != 0)
{
	if((string[index_in] >= 48)&&(string[index_in] <= 57))
	{
	   buffer[index_buf++]=string[index_in++];
	   NumFound=1;
	}
    else if(NumFound && string[index_in] == 32)   // space
	{  
	   buffer[index_buf]=0; // terminate with 0
	   Params[NumParams]=StringToInt(buffer);
	   NumParams++; // increment num params parsed
       for(index_buf=0;index_buf<10;index_buf++) buffer[index_buf]=0;   // null buffer
	   index_buf=0; // reset buffer index to beginning
	   index_in++;
	}
	else // illegal character, ignore
	{
	   index_in++;
    };

};

// if bytes left, parse them as last parameter (non-space terminated)
if(index_buf > 0)
{
  	   Params[NumParams]=StringToInt(buffer);
	   NumParams++; // increment num params parsed
};


return NumParams;
};


int IsAlpha(char val)
{
  if(((val > 64)&&(val < 91)) || ((val > 60)&&(val < 123))) return 1;
  else return 0;
};


unsigned int StringToInt(char *string)
{
int index=0;
int value=0;

  // assume string is of the form "NNNNN", unsigned integer, no leading blank space
  while((string[index] >= 48)&&(string[index] <= 57))
  {
     value *= 10;
	 value += string[index]-48;
	 index++;
  };

  return value; // test code, just return 0 every time -RPM
};


void IntToString(int value, char *string)
{
int index=0;

if(value < 0)
{
  string[index++]='-';
  value = value * -1; // remove sign
};

if(value > 9999)
  string[index++]= (value / 10000) + 48;
else
  string[index++]= 48;
value %= 10000;

if(value > 999)
  string[index++]= (value / 1000) + 48;
else
  string[index++]= 48;
value %= 1000;

if(value > 99)
  string[index++]= (value / 100) + 48;
else
  string[index++]= 48;
value %= 100;

if(value > 9)
  string[index++]= (value / 10) + 48;
else
  string[index++]= 48;

value %= 10;

string[index++]=value + 48;

string[index++]=0; // null terminate

};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc; //select sysclock 32MHz osc
// update baud rate control to match new clk
  USARTF0.BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
};

void Config2MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC2MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC2MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = CLK_SCLKSEL_RC2M_gc; //select sysclock 32MHz osc
// update baud rate control to match new clk
    USARTF0.BAUDCTRLA = 12; // 9600b  (BSCALE=13,BSEL=0)
};

void Config32KHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32KEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32KRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = CLK_SCLKSEL_RC32K_gc; //select sysclock 32MHz osc
// serial port doesn't work at this clk speed so demo program will stop
};

uint8_t ReadCalibrationByte( uint8_t index ) 
{ 
uint8_t result; 

/* Load the NVM Command register to read the calibration row. */ 
NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 
result = pgm_read_byte(index); 

/* Clean up NVM Command register. */ 
NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

return( result ); 
} 
