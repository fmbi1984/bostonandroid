#include <stdio.h>
#include <avr\io.h>
#include <avr\interrupt.h>

void Config32MHzClock(void);

volatile unsigned int gCycleTime=0;

int main(void)
{
int temp;

  Config32MHzClock(); // configure sysclk=32MHz RC oscillator

  CLK.PSCTRL = 0x00;  // no division on peripheral clock

  // configure PORTC:0 as output of compare/match interrupt event
  PORTC.DIR=(1<<0);   // EVAL-USB

  // configure timercounter on PORTD to capture input freq on PORTD:0
  // 16bit counter counts from ~60Hz-1MHz square wave signals
  // for <60Hz increase clk divider in CTRLA
  // for >1MHz decrease clk divider in CTRLA
  TCD0.PER   = 0xFFFF;
  TCD0.CTRLA = TC_CLKSEL_DIV8_gc; // clk/8 , 32M/8=4MHz (.25us per tick)
  TCD0.CTRLB = TC0_CCAEN_bm; // enable capture compare A
                             // input capture value will be stored here
  TCD0.CTRLD = TC_EVACT_FRW_gc | TC_EVSEL_CH0_gc ; // freq capture event ch0
  // TCD0.CTRLD = TC_EVACT_PW_gc | TC_EVSEL_CH0_gc;   // pulsewidth capture event ch0

  EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN0_gc;     // ch0 event trigger on PORTD:0 pin change

  // also setup interrupt on CCA (capture event)
  TCD0.INTCTRLB = TC_CCAINTLVL_HI_gc;    // hi level interrupt on CCA

  PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts

  // read to clear interrupt
  temp = TCD0.CCA;

  // enable interupts
  sei();

  while(1);
};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};


ISR(TCD0_CCA_vect)
{
   PORTC.OUT ^= (1<<0);
   gCycleTime = TCD0.CCA; // read position
//   TCD0.CNT=0; // reset count
};


